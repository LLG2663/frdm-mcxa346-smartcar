/*
 * SPDX-License-Identifier: MIT
 *
 * bt_car.c —— 无线串口遥控：命令分派、上锁状态机、失联保护。
 *
 * 命令表、为什么上电默认上锁、为什么解锁是单字节、状态锁存与失联保护的
 * 取舍（默认关）—— 都写在 bt_car.h 顶部。本文件里与源工程的对应关系：
 *
 *   源工程 main.c 的 BT_ReadByte()  → [PORT] rx_indicate + 信号量 + rt_device_read()
 *   源工程 main.c 的 BT_Apply()     → 这里逐行照搬（含注释与计数规则）
 *   源工程 main.c 的 while(1) 节拍  → [PORT] 独立线程
 *   源工程 main.c 的 Car_IsIdle()   → 没有移植，理由见 bt_car.h 末尾
 *   失联保护                        → [新增] 源工程只在注释里声称有，代码里没有
 */
#include <rtdevice.h>

#include "bt_car.h"
#include "car.h"
#include "motor.h"
#include "follow.h"         /* 遥控字节要能打断航点跟随，失联保护也要把它算作"在动" */
#include "speed_report.h"   /* 同一链路上的速度上报要在这里认领字节，见下面读取循环 */

#define DBG_TAG    "bt"
#define DBG_LVL    DBG_INFO
#include <rtdbg.h>

/* 低于 jy901s(22)：遥控命令要让 IMU 解析先走（IMU 丢帧只影响显示，
 * 遥控晚一拍就是车晚一拍）。高于显示线程(26)和 led(25)，且在 tshell(20) 之下。 */
#define BT_THREAD_PRIO      23
#define BT_THREAD_STACK     1024
#define BT_RX_CHUNK         32
#define BT_RX_BUFSZ         64      /* 环形缓冲：命令都是单字节，64 足够 */

static const char *const bt_state_name_tab[] = {
    "DISARM", "BRAKE", "FORWARD", "BACKWARD", "SPIN-L", "SPIN-R"
};

static rt_device_t          bt_dev = RT_NULL;
static struct rt_semaphore  bt_rx_sem;

/* ----------------------------------------------------------------------------
 *  状态。显示层和 msh 会读，遥控线程会写，所以都要 volatile。
 *  都是 32 位以内、地址对齐的单次读写，不用额外加临界区
 *  （这个理由和 jy901s.c 里的 filtered_yaw 一样）。
 * -------------------------------------------------------------------------- */
static volatile bt_state_t   g_state      = BT_ST_DISARM;
static volatile rt_uint8_t   g_armed      = 0;      /* 0 = 上锁，运动命令只计数不执行 */
static volatile rt_uint32_t  g_cmd_cnt    = 0;      /* 已识别的命令字节数（自检用） */
static volatile rt_int16_t   g_last_rx    = -1;     /* 最后收到的原始字节，-1 = 还没收到 */
static volatile rt_uint32_t  g_last_rx_tick = 0;    /* 最后一次收到字节的内核 tick */
static volatile rt_uint32_t  g_link_lost  = 0;      /* 失联保护触发次数 */
static volatile rt_uint32_t  g_link_to_ms = BT_LINK_TIMEOUT_MS;
static volatile rt_bool_t    g_echo       = RT_FALSE;

/* ============================================================================
 *  对外查询
 * ========================================================================== */
const char *bt_car_state_name(void)
{
    return bt_state_name_tab[g_state];
}

rt_bool_t bt_car_is_armed(void)
{
    return g_armed ? RT_TRUE : RT_FALSE;
}

rt_uint32_t bt_car_cmd_count(void)
{
    return g_cmd_cnt;
}

rt_int16_t bt_car_last_byte(void)
{
    return g_last_rx;
}

rt_uint8_t bt_car_link_lost(void)
{
    return (rt_uint8_t)g_link_lost;
}

void bt_car_set_link_timeout_ms(rt_uint32_t ms)
{
    g_link_to_ms = ms;
    /* 改完就重新起算，免得刚关掉又立刻按旧的到期时间炸一次 */
    g_last_rx_tick = rt_tick_get();
}

/* ============================================================================
 *  命令分派 —— 源工程 main.c 的 BT_Apply() 逐行照搬
 * ============================================================================
 *  上锁时只计数、不动电机 —— 但【仍然计数】（计数解锁后才显示在屏幕上），
 *  这样解锁后发一个 'W' 看到 "Ok" +1，就能确认字节不但在波特率上对，
 *  而且被正确识别成了命令。
 *
 *  【与源工程的唯一区别】Car_* 的参数从「占空比 BT_DUTY」换成
 *  「目标转速 Car_GetRunSpeed()」——闭环遥控的决定见 car.h。
 *
 *  【返回值】认出这个字节就是 RT_TRUE，调用方据此回一行「实际 + 预期」速度
 *  （speed_report_reply）。没认出来的返回 RT_FALSE —— 串口助手/上位机自动补
 *  的 '\r' '\n' 不该也触发一次回包，否则按住按键时会白刷一堆行。
 * ========================================================================== */
static rt_bool_t bt_apply(rt_uint8_t c)
{
    switch (c) {
    case 'W': case 'w':
        g_cmd_cnt++;
        if (g_armed) { Car_Forward(Car_GetRunSpeed());   g_state = BT_ST_FORWARD; }
        break;
    case 'S': case 's':
        g_cmd_cnt++;
        if (g_armed) { Car_Backward(Car_GetRunSpeed());  g_state = BT_ST_BACKWARD; }
        break;
    case 'A': case 'a':
        g_cmd_cnt++;
        if (g_armed) { Car_SpinLeft(Car_GetRunSpeed());  g_state = BT_ST_SPIN_LEFT; }
        break;
    case 'D': case 'd':
        g_cmd_cnt++;
        if (g_armed) { Car_SpinRight(Car_GetRunSpeed()); g_state = BT_ST_SPIN_RIGHT; }
        break;
    case ' ': case 'X': case 'x':
        g_cmd_cnt++;
        if (g_armed) { Car_Brake();                      g_state = BT_ST_BRAKE; }
        break;
    case 'O': case 'o':
        /* 解锁：只打开闸门，【绝不动车】。解锁后保持刹车，等第一个运动命令。
         * 不计数 —— 它和 H 一样是状态开关，不是运动命令。 */
        g_armed = 1;
        Car_Brake();
        g_state = BT_ST_BRAKE;
        break;
    case 'H': case 'h':
        /* 上锁：立刻刹车。不需要先解锁 —— 失败方向必须是安全的 */
        g_armed = 0;
        Car_Brake();
        g_state = BT_ST_DISARM;
        break;
    default:
        /* '\r' '\n' 是串口助手自动补的行尾，静默忽略。
         * 其余不认识的字节也不报错 —— 原始值照常显示在 Rx 行上，那就是自检信息。
         * 【也不回速度】：只有真被执行的遥控字节才值得占一次链路。 */
        return RT_FALSE;
    }

    return RT_TRUE;
}

/* ============================================================================
 *  失联保护 —— [新增] 源工程只在注释里声称有这个功能，且【默认是关的】
 * ============================================================================
 *  默认关的原因见 bt_car.h：串口助手按一下只发一个字节，开着就会「转一下就停」。
 *  打开用 msh 的 `bt link <ms>`（0 或 off 关）。打开后触发条件三件同时成立才刹车：
 *    1. 保护是开着的（超时 != 0）
 *    2. 已解锁（上锁时车本来就不动，没有可刹的东西）
 *    3. 车正在动，且距最后一次收到字节已超过超时
 *
 *  刹完保持解锁、状态回到 BRAKE：上位机重新连上直接按 W 就能继续，
 *  不需要再发一次 4F。这是刻意的——失联保护要防的是「车还在跑」，
 *  不是「把遥控器锁死」。
 *
 *  用无符号减法比较 tick，回绕（约 49.7 天）也是对的。
 * ========================================================================== */
static rt_bool_t bt_is_moving(bt_state_t st)
{
    return (st == BT_ST_FORWARD || st == BT_ST_BACKWARD ||
            st == BT_ST_SPIN_LEFT || st == BT_ST_SPIN_RIGHT) ? RT_TRUE : RT_FALSE;
}

static void bt_tick(void)
{
    rt_uint32_t timeout_ms = g_link_to_ms;

    /* 【航点跟随也算"在动"】跟随不走这个遥控状态机（它直接走 Car_DriveHeading），
     * g_state 停在 BRAKE，所以只看 bt_is_moving(g_state) 的话，车自己沿着轨迹
     * 跑着、无线串口断了也判不出"失联"来 —— 那正是最需要急停的时候。
     * 下面刹车之后还要 Follow_Abort()：只刹车不够，状态机下一拍会再下令走。 */
    rt_bool_t moving = (bt_is_moving(g_state) || Follow_IsRunning()) ? RT_TRUE : RT_FALSE;

    if (timeout_ms == 0 || !g_armed || !moving) {
        return;
    }

    if ((rt_tick_get() - g_last_rx_tick) <= rt_tick_from_millisecond(timeout_ms)) {
        return;
    }

    Car_Brake();
    g_state = BT_ST_BRAKE;
    g_link_lost++;
    Follow_Abort(FOLLOW_AB_LINK);
    LOG_W("link lost (%u ms without a byte), braked; stays armed", timeout_ms);
}

/* [PORT] 串口收到数据的回调。中断上下文里只能做"放行信号量"这种事，
 * 真正的读取和分派留给下面的线程（和 jy901s.c 同一个套路）。 */
static rt_err_t bt_rx_indicate(rt_device_t dev, rt_size_t size)
{
    RT_UNUSED(dev);
    RT_UNUSED(size);

    rt_sem_release(&bt_rx_sem);
    return RT_EOK;
}

/* ============================================================================
 *  遥控线程 —— 源工程 main() 的 while(1) 主体
 * ============================================================================
 *  节拍：有字节时由 rx_indicate 立刻唤醒（比源工程快，源工程最快也要等到
 *  下一次 10ms 轮询）；没字节时靠信号量超时每 BT_LOOP_MS 醒一次跑 bt_tick()，
 *  节拍与源工程一致。
 * ========================================================================== */
static void bt_car_thread(void *parameter)
{
    rt_uint8_t buf[BT_RX_CHUNK];
    rt_size_t  n, i;

    RT_UNUSED(parameter);

    while (1)
    {
        rt_sem_take(&bt_rx_sem, rt_tick_from_millisecond(BT_LOOP_MS));

        /* 一趟把环形缓冲里积压的字节全部取完：上位机可能一次发 "W\r\n" 三个字节 */
        while ((n = rt_device_read(bt_dev, -1, buf, sizeof(buf))) > 0)
        {
            for (i = 0; i < n; i++)
            {
                g_last_rx      = (rt_int16_t)buf[i];
                g_last_rx_tick = rt_tick_get();

                /* 【顺序要紧】先让 speed_report 认领 '#' 开头的速度上报命令，
                 * 没被认领的才当遥控命令执行。反过来的话 "#rate 200" 里的
                 * 'a'（原地左转）和空格（刹车）会当场把车开走 —— 那几个字符
                 * 在 bt_apply() 里都是有含义的。
                 * speed_report 没起来时（设备没找到）它一律返回 RT_FALSE，
                 * 遥控照旧，所以这里不依赖两者的初始化顺序。 */
                if (!speed_report_rx_byte(buf[i]))
                {
                    /* 解锁/上锁也在里面，无残留状态。
                     * 【认出来的字节才回一行速度】见 speed_report_reply()：
                     * 它内部有 50ms 最小间隔，按住按键连发也不会把 115200 灌满；
                     * '\r' '\n' 这种没认出来的（bt_apply 返回 RT_FALSE）不回。 */
                    if (bt_apply(buf[i]))
                    {
                        /* 【遥控随时能接管】只要有一个遥控字节被认出来
                         * （W/S/A/D/空格/X/O/H 全算），航点跟随就交还控制权。
                         * 认出来 = 人就在遥控器边上，这一刻他要的是"听我的"。
                         *
                         * 放在 bt_apply() 【之后】：那一条命令已经生效了，
                         * Follow_Abort() 只置一个标志，下一拍才由 Follow_Tick()
                         * 收手，而且那种情况下【不刹车】—— 见它的注释。 */
                        Follow_Abort(FOLLOW_AB_REMOTE);
                        speed_report_reply();
                    }
                }

                if (g_echo) {
                    rt_kprintf("<%02X %c>\n", buf[i],
                               (buf[i] >= 0x20 && buf[i] <= 0x7E) ? buf[i] : '.');
                }
            }
        }

        bt_tick();
    }
}

rt_err_t bt_car_init(void)
{
    struct serial_configure cfg = RT_SERIAL_CONFIG_DEFAULT;
    rt_thread_t tid;

    if (bt_dev != RT_NULL)
    {
        return RT_EOK;      /* 重复调用直接返回，别把线程建两遍 */
    }

    bt_dev = rt_device_find(BT_DEV_NAME);
    if (bt_dev == RT_NULL)
    {
        /* 多半是 .config 里没开 BSP_USING_UART5。只报错返回：
         * OLED、IMU、电机和 msh 照样能用，不影响其余功能的验证。 */
        LOG_E("serial device \"%s\" not found; check BSP_USING_UART5", BT_DEV_NAME);
        return -RT_ERROR;
    }

    cfg.baud_rate = BT_BAUD_RATE;
    cfg.data_bits = DATA_BITS_8;
    cfg.stop_bits = STOP_BITS_1;
    cfg.parity    = PARITY_NONE;
    cfg.bufsz     = BT_RX_BUFSZ;

    /* 【顺序】先 CONFIG 再 open，理由和 jy901s.c 那边一样（那边踩过）：
     * 设备 open 之后 ref_count 不为 0，只要 bufsz 和当前值不同，
     * RT_DEVICE_CTRL_CONFIG 会在调用 ops->configure() 之前直接返回 -RT_EBUSY，
     * 波特率就静默没落下去。这里 BT_RX_BUFSZ 恰好等于 RT_SERIAL_RB_BUFSZ（64），
     * 所以反过来也碰巧能用 —— 但那是巧合，改一下 BT_RX_BUFSZ 就会坏掉。
     * 放在 open 之前，缓冲大小和波特率由 rt_serial_open() 一次落好，没有这个依赖。 */
    if (rt_device_control(bt_dev, RT_DEVICE_CTRL_CONFIG, &cfg) != RT_EOK)
    {
        LOG_E("configure \"%s\" failed (baud %d, bufsz %u)",
              BT_DEV_NAME, BT_BAUD_RATE, (unsigned)cfg.bufsz);
        bt_dev = RT_NULL;
        return -RT_ERROR;
    }

    if (rt_device_open(bt_dev, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX) != RT_EOK)
    {
        LOG_E("open \"%s\" failed", BT_DEV_NAME);
        bt_dev = RT_NULL;
        return -RT_ERROR;
    }

    rt_device_set_rx_indicate(bt_dev, bt_rx_indicate);

    rt_sem_init(&bt_rx_sem, "bt", 0, RT_IPC_FLAG_FIFO);

    g_last_rx_tick = rt_tick_get();

    tid = rt_thread_create("bt", bt_car_thread, RT_NULL,
                           BT_THREAD_STACK, BT_THREAD_PRIO, 10);
    if (tid == RT_NULL)
    {
        LOG_E("failed to create the remote control thread");
        rt_device_close(bt_dev);
        bt_dev = RT_NULL;
        return -RT_ERROR;
    }
    rt_thread_startup(tid);

    LOG_I("%s open, %d 8N1, DISARMED at boot (send 4F to arm)", BT_DEV_NAME, BT_BAUD_RATE);
    return RT_EOK;
}

/* 上电自动起来，并保证默认是上锁的。 */
static int bt_car_auto_init(void)
{
    /* 先把终态钉死：即使 bt_car_init() 因为没开 UART5 而失败，
     * 也不可能是「解锁」状态。 */
    g_armed = 0;
    g_state = BT_ST_DISARM;
    Car_Brake();

    return (int)bt_car_init();
}
INIT_APP_EXPORT(bt_car_auto_init);

/* ============================================================================
 *  msh 命令
 * ============================================================================
 *  rt_kprintf 用的是 tiny vsnprintf，不支持 %f（会把 %f 原样打出来），
 *  LFLAGS 也没有 -lm。转速刻度上一个计数就是 24，小数位没有意义，
 *  所以这里直接四舍五入成整数打（motor.c / jy901s.c 里那两个定点打印
 *  是给角度用的，那边需要小数）。
 * ========================================================================== */
static void bt_print_int(float v)
{
    float mag = (v < 0.0f) ? -v : v;            /* 不用 fabsf，省掉 libm 依赖 */
    rt_int32_t total = (rt_int32_t)(mag + 0.5f);

    if (v < 0.0f && total != 0) {
        rt_kprintf("-");
    }
    rt_kprintf("%d", (int)total);
}

static void bt_print_status(void)
{
    rt_int16_t b = g_last_rx;

    rt_kprintf("bt: %s %d 8N1  armed=%s state=%s link=",
               BT_DEV_NAME, BT_BAUD_RATE, g_armed ? "yes" : "no",
               bt_car_state_name());

    if (g_link_to_ms == 0) {
        rt_kprintf("off");
    } else {
        rt_kprintf("%ums", g_link_to_ms);
    }
    rt_kprintf(" lost=%u cmd=%u speed=%u last_rx=", g_link_lost, g_cmd_cnt,
               Car_GetRunSpeed());

    if (b < 0) {
        rt_kprintf("-- -\n");
    } else {
        rt_kprintf("%02X %c\n", (unsigned)b,
                   (b >= 0x20 && b <= 0x7E) ? (char)b : '.');
    }

    rt_kprintf("motor1: target=");
    bt_print_int(motor_get_target(1));
    rt_kprintf(" speed=");
    bt_print_int(motor_get_speed(1));
    rt_kprintf(" duty=%u/%u\n", motor_get_duty(1), MOTOR_PWM_DUTY_MAX);

    rt_kprintf("motor2: target=");
    bt_print_int(motor_get_target(2));
    rt_kprintf(" speed=");
    bt_print_int(motor_get_speed(2));
    rt_kprintf(" duty=%u/%u\n", motor_get_duty(2), MOTOR_PWM_DUTY_MAX);
}

static void bt_msh_usage(void)
{
    rt_kprintf("usage: bt [status]                 show armed/state/link/motor\n");
    rt_kprintf("       bt speed <v>                 remote control target speed\n");
    rt_kprintf("       bt unlock | lock             same as sending 4F / 48\n");
    rt_kprintf("       bt link <ms> | off           link-loss brake timeout\n");
    rt_kprintf("       bt echo on | off             print every received byte\n");
    rt_kprintf("speed is in motor.h's Encoder-calibrated scale, 1 encoder count = %u\n",
               CAR_SPEED_PER_COUNT);
}

/* 解析一个非负整数（速度、超时都是整数：刻度上一个计数就是 24，
 * 小数位没有意义，而且 tiny vsnprintf 也不支持 %f）。 */
static rt_err_t bt_parse_uint(const char *s, rt_uint32_t *out)
{
    rt_uint32_t v = 0;
    int seen = 0;

    while (*s >= '0' && *s <= '9') {
        /* 溢出保护。不加这一句，`bt link 4294967296` 会绕回 0，而 0 在
         * bt_tick() 里的含义是"关掉失联保护"—— 手滑多打一位就把安全功能
         * 静默关掉了，失败方向太糟。上限取 (2^32-1-9)/10，保证 v*10+9 不绕回。 */
        if (v > 429496728u) {
            return -RT_EINVAL;
        }

        v = v * 10u + (rt_uint32_t)(*s - '0');
        seen = 1;
        s++;
    }

    if (!seen || *s != '\0') {
        return -RT_EINVAL;
    }

    *out = v;
    return RT_EOK;
}

/* 本体的入口对 speed_report 公开（无线串口上的 `#bt ...` 转发到这里），
 * msh 那边是个三行包装 —— 和 Car_CmdHead / Motor_Cmd 一样的分工。
 *
 * 【为什么它必须能走无线串口】失联急停（`bt link <ms>`）默认是关的，而它是
 * 上位机那套安全模型的一半。开车的时候车是电池供电的，插 USB 会让电机电源
 * 从 7.49V 塌到 3.56V（见 car.h），所以"上车之后再从 msh 打开它"这条路根本
 * 走不通 —— 要么在链路上开，要么这个功能事实上不存在。 */
rt_err_t Bt_Cmd(int argc, char **argv)
{
    rt_uint32_t v;

    if (argc < 2 || !rt_strcmp(argv[1], "status"))
    {
        bt_print_status();
        return RT_EOK;
    }

    if (!rt_strcmp(argv[1], "speed"))
    {
        if (argc < 3 || bt_parse_uint(argv[2], &v) != RT_EOK) {
            rt_kprintf("bt: usage: bt speed <v>   (0~%u)\n", CAR_SPEED_MAX);
            return -RT_EINVAL;
        }
        Car_SetRunSpeed((rt_uint16_t)v);
        rt_kprintf("bt: remote speed = %u (~%u encoder count(s) per 50ms sample)\n",
                   Car_GetRunSpeed(), Car_GetRunSpeed() / CAR_SPEED_PER_COUNT);
        return RT_EOK;
    }

    if (!rt_strcmp(argv[1], "link"))
    {
        if (argc < 3) {
            rt_kprintf("bt: usage: bt link <ms> | off\n");
            return -RT_EINVAL;
        }
        if (!rt_strcmp(argv[2], "off")) {
            bt_car_set_link_timeout_ms(0);
            rt_kprintf("bt: link-loss protection OFF (a dead phone will not stop the car)\n");
            return RT_EOK;
        }
        if (bt_parse_uint(argv[2], &v) != RT_EOK) {
            rt_kprintf("bt: bad timeout '%s'\n", argv[2]);
            return -RT_EINVAL;
        }
        bt_car_set_link_timeout_ms(v);
        rt_kprintf("bt: link-loss brake after %u ms of silence\n", v);
        return RT_EOK;
    }

    if (!rt_strcmp(argv[1], "unlock") || !rt_strcmp(argv[1], "lock"))
    {
        /* 走和串口收到 4F / 48 完全相同的路径，不用上位机也能验证状态机 */
        bt_apply((!rt_strcmp(argv[1], "unlock")) ? 'O' : 'H');
        rt_kprintf("bt: armed=%s state=%s\n", g_armed ? "yes" : "no", bt_car_state_name());
        return RT_EOK;
    }

    if (!rt_strcmp(argv[1], "echo"))
    {
        if (argc < 3) {
            rt_kprintf("bt: usage: bt echo on | off\n");
            return -RT_EINVAL;
        }
        g_echo = (!rt_strcmp(argv[2], "on")) ? RT_TRUE : RT_FALSE;
        rt_kprintf("bt: echo %s\n", g_echo ? "on" : "off");
        return RT_EOK;
    }

    bt_msh_usage();
    return -RT_EINVAL;
}
static int cmd_bt(int argc, char **argv)
{
    return (int)Bt_Cmd(argc, argv);
}
MSH_CMD_EXPORT_ALIAS(cmd_bt, bt, wireless serial remote control (status|speed|lock|unlock|link|echo));
