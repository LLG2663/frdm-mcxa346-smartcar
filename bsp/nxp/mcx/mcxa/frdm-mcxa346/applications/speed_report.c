/*
 * SPDX-License-Identifier: MIT
 *
 * speed_report.c —— 编码器速度的周期上报，以及从无线串口收命令。
 *
 * 接线、数据格式、为什么 TX 是 P1_10、为什么本模块不 open 设备 —— 都在
 * speed_report.h 顶部。本文件里与工程里其它模块的对应关系：
 *
 *   设备与波特率    → 用 bt_car.c 打开的 uart5，本模块只写不 open
 *   收字节的位置    → bt_car.c 读串口那里调 speed_report_rx_byte()
 *   回一行的位置    → bt_car.c 认出遥控字节后调 speed_report_reply()
 *   解锁状态        → bt_car_is_armed()（周期推送的门控，只读）
 *   线程 + 节拍      → 同 bt_car.c 的遥控线程，但这里只需要周期唤醒
 *   定点打印/解析    → 同 bt_car.c 的 bt_print_int() / bt_parse_uint()
 *   INIT_APP_EXPORT  → 同 bt_car.c，上电自动起
 */
#include <rtdevice.h>
#include <stdarg.h>

#include "speed_report.h"
#include "motor.h"
#include "car.h"
#include "bt_car.h"
#include "follow.h"

#define DBG_TAG    "spd"
#define DBG_LVL    DBG_INFO
#include <rtdbg.h>

/* 低于 bt(23) 和 led(25) 之间，且在 tshell(20) / motor(21) / jy901s(22) 之下 ——
 * 上报是纯输出，晚一拍没有任何影响。
 * 不设成 22 以下是因为它每周期要读 motor 的实测值，跟着 PID 之后走更合适。 */
#define SPD_THREAD_PRIO     24
#define SPD_THREAD_STACK    1024

/* 串口一行最多切几个词。最长命令是转发给 motor 的 "motor ff 1 4.2"，四个词
 * （"spd" 前缀写全了也一样长）。留到 6 是因为切词循环【词满了就停】，超出的部分
 * 是静默丢掉的 —— 不报错，命令会拿着缺了后半截的参数去执行。 */
#define SPD_ARGV_MAX        6

/* 一条命令的触发字节 —— 见 speed_report.h「为什么要那个 '#'」。 */
#define SPD_BT_TRIGGER      '#'

static rt_device_t          g_dev      = RT_NULL;
static rt_thread_t          g_tid      = RT_NULL;

/* 串口只有一个，但有两个线程会往它写：上报线程（周期性）和 bt_car 的遥控线程
 * （收到 '#' 命令时要回 ok/err）。drv_uart.c 的 poll 写是一条忙等自旋、没有
 * 内部锁（dev_serial.c 只在 INT_TX/DMA_TX 路径上加锁），两个线程同时写会把
 * 两行交错着发出去。所以这里自己上一把锁。 */
static rt_mutex_t           g_tx_lock  = RT_NULL;

/* 显示层、串口线程和 msh 都会读，上报线程会写 —— 都是 32 位以内、地址对齐的
 * 单次读写，不用额外加临界区（理由同 bt_car.c 里的 g_armed）。
 *
 * 【周期推送默认开】解锁之后每 SPD_RATE_DEFAULT_MS 自动推一行
 * `spd <m1> <m2> duty <d1> <d2> tgt <t1> <t2>` —— 两个电机的实时转速、
 * 占空比、以及外环真正下发的目标（见 spd_emit()）。
 * 这是看「两轮是不是被拉到同一档」最直接的一路：上位机解锁按住 W，屏幕上就是
 * 两列数字在跳，不需要再发 `#on`（上一版默认关，发 `#on` 又容易被串口助手的
 * 行尾设置坑掉）。
 *
 * 【仍然只在已解锁时推】上电和上锁时链路必须是安静的（bt_car_is_armed()），
 * 否则上位机一联上就是一屏 spd 0 0，真正的遥控回包还被压在后面。
 * 不想要就 `spd off` / 无线串口发 `#off`。 */
static volatile rt_bool_t   g_on       = RT_TRUE;
static volatile rt_uint32_t g_rate_ms  = SPD_RATE_DEFAULT_MS;
static volatile rt_uint32_t g_tx_ok    = 0;
static volatile rt_uint32_t g_tx_fail  = 0;
static volatile rt_uint32_t g_rx_line  = 0;     /* 收到并执行过的命令行数 */
static volatile rt_uint32_t g_rx_err   = 0;     /* 因过长/含不可打印字节被整行丢掉的条数 */

/* 最近发出去的一行，【不含行尾】—— 存带 \r\n 的原样会让 `spd once` 把引号
 * 打在 CRLF 后面，输出断成两行。 */
static char                 g_last_line[SPD_LINE_MAX];

/* 收行缓冲。只在 bt_car 的遥控线程里碰（就是 speed_report_rx_byte 的调用点），
 * 不在中断里碰，所以不用加锁。 */
static char                 g_rx_buf[SPD_RX_LINE_MAX];
static rt_size_t            g_rx_len    = 0;
static rt_bool_t            g_rx_active = RT_FALSE;   /* 正在收一条 '#' 命令 */
static rt_bool_t            g_rx_skip   = RT_FALSE;   /* 这一行已作废，丢到行尾为止 */
static rt_tick_t            g_rx_tick   = 0;          /* 上一个字节的时刻，用于超时 */

/* 上一次「收到遥控字节回一行」的时刻，给最小间隔用。只在遥控线程里读写。 */
static rt_tick_t            g_reply_tick = 0;

/* ============================================================================
 *  对外查询
 * ========================================================================== */
rt_bool_t speed_report_is_on(void)
{
    return g_on ? RT_TRUE : RT_FALSE;
}

rt_uint32_t speed_report_get_rate_ms(void)
{
    return g_rate_ms;
}

/* ============================================================================
 *  往串口写
 * ========================================================================== */
static void spd_tx_lock(void)
{
    /* g_tx_lock 为空只可能是 speed_report_init() 还没跑完 —— 那时 g_dev 也还是
     * 空的，写不出去，所以直接放行不会和谁撞上。 */
    if (g_tx_lock != RT_NULL)
    {
        rt_mutex_take(g_tx_lock, RT_WAITING_FOREVER);
    }
}

static void spd_tx_unlock(void)
{
    if (g_tx_lock != RT_NULL)
    {
        rt_mutex_release(g_tx_lock);
    }
}

static void spd_sendf(const char *fmt, ...)
{
    /* SPD_LINE_MAX 只够一行测量值，回包（比如 sta report=... ）更长，
     * 所以这里用两倍。64 字节放 1KB 栈上没问题。 */
    char      buf[SPD_LINE_MAX * 2];
    va_list   args;
    rt_size_t len;
    int       n;

    if (g_dev == RT_NULL)
    {
        return;
    }

    va_start(args, fmt);
    n = rt_vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (n <= 0)
    {
        return;
    }

    len = (rt_size_t)n;
    if (len >= sizeof(buf))
    {
        len = sizeof(buf) - 1;
    }

    spd_tx_lock();
    if (rt_device_write(g_dev, 0, buf, len) == 0)
    {
        g_tx_fail++;
    }
    else
    {
        g_tx_ok++;
    }
    spd_tx_unlock();
}

/* 公开版：给别的模块往这条链路上推自己的行用（目前只有 follow.c 的航点跟随
 * 遥测）。语义和 spd_sendf() 完全一样 —— 同一个 g_tx_lock、同一份 g_tx_ok/fail
 * 记账、g_dev 没起来时就当没写。
 *
 * 【只往链路写，不打控制台】调用方要是想让人在 USB 控制台上也看见，自己
 * rt_kprintf —— 这两条输出通路在 bt_car.c 的注释里解释过为什么不合并。 */
void speed_report_sendf(const char *fmt, ...)
{
    char    buf[SPD_LINE_MAX * 2];
    va_list args;
    rt_size_t len;
    int     n;

    if (g_dev == RT_NULL)
    {
        return;
    }

    va_start(args, fmt);
    n = rt_vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (n <= 0)
    {
        return;
    }

    len = (rt_size_t)n;
    if (len >= sizeof(buf))
    {
        len = sizeof(buf) - 1;
    }

    spd_tx_lock();
    if (rt_device_write(g_dev, 0, buf, len) == 0)
    {
        g_tx_fail++;
    }
    else
    {
        g_tx_ok++;
    }
    spd_tx_unlock();
}

/* ============================================================================
 *  拼一行 / 写一行
 * ========================================================================== */

/* 一行已经拼好的（带行尾）写出去：校验长度、给 msh 的 `spd status` 留一份回显、
 * 加锁写、记账。n 是 rt_snprintf 的返回值。
 *
 * 调用方的缓冲【必须是 SPD_LINE_MAX 大小】，下面按这个当上界判超长。
 * 格式串最短也产出 "spd 0 0\r\n"（9 字节）这种，所以 n < 2 只可能是
 * rt_snprintf 本身出了问题。两个都得挡：n < 2 时下面 len - 2 会无符号下溢，
 * 超长则说明那一行根本没完整装进缓冲。 */
static rt_size_t spd_put_line(const char *line, int n)
{
    rt_size_t len;

    if (n < 2 || (rt_size_t)n >= SPD_LINE_MAX)
    {
        return 0;
    }
    len = (rt_size_t)n;

    /* 顺手给 msh 和串口回包留一份【去掉行尾】的备份 —— 行尾固定两个字符。 */
    rt_memcpy(g_last_line, line, len - 2);
    g_last_line[len - 2] = '\0';

    if (g_dev == RT_NULL)
    {
        g_tx_fail++;
        return 0;
    }

    /* 写失败只记数、不刷日志：设备没打开时 rt_device_write() 安全返回 0
     * （bt_car 还没起来就是这种情况，init 里已经报过了）。 */
    spd_tx_lock();
    if (rt_device_write(g_dev, 0, line, len) == 0)
    {
        g_tx_fail++;
        spd_tx_unlock();
        return 0;
    }
    spd_tx_unlock();

    g_tx_ok++;
    return len;
}

/* 把速度值换成「每 50ms 采样的编码器计数」：四舍五入、保留符号。
 * 定点和 spd_print_val() 同一套规矩（刻度上一个计数就是 24，小数位没有意义；
 * 不用 fabsf 是为了不引 libm）。c != 0 那个判断是为了不打出 "-0"
 * （spd_print_val() 里同样的处理）。 */
static rt_int32_t spd_to_counts(float v)
{
    float      mag = (v < 0.0f) ? -v : v;
    rt_int32_t c   = (rt_int32_t)(mag / (float)CAR_SPEED_PER_COUNT + 0.5f);

    return (rt_int32_t)((v < 0.0f && c != 0) ? -c : c);
}

/* 周期推送那一行：spd <m1> <m2> duty <d1> <d2> tgt <t1> <t2>。
 *
 * 用四舍五入而不是截断（spd_to_counts 里就是）：轮子刚转起来时一个采样可能
 * 不满一个计数，截断会把它一直打成 0，看起来像「没转」。
 *
 * 没有方向 —— 编码器只接了 A 相，读数本身就是非负幅值（motor.c 文件头
 * 「只计 A 相」那一段），方向看这一行末尾的 tgt。
 *
 * ------------------------------------------------- 为什么带上 duty（0~4000）----
 *  因为【插着 USB 就量不到占空比】：电源模块的 VADJ 一插 USB 就从 7.49V 塌到
 *  3.56V（实测），msh 的 `motor status` 只有插 USB 才连得上 —— 也就是说，
 *  「车真正跑起来的时候占空比是多少」用 msh 【永远看不到】。又因为车正常是
 *  电池供电（7.49V），USB 下的读数根本不是它工作时的读数。
 *
 *  这一栏要回答的是：两轮的占空比是【在调节】还是【顶死在 4000】。
 *    - 都在 1000~3000 且随时间小幅变化 → 内环在正常工作，两轮转速差是
 *      别的原因（起步不同步，见 motor.c 的 DC_MOTOR_PID）。
 *    - 有一路长期 = 4000 → 那一路已经饱和，目标转速在这个电压下【够不到】，
 *      再怎么调 kp/ki/kd 或修 PID 都没用，得先从供电/机械上解决。
 *
 * ------------------------------------------------------ 为什么又加上 tgt ----
 *  【这是航向外环唯一能被看见的地方】tgt 就是 Car_Apply() 真正下发的目标
 *  （= 上层基准 ± corr），corr 是外环的全部输出、也是它唯一的手段。在这之前
 *  tgt 只出现在「认出一个遥控字节就回一行」那条路上（spd_send_pair），而按住
 *  W 时那条路每 SPD_RATE_MIN_MS 才回一行、还和按键节奏纠缠在一起 —— 车跑起来
 *  的那整段时间里，外环到底在不在动、动多少，一行都采不到。
 *
 *  加上之后，一条周期行就能读出三件事：
 *      spd 17 17 duty 1600 1970 tgt 17 17   → 两轮都到位，corr ≈ 0，外环没在使劲
 *      spd 17 16 duty 1600 1970 tgt 19 15   → 外环正在压右轮（corr ≈ +2 个计数），
 *                                             而实测仍差 1 个计数 ⇒ 权限不够/内环饱和
 *  corr 本身不单独打：它是 tgt 减去基准（`bt speed`，默认 CAR_SPEED_RUN），
 *  打两个数就是同一个信息说两遍。
 *
 *  一行最长 "spd 208 208 duty 4000 4000 tgt -208 -208\r\n" = 41 字节
 *  （实测满占空比约 54 个计数、目标上限 50 个计数，正常就是两位），
 *  SPD_LINE_MAX(48) 够。
 * 【不要】把这套再挪一份到 spd_send_pair() 去 —— 那一行是按键连发时最短
 *  间隔 50ms 回来的，再加一段会明显加重 115200 上那段忙等自旋。 */
static rt_size_t spd_emit(void)
{
    char line[SPD_LINE_MAX];            /* 真正写进串口的，带行尾 */
    int  n;

    /* 【别改成 %f】rt_snprintf 用的也是 tiny vsnprintf，不支持 %f，会原样
     * 打出 "%f"。刻度上一个计数就是 24，小数位本来也没有意义。 */
    n = rt_snprintf(line, sizeof(line), "spd %d %d duty %u %u tgt %d %d\r\n",
                    (int)spd_to_counts(motor_get_speed(1)),
                    (int)spd_to_counts(motor_get_speed(2)),
                    (unsigned)motor_get_duty(1),
                    (unsigned)motor_get_duty(2),
                    (int)spd_to_counts(motor_get_target(1)),
                    (int)spd_to_counts(motor_get_target(2)));

    return spd_put_line(line, n);
}

/* 【实际 + 预期】一行。两条路共用它：
 *   - 遥控线程每认出一个遥控字节（speed_report_reply）
 *   - msh 的 `spd both` / 无线串口上的 `#both`（主动要一份）
 *
 * spd 后面是【实际】（幅值，无方向），tgt 后面是【预期】（motor_get_target()，
 * 带正负号 —— 方向只能从这一行看，编码器只接了 A 相），刻度同上。
 * 前面那段 "spd <m1> <m2>" 和周期行【完全同形】，所以助手已有的解析不用改，
 * 只是后面多了一段 tgt。 */
static rt_size_t spd_send_pair(void)
{
    char line[SPD_LINE_MAX];
    int  n;

    n = rt_snprintf(line, sizeof(line), "spd %d %d tgt %d %d\r\n",
                    (int)spd_to_counts(motor_get_speed(1)),
                    (int)spd_to_counts(motor_get_speed(2)),
                    (int)spd_to_counts(motor_get_target(1)),
                    (int)spd_to_counts(motor_get_target(2)));

    return spd_put_line(line, n);
}

/* ============================================================================
 *  收到遥控字节就回一行 —— bt_car.c 调用
 * ========================================================================== */
void speed_report_reply(void)
{
    rt_tick_t now = rt_tick_get();

    /* 【最小间隔】按住按键/摇杆时遥控字节是连发的。每个字节都回一行的话，
     * 115200 下一行约 2ms 的忙等自旋会把遥控线程的读取节奏拖到后面去，RX
     * 环形缓冲（64 字节）也可能被灌满。下限取周期上报那个数，同一个理由。
     * 用无符号减法比 tick，回绕（约 49.7 天）也是对的（同 bt_car.c 的 bt_tick）。 */
    if ((rt_uint32_t)(now - g_reply_tick) <
        (rt_uint32_t)rt_tick_from_millisecond(SPD_RATE_MIN_MS))
    {
        return;
    }
    g_reply_tick = now;

    spd_send_pair();
}

/* ============================================================================
 *  命令解析（msh 和无线串口共用同一套）
 * ==========================================================================
 *  argv[0] 是子命令。rt_kprintf 用的是 tiny vsnprintf，不支持 %f，
 *  LFLAGS 也没有 -lm —— 刻度上一个计数就是 24，小数位没有意义，
 *  所以定点打印（和 bt_car.c 的 bt_print_int() 同一套路）。
 * ========================================================================== */
static void spd_print_val(float v)
{
    float mag = (v < 0.0f) ? -v : v;            /* 不用 fabsf，省掉 libm 依赖 */
    rt_int32_t c = (rt_int32_t)(mag / (float)CAR_SPEED_PER_COUNT + 0.5f);

    if (v < 0.0f && c != 0)
    {
        rt_kprintf("-");
    }
    rt_kprintf("%d", (int)c);
}

static void spd_print_status(void)
{
    if (g_dev == RT_NULL)
    {
        rt_kprintf("spd: report=%s rate=%ums uart=%s NOT FOUND (BSP_USING_UART5 没开？)\n",
                   g_on ? "on" : "off", (unsigned)g_rate_ms, SPD_DEV_NAME);
    }
    else
    {
        /* 这个口是 bt_car.c 打开的（115200），本模块只借来写 */
        rt_kprintf("spd: report=%s rate=%ums uart=%s (opened by bt_car, 115200 8N1)\n",
                   g_on ? "on" : "off", (unsigned)g_rate_ms, SPD_DEV_NAME);
        rt_kprintf("spd: tx=%u fail=%u rx_line=%u rx_dropped=%u\n",
                   (unsigned)g_tx_ok, (unsigned)g_tx_fail,
                   (unsigned)g_rx_line, (unsigned)g_rx_err);
    }

    if (g_last_line[0] != '\0')
    {
        rt_kprintf("spd: last: %s\n", g_last_line);
    }
    else
    {
        rt_kprintf("spd: last: (nothing sent yet)\n");
    }

    rt_kprintf("motor1: target=");
    spd_print_val(motor_get_target(1));
    rt_kprintf(" speed=");
    spd_print_val(motor_get_speed(1));
    rt_kprintf(" duty=%u/%u\n", (unsigned)motor_get_duty(1), MOTOR_PWM_DUTY_MAX);

    rt_kprintf("motor2: target=");
    spd_print_val(motor_get_target(2));
    rt_kprintf(" speed=");
    spd_print_val(motor_get_speed(2));
    rt_kprintf(" duty=%u/%u\n", (unsigned)motor_get_duty(2), MOTOR_PWM_DUTY_MAX);

    rt_kprintf("(unit: 1 encoder count / 50ms sample = %u, same scale as `bt speed`)\n",
               CAR_SPEED_PER_COUNT);
}

static void spd_usage(void)
{
    rt_kprintf("usage: spd [status]              report state, tx/rx counters, last line\n");
    rt_kprintf("       spd on | off              periodic push on/off (default: on)\n");
    rt_kprintf("       spd rate <%u~%u>       report period in ms (default %u)\n",
               SPD_RATE_MIN_MS, SPD_RATE_MAX_MS, SPD_RATE_DEFAULT_MS);
    rt_kprintf("       spd once                  send one line right now\n");
    rt_kprintf("       spd both                  send actual + target (one line)\n");
    rt_kprintf("the periodic push is: spd <a1> <a2> duty <d1> <d2> tgt <t1> <t2>\n");
    rt_kprintf("  a = actual (magnitude only), t = target (signed) = base +/- corr;\n");
    rt_kprintf("  it is where the straight-line assist is visible while the car drives.\n");
    rt_kprintf("1 count = %u on every column. The push waits for 4F (silent while disarmed)\n",
               CAR_SPEED_PER_COUNT);
    rt_kprintf("a recognised remote byte answers one line: spd <a1> <a2> tgt <t1> <t2>\n");
    rt_kprintf("over %s (wireless) the same words go behind a '%c', e.g. \"%crate 200\\n\"\n",
               SPD_DEV_NAME, SPD_BT_TRIGGER, SPD_BT_TRIGGER);
    rt_kprintf("the '%c' is required: W/S/A/D/space/X/H/O on that link are bt_car's\n",
               SPD_BT_TRIGGER);
    rt_kprintf("%chead ... and %cmotor ... are forwarded too (for tuning on battery \n",
               SPD_BT_TRIGGER, SPD_BT_TRIGGER);
    rt_kprintf("power, where USB would collapse the motor supply): e.g. \"%chead kd 5\"\n",
               SPD_BT_TRIGGER);
    rt_kprintf("%cfollow ... is forwarded too (waypoint following): \"%cfollow status\"\n",
               SPD_BT_TRIGGER, SPD_BT_TRIGGER);
    rt_kprintf("%cbt ... as well -- the link-loss brake is OFF by default and the car\n",
               SPD_BT_TRIGGER);
    rt_kprintf("runs on battery, so \"%cbt link 1000\" is the only way to arm it\n",
               SPD_BT_TRIGGER);
}

/* 解析一个非负整数（周期都是整数毫秒）。照抄 bt_car.c 的 bt_parse_uint()，
 * 包括那条溢出保护 —— 不加的话 `spd rate 4294967296` 会绕回 0。 */
static rt_err_t spd_parse_uint(const char *s, rt_uint32_t *out)
{
    rt_uint32_t v = 0;
    int seen = 0;

    while (*s >= '0' && *s <= '9') {
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

/* 执行一条子命令。msh 的 `spd` 和无线串口上收到的行都走这里 ——
 * 一套逻辑两个入口，不会出现「msh 能改、无线串口改了没反应」这种事。 */
static rt_err_t spd_dispatch(int argc, char **argv)
{
    rt_uint32_t v;

    if (argc < 1 || !rt_strcmp(argv[0], "status"))
    {
        spd_print_status();
        return RT_EOK;
    }

    if (!rt_strcmp(argv[0], "on") || !rt_strcmp(argv[0], "off"))
    {
        g_on = (!rt_strcmp(argv[0], "on")) ? RT_TRUE : RT_FALSE;
        rt_kprintf("spd: periodic report %s (every %u ms)\n",
                   g_on ? "on" : "off", (unsigned)g_rate_ms);
        return RT_EOK;
    }

    if (!rt_strcmp(argv[0], "rate"))
    {
        if (argc < 2 || spd_parse_uint(argv[1], &v) != RT_EOK) {
            rt_kprintf("spd: usage: spd rate <%u~%u>\n",
                       SPD_RATE_MIN_MS, SPD_RATE_MAX_MS);
            return -RT_EINVAL;
        }

        /* 【下限不能松】这条链路是 115200 且发送路径是忙等自旋（drv_uart.c 的
         * mcx_putc 里是个 while），一行要占掉约 4ms 的 CPU。周期设到个位数
         * 毫秒会让本线程把 led(25) 和显示(26) 饿死。 */
        if (v < SPD_RATE_MIN_MS || v > SPD_RATE_MAX_MS) {
            rt_kprintf("spd: rate out of range (%u~%u ms), got %u\n",
                       SPD_RATE_MIN_MS, SPD_RATE_MAX_MS, (unsigned)v);
            return -RT_EINVAL;
        }

        g_rate_ms = v;
        rt_kprintf("spd: report period = %u ms\n", (unsigned)v);
        return RT_EOK;
    }

    if (!rt_strcmp(argv[0], "once"))
    {
        /* 关掉周期上报时也能用，用来验证链路（上位机连上敲一下，看收到没有）。
         * 【阻塞注意】这里会自旋约 5ms。msh 上无所谓；从无线串口收到时它跑在
         * bt_car 的遥控线程里，那一下遥控会晚 5ms —— 一次性命令，可以接受。 */
        if (spd_emit() == 0) {
            rt_kprintf("spd: nothing sent (fail=%u)\n", (unsigned)g_tx_fail);
            return -RT_ERROR;
        }
        rt_kprintf("spd: sent \"%s\"\n", g_last_line);
        return RT_EOK;
    }

    if (!rt_strcmp(argv[0], "both"))
    {
        /* 把「实际 + 预期」那一行推到链路上（主要入口是无线串口，上位机上看那行）。
         * msh 上看不到链路，所以同样的数值也打一份到控制台 —— 敲了不该没反应。
         * 控制台上只想查数值可以直接用 `spd status`。 */
        spd_send_pair();

        rt_kprintf("spd: actual=");
        spd_print_val(motor_get_speed(1));
        rt_kprintf(" ");
        spd_print_val(motor_get_speed(2));
        rt_kprintf(" target=");
        spd_print_val(motor_get_target(1));
        rt_kprintf(" ");
        spd_print_val(motor_get_target(2));
        rt_kprintf("\n");
        return RT_EOK;
    }

    /* 航点跟随。转发而不是在这里实现，理由和上面 head/motor 一样 ——
     * 而且它更需要走无线串口：标定要走卷尺（车得落地），而 msh 一插 USB
     * 电机电源就从 7.49V 塌到 3.56V（见 car.h），航向、转向率全都不对。 */
    if (!rt_strcmp(argv[0], "follow"))
    {
        return Follow_Cmd(argc, argv);
    }

    spd_usage();
    return -RT_EINVAL;
}

/* ============================================================================
 *  从无线串口收命令
 * ==========================================================================
 *  入口是 speed_report_rx_byte()，由 bt_car.c 在遥控分派【之前】逐字节调用。
 *  这里只管按行攒字节、到行尾调 spd_run_line()，认领与否用一个返回值告诉
 *  调用方 —— 见 speed_report.h「为什么要那个 '#'」。
 * ========================================================================== */

/* 把收到的一行切词后执行，并按结果回一行 ok / err。
 * 就地改 line（把空格换成 '\0'），反正它是一次性缓冲。 */
static void spd_run_line(char *line)
{
    char       *argv[SPD_ARGV_MAX];
    char       *p = line;
    int         argc = 0;
    int         i;
    rt_err_t    rc;

    while (*p != '\0' && argc < SPD_ARGV_MAX)
    {
        while (*p == ' ' || *p == '\t') { p++; }
        if (*p == '\0') { break; }

        argv[argc++] = p;
        while (*p != '\0' && *p != ' ' && *p != '\t') { p++; }
        if (*p != '\0') { *p++ = '\0'; }
    }

    if (argc == 0)
    {
        return;                     /* 空行 */
    }

    /* "spd " 前缀可写可不写 —— '#' 已经表明身份了，写全了也能认
     * （和 msh 上敲的一模一样）。 */
    if (!rt_strcmp(argv[0], "spd"))
    {
        if (argc == 1)              /* 只发了 "#spd" */
        {
            spd_print_status();
            spd_sendf("ok status\r\n");
            return;
        }
        for (i = 1; i < argc; i++) { argv[i - 1] = argv[i]; }
        argc--;
    }

    /* ---------------------------------------------------------- 转发 -------------
     *  【head / motor 为什么也要走无线串口】这两个命令的旋钮只能"看着车"调，而
     *  车得落地跑、落地就得电池供电 —— 可 msh 要插 USB，一插 USB 电机电源
     *  VADJ 就从 7.49V 塌到 3.56V（见 car.h / 本文件底部）。也就是说：插着 USB
     *  调出来的 kp/ki/kd、前馈系数、修正符号，没有一个是车真正工作时的那份。
     *  `spd rate` 能走无线串口是同一条理由，这里只是把同一个入口开给另外两个模块。
     *
     *  【只转发这两个名字，不做通用 shell 转发】'#' 那套闸门（见 speed_report.h
     *  「为什么要那个 '#'」）本身就是为了让命令里的字符别被当成遥控命令执行；
     *  开成一条"什么都能跑"的远程 shell 等于把这个闸门废掉，而且那些命令的回包
     *  是 rt_kprintf，本来也只在 USB 控制台上看得见。
     *
     *  两个函数都按 msh 的约定收 argv：argv[0] 是命令名，子命令从 argv[1] 起。
     *  控制台那份输出照常打到 uart2（这里够不着，也没必要拦）；无线串口上只要
     *  一行 ack，数值从周期行的 tgt / duty 里读。 */
    if (!rt_strcmp(argv[0], "head") || !rt_strcmp(argv[0], "motor") ||
        !rt_strcmp(argv[0], "bt"))
    {
        rc = (!rt_strcmp(argv[0], "head")) ? Car_CmdHead(argc, argv)
           : (!rt_strcmp(argv[0], "motor")) ? Motor_Cmd(argc, argv)
                                            : Bt_Cmd(argc, argv);
        g_rx_line++;
        spd_sendf("%s %s\r\n", (rc == RT_EOK) ? "ok" : "err", argv[0]);
        return;
    }

    /* status 走紧凑回包：控制台那份是给人看的，无线串口上只要几个数。 */
    if (!rt_strcmp(argv[0], "status"))
    {
        spd_print_status();
        spd_sendf("sta report=%s rate=%u tx=%u fail=%u\r\n",
                  g_on ? "on" : "off", (unsigned)g_rate_ms,
                  (unsigned)g_tx_ok, (unsigned)g_tx_fail);
        return;
    }

    /* both 和 status 同一套路：回包里只要数据，不补 "ok both" 那一行 ——
     * 助手是按行解析的，多一行 ack 只会让界面多滚一行。走 spd_dispatch()
     * 的话回包会变成两行（数据那行 + ok），所以在这里就返回。
     * 注意这里【没走 dispatch】，所以 msh 上的 `spd both` 是另一个入口，
     * 变化见 spd_dispatch()。 */
    if (!rt_strcmp(argv[0], "both"))
    {
        spd_send_pair();
        return;
    }

    rc = spd_dispatch(argc, argv);
    g_rx_line++;
    spd_sendf("%s %s\r\n", (rc == RT_EOK) ? "ok" : "err", argv[0]);
}

/* 一行收着收着断了：可能是链路噪声里蹦出一个 '#' 然后再没下文，也可能是
 * 助手只发了半条。整条作废、恢复成普通遥控字节。
 *
 * 每个字节进来先查一次，所以没有「一直卡在收命令状态」这回事 —— 下一个字节
 * 到达时就会先把过期的状态清掉，再按普通字节处理它。 */
static void spd_rx_expire(void)
{
    if (!g_rx_active)
    {
        return;
    }

    if ((rt_tick_get() - g_rx_tick) <= rt_tick_from_millisecond(SPD_BT_LINE_TIMEOUT_MS))
    {
        return;
    }

    LOG_W("rx line timed out after %u ms, dropped", (unsigned)SPD_BT_LINE_TIMEOUT_MS);
    g_rx_active = RT_FALSE;
    g_rx_skip   = RT_FALSE;
    g_rx_len    = 0;
    g_rx_err++;
}

rt_bool_t speed_report_rx_byte(rt_uint8_t ch)
{
    spd_rx_expire();
    g_rx_tick = rt_tick_get();

    if (!g_rx_active)
    {
        if (ch != (rt_uint8_t)SPD_BT_TRIGGER)
        {
            return RT_FALSE;        /* 遥控的字节，原样交给 bt_car 去执行 */
        }

        g_rx_active = RT_TRUE;
        g_rx_skip   = RT_FALSE;
        g_rx_len    = 0;
        return RT_TRUE;             /* '#' 自己不是遥控命令，认领掉 */
    }

    /* 行尾：到这里才真的执行。\r 和 \n 都当行尾，所以 CRLF 只会产生一个空行。 */
    if (ch == '\r' || ch == '\n')
    {
        if (!g_rx_skip && g_rx_len > 0)
        {
            g_rx_buf[g_rx_len] = '\0';
            spd_run_line(g_rx_buf);
        }

        g_rx_active = RT_FALSE;
        g_rx_skip   = RT_FALSE;
        g_rx_len    = 0;
        return RT_TRUE;
    }

    if (g_rx_skip)
    {
        return RT_TRUE;             /* 已作废的那一行，丢到行尾为止 */
    }

    /* 不可打印的字节不可能是命令的一部分。整行作废。
     * 【不能只把 g_rx_len 清 0 接着收】那样这行的后半截会被当成完整命令执行。 */
    if (ch < 0x20 || ch > 0x7E)
    {
        LOG_W("rx line has a non-printable byte, dropped");
        g_rx_skip = RT_TRUE;
        g_rx_len  = 0;
        g_rx_err++;
        return RT_TRUE;
    }

    if (g_rx_len >= SPD_RX_LINE_MAX - 1)
    {
        LOG_W("rx line too long, dropped (bytes %u+)", (unsigned)SPD_RX_LINE_MAX);
        g_rx_skip = RT_TRUE;
        g_rx_len  = 0;
        g_rx_err++;
        return RT_TRUE;
    }

    g_rx_buf[g_rx_len++] = (char)ch;
    return RT_TRUE;
}

/* ============================================================================
 *  线程：只管周期推送
 * ==========================================================================
 *  收命令不在这里 —— 字节是 bt_car 的 rx_indicate 收到的，解析就贴在它的
 *  读取循环里（speed_report_rx_byte）。这样只有一个读者，收命令的延迟也不
 *  被上报周期拖累。回包（speed_report_reply）同理，走的是遥控线程。
 *
 *  【两道门都要开才推】g_on（默认【开】，`spd off` / `#off` 关）+ 已解锁。
 *  上锁/上电时链路必须是安静的：否则上位机一联上就是一屏 spd 0 0，真正的
 *  遥控回包还被压在后面。要改成「不管解没解锁都推」，把下面那个
 *  bt_car_is_armed() 判断去掉即可 —— 默认保留是因为安静的上电态更有用。
 * ========================================================================== */
static void spd_thread(void *parameter)
{
    RT_UNUSED(parameter);

    while (1)
    {
        rt_thread_mdelay(g_rate_ms);    /* 每轮重新读，`spd rate` 改完下一轮生效 */

        if (g_on && bt_car_is_armed())
        {
            spd_emit();
        }
    }
}

rt_err_t speed_report_init(void)
{
    rt_device_t dev;

    if (g_tid != RT_NULL) {
        return RT_EOK;      /* 重复调用直接返回，别把线程建两遍 */
    }

    dev = rt_device_find(SPD_DEV_NAME);
    if (dev == RT_NULL)
    {
        /* 多半是 .config 里没开 BSP_USING_UART5。只报错返回：
         * IMU、OLED、电机、无线串口遥控和 msh 照样能用。 */
        LOG_E("serial device \"%s\" not found; check BSP_USING_UART5", SPD_DEV_NAME);
        return -RT_ERROR;
    }

    /* 【这里不 open、不 configure、不设 rx_indicate】理由见 speed_report.h：
     * 那个口是 bt_car.c 打开的，参数（含 115200）都归它；本模块另开一次会把
     * 它的 open_flag / rx_indicate 挤掉。 */
    g_tx_lock = rt_mutex_create("spdtx", RT_IPC_FLAG_PRIO);
    if (g_tx_lock == RT_NULL)
    {
        LOG_E("failed to create the tx mutex");
        return -RT_ERROR;
    }

    /* 锁建好了才把 g_dev 放出去 —— 写路径靠「g_dev 非空 ⇒ 锁已建好」来省掉
     * 空指针判断。 */
    g_dev = dev;

    g_tid = rt_thread_create("spd", spd_thread, RT_NULL,
                             SPD_THREAD_STACK, SPD_THREAD_PRIO, 10);
    if (g_tid == RT_NULL)
    {
        LOG_E("failed to create the speed report thread");
        rt_mutex_delete(g_tx_lock);
        g_tx_lock = RT_NULL;
        g_dev     = RT_NULL;
        return -RT_ERROR;
    }
    rt_thread_startup(g_tid);

    LOG_I("%s (wireless link) report every %u ms, commands behind '%c'",
          SPD_DEV_NAME, SPD_RATE_DEFAULT_MS, SPD_BT_TRIGGER);
    return RT_EOK;
}

/* 上电自动起来。默认就开着 —— 上报是纯输出，不像 bt_car.c 那样需要考虑
 * 「上电别让车动起来」，没有设闸门的理由。 */
static int speed_report_auto_init(void)
{
    return (int)speed_report_init();
}
INIT_APP_EXPORT(speed_report_auto_init);

/* ============================================================================
 *  msh 命令：直接复用 spd_dispatch()，去掉 argv[0] 那个 "spd"
 * ========================================================================== */
static int cmd_spd(int argc, char **argv)
{
    return (int)spd_dispatch(argc - 1, argv + 1);
}
MSH_CMD_EXPORT_ALIAS(cmd_spd, spd, encoder speed report over the wireless link (status|on|off|rate|once|both));
