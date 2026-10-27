/*
 * SPDX-License-Identifier: MIT
 *
 * JY901S 姿态模块驱动，移植自 CCS 工程 113/user_driver/uart.c（TI MSPM0G3507）
 * 到 FRDM-MCXA346 上的 RT-Thread。
 *
 * 帧同步器、校验、yaw 校零/归一/符号、低通滤波这些与平台无关，逐行照搬，
 * 连注释都保留原样（包括那条"转向相反就改 JY61P_YAW_SIGN"的提醒）。
 * 真正改写的是数据通路，标了 [PORT]：
 *
 *   [PORT] 中断服务  TI 的 DL_UART_Main_* 五步硬件操作
 *                    → RT-Thread 串口 INT_RX 环形缓冲 + rt_device_read()
 *   [PORT] 解析线程  原工程在主循环里轮询 JY61P_HasNewData()，
 *                    这里把"收字节喂解析器"放进独立线程，主循环只读结果
 *   [PORT] 诊断计数  新增收字节数/有效帧数，用来区分"没收到"和"解不出"
 *
 * 接线：模块 TX → P3_13（Arduino D9，LPUART3_RXD，ALT3）
 *       模块 RX ← P3_12（LPUART3_TXD，ALT3，可选；源工程没接这根）
 *       模块 GND ↔ 板子 GND，模块 VCC ← 3V3 或 5V（按模块丝印）
 *       波特率 115200 8N1，与模块出厂默认一致。
 */

#include <rtdevice.h>

/* [自检] 只读 PORT3 / LPUART3 寄存器要用。SDK 头路径由 MCXA346/SConscript 的
 * DefineGroup(..., CPPPATH = path) 加进全局 CPPPATH（tools/building.py 里
 * Env.AppendUnique 到全局 env），drv_uart.c 就是这么拿到 fsl_lpuart.h 的。 */
#include "fsl_port.h"

#include "jy901s.h"

#define DBG_TAG    "jy901s"
#define DBG_LVL    DBG_INFO
#include <rtdbg.h>

/* [PORT] 串口设备名与波特率，对应 drv_uart.c 里的 "uart3" */
#define JY901S_DEV_NAME     "uart3"
#define JY901S_BAUD_RATE    115200

#define JY901S_THREAD_PRIO  22          /* 高于 led 线程(25)、低于 FinSH 的 tshell(20) */
#define JY901S_THREAD_STACK 1024
#define JY901S_RX_CHUNK     32

volatile JY61P_Data_t jy61p = {0};

/* 接收缓冲只在解析线程里访问，不需要 volatile；
 * 而 filtered_yaw / filtered_wz / new_data_flag 由解析线程写、使用方读，
 * 必须加 volatile，否则 -O2/LTO 下可能被缓存在寄存器里而永远读不到新值。 */
static uint8_t  jyp_rx_buf[32];
static uint8_t  jyp_rx_len = 0;

static volatile float   filtered_yaw = 0.0f;
static volatile float   filtered_wz  = 0.0f;
static volatile uint8_t filter_init_yaw = 0;
static volatile uint8_t filter_init_wz  = 0;
static volatile uint8_t new_data_flag = 0;

/* yaw 校零。yaw_offset 由解析线程写、使用方读（32 位对齐单次读写，不用临界区）；
 * yaw_boot_cnt 只在解析线程里增，yaw_zeroed 由解析线程置 1、使用方可清 0。 */
static volatile float   yaw_offset   = 0.0f;
static volatile uint8_t yaw_zeroed   = 0;
static uint8_t          yaw_boot_cnt = 0;

/* [PORT] 诊断计数：收了几个字节、解出几个校验正确的帧。
 * "什么都没显示"时先看这两个数：
 *   rx_bytes = 0            → 一个字节都没收到，查接线、查 P3_13 复用是不是 ALT3
 *   rx_bytes > 0, frames = 0 → 收到了但解不出，查波特率、查模块是不是 115200 输出 */
static volatile rt_uint32_t rx_bytes   = 0;
static volatile rt_uint32_t frame_count = 0;

/* [自检] 上面 rx_bytes 是在【解析线程】里自增的，所以"引脚上没有边沿"和
 * "边沿来了、中断也进了、但线程没把字节读走"这两种原因长得一模一样。
 * 这两个数只在 rx_indicate（中断上下文）里写，因此
 *   rx_irq_cnt > 0  ⇔  P3_13 上确实出现过电平跳变，
 * 与线程有没有被调度无关，正好把上面两种原因分开。
 *
 * ⚠ size 是【当前环形缓冲占用量】而不是本次增量
 * （dev_serial.c 传的是 put_index - get_index），所以只能取最大值，
 * 千万不能累加，累加出来的数没有意义。 */
static volatile rt_uint32_t rx_irq_cnt  = 0;   /* rx_indicate 被调用的次数 */
static volatile rt_uint32_t rx_irq_max  = 0;   /* 单次回调报告的缓冲积压字节数（最大值） */
static volatile rt_uint32_t rx_wake_cnt = 0;   /* 解析线程被信号量唤醒的次数 */

#define JY61P_FRAME_LEN    11
#define JY61P_HEAD1        0x55
#define JY61P_HEAD2_ANGLE  0x53   /* 角度包: Roll Pitch Yaw + 温度 */
#define JY61P_HEAD2_GYRO   0x52   /* 角速度包: Gx Gy Gz + 温度 */

#define KALMAN_K           0.3f   /* 一阶低通系数（不是卡尔曼，命名沿用） */

/* ---- yaw 校零与符号 --------------------------------------------------------
 * 模块上电后报的是它自己的绝对朝向，随摆放方向任意变化。
 * 这里在启动时锁定一次零点，之后对外给的都是"相对上电朝向"的转角：
 *     yaw = (raw_yaw - yaw_offset) * JY61P_YAW_SIGN   再归一到 -180~+180
 * 于是上电瞬间显示 0.00，转过 ±180 后从另一端回来。
 * -------------------------------------------------------------------------- */
#define JY61P_YAW_SIGN     (-1.0f)  /* 符号，见下方说明 */
#define JY61P_ZERO_SKIP    5        /* 上电先丢几帧，避开模块刚上电时的不稳定输出 */

/* 符号约定：JY901S 原始输出（右手法则、模块 Z 轴朝上）是"左转 yaw 增大"（CCW 为正），
 * 本工程要求"左转为负、右转为正"，所以默认取 -1。
 * 实测若发现转向与 OLED 读数相反，把这里改成 (+1.0f) 即可，别的都不用动。 */

/* yaw 是否需要再走一阶低通。
 * 模块内部已完成姿态融合，外挂低通只贡献相位滞后（10Hz 下时间常数约 300ms），
 * 而 filtered_yaw 同时用于转向 PID 和 |err|<TOLERANCE 的停车判据，滞后会直接变成过冲。
 * 默认关闭；wz 是原始陀螺输出，滤波有意义，故保留。 */
#define JY61P_YAW_LPF_EN   0

/* [PORT] 串口设备句柄、收到字节的信号量、解析线程 */
static rt_device_t jy901s_dev = RT_NULL;
static struct rt_semaphore jy901s_rx_sem;
static rt_thread_t jy901s_tid = RT_NULL;

float JY61P_GetYaw(void)
{
    return (float)jy61p.yaw;
}

float JY61P_GetFilteredYaw(void)
{
    return filtered_yaw;
}

float JY61P_GetFilteredWz(void)
{
    return filtered_wz;
}

uint8_t JY61P_HasNewData(void)
{
    return new_data_flag;
}

void JY61P_ClearNewData(void)
{
    new_data_flag = 0;
}

void JY61P_ResetFilter(float yaw, float wz)
{
    filtered_yaw = yaw;
    filtered_wz = wz;
    filter_init_yaw = 1;
    filter_init_wz  = 1;
}

void JY61P_ZeroYaw(void)
{
    /* 只清标志，真正的零点由下一帧在解析线程里用当时的角度记下来。
     * 这样 yaw_offset 始终只有解析线程一个写入者，不会出现别处和解析层
     * 交替写、读到"新 offset + 旧角度"这种半新半旧组合的情况。 */
    yaw_zeroed   = 0;
    yaw_boot_cnt = 0;
}

rt_uint32_t JY61P_GetRxBytes(void)
{
    return rx_bytes;
}

rt_uint32_t JY61P_GetFrameCount(void)
{
    return frame_count;
}

/* 把任意角度归一到 -180~+180。输入范围已知是 -360~+360，两步判断就够了。 */
static float JY61P_Wrap180(float a)
{
    if (a > 180.0f)  a -= 360.0f;
    if (a < -180.0f) a += 360.0f;
    return a;
}

static uint8_t JY61P_CheckSum(uint8_t *buf, uint8_t len)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += buf[i];
    }
    return sum;
}

static void JY61P_ParseFrame(uint8_t *buf)
{
    if (buf[0] != JY61P_HEAD1) return;

    switch (buf[1]) {

    /* ---- 0x53 角度包：Roll / Pitch / Yaw ---- */
    case JY61P_HEAD2_ANGLE:
        if (JY61P_CheckSum(buf, 10) == buf[10]) {
            frame_count++;                  /* [PORT] 新增：校验通过的帧计数 */
            jy61p.raw_roll  = (int16_t)((buf[3] << 8) | buf[2]);
            jy61p.raw_pitch = (int16_t)((buf[5] << 8) | buf[4]);
            jy61p.raw_yaw   = (int16_t)((buf[7] << 8) | buf[6]);
            /* Byte 8-9 = Temperature (TL TH)，不是角速度！不读 wz */

            jy61p.roll  = (float)jy61p.raw_roll  / 32768.0f * 180.0f;
            jy61p.pitch = (float)jy61p.raw_pitch / 32768.0f * 180.0f;

            /* ---- yaw：先锁定零点，再算相对上电朝向的转角 ---- */
            float yaw_now = (float)jy61p.raw_yaw / 32768.0f * 180.0f;

            if (!yaw_zeroed) {
                if (yaw_boot_cnt < JY61P_ZERO_SKIP) {
                    yaw_boot_cnt++;
                    /* 零点还没锁定，这一帧不当"新数据"：屏幕继续停在 Waiting，
                     * 免得先闪一个没校零的角度再跳回 0。 */
                    break;
                }
                yaw_offset = yaw_now;       /* 上电时所处的朝向 = 0 度 */
                yaw_zeroed = 1;
            }

            jy61p.yaw = JY61P_Wrap180((yaw_now - yaw_offset) * JY61P_YAW_SIGN);

#if JY61P_YAW_LPF_EN
            /* 一阶低通滤波 yaw（默认关闭，见文件头说明） */
            if (!filter_init_yaw) {
                filtered_yaw = jy61p.yaw;
                filter_init_yaw = 1;
            } else {
                filtered_yaw = filtered_yaw * (1.0f - KALMAN_K) + jy61p.yaw * KALMAN_K;
            }
#else
            filtered_yaw    = jy61p.yaw;   /* 直接用模块输出，避免相位滞后 */
            filter_init_yaw = 1;
#endif

            new_data_flag = 1;
        }
        break;

    /* ---- 0x52 角速度包：Gx / Gy / Gz ---- */
    case JY61P_HEAD2_GYRO:
        if (JY61P_CheckSum(buf, 10) == buf[10]) {
            frame_count++;                  /* [PORT] 新增：校验通过的帧计数 */
            /* Gz = Z轴角速度 = Byte 6-7（小端） */
            jy61p.raw_wz = (int16_t)((buf[7] << 8) | buf[6]);
            /* 符号跟 yaw 保持一致（左转为负）：模块原始输出是左转为正，这里取反。
             * 两者用同一套约定，控制律的阻尼项才能写成 -KD*wz。 */
            jy61p.wz     = -(float)jy61p.raw_wz / 32768.0f * 2000.0f; /* °/s，左转为负 */

            /* 一阶低通滤波 wz */
            if (!filter_init_wz) {
                filtered_wz = jy61p.wz;
                filter_init_wz = 1;
            } else {
                filtered_wz = filtered_wz * (1.0f - KALMAN_K) + jy61p.wz * KALMAN_K;
            }
            /* 注意：不设 new_data_flag，使用方由 0x53 触发；
               0x52 在后台静默更新 filtered_wz */
        }
        break;

    default:
        break;
    }
}

/* 喂一个字节给帧同步器。
 * 哨兵式帧同步：未锁定帧头时，非 0x55 的字节直接丢弃，
 * 保证 jyp_rx_buf[0] 永远是帧头。校验失败只丢这一帧，
 * 下一个 0x55 立刻重新对齐，不存在需要丢 11 帧的失步窗口。 */
static void JY61P_FeedByte(uint8_t byte)
{
    if (jyp_rx_len == 0 && byte != JY61P_HEAD1) {
        return;
    }

    jyp_rx_buf[jyp_rx_len++] = byte;

    if (jyp_rx_len >= JY61P_FRAME_LEN) {
        jyp_rx_len = 0;                     /* 先复位再解析：解析失败也能立刻重新找帧头 */
        JY61P_ParseFrame(jyp_rx_buf);
    }
}

/* [PORT] 串口收到数据的回调。在中断上下文里被调用，只能做"放行信号量"
 * 这种不能阻塞的事；真正的读取和解析留给下面的线程。 */
static rt_err_t jy901s_rx_indicate(rt_device_t dev, rt_size_t size)
{
    /* [自检] 中断侧计数，见 rx_irq_cnt 的说明：只计数和取 max，不累加 size */
    rx_irq_cnt++;
    if (size > rx_irq_max)
    {
        rx_irq_max = (rt_uint32_t)size;
    }

    rt_sem_release(&jy901s_rx_sem);
    return RT_EOK;
}

/* [PORT] 解析线程：被 rx_indicate 唤醒后把环形缓冲里的字节全部取走喂给帧同步器。
 * 原工程是在主循环里一边刷屏一边查标志，这里拆开：本线程只管收，
 * 显示层读 jy61p 的值即可。 */
static void jy901s_thread(void *parameter)
{
    rt_uint8_t buf[JY901S_RX_CHUNK];
    rt_size_t n, i;

    RT_UNUSED(parameter);

    while (1)
    {
        rt_sem_take(&jy901s_rx_sem, RT_WAITING_FOREVER);
        rx_wake_cnt++;                      /* [自检] 线程确实被唤醒过，而不只是中断到了 */

        while ((n = rt_device_read(jy901s_dev, -1, buf, sizeof(buf))) > 0)
        {
            rx_bytes += n;
            for (i = 0; i < n; i++)
            {
                JY61P_FeedByte(buf[i]);
            }
        }
    }
}

rt_err_t JY61P_Init(void)
{
    struct serial_configure cfg = RT_SERIAL_CONFIG_DEFAULT;

    if (jy901s_dev != RT_NULL)
    {
        return RT_EOK;      /* 重复调用直接返回，别把线程建两遍 */
    }

    jy901s_dev = rt_device_find(JY901S_DEV_NAME);
    if (jy901s_dev == RT_NULL)
    {
        LOG_E("serial device \"%s\" not found", JY901S_DEV_NAME);
        return -RT_ERROR;
    }

    cfg.baud_rate = JY901S_BAUD_RATE;
    cfg.data_bits = DATA_BITS_8;
    cfg.stop_bits = STOP_BITS_1;
    cfg.parity    = PARITY_NONE;
    cfg.bufsz     = 128;    /* 环形缓冲 128 字节：一帧 11 字节，够放约 11 帧 */

    /* 【顺序】先 CONFIG 再 open —— 反过来是错的，而且是静默的错。
     *
     * dev_serial.c 的 RT_DEVICE_CTRL_CONFIG 分支在开头就有这么一句：
     *     if (pconfig->bufsz != serial->config.bufsz && serial->parent.ref_count)
     *         return -RT_EBUSY;      // "can not change buffer size"
     * 设备一旦 open 过，ref_count 就不是 0，而 bufsz 又和默认值
     * RT_SERIAL_RB_BUFSZ（64）不同，于是整个配置【在调用 ops->configure() 之前】
     * 就被丢掉了 —— 波特率压根没落到硬件上，返回值不检查的话一点提示都没有。
     * 具体后果：缓冲其实是 64 字节（不是上面写的 128），日志里那句"够放约 11 帧"
     * 是假的；而且改 JY901S_BAUD_RATE 会静默失效，现象是"收到字节=0、有效帧=0"。
     *
     * 放在 open 之前就没这个问题：ref_count 为 0 时这句检查不拦，配置被记进
     * serial->config；随后 rt_serial_open() 自己会按 config.bufsz 分配环形缓冲，
     * 并调用 ops->configure(serial, &serial->config) 把波特率落下去。
     * 一句话：缓冲大小和波特率都得在 open 之前说，晚了就只能改大小以外的项。 */
    if (rt_device_control(jy901s_dev, RT_DEVICE_CTRL_CONFIG, &cfg) != RT_EOK)
    {
        LOG_E("configure \"%s\" failed (baud %d, bufsz %u)",
              JY901S_DEV_NAME, JY901S_BAUD_RATE, (unsigned)cfg.bufsz);
        jy901s_dev = RT_NULL;
        return -RT_ERROR;
    }

    if (rt_device_open(jy901s_dev, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX) != RT_EOK)
    {
        LOG_E("open \"%s\" failed", JY901S_DEV_NAME);
        jy901s_dev = RT_NULL;
        return -RT_ERROR;
    }

    rt_device_set_rx_indicate(jy901s_dev, jy901s_rx_indicate);

    rt_sem_init(&jy901s_rx_sem, "jy901s", 0, RT_IPC_FLAG_FIFO);

    jy901s_tid = rt_thread_create("jy901s", jy901s_thread, RT_NULL,
                                  JY901S_THREAD_STACK, JY901S_THREAD_PRIO, 10);
    if (jy901s_tid == RT_NULL)
    {
        LOG_E("failed to create the parser thread");
        rt_device_close(jy901s_dev);
        jy901s_dev = RT_NULL;
        return -RT_ERROR;
    }
    rt_thread_startup(jy901s_tid);

    LOG_I("%s open, %d 8N1", JY901S_DEV_NAME, JY901S_BAUD_RATE);
    return RT_EOK;
}

/* 上电自动起来。这样即使不看 OLED，也能在 msh 里用 `jy901s` 直接验数据通路，
 * 显示层和 IMU 互不依赖。 */
static int jy901s_auto_init(void)
{
    return (int)JY61P_Init();
}
INIT_APP_EXPORT(jy901s_auto_init);

/*
 * msh 调试命令。rt_kprintf 用的是 tiny vsnprintf，不支持 %f（会把 %f 原样打出来），
 * 所以定点打印：先四舍五入到 0.01 再拆成整数和小数两段，符号单独给。
 */
static void jy901s_print_fixed(const char *label, float value, int decimals)
{
    const char *sign = (value < 0.0f) ? "-" : " ";
    int scale = (decimals == 1) ? 10 : 100;
    float mag = (value < 0.0f) ? -value : value;    /* 不用 fabsf，省掉 libm 依赖 */
    int total = (int)(mag * (float)scale + 0.5f);

    if (decimals == 1)
        rt_kprintf("%-5s %s%d.%d\n", label, sign, total / scale, total % scale);
    else
        rt_kprintf("%-5s %s%d.%02d\n", label, sign, total / scale, total % scale);
}

/* [自检] 中断/线程计数 + 只读寄存器快照。目的只有一个：把"P3_13 上根本没有
 * 电平跳变"和"跳变有、但配置/通路把它吃掉了"分开。只读，不写任何寄存器；
 * 调用点已经确认 jy901s_dev != RT_NULL，此时 LPUART3 的时钟门必然是开的。
 *
 * 判读：
 *   PORT3->PCR[13] 的 MUX（低 3 位）必须是 3(ALT3)  —— 抓"启动后复用被别的代码抢走"
 *   CTRL 的 RE = 接收使能，RIE = 接收中断使能       —— rt_serial_open 会 SET_INT，RIE 应为 1
 *   STAT 的 RAF = 接收器正在收
 *   BAUD 的 SBR/OSR 非 0                            —— configure() 真的落到了硬件
 *                                                     （历史上 CONFIG 被 -RT_EBUSY 静默丢过） */
static void jy901s_dump_hw(void)
{
    uint32_t pcr  = PORT3->PCR[13];
    uint32_t mux  = (pcr & PORT_PCR_MUX_MASK) >> PORT_PCR_MUX_SHIFT;
    uint32_t baud = LPUART3->BAUD;
    uint32_t stat = LPUART3->STAT;
    uint32_t ctrl = LPUART3->CTRL;

    rt_kprintf("irq    %u 次中断, 单次最多积压 %u 字节\n",
               (unsigned)rx_irq_cnt, (unsigned)rx_irq_max);
    rt_kprintf("thread 唤醒 %u 次, 信号量还剩 %d\n",
               (unsigned)rx_wake_cnt, (int)jy901s_rx_sem.value);
    rt_kprintf("port   PORT3->PCR[13]=0x%08x mux=%u (%s)\n",
               (unsigned)pcr, (unsigned)mux,
               (mux == 3U) ? "ALT3 正确" : "不是 ALT3，复用被抢走了");
    rt_kprintf("lpuart STAT=0x%08x CTRL=0x%08x BAUD=0x%08x\n"
               "       SBR=%u OSR=%u RE=%u RIE=%u RAF=%u\n",
               (unsigned)stat, (unsigned)ctrl, (unsigned)baud,
               (unsigned)(baud & LPUART_BAUD_SBR_MASK),
               (unsigned)((baud & LPUART_BAUD_OSR_MASK) >> LPUART_BAUD_OSR_SHIFT),
               (unsigned)((ctrl & LPUART_CTRL_RE_MASK) ? 1U : 0U),
               (unsigned)((ctrl & LPUART_CTRL_RIE_MASK) ? 1U : 0U),
               (unsigned)((stat & LPUART_STAT_RAF_MASK) ? 1U : 0U));

    if (rx_irq_cnt == 0)
        rt_kprintf("自检   P3_13 上从来没有过电平跳变 → 查模块是否在发、模块 TX 到 D9 这根线、共地\n");
    else if (rx_wake_cnt == 0)
        rt_kprintf("自检   中断到了但解析线程一次都没醒 → 查信号量/线程优先级\n");
    else if (rx_bytes == 0)
        rt_kprintf("自检   中断和线程都动了却没读到字节 → 查串口环形缓冲与驱动\n");
}

static int cmd_jy901s(int argc, char **argv)
{
    if (argc >= 2 && !rt_strcmp(argv[1], "zero"))
    {
        JY61P_ZeroYaw();
        rt_kprintf("jy901s: 当前朝向已重新记为 0 度\n");
        return RT_EOK;
    }

    if (argc >= 2)
    {
        rt_kprintf("用法: jy901s [zero]   zero = 把当前朝向重新记为 0 度\n");
        return -RT_EINVAL;
    }

    if (jy901s_dev == RT_NULL)
    {
        rt_kprintf("jy901s: 串口 %s 没打开（BSP_USING_UART3 没开？）\n", JY901S_DEV_NAME);
        return -RT_ERROR;
    }

    jy901s_print_fixed("Yaw",   jy61p.yaw,   2);
    jy901s_print_fixed("Pitch", jy61p.pitch, 2);
    jy901s_print_fixed("Roll",  jy61p.roll,  2);
    jy901s_print_fixed("Wz",    jy61p.wz,    1);

    rt_kprintf("raw    roll %d pitch %d yaw %d wz %d\n",
               jy61p.raw_roll, jy61p.raw_pitch, jy61p.raw_yaw, jy61p.raw_wz);
    rt_kprintf("rx     %u bytes, %u frames, zero %s\n",
               (unsigned)rx_bytes, (unsigned)frame_count,
               yaw_zeroed ? "done" : "waiting");
    if (rx_bytes == 0)
        rt_kprintf("       一个字节都没收到：查 P3_13(D9) 接线与复用，查模块是否在发\n");
    else if (frame_count == 0)
        rt_kprintf("       收到字节但无有效帧：查波特率(115200)与模块输出协议\n");

    jy901s_dump_hw();

    return RT_EOK;
}
MSH_CMD_EXPORT_ALIAS(cmd_jy901s, jy901s, show JY901S attitude (jy901s zero to re-zero yaw));
