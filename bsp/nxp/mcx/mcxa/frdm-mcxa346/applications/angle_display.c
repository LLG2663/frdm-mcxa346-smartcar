/*
 * SPDX-License-Identifier: MIT
 *
 * OLED 显示，移植自 CCS 工程 113（TI MSPM0G3507）到 FRDM-MCXA346 上的 RT-Thread。
 *
 * 本文件现在管【两块屏】，共用同一个线程、同一份显存：
 *
 *   车控屏（默认，来自 113/main.c）      角度屏（来自 113 的角度显示部分）
 *   --------------------------------    --------------------------------
 *   Yaw   <值>                          Yaw   <值>
 *   St    <状态>                        Pitch <值>
 *   Rx    <hex> <字符>                  Roll  <值>
 *   send 4F / Ok <计数>                 Wz    <值>
 *
 * 用 `oled screen car` / `oled screen angle` 切换（默认车控屏）。
 * 一个线程画两块屏、用同一个开关切换，而不是两个线程各画各的：
 * OLED 是独占资源，两个写入者迟早会在 OLED_Refresh() 的 1024 次 I2C 事务
 * 中间被打断，画出半新半旧的混合屏。
 *
 * 显示布局、定宽右对齐补空格这些是纯算法，逐行照搬：
 *   OLED_ShowField() / ShowLine() / DrawAngleScreen() / DrawNoDataScreen()
 *   ShowLine1() / ShowLine2() / ShowLine3() / ShowBootScreen()
 * 连注释都保留原样，包括那条"转向相反改 JY61P_YAW_SIGN，不要在这里加负号"。
 *
 * 数据通路（源工程在 main() 的 while(1) 里，这里换成独立线程）：
 *   jy901s.c 解析 0x53/0x52 帧 → jy61p / new_data_flag
 *   bt_car.c 收字节 + 状态机   → bt_car_state_name() / last_byte / cmd_count
 *   本线程 → 取快照 → 内容变了才画到显存 → OLED_Refresh()
 *
 * 改动都标了 [PORT]：
 *   [PORT] main() 的 while(1) → 独立线程（和 led 线程一样不占 tshell）
 *   [PORT] fabsf()  → 手写取绝对值（LFLAGS 里没有 -lm，别去碰 libm）
 *   [PORT] oled 调试命令 + 两块屏的切换
 *   [PORT] 重画策略：见下面「什么时候重画」
 *
 * yaw 已在 jy901s.c 里做过校零和符号翻转：上电第一次朝向 = 0.00，
 * 范围 -180~+180，左转负、右转正。这里直接显示即可，不用再处理。
 * （若实测转向相反，改 jy901s.c 的 JY61P_YAW_SIGN，不要在这里加负号。）
 *
 * 显示布局：16 号字，4 行；标签占 0~47 列，数值字段固定在 48~127 列。
 * 数值字段宽度固定、左侧补空格，这样数字变短时不会留下上一次的残字。
 *
 * ------------------------------------------------------- 什么时候重画 ----
 * 源工程（113/main.c）有一条硬约束：OLED_Refresh() 阻塞约 300ms
 * （1024 次独立 I2C 事务），所以【车在动的期间一次 OLED 都不刷】，
 * 只在待机和上锁状态刷屏。
 *
 * 这条约束在 RT-Thread 下不成立：显示在自己的线程里，刷屏不挡遥控线程
 * （本线程优先级 26，低于遥控的 23 和 IMU 的 22）。所以这里不照搬
 * 「只在没动的时候刷」，而是【屏幕内容变了就刷】——每块屏各自比一下快照，
 * 内容没变就一个字节都不发。
 *
 * 顺带修掉了源工程一个真实的表现问题：源工程「车动期间零刷新」意味着
 * FORWARD / SPIN-L / SPIN-R 这三个状态名【从来没有在屏幕上出现过】
 * （刷屏写在读命令之前，而车一动就不再刷屏了）。这里状态一变就刷，
 * 按 W 屏幕立刻显示 FORWARD。
 *
 * 另外还留了一条兜底：每 KEEPALIVE_MS 强制重画一次。快照比较理论上不会
 * 漏，但 I2C 上偶发一次写失败就会让屏幕永远停在旧内容上；一条 1 秒的
 * 心跳能把这种「显示和状态不一致」在上限 1 秒内自愈。
 */

#include <rtdevice.h>

#include "oled.h"
#include "jy901s.h"
#include "bt_car.h"

#define DBG_TAG    "angle"
#define DBG_LVL    DBG_INFO
#include <rtdbg.h>

#define FONT_SIZE       16      /* 字号：16 号字行高 16 像素 */
#define ROW_H           16
#define VALUE_X         48      /* 数值字段起点（0~47 留给标签，最长 6 字符） */
#define VALUE_W         10      /* 数值字段宽度（字符），48 + 10*8 = 128 正好到屏幕右边缘 */

#define LOOP_MS         10      /* 主循环节拍 */
#define NO_DATA_N       100     /* 连续 100 轮（约 1s）收不到新帧 → 报无数据 */
#define KEEPALIVE_MS    1000    /* 内容没变也强制重画一次的兜底周期 */

#define DISPLAY_THREAD_PRIO  26     /* 低于 jy901s(22) 和 led(25)：屏幕晚一帧画没有代价 */
#define DISPLAY_THREAD_STACK 1536

/* `oled` 调试命令用：置位后自动重画暂停，屏幕交给命令手动控制。 */
static volatile rt_bool_t display_paused = RT_FALSE;

/* 当前显示哪块屏。默认车控屏——插上无线串口模块就能用，不依赖 IMU 在线。 */
#define OLED_SCREEN_CAR     0
#define OLED_SCREEN_ANGLE   1

static volatile rt_uint8_t g_screen = OLED_SCREEN_CAR;

/* ----------------------------------------------------------------------------
 *  在固定宽度的字段里右对齐显示一个带符号浮点数，左侧补空格。
 *
 *  value — 待显示的值
 *  field — 字段宽度（字符数），每次都写满 field 个字符，旧内容一定被覆盖
 *  dec   — 小数位数
 * -------------------------------------------------------------------------- */
static void OLED_ShowField(u8 x, u8 y, float value, u8 field, u8 dec, u8 size)
{
    char     txt[16];
    char     digits[12];
    u8       n = 0, m = 0, i, pad;
    uint32_t div = 1;
    uint32_t total;             /* 放大 10^dec 并四舍五入后的绝对值 */
    uint32_t ip, fp;
    uint8_t  neg;

    if (field > sizeof(txt)) field = sizeof(txt);
    if (dec > 6) dec = 6;
    for (i = 0; i < dec; i++) div *= 10;

    /* 先四舍五入再拆分：否则 9.97 会显示成 9.9，而 9.99 会显示成 9.10 */
    /* [PORT] 这里原是 fabsf(value)：本 BSP 的 LFLAGS 没带 -lm，
     * 手写取绝对值省掉对 libm 的依赖，行为完全一样。 */
    float mag = (value < 0.0f) ? -value : value;
    total = (uint32_t)(mag * (float)div + 0.5f);
    ip    = total / div;
    fp    = total % div;

    /* 四舍五入后为零就不打负号，避免零点附近来回闪 "-0.00" */
    neg = (value < 0.0f) && (total != 0);

    if (neg && n < field) txt[n++] = '-';

    if (ip == 0) digits[m++] = '0';
    while (ip > 0 && m < sizeof(digits)) {
        digits[m++] = (char)('0' + (ip % 10));
        ip /= 10;
    }
    while (m > 0 && n < field) txt[n++] = digits[--m];      /* 反转成高位在前 */

    if (dec > 0 && n < field) {
        txt[n++] = '.';
        for (i = 0; i < dec && n < field; i++) {
            div /= 10;
            txt[n++] = (char)('0' + (u8)((fp / div) % 10));
        }
    }

    pad = (u8)(field - n);      /* n <= field，补空格把字段填满 */
    for (i = 0; i < pad; i++) OLED_ShowChar((u8)(x + i * (size / 2)), y, ' ', size);
    for (i = 0; i < n; i++)   OLED_ShowChar((u8)(x + (pad + i) * (size / 2)), y, (u8)txt[i], size);
}

static void ShowLine(u8 row, const char *label, float value, u8 dec)
{
    OLED_ShowString(0, (u8)(row * ROW_H), (u8 *)label, FONT_SIZE);
    OLED_ShowField(VALUE_X, (u8)(row * ROW_H), value, VALUE_W, dec, FONT_SIZE);
}

/* ----------------------------------------------------------------------------
 *  整屏重画 4 行角度。只在收到新帧时调用，所以屏幕刷新率 ≈ 模块回传率。
 * -------------------------------------------------------------------------- */
static void DrawAngleScreen(float yaw, float pitch, float roll, float wz)
{
    ShowLine(0, "Yaw",   yaw,   2);//上电后后校准后的0度，范围-180到+180，左正右负
    ShowLine(1, "Pitch", pitch, 2);
    ShowLine(2, "Roll",  roll,  2);
    ShowLine(3, "Wz",    wz,    1);     /* °/s，角速度 */
    OLED_Refresh();
}

static void DrawNoDataScreen(void)
{
    OLED_Clear();
    OLED_ShowString(0, 0,  (u8 *)"No JY901S Data!", FONT_SIZE);
    OLED_ShowString(0, 16, (u8 *)"RX 115200 8N1", FONT_SIZE);
    OLED_ShowString(0, 32, (u8 *)"check TX->RX", FONT_SIZE);
    OLED_Refresh();
}

/* ============================================================================
 *  车控屏 —— 来自 113/main.c 的 ShowLine0/1/2/3 + DrawIdleScreen
 * ============================================================================
 *  4 行固定宽度排版。每行都写满固定宽度，避免旧内容残留。
 *
 *   第 0 行  Yaw   <值>       校准后的偏航角，-180~180，左转为负
 *   第 1 行  St    <状态>     当前运动状态 / DISARM
 *   第 2 行  Rx    <hex> <字符>  最后收到的原始字节 —— 波特率自检就看这行
 *   第 3 行  send 4F         上锁时：解锁指令（就是你现在该做的事）
 *            Ok    <计数>     解锁后：已识别的命令字节数
 * ========================================================================== */
static u8 HexDigit(u8 v)
{
    return (v < 10) ? (u8)('0' + v) : (u8)('A' + (v - 10));
}

/* 把 txt 补齐到 width 个字符并加上结尾。定宽的目的就是"旧内容一定被覆盖" */
static void PadTo(char *txt, u8 *n, u8 width)
{
    while (*n < width) txt[(*n)++] = ' ';
    txt[width] = '\0';
}

static void ShowCarLine0(float yaw, uint8_t imu_lost)
{
    OLED_ShowString(0, 0, (u8 *)(imu_lost ? "Yaw!" : "Yaw "), FONT_SIZE);
    OLED_ShowField(VALUE_X, 0, yaw, VALUE_W, 2, FONT_SIZE);
}

static void ShowCarLine1(void)
{
    char        txt[12];
    u8          n = 0;
    const char *nm = bt_car_state_name();

    txt[n++] = 'S';  txt[n++] = 't';
    txt[n++] = ' ';  txt[n++] = ' ';  txt[n++] = ' ';
    while (*nm && n < 11) txt[n++] = *nm++;
    PadTo(txt, &n, 11);
    OLED_ShowString(0, 1 * ROW_H, (u8 *)txt, FONT_SIZE);
}

/* 最后收到的原始字节。b < 0 表示还没收到过任何东西。 */
static void ShowCarLine2(int16_t b)
{
    char txt[12];
    u8   n = 0;

    txt[n++] = 'R';  txt[n++] = 'x';
    txt[n++] = ' ';  txt[n++] = ' ';
    if (b < 0) {
        txt[n++] = '-';  txt[n++] = '-';
        txt[n++] = ' ';  txt[n++] = '-';
    } else {
        txt[n++] = (char)HexDigit((u8)((uint8_t)b >> 4));
        txt[n++] = (char)HexDigit((u8)((uint8_t)b & 0x0F));
        txt[n++] = ' ';
        /* 不可打印的字节画成点，占位但看得出不是可见字符 */
        txt[n++] = ((b >= 0x20) && (b <= 0x7E)) ? (char)b : '.';
    }
    PadTo(txt, &n, 11);
    OLED_ShowString(0, 2 * ROW_H, (u8 *)txt, FONT_SIZE);
}

static void ShowCarLine3(void)
{
    char     txt[12];
    char     dig[10];
    u8       n = 0, m = 0;
    uint32_t v;

    /* 上锁时这一行直接写"怎么解锁"。
     * 之前这里一直只显示 Ok 计数，结果"Ok 在涨但车不动"看起来像程序坏了 ——
     * 其实只是还没解锁。计数在解锁后再显示，那时候它才有意义（当心跳看）。 */
    if (!bt_car_is_armed()) {
        const char *hint = "send 4F";
        while (*hint && n < 11) txt[n++] = *hint++;
        PadTo(txt, &n, 11);
        OLED_ShowString(0, 3 * ROW_H, (u8 *)txt, FONT_SIZE);
        return;
    }

    v = bt_car_cmd_count();
    txt[n++] = 'O';  txt[n++] = 'k';
    txt[n++] = ' ';  txt[n++] = ' ';

    if (v == 0) dig[m++] = '0';
    while (v > 0 && m < sizeof(dig)) {
        dig[m++] = (char)('0' + (v % 10));
        v /= 10;
    }
    while (m > 0 && n < 11) txt[n++] = dig[--m];
    PadTo(txt, &n, 11);
    OLED_ShowString(0, 3 * ROW_H, (u8 *)txt, FONT_SIZE);
}

/* [PORT] 源工程的 DrawIdleScreen()。名字里的 Idle 不再适用：源工程只在
 * 车没动的时候调它，这里内容一变就调（见文件头「什么时候重画」）。 */
static void DrawCarScreen(float yaw, int16_t last_rx, uint8_t imu_lost)
{
    ShowCarLine0(yaw, imu_lost);
    ShowCarLine1();
    ShowCarLine2(last_rx);
    ShowCarLine3();
    OLED_Refresh();
}

/* 开机提示屏，逐行照搬源工程 main() 的 ShowBootScreen()。
 * 每行最多 16 个字符（16 号字宽 8 像素，128 像素正好 16 个），
 * 超了会折行到下一行把下面的内容顶掉。
 *
 * 在源工程里它只是"上电先给你一张命令表"，在本 BSP 里它还是默认屏
 * （车控屏）的操作说明，所以取代了原来那张 "JY901S Angle / Waiting..."。 */
static void DrawBootScreen(void)
{
    OLED_ShowString(0, 0,  (u8 *)"UART HEX 115200",   FONT_SIZE);
    OLED_ShowString(0, 16, (u8 *)"W57 S53 A41 D44", FONT_SIZE);
    OLED_ShowString(0, 32, (u8 *)"20 stop  4F=arm",  FONT_SIZE);
    OLED_ShowString(0, 48, (u8 *)"H48=disarm", FONT_SIZE);
    OLED_Refresh();
}

/* ============================================================================
 *  快照比较：内容没变就一个 I2C 字节都不发
 * ========================================================================== */
typedef struct {
    float       yaw;
    const char *state;
    int16_t     last_rx;
    uint32_t    cmd_cnt;
    uint8_t     armed;
    uint8_t     imu_lost;
} car_snap_t;

static car_snap_t car_snap_now(uint8_t imu_lost)
{
    car_snap_t s;

    s.yaw      = jy61p.yaw;
    s.state    = bt_car_state_name();
    s.last_rx  = bt_car_last_byte();
    s.cmd_cnt  = bt_car_cmd_count();
    s.armed    = (uint8_t)bt_car_is_armed();
    s.imu_lost = imu_lost;

    return s;
}

/* yaw 用定点比较：浮点 != 对"同一个显示结果"太敏感，
 * 0.0001 度的差别也会触发一次 1024 字节的刷屏，而屏幕上看不出来。 */
static int car_snap_diff(const car_snap_t *a, const car_snap_t *b)
{
    if (a->state    != b->state)    return 1;
    if (a->last_rx  != b->last_rx)  return 1;
    if (a->cmd_cnt  != b->cmd_cnt)  return 1;
    if (a->armed    != b->armed)    return 1;
    if (a->imu_lost != b->imu_lost) return 1;

    return (rt_int32_t)(a->yaw * 100.0f) != (rt_int32_t)(b->yaw * 100.0f);
}

/* [PORT] 源工程 main() 的 while(1) 主体，逐行搬过来放进线程。
 * 两块屏共用一个线程，按 g_screen 分派（见文件头）。 */
static void angle_display_thread(void *parameter)
{
    /* 最近一帧的数值。roll/pitch/yaw 由同一帧 0x53 一起写入，wz 由 0x52 更新，
     * 每个字段都是 32 位、地址对齐的单次读写，不需要额外加临界区。 */
    float      yaw = 0.0f, pitch = 0.0f, roll = 0.0f, wz = 0.0f;
    uint32_t   no_data_cnt = 0;     /* 连续没收到新帧的轮数 */
    uint8_t    link_ok = 1;         /* 0 = 正在显示"无数据"提示（角度屏专用） */
    car_snap_t snap;                /* 车控屏上一次画出去的内容 */
    uint32_t   keepalive = 0;       /* 车控屏距上次重画的毫秒数 */
    uint8_t    had_snap = 0;        /* 车控屏还没画过一次 */
    uint8_t    prev_screen = 0xFF;  /* 0xFF = 还没画过任何一块屏 */
    uint8_t    new_frame;           /* 这一轮收到了新的一帧 */
    uint8_t    force;               /* 这一轮必须重画 */

    RT_UNUSED(parameter);

    while (1) {
        if (display_paused) {
            rt_thread_mdelay(LOOP_MS);
            continue;
        }

        new_frame = 0;
        if (JY61P_HasNewData()) {
            /* 取一次快照再显示：解析在 jy901s 线程里做，这里只负责画 */
            yaw   = jy61p.yaw;
            pitch = jy61p.pitch;
            roll  = jy61p.roll;
            wz    = jy61p.wz;
            JY61P_ClearNewData();

            no_data_cnt = 0;
            new_frame = 1;
        } else if (no_data_cnt < NO_DATA_N) {
            no_data_cnt++;
        }

        /* IMU 丢帧判断两块屏都要用："IMU 丢了"这件事和显示哪块屏无关 */
        uint8_t imu_lost = (uint8_t)(no_data_cnt >= NO_DATA_N);

        /* 切屏：整屏清一次，两边都强制重画，清掉上一块屏的残字。
         * 不清的话，从车控屏的 "Ok 12" 切到角度屏，短行会在右边留半截旧字符。 */
        force = 0;
        if (g_screen != prev_screen) {
            prev_screen = g_screen;
            OLED_Clear();
            link_ok = 1;
            had_snap = 0;
            force = 1;
        }

        if (g_screen == OLED_SCREEN_CAR) {
            car_snap_t now = car_snap_now(imu_lost);

            keepalive += LOOP_MS;

            if (!had_snap || car_snap_diff(&now, &snap) ||
                keepalive >= KEEPALIVE_MS) {
                snap = now;
                had_snap = 1;
                keepalive = 0;

                /* 用 now.yaw 而不是上面那个局部 yaw：快照和画出去的内容
                 * 必须是同一次读到的值，否则 diff 比的是 A、画的是 B。
                 * 局部 yaw 是"最后一次收到新帧时"的值，两者一般相等，
                 * 只在 IMU 恰好在这一轮更新时差一帧。 */
                DrawCarScreen(now.yaw, now.last_rx, imu_lost);
            }
        } else {
            /* 角度屏：源工程就是"收到新帧才画"，屏幕刷新率 ≈ 模块回传率，
             * 这里保持原样——角度屏没有别的变化源，也没有要限流的理由。 */
            if (imu_lost) {
                if (link_ok) {
                    /* 约 1s 一帧都没收到：写在屏上，别让人以为是程序卡死。
                     * 常见原因：模块还在出厂 9600bps（本工程串口是 115200）、
                     * 模块 TX 没接到 P3_13(D9)、模块没在输出 0x53 角度包。
                     * 具体是哪种，在 msh 里敲 `jy901s` 看收字节数和有效帧数。 */
                    link_ok = 0;
                    DrawNoDataScreen();
                }
            } else {
                if (!link_ok) {     /* 恢复：先清一次，清掉提示残字 */
                    link_ok = 1;
                    OLED_Clear();
                    force = 1;
                }
                if (force || new_frame) {
                    DrawAngleScreen(yaw, pitch, roll, wz);
                }
            }
        }

        rt_thread_mdelay(LOOP_MS);
    }
}

/* [PORT] 上电初始化 + 起线程。源工程这段在 main() 开头。 */
static int angle_display_init(void)
{
    rt_thread_t tid;

    if (OLED_Init() != RT_EOK)
    {
        /* 屏幕没接或 i2c2 没使能。只报错返回，不阻塞其它功能：
         * IMU 和 msh 命令照样能用，`jy901s` 就能验证数据通路。 */
        LOG_E("OLED init failed, display disabled");
        return -RT_ERROR;
    }

    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    OLED_Clear();
    DrawBootScreen();

    tid = rt_thread_create("angle", angle_display_thread, RT_NULL,
                           DISPLAY_THREAD_STACK, DISPLAY_THREAD_PRIO, 10);
    if (tid == RT_NULL)
    {
        LOG_E("failed to create the display thread");
        return -RT_ERROR;
    }
    rt_thread_startup(tid);

    return RT_EOK;
}
INIT_APP_EXPORT(angle_display_init);

/*
 * 调试命令：让屏幕能在 IMU 还没接的情况下单独验证阶段 1 的 I2C 通路。
 * 用了就暂停自动重画，否则下一帧角度会立刻把测试图案盖掉。
 */
static void oled_draw_test_pattern(void)
{
    u8 i;

    OLED_Clear();

    /* 边框：直接画点，不用 OLED_DrawLine —— 原版那条函数少画最后一个像素，
     * 测试图案要的是"四边都严丝合缝"，不跟它较劲。 */
    for (i = 0; i < 128; i++) {
        OLED_DrawPoint(i, 0);
        OLED_DrawPoint(i, 63);
    }
    for (i = 0; i < 64; i++) {
        OLED_DrawPoint(0, i);
        OLED_DrawPoint(127, i);
    }

    OLED_ShowString(16, 8,  (u8 *)"OLED OK", FONT_SIZE);
    OLED_ShowString(8,  24, (u8 *)"i2c2 0x3C", FONT_SIZE);

    /* 底部两页交替竖条：所有列都点一遍，能看出坏列/坏页 */
    for (i = 0; i < 128; i += 4) {
        OLED_DrawPoint(i, 48);
        OLED_DrawPoint(i + 1, 56);
    }

    OLED_Refresh();
}

static int cmd_oled(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("usage: oled <screen car|angle | test | clear | resume>\n");
        rt_kprintf("  screen car    显示车控屏（Yaw/状态/Rx/计数）—— 默认\n");
        rt_kprintf("  screen angle  显示角度屏（Yaw/Pitch/Roll/Wz）\n");
        rt_kprintf("  test          画测试图案并暂停自动刷新（验证 I2C 通路）\n");
        rt_kprintf("  clear         清屏并暂停自动刷新\n");
        rt_kprintf("  resume        恢复自动刷新\n");
        rt_kprintf("  screen = %s, paused = %s\n",
                   (g_screen == OLED_SCREEN_CAR) ? "car" : "angle",
                   display_paused ? "yes" : "no");
        return RT_EOK;
    }

    if (!rt_strcmp(argv[1], "screen"))
    {
        if (argc < 3)
        {
            rt_kprintf("oled: usage: oled screen <car|angle>\n");
            return -RT_EINVAL;
        }

        if (!rt_strcmp(argv[2], "car"))
        {
            g_screen = OLED_SCREEN_CAR;
        }
        else if (!rt_strcmp(argv[2], "angle"))
        {
            g_screen = OLED_SCREEN_ANGLE;
        }
        else
        {
            rt_kprintf("oled: bad screen '%s' (car|angle)\n", argv[2]);
            return -RT_EINVAL;
        }

        /* 只改标志位，不在命令里画：屏幕归显示线程一个人写，
         * 在这里插一次 OLED_Refresh() 就又多了一个写入者。线程下一轮
         * （10ms 内）会看到 g_screen 变了、清屏并重画。 */
        rt_kprintf("oled: screen = %s\n", (g_screen == OLED_SCREEN_CAR) ? "car" : "angle");
        return RT_EOK;
    }

    if (!rt_strcmp(argv[1], "test"))
    {
        display_paused = RT_TRUE;
        oled_draw_test_pattern();
        rt_kprintf("oled: 已画测试图案，自动刷新暂停（oled resume 恢复）\n");
        return RT_EOK;
    }

    if (!rt_strcmp(argv[1], "clear"))
    {
        display_paused = RT_TRUE;
        OLED_Clear();
        rt_kprintf("oled: 已清屏，自动刷新暂停（oled resume 恢复）\n");
        return RT_EOK;
    }

    if (!rt_strcmp(argv[1], "resume"))
    {
        display_paused = RT_FALSE;
        rt_kprintf("oled: 恢复自动刷新\n");
        return RT_EOK;
    }

    rt_kprintf("usage: oled <screen car|angle | test | clear | resume>\n");
    return -RT_EINVAL;
}
MSH_CMD_EXPORT_ALIAS(cmd_oled, oled, SSD1306 display (screen|test|clear|resume));
