/*
 * SPDX-License-Identifier: MIT
 *
 * follow.c —— 航点跟随的状态机，接口和线路协议见 follow.h。
 *
 * ------------------------------------------------------------ 怎么走的 ----
 *  每个航段两步：
 *      TURN   原地转到这条边的方位（不往前走，先把方向对好）
 *      DRIVE  沿这个方位走这条边的长度，外环负责别走歪
 *  分段停走（默认）时，每两步之间插一次 SETTLE（刹停 + 稳定一段时间）。
 *
 *  【为什么位置是"推"出来的】车上没有位置传感器：编码器只有 A 相、只给幅值，
 *  yaw 是唯一的绝对量（见 jy901s.h）。所以「走到哪了」= yaw 对方向 +
 *  编码器计数对距离，两者都会漂。每到一个航点停下来重新对一次方位，
 *  误差就不跨航段累积 —— 这就是分段停走存在的理由，不是为了稳，是为了准。
 *
 *  【为什么航向修正不写在这里】car.c 的 Car_StraightTick() 已经是调好的
 *  航向外环（20ms 一拍、积分抗饱和、90 度重锁保护）。跟随只是把它的"基准"
 *  换成航段的方位，走 Car_DriveHeading()。原地转（TURN）它管不了 ——
 *  那个外环第 1 步就把两轮不同速的情况排除了 —— 所以只有 TURN 是这里自己算的。
 */
#include <rtdevice.h>
#include <stdarg.h>

#include "follow.h"
#include "motor.h"
#include "car.h"
#include "jy901s.h"
#include "bt_car.h"
#include "speed_report.h"

#define DBG_TAG    "follow"
#define DBG_LVL    DBG_INFO
#include <rtdbg.h>

/* 坐标范围（mm）。不是洁癖：航段长度要在 32 位里做"平方和再开方"，
 * 卡在 ±20m 之内，(dx*dx + dy*dy) 才不会溢出。 */
#define FOLLOW_COORD_MAX        20000

typedef struct {
    rt_int32_t x_mm;
    rt_int32_t y_mm;
    rt_int16_t h_ddeg;          /* 该点指向下一点的方位，0.1 度，逆时针为正 */
} follow_wp_t;

typedef enum {
    FOLLOW_IDLE = 0,            /* 没装航点 */
    FOLLOW_LOAD,                /* 正在收航点；end 之后表示"装好了待命" */
    FOLLOW_TURN,                /* 原地转到本段方位 */
    FOLLOW_SETTLE,              /* 刹停 + 稳定 */
    FOLLOW_DRIVE,               /* 沿方位走完本段 */
    FOLLOW_DONE,                /* 整条路走完 */
    FOLLOW_ABORT                /* 被停/被故障打断 */
} follow_state_t;

/* ============================================================================
 *  状态
 * ========================================================================== */

/* ---- 航点表：收命令时写，跑的时候只读 ---- */
static follow_wp_t  g_wp[FOLLOW_MAX_POINTS];
static rt_uint32_t  g_leg_mm[FOLLOW_MAX_POINTS];    /* 段 i：wp[i] → wp[i+1] */
static rt_uint16_t  g_wp_count;
static rt_uint16_t  g_wp_expect;                    /* begin 声明的点数 */

/* ---- 运行期参数：`follow <名字> <值>` 改，都是台架上要试的 ---- */
static volatile float  g_sign    = FOLLOW_SIGN_DEF;
static volatile float  g_kp_turn = FOLLOW_KP_TURN;
static volatile float  g_lead_s  = FOLLOW_WZ_LEAD_S;
static volatile float  g_tol_deg = FOLLOW_TOL_DEG;
static rt_int16_t      g_spd      = FOLLOW_SPEED_DEF;
static rt_int16_t      g_vmin     = FOLLOW_VMIN_TURN;
static rt_int16_t      g_vmax     = FOLLOW_VMAX_TURN;
static rt_uint32_t     g_margin_mm = FOLLOW_STOP_MARGIN_MM;
static volatile float  g_blend_deg = FOLLOW_BLEND_DEG;
static rt_uint32_t     g_turnin_mm = FOLLOW_TURNIN_MM;
static rt_bool_t       g_cont      = RT_FALSE;

/* ---- 一次跟随的运行期状态 ---- */
static volatile follow_state_t g_state = FOLLOW_IDLE;
static rt_uint8_t   g_ab = FOLLOW_AB_NONE;
static float        g_yaw0;             /* start 那一拍的 yaw */
static rt_uint16_t  g_seg_count;        /* 航段数 = 点数 - 1 */
static rt_uint16_t  g_leg;              /* 当前航段，0 起 */
static float        g_turn_target;      /* 原地转的目标 yaw */
static rt_tick_t    g_turn_hold;        /* 这个时刻之前按住刹车（见 follow.h 的 TURN_HOLD） */
static rt_int32_t   g_leg_l0, g_leg_r0; /* 本段起点的累计计数 */
static rt_int32_t   g_d_mm;             /* 本段已走毫米 */
static rt_bool_t    g_leg_blend;        /* 本段结束时是缓弯（不停车）*/
static float        g_yaw_target_prev;  /* 上一拍下发的目标方位，算前馈角速度用 */
static rt_tick_t    g_yaw_target_tick;  /* 上一拍的时刻 */
static rt_tick_t    g_leg_tick;
static rt_uint32_t  g_leg_to_ms;
static rt_tick_t    g_settle_tick;
static rt_bool_t    g_settle_to_turn;   /* SETTLE 之后是去 TURN 还是去 DRIVE */
static rt_tick_t    g_stall_tick;
static rt_int32_t   g_stall_mm;
static rt_tick_t    g_err_tick;         /* 偏差开始超限的时刻，0 = 没在超 */
static rt_uint32_t  g_imu_frames;
static rt_tick_t    g_imu_tick;
static float        g_tele_err;         /* 最近一拍的航向误差，给遥测用 */

/* ---- 遥测 ---- */
static rt_tick_t          g_tele_tick;
static volatile rt_bool_t g_tele_dirty = RT_TRUE;

/* ---- 打断：别的线程只置这两个，真正的刹车在下一拍 Follow_Tick() 里做 ---- */
static volatile rt_bool_t  g_ab_pending;
static volatile rt_uint8_t g_ab_code;

/* ---- 起跑时外环本来是关的（启动前替用户打开，收手时还原） ---- */
static rt_bool_t g_head_was_off;

/* ============================================================================
 *  小工具
 * ========================================================================== */

/* 角度归一。入参保证在 ±360 以内（都是两个 ±180 的量相减），所以一次就够。 */
static float follow_wrap180(float a)
{
    if (a >  180.0f) a -= 360.0f;
    if (a < -180.0f) a += 360.0f;
    return a;
}

/* 度 → 0.1 度（线路上的角度单位）。tiny vsnprintf 不支持 %f。 */
static int follow_d10(float v)
{
    return (int)(v * 10.0f + ((v < 0.0f) ? -0.5f : 0.5f));
}

/* 整数开方（牛顿法）。不用 sqrtf()：本 BSP 没有 -lm，
 * 而平方和开方这里的量级很小，整数解完全够用。 */
static rt_uint32_t follow_isqrt(rt_uint32_t n)
{
    rt_uint32_t x, y;

    if (n == 0u) {
        return 0u;
    }
    x = n;
    y = (x + 1u) >> 1;
    while (y < x) {
        x = y;
        y = (x + n / x) >> 1;
    }
    return x;
}

/* 解析一个可带符号整数。理由同 bt_car.c 的 bt_parse_uint()：klibc 里没有
 * atoi/strtol，溢出也要挡（绕回去会变成一个看似合法的错误数）。 */
static rt_err_t follow_parse_int(const char *s, rt_int32_t *out)
{
    rt_int32_t v = 0;
    int neg = 0;
    int seen = 0;

    if (*s == '+' || *s == '-') {
        neg = (*s == '-');
        s++;
    }

    while (*s >= '0' && *s <= '9') {
        if (v > 214748364) {            /* (2^31-1-9)/10，保证 v*10+9 不绕回 */
            return -RT_EINVAL;
        }
        v = v * 10 + (*s - '0');
        seen = 1;
        s++;
    }

    if (!seen || *s != '\0') {
        return -RT_EINVAL;
    }

    *out = neg ? -v : v;
    return RT_EOK;
}

/* 解析一个可带符号、可带小数的数（增益、秒）。不用 atof()：同 car_parse_float()。 */
static rt_err_t follow_parse_float(const char *s, float *out)
{
    float value = 0.0f;
    float scale = 0.1f;
    int   neg = 0;
    int   seen = 0;
    int   fd = 0;

    if (*s == '+' || *s == '-') {
        neg = (*s == '-');
        s++;
    }

    while (*s >= '0' && *s <= '9') {
        value = value * 10.0f + (float)(*s - '0');
        seen = 1;
        s++;
    }

    if (*s == '.') {
        s++;
        while (*s >= '0' && *s <= '9') {
            if (fd < 3) {
                value += (float)(*s - '0') * scale;
                scale *= 0.1f;
                fd++;
            }
            seen = 1;
            s++;
        }
    }

    if (!seen || *s != '\0') {
        return -RT_EINVAL;
    }

    *out = neg ? -value : value;
    return RT_EOK;
}

/* 一句话同时说给两边听：控制台（rt_kprintf → uart2）和无线串口链路
 * （speed_report_sendf → uart5）。
 *
 * 【为什么错误原因必须上链路】`#follow ...` 的通用回包只有一行 "ok/err follow"
 * （speed_report.c 的 spd_run_line），谁被拒了、为什么被拒，原来的 rt_kprintf
 * 只有插着 USB 的控制台看得见 —— 而调这个功能的时候车一定是电池供电、插不了
 * USB。于是从链路发过去的错误永远是一句没有下文的 "err"，没法查。 */
static void follow_say(const char *fmt, ...)
{
    char    buf[SPD_LINE_MAX * 2];
    va_list args;
    int     n;

    va_start(args, fmt);
    n = rt_vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (n <= 0) {
        return;
    }

    rt_kprintf("%s", buf);
    speed_report_sendf("%s", buf);
}

static const char *follow_state_name(follow_state_t s)
{
    switch (s) {
    case FOLLOW_IDLE:   return "idle";
    case FOLLOW_LOAD:   return "load";
    case FOLLOW_TURN:   return "turn";
    case FOLLOW_SETTLE: return "settle";
    case FOLLOW_DRIVE:  return "drive";
    case FOLLOW_DONE:   return "done";
    default:            return "abort";
    }
}

/* 本段的目标 yaw（jy901s 刻度）。
 *
 * 【用相对角差，不用绝对角】起点方位 h0 对应的车头朝向就是 start 那一拍的
 * yaw0 —— 这样上位机有没有把第一段旋转到 +X 都无所谓，两者自动对齐。
 * 【先在 0.1 度整数上归一】直接在度上归一会遇到「-179 度 到 +179 度」这种
 * 跨界，算出来 358 度，车会朝反方向绕一大圈。 */
static float follow_leg_yaw(rt_uint16_t leg)
{
    rt_int32_t d = (rt_int32_t)g_wp[leg].h_ddeg - (rt_int32_t)g_wp[0].h_ddeg;

    while (d >  1800) d -= 3600;
    while (d < -1800) d += 3600;

    return follow_wrap180(g_yaw0 + g_sign * (float)d / 10.0f);
}

/* 从 leg 到 leg+1 的转角够不够小（够小就不停车，直接接着走）。
 * 【为什么看方位差而不是坐标】和 follow_leg_yaw() 同一个理由：位置是推出来的，
 * 两段的方位都相对 h0，差多少就是要转多少。 */
static rt_bool_t follow_blend_ok(rt_uint16_t leg)
{
    float d;

    if (g_blend_deg <= 0.0f || (rt_uint16_t)(leg + 1u) >= g_seg_count) {
        return RT_FALSE;            /* 最后一段没有"下一段"，照旧刹车 */
    }

    d = follow_wrap180(follow_leg_yaw((rt_uint16_t)(leg + 1u)) - follow_leg_yaw(leg));
    if (d < 0.0f) {
        d = -d;
    }

    return (d <= g_blend_deg) ? RT_TRUE : RT_FALSE;
}

/* 本拍该朝哪：缓弯段在最后 FOLLOW_TURNIN_MM 毫米里，把目标方位从本段逐渐
 * 拧到下一段（smoothstep，首尾斜率为 0）。到点时目标已经等于下一段方位，
 * follow_leg_done() 换段时航向外环看不到台阶。 */
static float follow_drive_yaw(rt_int32_t d_mm)
{
    float        target = follow_leg_yaw(g_leg);
    float        delta, p;
    rt_uint32_t  w;
    rt_int32_t   left;

    if (!g_leg_blend || g_turnin_mm == 0u) {
        return target;
    }

    /* 段比窗口还短时按段长算：否则一进入这一段的第一个 tick 就已经拧过去
     * 一大截，等于把这个航点整个切掉。 */
    w = g_turnin_mm;
    if (w > g_leg_mm[g_leg]) {
        w = g_leg_mm[g_leg];
    }
    if (w == 0u) {
        return target;
    }

    left = (rt_int32_t)g_leg_mm[g_leg] - d_mm;
    if (left >= (rt_int32_t)w) {
        return target;
    }
    if (left < 0) {
        left = 0;
    }

    p = 1.0f - (float)left / (float)w;
    p = p * p * (3.0f - 2.0f * p);

    delta = follow_wrap180(follow_leg_yaw((rt_uint16_t)(g_leg + 1u)) - target);
    return follow_wrap180(target + delta * p);
}

/* ============================================================================
 *  遥测：车 → 上位机
 * ========================================================================== */

/* 【为什么周期是 200ms/2000ms】这条发送路径是忙等自旋（drv_uart.c 的
 * mcx_putc），一行 60 字节要占约 5ms 的 CPU，而这里是 car_head_thread
 * （prio 21）里调的 —— 太快会把 20ms 的控制节拍顶掉。
 * 200ms 一趟 = 2.5% 的占用，换来上位机看得见进度。 */
static void follow_tele_flush(void)
{
    rt_uint16_t leg = (g_state == FOLLOW_IDLE || g_state == FOLLOW_LOAD) ? 0
                                                                        : (rt_uint16_t)(g_leg + 1);

    /* 走完最后一段时 g_leg 已经加到了 g_seg_count，不夹一下会报出 leg=4/3 */
    if (leg > g_seg_count) {
        leg = g_seg_count;
    }

    g_tele_tick  = rt_tick_get();
    g_tele_dirty = RT_FALSE;

    speed_report_sendf("fol st=%s leg=%u/%u d=%d yaw=%d err=%d ab=%u\r\n",
                       follow_state_name(g_state),
                       (unsigned)leg,
                       (unsigned)g_seg_count,
                       (int)g_d_mm,
                       follow_d10(JY61P_GetFilteredYaw()),
                       follow_d10(g_tele_err),
                       (unsigned)g_ab);
}

static void follow_tele_tick(void)
{
    rt_tick_t    now = rt_tick_get();
    rt_uint32_t  period;

    period = (g_state == FOLLOW_TURN || g_state == FOLLOW_DRIVE || g_state == FOLLOW_SETTLE)
             ? FOLLOW_TELE_MS : FOLLOW_TELE_IDLE_MS;

    if (!g_tele_dirty && (now - g_tele_tick) < rt_tick_from_millisecond(period)) {
        return;
    }

    follow_tele_flush();
}

/* ============================================================================
 *  收手
 * ========================================================================== */

/* 起跑时替用户打开的外环，收手时还原。（本来就是开的就不动它。） */
static void follow_restore_head(void)
{
    if (g_head_was_off) {
        Car_SetHeadOn(RT_FALSE);
        g_head_was_off = RT_FALSE;
    }
}

/* 统一的收手出口。brake = FALSE 只给"遥控打断"用，见 Follow_Abort() 的注释。 */
static void follow_fault(rt_uint8_t code, rt_bool_t brake)
{
    if (brake) {
        Car_Brake();
    }

    /* 丢掉航向基准：收手之后外环再拿"航段的方位"去修正就荒唐了
     * （最典型的是遥控打断，用户按着 W，车却自己往轨迹的朝向拐）。 */
    Car_HeadRelease();
    follow_restore_head();

    g_ab    = code;
    g_state = FOLLOW_ABORT;
    g_tele_dirty = RT_TRUE;
    follow_tele_flush();

    LOG_W("follow: abort code=%u at leg %u/%u, d=%d mm",
          (unsigned)code, (unsigned)(g_leg + 1), (unsigned)g_seg_count, (int)g_d_mm);
}

void Follow_Abort(rt_uint8_t code)
{
    if (g_state != FOLLOW_TURN && g_state != FOLLOW_SETTLE && g_state != FOLLOW_DRIVE) {
        return;                     /* 没在跟：别把 idle/done 改成 abort */
    }

    g_ab_code    = code;
    g_ab_pending = RT_TRUE;
}

rt_bool_t Follow_IsRunning(void)
{
    return (g_state == FOLLOW_TURN || g_state == FOLLOW_SETTLE ||
            g_state == FOLLOW_DRIVE) ? RT_TRUE : RT_FALSE;
}

/* ============================================================================
 *  各阶段
 * ========================================================================== */

static rt_bool_t follow_imu_alive(void)
{
    rt_uint32_t frames = JY61P_GetFrameCount();

    if (frames != g_imu_frames) {
        g_imu_frames = frames;
        g_imu_tick   = rt_tick_get();
        return RT_TRUE;
    }

    return ((rt_tick_get() - g_imu_tick) < rt_tick_from_millisecond(FOLLOW_IMU_MS))
           ? RT_TRUE : RT_FALSE;
}

/* 开始走 g_leg 这一段 */
static void follow_start_leg(void)
{
    rt_tick_t    now   = rt_tick_get();
    rt_uint32_t  len   = g_leg_mm[g_leg];
    float        mm_s  = (float)g_spd * FOLLOW_MM_S_PER_TICK;

    if (mm_s < 20.0f) {
        mm_s = 20.0f;               /* 极慢档别把超时算成 0，那样一起步就超时 */
    }
    g_leg_to_ms = (rt_uint32_t)((float)len / mm_s * 1000.0f) * 3u + 3000u;

    g_leg_l0     = motor_get_odo(CAR_MOTOR_LEFT);
    g_leg_r0     = motor_get_odo(CAR_MOTOR_RIGHT);
    g_d_mm       = 0;
    g_leg_tick   = now;
    g_stall_tick = now;
    g_stall_mm   = 0;
    g_err_tick   = 0;
    g_tele_err   = 0.0f;

    g_leg_blend  = follow_blend_ok(g_leg);
    g_state      = FOLLOW_DRIVE;
    g_tele_dirty = RT_TRUE;

    /* 前馈的起点就是本段方位：第一拍的变化率因此从 0 开始，不会一起步就猛拧。 */
    g_yaw_target_prev = follow_leg_yaw(g_leg);
    g_yaw_target_tick = now;
    Car_SetHeadingRate(0.0f);

    LOG_I("follow: leg %u/%u, %u mm%s", (unsigned)(g_leg + 1),
          (unsigned)g_seg_count, (unsigned)len, g_leg_blend ? " (blend)" : "");
}

/* 本段走完 / 原地转到位之后 */
static void follow_turn_done(void)
{
    if (g_cont) {
        follow_start_leg();         /* 连续模式：不停，直接走这一段 */
        return;
    }

    g_settle_to_turn = RT_FALSE;    /* SETTLE 之后去 DRIVE */
    g_settle_tick    = rt_tick_get();
    g_state          = FOLLOW_SETTLE;
    g_tele_dirty     = RT_TRUE;
}

static void follow_leg_done(void)
{
    rt_bool_t blend = g_leg_blend;

    g_leg++;

    if (g_leg >= g_seg_count) {
        Car_Brake();
        Car_HeadRelease();
        follow_restore_head();
        g_state      = FOLLOW_DONE;
        g_tele_dirty = RT_TRUE;
        follow_tele_flush();
        LOG_I("follow: done, %u leg(s)", (unsigned)g_seg_count);
        return;
    }

    /* 缓弯：不刹车、不 SETTLE、不原地转，下一段直接从当前速度和朝向接上。
     * Car_DriveHeading() 会自己重锁基准、清积分，所以这里连那一脚都不用补。 */
    if (blend) {
        follow_start_leg();
        return;
    }

    g_turn_target = follow_leg_yaw(g_leg);
    g_turn_hold   = 0;              /* 不用等，刚从一段里出来 */

    if (g_cont) {
        g_state = FOLLOW_TURN;
    } else {
        Car_Brake();
        g_settle_to_turn = RT_TRUE; /* SETTLE 之后去 TURN */
        g_settle_tick    = rt_tick_get();
        g_state          = FOLLOW_SETTLE;
    }
    g_tele_dirty = RT_TRUE;
}

/* ---- 原地转 ----
 *
 * 【为什么要提前刹车】yaw 大约 10Hz 更新。等"已经到位"那一拍再刹，
 * 车在这 100ms 里又转过去了（250 刻度 ≈ 88 度/秒 ⇒ 9 度）。所以用陀螺的
 * wz 估一个"再转 lead 秒会到哪"，够近就先刹，刹车的那段惯性正好补上剩下的角度。
 *
 * 【刹完为什么按住一会儿】惯性还没走完、wz 的一阶低通（时间常数约 0.33s）
 * 也还挂着旧的角速度。不等就直接按还没衰减的 wz 再判一次，会变成
 * 「提前刹 - 没到位 - 再转」来回蹭。 */
static void follow_turn_tick(void)
{
    rt_tick_t  now = rt_tick_get();
    float      yaw = JY61P_GetFilteredYaw();
    float      err = follow_wrap180(g_turn_target - yaw);
    float      wz  = JY61P_GetFilteredWz();
    float      aerr, mag, lead;
    rt_int16_t v;

    g_tele_err = err;

    if (now < g_turn_hold) {
        Car_Brake();
        return;
    }

    aerr = (err < 0.0f) ? -err : err;

    /* 到线了 */
    if (aerr <= g_tol_deg) {
        Car_Brake();
        g_turn_hold = now + rt_tick_from_millisecond(FOLLOW_TURN_HOLD_MS);
        follow_turn_done();
        return;
    }

    /* 还在往目标转（wz 和 err 同号），而且按当前角速度再转 lead 秒就会冲过头 */
    lead = ((wz < 0.0f) ? -wz : wz) * g_lead_s + g_tol_deg;
    if ((wz * err) > 0.0f && aerr <= lead) {
        Car_Brake();
        g_turn_hold = now + rt_tick_from_millisecond(FOLLOW_TURN_HOLD_MS);
        return;
    }

    mag = g_kp_turn * aerr;
    if (mag < (float)g_vmin) mag = (float)g_vmin;   /* 再慢就转不动了（占空比地板） */
    if (mag > (float)g_vmax) mag = (float)g_vmax;

    /* err > 0：目标比当前大，yaw 右正，要往右转 = 左轮前进、右轮后退。
     * 一直用"轮速"而不是 Car_SpinLeft/Right，是为了不依赖那两个函数的
     * 速度钳位（它们收的是 uint16 且按 CAR_SPEED_MAX 收，这里要的是精确小速度）。 */
    v = (err > 0.0f) ? (rt_int16_t)mag : -(rt_int16_t)mag;
    Car_Drive(v, -v);
}

static void follow_settle_tick(void)
{
    /* 每拍重下一遍：这段时间里谁要是把车发动起来（遥控、msh），立刻按住。
     * 也顺便避开 car.c 记的那个坑：刹车后不足 50ms 又起步会残留 BRAKE。 */
    Car_Brake();

    if ((rt_tick_get() - g_settle_tick) < rt_tick_from_millisecond(FOLLOW_SETTLE_MS)) {
        return;
    }

    if (g_settle_to_turn) {
        g_state      = FOLLOW_TURN;
        g_tele_dirty = RT_TRUE;
        return;
    }

    follow_start_leg();
}

static void follow_drive_tick(void)
{
    rt_tick_t   now = rt_tick_get();
    rt_int32_t  dl, dr, d_mm;
    rt_uint32_t remain;
    float       yaw, err, target;

    /* 本段里程 = 两轮计数增量取平均。走弧线时两边本来就不一样，
     * 而外环还在补左右轮的硬件差异，平均比随便挑一轮稳。 */
    dl = motor_get_odo(CAR_MOTOR_LEFT)  - g_leg_l0;
    dr = motor_get_odo(CAR_MOTOR_RIGHT) - g_leg_r0;
    if (dl < 0) dl = -dl;
    if (dr < 0) dr = -dr;
    d_mm = (rt_int32_t)((float)(dl + dr) * 0.5f * MOTOR_MM_PER_COUNT);

    if (d_mm < g_d_mm) {
        d_mm = g_d_mm;              /* 硬件只给幅值，只增不减是兜底 */
    }
    g_d_mm = d_mm;

    /* 到点了：按目标点往前留 margin 就刹（收速度的那段路是停不下来的）。
     * 【短于 margin 的航段不能直接减】减出来是负数，取 0 的话第一拍就判定
     * "到点了"，车一步都不走 —— 上位机画个 100mm 的小段就会看到它原地不动。
     * 这种段改成走一半就刹：走了多少算多少，剩下的交给下一段的方向修正。
     * 【缓弯段不能减 margin】这一段不停车，减掉的 margin 就等于这条路少走了
     * 这么多；曲线有十来个缓弯时能短掉一米多。走满整段，让下一个弯接着算。 */
    remain = g_leg_mm[g_leg];
    if (g_leg_blend) {
        /* 走满 */
    } else if (remain > g_margin_mm) {
        remain -= g_margin_mm;
    } else {
        remain /= 2u;
    }
    if ((rt_uint32_t)d_mm >= remain) {
        follow_leg_done();
        return;
    }

    /* 本段超时（名义速度的三倍 + 3 秒） */
    if ((now - g_leg_tick) > rt_tick_from_millisecond(g_leg_to_ms)) {
        follow_fault(FOLLOW_AB_LEGTO, RT_TRUE);
        return;
    }

    /* 堵转：命令够大、时间也够，里程却没涨 */
    if ((now - g_stall_tick) > rt_tick_from_millisecond(FOLLOW_STALL_MS)) {
        if ((d_mm - g_stall_mm) < FOLLOW_STALL_MM) {
            follow_fault(FOLLOW_AB_STALL, RT_TRUE);
            return;
        }
        g_stall_tick = now;
        g_stall_mm   = d_mm;
    }

    yaw    = JY61P_GetFilteredYaw();
    target = follow_drive_yaw(d_mm);
    err    = follow_wrap180(target - yaw);
    g_tele_err = err;

    /* 目标方位的角速度 → 航向外环的前馈。
     * 缓弯的 smoothstep 段首尾斜率为 0，所以这里算出来也是从 0 平滑地起来、
     * 到点前又回到 0；非缓弯段目标不变，恒为 0（行为和以前一样）。
     * 【为什么要喂这一项】只靠 P+I，车头要等误差攒出来才动。实测缓弯到点时
     * 目标已经拧到下一段，车头还差约 19 度，过弯后还得补 —— 就是"反应慢"。
     * dt 取真实节拍并限幅，避免调度抖动把变化率除出尖峰；上限 60 度/秒
     * 对应 45 度弯、120mm 窗口、95mm/s 的名义峰值（约 54），再大的弯或更短的
     * 窗口本来也跟不上，与其让差速顶死不如让外环在过点后补。 */
    {
        float dt_s = (float)(now - g_yaw_target_tick) / (float)RT_TICK_PER_SECOND;
        float rate;

        if (dt_s < 0.005f) dt_s = 0.005f;
        if (dt_s > 0.1f)   dt_s = 0.1f;

        rate = follow_wrap180(target - g_yaw_target_prev) / dt_s;
        if (rate >  60.0f) rate =  60.0f;
        if (rate < -60.0f) rate = -60.0f;

        Car_SetHeadingRate(rate);
        g_yaw_target_prev = target;
        g_yaw_target_tick = now;
    }

    /* 偏差超限：被手按住、撞上东西、原地打滑都会走到这儿。
     * 门限（45 度）压在 car.h 的 CAR_HEAD_RELATCH_DEG（90 度）以下 ——
     * 超过 90 度外环会静默重锁基准，那之后跟随看到的就是"没误差"了。 */
    if (err > FOLLOW_ERR_MAX_DEG || err < -FOLLOW_ERR_MAX_DEG) {
        if (g_err_tick == 0) {
            g_err_tick = now;
        } else if ((now - g_err_tick) > rt_tick_from_millisecond(FOLLOW_ERR_HOLD_MS)) {
            follow_fault(FOLLOW_AB_DEV, RT_TRUE);
            return;
        }
    } else {
        g_err_tick = 0;
    }

    Car_DriveHeading(g_spd, target);
}

/* ============================================================================
 *  每拍
 * ========================================================================== */
void Follow_Tick(void)
{
    /* 遥测放在最前面：idle/load 也要往上位机报，不然那边看不出车还在不在。 */
    follow_tele_tick();

    if (g_ab_pending) {
        rt_uint8_t code = g_ab_code;

        g_ab_pending = RT_FALSE;
        /* 【遥控打断不刹车】那一条遥控命令在 bt_car 那边已经生效了，
         * 这里再刹一脚等于把用户刚按下去的方向键吃掉。 */
        follow_fault(code, (code == FOLLOW_AB_REMOTE) ? RT_FALSE : RT_TRUE);
        return;
    }

    if (!Follow_IsRunning()) {
        return;
    }

    /* 闸门 1：解锁状态。遥控的 H 和失联保护都会清 g_armed 并把车刹住，
     * 状态机要是还在跑，下一拍就会重新下令走 —— 必须当场收手。
     * 【顺序：先判解锁再判 IMU】被解除解锁是最常见的打断，报它更有用。 */
    if (!bt_car_is_armed()) {
        follow_fault(FOLLOW_AB_ARM, RT_TRUE);
        return;
    }

    /* 闸门 2：IMU 还出帧吗（不出帧就没有航向，等于闭着眼走） */
    if (!follow_imu_alive()) {
        follow_fault(FOLLOW_AB_IMU, RT_TRUE);
        return;
    }

    switch (g_state) {
    case FOLLOW_TURN:   follow_turn_tick();   break;
    case FOLLOW_SETTLE: follow_settle_tick(); break;
    case FOLLOW_DRIVE:  follow_drive_tick();  break;
    default: break;
    }
}

/* ============================================================================
 *  命令
 * ========================================================================== */
static void follow_usage(void)
{
    rt_kprintf("usage: follow begin <n>          load a path (2~%u points)\n", FOLLOW_MAX_POINTS);
    rt_kprintf("       follow wp <x> <y> <h>     waypoint: mm mm (0.1 deg, CCW+)\n");
    rt_kprintf("       follow end                finish loading, compute leg lengths\n");
    rt_kprintf("       follow start [stop|cont]  run it (default stop-and-go)\n");
    rt_kprintf("       follow stop               stop and release the follower\n");
    rt_kprintf("       follow odom               accumulated encoder counts / mm\n");
    rt_kprintf("       follow status             state and every tunable\n");
    rt_kprintf("       follow sign <1|-1> | spd <v> | margin <mm> | tol <deg>\n");
    rt_kprintf("       follow kp <v> | lead <s> | vmin <v> | vmax <v>\n");
    rt_kprintf("       follow blend <deg>        corner <= deg: no stop (0 = off)\n");
    rt_kprintf("       follow turnin <mm>        start easing into the next bearing\n");
    rt_kprintf("                                 this many mm before the waypoint\n");
    rt_kprintf("speed is the target-speed scale; 1 count = %u (see car.h)\n",
               CAR_SPEED_PER_COUNT);
    rt_kprintf("over the wireless link the same words go behind '#', e.g. \"#follow status\"\n");
    rt_kprintf("  (run it on battery power: USB collapses the motor supply)\n");
}

static void follow_print_status(void)
{
    rt_int32_t l = motor_get_odo(CAR_MOTOR_LEFT);
    rt_int32_t r = motor_get_odo(CAR_MOTOR_RIGHT);

    follow_say("follow: state=%s pts=%u legs=%u leg=%u d=%d mm ab=%u\n",
               follow_state_name(g_state), (unsigned)g_wp_count,
               (unsigned)g_seg_count, (unsigned)(g_leg + 1), (int)g_d_mm, (unsigned)g_ab);
    follow_say("follow: spd=%d margin=%u mm tol=", g_spd, (unsigned)g_margin_mm);
    follow_say("%d.%d deg kp=", (int)g_tol_deg, (int)((g_tol_deg - (float)(int)g_tol_deg) * 10.0f));
    follow_say("%d.%d lead=", (int)g_kp_turn, (int)((g_kp_turn - (float)(int)g_kp_turn) * 10.0f));
    follow_say("%d.%d s vmin=%d vmax=%d blend=",
               (int)g_lead_s, (int)((g_lead_s - (float)(int)g_lead_s) * 10.0f),
               g_vmin, g_vmax);
    follow_say("%d.%d deg turnin=%u mm sign=%d cont=%s\n",
               (int)g_blend_deg, (int)((g_blend_deg - (float)(int)g_blend_deg) * 10.0f),
               (unsigned)g_turnin_mm, (g_sign < 0.0f) ? -1 : 1, g_cont ? "yes" : "no");
    follow_say("follow: odom l=%d r=%d counts, ~%d mm\n",
               (int)l, (int)r,
               (int)((float)(l + r) * 0.5f * MOTOR_MM_PER_COUNT));

    follow_tele_flush();
}

rt_err_t Follow_Cmd(int argc, char **argv)
{
    rt_int32_t v;
    float      f;

    if (argc < 2 || !rt_strcmp(argv[1], "status")) {
        follow_print_status();
        return RT_EOK;
    }

    /* ---- 装航点 ---------------------------------------------------------- */
    if (!rt_strcmp(argv[1], "begin"))
    {
        if (argc < 3 || follow_parse_int(argv[2], &v) != RT_EOK ||
            v < 2 || v > FOLLOW_MAX_POINTS) {
            follow_say("follow: usage: follow begin <2~%u>\n", FOLLOW_MAX_POINTS);
            return -RT_EINVAL;
        }
        if (Follow_IsRunning()) {
            follow_say("follow: busy, `follow stop` first\n");
            return -RT_EBUSY;
        }

        g_wp_expect  = (rt_uint16_t)v;
        g_wp_count   = 0;
        g_seg_count  = 0;
        g_leg        = 0;
        g_ab         = FOLLOW_AB_NONE;
        g_state      = FOLLOW_LOAD;
        g_tele_dirty = RT_TRUE;
        return RT_EOK;
    }

    if (!rt_strcmp(argv[1], "wp"))
    {
        rt_int32_t x, y, h;

        if (g_state != FOLLOW_LOAD) {
            follow_say("follow: wp without begin\n");
            return -RT_EINVAL;
        }
        if (argc < 5 ||
            follow_parse_int(argv[2], &x) != RT_EOK ||
            follow_parse_int(argv[3], &y) != RT_EOK ||
            follow_parse_int(argv[4], &h) != RT_EOK) {
            follow_say("follow: usage: follow wp <x_mm> <y_mm> <h_ddeg>\n");
            return -RT_EINVAL;
        }
        if (x > FOLLOW_COORD_MAX || x < -FOLLOW_COORD_MAX ||
            y > FOLLOW_COORD_MAX || y < -FOLLOW_COORD_MAX ||
            h > 1800 || h < -1800) {
            follow_say("follow: wp out of range (|x|,|y| <= %d mm, |h| <= 1800)\n",
                       FOLLOW_COORD_MAX);
            return -RT_EINVAL;
        }
        if (g_wp_count >= g_wp_expect) {
            follow_say("follow: already got %u waypoints\n", (unsigned)g_wp_count);
            return -RT_EINVAL;
        }

        g_wp[g_wp_count].x_mm   = x;
        g_wp[g_wp_count].y_mm   = y;
        g_wp[g_wp_count].h_ddeg = (rt_int16_t)h;
        g_wp_count++;
        return RT_EOK;
    }

    if (!rt_strcmp(argv[1], "end"))
    {
        rt_uint16_t i;
        rt_uint32_t total = 0;

        if (g_state != FOLLOW_LOAD) {
            follow_say("follow: end without begin\n");
            return -RT_EINVAL;
        }
        if (g_wp_count != g_wp_expect) {
            follow_say("follow: got %u of %u waypoints\n",
                       (unsigned)g_wp_count, (unsigned)g_wp_expect);
            return -RT_EINVAL;
        }

        for (i = 0; i + 1 < g_wp_count; i++) {
            rt_int32_t dx = g_wp[i + 1].x_mm - g_wp[i].x_mm;
            rt_int32_t dy = g_wp[i + 1].y_mm - g_wp[i].y_mm;

            g_leg_mm[i] = follow_isqrt((rt_uint32_t)(dx * dx) + (rt_uint32_t)(dy * dy));
            total      += g_leg_mm[i];
        }

        g_seg_count = (rt_uint16_t)(g_wp_count - 1);
        g_leg       = 0;
        g_ab        = FOLLOW_AB_NONE;
        g_tele_dirty = RT_TRUE;

        follow_say("follow: loaded %u points, %u legs, %u mm total\n",
                   (unsigned)g_wp_count, (unsigned)g_seg_count, (unsigned)total);
        /* 车上装的是什么，让上位机也能看见（点数/段数/总长），
         * 免得"明明发出去了车却说没收到"这种扯不清的事。 */
        speed_report_sendf("fol load n=%u seg=%u mm=%u\r\n",
                           (unsigned)g_wp_count, (unsigned)g_seg_count, (unsigned)total);
        return RT_EOK;
    }

    /* ---- 跑 / 停 --------------------------------------------------------- */
    if (!rt_strcmp(argv[1], "start"))
    {
        rt_bool_t cont = RT_FALSE;

        if (argc >= 3) {
            if (!rt_strcmp(argv[2], "cont"))      { cont = RT_TRUE; }
            else if (!rt_strcmp(argv[2], "stop")) { cont = RT_FALSE; }
            else {
                follow_say("follow: usage: follow start [stop|cont]\n");
                return -RT_EINVAL;
            }
        }

        if (Follow_IsRunning()) {
            follow_say("follow: already running\n");
            return -RT_EBUSY;
        }
        if (g_seg_count == 0) {
            follow_say("follow: no path loaded (begin / wp / end)\n");
            return -RT_EINVAL;
        }
        /* 【必须先解锁】跟随自己不产生"解锁"这件事，它借用遥控那一套：
         * 解锁 = 用户确认了"我现在要让它动"。 */
        if (!bt_car_is_armed()) {
            follow_say("follow: not armed (unlock with O on the remote first)\n");
            return -RT_ERROR;
        }
        if (JY61P_GetFrameCount() == 0) {
            follow_say("follow: no IMU frames, cannot hold a heading\n");
            return -RT_ERROR;
        }

        /* 外环关着的话跟随就是开环盲跑 —— 起跑前替用户打开，收手时还原 */
        g_head_was_off = (Car_GetHeadOn() == RT_TRUE) ? RT_FALSE : RT_TRUE;
        if (g_head_was_off) {
            Car_SetHeadOn(RT_TRUE);
            follow_say("follow: straight-line assist was off, turned it on\n");
        }

        g_cont       = cont;
        g_ab         = FOLLOW_AB_NONE;
        g_leg        = 0;
        g_yaw0       = JY61P_GetFilteredYaw();
        g_d_mm       = 0;
        g_tele_err   = 0.0f;
        g_err_tick   = 0;
        g_turn_hold  = 0;
        g_imu_frames = JY61P_GetFrameCount();
        g_imu_tick   = rt_tick_get();
        g_turn_target = follow_leg_yaw(0);
        g_state      = FOLLOW_TURN;
        g_tele_dirty = RT_TRUE;

        LOG_I("follow: start %s, %u leg(s), yaw0=%d ddeg",
              cont ? "cont" : "stop-and-go", (unsigned)g_seg_count, follow_d10(g_yaw0));
        follow_tele_flush();
        return RT_EOK;
    }

    if (!rt_strcmp(argv[1], "stop"))
    {
        if (Follow_IsRunning()) {
            /* 这里是命令上下文（msh 线程或 bt 遥控线程），刹车当场下 ——
             * 和 follow_fault() 的区别只是状态回 idle 而不是 abort。 */
            Car_Brake();
            Car_HeadRelease();
            follow_restore_head();

            g_state      = FOLLOW_IDLE;
            g_ab         = FOLLOW_AB_NONE;
            g_tele_dirty = RT_TRUE;
            follow_tele_flush();
            LOG_I("follow: stopped by command");
        }
        return RT_EOK;
    }

    /* ---- 心跳 ------------------------------------------------------------ */
    /* 【它什么都不做，这是设计】上位机在跟随期间必须持续发点东西，否则
     * "链路超过 bt link 的时限没有字节"就成立、失联保护会把车刹掉 ——
     * 而下载完路径之后上位机本来就没话说了。收到的任何一个字节都会刷新
     * bt_car 的接收计时，所以这条纯空命令就是最省的心跳。 */
    if (!rt_strcmp(argv[1], "ping")) {
        return RT_EOK;
    }

    /* ---- 读表 ------------------------------------------------------------ */
    if (!rt_strcmp(argv[1], "odom"))
    {
        rt_int32_t l = motor_get_odo(CAR_MOTOR_LEFT);
        rt_int32_t r = motor_get_odo(CAR_MOTOR_RIGHT);

        follow_say("follow: odom l=%d r=%d counts (1 count = %d um)\n",
                   (int)l, (int)r, (int)(MOTOR_MM_PER_COUNT * 1000.0f));
        speed_report_sendf("fol odo l=%d r=%d mm=%d\r\n", (int)l, (int)r,
                           (int)((float)(l + r) * 0.5f * MOTOR_MM_PER_COUNT));
        return RT_EOK;
    }

    /* ---- 整定旋钮 --------------------------------------------------------
     * 这些只能在台架/地面上试出来，写回宏再烧一次太慢，所以全开到运行期。 */
    if (!rt_strcmp(argv[1], "sign") || !rt_strcmp(argv[1], "tol") ||
        !rt_strcmp(argv[1], "kp")   || !rt_strcmp(argv[1], "lead") ||
        !rt_strcmp(argv[1], "blend"))
    {
        if (argc < 3 || follow_parse_float(argv[2], &f) != RT_EOK) {
            follow_usage();
            return -RT_EINVAL;
        }

        if (!rt_strcmp(argv[1], "sign")) {
            if (f != 1.0f && f != -1.0f) {
                follow_say("follow: sign must be 1 or -1\n");
                return -RT_EINVAL;
            }
            g_sign = f;
        } else if (!rt_strcmp(argv[1], "tol")) {
            /* 下限 0.5 度：yaw 是 10Hz 的，判据比这还细没有意义 */
            if (f < 0.5f || f > 45.0f) {
                follow_say("follow: tol out of range (0.5~45 deg)\n");
                return -RT_EINVAL;
            }
            g_tol_deg = f;
        } else if (!rt_strcmp(argv[1], "kp")) {
            if (f < 0.2f || f > 50.0f) {
                follow_say("follow: kp out of range (0.2~50)\n");
                return -RT_EINVAL;
            }
            g_kp_turn = f;
        } else if (!rt_strcmp(argv[1], "blend")) {
            if (f < 0.0f || f > 45.0f) {
                follow_say("follow: blend out of range (0~45 deg, 0 = off)\n");
                return -RT_EINVAL;
            }
            g_blend_deg = f;
        } else {
            if (f < 0.0f || f > 1.0f) {
                follow_say("follow: lead out of range (0~1 s)\n");
                return -RT_EINVAL;
            }
            g_lead_s = f;
        }
        return RT_EOK;
    }

    if (!rt_strcmp(argv[1], "spd") || !rt_strcmp(argv[1], "margin") ||
        !rt_strcmp(argv[1], "vmin") || !rt_strcmp(argv[1], "vmax") ||
        !rt_strcmp(argv[1], "turnin"))
    {
        if (argc < 3 || follow_parse_int(argv[2], &v) != RT_EOK) {
            follow_usage();
            return -RT_EINVAL;
        }

        if (!rt_strcmp(argv[1], "spd")) {
            if (v < CAR_SPEED_PER_COUNT || v > (rt_int32_t)CAR_SPEED_MAX) {
                follow_say("follow: spd out of range (%u~%u)\n",
                           (unsigned)CAR_SPEED_PER_COUNT, (unsigned)CAR_SPEED_MAX);
                return -RT_EINVAL;
            }
            g_spd = (rt_int16_t)v;
        } else if (!rt_strcmp(argv[1], "margin")) {
            if (v < 0 || v > 2000) {
                follow_say("follow: margin out of range (0~2000 mm)\n");
                return -RT_EINVAL;
            }
            g_margin_mm = (rt_uint32_t)v;
        } else if (!rt_strcmp(argv[1], "turnin")) {
            if (v < 0 || v > 500) {
                follow_say("follow: turnin out of range (0~500 mm)\n");
                return -RT_EINVAL;
            }
            g_turnin_mm = (rt_uint32_t)v;
        } else {
            if (v < 2 * CAR_SPEED_PER_COUNT || v > 600) {
                follow_say("follow: v%s out of range (%u~600)\n",
                           (!rt_strcmp(argv[1], "vmin")) ? "min" : "max",
                           (unsigned)(2 * CAR_SPEED_PER_COUNT));
                return -RT_EINVAL;
            }
            if (!rt_strcmp(argv[1], "vmin")) { g_vmin = (rt_int16_t)v; }
            else                             { g_vmax = (rt_int16_t)v; }
        }
        return RT_EOK;
    }

    follow_usage();
    return -RT_EINVAL;
}

static int cmd_follow(int argc, char **argv)
{
    return (int)Follow_Cmd(argc, argv);
}
MSH_CMD_EXPORT_ALIAS(cmd_follow, follow, waypoint following over the wireless link);
