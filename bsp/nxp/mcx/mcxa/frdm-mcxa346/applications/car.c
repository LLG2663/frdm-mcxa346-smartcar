/*
 * SPDX-License-Identifier: MIT
 *
 * car.c —— 小车运动动作层，实现在 car.h 里声明的接口。
 *
 * 本文件只做四件事：
 *   1. 把「带符号差速」翻译成两次 motor_set_target()
 *   2. 把差速组合成 前进 / 后退 / 弧线转弯 / 原地自转
 *   3. 刹车时立刻把 H 桥打成短路，不等 PID 迭代
 *   4. 直行辅助：用 JY901S 的 yaw 做航向外环，把左右轮的硬件差异在线配平
 *      （实车左轮明显快、走不了直线 —— 内环是对称的，偏置在硬件里）
 *
 * 不在这里做闭环控制、也不碰占空比和 H 桥方向——那些是 motor.c 的 PID
 * 线程的活。详见 car.h 头部「与源工程最大的差异」。
 */
#include <rtdevice.h>

#include "car.h"
#include "follow.h"
#include "jy901s.h"
#include "motor.h"

#define DBG_TAG    "car"
#define DBG_LVL    DBG_INFO
#include <rtdbg.h>

/* msh 的 `bt speed` 可以改它，所以要可变 */
static rt_uint16_t car_run_speed = CAR_SPEED_RUN;

/* ----------------------------------------------------------------------------
 *  直行辅助的状态
 * ----------------------------------------------------------------------------
 *  car_base_* : Car_Drive() 记下的「上层原始命令」，外环在它之上叠加修正
 *  car_last_* : 本线程最近真正写下去的目标值，用来发现「别处动过目标」
 *               （Car_Brake()、msh 的 motor stop / motor <1|2> <v>），
 *               一旦对不上就放弃这一轮，绝不把旧基准写回去 —— 刹车必须刹得住。
 *
 *  【为什么带 volatile】它们是三个线程共用的：外环（prio 21）、bt_car 的状态机
 *  （prio 23，通过 Car_Drive() 写 car_base_left/right、car_last_l/r）、
 *  tshell（msh 的 `head` 命令读写开关和增益）。这些 static 变量地址不外传，
 *  编译器有权认定
 *  「没有别的函数能改它」而把值缓存在寄存器里 —— 那样 `head off` / `head kp`
 * 就会"没反应"（README「已知问题」里 yaw_boot_cnt 那条就是这个坑）。
 *  head_last_tick 只有外环自己读写，所以不带。
 * -------------------------------------------------------------------------- */
static volatile rt_int16_t car_base_left  = 0;
static volatile rt_int16_t car_base_right = 0;
static volatile float      car_last_l = 0.0f;
static volatile float      car_last_r = 0.0f;

static volatile rt_bool_t  head_on       = CAR_HEAD_ASSIST_EN;
static volatile rt_bool_t  head_armed    = RT_FALSE;   /* 本轮直行是否已锁参考航向 */
static volatile float      head_ref_yaw  = 0.0f;       /* 起跑那一拍的 yaw */
static volatile float      head_integral = 0.0f;
static volatile float      head_corr     = 0.0f;       /* 最近一次修正量，msh 里看 */
static volatile float      head_rate     = 0.0f;       /* 航向基准的角速度（度/秒），前馈用 */
static rt_tick_t           head_last_tick = 0;
static volatile float      head_kp = CAR_HEAD_KP;
static volatile float      head_ki = CAR_HEAD_KI;
static volatile float      head_kd = CAR_HEAD_KD;
static volatile float      head_sign = CAR_HEAD_SIGN;  /* 运行期 `head sign` 可改 */

/* 外环线程，定义在文件后半（Car_Brake() 之后）；
 * car_hw_init() 在它前面，所以先声明。 */
static void car_head_thread(void *parameter);

/* ----------------------------------------------------------------------------
 *  钳位
 * ----------------------------------------------------------------------------
 *  收在 ±CAR_SPEED_MAX 之内。目标转速是 float，钳完再转，不用担心溢出。
 * -------------------------------------------------------------------------- */
static float Car_Clamp(rt_int32_t speed)
{
    if (speed >  (rt_int32_t)CAR_SPEED_MAX) return  (float)CAR_SPEED_MAX;
    if (speed < -(rt_int32_t)CAR_SPEED_MAX) return -(float)CAR_SPEED_MAX;
    return (float)speed;
}

/* 把上层传进来的 uint16_t 速度收进 0~CAR_SPEED_MAX。
 * 必须先收再取负：不然传个 50000 会在 rt_int16_t 里翻成负数，前进变后退
 * （源工程 car.c 的 Car_Speed() 出于同样的理由先收再取负）。 */
static float Car_Speed(rt_uint16_t speed)
{
    return (speed > (rt_uint16_t)CAR_SPEED_MAX) ? (float)CAR_SPEED_MAX
                                                : (float)speed;
}

/* ============================================================================
 *  Car_Init
 * ========================================================================== */
void Car_Init(void)
{
    Car_Brake();
}

/* 上电刹车，并把默认遥控速度打进日志——整定时先看这一行，再决定
 * `bt speed <v>` 往哪个方向调。 */
static int car_hw_init(void)
{
    rt_thread_t tid;

    Car_Init();

    LOG_I("car ready: run speed %u (~%u encoder count(s) per 50ms sample), max %u",
          Car_GetRunSpeed(), Car_GetRunSpeed() / CAR_SPEED_PER_COUNT, CAR_SPEED_MAX);

    /* 直行辅助的外环。起不来只记日志：车照样能开，只是没有航向修正
     * （Car_StraightTick() 永远不跑 = 回到纯差速开环），不影响别的功能。 */
    tid = rt_thread_create("carhead", car_head_thread, RT_NULL, 1024,
                           CAR_HEAD_THREAD_PRIO, 10);
    if (tid == RT_NULL)
    {
        LOG_E("straight-line assist thread NOT started; driving stays open-loop");
        return RT_EOK;
    }
    rt_thread_startup(tid);

    /* 增益不在这里打（rt_kprintf 不支持 %f）：msh 上 `head` 一次看全 */
    LOG_I("straight-line assist %s (yaw outer loop, corr max %u%% of base); 'head' to tune",
          head_on ? "ON" : "OFF", CAR_HEAD_CORR_PCT);

    return RT_EOK;
}
INIT_APP_EXPORT(car_hw_init);

/* ============================================================================
 *  Car_Drive —— 所有动作的公共出口
 * ============================================================================
 *  拆成两层是为了直行辅助：Car_Drive() 记下「上层要什么」，Car_Apply() 才是
 *  真正下发。外环每一拍用同一个基准 + 新的修正量重新下发，基准本身不被修正
 *  污染（否则 corr 会一拍一拍累加，几拍就顶到限幅）。
 * ========================================================================== */

/* 真正下发：两轮分别 ± corr（正 corr = 左轮加快 / 右轮减慢 = 往右修正）。
 * 钳位照旧用 Car_Clamp()，所以 base ± corr 超出 ±CAR_SPEED_MAX 的那一侧会被
 * 削平（外环想要更大的差速时，表现为两轮平均速度下沉一点，不影响差速方向）。 */
static void Car_Apply(rt_int16_t speed_left, rt_int16_t speed_right, float corr)
{
    car_last_l = Car_Clamp((rt_int32_t)((float)speed_left  + corr));
    car_last_r = Car_Clamp((rt_int32_t)((float)speed_right - corr));

    motor_set_target(CAR_MOTOR_LEFT,  car_last_l);
    motor_set_target(CAR_MOTOR_RIGHT, car_last_r);
}

/* 【base 没变就保住当前 corr】按住 W 时遥控字节是连发的，每个字节都会走到
 * 这里。要是无脑清 0，外环刚推上去的修正量就被按键重复率一遍遍抹掉，20ms 后
 * 它再补回来 —— 外环在"按住 W"的整段时间里始终只能发挥按键间隔比例的那一点
 * 权限（按键大约 30~50ms 一次，外环 20ms 一拍）。
 *
 * 判据只用「新命令和上一拍记下的基准是否一模一样」，不判断命令是不是"直行"：
 * 条件成立时走的确实是同一个直行动作，重新下发同一个结果是幂等的。而
 *   - W→S（前进改后退）：base 变号 ⇒ 清 0。必须清，旧 corr 的符号在倒车下
 *     是反的（见 Car_StraightTick() 第 10 步），留着会朝反方向拽。
 *   - W→A/D（进弧线/自转）：base 从 (+s,+s) 变成 (-s,+s) / (+s,-s) ⇒ 清 0。
 *     外环只管两轮同向同速，Car_StraightTick() 第一句也要求 base_l == base_r。
 *   - A/D 之间来回：一边是 (+s,-s)、另一边是 (-s,+s)，同样对不上 ⇒ 清 0。
 *   - 刹车之后再起步：Car_Brake() 刻意不动 car_base_*，所以它【不会】被这里
 *     当场认出来；下一拍 Car_StraightTick() 第 2 步会发现电机目标被外部改过，
 *     把 base 清成 0 —— 此后再按 W 就是"base 变了"那条路。中间那不到 20ms 的
 *     窗口里 car_base_* 还是旧值，这一拍照旧下发旧 corr（和改之前是一回事，
 *     旧代码只把它换成 0 再在下一拍由外环补回来，两者都不是"重新锁航向"）。
 *
 * 这几种情况清掉之后，下一拍就由 Car_StraightTick() 按新命令重新决定
 * （不再直行的话它第一句就把 head_armed/corr 一起清了）。
 *
 * 刹车不走这里（Car_Brake() 刻意绕开 Car_Drive/Car_Apply），所以这条改动
 * 不影响"刹车必须刹得住"。 */
void Car_Drive(rt_int16_t speed_left, rt_int16_t speed_right)
{
    rt_bool_t same = (speed_left  == car_base_left &&
                      speed_right == car_base_right) ? RT_TRUE : RT_FALSE;
    float     corr = (same && head_on && head_armed) ? head_corr : 0.0f;

    car_base_left  = speed_left;
    car_base_right = speed_right;

    Car_Apply(speed_left, speed_right, corr);
}

/* ============================================================================
 *  五个基本动作
 * ========================================================================== */
void Car_Forward(rt_uint16_t speed)
{
    float s = Car_Speed(speed);

    Car_Drive((rt_int16_t)s, (rt_int16_t)s);
}

void Car_Backward(rt_uint16_t speed)
{
    float s = Car_Speed(speed);

    Car_Drive((rt_int16_t)(-s), (rt_int16_t)(-s));
}

/* ----------------------------------------------------------------------------
 *  弧线转弯的内侧轮速度
 * ----------------------------------------------------------------------------
 *  【源工程的 40% 是按占空比定的】源工程那个 CAR_TURN_RATIO 是【占空比】刻度
 *  （4000 的 40% = 1600，转得动），移植后参数变成了【目标转速】。旧刻度下
 *  1 个编码器计数 = 809，下面的"下限抬到一个计数"会把 outer=800 算出的
 *  inner=320 抬回 809，内外几乎同速，弧线退化成了近似直行。
 *
 *  刻度标定修正后 1 个计数 = 24，默认档 400 的内侧是 160 ≫ 24，不再被抬，
 *  才真的是 60/40 的差速弧线。两道守卫保留给极低的目标速度：
 *    - 下限抬到 1 个编码器计数（24）：再低内侧轮转不动（也够不着占空比地板）。
 *    - outer 不到 2 个计数（48）直接返回 0：这时抬下限会算出 inner ≥ outer，
 *      **内侧比外侧快，车往反方向转**。返回 0 的意思是"这个速度做不出弧线，
 *      退化成绕内侧轮原地转"，而不是悄悄反向。
 *      outer ≥ 2 个计数时，40% 和下限都必然 < outer，不会反超。
 *      边界取 < 而不是 <=：outer 正好 2 个计数时 inner = 1 个计数，
 *      是一条正正经经的 2:1 弧线，不该被挡掉。
 * -------------------------------------------------------------------------- */
static float Car_TurnInner(float outer)
{
    float inner;

    if (outer < (float)(2 * CAR_SPEED_PER_COUNT)) {
        return 0.0f;
    }

    inner = outer * CAR_TURN_RATIO / 100.0f;
    if (inner < (float)CAR_SPEED_PER_COUNT) {
        inner = (float)CAR_SPEED_PER_COUNT;
    }

    return inner;
}

/* 弧线左转：两轮都正转，左轮是内侧所以减速 —— 左轮走的弧长短，车就往左弯。
 * 内侧速度见 Car_TurnInner()：默认档（400）就能做出弧线，只有外侧不到
 * 2 个编码器计数时才退化成原地转（见那边的守卫）。 */
void Car_TurnLeft(rt_uint16_t speed)
{
    float outer = Car_Speed(speed);

    Car_Drive((rt_int16_t)Car_TurnInner(outer), (rt_int16_t)outer);
}

void Car_TurnRight(rt_uint16_t speed)
{
    float outer = Car_Speed(speed);

    Car_Drive((rt_int16_t)outer, (rt_int16_t)Car_TurnInner(outer));
}

/* 原地左转（逆时针）：左轮后退、右轮前进，两轮线速度大小相等，车绕中心自转。
 * 无线串口遥控的 A 键走这里（源工程 main.c 的命令表也是把 A 映射到原地自转）。 */
void Car_SpinLeft(rt_uint16_t speed)
{
    float s = Car_Speed(speed);

    Car_Drive((rt_int16_t)(-s), (rt_int16_t)s);
}

void Car_SpinRight(rt_uint16_t speed)
{
    float s = Car_Speed(speed);

    Car_Drive((rt_int16_t)s, (rt_int16_t)(-s));
}

/* ============================================================================
 *  Car_Brake —— 立刻刹停
 * ============================================================================
 *  为什么不只调 Car_Drive(0, 0)：
 *  那只把目标清 0，真正停下来的动作要等 PID 线程下一拍（最多 50ms）里
 *  motor_update_direction() 发现目标进了死区、把 H 桥打成刹车，同时
 *  DC_MOTOR_PID() 把占空比降下来。对「按 H 停车」和「无线串口失联自动刹车」
 *  来说，50ms 是多出来的滑行距离，没必要等。
 *
 *  所以这里照 motor.c 的 motor_stop_all()（那个是 static，给 `motor stop`
 *  msh 命令用的）做一遍同样的事：先卸占空比，再把两路 H 桥打成刹车。
 *  两处刻意重复，是为了让 car.c 自己就是一条完整的刹车通路——刹车不该
 *  依赖另一个文件里某个 static 函数的可见性。
 *
 *  和 PID 线程不冲突：目标已经清 0，motor_update_direction() 算出来也是
 *  刹车，发现 motor1_dir 已经是 MOTOR_BRAKE 就不再写，方向只有一个真相来源。
 *
 *  【下面两行 motor_dir = MOTOR_BRAKE 必须留】拿掉它们，PID 线程那一拍就会
 *  自己算出"FORWARD → BRAKE 的方向变化"并走一遍卸力，看起来很干净；但会踩另一个
 *  坑：刹车后如果很快（<50ms，下一拍之前）又按了 W，motor_update_direction() 读到
 *  的 curr_dir 还是陈旧的 FORWARD、目标也是 FORWARD，于是【不写方向】——而硬件
 *  还停在 Car_Brake() 设的 BRAKE 上。结果是目标 800、占空比一路顶到 4000、
 *  电桥却始终刹着。这两行是让硬件状态和簿记保持一致的。
 *
 *  PWM_x_duty 那几个 static 变量这里够不着（在 motor.c 里），所以 PID 下一拍会
 *  把它记着的旧占空比写回寄存器 —— 因为方向已经是刹车，motor_update_direction()
 *  看不出方向变化、不会走 motor.c 的 motor_switch_dir() 卸力路径，占空比就一直
 *  挂着。这个残留无害：BRAKE 是 H 桥两臂同电平、绕组短接，占空比写多少都不产生
 *  力矩。唯一的可见后果是 `motor status` 在刹停期间会报一个非零的 duty（那是 PID
 *  的内部状态，不是电机上的实际电压）。
 *
 *  而【下一次起步一定是 BRAKE→FORWARD 的方向变化】，那条路径会走 motor_switch_dir()
 *  把占空比清 0，从 0/起步补偿重新爬 —— 所以按 W 不会再窜一下。
 *
 *  【直行辅助不会把车顶回来】这里刻意不走 Car_Drive()/Car_Apply()，所以
 *  car.c 的 car_last_l/r 会停在刹车之前的值。Car_StraightTick() 第 2 步就是
 *  拿电机当前目标和它比对，对不上就立刻放弃这一轮、并且【不下发】——
 *  刹车因此一定刹得住，用不着让 Car_Brake() 去关心直行辅助。
 * ========================================================================== */
void Car_Brake(void)
{
    motor_set_target(CAR_MOTOR_LEFT,  0.0f);
    motor_set_target(CAR_MOTOR_RIGHT, 0.0f);

    motor_set_duty(CAR_MOTOR_LEFT,  0);
    motor_set_duty(CAR_MOTOR_RIGHT, 0);

    motor_set_direction(CAR_MOTOR_LEFT,  MOTOR_BRAKE);
    motor_set_direction(CAR_MOTOR_RIGHT, MOTOR_BRAKE);

    motor1_dir = MOTOR_BRAKE;
    motor2_dir = MOTOR_BRAKE;
}

/* ============================================================================
 *  直行辅助 —— 用 JY901S 的 yaw 做航向外环
 * ============================================================================
 *  内环是 motor.c 里两轮各自的速度 PID（逐行对称），它把「左轮快一点」这种
 *  硬件差异原样保留下来；外环就是拿来消掉它的：直行时按 yaw 的误差给两轮
 *  差速修正，积分项自己收敛到抵消差异所需的那一点点差速，然后保持不动。
 *
 *  为什么必须带积分项：只用 P 的话，稳态时得留一个恒定的航向误差去产生那个
 *  恒定差速（P 的输出正比于误差），车是走直线了，但方向是"歪着"的、跟起跑
 *  时指的方向差一截；积分项把这份差速记住、误差归零，车头才真的朝原方向。
 *
 *  corr 为正是「左轮加快 / 右轮减慢」，也就是把车往右修正。
 * ========================================================================== */

/* 把两个角差归一到 -180~+180。jy901s.c 里那个同名 helper 是 static，这里照
 * car.c 一贯的做法自己留一份（和 Car_Brake() 与 motor_stop_all() 的关系一样：
 * 宁可两处重复，也不为了三行代码去动另一个模块的可见性）。 */
static float Car_Wrap180(float a)
{
    if (a > 180.0f)  a -= 360.0f;
    if (a < -180.0f) a += 360.0f;
    return a;
}

/* 本拍的修正量上限。上限跟着基准速度走（CAR_HEAD_CORR_PCT），
 * 理由见 car.h 那段：corr > base 会把慢的那一轮推成反转。
 * 再兜一道 CAR_SPEED_MAX —— base 由上层钳过，这里只是不假设。 */
static float Car_HeadCorrLimit(void)
{
    float lim = (float)car_base_left * CAR_HEAD_CORR_PCT / 100.0f;

    if (lim < 0.0f) lim = -lim;                     /* 后退时 base 为负 */
    if (lim > (float)CAR_SPEED_MAX) lim = (float)CAR_SPEED_MAX;
    return lim;
}

/* 航向直行外环，每 CAR_HEAD_PERIOD_MS 一拍。返回本拍是否发了修正。
 *
 * 只有「两轮同向同速」的直行动作参与：原地自转 / 弧线（base_l != base_r）
 * 和刹停后（base == 0）自动退出，不需要 bt_car 那边通知。
 */
static rt_bool_t Car_StraightTick(void)
{
    float yaw, err, corr, dt, i_max, corr_max;

    /* 1. 只在直行命令下生效 */
    if (!head_on || car_base_left == 0 || car_base_left != car_base_right) {
        head_armed    = RT_FALSE;
        head_integral = 0.0f;
        head_corr     = 0.0f;
        head_rate     = 0.0f;       /* 不是直行动作，前馈跟着作废 */
        return RT_FALSE;
    }

    /* 2. 别处动过目标就立刻放弃这一轮，绝不把旧基准写回去。
     *    Car_Brake()（遥控的空格/X/H）和 msh 的 `motor stop` / `motor <1|2> <v>`
     *    都会命中这里 —— 刹车必须刹得住，不能被外环在 20ms 后顶回来。
     *    注意这里【必须先返回再下发】，不能"发现不一致但照写"。
     *
     *    【已知边界】Car_Drive() 两句 motor_set_target() 之间如果正好被本线程
     *    抢占，会读到"半个新命令"而误判成外部改动 —— 窗口约 1us / 20ms，
     *    后果只是这一次直行辅助不生效（再发一次命令就恢复），不是安全问题。 */
    if (motor_get_target(CAR_MOTOR_LEFT)  != car_last_l ||
        motor_get_target(CAR_MOTOR_RIGHT) != car_last_r) {
        car_base_left  = 0;
        car_base_right = 0;
        head_armed     = RT_FALSE;
        head_corr      = 0.0f;
        head_rate      = 0.0f;
        return RT_FALSE;
    }

    /* 3. IMU 没接 / 刚上电还没出帧：不修正，按原样开环跑。
     *    不能拿 filtered_yaw 的初值 0 度当航向 —— 那会让车一上来就朝"0 度"
     *    猛拐。frame_count 是 jy901s 的现成诊断计数（没帧 = 一直 0）。 */
    if (JY61P_GetFrameCount() == 0) {
        return RT_FALSE;
    }

    /* 4. 起跑那一拍锁定参考航向：车现在指哪，"直行"就是哪个方向 */
    if (!head_armed) {
        head_ref_yaw   = JY61P_GetFilteredYaw();
        head_integral  = 0.0f;
        head_armed     = RT_TRUE;
        head_last_tick = rt_tick_get();
        return RT_FALSE;            /* 这一拍 err 恒为 0，不用算 */
    }

    /* 5. 积分用真实 dt：节拍改了、模块出帧率是多少，都不影响 KI 的含义。
     *    （模块一帧 0x53 大约 10Hz，同一帧会被重算几次，这是零阶保持，
     *      和普通数字 PID 面对慢传感器是一样的。） */
    {
        rt_tick_t now = rt_tick_get();

        dt = (float)(now - head_last_tick) / (float)RT_TICK_PER_SECOND;
        head_last_tick = now;
    }
    if (dt > 0.5f) {
        dt = 0.5f;                  /* 线程被饿住过，别让积分一步跳太远 */
    }

    yaw = JY61P_GetFilteredYaw();
    err = Car_Wrap180(head_ref_yaw - yaw);

    /* 6. 误差大得不像正常行驶（车被人挪过 / 轮子卡住打滑）：重新锁定参考航向。
     *    把车硬拽回一个已经没意义的旧朝向是危险的。 */
    if (err > CAR_HEAD_RELATCH_DEG || err < -CAR_HEAD_RELATCH_DEG) {
        head_ref_yaw  = yaw;
        head_integral = 0.0f;
        head_corr     = 0.0f;
        return RT_FALSE;
    }

    /* 7. 本拍的修正上限：跟着基准速度走（CAR_HEAD_CORR_PCT）。 */
    corr_max = Car_HeadCorrLimit();

    /* 8. PI(D)。积分限幅 = 让积分项单独最多贡献 corr_max（head_ki 由 msh
     *    收在 1~100，不会除零）。 */
    head_integral += err * dt;
    i_max = corr_max / head_ki;
    if (head_integral >  i_max) head_integral =  i_max;
    if (head_integral < -i_max) head_integral = -i_max;

    /* 前馈：目标方位自己正在以 head_rate 度/秒转动，先把维持这个角速度所需的
     * 差速直接给出去（含 D 项要扣掉的那份，见 car.h 的 CAR_HEAD_RATE_FF）。
     * 基准不动时它是 0，行为与原来完全一致。 */
    corr = head_kp * err + head_ki * head_integral
         - head_kd * JY61P_GetFilteredWz()
         + CAR_HEAD_RATE_FF * head_rate;

    /* 9. 抗饱和：输出顶到限幅了，就把这一拍新加的那份积分退回去 —— 积分只在
     *    「还有余量」时才继续攒。没有这一步，台架上车不动、航向误差常在，积分会
     *    一直攒在限幅上，一松手就是满舵（实测：手把车掰偏一点，两轮猛地一正一反）。
     *    退的是 err*dt 这一份，不是直接把积分清零：顶在限幅上的那部分稳态配平量
     *    得留着（那是外环的工作成果），只丢掉增量。 */
    if (corr > corr_max || corr < -corr_max)
    {
        head_integral -= err * dt;
        if (head_integral >  i_max) head_integral =  i_max;
        if (head_integral < -i_max) head_integral = -i_max;
        corr = head_kp * err + head_ki * head_integral
             - head_kd * JY61P_GetFilteredWz()
             + CAR_HEAD_RATE_FF * head_rate;
    }

    /* 10. 符号。后退时左右修正要反号：yaw 的误差是固定取向的（地理方向），
     *     而「左轮快 = 车往右转」只在前进行驶时成立 —— 倒着开时左轮快，
     *     车尾往右甩、车头反而往左转。 */
    if (car_base_left < 0) {
        corr = -corr;
    }
    corr *= head_sign;

    /* 11. 最后按本拍上限钳一次（符号翻转不会改变绝对值，但 sign 之外再钳一次
     *     就不必假设这件事）。 */
    if (corr >  corr_max) corr =  corr_max;
    if (corr < -corr_max) corr = -corr_max;

    head_corr = corr;
    Car_Apply(car_base_left, car_base_right, corr);
    return RT_TRUE;
}

/* ============================================================================
 *  Car_DriveHeading —— 「朝这个航向直行」的入口（给 follow.c 的航点跟随用）
 * ============================================================================
 *  航点跟随要的不是"保持起跑朝向"，是"保持某个指定朝向"，所以不能走
 *  Car_StraightTick() 第 4 步那套「谁先跑谁锁基准」。这里先把基准定好，再让
 *  Car_Drive() 照常下发；同一拍随后的 Car_StraightTick() 看到 head_armed 已经
 *  置位，就会用这个新基准跑既有的 P+I 修正 —— 一个航向环两处共用，
 *  不另写第二套（也就不会有两套抗饱和/重锁逻辑打架）。
 *
 *  【为什么必须同时刷新 head_last_tick】第 5 步的 dt 来自它。换段前车刚刹停过
 *  （或刚原地转过），那段时间 Car_StraightTick() 每次都在第 1/2 步就返回了，
 *  根本没碰过 head_last_tick —— 不在这儿补一刀，换段后第一拍会算出零点几秒的
 *  dt（还会被 0.5s 的上限截住），积分项一步就冲上去，起步那一下就是要偏的。
 */
void Car_DriveHeading(rt_int16_t speed, float target_yaw)
{
    float d = Car_Wrap180(target_yaw - head_ref_yaw);

    /* 目标跳了一大步（换段/换动作）就把积分清掉：旧段的积分是"为了抵消上一段
     * 那点硬件差速"攒的，挪到新基准上没有意义。
     *
     * 【基准必须每拍跟着目标走，不能只在跳变时才更新】航点跟随的缓弯会在
     * 转弯前把目标方位逐渐拧过去，每拍只动零点几度。旧实现只在 |d|>0.5 时
     * 才写 head_ref_yaw，结果缓弯里基准一直锁在旧值上，等累计差超过 0.5 度
     * 时再"跳"一下 —— 航向外环看到的就是一串台阶，拐弯一顿一顿。现在
     * 无论大小都更新基准，只有真正的跳变才清积分（门限见 CAR_HEAD_STEP_DEG）。 */
    if (!head_armed || d > CAR_HEAD_STEP_DEG || d < -CAR_HEAD_STEP_DEG)
    {
        head_integral  = 0.0f;
        head_last_tick = rt_tick_get();
    }

    head_ref_yaw = target_yaw;
    head_armed = RT_TRUE;           /* 第 4 步不再自己锁基准 */
    Car_Drive(speed, speed);
}

static void car_head_thread(void *parameter)
{
    RT_UNUSED(parameter);

    while (1)
    {
        /* 航点跟随的状态机。不在跟随状态时它是一句 return，开销为零。
         * 【顺序不能换】：跟随会通过 Car_DriveHeading() 设定基准，紧接着的
         * Car_StraightTick() 才是拿这个基准去算修正的那一拍。同一个线程里
         * 顺序调用，所以两者之间没有竞态，也不需要临界区。 */
        Follow_Tick();
        Car_StraightTick();
        rt_thread_mdelay(CAR_HEAD_PERIOD_MS);
    }
}

/* ============================================================================
 *  遥控速度旋钮
 * ========================================================================== */
rt_uint16_t Car_GetRunSpeed(void)
{
    return car_run_speed;
}

void Car_SetRunSpeed(rt_uint16_t speed)
{
    car_run_speed = (speed > (rt_uint16_t)CAR_SPEED_MAX) ? CAR_SPEED_MAX : speed;
}

/* 直行辅助开关的读写。msh 的 `head on|off` 走 Car_CmdHead()，那两个是给
 * 代码用的 —— follow.c 起跑时要先确认外环是开的（关着的话航点跟随就是
 * 纯开环盲跑），开之前把原值存下来，跑完/中止再还原。 */
rt_bool_t Car_GetHeadOn(void)
{
    return head_on;
}

/* 丢掉航向基准：下一拍 Car_StraightTick() 第 4 步会按"当前朝向"重新锁。
 *
 * 给航点跟随放手用。最典型的场景是「跟随中途按了遥控 W」：那一下 Car_Forward()
 * 已经把 base 和目标都下发了，这里要是留着头_armed，紧接着的 Car_StraightTick()
 * 就会拿【航段的航向】去修正用户刚发出的 W —— 用户想的明明是"就按现在这个方向
 * 往前走"，车却会自己往轨迹的朝向拐。松开之后 W 就是普通的直线保持。 */
void Car_HeadRelease(void)
{
    head_armed    = RT_FALSE;
    head_integral = 0.0f;
    head_corr     = 0.0f;
    head_rate     = 0.0f;
}

/* 航向速率前馈的入口（follow.c 的缓弯用）。只写这一个变量，真正生效在
 * 下一拍 Car_StraightTick()；两轮不同速时它会被第 1 步清掉。 */
void Car_SetHeadingRate(float deg_per_s)
{
    head_rate = deg_per_s;
}

void Car_SetHeadOn(rt_bool_t on)
{
    head_on = (on == RT_TRUE) ? RT_TRUE : RT_FALSE;
}

/* ============================================================================
 *  msh: head —— 直行辅助的状态与整定旋钮
 * ============================================================================
 *  为什么这些旋钮要放到运行期：kp/ki/kd 只能"看着车"调，写回宏再烧一次
 *  试一个数太慢。定下来之后把值填回 car.h 的 CAR_HEAD_* 就行。
 * ========================================================================== */

/* rt_kprintf 是 tiny vsnprintf，不支持 %f（LFLAGS 里也没有 -lm）：
 * 定点打印，照 bt_car.c 的 bt_print_int() / jy901s.c 的 jy901s_print_fixed()。
 * 不做四舍五入到负零那套（角度打印不需要那么讲究）。 */
static void car_print_fixed(float v, int decimals)
{
    float mag = (v < 0.0f) ? -v : v;
    int   scale = (decimals == 1) ? 10 : 1;
    int   total = (int)(mag * (float)scale + 0.5f);

    if (v < 0.0f && total != 0) {
        rt_kprintf("-");
    }
    rt_kprintf("%d", total / scale);
    if (decimals > 0) {
        rt_kprintf(".%d", total % scale);
    }
}

/* 解析一个可带符号、可带小数的增益。不用 atof()：本 BSP 是 RT-Thread 的
 * klibc，没有 atof/strtof（motor.c 的 motor_parse_speed() 同样的理由）。 */
static rt_err_t car_parse_float(const char *s, float *out)
{
    float value = 0.0f;
    float scale = 0.1f;
    int   neg = 0;
    int   seen_digit = 0;
    int   fd = 0;

    if (*s == '+' || *s == '-') {
        neg = (*s == '-');
        s++;
    }

    while (*s >= '0' && *s <= '9') {
        value = value * 10.0f + (float)(*s - '0');
        seen_digit = 1;
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
            seen_digit = 1;
            s++;
        }
    }

    if (!seen_digit || *s != '\0') {
        return -RT_EINVAL;
    }

    *out = neg ? -value : value;
    return RT_EOK;
}

static void car_msh_usage(void)
{
    rt_kprintf("usage: head                     show straight-line assist state\n");
    rt_kprintf("       head on | off             enable/disable (default %s)\n",
               CAR_HEAD_ASSIST_EN ? "on" : "off");
    rt_kprintf("       head kp <v> | ki <v> | kd <v>   tune the yaw outer loop\n");
    rt_kprintf("       head sign <1|-1>          flip correction direction\n");
    rt_kprintf("       head zero                 take current heading as \"straight\"\n");
    rt_kprintf("corr is in the target-speed scale; 1 encoder count = %u\n",
               CAR_SPEED_PER_COUNT);
    rt_kprintf("corr is capped at %u%% of the base speed (see CAR_HEAD_CORR_PCT)\n",
               CAR_HEAD_CORR_PCT);
    rt_kprintf("over the wireless link the same words go behind '#', e.g. \"#head sign -1\"\n");
    rt_kprintf("  (battery power: with USB plugged in the motor supply collapses)\n");
}

/* 实际转速换算成「几个编码器计数」（刻度见 car.h 头部：1 个计数 = 24）。
 * 只给 `head` 用：它和 base/corr 摆在一起看，"命令 2/2 但实测 3/1" 一眼就能
 * 看出外环正在补硬件差异。 */
static int car_speed_counts(rt_uint8_t motor_id)
{
    return (int)(motor_get_speed(motor_id) / (float)CAR_SPEED_PER_COUNT);
}

static void head_print_status(void)
{
    rt_kprintf("head: assist=%s armed=%s kp=", head_on ? "on" : "off",
               head_armed ? "yes" : "no");
    car_print_fixed(head_kp, 1);
    rt_kprintf(" ki=");
    car_print_fixed(head_ki, 1);
    rt_kprintf(" kd=");
    car_print_fixed(head_kd, 1);
    /* corr 上限是「基准速度的百分之多少」，不是一个具体数字 —— 具体值随档位
     * 变，要和同一行的 base= 一起看（base 400 + 100% ⇒ 实际钳在 ±400）。 */
    rt_kprintf(" corr_max=%u%%base sign=", CAR_HEAD_CORR_PCT);
    car_print_fixed(head_sign, 0);
    rt_kprintf("\n");

    rt_kprintf("      base=%d/%d yaw=", car_base_left, car_base_right);
    car_print_fixed(JY61P_GetFilteredYaw(), 1);
    rt_kprintf(" ref=");
    car_print_fixed(head_ref_yaw, 1);
    rt_kprintf(" corr=");
    car_print_fixed(head_corr, 1);
    rt_kprintf(" integ=");
    car_print_fixed(head_integral, 1);

    /* imu frames=0 就说明直行辅助根本拿不到角度（模块没接/没出帧），
     * 这时 Car_StraightTick() 第 3 步会直接返回，车是纯开环在跑。
     * speed 是两轮的实际转速（单位 = 编码器计数/50ms），和 base/corr 摆在一起看：
     * "命令 2/2 但实测 3/1" = 外环正在补硬件差异；再看 `motor status` 的 duty
     * 就知道慢的那一轮是不是已经顶到满占空比了。 */
    rt_kprintf("  imu frames=%u speed=%d/%d\n", JY61P_GetFrameCount(),
               car_speed_counts(CAR_MOTOR_LEFT), car_speed_counts(CAR_MOTOR_RIGHT));
}

/* 【msh 和无线串口共用这一个入口】无线串口上收到 "#head ..." 时由 speed_report.c
 * 原样转调这里（理由见 car.h 里 Car_CmdHead() 的说明：插 USB 调不了外环）。
 * 所以下面这些 rt_kprintf 从无线串口进来时打给的是 uart2 控制台 —— 那条链路上
 * 看不见，无线串口侧只回一行 ok/err，数值从周期上报行的 tgt 里读。
 * 参数不合法时走的 car_msh_usage() 同理：只有插着 USB 才看得到提示。 */
rt_err_t Car_CmdHead(int argc, char **argv)
{
    float v;

    if (argc < 2 || !rt_strcmp(argv[1], "status"))
    {
        head_print_status();
        return RT_EOK;
    }

    if (!rt_strcmp(argv[1], "on") || !rt_strcmp(argv[1], "off"))
    {
        head_on = (!rt_strcmp(argv[1], "on")) ? RT_TRUE : RT_FALSE;
        rt_kprintf("head: straight-line assist %s\n", head_on ? "ON" : "OFF");
        return RT_EOK;
    }

    if (!rt_strcmp(argv[1], "zero"))
    {
        /* 需要在直行时才动得了：head_armed 为假时，下一拍本来就会重新锁 */
        head_ref_yaw  = JY61P_GetFilteredYaw();
        head_integral = 0.0f;
        head_corr     = 0.0f;
        rt_kprintf("head: reference heading re-latched to current yaw\n");
        return RT_EOK;
    }

    if (!rt_strcmp(argv[1], "sign"))
    {
        /* 只收 ±1：这不是增益，是「修正方向」，别的值没有意义。
         * 这是实测「越修越偏」时不用重烧的那条路（理由：yaw 的正负取决于模块
         * 怎么装在车上，焊死一个符号就得为了一个正负号重新编译一次）。 */
        if (argc < 3 || car_parse_float(argv[2], &v) != RT_EOK ||
            (v != 1.0f && v != -1.0f)) {
            car_msh_usage();
            return -RT_EINVAL;
        }

        /* 积分必须一起清：旧符号下攒起来的那份"配平量"在新符号下是反的，
         * 留着等于起步就带一个方向相反的偏置。 */
        head_sign     = (v < 0.0f) ? -1.0f : 1.0f;
        head_integral = 0.0f;
        head_corr     = 0.0f;

        rt_kprintf("head: correction sign = ");
        car_print_fixed(head_sign, 0);
        rt_kprintf(" (integral reset)\n");
        return RT_EOK;
    }

    if (!rt_strcmp(argv[1], "kp") || !rt_strcmp(argv[1], "ki") || !rt_strcmp(argv[1], "kd"))
    {
        if (argc < 3 || car_parse_float(argv[2], &v) != RT_EOK) {
            car_msh_usage();
            return -RT_EINVAL;
        }

        /* ki 的下限不是美观问题：Car_StraightTick() 里有 CORR_MAX/ki 的除法 */
        if (!rt_strcmp(argv[1], "kp")) {
            if (v < 0.0f)   v = 0.0f;
            if (v > 200.0f) v = 200.0f;
            head_kp = v;
        } else if (!rt_strcmp(argv[1], "ki")) {
            if (v < 1.0f)   v = 1.0f;
            if (v > 100.0f) v = 100.0f;
            head_ki = v;
        } else {
            if (v < 0.0f)   v = 0.0f;
            if (v > 100.0f) v = 100.0f;
            head_kd = v;
        }

        head_print_status();
        return RT_EOK;
    }

    car_msh_usage();
    return -RT_EINVAL;
}
/* msh 侧的包装。和 speed_report.c 的 cmd_spd() → spd_dispatch() 是同一种做法：
 * 一套解析两个入口，不会出现「msh 上能改、无线串口改了没反应」（也不会反过来）。 */
static int cmd_head(int argc, char **argv)
{
    return (int)Car_CmdHead(argc, argv);
}
MSH_CMD_EXPORT_ALIAS(cmd_head, head, yaw-based straight-line assist (on|off|kp|ki|kd|sign|zero));
