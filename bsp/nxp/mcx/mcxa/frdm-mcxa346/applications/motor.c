/*
 * SPDX-License-Identifier: MIT
 *
 * TB6612 双电机 + 编码器速度闭环，移植自 CCS 工程 113/user_driver/motor.c
 * 与 113/user_driver/interrupt.c（TI MSPM0G3507）。
 *
 * 引脚分配、与源工程的差异、测速单位为什么保持原样 —— 都写在 motor.h 顶部。
 * 本文件里改动过的地方都标了 [PORT]，其余部分是源工程逐行照搬，
 * 包括中文注释。
 */

#include <rtdevice.h>
#include "drv_pin.h"

#include "motor.h"

#define DBG_TAG    "motor"
#define DBG_LVL    DBG_INFO
#include <rtdbg.h>

/* ============================================================================
 *  引脚与设备（对照表见 motor.h）
 * ============================================================================
 */
#define MOTOR1_AIN1_PIN     GET_PINS(3, 31)     /* D2  */
#define MOTOR1_AIN2_PIN     GET_PINS(4, 7)      /* D4  */
#define MOTOR2_BIN1_PIN     GET_PINS(3, 22)     /* D7  */
#define MOTOR2_BIN2_PIN     GET_PINS(4, 3)      /* D8  */
#define MOTOR_STBY_PIN      GET_PINS(3, 14)     /* D3，两路共用 */
#define MOTOR1_ENC_PIN      GET_PINS(3, 1)      /* D5  */
#define MOTOR2_ENC_PIN      GET_PINS(3, 17)     /* D6  */

#define MOTOR1_PWM_DEV      "pwm1"              /* P3_8  = PWM0_A1 = D11 */
#define MOTOR2_PWM_DEV      "pwm2"              /* P3_10 = PWM0_A2 = D13 */

/* [PORT] drv_pwm.c 的 mcx_drv_pwm_set() 忽略 configuration->channel，
 * 内部固定用 kPWM_PwmA。这里传 1 只是为了满足接口，值本身无意义。 */
#define MOTOR_PWM_CHANNEL   1

#define MOTOR_PID_PERIOD_MS 50          /* 源工程 PID 定时器周期 */
#define MOTOR_PID_THREAD_PRIO  21       /* 低于 tshell(20)，高于 jy901s(22) */
#define MOTOR_PID_THREAD_STACK 1024

/* ============================================================================
 *  电机方向状态（H桥当前状态）
 * ============================================================================
 *  由 motor_update_direction() 维护，带死区滞后
 */
volatile rt_uint8_t motor1_dir = MOTOR_BRAKE;
volatile rt_uint8_t motor2_dir = MOTOR_BRAKE;

/* [PORT] 源工程这里是 motor_control_mode = MOTOR_MODE_LINE_FOLLOWING，
 * 并靠它决定 PID 跑不跑。该变量全工程没有任何赋值点（原本由不存在的
 * gray.c 设置），那个 if 恒为真，所以直接删掉。原因见 motor.h 差异 1。 */

/* 两路 PWM 设备与编码器计数 */
static struct rt_device_pwm *motor1_pwm = RT_NULL;
static struct rt_device_pwm *motor2_pwm = RT_NULL;

/* ============================================================================
 *  编码器计数（由 GPIO 中断累加，对应源工程 interrupt.c）
 * ============================================================================
 *  [PORT] 只计 A 相，和源工程一致（interrupt.c 里只有 E1A / E2A 两个中断）。
 *         没有方向信息，所以 PID 里用的是 abs_target —— 两者是配套的。
 *
 *  变量名 counter_1_A / counter_1_B 沿用源工程：前者是电机1的 A 相，
 *  后者其实是【电机2】的 A 相。名字有误导性，但改名字会让 calculate_speed()
 *  和 PID 那几行跟源工程对不上，权衡后保留原名，在这里说清楚。
 *
 *  volatile + 32 位对齐：读写在 M33 上是单条指令，中断里自增不会读到半个值，
 *  不需要额外的临界区。
 */
static volatile rt_uint32_t counter_1_A = 0;    /* 电机1编码器A相计数值 */
static volatile rt_uint32_t counter_1_B = 0;    /* 电机2编码器A相计数值 */

/* 累计里程（单位：编码器计数，永远不清零）。
 *
 * 为什么另开一份而不是让 follow.c 去累加 counter_1_A/B：那两个是
 * calculate_speed() 每 50ms 清零一次的「本周期增量」，谁在外面读都是一场竞态。
 * 唯一没有竞态的地方就是清零的那一行本身，所以累加写在那儿（先累加再清零）。
 *
 * 为什么用计数而不是 motor_get_speed() 积分：speed_1/2 的刻度是台架上标定过的
 * （motor.h 差异 2，比真 mm/s 大 2.5 倍且刻度本身可调），拿它积出来的距离会跟着
 * 刻度跑。计数是硬件事实，只过一次 MOTOR_MM_PER_COUNT 就是毫米。 */
static volatile rt_int32_t  odo_1 = 0;
static volatile rt_int32_t  odo_2 = 0;

/* 实测速度（刻度见 motor.h 差异 2：已按实车标定，1 个刻度 ≈ 0.40 mm/s） */
static float speed_1 = 0.0f;
static float speed_2 = 0.0f;

/* ============================================================================
 *  编码器中断回调（对应源工程 interrupt.c 的 GROUP1_IRQHandler）
 * ============================================================================
 *  [PORT] 源工程在中断里判断是哪个脚、再清中断标志；RT-Thread 的 drv_pin.c
 *         已经把「清标志 + 按脚分发」做好了（pin_irq_handler()），
 *         回调里只需要给对应的计数器加一。
 *         给哪个计数器加，通过 args 传进来，省掉一份全局状态。
 */
static void motor_encoder_irq(void *args)
{
    volatile rt_uint32_t *counter = (volatile rt_uint32_t *)args;

    (*counter)++;
}

/* ============================================================================
 *  motor_init - 初始化单个电机
 * ============================================================================
 *  使能 STBY 引脚，启动 PWM 定时器，设置初始方向（刹车）。
 *  注意：PID 线程由 motor_hw_init() 在所有硬件就绪后统一启动。
 */
void motor_init(rt_uint8_t motor_id)
{
    /* 顺序很重要：先把占空比归零、方向置刹车，最后才抬 STBY 使能驱动芯片。
     *
     * 源工程的生成配置把 PWMA/PWMB 的比较值预置成 4000（100%）且定时器已经在跑，
     * 所以如果先抬 STBY，中间会有一个"驱动已使能 + 100% 占空比"的窗口。
     * 本板复位后 AIN1/AIN2 同为 LOW、两输入相等不产生驱动电压，靠推理才安全；
     * 但一旦哪天 motor_init() 漏调，控制代码先设方向再设占空比就是满速飞车。
     * 这个顺序是纯收益的保险。
     *
     * [PORT] 源工程是先设比较值、再 DL_Timer_startCounter()。这里 rt_pwm_set()
     *        写的是带缓冲的比值，要等 rt_pwm_enable() 让计数器跑起来、
     *        遇上一次 reload 才会真正生效，所以「归零 → 使能」的顺序
     *        同样保证了使能瞬间的占空比是 0。 */
    if (motor_id == 1) {
        motor_set_duty(1, 0);
        motor_set_direction(1, MOTOR_BRAKE);
        motor1_dir = MOTOR_BRAKE;
        if (motor1_pwm != RT_NULL) {
            rt_pwm_enable(motor1_pwm, MOTOR_PWM_CHANNEL);
        }
    } else if (motor_id == 2) {
        motor_set_duty(2, 0);
        motor_set_direction(2, MOTOR_BRAKE);
        motor2_dir = MOTOR_BRAKE;
        if (motor2_pwm != RT_NULL) {
            rt_pwm_enable(motor2_pwm, MOTOR_PWM_CHANNEL);
        }
    }

    rt_pin_write(MOTOR_STBY_PIN, PIN_HIGH);
}

/* ============================================================================
 *  motor_set_duty - 设置电机 PWM 占空比
 * ============================================================================
 *  duty: 0~4000（对应 PWM 周期 4000）
 */
void motor_set_duty(rt_uint8_t motor_id, rt_uint32_t duty)
{
    rt_uint32_t pulse;

    if (duty > MOTOR_PWM_DUTY_MAX) {
        duty = MOTOR_PWM_DUTY_MAX;
    }

    /* [PORT] 源工程直接把 duty 写进比较寄存器；rt_pwm_set() 收的是纳秒，
     * 所以按 MOTOR_PWM_DUTY_MAX 等比换算成脉宽。
     * 4000/4000 → 200000ns = 整个周期 = 100%。 */
    pulse = (rt_uint32_t)((rt_uint64_t)duty * MOTOR_PWM_PERIOD_NS / MOTOR_PWM_DUTY_MAX);

    if (motor_id == 1 && motor1_pwm != RT_NULL) {
        rt_pwm_set(motor1_pwm, MOTOR_PWM_CHANNEL, MOTOR_PWM_PERIOD_NS, pulse);
    }
    if (motor_id == 2 && motor2_pwm != RT_NULL) {
        rt_pwm_set(motor2_pwm, MOTOR_PWM_CHANNEL, MOTOR_PWM_PERIOD_NS, pulse);
    }
}

/* ============================================================================
 *  motor_set_direction - 设置电机 H桥方向
 * ============================================================================
 *  direction:
 *    MOTOR_BRAKE   (0): AIN1=H, AIN2=H（刹车）
 *    MOTOR_FORWARD (1): AIN1=H, AIN2=L（正转）
 *    MOTOR_REVERSE (2): AIN1=L, AIN2=H（反转）
 *
 *  注意：请使用 motor_update_direction() 而非直接调用此函数，
 *        以便获得死区保护。
 */
void motor_set_direction(rt_uint8_t motor_id, rt_uint8_t direction)
{
    if (motor_id == 1) {
        if (direction == MOTOR_BRAKE) {
            rt_pin_write(MOTOR1_AIN1_PIN, PIN_HIGH);
            rt_pin_write(MOTOR1_AIN2_PIN, PIN_HIGH);
        } else if (direction == MOTOR_FORWARD) {
            rt_pin_write(MOTOR1_AIN1_PIN, PIN_HIGH);
            rt_pin_write(MOTOR1_AIN2_PIN, PIN_LOW);
        } else if (direction == MOTOR_REVERSE) {
            rt_pin_write(MOTOR1_AIN1_PIN, PIN_LOW);
            rt_pin_write(MOTOR1_AIN2_PIN, PIN_HIGH);
        }
    }

    if (motor_id == 2) {
        if (direction == MOTOR_BRAKE) {
            rt_pin_write(MOTOR2_BIN1_PIN, PIN_HIGH);
            rt_pin_write(MOTOR2_BIN2_PIN, PIN_HIGH);
        } else if (direction == MOTOR_FORWARD) {
            rt_pin_write(MOTOR2_BIN1_PIN, PIN_HIGH);
            rt_pin_write(MOTOR2_BIN2_PIN, PIN_LOW);
        } else if (direction == MOTOR_REVERSE) {
            rt_pin_write(MOTOR2_BIN1_PIN, PIN_LOW);
            rt_pin_write(MOTOR2_BIN2_PIN, PIN_HIGH);
        }
    }
}

/* ============================================================================
 *  PWM 占空比状态（0~MOTOR_PWM_DUTY_MAX）
 * ============================================================================
 *  和下面那几个 PID 状态变量本来是堆在一起的，单独提到这里是因为
 *  motor_update_direction() 换向时要先把它清零，而 C 要求先声明后使用。
 *  谁写、什么时候清，见下面的 motor_switch_dir()。
 */
static rt_uint16_t PWM_1_duty = 0;
static rt_uint16_t PWM_2_duty = 0;

/* ============================================================================
 *  motor_switch_dir - 换向：先卸力，再翻桥
 * ============================================================================
 *  [PORT] 新增。源工程里这段是 car.c 的 Car_SetDir()/flip()，移植时按
 *  "car.c 只设目标转速、占空比和 H 桥方向全部归 PID 线程"的决策删掉了。
 *  删掉的是【重复的写入者】—— 卸力本身是必需的，所以在这里以唯一写入者的
 *  身份补回来：
 *
 *    不清零占空比就翻桥，轮子还在按旧方向转、PWM 还挂在旧占空比上，
 *    AIN1/AIN2 一翻就是反接 —— 电流冲击 + 机械顿挫。DIR_DEADBAND 只防
 *    "目标在 0 附近来回抖动"，防不了这个：命令级的符号翻转（+800 → -800）
 *    是一步到位的，根本不经过死区。
 *
 *  顺带修掉同根因的第二个现象：从刹车再起步本来是"0 → 上一轮残留的占空比"
 *  一步跳变（车窜一下）。现在每次 BRAKE→FORWARD 都走这里，改从 0/起步补偿
 *  重新爬。
 *
 *  顺序是先写占空比、再翻方向。rt_pwm_set() 写的是带缓冲的比较值，要等下一次
 *  reload（周期 200us）才真正生效，而 GPIO 是立即生效的，所以严格说翻桥瞬间
 *  还残留最多一个 PWM 周期的旧占空比。200us 远小于电机绕组的电气时间常数，
 *  电流来不及建立 —— 和原来残留整整一个 50ms 周期不是一回事。
 */
static void motor_switch_dir(rt_uint8_t motor_id, rt_uint8_t *curr_dir, rt_uint8_t dir)
{
    if (motor_id == 1) {
        PWM_1_duty = 0;
    } else {
        PWM_2_duty = 0;
    }
    motor_set_duty(motor_id, 0);

    *curr_dir = dir;
    motor_set_direction(motor_id, dir);
}

/* ============================================================================
 *  motor_update_direction - 带死区滞后的方向更新
 * ============================================================================
 *  仅在目标速度穿越死区阈值时才切换方向，防止速度在0附近波动时
 *  方向频繁翻转。
 *
 *  死区逻辑:
 *    当前 FORWARD  → 目标 < -DEADBAND 才切换到 REVERSE
 *    当前 REVERSE  → 目标 > +DEADBAND 才切换到 FORWARD
 *    目标在死区内 → 刹车（或保持当前方向）
 *
 *  真正的方向写入都走 motor_switch_dir()，那边会顺带卸力（见其注释）。
 *  这里只判断"该不该换"，不直接碰 H 桥。
 */
void motor_update_direction(rt_uint8_t motor_id)
{
    float    target_speed;
    rt_uint8_t *curr_dir;

    if (motor_id == 1) {
        target_speed = target_speed_1;
        curr_dir     = (rt_uint8_t *)&motor1_dir;
    } else {
        target_speed = target_speed_2;
        curr_dir     = (rt_uint8_t *)&motor2_dir;
    }

    if (target_speed > DIR_DEADBAND) {
        /* 目标正转 —— 仅当前非正转时切换 */
        if (*curr_dir != MOTOR_FORWARD) {
            motor_switch_dir(motor_id, curr_dir, MOTOR_FORWARD);
        }
    } else if (target_speed < -DIR_DEADBAND) {
        /* 目标反转 —— 仅当前非反转时切换 */
        if (*curr_dir != MOTOR_REVERSE) {
            motor_switch_dir(motor_id, curr_dir, MOTOR_REVERSE);
        }
    } else {
        /* 死区内 —— 刹车 */
        if (*curr_dir != MOTOR_BRAKE) {
            motor_switch_dir(motor_id, curr_dir, MOTOR_BRAKE);
        }
    }
}

/* ============================================================================
 *  calculate_speed - 编码器测速
 * ============================================================================
 *  公式: speed = (counts / Encoder) * PI * Diameter * SAMPLE_RATE
 *        = counts / 440 * 3.14 * 67 * 50
 *
 *  读取后清零计数器，准备下一个周期的测量。
 *
 *  刻度近似物理量：1 个计数 / 50ms ≈ 9.6 mm/s（1 个刻度 ≈ 0.40 mm/s）。与真
 *  mm/s 差的 2.5 倍来自 SPEED_SAMPLE_RATE 取 50（而实际周期是 50ms），见
 *  motor.h 差异 2。
 *  「先读后清」之间如果来一个边沿会被漏掉，最多丢 1 个计数（默认档 400 是
 *  17 个计数，等于约 6% 的读数）；源工程写法相同，这里不改。
 */
void calculate_speed(rt_uint8_t motor_id)
{
    if (motor_id == 1) {
        speed_1 = (float)counter_1_A / Encoder * PI * MOTOR_WHEEL_D * SPEED_SAMPLE_RATE;
        odo_1  += (rt_int32_t)counter_1_A;      /* 累加必须在清零之前，见 odo_1 的注释 */
        counter_1_A = 0;
    }
    if (motor_id == 2) {
        speed_2 = (float)counter_1_B / Encoder * PI * MOTOR_WHEEL_D * SPEED_SAMPLE_RATE;
        odo_2  += (rt_int32_t)counter_1_B;
        counter_1_B = 0;
    }
}

/* ============================================================================
 *  速度 PID 参数
 * ============================================================================
 */
/* --------------------------------------------------------- 【这三个数的来历】----
 *  【第一层：它们比刻度慢了一代】原值是 0.08 / 0.02 / 0.05，整定于
 *  1dc5fd8e【之前】那套刻度（motor.h 的 Encoder 13，1 个计数 = 809 刻度）。
 *  那条提交把 Encoder 改成 440（1 个计数 = 24 刻度）—— 快慢差 809/24 ≈ 33.7
 *  倍 —— 但【没同步动增益】，自己把它记成了「已知并接受」的遗留项。
 *
 *  【第二层：×33.7 一次补到位是错的，实测把车打飞了】
 *  2026-10-13 先按刻度因子把三个增益乘了 33.7（2.7 / 0.67 / 1.7），
 *  电池供电实测（无线串口回推的 spd/duty 那串日志）：
 *
 *      W 之后第一拍：duty 0 → 3760
 *          前馈给 2000，PID 又叠了 kp*e + kd*ė = 2.7*400 + 1.7*400 = 1760
 *      W 之后第二拍：顶到 MOTOR_PWM_DUTY_MAX(4000)
 *      转速冲到 30 / 27     ← 目标是 17 个计数，超调 76%
 *      占空比崩到 724，来回摆约 600ms 才稳
 *
 *  ⚠ 为什么"按刻度乘"这件事本身不成立，两个原因：
 *   1. 【工作点也搬了】刻度因子只管数字换算，可默认档同时从 2400（= 2.97 个
 *      计数）搬到了 400（= 16.7 个计数），物理速度高了约 5.6 倍。两个因素方向
 *      相反，净效果远小于 33.7 —— 拿它当唯一依据是错的。
 *   2. 【起步那一段被"测量死时间"主导】前馈把占空比放到工作点之后，电机还
 *      没转起来，speed 读数仍然是 0，于是这一拍 error 就是满的 400。kp 再大
 *      也只是在对着一个还没更新的读数猛打 —— 越猛越糟，跟环路刚度无关。
 *
 *  【现在这组数怎么来的】目标是让 kp 只做"微调"，起步交给前馈：
 *    - kp = 0.5：起步那一拍的 P 项是 0.5*400 = 200，只有前馈 2000 的 10%，
 *      不会再把它顶到限幅；同时稳态下 1 个计数（24 刻度）的误差仍能给出
 *      0.5*24 = 12 占空比/拍，够压住量化抖动。
 *    - ki = 0.12：**保持和 kp 的比值约 4**（老的 0.08/0.02 也是 4）。
 *      这个比值就是积分时间常数（约 4 拍 = 0.2s），换刻度不该动它。
 *    - kd = 0.2：0→400 的阶跃从"误差微分"上贡献 0.2*400 = 80，不再是 680。
 *      老值 0.05 按比值放大约该是 0.2 —— 正好。
 *
 *  ⚠ 仍然是【起步值，不是整定值】。上台架用 msh 的 `motor pid <kp> <ki> <kd>`
 *  扫（运行期改，不用重烧），看 `motor status`：起转那一两拍不顶到 4000、
 *  转速不冲过目标、稳态不来回抽，就是合适的。别把这几个数当定论。
 */
static float motor_kp = 0.5f;
static float motor_ki = 0.12f;
static float motor_kd = 0.2f;

/* 积分限幅，作用在 integral_x（【误差的累加值】，量纲和 error 相同）上。
 *
 * 【它必须跟着刻度一起缩】原来的 ±2000 是老刻度下的数：同一段物理过程，
 * 新刻度下累加出来的 integral_x 小 33.7 倍。按"和老刻度等效"算，
 * 2000/33.7 ≈ 59。
 *
 * 【但这里取 150，比等效值宽裕】因为稳态下积分要顶住
 *     ki*integral = -kp*e     ⇒     integral = (kp/ki) * e
 * 也就是 I 的稳态需求正比于 kp/ki（现在是 4）乘稳态误差。误差半个计数是 12，
 * 算出 I ≈ 50 —— 贴着 59 的限幅，一旦稳态误差大一点（比如一轮负载重、差 1.5
 * 个计数）积分就顶死、再也压不下去。150 让这套增益有 3 倍余量，
 * 而 ki*integral 的上限变成 0.12*150 = 18 占空比/拍，仍然是安全的量级
 * （对比：够不上一拍把占空比从 500 打到 4000 的那种失控）。 */
#define MOTOR_I_MAX             150.0f

/* 起步前馈：目标速度 → 一开始就给的占空比。见 motor_ff_duty()。
 *
 * PER_UNIT 是【两路各自】的上电初值（运行期用 `motor ff` 分开改，
 * 理由见 motor_ff_duty()）。FF_DUTY_MAX 是前馈自己的上限，比
 * MOTOR_PWM_DUTY_MAX 低得多 —— 前馈是"起点"不是"工作点"，也见那边。 */
#define MOTOR_FF_DUTY_PER_UNIT  5.0f
#define MOTOR_FF_DUTY_MIN       500
#define MOTOR_FF_DUTY_MAX       3000

/* PID 状态
 * [PORT] 源工程里 PWM_x_duty / target_speed_x 是不带 static 的全局变量，
 *        供外部直接读写。这里收成文件内 static，对外用 motor_set_target()
 *        和 motor_get_duty() 访问，避免别处随手改 PID 的内部状态。
 *        PWM_1_duty / PWM_2_duty 不在这里 —— 它们在 motor_update_direction()
 *        之前定义，因为换向时 motor_switch_dir() 要先把占空比卸掉。 */
float    target_speed_1 = 0.0f;
float    target_speed_2 = 0.0f;

/* ============================================================================
 *  motor_ff_duty - 起步前馈：按目标速度给一个起步占空比
 * ============================================================================
 *  返回【这一路】电机在 |目标速度| 下的起步占空比，夹在 [MOTOR_FF_DUTY_MIN,
 *  MOTOR_FF_DUTY_MAX] 之间。系数按电机分开，运行期可改（`motor ff`）。
 *
 *  ------------------------------------------------ 【这两个数是架空的】----
 *  2026-10-13 电池供电（7.49V）、车【架空】的实测：
 *
 *    目标 400（17 个计数）时
 *        左轮：占空比 ~680 才起转，稳态工作点 ~1680
 *        右轮：占空比 ~1816 才起转，稳态工作点 ~2038
 *
 *  ⚠ 【车架空量的数，落地不能直接用】轮子悬空只要克服自己的轴承/减速箱摩擦，
 *  落地还要克服静摩擦和车重。同一辆车落地实测：占空比已经给到 2280，两轮
 *  【都还没转】（遥控日志里连着两拍 "spd 0 0 duty 2280 ..."）。也就是说落地
 *  挣脱点比上面那两个数高一截，而 400×5.0 = 2000 这个"起点"是【不够】的 ——
 *  起步还是要靠 PID 往上爬一两百才真的转动，爬的这一两百就是起步滞后的来源，
 *  两轮挣脱点不同，滞后就不同，车出去就是一道弧。
 *
 *  ---------------------------------------- 【为什么必须按电机分开】----
 *  两轮曲线差一大截（架空挣脱点 680 vs 1816，稳态工作点 1680 vs 2038），这是
 *  两台电机的机械差异（右轮那台摩擦大/偏弱），不是谁算错了。闭环能把【稳态】
 *  工作点各自收敛过去（工作点不同、转速相同），但【起步那一拍是开环的】：
 *  前馈给多少就是多少，PID 要到第二拍才开始修。
 *
 *  于是同一个系数、同一个目标，给出来的占空比离两轮各自的挣脱点是【不同的
 *  距离】：2000 对左轮（挣脱 680）是多推 1300 多，对右轮（挣脱 1816）只多推
 *  不到 200。落地实测就是起步那一秒左轮最多领先 4 个编码器计数（24%），
 *  车往右画一道弧。这道弧【外环修不了】：20ms 拍的外环 + 模块 10Hz 出帧 +
 *  50ms 的内环，等它看见偏的时候弧已经画完了（car.h 的 CAR_HEAD_PERIOD_MS）。
 *
 *  --------------------------------------------- 【这两个数怎么标定】----
 *  落地、电池供电、车别架空：
 *    1. 用手按住车（或者让它顶着墙），按 W，盯无线串口周期行的 duty 列；
 *    2. spd 从 0 跳起来那一拍的 duty 就是【这一路的落地挣脱点】；
 *    3. 系数 = 落地挣脱点 × 1.1 / CAR_SPEED_RUN，无线串口上
 *       `#motor ff 1 <左系数>` / `#motor ff 2 <右系数>` 填进去，不用重烧；
 *    4. 验收：起步那 1 秒里 spd 两列的差从 4 个计数降到 1 个以内。
 *  定下来之后再把数填回 MOTOR_FF_DUTY_PER_UNIT 附近（或者就留着运行期设）。
 *
 *  【为什么做成运行期可改】标定要试好几个数，写回宏再烧一次试一个数太慢；
 *  而且整定必须落地跑，落地就得电池供电 —— `motor` 的 msh 命令却要插 USB
 *  （一插电机供电就塌，见 car.h）。所以它和 kp/ki/kd 一样走运行期，并且经
 *  无线串口的 `#motor ff` 可达。
 *
 *  【改速度旋钮之后要重标吗】要。"占空比 = 系数 × 目标速度"是直线近似，
 *  只在标定过的那一档附近准。默认档改到远离 400 就得重来一遍。
 *
 *  【为什么不用固定值】原来这里给的是固定 500 —— 那个值【两轮都够不着自己的
 *  挣脱点】，于是两轮都得靠增量 PID 每拍 0.08*error ≈ 32 的爬坡慢慢蹭上去，
 *  谁先蹭到谁先转：实测左轮 1.4 秒起转、右轮 5.3 秒起转，中间 4.2 秒只有
 *  左轮在转，车出去就是一道弧。换成按目标算的前馈之后，这 4.2 秒被压到
 *  接近 0，而占空比本身没变（稳态还是各回各的工作点）。
 *
 *  -------------------------------------------------- 【为什么要有上限】----
 *  前馈是【起点】，只需要"刚好越过挣脱点"，剩下的交给 PID 从工作点往上爬。
 *  没有上限的话 `bt speed 800` 会算出 4000 = 满油门起步（CAR_SPEED_MAX 是
 *  1200，README 还把 `bt speed` 当整定旋钮推荐），什么车都得窜出去。
 *  MOTOR_FF_DUTY_MAX(3000) 在系数 5.0 下对应目标 600，默认档 400（2000）上
 *  不参与。
 *
 *  ⚠ 两个边界一起看：
 *    - 上限【不能压到落地挣脱点以下】，那样前馈就白给了，起步退回成"靠 PID
 *      慢慢爬"。所以落地标定完之后，如果两路系数里有哪一路把结果顶到了上限，
 *      说明上限该抬了。
 *    - 上限顺带决定了同一档位上系数最多能调到多少：
 *      系数上限 = MOTOR_FF_DUTY_MAX / 目标速度（默认档 400 上就是 7.5）。
 *
 *  【为什么不用"把 kp 调大"代替它】试过，实测把车打飞（超调 76%、撞限幅），
 *  原因和两者的分工见 DC_MOTOR_PID() 头部「起步为什么必须前馈」那一段。
 * ========================================================================== */

/* 前馈系数，按电机分开。上电初值都是 MOTOR_FF_DUTY_PER_UNIT，所以
 * 【没标定之前的行为和只有一个系数时完全一样】—— 这里只把入口留出来，
 * 不预先塞一个没量过的值进去。 */
static float motor_ff_per_unit_1 = MOTOR_FF_DUTY_PER_UNIT;
static float motor_ff_per_unit_2 = MOTOR_FF_DUTY_PER_UNIT;

/* 和 motor_target_ptr() 同一套路。不合并成数组是因为这个文件里
 * target/speed/duty/前馈各自的取值方式都长这样。 */
static float *motor_ff_ptr(rt_uint8_t motor_id)
{
    return (motor_id == 1) ? &motor_ff_per_unit_1 : &motor_ff_per_unit_2;
}

static rt_uint16_t motor_ff_duty(rt_uint8_t motor_id, float abs_target)
{
    float d = abs_target * *motor_ff_ptr(motor_id);

    if (d < (float)MOTOR_FF_DUTY_MIN)  d = (float)MOTOR_FF_DUTY_MIN;
    if (d > (float)MOTOR_FF_DUTY_MAX)  d = (float)MOTOR_FF_DUTY_MAX;

    return (rt_uint16_t)d;
}
static float last_error_1 = 0.0f;
static float last_error_2 = 0.0f;
static float integral_1   = 0.0f;
static float integral_2   = 0.0f;

/* 增量式输出那一拍【没凑够 1】的零头，留到下一拍一起加。
 * 取值恒在 (-1, 1) 内：每拍先累加、再把整数部分拿出去、剩下的留在原地，
 * 所以它既不会漂也不会积起来（对比 integral_x，那个才需要单独限幅）。
 * 为什么要有它，见 DC_MOTOR_PID() 的注释。 */
static float duty_frac_1   = 0.0f;
static float duty_frac_2   = 0.0f;

/* ============================================================================
 *  DC_MOTOR_PID - 单电机速度闭环 PID
 * ============================================================================
 *  特性:
 *    - 起步前馈: 目标速度>10 且占空比正好为 0（＝刚从刹车/换向起步）时，
 *      直接把占空比顶到 motor_ff_duty() 算出的工作点附近
 *    - 积分条件: 仅当误差 < 目标速度20%时开启积分（防止启动阶段积分饱和）
 *    - 积分抗饱和: +/-MOTOR_I_MAX(150) 限幅 —— 跟着刻度缩过又留了余量，见该宏
 *    - 直接增量式输出: 在当前占空比上累加 PID 输出
 *
 * ------------------------------------------- 【零头去哪了】（duty_frac_x）----
 *  这是修「两轮同目标却跑出不同转速」的那一处。
 *
 *  PID 输出 pid_out 是浮点，而占空比是整数，原来那句
 *      duty_change = (int16_t)pid_out;
 *  是【朝零截断】：只要 |pid_out| < 1，就整个丢成 0，占空比一动不动。
 *  增量式结构下这会形成一个【停摆死区】—— 回路在误差还没归零的地方就
 *  停手了，而 PID 的输出本来是"误差越小输出越小"，所以它必然在误差
 *  某处停住而不是收敛到 0。
 *
 *  停在哪，取决于这台电机的负载曲线；两轮各自的停点不一样，于是
 *  【同一个目标、两个不同的稳态转速】。刻度下 1 个计数 = 24，
 *  停摆点相差半个计数就够让两轮读数差 1（默认档 400 上是 6%）——
 *  实测到的 17 / 16 正是这个量级。跟 kp/ki/kd 调多大都无关：
 *  增益只影响"多快走到停摆点"，改变不了"会在哪儿停"。
 *
 *  改法是把这个零头记在 duty_frac_x 里，下一拍一起算：
 *      duty_frac += pid_out;  duty_change = (int16_t)duty_frac;  duty_frac -= duty_change;
 *  于是一拍只出 0.1 的微调也不会被丢掉，攒够 1 就动一下 —— 死区没了，
 *  回路能一路把误差压到零附近，两轮各自都会去咬【自己的目标】而不是
 *  "就近停住"。读数上会看到速度在两个相邻计数之间缓慢来回蹭，
 *  那是刻度 24 带来的量化（1 个计数 = 6%），平均值才是对的；
 *  两轮的平均值相同，车才走得直。
 *
 *  注意 duty_frac_x 【不需要】抗饱和：每拍都是"取整 + 留余"，它的取值
 *  恒在 (-1, 1) 内，攒不起来。占空比顶到 4000 时的抗饱和仍由 integral_x
 *  的限幅和后面那句 MOTOR_PWM_DUTY_MAX 截断负责。
 *
 * --------------------------------------------- 【起步为什么必须前馈】----
 *  这是修「发 W 之后车先画一道弧」的那一处，跟上面那个零头是两个独立的病。
 *
 *  增量式结构下，占空比的爬坡速度【只】由 kp*error 决定。目标 400（17 个
 *  计数）时，老的 kp 是 0.08，也就是 0.08*400 ≈ 32/拍 —— 5 秒才把占空比
 *  从 500 推到 3500。而实测两轮的【挣脱】占空比差了 1100 多（左 680 /
 *  右 1816，见 motor_ff_duty() 那张表），于是这个"占空比之差"被翻译成了
 *  "时间之差"：左轮 1.4 秒起转，右轮 5.3 秒，中间 4.2 秒只有左轮在转。
 *
 *  【那为什么不干脆把 kp 调大】试过了，实测把车打飞：kp 按刻度乘到 2.7 时，
 *  起步那一拍 P 项就是 2.7*400 = 1080，占空比被从 2000 顶到 3760、下一拍撞
 *  限幅，转速冲到 30 个计数（目标 17，超调 76%）再崩回来，来回摆约 600ms。
 *  原因见上面那一大段的第 2 点：起步这一拍 speed 读数还是 0，误差是"满"的，
 *  kp 再大也只是对着一个还没更新的数字猛打。
 *  所以分工是明确的：**起点交给前馈（按目标算，一次到位）**，
 *  **kp 只负责微调（现在的 0.5，起步 P 项只有 200）**。
 *  上台架的标准：起转不顶限幅、转速不冲过目标、两轮同时起转。
 *
 *  另外它还顺带压掉了启动阶段的积分饱和：原来要爬 5 秒，这 5 秒里
 *  误差一直很大（> 20% 目标），积分被反复清零；等转速冲过目标线，
 *  积分才刚刚开始攒，于是要先超调一段再慢慢退回来（实测转速冲到 19 个
 *  计数、目标 17）。前馈把起步段缩短到一拍，这段超调也就没了。
 */
void DC_MOTOR_PID(rt_uint8_t motor_id)
{
    float    error;
    float    abs_target;
    float    derivative;
    float    pid_out;
    int16_t  duty_change;

    if (motor_id == 1)
    {
        abs_target = (target_speed_1 > 0.0f) ? target_speed_1 : -target_speed_1;

        /* 起步前馈（原来这里是固定给 500，见 motor_ff_duty() 的标定说明）。
         *
         * 条件写「占空比正好是 0」而不是「小于某个值」：只有换向/从刹车起步
         * 那一刻 motor_switch_dir() 会把它清零（它【必须】清 —— 不清零就翻桥
         * 等于带载反接，见那里的注释），所以这一句实际是【一次性】的。
         * 写成「< 某值」的话它会变成一个地板：占空比被压在下面降不下来，
         * 而两轮的工作点本来就不一样（1680 / 2038），地板会把低的那一路顶高。 */
        if (abs_target > 10.0f && PWM_1_duty == 0) {
            PWM_1_duty = motor_ff_duty(1, abs_target);
        }

        error = abs_target - speed_1;

        /*
         * 积分条件放宽到20%（原10%太严，可能导致稳态误差无法消除）
         * 误差 > 20% 目标速度时积分清零（防饱和）
         */
        if ((error > 0.0f ? error : -error) < (abs_target * 0.2f)) {
            integral_1 += error;
        } else {
            integral_1 = 0.0f;
        }

        /* 积分抗饱和（限幅跟着刻度缩过，见 MOTOR_I_MAX） */
        if (integral_1 >  MOTOR_I_MAX) integral_1 =  MOTOR_I_MAX;
        if (integral_1 < -MOTOR_I_MAX) integral_1 = -MOTOR_I_MAX;

        /* 微分 —— 误差变化率 */
        derivative   = error - last_error_1;
        last_error_1 = error;

        /* PID 合成 */
        pid_out = motor_kp * error + motor_ki * integral_1 + motor_kd * derivative;

        /* 增量式输出（在现有占空比上加减）—— 零头带到下一拍，见函数头 */
        duty_frac_1  += pid_out;
        duty_change   = (int16_t)duty_frac_1;
        duty_frac_1  -= (float)duty_change;
        if (duty_change < -((int16_t)PWM_1_duty))
            PWM_1_duty = 0;
        else
            PWM_1_duty = (rt_uint16_t)((int16_t)PWM_1_duty + duty_change);

        if (PWM_1_duty > MOTOR_PWM_DUTY_MAX) PWM_1_duty = MOTOR_PWM_DUTY_MAX;

        motor_set_duty(motor_id, PWM_1_duty);
    }

    if (motor_id == 2)
    {
        abs_target = (target_speed_2 > 0.0f) ? target_speed_2 : -target_speed_2;

        /* 起步前馈 —— 同 motor 1，见上面那段和 motor_ff_duty() */
        if (abs_target > 10.0f && PWM_2_duty == 0) {
            PWM_2_duty = motor_ff_duty(2, abs_target);
        }

        error = abs_target - speed_2;

        /* 积分条件：误差 < 20% 目标速度 */
        if ((error > 0.0f ? error : -error) < (abs_target * 0.2f)) {
            integral_2 += error;
        } else {
            integral_2 = 0.0f;
        }

        /* 积分抗饱和（限幅跟着刻度缩过，见 MOTOR_I_MAX） */
        if (integral_2 >  MOTOR_I_MAX) integral_2 =  MOTOR_I_MAX;
        if (integral_2 < -MOTOR_I_MAX) integral_2 = -MOTOR_I_MAX;

        /* 微分 */
        derivative   = error - last_error_2;
        last_error_2 = error;

        /* PID 合成 */
        pid_out = motor_kp * error + motor_ki * integral_2 + motor_kd * derivative;

        /* 增量式输出 —— 零头带到下一拍，见函数头 */
        duty_frac_2  += pid_out;
        duty_change   = (int16_t)duty_frac_2;
        duty_frac_2  -= (float)duty_change;
        if (duty_change < -((int16_t)PWM_2_duty))
            PWM_2_duty = 0;
        else
            PWM_2_duty = (rt_uint16_t)((int16_t)PWM_2_duty + duty_change);

        if (PWM_2_duty > MOTOR_PWM_DUTY_MAX) PWM_2_duty = MOTOR_PWM_DUTY_MAX;

        motor_set_duty(motor_id, PWM_2_duty);
    }
}

/* ============================================================================
 *  PID 节拍线程（对应源工程的 MOTOR_PID_INST_IRQHandler）
 * ============================================================================
 *  执行顺序（照抄源工程）:
 *    1. 编码器测速
 *    2. 带死区的方向更新
 *    3. 速度 PID 闭环
 *
 *  [PORT] 源工程是 50ms 硬件定时器中断，这里换成线程 + rt_thread_mdelay(50)。
 *         用线程而不是 rt_timer 回调：回调跑在定时器线程里，里面调
 *         DC_MOTOR_PID() 会连带调 rt_pwm_set()/rt_pin_write()，
 *         这类操作不该占着定时器线程（源工程注释里也提醒过中断里别做阻塞事）。
 *         mdelay 的周期会有 1 个 tick 的执行时间偏移，且测速公式用的是
 *         常数分母，所以偏移会变成速度读数上的少量噪声。
 */
static void motor_pid_thread(void *parameter)
{
    RT_UNUSED(parameter);

    while (1)
    {
        calculate_speed(1);
        calculate_speed(2);
        motor_update_direction(1);
        motor_update_direction(2);
        DC_MOTOR_PID(1);
        DC_MOTOR_PID(2);

        rt_thread_mdelay(MOTOR_PID_PERIOD_MS);
    }
}

/* ============================================================================
 *  对外的小接口（[PORT] 新增，源工程是外部代码直接改全局变量）
 * ============================================================================
 */
static float *motor_target_ptr(rt_uint8_t motor_id)
{
    return (motor_id == 1) ? &target_speed_1 : &target_speed_2;
}

void motor_set_target(rt_uint8_t motor_id, float speed)
{
    *motor_target_ptr(motor_id) = speed;
}

float motor_get_target(rt_uint8_t motor_id)
{
    return *motor_target_ptr(motor_id);
}

float motor_get_speed(rt_uint8_t motor_id)
{
    return (motor_id == 1) ? speed_1 : speed_2;
}

/* 自本次上电以来的累计编码器计数（只增不减，正反转都算正数 —— 硬件只给幅值）。
 * 见 odo_1 那段注释：距离的唯一可信来源，别拿 motor_get_speed() 积分。 */
rt_int32_t motor_get_odo(rt_uint8_t motor_id)
{
    return (motor_id == 1) ? odo_1 : odo_2;
}

rt_uint32_t motor_get_duty(rt_uint8_t motor_id)
{
    return (motor_id == 1) ? PWM_1_duty : PWM_2_duty;
}

/* 立刻停：目标归零 + 占空比清零 + 刹车。
 * 不靠 PID 慢慢把占空比降下来，msh 里喊停就应该马上停。 */
static void motor_stop_all(void)
{
    rt_uint8_t id;

    for (id = 1; id <= 2; id++)
    {
        motor_set_target(id, 0.0f);
    }

    PWM_1_duty = 0;
    PWM_2_duty = 0;
    motor_set_duty(1, 0);
    motor_set_duty(2, 0);
    motor_set_direction(1, MOTOR_BRAKE);
    motor_set_direction(2, MOTOR_BRAKE);
    motor1_dir = MOTOR_BRAKE;
    motor2_dir = MOTOR_BRAKE;
}

/* ============================================================================
 *  上电初始化
 * ============================================================================
 *  源工程由 main.c 在硬件就绪后依次调用 motor_init(1)/motor_init(2) 并启动
 *  PID 定时器，这里合成一个 INIT_APP_EXPORT 的函数：
 *  上电默认两路目标为 0（刹车），等 msh 命令来给目标转速。
 */
static int motor_hw_init(void)
{
    rt_thread_t tid;

    /* PWM：drv_pwm.c 已经用 INIT_DEVICE_EXPORT 注册过设备，
     * INIT_APP_EXPORT 一定在其之后，这里能找到设备。 */
    motor1_pwm = (struct rt_device_pwm *)rt_device_find(MOTOR1_PWM_DEV);
    motor2_pwm = (struct rt_device_pwm *)rt_device_find(MOTOR2_PWM_DEV);
    if (motor1_pwm == RT_NULL || motor2_pwm == RT_NULL)
    {
        /* 多半是 .config 里没开 BSP_USING_PWM1/PWM2，或 pin_mux.c 没配 ALT5。
         * 只报错返回：OLED、IMU 和 msh 照样能用，不影响其余功能的验证。 */
        LOG_E("PWM device not found (%s=%p, %s=%p); check BSP_USING_PWM1/PWM2",
              MOTOR1_PWM_DEV, motor1_pwm, MOTOR2_PWM_DEV, motor2_pwm);
        return -RT_ERROR;
    }

    /* 方向 + STBY：先全部落成输出、STBY 拉低（TB6612 关闭），
     * 等 motor_init() 里占空比归零、方向刹车之后再抬起来。 */
    rt_pin_mode(MOTOR1_AIN1_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(MOTOR1_AIN2_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(MOTOR2_BIN1_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(MOTOR2_BIN2_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(MOTOR_STBY_PIN,  PIN_MODE_OUTPUT);
    rt_pin_write(MOTOR_STBY_PIN, PIN_LOW);

    /* 编码器 A 相：上拉输入 + 上升沿中断。
     * 上拉是为了编码器输出是开漏/霍尔的时候也能有确定电平；
     * 如果是推挽输出，上拉无副作用。 */
    rt_pin_mode(MOTOR1_ENC_PIN, PIN_MODE_INPUT_PULLUP);
    rt_pin_mode(MOTOR2_ENC_PIN, PIN_MODE_INPUT_PULLUP);
    rt_pin_attach_irq(MOTOR1_ENC_PIN, PIN_IRQ_MODE_RISING,
                      motor_encoder_irq, (void *)&counter_1_A);
    rt_pin_attach_irq(MOTOR2_ENC_PIN, PIN_IRQ_MODE_RISING,
                      motor_encoder_irq, (void *)&counter_1_B);
    rt_pin_irq_enable(MOTOR1_ENC_PIN, PIN_IRQ_ENABLE);
    rt_pin_irq_enable(MOTOR2_ENC_PIN, PIN_IRQ_ENABLE);

    motor_init(1);
    motor_init(2);

    tid = rt_thread_create("motor", motor_pid_thread, RT_NULL,
                           MOTOR_PID_THREAD_STACK, MOTOR_PID_THREAD_PRIO, 10);
    if (tid == RT_NULL)
    {
        LOG_E("failed to create the PID thread");
        return -RT_ERROR;
    }
    rt_thread_startup(tid);

    LOG_I("motor ready: pwm1=D11(P3_8), pwm2=D13(P3_10), target 0 (brake)");
    return RT_EOK;
}
INIT_APP_EXPORT(motor_hw_init);

/* ============================================================================
 *  msh 命令
 * ============================================================================
 */
static const char *motor_dir_name(rt_uint8_t dir)
{
    switch (dir)
    {
    case MOTOR_FORWARD: return "FWD";
    case MOTOR_REVERSE: return "REV";
    default:            return "BRK";
    }
}

/* rt_kprintf 是精简版 vsnprintf，不支持 %f；LFLAGS 也没有 -lm，
 * 所以速度和目标值按定点打印。这一手在 jy901s.c 里也是同样的原因。 */
static void motor_print_fixed(float value, int dec)
{
    float mag = (value < 0.0f) ? -value : value;
    rt_uint32_t scale = 1;
    rt_uint32_t total;
    int i;

    for (i = 0; i < dec; i++) scale *= 10;
    total = (rt_uint32_t)(mag * (float)scale + 0.5f);

    if (value < 0.0f && total != 0) {
        rt_kprintf("-");
    }
    rt_kprintf("%u", total / scale);
    if (dec > 0) {
        rt_kprintf(".%0*u", dec, total % scale);
    }
}

/* 两路前馈系数一行。`motor ff` 和 `motor status` 都用它。
 *
 * 【顺带把"这个系数在当前目标下会给出多少起步占空比"算出来】—— 系数够不够，
 * 看的是它有没有越过【落地挣脱点】，光看系数本身看不出来。标定的做法就是拿
 * 这一行里的 duty 和实测的挣脱点对：它应当比挣脱点高一点点。
 * 目标取的是电机当前的目标值（带符号，取绝对值），所以先按 W 再敲 `motor ff`
 * 看到的就是下一次起步真正会给的占空比。 */
static void motor_print_ff(void)
{
    rt_uint8_t id;

    for (id = 1; id <= 2; id++)
    {
        float t   = motor_get_target(id);
        float abs = (t < 0.0f) ? -t : t;

        rt_kprintf("ff motor%u: per-unit=", id);
        motor_print_fixed(*motor_ff_ptr(id), 2);
        rt_kprintf(" -> at target ");
        motor_print_fixed(abs, 0);
        rt_kprintf(": duty ");

        /* 【目标太小的时候前馈根本不给】DC_MOTOR_PID() 的条件是 abs_target > 10；
         * 不在这里挡一下，目标 0 会打出 "duty 500"（被 FF_DUTY_MIN 抬起来的那个
         * 值），看起来像是"目标 0 也会给 500 占空比"。 */
        if (abs > 10.0f) {
            rt_kprintf("%u\n", (unsigned)motor_ff_duty(id, abs));
        } else {
            rt_kprintf("-\n");
        }
    }
    rt_kprintf("(duty per unit of target speed, clamped to %u~%u; 'duty -' means the\n",
               MOTOR_FF_DUTY_MIN, MOTOR_FF_DUTY_MAX);
    rt_kprintf("target is too small for the feed-forward to fire; "
               "motor ff <1|2> <c> to change)\n");
}

static void motor_print_status(void)
{
    rt_uint8_t id;

    for (id = 1; id <= 2; id++)
    {
        rt_kprintf("motor%d: dir=%s target=", id, motor_dir_name(id == 1 ? motor1_dir : motor2_dir));
        motor_print_fixed(motor_get_target(id), 1);
        rt_kprintf(" speed=");
        motor_print_fixed(motor_get_speed(id), 1);
        rt_kprintf(" duty=%u/%u i=", motor_get_duty(id), MOTOR_PWM_DUTY_MAX);
        motor_print_fixed(id == 1 ? integral_1 : integral_2, 1);
        rt_kprintf("/");
        motor_print_fixed(MOTOR_I_MAX, 0);
        rt_kprintf("\n");
    }

    /* 积分一并打出来：它顶在 ±MOTOR_I_MAX 上就是抗饱和在起作用（车速跟不上
     * 目标、或者一直被人按住），那是有用的诊断，不是显示噪音。 */
    rt_kprintf("pid: kp=");
    motor_print_fixed(motor_kp, 3);
    rt_kprintf(" ki=");
    motor_print_fixed(motor_ki, 3);
    rt_kprintf(" kd=");
    motor_print_fixed(motor_kd, 3);
    rt_kprintf("  (motor pid <kp> <ki> <kd> to change)\n");

    /* 前馈也要在这里露一面：它是两路【分开】的，而 `motor pid` 是两路共用，
     * 光看 pid 那行会以为整定了增益就完了。 */
    motor_print_ff();
}

/* 运行期改速度 PID 增益（msh 的 `motor pid`）。
 *
 * 【必须连带清状态】integral_x / last_error_x / duty_frac_x 都是按【旧增益】
 * 攒出来的：integral 的量纲跟着增益走（见 MOTOR_I_MAX 那段的推导），换了增益
 * 之后旧积分立刻就是错的量 —— 不清的话改完那一拍会被一个陈旧的积分项踢一脚。
 * last_error 影响微分项（误差变化率），duty_frac 影响下一拍的占空比零头。
 *
 * 只挡负数：kp/ki/kd 给 0 是有意义的用法（比如把 kp 关掉、只看前馈+积分），
 * 负增益只会把环路变成正反馈，那是纯粹的误输入。 */
static rt_err_t motor_set_pid(float kp, float ki, float kd)
{
    if (kp < 0.0f || ki < 0.0f || kd < 0.0f) {
        return -RT_EINVAL;
    }

    motor_kp = kp;
    motor_ki = ki;
    motor_kd = kd;

    integral_1   = integral_2   = 0.0f;
    last_error_1 = last_error_2 = 0.0f;
    duty_frac_1  = duty_frac_2  = 0.0f;

    return RT_EOK;
}

static void motor_msh_usage(void)
{
    rt_kprintf("usage: motor <1|2> <speed>   set target speed (may be negative)\n");
    rt_kprintf("       motor stop           both motors off (brake)\n");
    rt_kprintf("       motor status         show target / measured / duty / integral / gains\n");
    rt_kprintf("       motor pid <kp> <ki> <kd>   change speed-PID gains at runtime\n");
    rt_kprintf("       motor ff [<1|2>] <c>   launch feed-forward, per motor (no id = both)\n");
    rt_kprintf("speed is in motor.h's Encoder-calibrated scale (~0.4 mm/s per unit)\n");
    rt_kprintf("gains apply to both motors and reset the PID state; "
               "current values are in `motor status`\n");
    rt_kprintf("over the wireless link this goes behind '#', e.g. \"#motor ff 1 6.2\" -- the ff\n");
    rt_kprintf("  coefficient must be calibrated on the ground, where USB cannot\n");
    rt_kprintf("  be plugged in (it collapses the motor supply)\n");
}

/*
 * 解析一个可带符号、可带小数的目标速度。
 *
 * 为什么不用 atof(): 本 BSP 用的是 RT-Thread 自带的 klibc，里面没有
 * atof/strtof，LFLAGS 也没有 -lm。手写这么十几行比引一个 libc 依赖划算。
 * 小数部分只取前 3 位，多的位数只消费不累加（不因此报错）。
 */
static rt_err_t motor_parse_speed(const char *s, float *out)
{
    float value = 0.0f;
    float scale = 0.1f;
    int   neg = 0;
    int   seen_digit = 0;

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
        int fd = 0;

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

/* 【msh 和无线串口共用这一个入口】无线串口上收到 "#motor ..." 时由 speed_report.c
 * 原样转调这里（理由见 motor_ff_duty() 上面那段「为什么做成运行期可改」：
 * 前馈系数必须落地标定，而落地就得电池供电、msh 就插不上 USB）。
 *
 * 转发的是【整个 motor 命令】，不只是 `motor ff` —— 顺带把 `motor stop`
 * 也变成了无线串口可达的急停手段，这是净收益。输出走 rt_kprintf（uart2 控制台），
 * 无线串口那条链路上看不见，所以那边只回一行 ok/err。 */
int Motor_Cmd(int argc, char **argv)
{
    float speed;

    if (argc < 2)
    {
        motor_msh_usage();
        motor_print_status();
        return RT_EOK;
    }

    if (!rt_strcmp(argv[1], "stop"))
    {
        motor_stop_all();
        rt_kprintf("motor: both stopped (target 0, duty 0, brake)\n");
        return RT_EOK;
    }

    if (!rt_strcmp(argv[1], "status"))
    {
        motor_print_status();
        return RT_EOK;
    }

    if (!rt_strcmp(argv[1], "pid"))
    {
        float kp, ki, kd;

        if (argc < 5)
        {
            motor_msh_usage();
            return -RT_EINVAL;
        }

        /* 复用 motor_parse_speed()：它就是"把一串字符盘成一个 float"，
         * 不管语义是速度还是增益，同样绕开 klibc 没有 atof 的问题。 */
        if (motor_parse_speed(argv[2], &kp) != RT_EOK ||
            motor_parse_speed(argv[3], &ki) != RT_EOK ||
            motor_parse_speed(argv[4], &kd) != RT_EOK)
        {
            rt_kprintf("motor: bad gain in '%s %s %s'\n", argv[2], argv[3], argv[4]);
            return -RT_EINVAL;
        }

        if (motor_set_pid(kp, ki, kd) != RT_EOK)
        {
            rt_kprintf("motor: gains must be >= 0\n");
            return -RT_EINVAL;
        }

        rt_kprintf("motor: pid kp=");
        motor_print_fixed(kp, 3);
        rt_kprintf(" ki=");
        motor_print_fixed(ki, 3);
        rt_kprintf(" kd=");
        motor_print_fixed(kd, 3);
        rt_kprintf(" (PID state cleared)\n");
        return RT_EOK;
    }

    if (!rt_strcmp(argv[1], "ff"))
    {
        /* 起步前馈系数（占空比 / 目标速度，见 motor_ff_duty()）。
         * 标定必须落地做（按住车，看 spd 从 0 跳起来那一拍的 duty），所以
         * 这条路也开在无线串口上 —— speed_report.c 把 "motor" 整条转进来。 */
        rt_uint8_t id   = 0;
        int        argi = 2;
        float      v;

        if (argc < 3)
        {
            motor_print_ff();
            return RT_EOK;
        }

        /* 【id 只在 argc >= 4 时才认】否则 "motor ff 2"（想两路都设成 2）会被
         * 当成"给电机 2 设系数"，然后因为缺参数报错。想两路都设 2 就写
         * "motor ff 2.0" —— 小数点是区分标志。 */
        if (argc >= 4 &&
            (argv[2][0] == '1' || argv[2][0] == '2') && argv[2][1] == '\0')
        {
            id   = (rt_uint8_t)(argv[2][0] - '0');
            argi = 3;
        }

        /* 复用 motor_parse_speed()：它就是"把一串字符盘成一个 float"，
         * 和上面 pid 分支同一个理由。系数不带符号 —— 前馈只在起转那一刻给
         * 一次占空比，方向由 motor_update_direction() 按目标符号决定。 */
        if (motor_parse_speed(argv[argi], &v) != RT_EOK || v <= 0.0f || v > 100.0f)
        {
            rt_kprintf("motor: bad ff coefficient '%s' (must be 0 < c <= 100)\n",
                       argv[argi]);
            return -RT_EINVAL;
        }

        if (id == 0)
        {
            motor_ff_per_unit_1 = v;
            motor_ff_per_unit_2 = v;
        }
        else
        {
            *motor_ff_ptr(id) = v;
        }

        /* 【不用清 PID 状态】系数只在"占空比正好为 0 的起转那一拍"被读一次
         * （DC_MOTOR_PID 里那句），改它不影响已经在跑的那一轮 —— 和
         * motor_set_pid() 必须清状态是两回事。 */
        motor_print_ff();
        return RT_EOK;
    }

    if ((argv[1][0] == '1' || argv[1][0] == '2') && argv[1][1] == '\0')
    {
        rt_uint8_t id = (rt_uint8_t)(argv[1][0] - '0');

        if (argc < 3)
        {
            motor_msh_usage();
            return -RT_EINVAL;
        }

        if (motor_parse_speed(argv[2], &speed) != RT_EOK)
        {
            rt_kprintf("motor: bad speed '%s'\n", argv[2]);
            return -RT_EINVAL;
        }

        motor_set_target(id, speed);
        rt_kprintf("motor%d: target=", id);
        motor_print_fixed(speed, 1);
        rt_kprintf("\n");
        return RT_EOK;
    }

    motor_msh_usage();
    return -RT_EINVAL;
}
/* msh 侧的包装。和 speed_report.c 的 cmd_spd() → spd_dispatch()、car.c 的
 * cmd_head() → Car_CmdHead() 是同一种做法：一套解析两个入口。 */
static int cmd_motor(int argc, char **argv)
{
    return Motor_Cmd(argc, argv);
}
MSH_CMD_EXPORT_ALIAS(cmd_motor, motor, TB6612 dual motor closed-loop speed control);
