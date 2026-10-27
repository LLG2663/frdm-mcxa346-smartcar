/*
 * SPDX-License-Identifier: MIT
 *
 * TB6612 双电机 + 编码器速度闭环，移植自 CCS 工程 113/user_driver/motor.{c,h}
 * 与 113/user_driver/interrupt.c（TI MSPM0G3507）到 FRDM-MCXA346 上的 RT-Thread。
 *
 * 结构与参数逐条照搬源工程：
 *   方向控制 motor_set_direction()、死区滞后 motor_update_direction()、
 *   测速 calculate_speed()、增量式 PID DC_MOTOR_PID()，算法与增益都没动。
 *   连注释都保留原样。
 *
 * 平台相关的部分换成了 RT-Thread 的接口：
 *   [PORT] DL_GPIO_setPins/clearPins  → rt_pin_write()
 *   [PORT] DL_Timer_setCaptureCompareValue → rt_pwm_set()（PWM 周期纳秒）
 *   [PORT] 编码器 GPIO 中断           → rt_pin_attach_irq()
 *   [PORT] MOTOR_PID_INST_IRQHandler  → 独立线程 + rt_thread_mdelay(50)
 *   [PORT] motor_control_mode 变量删除（见下）
 *
 * ---------------------------------------------------------------- 接线 ----
 * 全部落在 Arduino 排针 J1/J2 上，不用碰 mikroBUS 座：
 *
 *   功能              MCU 脚   Arduino   信号
 *   电机1 方向 AIN1   P3_31    D2        GPIO
 *   电机1 方向 AIN2   P4_7     D4        GPIO
 *   电机2 方向 BIN1   P3_22    D7        GPIO
 *   电机2 方向 BIN2   P4_3     D8        GPIO
 *   STBY（两路共用）  P3_14    D3        GPIO，高电平使能 TB6612
 *   电机1 编码器 A 相 P3_1     D5        GPIO 上升沿中断
 *   电机2 编码器 A 相 P3_17    D6        GPIO 上升沿中断
 *   电机1 调速 PWM    P3_8     D11       PWM0_A1 → 设备 "pwm1"
 *   电机2 调速 PWM    P3_10    D13       PWM0_A2 → 设备 "pwm2"
 *
 * 两路 PWM 的 MUX 在 board/MCUX_Config/board/pin_mux.c 里配好（ALT5）。
 * 注意别对这些脚调 rt_pin_mode()：drv_pin.c 的 mcx_pin_mode() 会把整个
 * PCR 重写成 kPORT_MuxAlt0，PWM 输出会被改回普通 GPIO。
 *
 * 方向/使能/编码器这些 GPIO 脚不用写进 pin_mux.c，rt_pin_mode() 自己设 MUX。
 * 这 7 个脚在 pin_mux.c 里都没被别的外设占用，不会互相踩。
 *
 * ------------------------------------------------------- 与源工程的差异 ----
 * 1. motor_control_mode / MOTOR_MODE_* 已删除。
 *    源工程里那个变量只在 motor.c 里被初始化，全工程没有任何赋值点
 *    （原本由不存在的 gray.c 循迹模块设置），所以 ISR 里的
 *    `if (motor_control_mode == MOTOR_MODE_LINE_FOLLOWING)` 恒为真。
 *    留着只会让人以为还有循迹模式，直接去掉，PID 每拍都跑。
 *    源工程里 target_speed_1/2 同样没有任何赋值点，电机实际一直停在
 *    目标 0（刹车）——目标值现在由 motor_set_target() / msh 命令给。
 *
 * 2. 测速刻度已按实车标定（Encoder 13 → 440，见下面「电机物理参数」），
 *    所以 speed 现在近似物理量：1 个刻度 ≈ 0.40 mm/s
 *    （1 个编码器计数 / 50ms 采样 ≈ 9.6 mm/s）。
 *    差的那一个固定系数 2.5 来自 SPEED_SAMPLE_RATE：公式里写 50
 *    （= 1000ms/20ms），而 PID 的实际采样周期是 50ms（MOTOR_PID_PERIOD_MS），
 *    按实际周期算应该是 20。
 *    【本次仍保持 50】：改成 20 等于把刻度整体再缩放一次，kp/ki/kd 得跟着
 *    重调整定；保持原样则只差这个已知的 2.5 倍，量纲对得上就够用。
 *
 * 3. 采样节拍由硬件定时器中断换成线程 rt_thread_mdelay(50)。
 *    周期同样是 50ms，但有 tick 抖动（1ms 量级）。测速公式里的分母是
 *    常数而不是实测间隔，所以抖动会直接变成速度读数的噪声，
 *    这也是源工程固有的性质。
 *
 * 4. 源工程 motor_init() 的调用顺序（占空比归零 → 方向刹车 → 抬 STBY）
 *    原样保留，理由见 motor.c 里的注释。
 */
#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <rtthread.h>

/* ============================================================================
 *  电机物理参数
 * ============================================================================
 *  Encoder: 编码器线数（每转脉冲数）。源工程写的是 13，与实车不符，已按实测
 *           标定：旧刻度下【目标 2400 直行 10 圈用 74 秒】= 0.135 转/秒，
 *           而 2400 对应 59.3 计数/秒，所以每圈约 439 ≈ 440。
 *           写 13 会让「目标转速」这个数字比实际慢约 34 倍，量化台阶也粗 34 倍。
 *  MOTOR_WHEEL_D: 车速刻度用的标定轮径 (mm)。只进 calculate_speed()，
 *           是源工程沿用的名义值，【不是】实测轮径 —— 别拿它当几何参数。
 *  MOTOR_WHEEL_D_MEAS: 实测轮径 (mm)，只进里程换算。
 *  SPEED_SAMPLE_RATE: 速度采样率。取值是 50（= 1000ms/20ms），但 PID 的实际
 *           周期是 50ms ⇒ 算出来的刻度比真 mm/s 大 2.5 倍（见文件头差异 2）。
 */
#define PI                  3.14f
#define Encoder             440
#define MOTOR_WHEEL_D       67
#define SPEED_SAMPLE_RATE   50      // = 1000ms/20ms，即按 20ms 采样换算；实际周期是 50ms，见差异 2

/* 实测轮径：2026-10-17 卷尺量车轮直径 40mm ⇒ 周长 126mm。
 * 之所以和 MOTOR_WHEEL_D 分开：速度刻度（含 PID 增益、默认速度、转向 vmin/vmax）
 * 全是在 67 那套刻度上整定过的，把 67 改成 40 会让同一目标值的实车速变成
 * 1.67 倍（约 160mm/s，正是之前「走成弧线」的那档），只能单独修里程。 */
#define MOTOR_WHEEL_D_MEAS  40

/* 1 个编码器计数走过的直线距离 (mm) = 实测轮周长 / 每转计数。
 * 40mm 轮径 ⇒ 一圈 126mm，440 个计数一圈 ⇒ 0.286mm/计数。
 * 【原来的坑】曾用名义 67mm 算成 0.478mm/计数，航段按 250mm 走、地面只有
 * 150mm（比例 0.600）—— 两条腿一模一样，说明是常数错，不是打滑。 */
#define MOTOR_MM_PER_COUNT  (PI * MOTOR_WHEEL_D_MEAS / Encoder)

/* ============================================================================
 *  电机方向状态
 * ============================================================================
 */
#define MOTOR_BRAKE          0       // 刹车（H桥两上臂或两下臂导通）
#define MOTOR_FORWARD        1       // 正转
#define MOTOR_REVERSE        2       // 反转

/* ============================================================================
 *  方向切换死区
 * ============================================================================
 *  目标速度必须越过此阈值才能翻转电机方向，防止速度在0附近波动时
 *  方向频繁切换导致抖动和H桥损耗。
 *
 *  例如: 当前 FORWARD，目标速度必须 < -30 才切换到 REVERSE
 *        当前 REVERSE，目标速度必须 > +30 才切换到 FORWARD
 */
#define DIR_DEADBAND         3.0f

/* ============================================================================
 *  PWM 参数
 * ============================================================================
 *  源工程用 4000 计数的 PWM 周期（约 5kHz），这里保持一致：
 *  200000ns = 5kHz。drv_pwm.c 在本 BSP 上用 BSP_PWM_FINE_PRESCALE 把计数器
 *  时钟定成 BusClk/2 = 22.5MHz，5kHz 周期正好约 4500 个计数，
 *  所以 PID 输出 0~4000 这一整段刻度几乎能一一对应到计数器上，
 *  不需要为了精度改 PID 的标度。
 */
#define MOTOR_PWM_PERIOD_NS     200000      /* 5kHz */
#define MOTOR_PWM_DUTY_MAX      4000        /* 源工程满占空比刻度，也是 PID 输出上限 */

/* ============================================================================
 *  全局方向状态
 * ============================================================================
 */
extern volatile rt_uint8_t motor1_dir;
extern volatile rt_uint8_t motor2_dir;

/* ============================================================================
 *  全局目标速度（源工程由 gray.c 的 Line_Following_Control 设置）
 * ============================================================================
 */
extern float target_speed_1;
extern float target_speed_2;

/* ============================================================================
 *  公开接口
 * ============================================================================
 */
void motor_init(rt_uint8_t motor_id);
void motor_set_duty(rt_uint8_t motor_id, rt_uint32_t duty);
void motor_set_direction(rt_uint8_t motor_id, rt_uint8_t direction);
void motor_update_direction(rt_uint8_t motor_id);
void calculate_speed(rt_uint8_t motor_id);
void DC_MOTOR_PID(rt_uint8_t motor_id);

/* [PORT] 新增：给目标速度和读回状态用，源工程是外部代码直接写全局变量 */
void  motor_set_target(rt_uint8_t motor_id, float speed);
float motor_get_target(rt_uint8_t motor_id);
float motor_get_speed(rt_uint8_t motor_id);
rt_uint32_t motor_get_duty(rt_uint8_t motor_id);

/* [PORT] 新增：累计编码器计数，供 follow.c 算航段里程用（见 motor.c 里 odo_1
 * 的注释：距离只能从计数推，不能拿 motor_get_speed() 积分）。单位是计数，
 * 乘 MOTOR_MM_PER_COUNT 得毫米。掉电/复位才归零。 */
rt_int32_t motor_get_odo(rt_uint8_t motor_id);

/* ---- msh 的 `motor` 命令本体，msh 和无线串口上的 `#motor` 是同一个入口 ----
 * argv 按 msh 的约定：argv[0] 是命令名 "motor"，子命令从 argv[1] 起
 * （所以 msh 的 cmd_motor() 只是个三行包装）。
 *
 * 【为什么它要被转发到无线串口】`motor ff`（起步前馈，按电机分开的系数）必须
 * 【落地】标定：按住车、按 W、看 spd 从 0 跳起来那一拍的 duty。落地就得电池
 * 供电，而 msh 偏偏要插 USB —— 一插 USB 电机电源 VADJ 就从 7.49V 塌到 3.56V
 * （见 car.h），量到的根本不是工作时的数。标定办法写在 motor.c 的
 * motor_ff_duty() 上面。
 *
 * 顺带把 `motor stop` 也变成无线串口可达的急停手段。输出走 rt_kprintf（uart2
 * 控制台），无线串口那条链路上看不见，所以那边只回一行 ok/err，数值从速度上报的
 * 周期行里读（speed_report.h）。 */
int Motor_Cmd(int argc, char **argv);

#endif /* __MOTOR_H__ */
