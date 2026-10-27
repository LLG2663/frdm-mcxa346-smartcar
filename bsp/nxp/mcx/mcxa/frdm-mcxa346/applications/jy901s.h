/*
 * SPDX-License-Identifier: MIT
 *
 * JY901S 姿态模块（串口 IMU），移植自 CCS 工程 113/user_driver/uart.{c,h}
 * （TI MSPM0G3507）到 FRDM-MCXA346 上的 RT-Thread。
 *
 * 帧同步、校验、yaw 校零/归一/符号这些全是纯算法，与平台无关，逐行照搬；
 * 真正改动的只有"字节从哪来"：TI 那边是 UART 中断里直接解析，
 * 这里改成 RT-Thread 串口设备（INT_RX 环形缓冲）+ 一个解析线程。
 *
 * 函数名保留源工程的 JY61P_ 前缀（JY61P 和 JY901S 是同一系列模块，
 * 源工程内部一直叫它 JY61P）。这样那边 main.c 的显示代码可以一字不改搬过来，
 * 也不会和"JY901S 是这个新文件"这件事打架。
 */

#ifndef __JY901S_H
#define __JY901S_H

#include <rtthread.h>
#include <stdint.h>

/* 说明：不再保留 data_ready 字段。
 * "是否有新数据"统一由解析层置位、使用方清除的 new_data_flag 表达，
 * 通过 JY61P_HasNewData() / JY61P_ClearNewData() 访问，避免两套机制并存。 */
typedef struct {
    float   roll, pitch, yaw, wz;
    int16_t raw_roll, raw_pitch, raw_yaw, raw_wz;
} JY61P_Data_t;

extern volatile JY61P_Data_t jy61p;

/**
 * @brief 打开 IMU 串口并启动解析线程。
 *
 * 也可以由 INIT_APP_EXPORT 自动调用（本文件里已经这么做了）。
 * @return RT_EOK 成功；-RT_ERROR 表示没找到 uart3
 *         （多半是 rtconfig.h 里 BSP_USING_UART3 没开）。
 */
rt_err_t JY61P_Init(void);

/* jy61p.yaw / JY61P_GetYaw() 给的都是"已校零"的 yaw：
 * 上电后第一次朝向 = 0 度，范围 -180~+180，左转负、右转正。 */
float JY61P_GetYaw(void);
float JY61P_GetFilteredYaw(void);
float JY61P_GetFilteredWz(void);
uint8_t JY61P_HasNewData(void);
void   JY61P_ClearNewData(void);
void   JY61P_ResetFilter(float yaw, float wz);
/* 把"当前朝向"重新记为 0 度。上电后自动执行一次，运行中也可随时调用。 */
void   JY61P_ZeroYaw(void);

/* 诊断计数，只在 msh 命令里看，用来区分"一个字节都没收到"和"收到了但解不出帧"。
 * 前者是接线/复用配错了，后者是波特率或协议对不上。 */
rt_uint32_t JY61P_GetRxBytes(void);
rt_uint32_t JY61P_GetFrameCount(void);

#endif /* __JY901S_H */
