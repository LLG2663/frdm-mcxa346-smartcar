/*
 * SPDX-License-Identifier: MIT
 *
 * SSD1306 128x64 OLED driver, ported from the CCS project
 * 113/user_driver/oled.{c,h} (TI MSPM0G3507) to RT-Thread on FRDM-MCXA346.
 *
 * The drawing layer is unchanged on purpose: keeping the original API means the
 * display code in that project's main.c moves over verbatim, layout included.
 * The only real port work is OLED_WR_Byte(), which used TI's DL_I2C_* and now
 * goes through rt_i2c_master_send().
 */

#ifndef __OLED_H
#define __OLED_H

#include <rtthread.h>
#include <stdint.h>

#define OLED_CMD  0     /* 写命令 */
#define OLED_DATA 1     /* 写数据 */

typedef unsigned char u8;
typedef unsigned int  u32;

/* 总线与从机地址。改动这里就可以换到别的 I2C 口。 */
#define OLED_I2C_BUS_NAME   "i2c2"
#define OLED_I2C_ADDR       0x3C

/**
 * @brief 初始化 OLED。
 *
 * 除了原来那串 SSD1306 配置命令，还要先在 RT-Thread 里找到 I2C 总线，
 * 所以返回值有意义：-RT_ERROR 表示没找到总线（多半是 i2c2 没使能）。
 * 按语句调用（忽略返回值）也是合法的，和原工程写法一致。
 */
rt_err_t OLED_Init(void);

void OLED_ColorTurn(u8 i);          /* 0：正常显示  1：反色显示 */
void OLED_DisplayTurn(u8 i);        /* 0：正常方向  1：旋转 180° */

/**
 * @brief 向 OLED 写一个字节。
 *
 * @param dat  要写入的内容（命令或显存数据）
 * @param mode OLED_CMD(0) 写命令，OLED_DATA(1) 写数据
 *
 * 这是唯一一个平台相关函数：控制字节 0x00/0x40 + 数据，一次 I2C 传输发完。
 */
void OLED_WR_Byte(u8 dat, u8 mode);

void OLED_DisPlay_On(void);
void OLED_DisPlay_Off(void);

/**
 * @brief 把整块显存 OLED_GRAM 刷到屏幕。
 *
 * 原工程是逐字节发（一帧 1048 次 I2C 传输）；这里按页合并成
 * 「3 字节定位命令 + 128 字节数据」两次传输，一帧只剩 16 次，
 * 100 kHz 下从约 100 ms 降到约 13 ms。命令序列本身没有变。
 */
void OLED_Refresh(void);

void OLED_Clear(void);                                  /* 清显存并刷新 */
void OLED_DrawPoint(u8 x, u8 y);                        /* 置位显存中的点 */
void OLED_ClearPoint(u8 x, u8 y);                       /* 清掉显存中的点 */
void OLED_DrawLine(u8 x1, u8 y1, u8 x2, u8 y2);
void OLED_DrawCircle(u8 x, u8 y, u8 r);

/**
 * @brief 显示一个 ASCII 字符。
 *
 * @param size1 字符高度，只支持 16（8x16 字模）。
 *              原工程还带 12/24 号字，本移植只需要 16 号，
 *              为省 flash 只保留了 asc2_1608 这一张表。
 */
void OLED_ShowChar(u8 x, u8 y, u8 chr, u8 size1);

/** @brief 显示一个以 '\0' 结尾的 ASCII 字符串。 */
void OLED_ShowString(u8 x, u8 y, u8 *chr, u8 size1);

/** @brief 显示定长无符号十进制数，不足 len 位左侧补 '0'。 */
void OLED_ShowNum(u8 x, u8 y, u32 num, u8 len, u8 size1);

#endif /* __OLED_H */
