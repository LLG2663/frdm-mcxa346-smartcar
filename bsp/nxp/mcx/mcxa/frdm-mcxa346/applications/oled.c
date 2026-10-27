/*
 * SPDX-License-Identifier: MIT
 *
 * SSD1306 128x64 OLED driver, ported from the CCS project
 * 113/user_driver/oled.{c,h} (TI MSPM0G3507) to RT-Thread on FRDM-MCXA346.
 *
 * 绘图层（画点/画线/画圆/显示字符）与原工程逐行一致，所以那边 main.c 的
 * 显示代码可以原样搬过来。真正改动的只有三处，都在下面标了 [PORT]：
 *   [PORT] OLED_WR_Byte()   TI 的 DL_I2C_* → rt_i2c_master_send()
 *   [PORT] OLED_Refresh()   按页合并传输，一帧 1048 次 → 16 次 I2C
 *   [PORT] OLED_Init()      找 I2C 总线，delay_ms() → rt_thread_mdelay()
 * 另外给画点加了越界保护，越界不再往外写显存。
 */

#include <rtdevice.h>

#include "oled.h"
#include "oledfont.h"

#define OLED_WIDTH          128
#define OLED_PAGES          8

/* 显存：按列存放，OLED_GRAM[x][page]，每页 8 个像素、bit0 在最上面。
 * 原工程声明成 144 列但只用 0~127，这里按实际宽度开。 */
u8 OLED_GRAM[OLED_WIDTH][OLED_PAGES];

static struct rt_i2c_bus_device *oled_i2c = RT_NULL;

/* [PORT] 一次 I2C 写。原工程这里是等 IDLE、填 FIFO、启动、再等 IDLE 的
 * 五步硬件操作；RT-Thread 下 rt_i2c_master_send() 已经把等待和错误处理
 * 都包在阻塞传输里了。 */
static void oled_i2c_write(const u8 *buf, rt_uint32_t len)
{
    if (oled_i2c == RT_NULL)
        return;

    rt_i2c_master_send(oled_i2c, OLED_I2C_ADDR, RT_I2C_WR, buf, len);
}

//反显函数
void OLED_ColorTurn(u8 i)
{
    if (i == 0) OLED_WR_Byte(0xA6, OLED_CMD);   //正常显示
    if (i == 1) OLED_WR_Byte(0xA7, OLED_CMD);   //反色显示
}

//屏幕旋转180度
void OLED_DisplayTurn(u8 i)
{
    if (i == 0)
    {
        OLED_WR_Byte(0xC8, OLED_CMD);   //正常显示
        OLED_WR_Byte(0xA1, OLED_CMD);
    }
    if (i == 1)
    {
        OLED_WR_Byte(0xC0, OLED_CMD);   //反转显示
        OLED_WR_Byte(0xA0, OLED_CMD);
    }
}

void OLED_WR_Byte(uint8_t dat, uint8_t mode)
{
    u8 txData[2];

    /* 控制字节：0x00 为命令，0x40 为数据 */
    txData[0] = mode ? 0x40 : 0x00;
    txData[1] = dat;

    oled_i2c_write(txData, 2);
}

//开启OLED显示
void OLED_DisPlay_On(void)
{
    OLED_WR_Byte(0x8D, OLED_CMD);   //电荷泵使能
    OLED_WR_Byte(0x14, OLED_CMD);   //开启电荷泵
    OLED_WR_Byte(0xAF, OLED_CMD);   //点亮屏幕
}

//关闭OLED显示
void OLED_DisPlay_Off(void)
{
    OLED_WR_Byte(0x8D, OLED_CMD);   //电荷泵使能
    OLED_WR_Byte(0x10, OLED_CMD);   //关闭电荷泵
    OLED_WR_Byte(0xAF, OLED_CMD);   //关闭屏幕
}

//更新显存到OLED
void OLED_Refresh(void)
{
    u8 i, n;
    u8 cmd[4];
    u8 data[OLED_WIDTH + 1];

    /* [PORT] 原工程每个字节一次 I2C 传输（一帧 8 页 × 131 次 = 1048 次），
     * 在 RT-Thread 下每次传输都有独立的启动/地址/停止开销，太慢。
     * 这里改成每页两次传输：3 字节定位命令合成一条，128 字节显存数据
     * 合成一条（都以控制字节打头）。命令序列与原来完全相同。 */
    data[0] = 0x40;
    cmd[0]  = 0x00;

    for (i = 0; i < OLED_PAGES; i++)
    {
        cmd[1] = (u8)(0xb0 + i);    //设置页地址
        cmd[2] = 0x00;              //设置低列地址
        cmd[3] = 0x10;              //设置高列地址
        oled_i2c_write(cmd, 4);

        for (n = 0; n < OLED_WIDTH; n++)
            data[n + 1] = OLED_GRAM[n][i];
        oled_i2c_write(data, OLED_WIDTH + 1);
    }
}

//清屏函数
void OLED_Clear(void)
{
    u8 i, n;
    for (i = 0; i < OLED_PAGES; i++)
    {
        for (n = 0; n < OLED_WIDTH; n++)
        {
            OLED_GRAM[n][i] = 0;    //清除所有数据
        }
    }
    OLED_Refresh();                 //更新显示
}

//画点
void OLED_DrawPoint(u8 x, u8 y)
{
    u8 i, m, n;
    if (x >= OLED_WIDTH || y >= (OLED_PAGES * 8)) return;   //越界保护，见文件头
    i = y / 8;
    m = y % 8;
    n = 1 << m;
    OLED_GRAM[x][i] |= n;
}

//清除一个点
void OLED_ClearPoint(u8 x, u8 y)
{
    u8 i, m, n;
    if (x >= OLED_WIDTH || y >= (OLED_PAGES * 8)) return;   //越界保护，见文件头
    i = y / 8;
    m = y % 8;
    n = 1 << m;
    OLED_GRAM[x][i] &= (u8)~n;
}

//画线
void OLED_DrawLine(u8 x1, u8 y1, u8 x2, u8 y2)
{
    u8 i, k, k1, k2;
    if ((x2 >= OLED_WIDTH) || (y2 >= (OLED_PAGES * 8)) || (x1 > x2) || (y1 > y2)) return;
    if (x1 == x2)       //画竖线
    {
        for (i = 0; i < (y2 - y1); i++) OLED_DrawPoint(x1, y1 + i);
    }
    else if (y1 == y2)  //画横线
    {
        for (i = 0; i < (x2 - x1); i++) OLED_DrawPoint(x1 + i, y1);
    }
    else                //画斜线
    {
        k1 = y2 - y1;
        k2 = x2 - x1;
        k = k1 * 10 / k2;
        for (i = 0; i < (x2 - x1); i++) OLED_DrawPoint(x1 + i, y1 + i * k / 10);
    }
}

//画圆
void OLED_DrawCircle(u8 x, u8 y, u8 r)
{
    int a = 0, b = r, num;
    while (2 * b * b >= r * r)
    {
        OLED_DrawPoint(x + a, y - b);
        OLED_DrawPoint(x - a, y - b);
        OLED_DrawPoint(x - a, y + b);
        OLED_DrawPoint(x + a, y + b);
        OLED_DrawPoint(x + b, y + a);
        OLED_DrawPoint(x + b, y - a);
        OLED_DrawPoint(x - b, y - a);
        OLED_DrawPoint(x - b, y + a);

        a++;
        num = (a * a + b * b) - r * r;
        if (num > 0) { b--; a--; }
    }
}

//显示字符
void OLED_ShowChar(u8 x, u8 y, u8 chr, u8 size1)
{
    u8 i, m, temp, size2, chr1;
    u8 y0 = y;

    if (size1 != 16) return;                //只保留 16 号字，见 oled.h
    if (chr < ' ' || chr > '~') return;     //字模表只覆盖 0x20~0x7E

    size2 = (size1 / 8 + ((size1 % 8) ? 1 : 0)) * (size1 / 2);
    chr1 = chr - ' ';
    for (i = 0; i < size2; i++)
    {
        temp = asc2_1608[chr1][i];
        for (m = 0; m < 8; m++)
        {
            if (temp & 0x80) OLED_DrawPoint(x, y);
            else OLED_ClearPoint(x, y);
            temp <<= 1;
            y++;
            if ((y - y0) == size1)
            {
                y = y0;
                x++;
                break;
            }
        }
    }
}

//显示字符串
void OLED_ShowString(u8 x, u8 y, u8 *chr, u8 size1)
{
    while ((*chr >= ' ') && (*chr <= '~'))
    {
        OLED_ShowChar(x, y, *chr, size1);
        x += size1 / 2;
        if (x > 128 - size1)    //换行
        {
            x = 0;
            y += size1;         //修复了原代码的 y+=2 的 bug
        }
        chr++;
    }
}

//m^n
u32 OLED_Pow(u8 m, u8 n)
{
    u32 result = 1;
    while (n--) result *= m;
    return result;
}

//显示数字
void OLED_ShowNum(u8 x, u8 y, u32 num, u8 len, u8 size1)
{
    u8 t, temp;
    for (t = 0; t < len; t++)
    {
        temp = (num / OLED_Pow(10, len - t - 1)) % 10;
        if (temp == 0) OLED_ShowChar(x + (size1 / 2) * t, y, '0', size1);
        else OLED_ShowChar(x + (size1 / 2) * t, y, temp + '0', size1);
    }
}

//OLED的初始化
rt_err_t OLED_Init(void)
{
    /* [PORT] 先拿到 I2C 总线。找不到就直接返回，别让后面每个字节都去
     * 访问空指针——那样屏幕上什么都不显示，却看不出是配错了总线名。 */
    oled_i2c = rt_i2c_bus_device_find(OLED_I2C_BUS_NAME);
    if (oled_i2c == RT_NULL)
    {
        rt_kprintf("oled: i2c bus \"%s\" not found\n", OLED_I2C_BUS_NAME);
        return -RT_ERROR;
    }

    /* 4针OLED没有RST引脚，直接延时等待屏幕内部RC电路上电复位完成 */
    rt_thread_mdelay(100);

    OLED_WR_Byte(0xAE, OLED_CMD);   //--turn off oled panel
    OLED_WR_Byte(0x00, OLED_CMD);   //---set low column address
    OLED_WR_Byte(0x10, OLED_CMD);   //---set high column address
    OLED_WR_Byte(0x40, OLED_CMD);   //--set start line address
    OLED_WR_Byte(0x81, OLED_CMD);   //--set contrast control register
    OLED_WR_Byte(0xCF, OLED_CMD);   // Set SEG Output Current Brightness
    OLED_WR_Byte(0xA1, OLED_CMD);   //--Set SEG/Column Mapping
    OLED_WR_Byte(0xC8, OLED_CMD);   //Set COM/Row Scan Direction
    OLED_WR_Byte(0xA6, OLED_CMD);   //--set normal display
    OLED_WR_Byte(0xA8, OLED_CMD);   //--set multiplex ratio(1 to 64)
    OLED_WR_Byte(0x3f, OLED_CMD);   //--1/64 duty
    OLED_WR_Byte(0xD3, OLED_CMD);   //-set display offset
    OLED_WR_Byte(0x00, OLED_CMD);   //-not offset
    OLED_WR_Byte(0xd5, OLED_CMD);   //--set display clock divide ratio/oscillator frequency
    OLED_WR_Byte(0x80, OLED_CMD);   //--set divide ratio, Set Clock as 100 Frames/Sec
    OLED_WR_Byte(0xD9, OLED_CMD);   //--set pre-charge period
    OLED_WR_Byte(0xF1, OLED_CMD);   //Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
    OLED_WR_Byte(0xDA, OLED_CMD);   //--set com pins hardware configuration
    OLED_WR_Byte(0x12, OLED_CMD);
    OLED_WR_Byte(0xDB, OLED_CMD);   //--set vcomh
    OLED_WR_Byte(0x40, OLED_CMD);   //Set VCOM Deselect Level
    OLED_WR_Byte(0x20, OLED_CMD);   //-Set Page Addressing Mode (0x00/0x01/0x02)
    OLED_WR_Byte(0x02, OLED_CMD);   //
    OLED_WR_Byte(0x8D, OLED_CMD);   //--set Charge Pump enable/disable
    OLED_WR_Byte(0x14, OLED_CMD);   //--set(0x10) disable
    OLED_WR_Byte(0xA4, OLED_CMD);   // Disable Entire Display On (0xa4/0xa5)
    OLED_WR_Byte(0xA6, OLED_CMD);   // Disable Inverse Display On (0xa6/a7)
    OLED_WR_Byte(0xAF, OLED_CMD);
    OLED_Clear();

    return RT_EOK;
}
