/*
 * Copyright (c) 2006-2025, RT-Thread Development Team
 * Copyright (c) 2019-2020, Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-10-24     Magicoe      first version
 * 2020-01-10     Kevin/Karl   Add PS demo
 * 2020-09-21     supperthomas fix the main.c
 * 2025-08-18     Alex Yang    Add P1_7 button with LED blink control
 * 2025-09-13     demo         RGB three-colour cycle + msh command
 * 2026-09-18     demo         Build stamp + LED boot signature
 */

#include <rtdevice.h>
#include "drv_pin.h"

/*
 * Build stamp, for telling a freshly flashed image apart from the one that was
 * already on the chip.  The RGB cycle below looks identical in every build, so
 * this string is the only thing that actually differs.
 *
 * __DATE__ and __TIME__ are expanded by the preprocessor each time this file is
 * compiled, so a rebuild of main.c always yields a new stamp.
 *
 * Caveat: it refreshes only when main.c itself is recompiled.  If you rebuild
 * after editing some other file the stamp stays the same - run
 * `touch applications/main.c` first to force a new one.
 */
#define APP_BUILD_STAMP     __DATE__ " " __TIME__

/*
 * Onboard RGB LED D1 for FRDM-MCXA346.
 * Pin numbering follows drv_pin.c: GET_PINS(port, pin) == 32 * port + pin.
 *
 * The LED is ACTIVE LOW - the MCUXpresso SDK board.h for this board defines
 * LOGIC_LED_ON as 0 and LOGIC_LED_OFF as 1, so driving the pin low lights it.
 */
#define LED_RED_PIN     GET_PINS(3, 18)     /* P3_18 */
#define LED_GREEN_PIN   GET_PINS(3, 19)     /* P3_19 */
#define LED_BLUE_PIN    GET_PINS(3, 21)     /* P3_21 */

#define LED_ON          PIN_LOW
#define LED_OFF         PIN_HIGH

/*
 * SW2 is the wake-up button on P1_7 (UM12348 Table 5). It has no entry in
 * board/MCUX_Config/board/pin_mux.c, so it is muxed on demand by mcx_pin_mode()
 * below. This path is untested on hardware - comment the define out if the
 * button turns out to misbehave.
 */
#define ENABLE_SW2_BUTTON
#ifdef ENABLE_SW2_BUTTON
#define SW2_PIN         GET_PINS(1, 7)      /* P1_7 */
#endif

#define LED_CYCLE_MS    500
#define LED_THREAD_PRIO 25                  /* below FinSH's tshell (20) */
#define LED_THREAD_STACK 1024

/* Boot signature - see led_boot_signature() */
#define LED_BOOT_FLASHES    3
#define LED_BOOT_FLASH_MS   100

static const rt_base_t rgb_pins[] = { LED_RED_PIN, LED_GREEN_PIN, LED_BLUE_PIN };
static const char *const rgb_names[] = { "red", "green", "blue" };
#define RGB_PIN_COUNT   (sizeof(rgb_pins) / sizeof(rgb_pins[0]))

/* Set by the `led` msh command to pause/resume the automatic colour cycle. */
static volatile rt_bool_t led_cycling = RT_TRUE;

static void led_set(rt_base_t pin, rt_bool_t on)
{
    rt_pin_write(pin, on ? LED_ON : LED_OFF);
}

static void led_all(rt_bool_t on)
{
    rt_size_t i;

    for (i = 0; i < RGB_PIN_COUNT; i++)
    {
        led_set(rgb_pins[i], on);
    }
}

/*
 * Three white flashes before the colour cycle starts - the half of the flash
 * check that needs no serial console.  All three channels lit at once is a
 * pattern the colour cycle never produces, so it is unambiguous on sight.
 */
static void led_boot_signature(void)
{
    int i;

    for (i = 0; i < LED_BOOT_FLASHES; i++)
    {
        led_all(RT_TRUE);
        rt_thread_mdelay(LED_BOOT_FLASH_MS);
        led_all(RT_FALSE);
        rt_thread_mdelay(LED_BOOT_FLASH_MS);
    }
}

static void led_cycle_thread(void *parameter)
{
    rt_size_t i = 0;

    while (1)
    {
        if (led_cycling)
        {
            led_all(RT_FALSE);
            led_set(rgb_pins[i], RT_TRUE);
            i = (i + 1) % RGB_PIN_COUNT;
        }

        rt_thread_mdelay(LED_CYCLE_MS);
    }
}

#ifdef ENABLE_SW2_BUTTON
static void sw2_irq_callback(void *args)
{
    rt_kprintf("SW2 pressed\n");
}
#endif

static int cmd_led(int argc, char **argv)
{
    rt_size_t i;

    if (argc < 2)
    {
        goto usage;
    }

    if (!rt_strcmp(argv[1], "on"))
    {
        led_cycling = RT_FALSE;
        led_all(RT_TRUE);
        rt_kprintf("led: all channels on\n");
        return RT_EOK;
    }

    if (!rt_strcmp(argv[1], "off"))
    {
        led_cycling = RT_FALSE;
        led_all(RT_FALSE);
        rt_kprintf("led: all channels off\n");
        return RT_EOK;
    }

    if (!rt_strcmp(argv[1], "cycle"))
    {
        led_cycling = RT_TRUE;
        rt_kprintf("led: cycling red -> green -> blue\n");
        return RT_EOK;
    }

    for (i = 0; i < RGB_PIN_COUNT; i++)
    {
        if (!rt_strcmp(argv[1], rgb_names[i]))
        {
            led_cycling = RT_FALSE;
            led_all(RT_FALSE);
            led_set(rgb_pins[i], RT_TRUE);
            rt_kprintf("led: %s on\n", rgb_names[i]);
            return RT_EOK;
        }
    }

usage:
    rt_kprintf("usage: led <on|off|red|green|blue|cycle>\n");
    return -RT_EINVAL;
}
MSH_CMD_EXPORT_ALIAS(cmd_led, led, control the onboard RGB LED);

int main(void)
{
    rt_thread_t tid;
    rt_size_t i;

#if defined(__CC_ARM)
    rt_kprintf("using armcc, version: %d\n", __ARMCC_VERSION);
#elif defined(__clang__)
    rt_kprintf("using armclang, version: %d\n", __ARMCC_VERSION);
#elif defined(__ICCARM__)
    rt_kprintf("using iccarm, version: %d\n", __VER__);
#elif defined(__GNUC__)
    rt_kprintf("using gcc, version: %d.%d\n", __GNUC__, __GNUC_MINOR__);
#endif

    rt_kprintf("FRDM-MCXA346 RT-Thread demo\n");
    rt_kprintf("console: %s, 115200 8N1 - type 'help' for msh commands\n",
               RT_CONSOLE_DEVICE_NAME);
    rt_kprintf("build: %s\n", APP_BUILD_STAMP);

    /* RGB LED pins as outputs, all channels dark to start with */
    for (i = 0; i < RGB_PIN_COUNT; i++)
    {
        rt_pin_mode(rgb_pins[i], PIN_MODE_OUTPUT);
    }
    led_all(RT_FALSE);

    /* Flash the boot signature while nothing else is driving the LED yet. */
    led_boot_signature();

#ifdef ENABLE_SW2_BUTTON
    rt_pin_mode(SW2_PIN, PIN_MODE_INPUT_PULLUP);
    rt_pin_attach_irq(SW2_PIN, PIN_IRQ_MODE_FALLING, sw2_irq_callback, RT_NULL);
    rt_pin_irq_enable(SW2_PIN, PIN_IRQ_ENABLE);
#endif

    /*
     * Drive the colour cycle from its own thread rather than from main(), so
     * that main() can return and the tshell thread stays free to service the
     * console.
     */
    tid = rt_thread_create("led", led_cycle_thread, RT_NULL,
                           LED_THREAD_STACK, LED_THREAD_PRIO, 10);
    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
    }
    else
    {
        rt_kprintf("failed to create the led thread\n");
    }

    return RT_EOK;
}
