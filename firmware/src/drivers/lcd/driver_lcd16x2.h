#ifndef INIT_LCD16X2_H
#define INIT_LCD16X2_H

#include <stm32f411xe.h>

#include "drivers/gpio/driver_gpio.h"

#define LCD_GPIO_D4 0
#define LCD_GPIO_D5 1
#define LCD_GPIO_D6 2
#define LCD_GPIO_D7 3
#define LCD_GPIO_EN 4
#define LCD_GPIO_RS 5

#define LCD_DATA_PORT GPIOA
#define LCD_CMD_PORT  GPIOA

// =================================
// SETUP PARA O LCD16x2
// =================================

static inline driver_gpio_err_t driver_lcd16x2_init(void)
{
    static const uint16_t data_pins[] = {LCD_GPIO_D4, LCD_GPIO_D5, LCD_GPIO_D6, LCD_GPIO_D7};
    static const uint16_t cmd_pins[]  = {LCD_GPIO_EN, LCD_GPIO_RS};

    driver_gpio_err_t err;

    err = driver_gpio_enable_clock(LCD_DATA_PORT);
    if (err != DRIVER_GPIO_OK)
        return err;

    err = driver_gpio_enable_clock(LCD_CMD_PORT);
    if (err != DRIVER_GPIO_OK)
        return err;

    for (uint8_t i = 0; i < sizeof(data_pins) / sizeof(data_pins[0]); i++)
    {
        err = driver_gpio_set_mode(LCD_DATA_PORT, data_pins[i], DRIVER_GPIO_OUTPUT);
        if (err != DRIVER_GPIO_OK)
            return err;
    }

    for (uint8_t i = 0; i < sizeof(cmd_pins) / sizeof(cmd_pins[0]); i++)
    {
        err = driver_gpio_set_mode(LCD_CMD_PORT, cmd_pins[i], DRIVER_GPIO_OUTPUT);
        if (err != DRIVER_GPIO_OK)
            return err;
    }

    return DRIVER_GPIO_OK;
}

static inline void write_d4(uint8_t state)
{
    driver_gpio_write_pin(LCD_DATA_PORT, LCD_GPIO_D4, state ? DRIVER_GPIO_PIN_SET : DRIVER_GPIO_PIN_RESET);
}

static inline void write_d5(uint8_t state)
{
    driver_gpio_write_pin(LCD_DATA_PORT, LCD_GPIO_D5, state ? DRIVER_GPIO_PIN_SET : DRIVER_GPIO_PIN_RESET);
}

static inline void write_d6(uint8_t state)
{
    driver_gpio_write_pin(LCD_DATA_PORT, LCD_GPIO_D6, state ? DRIVER_GPIO_PIN_SET : DRIVER_GPIO_PIN_RESET);
}

static inline void write_d7(uint8_t state)
{
    driver_gpio_write_pin(LCD_DATA_PORT, LCD_GPIO_D7, state ? DRIVER_GPIO_PIN_SET : DRIVER_GPIO_PIN_RESET);
}

static inline void write_en(uint8_t state)
{
    driver_gpio_write_pin(LCD_CMD_PORT, LCD_GPIO_EN, state ? DRIVER_GPIO_PIN_SET : DRIVER_GPIO_PIN_RESET);
}

static inline void write_rs(uint8_t state)
{
    driver_gpio_write_pin(LCD_CMD_PORT, LCD_GPIO_RS, state ? DRIVER_GPIO_PIN_SET : DRIVER_GPIO_PIN_RESET);
}

#endif