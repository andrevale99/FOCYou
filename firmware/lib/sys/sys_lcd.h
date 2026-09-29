#ifndef SYS_LCD_H
#define SYS_LCD_H

#include <stm32f4xx_hal.h>

#include "lcd16x2.h"
#include "drivers/driver_lcd16x2.h"

static lcd16x2_handle_t lcd = {
    .d4.write = write_d4,
    .d5.write = write_d5,
    .d6.write = write_d6,
    .d7.write = write_d7,

    .en.write = write_en,
    .rs.write = write_rs,

    .delay_ms = HAL_Delay,
};

void sys_lcd_init(void)
{
    if (lcd16x2_init_4bits(&lcd, init_periferico_lcd16x2) != LCD_OK)
    {
        // Tratamento
    }
}

#endif