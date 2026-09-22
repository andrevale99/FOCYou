#ifndef INIT_LCD16X2_H
#define INIT_LCD16X2_H

#include <stm32f411xe.h>

#define LCD_GPIO_D4 0
#define LCD_GPIO_D5 1
#define LCD_GPIO_D6 2
#define LCD_GPIO_D7 3
#define LCD_GPIO_EN 4
#define LCD_GPIO_RS 5

#define LCD_DATA_PORT GPIOA
#define LCD_CMD_PORT GPIOA

#define LCD_BRIDGE_RCC RCC->AHB1ENR
#define LCD_BRIDGE_RCC_ENABLE RCC_AHB1ENR_GPIOAEN


// =================================
// SETUP PARA O LCD16x2
// =================================

void init_periferico_lcd16x2(void)
{
    LCD_BRIDGE_RCC |= RCC_AHB1ENR_GPIOAEN;
    

    LCD_DATA_PORT->MODER &= ~((0x3 << (LCD_GPIO_D4 << 1)) |
                              (0x3 << (LCD_GPIO_D5 << 1)) |
                              (0x3 << (LCD_GPIO_D6 << 1)) |
                              (0x3 << (LCD_GPIO_D7 << 1)));

    LCD_DATA_PORT->MODER |= ((0x1 << (LCD_GPIO_D4 << 1)) |
                             (0x1 << (LCD_GPIO_D5 << 1)) |
                             (0x1 << (LCD_GPIO_D6 << 1)) |
                             (0x1 << (LCD_GPIO_D7 << 1)));

    // Pinos EN e RS como output
    LCD_CMD_PORT->MODER &= ~((0x3 << (LCD_GPIO_EN << 1)) |
                             (0x3 << (LCD_GPIO_RS << 1)));

    LCD_CMD_PORT->MODER |= ((0x1 << (LCD_GPIO_EN << 1)) |
                            (0x1 << (LCD_GPIO_RS << 1)));
}

void write_d4(uint8_t state)
{
    if (state)
        LCD_DATA_PORT->BSRR |= (1 << LCD_GPIO_D4);
    else
        LCD_DATA_PORT->BSRR |= (1 << (LCD_GPIO_D4 + 16));
}

void write_d5(uint8_t state)
{
    if (state)
        LCD_DATA_PORT->BSRR |= (1 << LCD_GPIO_D5);
    else
        LCD_DATA_PORT->BSRR |= (1 << (LCD_GPIO_D5 + 16));
}

void write_d6(uint8_t state)
{
    if (state)
        LCD_DATA_PORT->BSRR |= (1 << LCD_GPIO_D6);
    else
        LCD_DATA_PORT->BSRR |= (1 << (LCD_GPIO_D6 + 16));
}

void write_d7(uint8_t state)
{
    if (state)
        LCD_DATA_PORT->BSRR |= (1 << LCD_GPIO_D7);
    else
        LCD_DATA_PORT->BSRR |= (1 << (LCD_GPIO_D7 + 16));
}

void write_en(uint8_t state)
{
    if (state)
        LCD_CMD_PORT->BSRR |= (1 << LCD_GPIO_EN); // EN = 1
    else
        LCD_CMD_PORT->BSRR |= (1 << (LCD_GPIO_EN + 16)); // EN = 0
}

void write_rs(uint8_t state)
{
    if (state)
        LCD_CMD_PORT->BSRR |= (1 << LCD_GPIO_RS);
    else
        LCD_CMD_PORT->BSRR |= (1 << (LCD_GPIO_RS + 16));
}
#endif