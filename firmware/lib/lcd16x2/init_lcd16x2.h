#ifndef INIT_LCD16X2_H
#define INIT_LCD16X2_H

#include "libmcu.h"

#define LCD_GPIO_D4 0
#define LCD_GPIO_D5 1
#define LCD_GPIO_D6 2
#define LCD_GPIO_D7 3
#define LCD_GPIO_EN 4
#define LCD_GPIO_RS 5

// =================================
// SETUP PARA O LCD16x2
// =================================

void init_periferico_lcd16x2(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Pinos A0,A1,A2,A3,A4 e A5 como output
    GPIOA->MODER |= (0x555);
    GPIOA->OSPEEDR |= (0xAAA);
}

void write_d4(uint8_t state)
{
    if (state)
        GPIOA->BSRR |= (1 << LCD_GPIO_D4);
    else
        GPIOA->BSRR |= (1 << (LCD_GPIO_D4 + 16));
}

void write_d5(uint8_t state)
{
    if (state)
        GPIOA->BSRR |= (1 << LCD_GPIO_D5);
    else
        GPIOA->BSRR |= (1 << (LCD_GPIO_D5 + 16));
}

void write_d6(uint8_t state)
{
    if (state)
        GPIOA->BSRR |= (1 << LCD_GPIO_D6);
    else
        GPIOA->BSRR |= (1 << (LCD_GPIO_D6 + 16));
}

void write_d7(uint8_t state)
{
    if (state)
        GPIOA->BSRR |= (1 << LCD_GPIO_D7);
    else
        GPIOA->BSRR |= (1 << (LCD_GPIO_D7 + 16));
}

void write_en(uint8_t state)
{
    if (state)
        GPIOA->BSRR |= (1 << LCD_GPIO_EN); // EN = 1
    else
        GPIOA->BSRR |= (1 << (LCD_GPIO_EN + 16)); // EN = 0
}

void write_rs(uint8_t state)
{
    if (state)
        GPIOA->BSRR |= (1 << LCD_GPIO_RS);
    else
        GPIOA->BSRR |= (1 << (LCD_GPIO_RS + 16));
}


#endif