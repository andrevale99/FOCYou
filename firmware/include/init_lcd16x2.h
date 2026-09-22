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

#define DELAY_TIMER TIM11

// frequencia de 1907.37Hz
#define DELAY_TIMER_PRESCALE (0) // No stm arr e psc ssao somados com +1
#define DELAY_TIEMR_ARR (0xFFFF)

// =================================
// SETUP PARA O LCD16x2
// =================================

void init_periferico_lcd16x2(void)
{
    LCD_BRIDGE_RCC |= RCC_AHB1ENR_GPIOAEN;

    // Pinos A0,A1,A2,A3,A4 e A5 como output
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

    DELAY_TIMER->PSC = DELAY_TIMER_PRESCALE;
    DELAY_TIMER->ARR = DELAY_TIEMR_ARR;
    DELAY_TIMER->EGR |= TIM_EGR_UG;
    DELAY_TIMER->CNT = 0;

    DELAY_TIMER->CR1 |= TIM_CR1_CEN;
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

// @brief Caso nao tenha modificado nada, cada valor
// do contador correponde a 40e-8 s (40 ns)
void delay_lcd(uint32_t ticks)
{
    uint16_t start = DELAY_TIMER->CNT;

    while ((uint16_t)(DELAY_TIMER->CNT - start) < ticks)
        ;
}
#endif