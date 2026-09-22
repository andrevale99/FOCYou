#ifndef INIT_DELAY_TIMER
#define INIT_DELAY_TIMER

#include <stm32f411xe.h>

#define DELAY_TIMER_BRIDGE RCC->APB2ENR
#define DELAY_TIMER TIM11

// frequencia de 1907.37Hz
#define DELAY_TIMER_PRESCALE (0) // No stm arr e psc ssao somados com +1
#define DELAY_TIEMR_ARR (0xFFFF)

void init_delya_timer11(void)
{
    DELAY_TIMER_BRIDGE |= RCC_APB2ENR_TIM11EN;

    DELAY_TIMER->PSC = DELAY_TIMER_PRESCALE;
    DELAY_TIMER->ARR = DELAY_TIEMR_ARR;
    DELAY_TIMER->EGR |= TIM_EGR_UG;
    DELAY_TIMER->CNT = 0;

    DELAY_TIMER->CR1 |= TIM_CR1_CEN;
}

void delay_ns(uint32_t ticks)
{
    uint16_t start = DELAY_TIMER->CNT;

    while ((uint16_t)(DELAY_TIMER->CNT - start) < ticks)
        ;
}

#endif