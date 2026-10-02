#include "inverter.h"

inverter_error_t inverter_init(inverter_config_t *config)
{
    if (config == NULL)
        return INVERTER_ERROR_INVALID_ARGUMENT;

    TIM_TypeDef *const timer = config->advanced_timer;
    GPIO_TypeDef *const gpioxH = config->gpioH;
    GPIO_TypeDef *const gpioxL = config->gpioL;

    if (timer == NULL)
        return INVERTER_ERROR_INVALID_ARGUMENT;

    if (driver_gpio_enable_clock(gpioxH) != DRIVER_GPIO_OK ||
        driver_gpio_enable_clock(gpioxL) != DRIVER_GPIO_OK)
        return INVERTER_ERROR_INVALID_ARGUMENT;

#if defined(TIM1) && defined(RCC_APB2ENR_TIM1EN)
    if (timer == TIM1)
    {
        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    }
#endif

#if defined(TIM8) && defined(RCC_APB2ENR_TIM8EN)
    else if (timer == TIM8)
    {
        RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
    }
#endif

#if defined(TIM20) && defined(RCC_APB2ENR_TIM20EN)
    else if (timer == TIM20)
    {
        RCC->APB2ENR |= RCC_APB2ENR_TIM20EN;
    }
#endif
    else
        return INVERTER_ERROR_INVALID_ARGUMENT;

    for (uint8_t idx = 0; idx < MAX_CHN_INVERTER; ++idx)
    {
        const inverter_gpio_config_t *cfgH = &config->config_gpioH[idx];
        const inverter_gpio_config_t *cfgL = &config->config_gpioL[idx];

        if (driver_gpio_set_alternate_function(gpioxH, cfgH->pin, cfgH->alternate_function) != DRIVER_GPIO_OK ||
            driver_gpio_set_alternate_function(gpioxL, cfgL->pin, cfgL->alternate_function) != DRIVER_GPIO_OK)
            return INVERTER_ERROR_INVALID_ARGUMENT;

        if (driver_gpio_set_output_type(gpioxH, cfgH->pin, DRIVER_GPIO_OTYPE_PUSHPULL) != DRIVER_GPIO_OK ||
            driver_gpio_set_output_type(gpioxL, cfgL->pin, DRIVER_GPIO_OTYPE_PUSHPULL) != DRIVER_GPIO_OK)
            return INVERTER_ERROR_INVALID_ARGUMENT;

        if (driver_gpio_set_speed(gpioxH, cfgH->pin, DRIVER_GPIO_SPEED_VERY_HIGH) != DRIVER_GPIO_OK ||
            driver_gpio_set_speed(gpioxL, cfgL->pin, DRIVER_GPIO_SPEED_VERY_HIGH) != DRIVER_GPIO_OK)
            return INVERTER_ERROR_INVALID_ARGUMENT;

        if (driver_gpio_set_pull(gpioxH, cfgH->pin, DRIVER_GPIO_PULL_NONE) != DRIVER_GPIO_OK ||
            driver_gpio_set_pull(gpioxL, cfgL->pin, DRIVER_GPIO_PULL_NONE) != DRIVER_GPIO_OK)
            return INVERTER_ERROR_INVALID_ARGUMENT;
    }

    timer->ARR = config->autorreload;
    timer->PSC = config->prescale;

    timer->CR1 &= ~TIM_CR1_CMS_Msk;
    timer->CR1 |= TIM_CR1_CMS; /* center-aligned mode 3 */
    timer->CR1 |= TIM_CR1_ARPE;

    timer->CCMR1 &= ~(TIM_CCMR1_OC1M_Msk | TIM_CCMR1_OC2M_Msk);
    timer->CCMR1 |= ((0x6UL << TIM_CCMR1_OC1M_Pos) |
                     (0x6UL << TIM_CCMR1_OC2M_Pos));

    timer->CCMR2 &= ~TIM_CCMR2_OC3M_Msk;
    timer->CCMR2 |= (0x6UL << TIM_CCMR2_OC3M_Pos);

    timer->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
    timer->CCMR2 |= TIM_CCMR2_OC3PE;

    /* Zera os duty cycles antes de habilitar as saídas */
    timer->CCR1 = 0;
    timer->CCR2 = 0;
    timer->CCR3 = 0;

    timer->CCER &= ~(TIM_CCER_CC1E_Msk | TIM_CCER_CC2E_Msk | TIM_CCER_CC3E_Msk |
                     TIM_CCER_CC1NE_Msk | TIM_CCER_CC2NE_Msk | TIM_CCER_CC3NE_Msk);

    timer->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E |
                    TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE);

    timer->BDTR &= ~TIM_BDTR_DTG_Msk;
    timer->BDTR |= (((uint32_t)config->deadtime << TIM_BDTR_DTG_Pos) & TIM_BDTR_DTG_Msk);

    timer->BDTR |= TIM_BDTR_MOE;

    timer->EGR = TIM_EGR_UG;

    return INVERTER_OK;
}

inverter_error_t inverter_set_state(inverter_duty_cycle_t *inverter, inverter_state_t state)
{
    if (inverter == NULL)
        return INVERTER_ERROR_INVALID_ARGUMENT;

    switch (state)
    {
    case INVERTER_STATE_ON:
        inverter->advanced_timer->BDTR |= TIM_BDTR_MOE;
        break;

    case INVERTER_STATE_OFF:
        inverter->advanced_timer->BDTR &= ~TIM_BDTR_MOE;
        break;

    default:
        return INVERTER_ERROR_NO_STATE;
        break;
    }

    return INVERTER_OK;
}

inverter_error_t inverter_set_duty_cycle(inverter_duty_cycle_t *duty_cycle)
{
    if (duty_cycle == NULL)
        return INVERTER_ERROR_INVALID_ARGUMENT;

    duty_cycle->advanced_timer->CCR1 = duty_cycle->duty_cycle_a;
    duty_cycle->advanced_timer->CCR2 = duty_cycle->duty_cycle_b;
    duty_cycle->advanced_timer->CCR3 = duty_cycle->duty_cycle_c;

    return INVERTER_OK;
}

inverter_state_t inverter_get_state(inverter_duty_cycle_t *inverter)
{
    if (inverter == NULL)
        return INVERTER_ERROR_INVALID_ARGUMENT;

    return ((inverter->advanced_timer->BDTR & TIM_BDTR_MOE) ? INVERTER_STATE_ON : INVERTER_STATE_OFF);
}