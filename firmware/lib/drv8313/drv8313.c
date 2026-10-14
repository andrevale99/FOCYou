#include "drv8313.h"

#include "drivers/driver_err.h"
#include "drivers/gpio/driver_gpio.h"

/* ------------------------------------------------------------------ */
/* Helpers internos                                                    */
/* ------------------------------------------------------------------ */

static bool pin_used(const drv8313_pin_t *p)
{
    return p->port != NULL;
}

static drv8313_err_t from_gpio(driver_err_t err)
{
    return (err == DRIVER_OK) ? DRV8313_OK : DRV8313_ERR_GPIO;
}

static drv8313_err_t check_ready(const drv8313_t *drv)
{
    if (drv == NULL)
        return DRV8313_ERR_INVALID_ARG;
    if (!drv->initialized)
        return DRV8313_ERR_NOT_INITIALIZED;
    return DRV8313_OK;
}

static drv8313_err_t pin_write(const drv8313_pin_t *p, bool level)
{
    return from_gpio(driver_gpio_write_pin(p->port, p->pin,
                                           level ? DRIVER_GPIO_PIN_SET : DRIVER_GPIO_PIN_RESET));
}

/** Configura um pino de saída push-pull já com o nível inicial desejado. */
static drv8313_err_t config_output(const drv8313_pin_t *p, bool initial_level)
{
    driver_err_t err = driver_gpio_enable_clock(p->port);
    if (err != DRIVER_OK)
        return DRV8313_ERR_GPIO;

    drv8313_err_t derr = pin_write(p, initial_level); /* BSRR vale mesmo antes de virar saída */
    if (derr != DRV8313_OK)
        return derr;

    err = driver_gpio_set_mode(p->port, p->pin, DRIVER_GPIO_OUTPUT);
    if (err != DRIVER_OK)
        return DRV8313_ERR_GPIO;

    err = driver_gpio_set_output_type(p->port, p->pin, DRIVER_GPIO_OTYPE_PUSHPULL);
    if (err != DRIVER_OK)
        return DRV8313_ERR_GPIO;

    return from_gpio(driver_gpio_set_speed(p->port, p->pin, DRIVER_GPIO_SPEED_LOW));
}

/** nFAULT é open-drain: entrada com pull-up interno. */
static drv8313_err_t config_fault_input(const drv8313_pin_t *p)
{
    driver_err_t err = driver_gpio_enable_clock(p->port);
    if (err != DRIVER_OK)
        return DRV8313_ERR_GPIO;

    err = driver_gpio_set_mode(p->port, p->pin, DRIVER_GPIO_INPUT);
    if (err != DRIVER_OK)
        return DRV8313_ERR_GPIO;

    return from_gpio(driver_gpio_set_pull(p->port, p->pin, DRIVER_GPIO_PULL_UP));
}

static drv8313_err_t config_pwm_pin(const drv8313_pwm_pin_t *p)
{
    driver_err_t err = driver_gpio_enable_clock(p->port);
    if (err != DRIVER_OK)
        return DRV8313_ERR_GPIO;

    err = driver_gpio_set_alternate_function(p->port, p->pin, p->alternate_function);
    if (err != DRIVER_OK)
        return DRV8313_ERR_GPIO;

    err = driver_gpio_set_output_type(p->port, p->pin, DRIVER_GPIO_OTYPE_PUSHPULL);
    if (err != DRIVER_OK)
        return DRV8313_ERR_GPIO;

    err = driver_gpio_set_speed(p->port, p->pin, DRIVER_GPIO_SPEED_VERY_HIGH);
    if (err != DRIVER_OK)
        return DRV8313_ERR_GPIO;

    return from_gpio(driver_gpio_set_pull(p->port, p->pin, DRIVER_GPIO_PULL_NONE));
}

static bool timer_supported(const TIM_TypeDef *t)
{
    return (t == TIM1) || (t == TIM2) || (t == TIM3) || (t == TIM4) || (t == TIM5);
}

static void timer_clock(const TIM_TypeDef *t, bool on)
{
    volatile uint32_t *reg;
    uint32_t mask;

    if (t == TIM1)
    {
        reg = &RCC->APB2ENR;
        mask = RCC_APB2ENR_TIM1EN;
    }
    else if (t == TIM2)
    {
        reg = &RCC->APB1ENR;
        mask = RCC_APB1ENR_TIM2EN;
    }
    else if (t == TIM3)
    {
        reg = &RCC->APB1ENR;
        mask = RCC_APB1ENR_TIM3EN;
    }
    else if (t == TIM4)
    {
        reg = &RCC->APB1ENR;
        mask = RCC_APB1ENR_TIM4EN;
    }
    else
    {
        reg = &RCC->APB1ENR;
        mask = RCC_APB1ENR_TIM5EN;
    }

    if (on)
        *reg |= mask;
    else
        *reg &= ~mask;
}

/**
 * Center-aligned: (PSC+1) * ARR     = fclk / (2 f)
 * Edge-aligned:   (PSC+1) * (ARR+1) = fclk / f
 */
static drv8313_err_t compute_timebase(uint32_t clock_hz, uint32_t freq_hz, bool center,
                                      uint16_t *psc, uint16_t *arr)
{
    const uint64_t total = center ? ((uint64_t)clock_hz / (2ULL * freq_hz))
                                  : ((uint64_t)clock_hz / freq_hz);
    const uint64_t limit = center ? 65535ULL : 65536ULL;

    if (total < 2U)
        return DRV8313_ERR_OUT_OF_RANGE;

    const uint64_t div = (total + limit - 1U) / limit; /* PSC + 1 */
    if (div > 65536ULL)
        return DRV8313_ERR_OUT_OF_RANGE;

    const uint64_t ticks = total / div;
    const uint64_t a = center ? ticks : (ticks - 1U);
    if (a == 0U || a > 65535ULL)
        return DRV8313_ERR_OUT_OF_RANGE;

    *psc = (uint16_t)(div - 1U);
    *arr = (uint16_t)a;
    return DRV8313_OK;
}

static uint16_t norm_to_ccr(float x, uint16_t arr)
{
    if (!(x > 0.0f)) /* trata NaN e negativos */
        return 0U;
    if (x >= 1.0f)
        return arr;
    return (uint16_t)(x * (float)arr + 0.5f);
}

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */

drv8313_err_t drv8313_init(drv8313_t *drv, const drv8313_config_t *cfg)
{
    if (drv == NULL || cfg == NULL)
        return DRV8313_ERR_INVALID_ARG;

    if (!timer_supported(cfg->timer))
        return DRV8313_ERR_INVALID_TIMER;

    if (cfg->en.port == NULL || cfg->timer_clock_hz == 0U || cfg->pwm_frequency_hz == 0U)
        return DRV8313_ERR_INVALID_ARG;

    for (uint8_t i = 0; i < DRV8313_NUM_PHASES; ++i)
    {
        if (cfg->in[i].port == NULL)
            return DRV8313_ERR_INVALID_ARG;
    }

    if ((pin_used(&cfg->n_reset) || pin_used(&cfg->n_sleep)) && cfg->delay_ms == NULL)
        return DRV8313_ERR_NO_DELAY;

    uint16_t psc = 0;
    uint16_t arr = 0;
    drv8313_err_t err = compute_timebase(cfg->timer_clock_hz, cfg->pwm_frequency_hz,
                                         cfg->center_aligned, &psc, &arr);
    if (err != DRV8313_OK)
        return err;

    drv->initialized = false;
    drv->cfg = *cfg;
    drv->arr = arr;
    drv->enabled = false;
    drv->sleeping = false;

    /* Pinos de controle: EN primeiro e em nível baixo, para o driver nascer desabilitado */
    err = config_output(&cfg->en, false);
    if (err != DRV8313_OK)
        return err;

    if (pin_used(&cfg->n_reset))
    {
        err = config_output(&cfg->n_reset, true); /* sem reset */
        if (err != DRV8313_OK)
            return err;
    }

    if (pin_used(&cfg->n_sleep))
    {
        err = config_output(&cfg->n_sleep, true); /* acordado */
        if (err != DRV8313_OK)
            return err;
    }

    if (pin_used(&cfg->n_fault))
    {
        err = config_fault_input(&cfg->n_fault);
        if (err != DRV8313_OK)
            return err;
    }

    /* Pinos de PWM */
    for (uint8_t i = 0; i < DRV8313_NUM_PHASES; ++i)
    {
        err = config_pwm_pin(&cfg->in[i]);
        if (err != DRV8313_OK)
            return err;
    }

    /* Timer: PWM modo 1 em CH1..CH3, preload, polaridade ativa em alto */
    TIM_TypeDef *t = cfg->timer;

    timer_clock(t, true);
    (void)RCC->APB1ENR;

    t->CR1 = 0;
    t->CR2 = 0;
    t->DIER = 0;

    t->PSC = psc;
    t->ARR = arr;

    t->CCMR1 = (0x6UL << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE |
               (0x6UL << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    t->CCMR2 = (0x6UL << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;

    t->CCR1 = 0;
    t->CCR2 = 0;
    t->CCR3 = 0;

    t->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;

    if (t == TIM1)
        t->BDTR = TIM_BDTR_MOE; /* o TIM1 só emite PWM com MOE = 1 */

    t->CR1 = TIM_CR1_ARPE | (cfg->center_aligned ? (1UL << TIM_CR1_CMS_Pos) : 0UL);

    t->EGR = TIM_EGR_UG;
    t->SR = 0;

    t->CR1 |= TIM_CR1_CEN;

    drv->initialized = true;
    return DRV8313_OK;
}

drv8313_err_t drv8313_deinit(drv8313_t *drv)
{
    drv8313_err_t err = check_ready(drv);
    if (err != DRV8313_OK)
        return err;

    (void)pin_write(&drv->cfg.en, false);

    TIM_TypeDef *t = drv->cfg.timer;

    t->CR1 &= ~TIM_CR1_CEN;
    t->CCER = 0;
    if (t == TIM1)
        t->BDTR &= ~TIM_BDTR_MOE;

    timer_clock(t, false);

    drv->enabled = false;
    drv->initialized = false;
    return DRV8313_OK;
}

drv8313_err_t drv8313_enable(drv8313_t *drv)
{
    drv8313_err_t err = check_ready(drv);
    if (err != DRV8313_OK)
        return err;

    if (drv->sleeping)
        return DRV8313_ERR_ASLEEP;

    if (pin_used(&drv->cfg.n_fault))
    {
        bool fault = false;
        err = drv8313_get_fault(drv, &fault);
        if (err != DRV8313_OK)
            return err;
        if (fault)
            return DRV8313_ERR_FAULT;
    }

    err = pin_write(&drv->cfg.en, true);
    if (err != DRV8313_OK)
        return err;

    drv->enabled = true;
    return DRV8313_OK;
}

drv8313_err_t drv8313_disable(drv8313_t *drv)
{
    drv8313_err_t err = check_ready(drv);
    if (err != DRV8313_OK)
        return err;

    err = pin_write(&drv->cfg.en, false);
    if (err != DRV8313_OK)
        return err;

    drv->enabled = false;
    return DRV8313_OK;
}

drv8313_err_t drv8313_sleep(drv8313_t *drv)
{
    drv8313_err_t err = check_ready(drv);
    if (err != DRV8313_OK)
        return err;

    if (!pin_used(&drv->cfg.n_sleep))
        return DRV8313_ERR_NOT_CONFIGURED;

    err = drv8313_disable(drv);
    if (err != DRV8313_OK)
        return err;

    err = pin_write(&drv->cfg.n_sleep, false);
    if (err != DRV8313_OK)
        return err;

    drv->sleeping = true;
    return DRV8313_OK;
}

drv8313_err_t drv8313_wake(drv8313_t *drv)
{
    drv8313_err_t err = check_ready(drv);
    if (err != DRV8313_OK)
        return err;

    if (!pin_used(&drv->cfg.n_sleep))
        return DRV8313_ERR_NOT_CONFIGURED;

    if (!drv->sleeping)
        return DRV8313_OK;

    err = pin_write(&drv->cfg.n_sleep, true);
    if (err != DRV8313_OK)
        return err;

    drv->cfg.delay_ms(DRV8313_WAKE_DELAY_MS);

    drv->sleeping = false;
    return DRV8313_OK;
}

drv8313_err_t drv8313_reset(drv8313_t *drv)
{
    drv8313_err_t err = check_ready(drv);
    if (err != DRV8313_OK)
        return err;

    if (!pin_used(&drv->cfg.n_reset))
        return DRV8313_ERR_NOT_CONFIGURED;

    if (drv->sleeping)
        return DRV8313_ERR_ASLEEP;

    err = drv8313_disable(drv);
    if (err != DRV8313_OK)
        return err;

    err = pin_write(&drv->cfg.n_reset, false);
    if (err != DRV8313_OK)
        return err;

    drv->cfg.delay_ms(DRV8313_RESET_PULSE_MS);

    err = pin_write(&drv->cfg.n_reset, true);
    if (err != DRV8313_OK)
        return err;

    drv->cfg.delay_ms(DRV8313_WAKE_DELAY_MS);

    return DRV8313_OK;
}

drv8313_err_t drv8313_get_fault(const drv8313_t *drv, bool *fault)
{
    if (drv != NULL && fault == NULL)
        return DRV8313_ERR_INVALID_ARG;

    drv8313_err_t err = check_ready(drv);
    if (err != DRV8313_OK)
        return err;

    if (fault == NULL)
        return DRV8313_ERR_INVALID_ARG;

    if (!pin_used(&drv->cfg.n_fault))
        return DRV8313_ERR_NOT_CONFIGURED;

    *fault = (driver_gpio_read_pin(drv->cfg.n_fault.port, drv->cfg.n_fault.pin) == DRIVER_GPIO_PIN_RESET); /* nFAULT é ativo em nível baixo */
    return DRV8313_OK;
}

drv8313_err_t drv8313_set_duty_cycle(drv8313_t *drv, const drv8313_duty_t *duty)
{
    drv8313_err_t err = check_ready(drv);
    if (err != DRV8313_OK)
        return err;

    if (duty == NULL)
        return DRV8313_ERR_INVALID_ARG;

    TIM_TypeDef *t = drv->cfg.timer;

    t->CCR1 = norm_to_ccr(duty->a, drv->arr);
    t->CCR2 = norm_to_ccr(duty->b, drv->arr);
    t->CCR3 = norm_to_ccr(duty->c, drv->arr);

    return DRV8313_OK;
}

bool drv8313_is_enabled(const drv8313_t *drv)
{
    return (drv != NULL) && drv->initialized && drv->enabled;
}