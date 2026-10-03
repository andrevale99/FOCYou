#include "inverter.h"

/* ------------------------------------------------------------------ */
/* Helpers internos                                                    */
/* ------------------------------------------------------------------ */

#define INV_LOCK_LEVEL(t) ((uint32_t)(((t)->BDTR & TIM_BDTR_LOCK_Msk) >> TIM_BDTR_LOCK_Pos))

static const uint32_t k_cce_high[MAX_CHN_INVERTER] = {TIM_CCER_CC1E, TIM_CCER_CC2E, TIM_CCER_CC3E};
static const uint32_t k_cce_low[MAX_CHN_INVERTER] = {TIM_CCER_CC1NE, TIM_CCER_CC2NE, TIM_CCER_CC3NE};
static const uint32_t k_ccp_high[MAX_CHN_INVERTER] = {TIM_CCER_CC1P, TIM_CCER_CC2P, TIM_CCER_CC3P};
static const uint32_t k_ccp_low[MAX_CHN_INVERTER] = {TIM_CCER_CC1NP, TIM_CCER_CC2NP, TIM_CCER_CC3NP};
static const uint32_t k_ois_high[MAX_CHN_INVERTER] = {TIM_CR2_OIS1, TIM_CR2_OIS2, TIM_CR2_OIS3};
static const uint32_t k_ois_low[MAX_CHN_INVERTER] = {TIM_CR2_OIS1N, TIM_CR2_OIS2N, TIM_CR2_OIS3N};

static inverter_error_t check_ready(const inverter_t *inv)
{
    if (inv == NULL || inv->timer == NULL)
        return INVERTER_ERROR_INVALID_ARGUMENT;
    if (!inv->initialized)
        return INVERTER_ERROR_NOT_INITIALIZED;
    return INVERTER_OK;
}

static bool is_center_aligned(const TIM_TypeDef *t)
{
    return (t->CR1 & TIM_CR1_CMS_Msk) != 0U;
}

static void write_ocm(TIM_TypeDef *t, uint8_t ch, uint32_t mode)
{
    switch (ch)
    {
    case INVERTER_CH_A:
        t->CCMR1 = (t->CCMR1 & ~TIM_CCMR1_OC1M_Msk) | (mode << TIM_CCMR1_OC1M_Pos);
        break;
    case INVERTER_CH_B:
        t->CCMR1 = (t->CCMR1 & ~TIM_CCMR1_OC2M_Msk) | (mode << TIM_CCMR1_OC2M_Pos);
        break;
    default:
        t->CCMR2 = (t->CCMR2 & ~TIM_CCMR2_OC3M_Msk) | (mode << TIM_CCMR2_OC3M_Pos);
        break;
    }
}

static void apply_polarity(TIM_TypeDef *t, inverter_polarity_t high, inverter_polarity_t low)
{
    uint32_t ccer = t->CCER;

    for (uint8_t i = 0; i < MAX_CHN_INVERTER; ++i)
    {
        ccer &= ~(k_ccp_high[i] | k_ccp_low[i]);
        if (high == INVERTER_POLARITY_ACTIVE_LOW)
            ccer |= k_ccp_high[i];
        if (low == INVERTER_POLARITY_ACTIVE_LOW)
            ccer |= k_ccp_low[i];
    }
    t->CCER = ccer;
}

static void apply_idle(TIM_TypeDef *t, const inverter_idle_config_t *cfg)
{
    uint32_t cr2 = t->CR2;

    for (uint8_t i = 0; i < MAX_CHN_INVERTER; ++i)
    {
        cr2 &= ~(k_ois_high[i] | k_ois_low[i]);
        if (cfg->high_idle_level[i])
            cr2 |= k_ois_high[i];
        if (cfg->low_idle_level[i])
            cr2 |= k_ois_low[i];
    }
    t->CR2 = cr2;

    uint32_t bdtr = t->BDTR & ~(TIM_BDTR_OSSR | TIM_BDTR_OSSI);
    if (cfg->off_state_run)
        bdtr |= TIM_BDTR_OSSR;
    if (cfg->off_state_idle)
        bdtr |= TIM_BDTR_OSSI;
    t->BDTR = bdtr;
}

static void apply_break(TIM_TypeDef *t, const inverter_break_config_t *cfg)
{
    uint32_t bdtr = t->BDTR & ~(TIM_BDTR_BKE | TIM_BDTR_BKP | TIM_BDTR_AOE);

    if (cfg->enable)
        bdtr |= TIM_BDTR_BKE;
    if (cfg->active_high)
        bdtr |= TIM_BDTR_BKP;
    if (cfg->automatic_output)
        bdtr |= TIM_BDTR_AOE;
    t->BDTR = bdtr;
}

/* ------------------------------------------------------------------ */
/* Ciclo de vida                                                       */
/* ------------------------------------------------------------------ */

inverter_error_t inverter_init(inverter_t *inv, const inverter_config_t *config)
{
    if (inv == NULL || config == NULL)
        return INVERTER_ERROR_INVALID_ARGUMENT;

    if (config->advanced_timer != INVERTER_TIMER ||
        config->gpioH == NULL || config->gpioL == NULL)
        return INVERTER_ERROR_INVALID_ARGUMENT;

    if (config->autorreload == 0U || config->timer_clock_hz == 0U ||
        (uint32_t)config->alignment > (uint32_t)INVERTER_ALIGN_CENTER_3)
        return INVERTER_ERROR_INVALID_ARGUMENT;

    TIM_TypeDef *const timer = config->advanced_timer;

    if (driver_gpio_enable_clock(config->gpioH) != DRIVER_GPIO_OK ||
        driver_gpio_enable_clock(config->gpioL) != DRIVER_GPIO_OK)
        return INVERTER_ERROR_INVALID_ARGUMENT;

    for (uint8_t idx = 0; idx < MAX_CHN_INVERTER; ++idx)
    {
        const inverter_gpio_config_t *cfgH = &config->config_gpioH[idx];
        const inverter_gpio_config_t *cfgL = &config->config_gpioL[idx];

        if (driver_gpio_set_alternate_function(config->gpioH, cfgH->pin, cfgH->alternate_function) != DRIVER_GPIO_OK ||
            driver_gpio_set_alternate_function(config->gpioL, cfgL->pin, cfgL->alternate_function) != DRIVER_GPIO_OK)
            return INVERTER_ERROR_INVALID_ARGUMENT;

        if (driver_gpio_set_output_type(config->gpioH, cfgH->pin, DRIVER_GPIO_OTYPE_PUSHPULL) != DRIVER_GPIO_OK ||
            driver_gpio_set_output_type(config->gpioL, cfgL->pin, DRIVER_GPIO_OTYPE_PUSHPULL) != DRIVER_GPIO_OK)
            return INVERTER_ERROR_INVALID_ARGUMENT;

        if (driver_gpio_set_speed(config->gpioH, cfgH->pin, DRIVER_GPIO_SPEED_VERY_HIGH) != DRIVER_GPIO_OK ||
            driver_gpio_set_speed(config->gpioL, cfgL->pin, DRIVER_GPIO_SPEED_VERY_HIGH) != DRIVER_GPIO_OK)
            return INVERTER_ERROR_INVALID_ARGUMENT;

        if (driver_gpio_set_pull(config->gpioH, cfgH->pin, DRIVER_GPIO_PULL_NONE) != DRIVER_GPIO_OK ||
            driver_gpio_set_pull(config->gpioL, cfgL->pin, DRIVER_GPIO_PULL_NONE) != DRIVER_GPIO_OK)
            return INVERTER_ERROR_INVALID_ARGUMENT;
    }

    /* Clock + reset do timer (também limpa um LOCK anterior do BDTR) */
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    (void)RCC->APB2ENR;
    RCC->APB2RSTR |= RCC_APB2RSTR_TIM1RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM1RST;

    /* Base de tempo */
    timer->PSC = config->prescale;
    timer->ARR = config->autorreload;
    timer->RCR = config->repetition_counter;

    timer->CR1 = TIM_CR1_ARPE | ((uint32_t)config->alignment << TIM_CR1_CMS_Pos);

    /* PWM1 + preload nos 3 canais */
    for (uint8_t ch = 0; ch < MAX_CHN_INVERTER; ++ch)
        write_ocm(timer, ch, INVERTER_OUT_PWM1);

    timer->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
    timer->CCMR2 |= TIM_CCMR2_OC3PE;

    /* Duty zerado antes de habilitar as saídas */
    timer->CCR1 = 0;
    timer->CCR2 = 0;
    timer->CCR3 = 0;

    /* Polaridade e habilitação dos canais (MOE ainda desligado) */
    apply_polarity(timer, config->polarity_high, config->polarity_low);
    timer->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E |
                    TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE);

    /* Idle state, break e dead-time. MOE permanece em 0. */
    apply_idle(timer, &config->idle);
    apply_break(timer, &config->brk);

    timer->BDTR = (timer->BDTR & ~(TIM_BDTR_DTG_Msk | TIM_BDTR_MOE)) |
                  (((uint32_t)config->deadtime << TIM_BDTR_DTG_Pos) & TIM_BDTR_DTG_Msk);

    timer->DIER = 0;

    /* Carrega os registradores de preload */
    timer->EGR = TIM_EGR_UG;
    timer->SR = 0;

    inv->timer = timer;
    inv->timer_clock_hz = config->timer_clock_hz;
    inv->autorreload = config->autorreload;
    inv->prescale = config->prescale;
    inv->deadtime = config->deadtime;
    inv->alignment = config->alignment;
    inv->callbacks.on_update = NULL;
    inv->callbacks.on_break = NULL;
    inv->callbacks.context = NULL;
    inv->initialized = true;

    return INVERTER_OK;
}

inverter_error_t inverter_deinit(inverter_t *inv)
{
    inverter_error_t err = check_ready(inv);
    if (err != INVERTER_OK)
        return err;

    TIM_TypeDef *t = inv->timer;

    t->BDTR &= ~TIM_BDTR_MOE;
    t->CR1 &= ~TIM_CR1_CEN;
    t->DIER = 0;

    NVIC_DisableIRQ(TIM1_BRK_TIM9_IRQn);

    RCC->APB2RSTR |= RCC_APB2RSTR_TIM1RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM1RST;
    RCC->APB2ENR &= ~RCC_APB2ENR_TIM1EN;

    inv->initialized = false;
    return INVERTER_OK;
}

inverter_error_t inverter_start(inverter_t *inv)
{
    inverter_error_t err = check_ready(inv);
    if (err != INVERTER_OK)
        return err;

    inv->timer->CR1 |= TIM_CR1_CEN;
    return INVERTER_OK;
}

inverter_error_t inverter_stop(inverter_t *inv)
{
    inverter_error_t err = check_ready(inv);
    if (err != INVERTER_OK)
        return err;

    inv->timer->BDTR &= ~TIM_BDTR_MOE;
    inv->timer->CR1 &= ~TIM_CR1_CEN;
    return INVERTER_OK;
}

/* ------------------------------------------------------------------ */
/* Estado das saídas                                                   */
/* ------------------------------------------------------------------ */

inverter_error_t inverter_set_state(inverter_t *inv, inverter_state_t state)
{
    inverter_error_t err = check_ready(inv);
    if (err != INVERTER_OK)
        return err;

    switch (state)
    {
    case INVERTER_STATE_ON:
        if (inv->timer->SR & TIM_SR_BIF)
            return INVERTER_ERROR_FAULT; /* limpar a falha antes de religar */
        inv->timer->BDTR |= TIM_BDTR_MOE;
        break;

    case INVERTER_STATE_OFF:
        inv->timer->BDTR &= ~TIM_BDTR_MOE;
        break;

    default:
        return INVERTER_ERROR_NO_STATE;
    }

    return INVERTER_OK;
}

inverter_state_t inverter_get_state(const inverter_t *inv)
{
    if (inv == NULL || inv->timer == NULL)
        return INVERTER_STATE_OFF;

    return (inv->timer->BDTR & TIM_BDTR_MOE) ? INVERTER_STATE_ON : INVERTER_STATE_OFF;
}

/* ------------------------------------------------------------------ */
/* Frequência / dead-time / duty                                       */
/* ------------------------------------------------------------------ */

inverter_error_t inverter_set_frequency(inverter_t *inv, uint32_t frequency_hz)
{
    inverter_error_t err = check_ready(inv);
    if (err != INVERTER_OK)
        return err;
    if (frequency_hz == 0U)
        return INVERTER_ERROR_INVALID_ARGUMENT;

    TIM_TypeDef *t = inv->timer;
    const bool center = is_center_aligned(t);

    /* center: (PSC+1)*ARR = fclk/(2f)   |   edge: (PSC+1)*(ARR+1) = fclk/f */
    const uint64_t total = center ? ((uint64_t)inv->timer_clock_hz / (2ULL * frequency_hz))
                                  : ((uint64_t)inv->timer_clock_hz / frequency_hz);
    const uint64_t limit = center ? 65535ULL : 65536ULL;

    if (total < 2U)
        return INVERTER_ERROR_OUT_OF_RANGE;

    const uint64_t div = (total + limit - 1U) / limit; /* PSC + 1 */
    if (div > 65536ULL)
        return INVERTER_ERROR_OUT_OF_RANGE;

    const uint64_t ticks = total / div;
    const uint64_t arr = center ? ticks : (ticks - 1U);
    if (arr == 0U || arr > 65535ULL)
        return INVERTER_ERROR_OUT_OF_RANGE;

    t->PSC = (uint16_t)(div - 1U);
    t->ARR = (uint16_t)arr;
    inv->prescale = (uint16_t)(div - 1U);
    inv->autorreload = (uint16_t)arr;

    /* Com o timer parado, aplica na hora; rodando, vale no próximo update. */
    if (!(t->CR1 & TIM_CR1_CEN))
    {
        t->EGR = TIM_EGR_UG;
        t->SR = ~TIM_SR_UIF;
    }

    return INVERTER_OK;
}

inverter_error_t inverter_get_frequency(const inverter_t *inv, uint32_t *frequency_hz)
{
    inverter_error_t err = check_ready(inv);
    if (err != INVERTER_OK)
        return err;
    if (frequency_hz == NULL)
        return INVERTER_ERROR_INVALID_ARGUMENT;

    const TIM_TypeDef *t = inv->timer;
    const uint64_t psc1 = (uint64_t)t->PSC + 1U;
    const uint64_t arr = t->ARR;

    if (arr == 0U)
        return INVERTER_ERROR_OUT_OF_RANGE;

    const uint64_t denom = is_center_aligned(t) ? (2ULL * psc1 * arr) : (psc1 * (arr + 1U));
    *frequency_hz = (uint32_t)((uint64_t)inv->timer_clock_hz / denom);

    return INVERTER_OK;
}

inverter_error_t inverter_set_deadtime(inverter_t *inv, uint8_t dtg)
{
    inverter_error_t err = check_ready(inv);
    if (err != INVERTER_OK)
        return err;

    if (INV_LOCK_LEVEL(inv->timer) >= 1U)
        return INVERTER_ERROR_LOCKED;

    inv->timer->BDTR = (inv->timer->BDTR & ~TIM_BDTR_DTG_Msk) |
                       (((uint32_t)dtg << TIM_BDTR_DTG_Pos) & TIM_BDTR_DTG_Msk);
    inv->deadtime = dtg;

    return INVERTER_OK;
}

inverter_error_t inverter_set_deadtime_ns(inverter_t *inv, uint32_t deadtime_ns)
{
    inverter_error_t err = check_ready(inv);
    if (err != INVERTER_OK)
        return err;

    /* tDTS = 1/fclk (CKD = 0) */
    const uint64_t ticks = ((uint64_t)deadtime_ns * inv->timer_clock_hz + 500000000ULL) / 1000000000ULL;
    uint8_t dtg;

    if (ticks <= 127U)
    {
        dtg = (uint8_t)ticks; /* 0xx: DT = DTG * tDTS */
    }
    else if (ticks <= 254U)
    {
        dtg = (uint8_t)(0x80U | ((ticks / 2U) - 64U)); /* 10x: (64+DTG[5:0]) * 2 tDTS */
    }
    else if (ticks <= 504U)
    {
        uint64_t v = ticks / 8U; /* 110: (32+DTG[4:0]) * 8 tDTS */
        if (v < 32U)
            v = 32U;
        dtg = (uint8_t)(0xC0U | (v - 32U));
    }
    else if (ticks <= 1008U)
    {
        uint64_t v = ticks / 16U; /* 111: (32+DTG[4:0]) * 16 tDTS */
        if (v < 32U)
            v = 32U;
        dtg = (uint8_t)(0xE0U | (v - 32U));
    }
    else
    {
        return INVERTER_ERROR_OUT_OF_RANGE;
    }

    return inverter_set_deadtime(inv, dtg);
}

inverter_error_t inverter_set_duty_cycle(inverter_t *inv, const inverter_duty_t *duty)
{
    inverter_error_t err = check_ready(inv);
    if (err != INVERTER_OK)
        return err;
    if (duty == NULL)
        return INVERTER_ERROR_INVALID_ARGUMENT;

    TIM_TypeDef *t = inv->timer;
    const uint32_t arr = t->ARR;

    if (duty->a > arr || duty->b > arr || duty->c > arr)
        return INVERTER_ERROR_OUT_OF_RANGE;

    t->CCR1 = duty->a;
    t->CCR2 = duty->b;
    t->CCR3 = duty->c;

    return INVERTER_OK;
}

static uint16_t norm_to_ccr(float x, uint16_t arr)
{
    if (!(x > 0.0f)) /* trata NaN e negativos */
        return 0U;
    if (x >= 1.0f)
        return arr;
    return (uint16_t)(x * (float)arr + 0.5f);
}

inverter_error_t inverter_set_duty_cycle_normalized(inverter_t *inv, const inverter_duty_norm_t *duty)
{
    inverter_error_t err = check_ready(inv);
    if (err != INVERTER_OK)
        return err;
    if (duty == NULL)
        return INVERTER_ERROR_INVALID_ARGUMENT;

    TIM_TypeDef *t = inv->timer;
    const uint16_t arr = (uint16_t)t->ARR;

    t->CCR1 = norm_to_ccr(duty->a, arr);
    t->CCR2 = norm_to_ccr(duty->b, arr);
    t->CCR3 = norm_to_ccr(duty->c, arr);

    return INVERTER_OK;
}

inverter_error_t inverter_get_duty_cycle(const inverter_t *inv, inverter_duty_t *duty)
{
    inverter_error_t err = check_ready(inv);
    if (err != INVERTER_OK)
        return err;
    if (duty == NULL)
        return INVERTER_ERROR_INVALID_ARGUMENT;

    duty->a = (uint16_t)inv->timer->CCR1;
    duty->b = (uint16_t)inv->timer->CCR2;
    duty->c = (uint16_t)inv->timer->CCR3;

    return INVERTER_OK;
}

/* ------------------------------------------------------------------ */
/* Proteção                                                            */
/* ------------------------------------------------------------------ */

inverter_error_t inverter_config_break(inverter_t *inv, const inverter_break_config_t *cfg)
{
    inverter_error_t err = check_ready(inv);
    if (err != INVERTER_OK)
        return err;
    if (cfg == NULL)
        return INVERTER_ERROR_INVALID_ARGUMENT;
    if (INV_LOCK_LEVEL(inv->timer) >= 1U)
        return INVERTER_ERROR_LOCKED;

    apply_break(inv->timer, cfg);
    return INVERTER_OK;
}

bool inverter_get_fault(const inverter_t *inv)
{
    if (inv == NULL || inv->timer == NULL)
        return false;

    return (inv->timer->SR & TIM_SR_BIF) != 0U;
}

inverter_error_t inverter_clear_fault(inverter_t *inv)
{
    inverter_error_t err = check_ready(inv);
    if (err != INVERTER_OK)
        return err;

    inv->timer->SR = ~TIM_SR_BIF;

    /* Se a entrada de break ainda está ativa, o flag volta a ser setado */
    if (inv->timer->SR & TIM_SR_BIF)
        return INVERTER_ERROR_FAULT;

    return INVERTER_OK;
}

inverter_error_t inverter_set_idle_state(inverter_t *inv, const inverter_idle_config_t *cfg)
{
    inverter_error_t err = check_ready(inv);
    if (err != INVERTER_OK)
        return err;
    if (cfg == NULL)
        return INVERTER_ERROR_INVALID_ARGUMENT;
    if (INV_LOCK_LEVEL(inv->timer) >= 1U)
        return INVERTER_ERROR_LOCKED;

    apply_idle(inv->timer, cfg);
    return INVERTER_OK;
}

inverter_error_t inverter_lock_config(inverter_t *inv, inverter_lock_level_t level)
{
    inverter_error_t err = check_ready(inv);
    if (err != INVERTER_OK)
        return err;
    if ((uint32_t)level > (uint32_t)INVERTER_LOCK_LEVEL3)
        return INVERTER_ERROR_INVALID_ARGUMENT;
    if (level == INVERTER_LOCK_OFF)
        return INVERTER_OK;

    TIM_TypeDef *t = inv->timer;

    /* LOCK só pode ser escrito uma vez após o reset */
    if (INV_LOCK_LEVEL(t) != 0U)
        return INVERTER_ERROR_LOCKED;

    t->BDTR = (t->BDTR & ~TIM_BDTR_LOCK_Msk) | ((uint32_t)level << TIM_BDTR_LOCK_Pos);

    return (INV_LOCK_LEVEL(t) == (uint32_t)level) ? INVERTER_OK : INVERTER_ERROR_LOCKED;
}

/* ------------------------------------------------------------------ */
/* Sincronismo com ADC / malha de controle                             */
/* ------------------------------------------------------------------ */

inverter_error_t inverter_config_adc_trigger(inverter_t *inv, const inverter_adc_trigger_config_t *cfg)
{
    inverter_error_t err = check_ready(inv);
    if (err != INVERTER_OK)
        return err;
    if (cfg == NULL)
        return INVERTER_ERROR_INVALID_ARGUMENT;

    TIM_TypeDef *t = inv->timer;

    if (!cfg->enable)
    {
        t->CR2 &= ~TIM_CR2_MMS_Msk; /* TRGO = reset */
        t->CCR4 = 0;
        return INVERTER_OK;
    }

    if (cfg->compare_value > t->ARR)
        return INVERTER_ERROR_OUT_OF_RANGE;

    /* CH4 sem saída no pino (CC4E = 0): serve só como evento interno */
    t->CCER &= ~TIM_CCER_CC4E;
    t->CCMR2 = (t->CCMR2 & ~TIM_CCMR2_OC4M_Msk) | (0x6UL << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;
    t->CCR4 = cfg->compare_value;

    t->CR2 &= ~TIM_CR2_MMS_Msk;
    if (cfg->use_trgo)
        t->CR2 |= (0x7UL << TIM_CR2_MMS_Pos); /* OC4REF -> TRGO */

    return INVERTER_OK;
}

inverter_error_t inverter_set_repetition_counter(inverter_t *inv, uint8_t rcr)
{
    inverter_error_t err = check_ready(inv);
    if (err != INVERTER_OK)
        return err;

    inv->timer->RCR = rcr; /* tem preload: vale no próximo update */
    return INVERTER_OK;
}

inverter_error_t inverter_set_callbacks(inverter_t *inv, const inverter_callbacks_t *callbacks)
{
    inverter_error_t err = check_ready(inv);
    if (err != INVERTER_OK)
        return err;

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (callbacks == NULL)
    {
        inv->callbacks.on_update = NULL;
        inv->callbacks.on_break = NULL;
        inv->callbacks.context = NULL;
    }
    else
    {
        inv->callbacks = *callbacks;
    }

    __set_PRIMASK(primask);
    return INVERTER_OK;
}

inverter_error_t inverter_enable_update_irq(inverter_t *inv, bool enable)
{
    inverter_error_t err = check_ready(inv);
    if (err != INVERTER_OK)
        return err;

    if (enable)
    {
        inv->timer->SR = ~TIM_SR_UIF;
        inv->timer->DIER |= TIM_DIER_UIE;
        NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
    }
    else
    {
        /* NVIC não é desabilitado: o vetor é compartilhado com o TIM10 */
        inv->timer->DIER &= ~TIM_DIER_UIE;
    }

    return INVERTER_OK;
}

inverter_error_t inverter_enable_break_irq(inverter_t *inv, bool enable)
{
    inverter_error_t err = check_ready(inv);
    if (err != INVERTER_OK)
        return err;

    if (enable)
    {
        inv->timer->DIER |= TIM_DIER_BIE;
        NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
    }
    else
    {
        inv->timer->DIER &= ~TIM_DIER_BIE;
    }

    return INVERTER_OK;
}

void inverter_irq_update_handler(inverter_t *inv)
{
    if (inv == NULL || inv->timer == NULL)
        return;

    TIM_TypeDef *t = inv->timer;

    if ((t->SR & TIM_SR_UIF) && (t->DIER & TIM_DIER_UIE))
    {
        t->SR = ~TIM_SR_UIF;

        if (inv->callbacks.on_update != NULL)
            inv->callbacks.on_update(inv->callbacks.context);
    }
}

void inverter_irq_break_handler(inverter_t *inv)
{
    if (inv == NULL || inv->timer == NULL)
        return;

    TIM_TypeDef *t = inv->timer;

    if ((t->SR & TIM_SR_BIF) && (t->DIER & TIM_DIER_BIE))
    {
        /* O flag BIF é mantido (para inverter_get_fault) e a IRQ é desabilitada
         * para não ficar re-disparando. Após inverter_clear_fault(), religue
         * com inverter_enable_break_irq(inv, true). */
        t->DIER &= ~TIM_DIER_BIE;

        if (inv->callbacks.on_break != NULL)
            inv->callbacks.on_break(inv->callbacks.context);
    }
}

/* ------------------------------------------------------------------ */
/* Flexibilidade                                                       */
/* ------------------------------------------------------------------ */

inverter_error_t inverter_set_channel_enable(inverter_t *inv, inverter_channel_t ch, bool high, bool low)
{
    inverter_error_t err = check_ready(inv);
    if (err != INVERTER_OK)
        return err;
    if ((uint32_t)ch >= MAX_CHN_INVERTER)
        return INVERTER_ERROR_INVALID_ARGUMENT;

    uint32_t ccer = inv->timer->CCER & ~(k_cce_high[ch] | k_cce_low[ch]);
    if (high)
        ccer |= k_cce_high[ch];
    if (low)
        ccer |= k_cce_low[ch];
    inv->timer->CCER = ccer;

    return INVERTER_OK;
}

inverter_error_t inverter_set_polarity(inverter_t *inv, inverter_polarity_t high, inverter_polarity_t low)
{
    inverter_error_t err = check_ready(inv);
    if (err != INVERTER_OK)
        return err;
    if (INV_LOCK_LEVEL(inv->timer) >= 2U)
        return INVERTER_ERROR_LOCKED;

    apply_polarity(inv->timer, high, low);
    return INVERTER_OK;
}

inverter_error_t inverter_set_alignment(inverter_t *inv, inverter_align_t alignment)
{
    inverter_error_t err = check_ready(inv);
    if (err != INVERTER_OK)
        return err;
    if ((uint32_t)alignment > (uint32_t)INVERTER_ALIGN_CENTER_3)
        return INVERTER_ERROR_INVALID_ARGUMENT;

    TIM_TypeDef *t = inv->timer;

    /* Não é permitido trocar de edge para center com o contador rodando */
    if (t->CR1 & TIM_CR1_CEN)
        return INVERTER_ERROR_INVALID_ARGUMENT;

    t->CR1 = (t->CR1 & ~(TIM_CR1_CMS_Msk | TIM_CR1_DIR)) | ((uint32_t)alignment << TIM_CR1_CMS_Pos);
    inv->alignment = alignment;

    /* A frequência muda com o alinhamento: recalcule com inverter_set_frequency() */
    return INVERTER_OK;
}

inverter_error_t inverter_set_output_mode(inverter_t *inv, inverter_channel_t ch, inverter_output_mode_t mode)
{
    inverter_error_t err = check_ready(inv);
    if (err != INVERTER_OK)
        return err;
    if ((uint32_t)ch >= MAX_CHN_INVERTER)
        return INVERTER_ERROR_INVALID_ARGUMENT;
    if (mode != INVERTER_OUT_PWM1 && mode != INVERTER_OUT_PWM2 &&
        mode != INVERTER_OUT_FORCE_ACTIVE && mode != INVERTER_OUT_FORCE_INACTIVE)
        return INVERTER_ERROR_INVALID_ARGUMENT;
    if (INV_LOCK_LEVEL(inv->timer) >= 3U)
        return INVERTER_ERROR_LOCKED;

    write_ocm(inv->timer, (uint8_t)ch, (uint32_t)mode);
    return INVERTER_OK;
}

inverter_error_t inverter_force_output(inverter_t *inv, inverter_channel_t ch, bool active)
{
    /* Para voltar ao PWM use inverter_set_output_mode(inv, ch, INVERTER_OUT_PWM1) */
    return inverter_set_output_mode(inv, ch, active ? INVERTER_OUT_FORCE_ACTIVE : INVERTER_OUT_FORCE_INACTIVE);
}