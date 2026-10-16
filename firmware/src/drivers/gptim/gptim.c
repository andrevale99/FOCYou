#include "gptim.h"

/* ------------------------------------------------------------------ */
/* Helpers internos                                                    */
/* ------------------------------------------------------------------ */

/** Dados de cada instância suportada. */
typedef struct
{
    volatile uint32_t *enr;  /* registrador de clock (APB1ENR/APB2ENR) */
    volatile uint32_t *rstr; /* registrador de reset (APB1RSTR/APB2RSTR) */
    uint32_t bit;            /* mesma posição nos dois registradores */
    IRQn_Type irq;
    uint32_t max_arr;
    bool shared_irq;
} gptim_info_t;

static bool get_info(const TIM_TypeDef *t, gptim_info_t *info)
{
    if (t == TIM2)
        *info = (gptim_info_t){&RCC->APB1ENR, &RCC->APB1RSTR, RCC_APB1ENR_TIM2EN, TIM2_IRQn, 0xFFFFFFFFU, false};
    else if (t == TIM3)
        *info = (gptim_info_t){&RCC->APB1ENR, &RCC->APB1RSTR, RCC_APB1ENR_TIM3EN, TIM3_IRQn, 0xFFFFU, false};
    else if (t == TIM4)
        *info = (gptim_info_t){&RCC->APB1ENR, &RCC->APB1RSTR, RCC_APB1ENR_TIM4EN, TIM4_IRQn, 0xFFFFU, false};
    else if (t == TIM5)
        *info = (gptim_info_t){&RCC->APB1ENR, &RCC->APB1RSTR, RCC_APB1ENR_TIM5EN, TIM5_IRQn, 0xFFFFFFFFU, false};
    else if (t == TIM9)
        *info = (gptim_info_t){&RCC->APB2ENR, &RCC->APB2RSTR, RCC_APB2ENR_TIM9EN, TIM1_BRK_TIM9_IRQn, 0xFFFFU, true};
    else if (t == TIM10)
        *info = (gptim_info_t){&RCC->APB2ENR, &RCC->APB2RSTR, RCC_APB2ENR_TIM10EN, TIM1_UP_TIM10_IRQn, 0xFFFFU, true};
    else if (t == TIM11)
        *info = (gptim_info_t){&RCC->APB2ENR, &RCC->APB2RSTR, RCC_APB2ENR_TIM11EN, TIM1_TRG_COM_TIM11_IRQn, 0xFFFFU, true};
    else
        return false;

    return true;
}

static driver_err_t check_ready(const gptim_t *h)
{
    if (h == NULL)
        return DRIVER_ERR_INVALID_ARG;
    if (!h->initialized)
        return DRIVER_ERR_TIM_NOT_INITIALIZED;
    return DRIVER_OK;
}

/**
 * Período = (PSC+1) · (ARR+1) / fclk. O total de contagens é arredondado para o
 * inteiro mais próximo.
 *
 * Procura, a partir do menor PSC em que ARR cabe no registrador, um PSC que
 * divida o total exatamente (ex.: 16 MHz e 10 ms em 16 bits -> PSC+1 = 4,
 * ARR+1 = 40000). Se não houver, usa o menor PSC (erro de até 1 contagem).
 */
static driver_err_t compute_timebase(uint32_t clock_hz, uint32_t period_us, uint32_t max_arr,
                                     uint16_t *psc, uint32_t *arr)
{
    const uint64_t total = ((uint64_t)clock_hz * period_us + 500000U) / 1000000U;
    const uint64_t max_counts = (uint64_t)max_arr + 1U;

    if (total < 2U)
        return DRIVER_ERR_TIM_OUT_OF_RANGE;

    const uint64_t min_div = (total + max_counts - 1U) / max_counts; /* menor PSC + 1 */
    if (min_div > 65536U)
        return DRIVER_ERR_TIM_OUT_OF_RANGE;

    uint64_t div = min_div;
    for (uint64_t d = min_div; d <= 65536U && total / d >= 2U; d++)
    {
        if (total % d == 0U)
        {
            div = d;
            break;
        }
    }

    const uint64_t counts = total / div; /* ARR + 1 */
    if (counts < 2U)
        return DRIVER_ERR_TIM_OUT_OF_RANGE;

    *psc = (uint16_t)(div - 1U);
    *arr = (uint32_t)(counts - 1U);
    return DRIVER_OK;
}

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */

driver_err_t gptim_init(gptim_t *h, const gptim_config_t *cfg)
{
    if (h == NULL || cfg == NULL || cfg->timer_clock_hz == 0U || cfg->period_us == 0U ||
        cfg->irq_priority > 15U)
        return DRIVER_ERR_INVALID_ARG;

    gptim_info_t info;
    if (!get_info(cfg->timer, &info))
        return DRIVER_ERR_INVALID_ARG;

    uint16_t psc;
    uint32_t arr;

    const driver_err_t err = compute_timebase(cfg->timer_clock_hz, cfg->period_us, info.max_arr, &psc, &arr);
    if (err != DRIVER_OK)
        return err;

    h->initialized = false;
    h->timer = cfg->timer;
    h->timer_clock_hz = cfg->timer_clock_hz;
    h->max_arr = info.max_arr;
    h->irq = info.irq;
    h->shared_irq = info.shared_irq;
    h->callback = cfg->callback;
    h->context = cfg->context;

    /* Clock e reset do periférico */
    *info.enr |= info.bit;
    (void)*info.enr;
    *info.rstr |= info.bit;
    *info.rstr &= ~info.bit;

    TIM_TypeDef *const t = cfg->timer;

    t->PSC = psc;
    t->ARR = arr;
    t->CR1 = TIM_CR1_ARPE | TIM_CR1_URS; /* só o overflow gera interrupção */

    t->EGR = TIM_EGR_UG; /* carrega PSC/ARR agora */
    t->SR = ~TIM_SR_UIF;
    t->DIER = TIM_DIER_UIE;

    NVIC_SetPriority(info.irq, cfg->irq_priority);
    NVIC_ClearPendingIRQ(info.irq);
    NVIC_EnableIRQ(info.irq);

    h->initialized = true;
    return DRIVER_OK;
}

driver_err_t gptim_deinit(gptim_t *h)
{
    const driver_err_t err = check_ready(h);
    if (err != DRIVER_OK)
        return err;

    gptim_info_t info;
    (void)get_info(h->timer, &info);

    TIM_TypeDef *const t = h->timer;

    t->CR1 &= ~TIM_CR1_CEN;
    t->DIER = 0;
    t->SR = ~TIM_SR_UIF;

    if (!h->shared_irq)
        NVIC_DisableIRQ(h->irq);

    *info.enr &= ~info.bit;

    h->callback = NULL;
    h->context = NULL;
    h->initialized = false;

    return DRIVER_OK;
}

driver_err_t gptim_start(gptim_t *h)
{
    const driver_err_t err = check_ready(h);
    if (err != DRIVER_OK)
        return err;

    h->timer->CNT = 0;
    h->timer->CR1 |= TIM_CR1_CEN;
    return DRIVER_OK;
}

driver_err_t gptim_stop(gptim_t *h)
{
    const driver_err_t err = check_ready(h);
    if (err != DRIVER_OK)
        return err;

    h->timer->CR1 &= ~TIM_CR1_CEN;
    return DRIVER_OK;
}

driver_err_t gptim_set_period_us(gptim_t *h, uint32_t period_us)
{
    driver_err_t err = check_ready(h);
    if (err != DRIVER_OK)
        return err;
    if (period_us == 0U)
        return DRIVER_ERR_INVALID_ARG;

    uint16_t psc;
    uint32_t arr;

    err = compute_timebase(h->timer_clock_hz, period_us, h->max_arr, &psc, &arr);
    if (err != DRIVER_OK)
        return err;

    TIM_TypeDef *const t = h->timer;

    /* PSC e ARR (ARPE = 1) têm preload: valem juntos no próximo update */
    t->PSC = psc;
    t->ARR = arr;

    /* Parado, aplica já (UG com URS = 1 não gera interrupção) */
    if (!(t->CR1 & TIM_CR1_CEN))
        t->EGR = TIM_EGR_UG;

    return DRIVER_OK;
}

bool gptim_is_running(const gptim_t *h)
{
    return (h != NULL) && h->initialized && (h->timer->CR1 & TIM_CR1_CEN);
}

void gptim_irq_handler(gptim_t *h)
{
    if (h == NULL || !h->initialized)
        return;

    TIM_TypeDef *const t = h->timer;

    /* Nos vetores compartilhados com o TIM1, só trata se o update veio deste timer */
    if ((t->SR & TIM_SR_UIF) && (t->DIER & TIM_DIER_UIE))
    {
        t->SR = ~TIM_SR_UIF;

        if (h->callback != NULL)
            h->callback(h->context);
    }
}
