#include "adc.h"

#include "drivers/gpio/driver_gpio.h"

/* Clock máximo do ADC com VDDA de 2,4 V a 3,6 V (datasheet do STM32F411) */
#define ADC_MAX_CLOCK_HZ 36000000U

/* Maior canal (16 = temperatura, 17 = VREFINT, 18 = VBAT) */
#define ADC_MAX_CHANNEL 18U

/* ------------------------------------------------------------------ */
/* Helpers internos                                                    */
/* ------------------------------------------------------------------ */

static driver_err_t check_ready(const adc_t *h)
{
    if (h == NULL)
        return DRIVER_ERR_INVALID_ARG;
    if (!h->initialized)
        return DRIVER_ERR_ADC_NOT_INITIALIZED;
    return DRIVER_OK;
}

/** Converte porta/pino no canal do ADC1 (RM0383, tabela de funções alternativas). */
static bool pin_to_channel(const GPIO_TypeDef *port, uint16_t pin, uint8_t *channel)
{
    if (port == GPIOA && pin <= 7U)
        *channel = (uint8_t)pin;
    else if (port == GPIOB && pin <= 1U)
        *channel = (uint8_t)(8U + pin);
    else if (port == GPIOC && pin <= 5U)
        *channel = (uint8_t)(10U + pin);
    else
        return false;

    return true;
}

/** Escreve o tempo de amostragem do canal em SMPR1 (10..18) ou SMPR2 (0..9). */
static void set_sample_time(ADC_TypeDef *adc, uint8_t channel, adc_sample_time_t st)
{
    if (channel <= 9U)
    {
        const uint32_t shift = 3U * channel;
        adc->SMPR2 = (adc->SMPR2 & ~(7UL << shift)) | ((uint32_t)st << shift);
    }
    else
    {
        const uint32_t shift = 3U * (channel - 10U);
        adc->SMPR1 = (adc->SMPR1 & ~(7UL << shift)) | ((uint32_t)st << shift);
    }
}

/** Espera tSTAB (~3 µs) depois de ligar o ADC. */
static void stab_delay(void)
{
    for (volatile uint32_t i = SystemCoreClock / 1000000U * 3U; i > 0U; i--)
        ;
}

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */

driver_err_t adc_init(adc_t *h, const adc_config_t *cfg)
{
    if (h == NULL || cfg == NULL || cfg->adc != ADC1 || cfg->pclk2_hz == 0U ||
        (uint32_t)cfg->sample_time > (uint32_t)ADC_SAMPLE_480)
        return DRIVER_ERR_INVALID_ARG;

    /* Menor divisor (2, 4, 6 ou 8) com ADCCLK <= 36 MHz */
    uint32_t adcpre = 0;
    while (cfg->pclk2_hz / (2U * (adcpre + 1U)) > ADC_MAX_CLOCK_HZ)
    {
        if (++adcpre > 3U)
            return DRIVER_ERR_INVALID_ARG;
    }

    h->initialized = false;
    h->adc = cfg->adc;
    h->sample_time = cfg->sample_time;
    h->timeout = (cfg->timeout == 0U) ? ADC_DEFAULT_TIMEOUT : cfg->timeout;

    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    (void)RCC->APB2ENR;
    RCC->APB2RSTR |= RCC_APB2RSTR_ADCRST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_ADCRST;

    ADC1_COMMON->CCR = (ADC1_COMMON->CCR & ~ADC_CCR_ADCPRE) | (adcpre << ADC_CCR_ADCPRE_Pos);

    ADC1->CR1 = 0;            /* 12 bits, sem scan, sem interrupções */
    ADC1->CR2 = 0;            /* conversão única, alinhado à direita, trigger por software */
    ADC1->SQR1 = 0;           /* L = 0: uma conversão por sequência */
    ADC1->CR2 = ADC_CR2_ADON;
    stab_delay();

    h->initialized = true;
    return DRIVER_OK;
}

driver_err_t adc_deinit(adc_t *h)
{
    const driver_err_t err = check_ready(h);
    if (err != DRIVER_OK)
        return err;

    h->adc->CR2 = 0;
    RCC->APB2ENR &= ~RCC_APB2ENR_ADC1EN;
    h->initialized = false;

    return DRIVER_OK;
}

driver_err_t adc_config_pin(adc_t *h, GPIO_TypeDef *port, uint16_t pin, uint8_t *channel)
{
    driver_err_t err = check_ready(h);
    if (err != DRIVER_OK)
        return err;

    uint8_t ch;
    if (!pin_to_channel(port, pin, &ch))
        return DRIVER_ERR_ADC_INVALID_CHANNEL;

    err = driver_gpio_enable_clock(port);
    if (err != DRIVER_OK)
        return err;

    err = driver_gpio_set_pull(port, pin, DRIVER_GPIO_PULL_NONE);
    if (err != DRIVER_OK)
        return err;

    err = driver_gpio_set_mode(port, pin, DRIVER_GPIO_ANALOG);
    if (err != DRIVER_OK)
        return err;

    set_sample_time(h->adc, ch, h->sample_time);

    if (channel != NULL)
        *channel = ch;

    return DRIVER_OK;
}

driver_err_t adc_read(adc_t *h, uint8_t channel, uint16_t *value)
{
    const driver_err_t err = check_ready(h);
    if (err != DRIVER_OK)
        return err;
    if (value == NULL)
        return DRIVER_ERR_INVALID_ARG;
    if (channel > ADC_MAX_CHANNEL)
        return DRIVER_ERR_ADC_INVALID_CHANNEL;

    ADC_TypeDef *const adc = h->adc;

    adc->SQR3 = channel; /* SQ1 = canal */
    adc->SR = ~(ADC_SR_EOC | ADC_SR_OVR);
    adc->CR2 |= ADC_CR2_SWSTART;

    for (uint32_t n = h->timeout; n > 0U; n--)
    {
        if (adc->SR & ADC_SR_EOC)
        {
            *value = (uint16_t)(adc->DR & ADC_MAX_VALUE); /* ler DR limpa o EOC */
            return DRIVER_OK;
        }
    }

    return DRIVER_ERR_ADC_TIMEOUT;
}

driver_err_t adc_read_average(adc_t *h, uint8_t channel, uint8_t samples, uint16_t *value)
{
    if (value == NULL)
        return DRIVER_ERR_INVALID_ARG;
    if (samples == 0U)
        samples = 1U;

    uint32_t sum = 0;

    for (uint8_t i = 0; i < samples; i++)
    {
        uint16_t v;
        const driver_err_t err = adc_read(h, channel, &v);
        if (err != DRIVER_OK)
            return err;
        sum += v;
    }

    *value = (uint16_t)((sum + samples / 2U) / samples);
    return DRIVER_OK;
}
