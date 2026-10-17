#include "uart.h"

/**
 * @brief Habilita o clock do barramento para a instância USART.
 * @retval DRIVER_OK                 Clock habilitado.
 * @retval DRIVER_ERR_INVALID_ARG    Instância não suportada no STM32F411.
 */
static driver_err_t uart_enable_clock(const USART_TypeDef *usart)
{
    if (usart == USART1)
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    else if (usart == USART2)
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    else if (usart == USART6)
        RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
    else
        return DRIVER_ERR_INVALID_ARG;

    return DRIVER_OK;
}

/**
 * @brief Configura um pino como alternate function push-pull, very high speed, sem pull.
 */
static driver_err_t uart_config_pin(GPIO_TypeDef *gpio, uint16_t pin, uint8_t af)
{
    driver_err_t err;

    err = driver_gpio_set_alternate_function(gpio, pin, af);
    if (err != DRIVER_OK)
        return err;

    err = driver_gpio_set_output_type(gpio, pin, DRIVER_GPIO_OTYPE_PUSHPULL);
    if (err != DRIVER_OK)
        return err;

    err = driver_gpio_set_speed(gpio, pin, DRIVER_GPIO_SPEED_VERY_HIGH);
    if (err != DRIVER_OK)
        return err;

    return driver_gpio_set_pull(gpio, pin, DRIVER_GPIO_PULL_NONE);
}

driver_err_t uart_init(const uart_config_t *config)
{
    if (config == NULL || config->usart == NULL)
        return DRIVER_ERR_INVALID_ARG;

    if (config->clock_hz == 0U || config->baudrate == 0U)
        return DRIVER_ERR_INVALID_ARG;

    /*
     * Oversampling = 16: BRR = fclk / baud (mantissa 12 bits + fração 4 bits),
     * arredondado para o inteiro mais próximo.
     */
    const uint32_t brr = (config->clock_hz + (config->baudrate / 2U)) / config->baudrate;

    if (brr == 0U || brr > 0xFFFFU)
        return DRIVER_ERR_INVALID_ARG;

    USART_TypeDef *const usart = config->usart;
    driver_err_t err;

    /* Clock do periférico (também valida a instância) */
    err = uart_enable_clock(usart);
    if (err != DRIVER_OK)
        return err;

    /* GPIO: clock e pinos TX/RX */
    err = driver_gpio_enable_clock(config->gpio);
    if (err != DRIVER_OK)
        return err;

    err = uart_config_pin(config->gpio, config->tx_pin, config->alternate_function);
    if (err != DRIVER_OK)
        return err;

    err = uart_config_pin(config->gpio, config->rx_pin, config->alternate_function);
    if (err != DRIVER_OK)
        return err;

    /* Configuração do periférico */
    usart->CR1 = 0;          /* UE = 0 durante a configuração */
    usart->CR2 = 0;          /* 1 bit de parada */
    usart->CR3 = 0;          /* sem controle de fluxo */

    usart->BRR = brr;

    usart->CR1 = USART_CR1_TE |
                 USART_CR1_RE |
                 USART_CR1_UE;

    return DRIVER_OK;
}

driver_err_t uart_send_char(USART_TypeDef *usart, char c)
{
    if (usart == NULL)
        return DRIVER_ERR_INVALID_ARG;

    /*
     * TXE = Transmit data register empty
     */
    while (!(usart->SR & USART_SR_TXE))
        ;

    usart->DR = (uint8_t)c;

    return DRIVER_OK;
}

/* ------------------------------------------------------------------ */
/* DMA                                                                 */
/* ------------------------------------------------------------------ */

/** Stream e canal do DMA de TX de cada USART. */
typedef struct
{
    DMA_TypeDef *dma;
    DMA_Stream_TypeDef *stream;
    uint8_t stream_num; /* 0 a 7: posição dos flags em LISR/HISR */
    uint8_t channel;
} uart_dma_map_t;

static bool dma_map(const USART_TypeDef *usart, uart_dma_map_t *m)
{
    if (usart == USART1)
        *m = (uart_dma_map_t){DMA2, DMA2_Stream7, 7U, 4U};
    else if (usart == USART2)
        *m = (uart_dma_map_t){DMA1, DMA1_Stream6, 6U, 4U};
    else if (usart == USART6)
        *m = (uart_dma_map_t){DMA2, DMA2_Stream6, 6U, 5U};
    else
        return false;

    return true;
}

/** Limpa todos os flags (FE, DME, TE, HT, TC) do stream. */
static void dma_clear_flags(const uart_dma_map_t *m)
{
    static const uint8_t shift[4] = {0U, 6U, 16U, 22U};
    const uint32_t mask = 0x3DUL << shift[m->stream_num & 3U];

    if (m->stream_num < 4U)
        m->dma->LIFCR = mask;
    else
        m->dma->HIFCR = mask;
}

driver_err_t uart_dma_init(USART_TypeDef *usart)
{
    uart_dma_map_t m;
    if (usart == NULL || !dma_map(usart, &m))
        return DRIVER_ERR_INVALID_ARG;

    RCC->AHB1ENR |= (m.dma == DMA1) ? RCC_AHB1ENR_DMA1EN : RCC_AHB1ENR_DMA2EN;
    (void)RCC->AHB1ENR;

    /* Stream desligado antes de configurar */
    m.stream->CR &= ~DMA_SxCR_EN;
    while (m.stream->CR & DMA_SxCR_EN)
        ;
    dma_clear_flags(&m);

    m.stream->PAR = (uint32_t)&usart->DR;
    m.stream->FCR = 0;                                   /* modo direto (sem FIFO) */
    m.stream->CR = ((uint32_t)m.channel << DMA_SxCR_CHSEL_Pos) |
                   DMA_SxCR_MINC |                        /* memória incrementa, periférico fixo */
                   DMA_SxCR_DIR_0;                        /* memória -> periférico, bytes */

    usart->CR3 |= USART_CR3_DMAT;
    return DRIVER_OK;
}

driver_err_t uart_send_dma(USART_TypeDef *usart, const uint8_t *data, uint16_t len)
{
    uart_dma_map_t m;
    if (usart == NULL || data == NULL || !dma_map(usart, &m))
        return DRIVER_ERR_INVALID_ARG;
    if (!(usart->CR3 & USART_CR3_DMAT))
        return DRIVER_ERR_UART_NO_DMA;
    if (m.stream->CR & DMA_SxCR_EN)
        return DRIVER_ERR_UART_BUSY;
    if (len == 0U)
        return DRIVER_OK;

    dma_clear_flags(&m);
    m.stream->M0AR = (uint32_t)data;
    m.stream->NDTR = len;
    m.stream->CR |= DMA_SxCR_EN; /* o EN volta a 0 sozinho quando NDTR chega a 0 */

    return DRIVER_OK;
}

bool uart_dma_busy(USART_TypeDef *usart)
{
    uart_dma_map_t m;
    if (usart == NULL || !dma_map(usart, &m))
        return false;

    return (m.stream->CR & DMA_SxCR_EN) != 0U;
}

driver_err_t uart_send_string(USART_TypeDef *usart, const char *str)
{
    if (usart == NULL || str == NULL)
        return DRIVER_ERR_INVALID_ARG;

    /* Não mistura com uma transmissão por DMA em andamento */
    while (uart_dma_busy(usart))
        ;

    while (*str)
    {
        uart_send_char(usart, *str++);
    }

    return DRIVER_OK;
}