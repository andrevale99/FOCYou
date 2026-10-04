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

driver_err_t uart_send_string(USART_TypeDef *usart, const char *str)
{
    if (usart == NULL || str == NULL)
        return DRIVER_ERR_INVALID_ARG;

    while (*str)
    {
        uart_send_char(usart, *str++);
    }

    return DRIVER_OK;
}