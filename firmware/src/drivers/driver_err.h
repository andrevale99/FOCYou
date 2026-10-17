/**
 * @file driver_err.h
 * @brief Código de retorno universal da biblioteca de drivers.
 *
 * Todas as funções dos drivers (GPIO, RCC, UART, ...) retornam #driver_err_t.
 * Os valores são únicos em toda a biblioteca, agrupados por módulo:
 *
 * - 0           sucesso
 * - -1 a -9     genéricos
 * - -10 a -19   GPIO
 * - -20 a -29   RCC
 * - -30 a -39   UART
 * - -40 a -49   I2C
 * - -50 a -59   timers
 * - -60 a -69   ADC
 * - -70 em diante: reservado para novos periféricos
 */

#ifndef DRIVER_ERR_H
#define DRIVER_ERR_H

/** @brief Código de retorno universal dos drivers. */
typedef enum
{
    DRIVER_OK = 0,                              /**< Operação executada com sucesso. */

    /* Genéricos */
    DRIVER_ERR_INVALID_ARG = -1,                /**< Ponteiro NULL ou parâmetro inválido. */

    /* GPIO */
    DRIVER_ERR_NO_GPIO = -10,                   /**< Ponteiro da porta GPIO é NULL. */
    DRIVER_ERR_NO_CHANNEL_LOCATE = -11,         /**< Porta GPIO não suportada. */
    DRIVER_ERR_INVALID_PIN = -12,               /**< Número de pino fora de 0 a 15. */
    DRIVER_ERR_INVALID_MODE = -13,              /**< Modo/valor de configuração inválido. */

    /* RCC */
    DRIVER_ERR_INVALID_CLOCK_SOURCE = -20,      /**< Fonte de clock inválida. */
    DRIVER_ERR_INVALID_APB_NUM = -21,           /**< Número do barramento APB inválido. */
    DRIVER_ERR_INVALID_APB_DIVIDER = -22,       /**< Divisor do barramento APB inválido. */
    DRIVER_ERR_INVALID_AHB_DIVIDER = -23,       /**< Divisor do barramento AHB inválido. */
    DRIVER_ERR_CLOCK_NOT_RDY = -24,             /**< Fonte de clock não está pronta. */
    DRIVER_ERR_SET_CLOCK_SOURCE_FAILED = -25,   /**< Falha ao selecionar a fonte de clock. */
    DRIVER_ERR_INVALID_PLL_P_FACTOR = -26,      /**< Fator de divisão P do PLL inválido. */
    DRIVER_ERR_INVALID_PLL_M_FACTOR = -27,      /**< Fator de divisão M do PLL inválido. */
    DRIVER_ERR_INVALID_PLL_N_FACTOR = -28,      /**< Fator de multiplicação N do PLL inválido. */

    /* UART */
    DRIVER_ERR_UART_BUSY = -30,                 /**< Transmissão por DMA em andamento. */
    DRIVER_ERR_UART_NO_DMA = -31,               /**< Instância sem DMA configurado (chame uart_dma_init()). */

    /* I2C */
    DRIVER_ERR_I2C_TIMEOUT = -40,               /**< Flag esperado não setou dentro do timeout. */
    DRIVER_ERR_I2C_NACK = -41,                  /**< Endereço ou dado sem ACK (flag AF). */
    DRIVER_ERR_I2C_BUS_ERROR = -42,             /**< START/STOP fora de hora (flag BERR). */
    DRIVER_ERR_I2C_ARLO = -43,                  /**< Perda de arbitragem (flag ARLO). */
    DRIVER_ERR_I2C_BUSY = -44,                  /**< Barramento ocupado antes do START. */
    DRIVER_ERR_I2C_INVALID_SPEED = -45,         /**< Velocidade ou clock do APB1 incompatíveis. */
    DRIVER_ERR_I2C_NOT_INITIALIZED = -46,       /**< i2c_init() ainda não foi chamado. */

    /* Timers */
    DRIVER_ERR_TIM_OUT_OF_RANGE = -50,          /**< Período inatingível com o clock do timer. */
    DRIVER_ERR_TIM_NOT_INITIALIZED = -51,       /**< Timer ainda não inicializado. */

    /* ADC */
    DRIVER_ERR_ADC_TIMEOUT = -60,               /**< A conversão não terminou dentro do timeout. */
    DRIVER_ERR_ADC_NOT_INITIALIZED = -61,       /**< adc_init() ainda não foi chamado. */
    DRIVER_ERR_ADC_INVALID_CHANNEL = -62,       /**< Canal fora de 0 a 18 ou pino sem entrada analógica. */
} driver_err_t;

#endif /* DRIVER_ERR_H */