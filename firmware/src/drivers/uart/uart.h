#ifndef UART_H
#define UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <stm32f411xe.h>

#include "drivers/driver_err.h"
#include "drivers/gpio/driver_gpio.h"

/**
 * @brief Configuração de uma interface USART.
 *
 * Agrupa o periférico, o clock, o baudrate e os pinos de TX/RX.
 * Instâncias suportadas no STM32F411: USART1, USART2 e USART6.
 *
 * Exemplo (USART1 em PA9/PA10, APB2 = 25 MHz):
 * @code
 * const uart_config_t cfg = {
 *     .usart = USART1,
 *     .clock_hz = 25000000U,
 *     .baudrate = 115200U,
 *     .gpio = GPIOA,
 *     .tx_pin = 9,
 *     .rx_pin = 10,
 *     .alternate_function = 7,
 * };
 * @endcode
 */
typedef struct
{
    USART_TypeDef *usart;        /**< Periférico: USART1, USART2 ou USART6. */

    uint32_t clock_hz;           /**< Clock do periférico (APB2 para USART1/6, APB1 para USART2), em Hz. */
    uint32_t baudrate;           /**< Taxa de transmissão desejada, em baud. */

    GPIO_TypeDef *gpio;          /**< Porta GPIO dos pinos TX e RX. */
    uint16_t tx_pin;             /**< Pino de TX (0 a 15). */
    uint16_t rx_pin;             /**< Pino de RX (0 a 15). */
    uint8_t alternate_function;  /**< Alternate function dos pinos (AF7 para USART1/2, AF8 para USART6). */
} uart_config_t;

/**
 * @brief Inicializa uma interface USART.
 *
 * Habilita os clocks, configura os pinos de TX/RX e o periférico com:
 * - 8 bits de dados;
 * - 1 bit de parada;
 * - sem controle de fluxo por hardware;
 * - oversampling por 16;
 * - baudrate definido em @c config->baudrate.
 *
 * @param[in] config Configuração da interface.
 *
 * @return Código indicando o resultado da operação.
 *
 * @retval DRIVER_OK
 *     USART configurada com sucesso.
 *
 * @retval DRIVER_ERR_INVALID_ARG
 *     @p config ou @c config->usart é NULL, instância não suportada,
 *     @c clock_hz ou @c baudrate igual a zero, ou divisão fora do alcance
 *     do registrador BRR (16 bits).
 *
 * @return Também propaga os erros do driver GPIO (ex.: DRIVER_ERR_NO_GPIO,
 *         DRIVER_ERR_INVALID_PIN) caso a configuração dos pinos falhe.
 *
 * @warning
 *     @c clock_hz deve corresponder ao clock real do barramento do periférico.
 *     Um valor incorreto resulta em erro no baudrate configurado.
 */
driver_err_t uart_init(const uart_config_t *config);

/**
 * @brief Transmite um caractere por uma USART.
 *
 * Aguarda (busy-wait) o registrador de transmissão ficar livre e escreve
 * o caractere em DR.
 *
 * @param[in] usart Periférico previamente inicializado com uart_init().
 * @param[in] c     Caractere a ser transmitido.
 *
 * @retval DRIVER_OK
 *     Caractere escrito no registrador DR.
 *
 * @retval DRIVER_ERR_INVALID_ARG
 *     @p usart é NULL.
 */
driver_err_t uart_send_char(USART_TypeDef *usart, char c);

/**
 * @brief Transmite uma string por uma USART.
 *
 * Bloqueia até que todos os caracteres sejam enviados ao periférico. Se houver
 * transmissão por DMA em andamento, espera ela terminar antes.
 *
 * @param[in] usart Periférico previamente inicializado com uart_init().
 * @param[in] str   String terminada em '\0'.
 *
 * @retval DRIVER_OK
 *     String transmitida por completo.
 *
 * @retval DRIVER_ERR_INVALID_ARG
 *     @p usart ou @p str é NULL.
 */
driver_err_t uart_send_string(USART_TypeDef *usart, const char *str);

/* ------------------------------------------------------------------ */
/* Transmissão por DMA                                                 */
/* ------------------------------------------------------------------ */

/**
 * @brief Prepara a transmissão por DMA (TX) de uma USART já inicializada.
 *
 * Streams usados (RM0383, tabelas de requisição do DMA):
 *
 * | USART  | DMA  | Stream | Canal |
 * |--------|------|--------|-------|
 * | USART1 | DMA2 | 7      | 4     |
 * | USART2 | DMA1 | 6      | 4     |
 * | USART6 | DMA2 | 6      | 5     |
 *
 * Não usa interrupções: o fim da transmissão é consultado com uart_dma_busy().
 *
 * @param[in] usart Periférico inicializado com uart_init().
 * @retval DRIVER_OK              Sucesso.
 * @retval DRIVER_ERR_INVALID_ARG @p usart é NULL ou não suportada.
 */
driver_err_t uart_dma_init(USART_TypeDef *usart);

/**
 * @brief Inicia a transmissão de @p len bytes por DMA e retorna na hora.
 *
 * @param[in] usart Periférico com uart_dma_init() feito.
 * @param[in] data  Dados; o buffer precisa continuar válido e sem alteração até
 *                  uart_dma_busy() retornar false.
 * @param[in] len   Quantidade de bytes (1 a 65535; 0 não faz nada).
 *
 * @retval DRIVER_OK              Transmissão iniciada.
 * @retval DRIVER_ERR_INVALID_ARG NULL ou instância não suportada.
 * @retval DRIVER_ERR_UART_NO_DMA uart_dma_init() não foi chamado.
 * @retval DRIVER_ERR_UART_BUSY   Uma transmissão anterior ainda está em andamento.
 */
driver_err_t uart_send_dma(USART_TypeDef *usart, const uint8_t *data, uint16_t len);

/**
 * @brief Informa se há transmissão por DMA em andamento.
 * @param[in] usart Periférico.
 * @return true enquanto o DMA ainda tem bytes para enviar.
 */
bool uart_dma_busy(USART_TypeDef *usart);

#endif