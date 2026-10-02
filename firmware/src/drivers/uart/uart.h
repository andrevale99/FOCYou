#ifndef UART_H
#define UART_H

#include <stm32f411xe.h>


/**
 * @brief Inicializa a interface USART1.
 *
 * Configura os pinos PA9 e PA10 para a função alternativa AF7,
 * habilita os clocks necessários e configura o periférico USART1
 * para transmissão e recepção de dados.
 *
 * A comunicação é configurada com:
 * - 8 bits de dados;
 * - 1 bit de parada;
 * - Sem controle de fluxo por hardware;
 * - Oversampling por 16;
 * - Baudrate definido pelo parâmetro @p baudrate.
 *
 * O clock utilizado para o cálculo do registrador BRR deve
 * corresponder à frequência do clock fornecido ao USART1 pelo APB2.
 *
 * @param[in] clock
 *     Frequência do clock do USART1, em Hz.
 *
 * @param[in] baudrate
 *     Taxa de transmissão desejada, em baud.
 *
 * @note
 *     O USART1 é conectado ao barramento APB2 do STM32F411.
 *
 * @note
 *     Os pinos PA9 e PA10 são configurados com Alternate Function 7,
 *     correspondente ao USART1.
 *
 * @warning
 *     O valor de @p clock deve corresponder ao clock real do APB2.
 *     Um valor incorreto resulta em erro no baudrate configurado.
 */
void usart1_init(int clock, int baudrate);

/**
 * @brief Transmite um caractere pela USART1.
 *
 * Aguarda até que o registrador de dados de transmissão esteja
 * disponível e, em seguida, escreve o caractere no registrador DR
 * do USART1.
 *
 * @param[in] c
 *     Caractere a ser transmitido.
 *
 * @note
 *     A função utiliza espera ocupada (busy-wait) enquanto o
 *     registrador de transmissão não estiver disponível.
 */
void usart1_send_char(char c);

/**
 * @brief Transmite uma string pela USART1.
 *
 * Percorre a string caractere por caractere e utiliza
 * @ref usart1_send_char para realizar a transmissão.
 *
 * @param[in] str
 *     Ponteiro para a string terminada pelo caractere nulo
 *     ('\0') a ser transmitida.
 *
 * @note
 *     A função permanece bloqueada até que todos os caracteres
 *     da string sejam transmitidos para o periférico USART1.
 *
 * @warning
 *     O ponteiro @p str deve apontar para uma string válida
 *     terminada em '\0'.
 */
void usart1_send_string(const char *str);

#endif
