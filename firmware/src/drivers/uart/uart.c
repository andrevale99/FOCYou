#include "uart.h"

void usart1_init(int clock, int baudrate)
{
    /*
     * Habilita clock do GPIOA
     */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    /*
     * Habilita clock do USART1
     *
     * USART1 está no APB2.
     */
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    /* ========================================================
     * PA9 e PA10 como Alternate Function
     * ======================================================== */

    /*
     * MODER:
     *
     * 00 = Input
     * 01 = Output
     * 10 = Alternate Function
     * 11 = Analog
     */

    GPIOA->MODER &= ~(
        GPIO_MODER_MODER9_Msk |
        GPIO_MODER_MODER10_Msk);

    GPIOA->MODER |=
        (2U << GPIO_MODER_MODER9_Pos) |
        (2U << GPIO_MODER_MODER10_Pos);

    /* ========================================================
     * Alternate Function 7
     *
     * AF7 = USART1 / USART2
     * ======================================================== */

    GPIOA->AFR[1] &= ~(
        GPIO_AFRH_AFSEL9_Msk |
        GPIO_AFRH_AFSEL10_Msk);

    GPIOA->AFR[1] |=
        (7U << GPIO_AFRH_AFSEL9_Pos) |
        (7U << GPIO_AFRH_AFSEL10_Pos);

    /* ========================================================
     * Configuração elétrica
     * ======================================================== */

    /* Push-pull */
    GPIOA->OTYPER &= ~(
        GPIO_OTYPER_OT9 |
        GPIO_OTYPER_OT10);

    /* High speed */
    GPIOA->OSPEEDR |=
        (3U << GPIO_OSPEEDR_OSPEED9_Pos) |
        (3U << GPIO_OSPEEDR_OSPEED10_Pos);

    /* Sem pull-up/pull-down */
    GPIOA->PUPDR &= ~(
        GPIO_PUPDR_PUPD9_Msk |
        GPIO_PUPDR_PUPD10_Msk);

    /* ========================================================
     * USART1
     * ======================================================== */

    /*
     * USART1 recebe clock de APB2.
     *
     * APB2 = 25 MHz
     *
     * Baudrate = 115200
     *
     * Oversampling = 16
     *
     * USARTDIV = 25 MHz / (16 * 115200)
     *
     * BRR = 25 MHz / 115200
     *
     * BRR ≈ 217
     */

    USART1->BRR = clock / baudrate;

    /*
     * CR1
     *
     * TE = Transmitter Enable
     * RE = Receiver Enable
     * UE = USART Enable
     */

    USART1->CR1 =
        USART_CR1_TE |
        USART_CR1_RE |
        USART_CR1_UE;

    /*
     * CR2
     *
     * Stop bits = 1
     *
     * STOP = 00
     */

    USART1->CR2 = 0;

    /*
     * CR3
     *
     * Sem hardware flow control
     */

    USART1->CR3 = 0;
}

void usart1_send_char(char c)
{
    /*
     * TXE = Transmit data register empty
     */
    while (!(USART1->SR & USART_SR_TXE))
        ;

    USART1->DR = (uint8_t)c;
}

void usart1_send_string(const char *str)
{
    while (*str)
    {
        usart1_send_char(*str++);
    }
}