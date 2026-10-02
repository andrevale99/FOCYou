#ifndef DRIVER_GPIO_H
#define DRIVER_GPIO_H

#include <stddef.h>
#include <stdint.h>

#include <stm32f411xe.h>

#define CLEAR_GPIO_MODER_MSK(pin) (0x3 << (pin << 1))

typedef enum
{
    DRIVER_GPIO_OK = 0,
    DRIVER_ERR_NO_GPIO = -1,
    DRIVER_ERR_NO_CHANNEL_LOCATE = -2,
    DRIVER_ERR_INVALID_PIN = -3,
    DRIVER_ERR_INVALID_MODE = -4,
} driver_gpio_err_t;

typedef enum
{
    DRIVER_GPIO_INPUT = 0,
    DRIVER_GPIO_OUTPUT,
    DRIVER_GPIO_ALTERNATE,
    DRIVER_GPIO_ANALOG,

} driver_gpio_moder_t;

typedef enum
{
    DRIVER_GPIO_OTYPE_PUSHPULL = 0,
    DRIVER_GPIO_OTYPE_OPENDRAIN,
} driver_gpio_otype_t;

typedef enum
{
    DRIVER_GPIO_SPEED_LOW = 0,
    DRIVER_GPIO_SPEED_MEDIUM,
    DRIVER_GPIO_SPEED_HIGH,
    DRIVER_GPIO_SPEED_VERY_HIGH,
} driver_gpio_speed_t;

typedef enum
{
    DRIVER_GPIO_PULL_NONE = 0,
    DRIVER_GPIO_PULL_UP,
    DRIVER_GPIO_PULL_DOWN,
} driver_gpio_pull_t;

typedef enum
{
    DRIVER_GPIO_PIN_RESET = 0,
    DRIVER_GPIO_PIN_SET,
} driver_gpio_pin_state_t;

/**
 * @brief Habilita o clock do periferico GPIO no barramento AHB1.
 *
 * @param gpiox Ponteiro para a porta GPIO (GPIOA, GPIOB, GPIOC, GPIOD, GPIOE ou GPIOH).
 *
 * @retval DRIVER_GPIO_OK                  Clock habilitado com sucesso.
 * @retval DRIVER_ERR_NO_GPIO              @p gpiox e NULL.
 * @retval DRIVER_ERR_NO_CHANNEL_LOCATE    @p gpiox nao corresponde a uma porta suportada.
 */
driver_gpio_err_t driver_gpio_enable_clock(GPIO_TypeDef *gpiox);

/**
 * @brief Configura o modo de operacao de um pino (registrador MODER).
 *
 * @param gpiox Ponteiro para a porta GPIO.
 * @param pin   Numero do pino (0 a 15).
 * @param mode  Modo desejado (entrada, saida, alternate function ou analogico).
 *
 * @retval DRIVER_GPIO_OK               Configuracao aplicada.
 * @retval DRIVER_ERR_NO_GPIO           @p gpiox e NULL.
 * @retval DRIVER_ERR_INVALID_PIN       @p pin fora do intervalo 0 a 15.
 * @retval DRIVER_ERR_INVALID_MODE      @p mode invalido.
 */
driver_gpio_err_t driver_gpio_set_mode(GPIO_TypeDef *gpiox, uint16_t pin,
                                       driver_gpio_moder_t mode);

/**
 * @brief Configura o tipo de saida de um pino (registrador OTYPER).
 *
 * @param gpiox Ponteiro para a porta GPIO.
 * @param pin   Numero do pino (0 a 15).
 * @param type  Push-pull ou open-drain.
 *
 * @retval DRIVER_GPIO_OK               Configuracao aplicada.
 * @retval DRIVER_ERR_NO_GPIO           @p gpiox e NULL.
 * @retval DRIVER_ERR_INVALID_PIN       @p pin fora do intervalo 0 a 15.
 * @retval DRIVER_ERR_INVALID_MODE      @p type invalido.
 */
driver_gpio_err_t driver_gpio_set_output_type(GPIO_TypeDef *gpiox, uint16_t pin,
                                              driver_gpio_otype_t type);

/**
 * @brief Configura a velocidade de saida de um pino (registrador OSPEEDR).
 *
 * @param gpiox Ponteiro para a porta GPIO.
 * @param pin   Numero do pino (0 a 15).
 * @param speed Velocidade (low, medium, high ou very high).
 *
 * @retval DRIVER_GPIO_OK               Configuracao aplicada.
 * @retval DRIVER_ERR_NO_GPIO           @p gpiox e NULL.
 * @retval DRIVER_ERR_INVALID_PIN       @p pin fora do intervalo 0 a 15.
 * @retval DRIVER_ERR_INVALID_MODE      @p speed invalido.
 */
driver_gpio_err_t driver_gpio_set_speed(GPIO_TypeDef *gpiox, uint16_t pin,
                                        driver_gpio_speed_t speed);

/**
 * @brief Configura o resistor de pull-up/pull-down de um pino (registrador PUPDR).
 *
 * @param gpiox Ponteiro para a porta GPIO.
 * @param pin   Numero do pino (0 a 15).
 * @param pull  Sem pull, pull-up ou pull-down.
 *
 * @retval DRIVER_GPIO_OK               Configuracao aplicada.
 * @retval DRIVER_ERR_NO_GPIO           @p gpiox e NULL.
 * @retval DRIVER_ERR_INVALID_PIN       @p pin fora do intervalo 0 a 15.
 * @retval DRIVER_ERR_INVALID_MODE      @p pull invalido.
 */
driver_gpio_err_t driver_gpio_set_pull(GPIO_TypeDef *gpiox, uint16_t pin,
                                       driver_gpio_pull_t pull);

/**
 * @brief Escreve o nivel logico de um pino de saida de forma atomica (registrador BSRR).
 *
 * @param gpiox Ponteiro para a porta GPIO.
 * @param pin   Numero do pino (0 a 15).
 * @param state Nivel desejado (DRIVER_GPIO_PIN_SET ou DRIVER_GPIO_PIN_RESET).
 *
 * @retval DRIVER_GPIO_OK               Escrita realizada.
 * @retval DRIVER_ERR_NO_GPIO           @p gpiox e NULL.
 * @retval DRIVER_ERR_INVALID_PIN       @p pin fora do intervalo 0 a 15.
 * @retval DRIVER_ERR_INVALID_MODE      @p state invalido.
 *
 * @note Usa BSRR, portanto nao ha read-modify-write e a operacao e segura
 *       contra interrupcoes.
 */
driver_gpio_err_t driver_gpio_write_pin(GPIO_TypeDef *gpiox, uint16_t pin,
                                        driver_gpio_pin_state_t state);

/**
 * @brief Troca o nivel do pino de saida
 *
 * @param gpiox Ponteiro para a porta GPIO.
 * @param pin   Numero do pino (0 a 15).
 *
 * @retval DRIVER_GPIO_OK               Escrita realizada.
 * @retval DRIVER_ERR_NO_GPIO           @p gpiox e NULL.
 * @retval DRIVER_ERR_INVALID_PIN       @p pin fora do intervalo 0 a 15.
 *
 * @note Usa o registrador ODR, operacao sujeita a modificacoes indevidas
 *       por interrupcoes
 */
driver_gpio_err_t driver_gpio_toggle_pin(GPIO_TypeDef *gpiox, uint16_t pin);

/**
 * @brief Configura um pino como alternate function e seleciona a funcao (MODER e AFR).
 *
 * @param gpiox          Ponteiro para a porta GPIO.
 * @param pin            Numero do pino (0 a 15).
 * @param alternate_mode Numero da alternate function (AF0 a AF15).
 *
 * @retval DRIVER_GPIO_OK               Configuracao aplicada.
 * @retval DRIVER_ERR_NO_GPIO           @p gpiox e NULL.
 * @retval DRIVER_ERR_INVALID_PIN       @p pin fora do intervalo 0 a 15.
 * @retval DRIVER_ERR_INVALID_MODE      @p alternate_mode maior que 15.
 */
driver_gpio_err_t driver_gpio_set_alternate_function(GPIO_TypeDef *gpiox,
                                                     uint16_t pin,
                                                     uint8_t alternate_mode);

#endif