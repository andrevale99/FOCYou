/**
 * @file i2c.h
 * @brief Driver I2C mestre (polling com timeout) para o STM32F411.
 *
 * Instâncias suportadas: I2C1, I2C2 e I2C3. Endereços de 7 bits.
 * Todas as esperas têm timeout; em caso de erro o driver gera STOP e
 * libera o barramento.
 *
 * Os pinos ficam em open-drain sem pull interno: use pull-up externo
 * (2,2 kΩ a 4,7 kΩ).
 *
 * Exemplo (I2C1 em PB6/PB7, AF4, APB1 = 16 MHz, leitura do ângulo do AS5600):
 * @code
 * static i2c_t i2c;
 *
 * const i2c_config_t cfg = {
 *     .i2c = I2C1,
 *     .pclk1_hz = 16000000U,
 *     .speed = I2C_SPEED_FAST,
 *     .scl = {GPIOB, 6, 4},
 *     .sda = {GPIOB, 7, 4},
 *     .timeout = 0,                  // 0 = I2C_DEFAULT_TIMEOUT
 * };
 *
 * i2c_init(&i2c, &cfg);
 *
 * uint8_t raw[2];
 * if (i2c_mem_read(&i2c, 0x36, 0x0E, 1, raw, 2) == DRIVER_OK)
 *     angle = ((uint16_t)(raw[0] & 0x0F) << 8) | raw[1];
 * @endcode
 */

#ifndef I2C_H
#define I2C_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <stm32f411xe.h>

#include "drivers/driver_err.h"
#include "drivers/gpio/driver_gpio.h"

/** @brief Timeout padrão, em iterações do laço de espera de cada flag. */
#ifndef I2C_DEFAULT_TIMEOUT
#define I2C_DEFAULT_TIMEOUT 100000U
#endif

/** @brief Velocidade do barramento. */
typedef enum
{
    I2C_SPEED_STANDARD = 100000U, /**< Standard mode, 100 kHz. */
    I2C_SPEED_FAST = 400000U,     /**< Fast mode, 400 kHz (duty 2:1). */
} i2c_speed_t;

/** @brief Pino de SCL ou SDA. */
typedef struct
{
    GPIO_TypeDef *port;         /**< Porta GPIO. */
    uint16_t pin;               /**< Número do pino (0 a 15). */
    uint8_t alternate_function; /**< AF4 na maioria dos pinos; AF9 em PB3/PB4 (I2C2/I2C3 SDA). */
} i2c_pin_t;

/** @brief Configuração do driver. */
typedef struct
{
    I2C_TypeDef *i2c;  /**< I2C1, I2C2 ou I2C3. */
    uint32_t pclk1_hz; /**< Clock real do APB1, em Hz (2 a 50 MHz; >= 4 MHz em fast mode). */
    i2c_speed_t speed; /**< Velocidade do barramento. */

    i2c_pin_t scl; /**< Pino SCL. */
    i2c_pin_t sda; /**< Pino SDA. */

    uint32_t timeout; /**< Iterações por espera de flag (0 = #I2C_DEFAULT_TIMEOUT). */
} i2c_config_t;

/**
 * @brief Handle do driver.
 * @note Os campos são de uso interno; não os modifique diretamente.
 */
typedef struct
{
    i2c_config_t cfg; /**< Cópia da configuração. */
    bool initialized; /**< true após i2c_init() bem-sucedido. */
} i2c_t;

/**
 * @brief Configura os pinos e o periférico.
 *
 * Reseta o periférico (RCC) antes de configurar CR2.FREQ, CCR e TRISE.
 * O CCR é arredondado para cima: a frequência real fica igual ou abaixo da pedida.
 *
 * @param[out] h   Handle a ser preenchido.
 * @param[in]  cfg Configuração desejada.
 *
 * @retval DRIVER_OK                    Sucesso.
 * @retval DRIVER_ERR_INVALID_ARG       NULL ou instância não suportada.
 * @retval DRIVER_ERR_I2C_INVALID_SPEED Velocidade inválida ou @c pclk1_hz fora da faixa.
 * @return Também propaga os erros do driver GPIO.
 */
driver_err_t i2c_init(i2c_t *h, const i2c_config_t *cfg);

/**
 * @brief Desliga o periférico e o clock dele.
 * @param[in,out] h Handle inicializado.
 * @retval DRIVER_OK                      Sucesso.
 * @retval DRIVER_ERR_I2C_NOT_INITIALIZED Handle não inicializado.
 * @note Os pinos continuam em alternate function.
 */
driver_err_t i2c_deinit(i2c_t *h);

/**
 * @brief Escreve @p len bytes no dispositivo (START, endereço, dados, STOP).
 * @param[in,out] h    Handle inicializado.
 * @param[in]     addr Endereço de 7 bits.
 * @param[in]     data Dados (pode ser NULL se @p len = 0).
 * @param[in]     len  Quantidade de bytes.
 * @retval DRIVER_OK Sucesso.
 * @return Erros DRIVER_ERR_I2C_* em caso de NACK, timeout, erro de barramento etc.
 */
driver_err_t i2c_write(i2c_t *h, uint8_t addr, const uint8_t *data, uint16_t len);

/**
 * @brief Lê @p len bytes do dispositivo (START, endereço, dados, STOP).
 * @param[in,out] h    Handle inicializado.
 * @param[in]     addr Endereço de 7 bits.
 * @param[out]    data Buffer de destino.
 * @param[in]     len  Quantidade de bytes (> 0).
 * @retval DRIVER_OK Sucesso.
 * @return Erros DRIVER_ERR_I2C_* em caso de NACK, timeout, erro de barramento etc.
 */
driver_err_t i2c_read(i2c_t *h, uint8_t addr, uint8_t *data, uint16_t len);

/**
 * @brief Escreve @p len bytes a partir do registrador @p reg do dispositivo.
 * @param[in,out] h        Handle inicializado.
 * @param[in]     addr     Endereço de 7 bits.
 * @param[in]     reg      Endereço do registrador (MSB primeiro se @p reg_size = 2).
 * @param[in]     reg_size Tamanho do endereço do registrador: 1 ou 2 bytes.
 * @param[in]     data     Dados (pode ser NULL se @p len = 0).
 * @param[in]     len      Quantidade de bytes.
 * @retval DRIVER_OK Sucesso.
 * @return Erros DRIVER_ERR_I2C_* em caso de NACK, timeout, erro de barramento etc.
 */
driver_err_t i2c_mem_write(i2c_t *h, uint8_t addr, uint16_t reg, uint8_t reg_size,
                           const uint8_t *data, uint16_t len);

/**
 * @brief Lê @p len bytes a partir do registrador @p reg (escrita do registrador + repeated start).
 * @param[in,out] h        Handle inicializado.
 * @param[in]     addr     Endereço de 7 bits.
 * @param[in]     reg      Endereço do registrador (MSB primeiro se @p reg_size = 2).
 * @param[in]     reg_size Tamanho do endereço do registrador: 1 ou 2 bytes.
 * @param[out]    data     Buffer de destino.
 * @param[in]     len      Quantidade de bytes (> 0).
 * @retval DRIVER_OK Sucesso.
 * @return Erros DRIVER_ERR_I2C_* em caso de NACK, timeout, erro de barramento etc.
 */
driver_err_t i2c_mem_read(i2c_t *h, uint8_t addr, uint16_t reg, uint8_t reg_size,
                          uint8_t *data, uint16_t len);

/**
 * @brief Verifica se um dispositivo responde (ACK) no endereço.
 * @param[in,out] h      Handle inicializado.
 * @param[in]     addr   Endereço de 7 bits.
 * @param[in]     trials Número de tentativas (0 é tratado como 1).
 * @retval DRIVER_OK           Dispositivo respondeu.
 * @retval DRIVER_ERR_I2C_NACK Nenhuma resposta em todas as tentativas.
 * @return Outros erros DRIVER_ERR_I2C_* em caso de problema no barramento.
 */
driver_err_t i2c_is_device_ready(i2c_t *h, uint8_t addr, uint8_t trials);

/**
 * @brief Destrava o barramento quando um escravo prende o SDA em nível baixo.
 *
 * Desliga o periférico, gera até 9 pulsos em SCL por GPIO (até o SDA subir),
 * gera um STOP, devolve os pinos ao modo AF e reconfigura o periférico.
 * Use após DRIVER_ERR_I2C_BUSY ou DRIVER_ERR_I2C_TIMEOUT recorrentes.
 *
 * @param[in,out] h Handle inicializado.
 * @retval DRIVER_OK          Barramento livre.
 * @retval DRIVER_ERR_I2C_BUSY O SDA continua em nível baixo.
 * @return Também propaga os erros do driver GPIO.
 */
driver_err_t i2c_bus_recover(i2c_t *h);

#endif /* I2C_H */
