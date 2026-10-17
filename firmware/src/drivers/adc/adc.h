/**
 * @file adc.h
 * @brief Driver do ADC1 do STM32F411: conversão única por software (polling com timeout).
 *
 * Resolução de 12 bits, alinhamento à direita. O clock do ADC é o PCLK2
 * dividido por 2, 4, 6 ou 8 (o menor divisor com ADCCLK <= 36 MHz).
 *
 * Tempo de uma conversão = (amostragem + 12) ciclos de ADCCLK. Com PCLK2 = 16 MHz
 * (ADCCLK = 8 MHz) e 56 ciclos de amostragem: (56 + 12) / 8 MHz = 8,5 µs.
 *
 * Pinos com entrada analógica no STM32F411:
 *
 * | Pinos     | Canais    |
 * |-----------|-----------|
 * | PA0..PA7  | IN0..IN7  |
 * | PB0, PB1  | IN8, IN9  |
 * | PC0..PC5  | IN10..IN15|
 *
 * Exemplo:
 * @code
 * static adc_t adc;
 * uint8_t ch;
 * uint16_t value;
 *
 * const adc_config_t cfg = {
 *     .adc = ADC1,
 *     .pclk2_hz = 16000000U,
 *     .sample_time = ADC_SAMPLE_56,
 *     .timeout = 0,                       // 0 = ADC_DEFAULT_TIMEOUT
 * };
 *
 * adc_init(&adc, &cfg);
 * adc_config_pin(&adc, GPIOA, 1, &ch);    // PA1 -> IN1
 * adc_read(&adc, ch, &value);             // 0 a 4095
 * @endcode
 */

#ifndef ADC_H
#define ADC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <stm32f411xe.h>

#include "drivers/driver_err.h"

/** @brief Timeout padrão, em iterações do laço de espera do fim de conversão. */
#ifndef ADC_DEFAULT_TIMEOUT
#define ADC_DEFAULT_TIMEOUT 10000U
#endif

/** @brief Maior valor de uma conversão de 12 bits. */
#define ADC_MAX_VALUE 4095U

/** @brief Tempo de amostragem, em ciclos de ADCCLK (campo SMPx). */
typedef enum
{
    ADC_SAMPLE_3 = 0,   /**< 3 ciclos. */
    ADC_SAMPLE_15 = 1,  /**< 15 ciclos. */
    ADC_SAMPLE_28 = 2,  /**< 28 ciclos. */
    ADC_SAMPLE_56 = 3,  /**< 56 ciclos. */
    ADC_SAMPLE_84 = 4,  /**< 84 ciclos. */
    ADC_SAMPLE_112 = 5, /**< 112 ciclos. */
    ADC_SAMPLE_144 = 6, /**< 144 ciclos. */
    ADC_SAMPLE_480 = 7, /**< 480 ciclos (fontes de alta impedância). */
} adc_sample_time_t;

/** @brief Configuração do ADC. */
typedef struct
{
    ADC_TypeDef *adc;              /**< Só ADC1 no STM32F411. */
    uint32_t pclk2_hz;             /**< Clock real do APB2, em Hz. */
    adc_sample_time_t sample_time; /**< Tempo de amostragem aplicado em adc_config_pin(). */
    uint32_t timeout;              /**< Iterações de espera por conversão (0 = #ADC_DEFAULT_TIMEOUT). */
} adc_config_t;

/**
 * @brief Handle do ADC.
 * @note Os campos são de uso interno; não os modifique diretamente.
 */
typedef struct
{
    ADC_TypeDef *adc;              /**< ADC em uso. */
    adc_sample_time_t sample_time; /**< Tempo de amostragem dos canais. */
    uint32_t timeout;              /**< Iterações de espera por conversão. */
    bool initialized;              /**< true após adc_init() bem-sucedido. */
} adc_t;

/**
 * @brief Liga o clock, escolhe o prescaler e liga o ADC (12 bits, conversão única).
 * @param[out] h   Handle a ser preenchido.
 * @param[in]  cfg Configuração desejada.
 * @retval DRIVER_OK              Sucesso.
 * @retval DRIVER_ERR_INVALID_ARG NULL, ADC diferente de ADC1, PCLK2 zero ou alto demais (> 288 MHz)
 *                                ou tempo de amostragem inválido.
 */
driver_err_t adc_init(adc_t *h, const adc_config_t *cfg);

/**
 * @brief Desliga o ADC e o clock dele.
 * @param[in,out] h Handle inicializado.
 * @retval DRIVER_OK                      Sucesso.
 * @retval DRIVER_ERR_ADC_NOT_INITIALIZED Handle não inicializado.
 */
driver_err_t adc_deinit(adc_t *h);

/**
 * @brief Coloca o pino em modo analógico e configura o tempo de amostragem do canal.
 * @param[in,out] h       Handle inicializado.
 * @param[in]     port    Porta GPIO.
 * @param[in]     pin     Número do pino.
 * @param[out]    channel Canal do ADC correspondente (pode ser NULL).
 * @retval DRIVER_OK                      Sucesso.
 * @retval DRIVER_ERR_ADC_INVALID_CHANNEL O pino não tem entrada analógica.
 * @retval DRIVER_ERR_ADC_NOT_INITIALIZED Handle não inicializado.
 * @return Também propaga os erros do driver GPIO.
 */
driver_err_t adc_config_pin(adc_t *h, GPIO_TypeDef *port, uint16_t pin, uint8_t *channel);

/**
 * @brief Faz uma conversão do canal e espera o resultado.
 * @param[in,out] h       Handle inicializado.
 * @param[in]     channel Canal (0 a 18).
 * @param[out]    value   Resultado, de 0 a 4095.
 * @retval DRIVER_OK                      Sucesso.
 * @retval DRIVER_ERR_INVALID_ARG         @p value é NULL.
 * @retval DRIVER_ERR_ADC_INVALID_CHANNEL Canal acima de 18.
 * @retval DRIVER_ERR_ADC_TIMEOUT         A conversão não terminou.
 * @retval DRIVER_ERR_ADC_NOT_INITIALIZED Handle não inicializado.
 * @note Bloqueia durante a conversão (alguns µs); não chame de vários contextos ao mesmo tempo.
 */
driver_err_t adc_read(adc_t *h, uint8_t channel, uint16_t *value);

/**
 * @brief Faz @p samples conversões seguidas e retorna a média (reduz o ruído).
 * @param[in,out] h       Handle inicializado.
 * @param[in]     channel Canal (0 a 18).
 * @param[in]     samples Número de conversões (0 é tratado como 1).
 * @param[out]    value   Média, de 0 a 4095.
 * @return Os mesmos códigos de adc_read().
 */
driver_err_t adc_read_average(adc_t *h, uint8_t channel, uint8_t samples, uint16_t *value);

#endif /* ADC_H */
