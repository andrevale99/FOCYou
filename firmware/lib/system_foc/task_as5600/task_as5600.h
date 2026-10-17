/**
 * @file task_as5600.h
 * @brief Task do FreeRTOS que lê o encoder magnético AS5600 pelo I2C ou pela saída analógica (OUT).
 *
 * Fonte do ângulo (task_as5600_config_t::source):
 * - **I2C** (#TASK_AS5600_SOURCE_I2C): lê o RAW ANGLE pelo barramento. Cada
 *   leitura leva ~150 µs a 400 kHz e ocupa a CPU (polling): use até ~1 kHz.
 *   Sem o sensor em 0x36 nem 0x40, a task fica presa avisando a cada
 *   #TASK_AS5600_ERROR_MS.
 * - **ADC** (#TASK_AS5600_SOURCE_ADC): descrito abaixo.
 *
 * Na fonte ADC, o ângulo é lido no pino OUT do AS5600 (estágio de saída analógico,
 * 0% a 100% de VDD). Com o AS5600 e o ADC na mesma alimentação de 3,3 V a leitura
 * é ratiométrica: 0 a 4095 contagens do ADC = 0 a 360°.
 *
 * Fluxo da task:
 * 1. espera o power-up do sensor;
 * 2. se houver I2C, procura o encoder em 0x36 e depois em 0x40, grava o CONF
 *    (saída analógica de faixa cheia) e confere o ímã. Sem resposta, segue com a
 *    configuração de fábrica, que já é a saída analógica de faixa cheia;
 * 3. a cada notificação, converte a tensão de OUT em ângulo (0 a 4095), descarta
 *    leituras impossíveis (ver abaixo), atualiza as voltas (as5600_track_raw()),
 *    calcula a velocidade na janela @c velocity_window_us e chama o callback de amostra.
 *
 * Taxa de amostragem: o AS5600 atualiza o ângulo a cada ~150 µs (~6,7 kHz);
 * amostrar mais rápido que isso só repete valores.
 *
 * Velocidade: derivar amostra a amostra piora o ruído com a taxa (a 5 kHz, 1
 * contagem = 73 rpm). Por isso a velocidade é a variação de posição dentro de uma
 * janela de tempo fixa (ex.: 1 ms = 15 rpm por contagem), recalculada a cada
 * amostra: a resolução não depende da taxa e o atraso é de meia janela.
 *
 * Leituras descartadas: quando o ângulo passa por 0°/360°, a saída analógica
 * salta de VDD para 0 V (ou o contrário) e uma conversão no meio do salto dá um
 * ângulo qualquer. Uma variação maior que a possível com @c max_rpm é descartada,
 * até #TASK_AS5600_MAX_REJECT vezes seguidas (depois disso a leitura é aceita).
 *
 * A leitura é disparada por uma interrupção periódica (um timer gptim, por exemplo)
 * que chama task_as5600_notify_from_isr(). Se a task atrasar, as notificações
 * acumuladas valem por uma só leitura (o dt passado ao callback conta todas).
 *
 * O callback de amostra (task_as5600_set_sample_callback()) roda no contexto
 * desta task logo após cada leitura: é o ponto para avisar a malha de controle
 * por task notify, sem esperar o próximo ciclo dela.
 *
 * Uso:
 * @code
 * adc_init(&adc, &adc_cfg);
 * adc_config_pin(&adc, GPIOA, 1, &ch);   // OUT do AS5600 em PA1
 *
 * const task_as5600_config_t cfg = {
 *     .adc = &adc, .adc_channel = ch, .oversample = 4,
 *     .adc_min = 0, .adc_max = 4095, .max_rpm = 1500.0f,
 *     .period_us = 200U, .velocity_window_us = 1000U,
 *     .i2c = &i2c,                       // ou NULL sem I2C
 * };
 * task_as5600_create(&cfg, log_write);
 * task_as5600_set_sample_callback(task_drv8313_encoder_sample);
 *
 * gptim_init(&tim, &tim_cfg);            // .callback = task_as5600_notify_from_isr
 * gptim_start(&tim);
 * vTaskStartScheduler();
 * @endcode
 */

#ifndef SYSTEM_AS5600
#define SYSTEM_AS5600

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "as5600.h"
#include "drivers/adc/adc.h"

/** @brief Tempo sem notificação até a task avisar que o timer parou, em ms. */
#ifndef TASK_AS5600_NOTIFY_TIMEOUT_MS
#define TASK_AS5600_NOTIFY_TIMEOUT_MS 1000U
#endif

/** @brief Intervalo dos logs de erro e de leituras descartadas, em ms. */
#ifndef TASK_AS5600_ERROR_MS
#define TASK_AS5600_ERROR_MS 1000U
#endif

/** @brief Espera inicial pelo power-up do sensor, em ms (datasheet: até 10 ms). */
#ifndef TASK_AS5600_POWERUP_MS
#define TASK_AS5600_POWERUP_MS 10U
#endif

/**
 * @brief Intervalo do diagnóstico pelo I2C, em ms (0 desativa).
 *
 * Com o sensor no I2C, loga a cada intervalo o RAW ANGLE e o ANGLE lidos pelo
 * I2C ao lado da leitura do ADC: se o I2C muda com o motor girando e o ADC não,
 * o problema está na saída analógica (ligação ou faixa ZPOS/MPOS/MANG).
 * Cada diagnóstico atrasa uma amostra em ~150 µs (duas leituras I2C).
 */
#ifndef TASK_AS5600_DIAG_MS
#define TASK_AS5600_DIAG_MS 1000U
#endif

/** @brief Maior número de amostras na janela da velocidade. */
#ifndef TASK_AS5600_VEL_MAX_SAMPLES
#define TASK_AS5600_VEL_MAX_SAMPLES 32U
#endif

/** @brief Leituras impossíveis seguidas descartadas antes de aceitar a próxima. */
#ifndef TASK_AS5600_MAX_REJECT
#define TASK_AS5600_MAX_REJECT 3U
#endif

/** @brief Folga para o ruído na rejeição de leituras, em contagens. */
#ifndef TASK_AS5600_NOISE_COUNTS
#define TASK_AS5600_NOISE_COUNTS 16U
#endif

/** @brief Tamanho da stack da task, em words (o vsnprintf dos logs precisa de folga). */
#ifndef TASK_AS5600_STACK_SIZE
#define TASK_AS5600_STACK_SIZE (configMINIMAL_STACK_SIZE * 4)
#endif

/** @brief Tamanho máximo de uma mensagem de log, com o '\0'. */
#ifndef TASK_AS5600_LOG_SIZE
#define TASK_AS5600_LOG_SIZE 80U
#endif

/** @brief Prioridade da task. */
#ifndef TASK_AS5600_PRIORITY
#define TASK_AS5600_PRIORITY 2
#endif

/**
 * @brief Função de log usada pela task.
 *
 * Recebe uma linha já formatada, terminada em "\r\n". É chamada no contexto da
 * task; se outras tasks também escrevem no mesmo destino, proteja-a com um mutex.
 */
typedef void (*task_as5600_log_t)(const char *msg);

/**
 * @brief Callback chamado após cada leitura aceita, no contexto da task.
 * @param raw            Ângulo, de 0 a 4095.
 * @param velocity_rad_s Velocidade mecânica na janela, em rad/s (positiva quando o ângulo cresce).
 * @param dt_s           Tempo desde a amostra anterior, em s.
 */
typedef void (*task_as5600_sample_cb_t)(uint16_t raw, float velocity_rad_s, float dt_s);

/** @brief Fonte do ângulo. */
typedef enum
{
    TASK_AS5600_SOURCE_I2C = 0, /**< RAW ANGLE pelo I2C. */
    TASK_AS5600_SOURCE_ADC = 1, /**< Saída analógica OUT pelo ADC. */
} task_as5600_source_t;

/** @brief Configuração da task. */
typedef struct
{
    task_as5600_source_t source; /**< Fonte do ângulo. */

    /* Só para a fonte ADC */
    adc_t *adc;          /**< ADC já inicializado com adc_init(). */
    uint8_t adc_channel; /**< Canal do pino OUT (retornado por adc_config_pin()). */
    uint8_t oversample;  /**< Conversões por amostra, de 1 a 16 (média que respeita a passagem por 0°). */

    uint16_t adc_min; /**< Leitura do ADC em 0°  (0 com a faixa cheia ideal). */
    uint16_t adc_max; /**< Leitura do ADC em 360° (4095 com a faixa cheia ideal). */

    float max_rpm;      /**< Maior velocidade esperada; define a rejeição de leituras impossíveis. */
    uint32_t period_us; /**< Período do timer que chama task_as5600_notify_from_isr(), em µs. */

    uint32_t velocity_window_us; /**< Janela da velocidade, em µs (de 1 a #TASK_AS5600_VEL_MAX_SAMPLES
                                      amostras; 0 = derivada amostra a amostra). */

    i2c_t *i2c; /**< Fonte I2C: obrigatório. Fonte ADC: opcional, configura o sensor e confere o ímã (NULL = sem I2C). */
} task_as5600_config_t;

/** @brief Handle da task (recebe as notificações de leitura). */
extern TaskHandle_t task_as5600_handle;

/**
 * @brief Cria a task do encoder.
 * @param[in] cfg Configuração (copiada).
 * @param[in] log Função de log (NULL desativa os logs).
 * @retval pdPASS Task criada.
 * @retval pdFAIL Configuração inválida (fonte I2C sem @c i2c; fonte ADC com ADC NULL,
 *                canal > 18, faixa vazia ou oversample fora de 1 a 16; max_rpm <= 0;
 *                período zero) ou falta de heap.
 */
BaseType_t task_as5600_create(const task_as5600_config_t *cfg, task_as5600_log_t log);

/**
 * @brief Registra o callback chamado após cada leitura (NULL remove).
 * @param[in] cb Callback; roda no contexto da task do encoder, mantenha-o curto.
 */
void task_as5600_set_sample_callback(task_as5600_sample_cb_t cb);

/**
 * @brief Pede uma leitura à task; chame de dentro de uma ISR.
 *
 * Tem a assinatura de gptim_callback_t, para ser registrada direto no timer.
 * Não faz nada se a task ainda não foi criada.
 *
 * @param context Não usado.
 * @warning A ISR deve ter prioridade >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY.
 */
void task_as5600_notify_from_isr(void *context);

/**
 * @brief Último ângulo aceito.
 * @return Ângulo de 0 a 4095 (0 enquanto task_as5600_is_ready() for false).
 */
uint16_t task_as5600_get_raw(void);

/**
 * @brief Último ângulo e a posição acumulada correspondente, lidos juntos.
 * @param[out] raw      Ângulo, de 0 a 4095 (com wrap). Pode ser NULL.
 * @param[out] position Posição sem wrap: voltas · 4096 + ângulo (dá a volta em 32 bits). Pode ser NULL.
 */
void task_as5600_get_sample(uint16_t *raw, int32_t *position);

/**
 * @brief Última leitura do ADC, sem conversão (útil para calibrar adc_min e adc_max).
 * @return Contagens do ADC, de 0 a 4095.
 */
uint16_t task_as5600_get_adc(void);

/**
 * @brief Informa se a task já tem uma leitura válida.
 * @return true após a primeira leitura aceita.
 */
bool task_as5600_is_ready(void);

/**
 * @brief Velocidade mecânica da última leitura, na janela, sem outro filtro.
 * @return Velocidade em rpm (positiva quando o ângulo cresce).
 */
float task_as5600_get_velocity_rpm(void);

#endif /* SYSTEM_AS5600 */
