/**
 * @file drv8313.h
 * @brief Driver do DRV8313 (3 meias pontes) para controle de motor BLDC no STM32F411.
 *
 * Pensado para o módulo SimpleFOCmini:
 * - os três ENx estão ligados em um único pino EN;
 * - IN1, IN2 e IN3 recebem PWM, um por fase;
 * - nFAULT, nRESET e nSLEEP são opcionais.
 *
 * Cada IN comanda uma meia ponte; não são usados pares complementares (CHxN)
 * nem o dead-time do timer. O PWM sai nos canais CH1, CH2 e CH3 de um único timer:
 * IN1 = CH1, IN2 = CH2, IN3 = CH3. Timers suportados: TIM1 a TIM5.
 *
 * Exemplo (TIM3 em PA6, PA7 e PB0, AF2; PWM de 20 kHz center-aligned):
 * @code
 * static drv8313_t drv;
 *
 * const drv8313_config_t cfg = {
 *     .timer = TIM3,
 *     .timer_clock_hz = 16000000U,
 *     .pwm_frequency_hz = 20000U,
 *     .center_aligned = true,
 *     .in = {{GPIOA, 6, 2}, {GPIOA, 7, 2}, {GPIOB, 0, 2}},
 *
 *     .en = {GPIOB, 4},
 *     .n_fault = {GPIOB, 5},        // opcionais: port = NULL quando não usados
 *     .n_reset = {NULL, 0},
 *     .n_sleep = {NULL, 0},
 *
 *     .delay_ms = NULL,             // obrigatório só com n_reset ou n_sleep
 * };
 *
 * drv8313_init(&drv, &cfg);
 * drv8313_enable(&drv);
 *
 * const drv8313_duty_t duty = {0.5f, 0.5f, 0.5f};
 * drv8313_set_duty_cycle(&drv, &duty);
 * @endcode
 */

#ifndef DRV8313_H
#define DRV8313_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <stm32f411xe.h>

/** @brief Número de fases (meias pontes) do DRV8313. */
#define DRV8313_NUM_PHASES 3U

/** @brief Duração do pulso em nRESET, em ms (valor conservador; confira o datasheet). */
#ifndef DRV8313_RESET_PULSE_MS
#define DRV8313_RESET_PULSE_MS 1U
#endif

/** @brief Espera após sair de sleep ou reset, em ms (valor conservador; confira o datasheet). */
#ifndef DRV8313_WAKE_DELAY_MS
#define DRV8313_WAKE_DELAY_MS 2U
#endif

/** @brief Códigos de retorno do driver DRV8313. */
typedef enum
{
    DRV8313_OK = 0,                  /**< Operação executada com sucesso. */
    DRV8313_ERR_INVALID_ARG = -1,    /**< Ponteiro NULL ou parâmetro inválido. */
    DRV8313_ERR_NOT_INITIALIZED = -2,/**< drv8313_init() ainda não foi chamado. */
    DRV8313_ERR_INVALID_TIMER = -3,  /**< Timer diferente de TIM1..TIM5. */
    DRV8313_ERR_OUT_OF_RANGE = -4,   /**< Frequência de PWM inatingível com o clock do timer. */
    DRV8313_ERR_NO_DELAY = -5,       /**< delay_ms é NULL, mas nRESET ou nSLEEP foi configurado. */
    DRV8313_ERR_GPIO = -6,           /**< O driver GPIO rejeitou a porta, o pino ou a configuração. */
    DRV8313_ERR_NOT_CONFIGURED = -7, /**< Pino opcional (nFAULT, nRESET ou nSLEEP) não configurado. */
    DRV8313_ERR_FAULT = -8,          /**< nFAULT ativo (falha no driver). */
    DRV8313_ERR_ASLEEP = -9,         /**< O driver está em sleep. */
} drv8313_err_t;

/** @brief Pino digital de controle. Com @c port = NULL o pino é considerado não usado. */
typedef struct
{
    GPIO_TypeDef *port; /**< Porta GPIO (NULL = pino não usado, só para os opcionais). */
    uint16_t pin;       /**< Número do pino (0 a 15). */
} drv8313_pin_t;

/** @brief Pino de PWM (entrada INx do DRV8313). */
typedef struct
{
    GPIO_TypeDef *port;         /**< Porta GPIO. */
    uint16_t pin;               /**< Número do pino (0 a 15). */
    uint8_t alternate_function; /**< Alternate function do timer para o pino. */
} drv8313_pwm_pin_t;

/** @brief Configuração do driver. */
typedef struct
{
    TIM_TypeDef *timer;          /**< Timer do PWM: TIM1, TIM2, TIM3, TIM4 ou TIM5. */
    uint32_t timer_clock_hz;     /**< Clock real do timer, em Hz. */
    uint32_t pwm_frequency_hz;   /**< Frequência do PWM, em Hz. */
    bool center_aligned;         /**< true: center-aligned (recomendado para FOC); false: edge-aligned. */

    drv8313_pwm_pin_t in[DRV8313_NUM_PHASES]; /**< IN1, IN2, IN3 (CH1, CH2, CH3 do timer). */

    drv8313_pin_t en;      /**< EN (obrigatório; os três ENx do módulo estão ligados juntos). */
    drv8313_pin_t n_fault; /**< nFAULT (opcional, entrada com pull-up). */
    drv8313_pin_t n_reset; /**< nRESET (opcional). */
    drv8313_pin_t n_sleep; /**< nSLEEP (opcional). */

    void (*delay_ms)(uint32_t); /**< Atraso em ms; obrigatório se n_reset ou n_sleep for usado. */
} drv8313_config_t;

/**
 * @brief Handle do driver.
 * @note Os campos são de uso interno; não os modifique diretamente.
 */
typedef struct
{
    drv8313_config_t cfg; /**< Cópia da configuração. */
    uint16_t arr;         /**< ARR calculado para a frequência de PWM. */
    bool initialized;     /**< true após drv8313_init() bem-sucedido. */
    bool enabled;         /**< true com EN em nível alto. */
    bool sleeping;        /**< true com nSLEEP em nível baixo. */
} drv8313_t;

/** @brief Duty cycles normalizados (0.0 a 1.0) das três fases. */
typedef struct
{
    float a; /**< Fase 1 (IN1, CH1). */
    float b; /**< Fase 2 (IN2, CH2). */
    float c; /**< Fase 3 (IN3, CH3). */
} drv8313_duty_t;

/**
 * @brief Configura os pinos de controle, os pinos de PWM e o timer.
 *
 * O driver sai da inicialização desabilitado (EN em nível baixo), acordado
 * (nSLEEP em nível alto) e sem reset (nRESET em nível alto), com duty em 0%.
 * O PWM já roda; as saídas só ficam ativas após drv8313_enable().
 *
 * @param[out] drv Handle a ser preenchido.
 * @param[in]  cfg Configuração desejada.
 *
 * @retval DRV8313_OK                 Sucesso.
 * @retval DRV8313_ERR_INVALID_ARG    NULL, EN ou pino de PWM sem porta, clock ou frequência zero.
 * @retval DRV8313_ERR_INVALID_TIMER  Timer diferente de TIM1..TIM5.
 * @retval DRV8313_ERR_NO_DELAY       nRESET ou nSLEEP usado sem @c delay_ms.
 * @retval DRV8313_ERR_OUT_OF_RANGE   Frequência inatingível com o clock do timer.
 * @retval DRV8313_ERR_GPIO           Porta, pino ou configuração rejeitados pelo driver GPIO.
 */
drv8313_err_t drv8313_init(drv8313_t *drv, const drv8313_config_t *cfg);

/**
 * @brief Desabilita o driver, para o timer e desliga o clock do timer.
 *
 * @param[in,out] drv Handle inicializado.
 *
 * @retval DRV8313_OK                   Sucesso.
 * @retval DRV8313_ERR_NOT_INITIALIZED  Handle não inicializado.
 */
drv8313_err_t drv8313_deinit(drv8313_t *drv);

/**
 * @brief Habilita as saídas do driver (EN em nível alto).
 *
 * @param[in,out] drv Handle inicializado.
 *
 * @retval DRV8313_OK                   Saídas habilitadas.
 * @retval DRV8313_ERR_NOT_INITIALIZED  Handle não inicializado.
 * @retval DRV8313_ERR_ASLEEP           Driver em sleep; chame drv8313_wake().
 * @retval DRV8313_ERR_FAULT            nFAULT ativo (só verificado se nFAULT estiver configurado).
 * @retval DRV8313_ERR_GPIO             Falha ao ler ou escrever nos pinos.
 */
drv8313_err_t drv8313_enable(drv8313_t *drv);

/**
 * @brief Desabilita as saídas do driver (EN em nível baixo, meias pontes em alta impedância).
 *
 * @param[in,out] drv Handle inicializado.
 *
 * @retval DRV8313_OK                   Sucesso.
 * @retval DRV8313_ERR_NOT_INITIALIZED  Handle não inicializado.
 * @retval DRV8313_ERR_GPIO             Falha ao escrever no pino.
 */
drv8313_err_t drv8313_disable(drv8313_t *drv);

/**
 * @brief Coloca o driver em sleep (desabilita as saídas e leva nSLEEP a nível baixo).
 *
 * @param[in,out] drv Handle inicializado.
 *
 * @retval DRV8313_OK                    Sucesso.
 * @retval DRV8313_ERR_NOT_INITIALIZED   Handle não inicializado.
 * @retval DRV8313_ERR_NOT_CONFIGURED    nSLEEP não configurado.
 * @retval DRV8313_ERR_GPIO              Falha ao escrever nos pinos.
 */
drv8313_err_t drv8313_sleep(drv8313_t *drv);

/**
 * @brief Tira o driver do sleep (nSLEEP em nível alto) e aguarda #DRV8313_WAKE_DELAY_MS.
 *
 * As saídas continuam desabilitadas; use drv8313_enable() em seguida.
 *
 * @param[in,out] drv Handle inicializado.
 *
 * @retval DRV8313_OK                    Sucesso (também se já estava acordado).
 * @retval DRV8313_ERR_NOT_INITIALIZED   Handle não inicializado.
 * @retval DRV8313_ERR_NOT_CONFIGURED    nSLEEP não configurado.
 * @retval DRV8313_ERR_GPIO              Falha ao escrever no pino.
 */
drv8313_err_t drv8313_wake(drv8313_t *drv);

/**
 * @brief Gera um pulso em nRESET para reiniciar a lógica do driver.
 *
 * Desabilita as saídas antes do pulso; use drv8313_enable() depois.
 * Consulte o datasheet para saber quais falhas exigem reset.
 *
 * @param[in,out] drv Handle inicializado.
 *
 * @retval DRV8313_OK                    Sucesso.
 * @retval DRV8313_ERR_NOT_INITIALIZED   Handle não inicializado.
 * @retval DRV8313_ERR_NOT_CONFIGURED    nRESET não configurado.
 * @retval DRV8313_ERR_ASLEEP            Driver em sleep.
 * @retval DRV8313_ERR_GPIO              Falha ao escrever nos pinos.
 */
drv8313_err_t drv8313_reset(drv8313_t *drv);

/**
 * @brief Lê o pino nFAULT (ativo em nível baixo).
 *
 * @param[in]  drv   Handle inicializado.
 * @param[out] fault true se o driver sinaliza falha.
 *
 * @retval DRV8313_OK                    Leitura realizada.
 * @retval DRV8313_ERR_INVALID_ARG       @p fault é NULL.
 * @retval DRV8313_ERR_NOT_INITIALIZED   Handle não inicializado.
 * @retval DRV8313_ERR_NOT_CONFIGURED    nFAULT não configurado.
 * @retval DRV8313_ERR_GPIO              Falha ao ler o pino.
 */
drv8313_err_t drv8313_get_fault(const drv8313_t *drv, bool *fault);

/**
 * @brief Atualiza o duty cycle das três fases.
 *
 * Valores fora de 0.0 a 1.0 (e NaN) são saturados. Os registradores têm preload:
 * o novo valor passa a valer no próximo update event.
 * Em edge-aligned, 100% fica uma contagem abaixo do máximo.
 *
 * @param[in,out] drv  Handle inicializado.
 * @param[in]     duty Duty cycles normalizados.
 *
 * @retval DRV8313_OK                    Sucesso.
 * @retval DRV8313_ERR_INVALID_ARG       @p duty é NULL.
 * @retval DRV8313_ERR_NOT_INITIALIZED   Handle não inicializado.
 */
drv8313_err_t drv8313_set_duty_cycle(drv8313_t *drv, const drv8313_duty_t *duty);

/**
 * @brief Informa se as saídas estão habilitadas (EN em nível alto).
 *
 * @param[in] drv Handle do driver.
 * @return true se habilitadas; false caso contrário (ou se @p drv for NULL).
 */
bool drv8313_is_enabled(const drv8313_t *drv);

#endif /* DRV8313_H */