/**
 * @file gptim.h
 * @brief Base de tempo periódica com os timers general purpose do STM32F411.
 *
 * Gera uma interrupção de update a cada período e chama um callback.
 *
 * | Timer      | Barramento | ARR     | Vetor                                   |
 * |------------|------------|---------|-----------------------------------------|
 * | TIM2, TIM5 | APB1       | 32 bits | próprio (TIM2_IRQHandler, TIM5_...)     |
 * | TIM3, TIM4 | APB1       | 16 bits | próprio (TIM3_IRQHandler, TIM4_...)     |
 * | TIM9       | APB2       | 16 bits | TIM1_BRK_TIM9_IRQHandler (com o TIM1)   |
 * | TIM10      | APB2       | 16 bits | TIM1_UP_TIM10_IRQHandler (com o TIM1)   |
 * | TIM11      | APB2       | 16 bits | TIM1_TRG_COM_TIM11_IRQHandler (com o TIM1) |
 *
 * O driver não define os handlers de interrupção: a aplicação chama
 * gptim_irq_handler() de dentro do handler do timer usado. Nos vetores
 * compartilhados com o TIM1, chame também o handler do inverter, se usado.
 *
 * O timer não pode ser usado ao mesmo tempo para PWM (drv8313 usa TIM1 a TIM5).
 *
 * Exemplo (TIM10 a cada 10 ms, notificando uma task):
 * @code
 * static gptim_t tim;
 *
 * const gptim_config_t cfg = {
 *     .timer = TIM10,
 *     .timer_clock_hz = 16000000U,
 *     .period_us = 10000U,
 *     .irq_priority = 6,              // >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
 *     .callback = task_as5600_notify_from_isr,
 *     .context = NULL,
 * };
 *
 * gptim_init(&tim, &cfg);
 * gptim_start(&tim);
 *
 * void TIM1_UP_TIM10_IRQHandler(void)
 * {
 *     gptim_irq_handler(&tim);
 * }
 * @endcode
 */

#ifndef GPTIM_H
#define GPTIM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <stm32f411xe.h>

#include "drivers/driver_err.h"

/**
 * @brief Callback chamado a cada update event, em contexto de interrupção.
 * @param context Ponteiro definido em gptim_config_t::context.
 */
typedef void (*gptim_callback_t)(void *context);

/** @brief Configuração do timer. */
typedef struct
{
    TIM_TypeDef *timer;        /**< TIM2, TIM3, TIM4, TIM5, TIM9, TIM10 ou TIM11. */
    uint32_t timer_clock_hz;   /**< Clock real do timer, em Hz (o dobro do PCLK se o prescaler do APB for != 1). */
    uint32_t period_us;        /**< Período da interrupção, em µs. */
    uint8_t irq_priority;      /**< Prioridade no NVIC (0 a 15). Para usar a API *FromISR
                                    do FreeRTOS, deve ser >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY. */
    gptim_callback_t callback; /**< Chamado a cada período (NULL = nenhum). */
    void *context;             /**< Argumento repassado ao callback. */
} gptim_config_t;

/**
 * @brief Handle do timer.
 * @note Os campos são de uso interno; não os modifique diretamente.
 */
typedef struct
{
    TIM_TypeDef *timer;        /**< Timer em uso. */
    uint32_t timer_clock_hz;   /**< Clock do timer, em Hz. */
    uint32_t max_arr;          /**< 0xFFFF ou 0xFFFFFFFF (TIM2/TIM5). */
    IRQn_Type irq;             /**< Vetor de interrupção. */
    bool shared_irq;           /**< true se o vetor é compartilhado com o TIM1. */
    gptim_callback_t callback; /**< Callback registrado. */
    void *context;             /**< Argumento do callback. */
    bool initialized;          /**< true após gptim_init() bem-sucedido. */
} gptim_t;

/**
 * @brief Configura o timer (PSC/ARR e interrupção de update). O timer fica parado.
 *
 * PSC e ARR são escolhidos para o período exato quando possível; senão, com o
 * menor prescaler (erro de até 1 contagem).
 *
 * @param[out] h   Handle a ser preenchido.
 * @param[in]  cfg Configuração desejada.
 * @retval DRIVER_OK                   Sucesso.
 * @retval DRIVER_ERR_INVALID_ARG      NULL, timer não suportado, clock ou período zero, prioridade > 15.
 * @retval DRIVER_ERR_TIM_OUT_OF_RANGE Período curto ou longo demais para o clock.
 */
driver_err_t gptim_init(gptim_t *h, const gptim_config_t *cfg);

/**
 * @brief Para o timer, desabilita a interrupção e desliga o clock dele.
 * @note Nos vetores compartilhados com o TIM1, o NVIC continua habilitado.
 * @param[in,out] h Handle inicializado.
 * @retval DRIVER_OK                      Sucesso.
 * @retval DRIVER_ERR_TIM_NOT_INITIALIZED Handle não inicializado.
 */
driver_err_t gptim_deinit(gptim_t *h);

/**
 * @brief Zera o contador e inicia o timer.
 * @param[in,out] h Handle inicializado.
 * @retval DRIVER_OK                      Sucesso.
 * @retval DRIVER_ERR_TIM_NOT_INITIALIZED Handle não inicializado.
 */
driver_err_t gptim_start(gptim_t *h);

/**
 * @brief Para o timer (o callback deixa de ser chamado).
 * @param[in,out] h Handle inicializado.
 * @retval DRIVER_OK                      Sucesso.
 * @retval DRIVER_ERR_TIM_NOT_INITIALIZED Handle não inicializado.
 */
driver_err_t gptim_stop(gptim_t *h);

/**
 * @brief Muda o período; com o timer rodando, vale a partir do próximo update.
 * @param[in,out] h         Handle inicializado.
 * @param[in]     period_us Novo período, em µs (> 0).
 * @retval DRIVER_OK                      Sucesso.
 * @retval DRIVER_ERR_INVALID_ARG         @p period_us é zero.
 * @retval DRIVER_ERR_TIM_OUT_OF_RANGE    Período inatingível com o clock do timer.
 * @retval DRIVER_ERR_TIM_NOT_INITIALIZED Handle não inicializado.
 */
driver_err_t gptim_set_period_us(gptim_t *h, uint32_t period_us);

/**
 * @brief Informa se o timer está contando.
 * @param[in] h Handle do timer.
 * @return true se o timer foi iniciado (false se @p h for NULL ou não inicializado).
 */
bool gptim_is_running(const gptim_t *h);

/**
 * @brief Tratamento da interrupção; chame do handler do vetor do timer.
 *
 * Limpa o flag UIF e executa o callback. Não faz nada se o update não veio
 * deste timer (vetores compartilhados com o TIM1).
 *
 * @param[in,out] h Handle do timer.
 */
void gptim_irq_handler(gptim_t *h);

#endif /* GPTIM_H */
