/**
 * @file task_as5600.h
 * @brief Task do FreeRTOS que detecta e lê o encoder magnético AS5600.
 *
 * Fluxo da task:
 * 1. espera o power-up do sensor e procura o encoder no endereço primário
 *    (0x36, AS5600) e depois no secundário (0x40, AS5600L);
 * 2. sem nenhum dos dois, fica presa avisando a cada #TASK_AS5600_ERROR_MS;
 * 3. configura o encoder e lê o RAW ANGLE (0 a 4095) a cada notificação.
 *
 * A leitura é disparada por uma interrupção periódica (um timer gptim, por exemplo) que
 * chama task_as5600_notify_from_isr(). Notificações que chegam antes de a task
 * estar pronta para ler são descartadas; se a task atrasar, as notificações
 * acumuladas valem por uma só leitura.
 *
 * Uso:
 * @code
 * static i2c_t i2c;
 *
 * i2c_init(&i2c, &i2c_cfg);
 * task_as5600_create(&i2c, log_write);   // log_write: void (const char *)
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

/**
 * @brief Período esperado entre notificações, em ms.
 * @note Quem define o período é o timer; este valor só é usado para agrupar os logs de erro.
 */
#ifndef TASK_AS5600_PERIOD_MS
#define TASK_AS5600_PERIOD_MS 10U
#endif

/** @brief Tempo sem notificação até a task avisar que o timer parou, em ms. */
#ifndef TASK_AS5600_NOTIFY_TIMEOUT_MS
#define TASK_AS5600_NOTIFY_TIMEOUT_MS 1000U
#endif

/** @brief Intervalo da mensagem de erro quando o encoder não é encontrado, em ms. */
#ifndef TASK_AS5600_ERROR_MS
#define TASK_AS5600_ERROR_MS 1000U
#endif

/** @brief Espera inicial pelo power-up do sensor, em ms (datasheet: até 10 ms). */
#ifndef TASK_AS5600_POWERUP_MS
#define TASK_AS5600_POWERUP_MS 10U
#endif

/** @brief Tamanho da stack da task, em words (o vsnprintf dos logs precisa de folga). */
#ifndef TASK_AS5600_STACK_SIZE
#define TASK_AS5600_STACK_SIZE (configMINIMAL_STACK_SIZE * 4)
#endif

/** @brief Tamanho máximo de uma mensagem de log, com o '\0'. */
#ifndef TASK_AS5600_LOG_SIZE
#define TASK_AS5600_LOG_SIZE 80U
#endif

/**
 * @brief Função de log usada pela task.
 *
 * Recebe uma linha já formatada, terminada em "\r\n". É chamada no contexto da
 * task; se outras tasks também escrevem no mesmo destino, proteja-a com um mutex.
 */
typedef void (*task_as5600_log_t)(const char *msg);

/** @brief Prioridade da task. */
#ifndef TASK_AS5600_PRIORITY
#define TASK_AS5600_PRIORITY 2
#endif

/** @brief Handle da task (recebe as notificações de leitura). */
extern TaskHandle_t task_as5600_handle;

/**
 * @brief Cria a task do encoder.
 * @param[in] i2c Barramento já inicializado com i2c_init() (pode ser compartilhado).
 * @param[in] log Função de log (NULL desativa os logs).
 * @retval pdPASS Task criada.
 * @retval pdFAIL @p i2c é NULL ou falta heap para a task.
 */
BaseType_t task_as5600_create(i2c_t *i2c, task_as5600_log_t log);

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
 * @brief Último RAW ANGLE lido.
 * @return Ângulo de 0 a 4095 (0 enquanto task_as5600_is_ready() for false).
 */
uint16_t task_as5600_get_raw(void);

/**
 * @brief Informa se o encoder foi encontrado e já tem uma leitura válida.
 * @return true após a primeira leitura bem-sucedida.
 */
bool task_as5600_is_ready(void);

#endif /* SYSTEM_AS5600 */
