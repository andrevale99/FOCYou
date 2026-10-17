/**
 * @file task_logger.h
 * @brief Task do FreeRTOS que envia um CSV periódico e os logs de texto pela UART com DMA.
 *
 * Saída na serial (para gravar com `cat /dev/ttyUSB0 > log.csv`):
 *
 *     t_us,<colunas>          <- cabeçalho, enviado uma vez quando a task começa
 *     1234567,875,375,...     <- uma linha a cada period_ms
 *     # [drv8313] estado: ... <- logs de texto, sempre começando com '#'
 *
 * No Python: `pd.read_csv("log.csv", comment="#")`.
 *
 * Funcionamento:
 * - a cada @c period_ms a task chama @c sample, que preenche os valores das
 *   colunas, monta a linha em texto (sem printf) com o tempo em µs na frente e
 *   envia por DMA, sem bloquear;
 * - os logs de texto (task_logger_write()) entram numa fila; no máximo uma
 *   mensagem vai junto com cada linha do CSV;
 * - se o DMA ainda estiver ocupado quando a linha seguinte ficar pronta, ela é
 *   descartada (nunca atrasa o controle). As descartadas aparecem como saltos no
 *   t_us e são contadas num log a cada 1 s.
 *
 * Tempo: contador de ciclos do núcleo (DWT->CYCCNT) estendido para 64 bits e
 * convertido para µs; não usa nenhum timer.
 */

#ifndef TASK_LOGGER_H
#define TASK_LOGGER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "drivers/uart/uart.h"

/** @brief Maior número de colunas do CSV (sem contar t_us). */
#ifndef TASK_LOGGER_MAX_COLUMNS
#define TASK_LOGGER_MAX_COLUMNS 12U
#endif

/** @brief Tamanho máximo de uma mensagem de texto, já com o "# ". */
#ifndef TASK_LOGGER_MSG_MAX
#define TASK_LOGGER_MSG_MAX 100U
#endif

/** @brief Capacidade da fila de mensagens de texto, em bytes. */
#ifndef TASK_LOGGER_QUEUE_BYTES
#define TASK_LOGGER_QUEUE_BYTES 1024U
#endif

/** @brief Intervalo do log de linhas descartadas, em ms. */
#ifndef TASK_LOGGER_DROP_REPORT_MS
#define TASK_LOGGER_DROP_REPORT_MS 1000U
#endif

/** @brief Tamanho da stack da task, em words. */
#ifndef TASK_LOGGER_STACK_SIZE
#define TASK_LOGGER_STACK_SIZE (configMINIMAL_STACK_SIZE * 2)
#endif

/** @brief Prioridade da task (a mais baixa: nunca atrasa o controle). */
#ifndef TASK_LOGGER_PRIORITY
#define TASK_LOGGER_PRIORITY 1
#endif

/**
 * @brief Preenche os valores de uma linha do CSV, na ordem do cabeçalho.
 * @param values Vetor com @c n_columns posições.
 */
typedef void (*task_logger_sample_t)(int32_t *values);

/** @brief Configuração do logger. */
typedef struct
{
    USART_TypeDef *usart;        /**< UART já inicializada (uart_init()); o DMA é preparado aqui. */
    const char *header;          /**< Nomes das colunas separados por vírgula, sem o t_us. */
    uint8_t n_columns;           /**< Quantidade de colunas (1 a #TASK_LOGGER_MAX_COLUMNS). */
    task_logger_sample_t sample; /**< Leitura dos valores (NULL = só logs de texto). */
    uint32_t period_ms;          /**< Período das linhas do CSV, em ms (>= 1). */
} task_logger_config_t;

/** @brief Handle da task. */
extern TaskHandle_t task_logger_handle;

/**
 * @brief Prepara o DMA da UART e cria a task.
 * @param[in] cfg Configuração (copiada; o texto do cabeçalho não é copiado e precisa continuar válido).
 * @retval pdPASS Task criada.
 * @retval pdFAIL Configuração inválida, falha no DMA ou falta de heap.
 */
BaseType_t task_logger_create(const task_logger_config_t *cfg);

/**
 * @brief Envia uma mensagem de texto; cada linha sai com "# " na frente.
 *
 * Com o logger rodando, a mensagem entra na fila e a função retorna na hora (se a
 * fila estiver cheia, a mensagem é descartada). Antes disso (ou antes do
 * scheduler), envia direto pela UART, bloqueando.
 *
 * Pode ser chamada de qualquer task; não chame de ISR.
 *
 * @param[in] msg Texto; linhas vazias são ignoradas.
 */
void task_logger_write(const char *msg);

/** @brief Total de linhas do CSV descartadas desde o início. */
uint32_t task_logger_get_dropped(void);

#endif /* TASK_LOGGER_H */
