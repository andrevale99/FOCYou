/**
 * @file task_drv8313.h
 * @brief Task do FreeRTOS que controla o motor BLDC por six-step com o DRV8313.
 *
 * Divisão do trabalho:
 * - **ISR de comutação** (timer gptim, registrado pela aplicação): a cada
 *   interrupção avança um setor e aplica os duty cycles. Precisa ser precisa,
 *   por isso não fica na task (o tick do FreeRTOS é de 1 ms).
 * - **Task**: máquina de estados, rampa de velocidade, ajuste do período de
 *   comutação, verificação do nFAULT e comandos vindos de outras tasks.
 *
 * Máquina de estados:
 *
 *     IDLE --start--> ALIGN --align_ms--> RAMP --rpm = alvo--> RUN
 *       ^               |                  |                    |
 *       +----stop-------+------------------+--------------------+
 *     (nFAULT ativo em qualquer estado ligado -> FAULT; clear_fault -> IDLE)
 *
 * - ALIGN: aplica o setor 0 com @c align_u por @c align_ms para posicionar o rotor.
 * - RAMP:  malha aberta; a velocidade sobe de @c start_rpm até o alvo a
 *          @c ramp_rpm_per_s, com tensão u = u_offset + u_per_rpm · rpm (V/f).
 * - RUN:   segue em malha aberta na velocidade alvo; mudanças de alvo usam a
 *          mesma rampa. A malha fechada (com o AS5600) entra aqui no futuro.
 *
 * Uso:
 * @code
 * static drv8313_t drv;
 * static gptim_t comm_tim;
 *
 * drv8313_init(&drv, &drv_cfg);
 * gptim_init(&comm_tim, &tim_cfg);   // .callback = task_drv8313_commutate_from_isr, parado
 * task_drv8313_create(&drv, &comm_tim, &motor_cfg, log_write);
 * task_drv8313_start();
 * vTaskStartScheduler();
 * @endcode
 */

#ifndef TASK_DRV8313_H
#define TASK_DRV8313_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "drv8313.h"
#include "sixstep.h"
#include "drivers/gptim/gptim.h"

/** @brief Período da task (rampa, nFAULT e comandos), em ms. */
#ifndef TASK_DRV8313_PERIOD_MS
#define TASK_DRV8313_PERIOD_MS 1U
#endif

/** @brief Tamanho da stack da task, em words (o vsnprintf dos logs precisa de folga). */
#ifndef TASK_DRV8313_STACK_SIZE
#define TASK_DRV8313_STACK_SIZE (configMINIMAL_STACK_SIZE * 4)
#endif

/** @brief Prioridade da task (acima da task do encoder). */
#ifndef TASK_DRV8313_PRIORITY
#define TASK_DRV8313_PRIORITY 3
#endif

/** @brief Tamanho máximo de uma mensagem de log, com o '\0'. */
#ifndef TASK_DRV8313_LOG_SIZE
#define TASK_DRV8313_LOG_SIZE 80U
#endif

/** @brief Estado do controle do motor. */
typedef enum
{
    TASK_DRV8313_STATE_IDLE = 0, /**< Saídas desligadas, esperando task_drv8313_start(). */
    TASK_DRV8313_STATE_ALIGN,    /**< Alinhando o rotor no setor 0. */
    TASK_DRV8313_STATE_RAMP,     /**< Partida em malha aberta, acelerando até o alvo. */
    TASK_DRV8313_STATE_RUN,      /**< Na velocidade alvo (malha aberta por enquanto). */
    TASK_DRV8313_STATE_FAULT,    /**< nFAULT ativo ou erro do driver; saídas desligadas. */
} task_drv8313_state_t;

/** @brief Parâmetros do motor e da partida em malha aberta. */
typedef struct
{
    uint8_t pole_pairs;    /**< Pares de polos do motor (polos / 2). */
    sixstep_dir_t direction; /**< Sentido de giro. */

    float align_u;         /**< Tensão normalizada do alinhamento (0.0 a 1.0). */
    uint32_t align_ms;     /**< Duração do alinhamento, em ms. */

    float start_rpm;       /**< Velocidade mecânica no início da rampa (> 0). */
    float target_rpm;      /**< Velocidade alvo inicial (>= start_rpm). */
    float max_rpm;         /**< Limite para task_drv8313_set_target_rpm(). */
    float ramp_rpm_per_s;  /**< Aceleração da rampa, em rpm/s (> 0). */

    float u_offset;        /**< V/f: tensão com rpm = 0 (vence a resistência do enrolamento). */
    float u_per_rpm;       /**< V/f: ganho de tensão por rpm (acompanha a BEMF). */
    float u_max;           /**< Limite da tensão normalizada (0.0 a 1.0). */
} task_drv8313_motor_cfg_t;

/**
 * @brief Função de log usada pela task.
 *
 * Recebe uma linha já formatada, terminada em "\r\n". É chamada no contexto da
 * task; se outras tasks também escrevem no mesmo destino, proteja-a com um mutex.
 */
typedef void (*task_drv8313_log_t)(const char *msg);

/** @brief Handle da task (recebe os comandos por task notify). */
extern TaskHandle_t task_drv8313_handle;

/**
 * @brief Cria a task do motor.
 *
 * @param[in] drv   Driver já inicializado com drv8313_init() (saídas desligadas).
 * @param[in] tim   Timer de comutação já inicializado com gptim_init() e parado, com
 *                  @c callback = task_drv8313_commutate_from_isr. A task controla
 *                  o período e o start/stop dele.
 * @param[in] motor Parâmetros do motor (copiados).
 * @param[in] log   Função de log (NULL desativa os logs).
 *
 * @retval pdPASS Task criada.
 * @retval pdFAIL Ponteiro NULL, parâmetro inválido ou falta de heap.
 */
BaseType_t task_drv8313_create(drv8313_t *drv, gptim_t *tim,
                               const task_drv8313_motor_cfg_t *motor,
                               task_drv8313_log_t log);

/**
 * @brief Pede a partida (alinhamento + rampa). Só tem efeito em IDLE.
 * @retval pdPASS Comando enviado.
 * @retval pdFAIL A task não foi criada.
 */
BaseType_t task_drv8313_start(void);

/**
 * @brief Pede a parada: desliga as saídas e o motor para por inércia.
 * @retval pdPASS Comando enviado.
 * @retval pdFAIL A task não foi criada.
 */
BaseType_t task_drv8313_stop(void);

/**
 * @brief Limpa uma falha (pulso em nRESET, se configurado) e volta para IDLE.
 * @retval pdPASS Comando enviado.
 * @retval pdFAIL A task não foi criada.
 */
BaseType_t task_drv8313_clear_fault(void);

/**
 * @brief Muda a velocidade alvo; em RUN, a velocidade segue a rampa até o novo valor.
 * @param[in] rpm Nova velocidade, saturada entre @c start_rpm e @c max_rpm.
 * @return Velocidade alvo efetivamente aplicada.
 */
float task_drv8313_set_target_rpm(float rpm);

/**
 * @brief Muda o sentido de giro. Só aceito em IDLE.
 * @param[in] dir Novo sentido.
 * @return true se aceito.
 */
bool task_drv8313_set_direction(sixstep_dir_t dir);

/** @brief Estado atual da máquina de estados. */
task_drv8313_state_t task_drv8313_get_state(void);

/** @brief Nome do estado (para logs). */
const char *task_drv8313_state_str(task_drv8313_state_t state);

/** @brief Velocidade comandada atual, em rpm (0 fora de RAMP/RUN). */
float task_drv8313_get_rpm(void);

/** @brief Velocidade alvo, em rpm. */
float task_drv8313_get_target_rpm(void);

/** @brief Tensão normalizada aplicada atualmente (0.0 a 1.0). */
float task_drv8313_get_u(void);

/**
 * @brief Avança um setor e aplica os duty cycles; registre como callback do timer de comutação.
 *
 * Tem a assinatura de gptim_callback_t. Não usa a API do FreeRTOS, então o timer
 * pode ter prioridade acima de configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
 * (menor jitter na comutação).
 *
 * @param context Não usado.
 */
void task_drv8313_commutate_from_isr(void *context);

#endif /* TASK_DRV8313_H */
