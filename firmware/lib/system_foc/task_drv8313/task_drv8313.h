/**
 * @file task_drv8313.h
 * @brief Task do FreeRTOS que controla o motor BLDC por six-step com o DRV8313.
 *
 * Divisão do trabalho:
 * - **ISR de comutação** (timer gptim, registrado pela aplicação): escolhe o
 *   setor e aplica os duty cycles.
 *   - malha aberta: avança um setor a cada interrupção (o período define a velocidade);
 *   - malha fechada: roda a taxa fixa (@c comm_rate_hz), estima o ângulo elétrico
 *     entre as amostras do encoder (ângulo + velocidade · tempo) e aplica o setor
 *     90° elétricos à frente do rotor.
 * - **Task**: máquina de estados, rampa, PID de velocidade, nFAULT e comandos.
 *   Na malha fechada ela dorme até a notificação de amostra nova do encoder
 *   (task_drv8313_encoder_sample()), então o PID roda logo após cada leitura.
 *
 * Máquina de estados:
 *
 *     IDLE --start--> ALIGN --2 · align_ms--> RAMP --rpm = fim da rampa--> RUN
 *       ^                                                                   |
 *       +---------------------------- stop ---------------------------------+
 *     (nFAULT ativo, erro do driver ou encoder parado -> FAULT; clear_fault -> IDLE)
 *
 * - ALIGN: alinha o rotor no setor 0 e depois no setor 1 (sem calibração).
 * - RAMP:  partida em malha aberta de @c start_rpm até @c ramp_end_rpm, com
 *          tensão u = u_offset + u_per_rpm · rpm (V/f).
 * - RUN:   com @c closed_loop, troca para a malha fechada sempre no fim da rampa
 *          (como em simulation_bldc_velocidade da simulação):
 *          - a referência do ângulo elétrico é tirada na troca: sentido pelo sinal
 *            da velocidade medida e zero pelo setor atual (setor · 60°) menos
 *            @c switch_lag_deg;
 *          - o setpoint parte da velocidade medida (erro nulo na troca) e sobe
 *            em rampa até o alvo;
 *          - o PID trabalha em pu (base = @c target_rpm da configuração) e a
 *            saída é a tensão inteira, limitada a [0, u_max] no próprio PID
 *            (anti-windup no limite real); o V/f não é mais usado;
 *          - o integrador começa com a tensão da rampa (troca sem degrau).
 *          Sem amostras do encoder, continua em malha aberta seguindo o alvo.
 *
 * Uso:
 * @code
 * drv8313_init(&drv, &drv_cfg);
 * gptim_init(&comm_tim, &tim_cfg);   // .callback = task_drv8313_commutate_from_isr, parado
 * task_drv8313_create(&drv, &comm_tim, &motor_cfg, log_write);
 * task_as5600_set_sample_callback(task_drv8313_encoder_sample);
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
#include "pid.h"
#include "sixstep.h"
#include "drivers/gptim/gptim.h"

/** @brief Período da task fora da malha fechada (alinhamento e rampa), em ms. */
#ifndef TASK_DRV8313_PERIOD_MS
#define TASK_DRV8313_PERIOD_MS 1U
#endif

/** @brief Tempo máximo sem amostra do encoder na malha fechada, em ms (excedido = FAULT). */
#ifndef TASK_DRV8313_ENCODER_TIMEOUT_MS
#define TASK_DRV8313_ENCODER_TIMEOUT_MS 20U
#endif

/** @brief Intervalo do log de estado na malha fechada, em ms (0 desativa). */
#ifndef TASK_DRV8313_STATUS_MS
#define TASK_DRV8313_STATUS_MS 1000U
#endif

/** @brief Tamanho da stack da task, em words (o vsnprintf dos logs precisa de folga). */
#ifndef TASK_DRV8313_STACK_SIZE
#define TASK_DRV8313_STACK_SIZE (configMINIMAL_STACK_SIZE * 4)
#endif

/** @brief Prioridade da task (acima da task do encoder, para rodar logo após a amostra). */
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
    TASK_DRV8313_STATE_ALIGN,    /**< Alinhando o rotor. */
    TASK_DRV8313_STATE_RAMP,     /**< Partida em malha aberta. */
    TASK_DRV8313_STATE_RUN,      /**< Em operação (malha fechada ou aberta). */
    TASK_DRV8313_STATE_FAULT,    /**< Falha; saídas desligadas. */
} task_drv8313_state_t;

/** @brief Parâmetros do motor, da partida e do controle de velocidade. */
typedef struct
{
    uint8_t pole_pairs;      /**< Pares de polos do motor (polos / 2). */
    sixstep_dir_t direction; /**< Sentido de giro. */

    /* Alinhamento */
    float align_u;     /**< Tensão normalizada do alinhamento (0.0 a 1.0). */
    uint32_t align_ms; /**< Duração de cada um dos dois passos do alinhamento, em ms. */

    /* Partida em rampa (malha aberta) */
    float start_rpm;      /**< Velocidade no início da rampa (> 0). */
    float ramp_end_rpm;   /**< Fim da rampa: passagem para a malha fechada (>= start_rpm). */
    float ramp_rpm_per_s; /**< Aceleração da rampa e do setpoint, em rpm/s (> 0). */
    float u_offset;       /**< V/f: tensão com rpm = 0 (vence a resistência do enrolamento). */
    float u_per_rpm;      /**< V/f: ganho de tensão por rpm. */
    float u_max;          /**< Limite da tensão normalizada (0.0 a 1.0). */

    /* Velocidade */
    float target_rpm; /**< Velocidade alvo inicial (start_rpm a max_rpm). */
    float max_rpm;    /**< Limite para task_drv8313_set_target_rpm(). */

    /* Malha fechada */
    bool closed_loop;      /**< true: usa o encoder e o PID depois da rampa. */
    float kp;              /**< Ganho proporcional, em u por pu de velocidade (base = target_rpm). */
    float ki;              /**< Ganho integral, em u por (pu · s). */
    float kd;              /**< Ganho derivativo, em u por (pu / s). */
    float speed_filter_ms; /**< Constante de tempo do filtro passa-baixa da velocidade medida (0 = sem filtro). */
    float switch_lag_deg;  /**< Atraso do rotor ao vetor da rampa na troca, em graus elétricos (0 a 90). */
    uint32_t comm_rate_hz; /**< Taxa da ISR de comutação na malha fechada (ex.: 10000). */
} task_drv8313_motor_cfg_t;

/**
 * @brief Função de log usada pela task.
 *
 * Recebe uma linha já formatada, terminada em "\r\n". É chamada no contexto da
 * task; se outras tasks também escrevem no mesmo destino, proteja-a com um mutex.
 */
typedef void (*task_drv8313_log_t)(const char *msg);

/** @brief Handle da task (recebe comandos e amostras do encoder por task notify). */
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
 * @brief Pede a partida (alinhamento + rampa + malha fechada). Só tem efeito em IDLE.
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
 * @brief Muda a velocidade alvo; o setpoint segue a rampa até o novo valor.
 * @param[in] rpm Nova velocidade, saturada entre @c start_rpm e @c max_rpm.
 * @return Velocidade alvo efetivamente aplicada.
 */
float task_drv8313_set_target_rpm(float rpm);

/**
 * @brief Muda os ganhos do PID (vale na próxima amostra; útil para sintonia).
 * @param[in] kp Ganho proporcional, em u por pu de velocidade (base = target_rpm da configuração).
 * @param[in] ki Ganho integral, em u por (pu · s).
 * @param[in] kd Ganho derivativo, em u por (pu / s).
 */
void task_drv8313_set_pid_gains(float kp, float ki, float kd);

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

/** @brief true se o motor está em RUN com a malha fechada ativa. */
bool task_drv8313_is_closed_loop(void);

/** @brief Setpoint atual (com rampa) em rpm; na malha aberta, a velocidade comandada. */
float task_drv8313_get_rpm(void);

/** @brief Velocidade medida pelo encoder, filtrada, em rpm (positiva no sentido comandado). */
float task_drv8313_get_speed_rpm(void);

/** @brief Velocidade alvo, em rpm. */
float task_drv8313_get_target_rpm(void);

/** @brief Tensão normalizada aplicada atualmente (0.0 a 1.0). */
float task_drv8313_get_u(void);

/** @brief Último setor aplicado pela ISR de comutação (0 a 5). */
uint8_t task_drv8313_get_sector(void);

/**
 * @brief Recebe uma amostra do encoder; registre com task_as5600_set_sample_callback().
 *
 * Roda no contexto da task do encoder: guarda a amostra, atualiza o ângulo usado
 * pela ISR de comutação e, na malha fechada, notifica a task do motor.
 *
 * @param raw            RAW ANGLE, de 0 a 4095.
 * @param velocity_rad_s Velocidade mecânica, em rad/s (positiva quando o RAW ANGLE cresce).
 * @param dt_s           Tempo desde a amostra anterior, em s.
 */
void task_drv8313_encoder_sample(uint16_t raw, float velocity_rad_s, float dt_s);

/**
 * @brief Escolhe o setor e aplica os duty cycles; registre como callback do timer de comutação.
 *
 * Tem a assinatura de gptim_callback_t. Não usa a API do FreeRTOS, então o timer
 * pode ter prioridade acima de configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
 * (menor jitter na comutação).
 *
 * @param context Não usado.
 */
void task_drv8313_commutate_from_isr(void *context);

#endif /* TASK_DRV8313_H */
