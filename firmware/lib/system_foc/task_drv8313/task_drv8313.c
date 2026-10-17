#include <stdarg.h>
#include <stdio.h>

#include "task_drv8313.h"

/* Bits recebidos por task notify: comandos e eventos */
#define CMD_START (1UL << 0)
#define CMD_STOP (1UL << 1)
#define CMD_CLEAR_FAULT (1UL << 2)
#define EVT_SAMPLE (1UL << 3) /* amostra nova do encoder (malha fechada) */
#define NOTIFY_ALL (CMD_START | CMD_STOP | CMD_CLEAR_FAULT | EVT_SAMPLE)

/* Um período elétrico tem 6 setores; tempo de um setor em µs = 60e6 / (rpm · pp · 6) */
#define STEP_US_NUMERATOR 10000000.0f

#define TWO_PI 6.28318530718f
#define PI_OVER_2 1.57079632679f
#define PI_OVER_3 1.04719755120f
#define THREE_OVER_PI 0.95492965855f
#define RAD_S_TO_RPM 9.54929658551f /* 60 / (2π) */

#define ENC_COUNTS 4096 /* contagens por volta do AS5600 */

/** Modo da ISR de comutação. */
typedef enum
{
    COMM_OFF = 0, /* não faz nada */
    COMM_OPEN,    /* avança um setor por interrupção */
    COMM_CLOSED,  /* setor pelo ângulo estimado do encoder */
} comm_mode_t;

TaskHandle_t task_drv8313_handle = NULL;

static drv8313_t *drv;
static gptim_t *tim;
static task_drv8313_motor_cfg_t motor;
static task_drv8313_log_t log_fn;

static volatile task_drv8313_state_t state = TASK_DRV8313_STATE_IDLE;
static volatile float target_rpm;
static volatile float rpm; /* malha aberta: velocidade comandada; malha fechada: setpoint com rampa */

/* Controle de velocidade */
static pid speed_pid;
static volatile bool closed_active;
static bool run_closed; /* resultado da calibração desta partida */

/* Amostras do encoder (escritas no contexto da task do AS5600) */
static volatile uint16_t enc_raw;
static volatile float enc_vel_rad_s;
static volatile float enc_dt_s;
static volatile bool enc_valid;
static volatile bool want_samples; /* notifica a task a cada amostra */
static volatile float speed_meas;  /* rpm filtrado, positivo no sentido comandado */
static volatile bool speed_init;

/* Calibração do encoder (feita no alinhamento) */
static volatile int8_t enc_sign; /* +1/-1: sentido do RAW ANGLE na sequência direta; 0 = sem calibração */
static volatile uint16_t enc_raw_ref; /* RAW ANGLE com o rotor alinhado ao setor 1 */

/* Compartilhado com a ISR de comutação (escritas de 32 bits são atômicas no M4) */
static volatile comm_mode_t comm_mode;
static volatile uint8_t comm_sector;
static volatile float comm_u;
static volatile sixstep_dir_t comm_dir;
static volatile float isr_theta_e; /* ângulo elétrico na última amostra, rad */
static volatile float isr_omega_e; /* velocidade elétrica, rad/s */
static volatile uint32_t isr_ticks; /* interrupções desde a última amostra */
static float comm_dt_s;             /* período da ISR na malha fechada */
static float comm_lead;             /* +90° (direto) ou -90° (reverso) */

/* ------------------------------------------------------------------ */
/* Helpers internos                                                    */
/* ------------------------------------------------------------------ */

/** Formata a mensagem com o prefixo "[drv8313] " e "\r\n" e envia para a função de log. */
static void log_msg(const char *fmt, ...)
{
    if (log_fn == NULL)
        return;

    char buf[TASK_DRV8313_LOG_SIZE];
    int n = snprintf(buf, sizeof(buf), "[drv8313] ");

    va_list args;
    va_start(args, fmt);
    n += vsnprintf(&buf[n], sizeof(buf) - (size_t)n, fmt, args);
    va_end(args);

    /* Garante o "\r\n" no fim, mesmo se a mensagem foi truncada */
    if (n > (int)sizeof(buf) - 3)
        n = (int)sizeof(buf) - 3;
    buf[n++] = '\r';
    buf[n++] = '\n';
    buf[n] = '\0';

    log_fn(buf);
}

/* O printf da newlib-nano não formata float: os logs usam inteiros */
static long to_long(float x)
{
    return (long)(x + ((x >= 0.0f) ? 0.5f : -0.5f));
}

static float clampf(float x, float lo, float hi)
{
    if (x < lo)
        return lo;
    if (x > hi)
        return hi;
    return x;
}

/** Tensão V/f para uma velocidade (rampa e feedforward do PID). */
static float vf_u(float r)
{
    return clampf(motor.u_offset + motor.u_per_rpm * r, 0.0f, motor.u_max);
}

static float dir_sign(void)
{
    return (comm_dir == SIXSTEP_DIR_FORWARD) ? 1.0f : -1.0f;
}

static void set_state(task_drv8313_state_t s)
{
    if (s != state)
        log_msg("estado: %s -> %s", task_drv8313_state_str(state), task_drv8313_state_str(s));
    state = s;
}

/** Aplica um setor direto, fora da ISR (alinhamento). */
static drv8313_err_t apply_sector(uint8_t sector, float u)
{
    sixstep_output_t out;
    (void)sixstep_commutate(sector, u, &out);

    const drv8313_duty_t duty = {out.a, out.b, out.c};
    return drv8313_set_duty_cycle(drv, &duty);
}

/** Atualiza o período do timer de comutação para a velocidade @p r (malha aberta). */
static bool set_step_period(float r)
{
    const float step_us = STEP_US_NUMERATOR / (r * (float)motor.pole_pairs);
    return gptim_set_period_us(tim, (uint32_t)(step_us + 0.5f)) == DRIVER_OK;
}

/**
 * Ângulo elétrico do rotor a partir do RAW ANGLE, no sentido da sequência direta
 * de setores. Referência: rotor alinhado ao setor 1 = 60°.
 * A conta em inteiros evita perder resolução com várias voltas.
 */
static float elec_angle(uint16_t raw)
{
    const int32_t d = ((int32_t)raw - (int32_t)enc_raw_ref) * (int32_t)enc_sign;
    int32_t e = (d * (int32_t)motor.pole_pairs) % ENC_COUNTS;
    if (e < 0)
        e += ENC_COUNTS;

    return (float)e * (TWO_PI / (float)ENC_COUNTS) + PI_OVER_3;
}

/** Para a comutação e desliga as saídas (o motor para por inércia). */
static void outputs_off(void)
{
    want_samples = false;
    closed_active = false;
    comm_mode = COMM_OFF; /* uma IRQ pendente sai sem fazer nada */
    (void)gptim_stop(tim);

    const drv8313_duty_t zero = {0.0f, 0.0f, 0.0f};
    (void)drv8313_set_duty_cycle(drv, &zero);
    (void)drv8313_disable(drv); /* EN baixo: meias pontes em alta impedância */

    rpm = 0.0f;
    comm_u = 0.0f;
}

static void enter_fault(const char *reason, int code)
{
    outputs_off();
    log_msg("falha: %s (%d)", reason, code);
    set_state(TASK_DRV8313_STATE_FAULT);
}

static void enter_idle(void)
{
    outputs_off();
    set_state(TASK_DRV8313_STATE_IDLE);
}

/* ------------------------------------------------------------------ */
/* Alinhamento e calibração                                            */
/* ------------------------------------------------------------------ */

static uint8_t align_phase;
static uint32_t align_elapsed_ms;
static uint16_t align_raw0;
static bool align_enc0;

static void enter_align(void)
{
    comm_mode = COMM_OFF;
    (void)gptim_stop(tim);

    comm_dir = motor.direction;
    comm_sector = 0;
    comm_u = motor.align_u;

    enc_sign = 0; /* nova calibração a cada partida */
    speed_init = false;
    run_closed = false;

    align_phase = 0;
    align_elapsed_ms = 0;

    drv8313_err_t err = apply_sector(0, motor.align_u);
    if (err == DRV8313_OK)
        err = drv8313_enable(drv);

    if (err != DRV8313_OK)
    {
        enter_fault("drv8313_enable", err);
        return;
    }

    log_msg("alinhando: u = %ld/1000, 2 x %lu ms", to_long(motor.align_u * 1000.0f),
            (unsigned long)motor.align_ms);
    set_state(TASK_DRV8313_STATE_ALIGN);
}

/**
 * Compara as leituras do encoder nos setores 0 e 1 (60° elétricos):
 * o sinal dá o sentido de contagem e o módulo confere os pares de polos.
 */
static void calibrate_encoder(void)
{
    if (!motor.closed_loop)
        return;

    if (!align_enc0 || !enc_valid)
    {
        log_msg("sem amostras do encoder: segue em malha aberta");
        return;
    }

    const uint16_t raw1 = enc_raw;
    int32_t delta = (int32_t)raw1 - (int32_t)align_raw0;
    if (delta > ENC_COUNTS / 2)
        delta -= ENC_COUNTS;
    else if (delta < -ENC_COUNTS / 2)
        delta += ENC_COUNTS;

    const int32_t expected = ENC_COUNTS / (6 * (int32_t)motor.pole_pairs);
    const int32_t mag = (delta < 0) ? -delta : delta;

    if (mag < expected / 2 || mag > (expected * 3) / 2)
    {
        log_msg("calibracao suspeita: delta = %ld, esperado %ld; segue em malha aberta",
                (long)delta, (long)expected);
        return;
    }

    enc_raw_ref = raw1;
    enc_sign = (delta > 0) ? 1 : -1; /* depois do enc_raw_ref: libera o cálculo do ângulo */
    run_closed = true;

    log_msg("calibracao: delta = %ld (esperado %ld), sentido %d", (long)delta, (long)expected,
            enc_sign);
}

/** Avança o alinhamento; retorna true quando termina. */
static bool align_step(void)
{
    align_elapsed_ms += TASK_DRV8313_PERIOD_MS;
    if (align_elapsed_ms < motor.align_ms)
        return false;

    align_elapsed_ms = 0;

    if (align_phase == 0U)
    {
        /* Rotor parado no setor 0: primeira leitura; depois avança 60° */
        align_enc0 = enc_valid;
        align_raw0 = enc_raw;
        align_phase = 1;

        const drv8313_err_t err = apply_sector(1, motor.align_u);
        if (err != DRV8313_OK)
            enter_fault("alinhamento no setor 1", err);
        return false;
    }

    calibrate_encoder();
    return true;
}

/* ------------------------------------------------------------------ */
/* Rampa (malha aberta)                                                */
/* ------------------------------------------------------------------ */

/** Velocidade final da rampa: passagem para a malha fechada ou o próprio alvo. */
static float ramp_goal(void)
{
    return run_closed ? motor.ramp_end_rpm : target_rpm;
}

static void enter_ramp(void)
{
    rpm = motor.start_rpm;
    comm_u = vf_u(rpm);
    comm_sector = 1; /* o rotor está no setor 1; a ISR começa pelo próximo */

    if (!set_step_period(rpm))
    {
        enter_fault("periodo de comutacao fora da faixa", 0);
        return;
    }

    comm_mode = COMM_OPEN;
    (void)gptim_start(tim);

    log_msg("rampa: %ld -> %ld rpm a %ld rpm/s", to_long(motor.start_rpm), to_long(ramp_goal()),
            to_long(motor.ramp_rpm_per_s));
    set_state(TASK_DRV8313_STATE_RAMP);
}

/** Move a velocidade comandada em direção a @p goal; retorna true quando chega. */
static bool ramp_step(float goal)
{
    const float step = motor.ramp_rpm_per_s * ((float)TASK_DRV8313_PERIOD_MS / 1000.0f);
    float r = rpm;

    if (r < goal)
        r = (r + step > goal) ? goal : r + step;
    else if (r > goal)
        r = (r - step < goal) ? goal : r - step;

    if (r != rpm)
    {
        if (!set_step_period(r))
        {
            enter_fault("periodo de comutacao fora da faixa", 0);
            return false;
        }
        rpm = r;
        comm_u = vf_u(r);
    }

    return r == goal;
}

/* ------------------------------------------------------------------ */
/* Malha fechada                                                       */
/* ------------------------------------------------------------------ */

static float status_elapsed_s;

static void enter_closed_loop(void)
{
    speed_pid.kp = motor.kp;
    speed_pid.ki = motor.ki;
    speed_pid.kd = motor.kd;
    speed_pid.saturation = motor.u_max;
    (void)pid_clear(&speed_pid);

    /* Transferência sem degrau: o integrador começa com o que falta para a tensão atual */
    speed_pid.I = clampf(comm_u - vf_u(rpm), -motor.u_max, motor.u_max);

    comm_dt_s = 1.0f / (float)motor.comm_rate_hz;
    comm_lead = (comm_dir == SIXSTEP_DIR_FORWARD) ? PI_OVER_2 : -PI_OVER_2;

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    isr_ticks = 0;
    comm_mode = COMM_CLOSED;
    __set_PRIMASK(primask);

    /* Vale a partir do próximo update (fim do setor atual da rampa) */
    if (gptim_set_period_us(tim, 1000000U / motor.comm_rate_hz) != DRIVER_OK)
    {
        enter_fault("taxa de comutacao fora da faixa", 0);
        return;
    }

    status_elapsed_s = 0.0f;
    closed_active = true;
    want_samples = true;

    log_msg("malha fechada: %ld rpm, u = %ld/1000", to_long(rpm), to_long(comm_u * 1000.0f));
    set_state(TASK_DRV8313_STATE_RUN);
}

/** Um passo do PID de velocidade; roda logo após cada amostra do encoder. */
static void control_step(void)
{
    const float dt = enc_dt_s;
    if (!(dt > 0.0f))
        return;

    /* Setpoint com rampa: evita degraus de corrente quando o alvo muda */
    const float step = motor.ramp_rpm_per_s * dt;
    const float tgt = target_rpm;
    float r = rpm;

    if (r < tgt)
        r = (r + step > tgt) ? tgt : r + step;
    else if (r > tgt)
        r = (r - step < tgt) ? tgt : r - step;
    rpm = r;

    const float meas = speed_meas;
    (void)pid_control(&speed_pid, r, meas, dt);

    /* Feedforward V/f + correção do PID */
    comm_u = clampf(vf_u(r) + speed_pid.output, 0.0f, motor.u_max);

#if TASK_DRV8313_STATUS_MS > 0
    status_elapsed_s += dt;
    if (status_elapsed_s >= (float)TASK_DRV8313_STATUS_MS / 1000.0f)
    {
        status_elapsed_s = 0.0f;
        log_msg("ref %ld rpm, medido %ld rpm, u = %ld/1000", to_long(r), to_long(meas),
                to_long(comm_u * 1000.0f));
    }
#endif
}

/* ------------------------------------------------------------------ */
/* Comandos                                                            */
/* ------------------------------------------------------------------ */

/** Retorna true se o nFAULT estiver ativo (pino não configurado conta como sem falha). */
static bool fault_active(void)
{
    bool fault = false;
    return (drv8313_get_fault(drv, &fault) == DRV8313_OK) && fault;
}

static void handle_commands(uint32_t cmd)
{
    if (cmd & CMD_STOP)
    {
        if (state != TASK_DRV8313_STATE_IDLE && state != TASK_DRV8313_STATE_FAULT)
        {
            log_msg("parada pedida");
            enter_idle();
        }
    }

    if (cmd & CMD_CLEAR_FAULT)
    {
        if (state == TASK_DRV8313_STATE_FAULT)
        {
            const drv8313_err_t err = drv8313_reset(drv);
            if (err != DRV8313_OK && err != DRV8313_ERR_NOT_CONFIGURED)
                log_msg("nRESET falhou (%d)", err);

            if (fault_active())
                log_msg("nFAULT continua ativo; falha mantida");
            else
                enter_idle();
        }
    }

    if (cmd & CMD_START)
    {
        if (state == TASK_DRV8313_STATE_IDLE)
            enter_align();
        else
            log_msg("start ignorado no estado %s", task_drv8313_state_str(state));
    }
}

static bool is_off(void)
{
    return state == TASK_DRV8313_STATE_IDLE || state == TASK_DRV8313_STATE_FAULT;
}

/* ------------------------------------------------------------------ */
/* Task                                                                */
/* ------------------------------------------------------------------ */

static void vDrv8313Task(void *pvParameters)
{
    (void)pvParameters;

    outputs_off();
    log_msg("task iniciada: %u pares de polos, alvo %ld rpm, malha %s", motor.pole_pairs,
            to_long(target_rpm), motor.closed_loop ? "fechada" : "aberta");

    TickType_t last_wake = xTaskGetTickCount();

    for (;;)
    {
        uint32_t bits = 0;

        /* Desligado: dorme até chegar um comando */
        if (is_off())
        {
            (void)xTaskNotifyWait(0, NOTIFY_ALL, &bits, portMAX_DELAY);
            handle_commands(bits);
            last_wake = xTaskGetTickCount();
            continue;
        }

        /*
         * Malha fechada: acorda a cada amostra do encoder (task notify), então o
         * PID roda logo após a leitura, sem esperar um ciclo fixo da task.
         */
        if (closed_active)
        {
            if (xTaskNotifyWait(0, NOTIFY_ALL, &bits,
                                pdMS_TO_TICKS(TASK_DRV8313_ENCODER_TIMEOUT_MS)) == pdFALSE)
            {
                enter_fault("sem amostras do encoder", 0);
                continue;
            }

            handle_commands(bits);
            if (is_off())
                continue;

            if (fault_active())
            {
                enter_fault("nFAULT ativo (sobrecorrente, sobretemperatura ou subtensao)", 0);
                continue;
            }

            if (bits & EVT_SAMPLE)
                control_step();
            continue;
        }

        /* Alinhamento, rampa e malha aberta: ciclo fixo */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TASK_DRV8313_PERIOD_MS));

        if (xTaskNotifyWait(0, NOTIFY_ALL, &bits, 0) == pdTRUE)
        {
            handle_commands(bits);
            if (is_off())
                continue;
        }

        if (fault_active())
        {
            enter_fault("nFAULT ativo (sobrecorrente, sobretemperatura ou subtensao)", 0);
            continue;
        }

        switch (state)
        {
        case TASK_DRV8313_STATE_ALIGN:
            if (align_step() && state == TASK_DRV8313_STATE_ALIGN)
                enter_ramp();
            break;

        case TASK_DRV8313_STATE_RAMP:
            if (ramp_step(ramp_goal()) && state == TASK_DRV8313_STATE_RAMP)
            {
                if (run_closed)
                {
                    enter_closed_loop();
                }
                else
                {
                    log_msg("alvo atingido em malha aberta: %ld rpm, u = %ld/1000", to_long(rpm),
                            to_long(comm_u * 1000.0f));
                    set_state(TASK_DRV8313_STATE_RUN);
                }
            }
            break;

        case TASK_DRV8313_STATE_RUN:
            /* Malha aberta (sem encoder ou closed_loop = false): segue o alvo com a rampa */
            (void)ramp_step(target_rpm);
            break;

        default:
            break;
        }
    }
}

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */

static bool motor_cfg_valid(const task_drv8313_motor_cfg_t *m)
{
    const bool base = m->pole_pairs > 0U &&
                      m->start_rpm > 0.0f &&
                      m->ramp_end_rpm >= m->start_rpm &&
                      m->max_rpm >= m->ramp_end_rpm &&
                      m->target_rpm >= m->start_rpm && m->target_rpm <= m->max_rpm &&
                      m->ramp_rpm_per_s > 0.0f &&
                      m->align_u >= 0.0f && m->align_u <= 1.0f &&
                      m->u_max > 0.0f && m->u_max <= 1.0f &&
                      m->u_offset >= 0.0f &&
                      m->u_per_rpm >= 0.0f;

    if (!base || !m->closed_loop)
        return base;

    return m->kp >= 0.0f && m->ki >= 0.0f && m->kd >= 0.0f &&
           m->speed_filter_ms >= 0.0f &&
           m->comm_rate_hz > 0U && m->comm_rate_hz <= 100000U;
}

BaseType_t task_drv8313_create(drv8313_t *d, gptim_t *t,
                               const task_drv8313_motor_cfg_t *m,
                               task_drv8313_log_t log)
{
    if (d == NULL || t == NULL || m == NULL || !motor_cfg_valid(m))
        return pdFAIL;

    drv = d;
    tim = t;
    motor = *m;
    log_fn = log;

    target_rpm = m->target_rpm;
    rpm = 0.0f;
    comm_mode = COMM_OFF;
    comm_dir = m->direction;
    enc_sign = 0;
    state = TASK_DRV8313_STATE_IDLE;

    return xTaskCreate(vDrv8313Task, "drv8313", TASK_DRV8313_STACK_SIZE, NULL,
                       TASK_DRV8313_PRIORITY, &task_drv8313_handle);
}

BaseType_t task_drv8313_start(void)
{
    if (task_drv8313_handle == NULL)
        return pdFAIL;
    return xTaskNotify(task_drv8313_handle, CMD_START, eSetBits);
}

BaseType_t task_drv8313_stop(void)
{
    if (task_drv8313_handle == NULL)
        return pdFAIL;
    return xTaskNotify(task_drv8313_handle, CMD_STOP, eSetBits);
}

BaseType_t task_drv8313_clear_fault(void)
{
    if (task_drv8313_handle == NULL)
        return pdFAIL;
    return xTaskNotify(task_drv8313_handle, CMD_CLEAR_FAULT, eSetBits);
}

float task_drv8313_set_target_rpm(float r)
{
    if (!(r == r)) /* NaN */
        return target_rpm;

    target_rpm = clampf(r, motor.start_rpm, motor.max_rpm);
    return target_rpm;
}

void task_drv8313_set_pid_gains(float kp, float ki, float kd)
{
    if (kp < 0.0f || ki < 0.0f || kd < 0.0f)
        return;

    /* A task do motor tem prioridade maior: o trecho crítico evita trocar os ganhos no meio de um passo */
    taskENTER_CRITICAL();
    motor.kp = kp;
    motor.ki = ki;
    motor.kd = kd;
    speed_pid.kp = kp;
    speed_pid.ki = ki;
    speed_pid.kd = kd;
    taskEXIT_CRITICAL();
}

bool task_drv8313_set_direction(sixstep_dir_t dir)
{
    if (state != TASK_DRV8313_STATE_IDLE)
        return false;

    motor.direction = dir;
    return true;
}

task_drv8313_state_t task_drv8313_get_state(void)
{
    return state;
}

const char *task_drv8313_state_str(task_drv8313_state_t s)
{
    switch (s)
    {
    case TASK_DRV8313_STATE_IDLE:
        return "IDLE";
    case TASK_DRV8313_STATE_ALIGN:
        return "ALIGN";
    case TASK_DRV8313_STATE_RAMP:
        return "RAMP";
    case TASK_DRV8313_STATE_RUN:
        return "RUN";
    case TASK_DRV8313_STATE_FAULT:
        return "FAULT";
    default:
        return "?";
    }
}

bool task_drv8313_is_closed_loop(void)
{
    return closed_active;
}

float task_drv8313_get_rpm(void)
{
    return rpm;
}

float task_drv8313_get_speed_rpm(void)
{
    return speed_meas;
}

float task_drv8313_get_target_rpm(void)
{
    return target_rpm;
}

float task_drv8313_get_u(void)
{
    return comm_u;
}

uint8_t task_drv8313_get_sector(void)
{
    return comm_sector;
}

void task_drv8313_encoder_sample(uint16_t raw, float velocity_rad_s, float dt_s)
{
    enc_raw = raw;
    enc_vel_rad_s = velocity_rad_s;
    enc_dt_s = dt_s;
    enc_valid = true;

    const int8_t sign = enc_sign;

    /* Velocidade no sentido comandado (antes da calibração, no sentido do RAW ANGLE) */
    const float rpm_inst = dir_sign() * (float)((sign != 0) ? sign : 1) * velocity_rad_s * RAD_S_TO_RPM;

    if (!speed_init)
    {
        speed_meas = rpm_inst;
        speed_init = true;
    }
    else
    {
        const float tau = motor.speed_filter_ms * 1e-3f;
        const float alpha = (tau > 0.0f) ? dt_s / (tau + dt_s) : 1.0f;
        speed_meas = speed_meas + alpha * (rpm_inst - speed_meas);
    }

    /* Ângulo e velocidade elétricos para a ISR de comutação */
    if (sign != 0)
    {
        const float theta = elec_angle(raw);
        const float omega = (float)sign * velocity_rad_s * (float)motor.pole_pairs;

        /* A ISR de comutação fica acima do limite do FreeRTOS: atualiza o trio sem interrupção */
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        isr_theta_e = theta;
        isr_omega_e = omega;
        isr_ticks = 0;
        __set_PRIMASK(primask);
    }

    if (want_samples && task_drv8313_handle != NULL)
        (void)xTaskNotify(task_drv8313_handle, EVT_SAMPLE, eSetBits);
}

void task_drv8313_commutate_from_isr(void *context)
{
    (void)context;

    const comm_mode_t mode = comm_mode;
    if (mode == COMM_OFF || drv == NULL)
        return;

    uint8_t sector;

    if (mode == COMM_OPEN)
    {
        sector = sixstep_next_sector(comm_sector, comm_dir);
    }
    else
    {
        /* Ângulo estimado entre amostras + avanço de 90° elétricos */
        const uint32_t n = ++isr_ticks;
        const float theta = isr_theta_e + isr_omega_e * ((float)n * comm_dt_s) + comm_lead;

        /* Setor s tem o vetor em s · 60°: arredonda para o mais próximo */
        const float k = theta * THREE_OVER_PI + 0.5f;
        int32_t ki = (int32_t)k;
        if (k < (float)ki)
            ki--; /* floor para negativos */

        ki %= (int32_t)SIXSTEP_NUM_SECTORS;
        if (ki < 0)
            ki += (int32_t)SIXSTEP_NUM_SECTORS;
        sector = (uint8_t)ki;
    }

    comm_sector = sector;

    sixstep_output_t out;
    (void)sixstep_commutate(sector, comm_u, &out);

    const drv8313_duty_t duty = {out.a, out.b, out.c};
    (void)drv8313_set_duty_cycle(drv, &duty);
}
