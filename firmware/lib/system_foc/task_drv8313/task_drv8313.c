#include <stdarg.h>
#include <stdio.h>

#include "task_drv8313.h"

/* Comandos recebidos por task notify (bits) */
#define CMD_START (1UL << 0)
#define CMD_STOP (1UL << 1)
#define CMD_CLEAR_FAULT (1UL << 2)
#define CMD_ALL (CMD_START | CMD_STOP | CMD_CLEAR_FAULT)

/* Um período elétrico tem 6 setores; tempo de um setor em µs = 60e6 / (rpm · pp · 6) */
#define STEP_US_NUMERATOR 10000000.0f

TaskHandle_t task_drv8313_handle = NULL;

static drv8313_t *drv;
static gptim_t *tim;
static task_drv8313_motor_cfg_t motor;
static task_drv8313_log_t log_fn;

static volatile task_drv8313_state_t state = TASK_DRV8313_STATE_IDLE;
static volatile float target_rpm;
static volatile float rpm; /* velocidade comandada */

/* Compartilhado com a ISR de comutação (escritas de 32 bits são atômicas no M4) */
static volatile bool comm_running;
static volatile uint8_t comm_sector;
static volatile float comm_u;
static volatile sixstep_dir_t comm_dir;

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

/** Tensão da rampa V/f para uma velocidade. */
static float vf_u(float r)
{
    return clampf(motor.u_offset + motor.u_per_rpm * r, 0.0f, motor.u_max);
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

/** Atualiza o período do timer de comutação para a velocidade @p r. */
static bool set_step_period(float r)
{
    const float step_us = STEP_US_NUMERATOR / (r * (float)motor.pole_pairs);
    return gptim_set_period_us(tim, (uint32_t)(step_us + 0.5f)) == DRIVER_OK;
}

/** Para a comutação e desliga as saídas (o motor para por inércia). */
static void outputs_off(void)
{
    comm_running = false; /* uma IRQ pendente sai sem fazer nada */
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

static void enter_align(void)
{
    comm_running = false;
    (void)gptim_stop(tim);

    comm_dir = motor.direction;
    comm_sector = 0;
    comm_u = motor.align_u;

    drv8313_err_t err = apply_sector(0, motor.align_u);
    if (err == DRV8313_OK)
        err = drv8313_enable(drv);

    if (err != DRV8313_OK)
    {
        enter_fault("drv8313_enable", err);
        return;
    }

    log_msg("alinhando: u = %ld/1000 por %lu ms", to_long(motor.align_u * 1000.0f),
            (unsigned long)motor.align_ms);
    set_state(TASK_DRV8313_STATE_ALIGN);
}

static void enter_ramp(void)
{
    rpm = motor.start_rpm;
    comm_u = vf_u(rpm);

    if (!set_step_period(rpm))
    {
        enter_fault("periodo de comutacao fora da faixa", 0);
        return;
    }

    /* Primeiro setor da ISR = 1: o vetor avança 60° em relação ao alinhamento */
    comm_running = true;
    (void)gptim_start(tim);

    log_msg("rampa: %ld -> %ld rpm a %ld rpm/s", to_long(motor.start_rpm), to_long(target_rpm),
            to_long(motor.ramp_rpm_per_s));
    set_state(TASK_DRV8313_STATE_RAMP);
}

/** Move a velocidade em direção ao alvo e atualiza tensão e período. Retorna true no alvo. */
static bool ramp_step(void)
{
    const float step = motor.ramp_rpm_per_s * ((float)TASK_DRV8313_PERIOD_MS / 1000.0f);
    const float tgt = target_rpm;
    float r = rpm;

    if (r < tgt)
        r = (r + step > tgt) ? tgt : r + step;
    else if (r > tgt)
        r = (r - step < tgt) ? tgt : r - step;

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

    return r == tgt;
}

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

/* ------------------------------------------------------------------ */
/* Task                                                                */
/* ------------------------------------------------------------------ */

static void vDrv8313Task(void *pvParameters)
{
    (void)pvParameters;

    outputs_off();
    log_msg("task iniciada: %u pares de polos, alvo %ld rpm", motor.pole_pairs, to_long(target_rpm));

    uint32_t align_elapsed_ms = 0;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;)
    {
        uint32_t cmd = 0;

        if (state == TASK_DRV8313_STATE_IDLE || state == TASK_DRV8313_STATE_FAULT)
        {
            /* Desligado: dorme até chegar um comando */
            (void)xTaskNotifyWait(0, CMD_ALL, &cmd, portMAX_DELAY);
            handle_commands(cmd);

            align_elapsed_ms = 0;
            last_wake = xTaskGetTickCount();
            continue;
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TASK_DRV8313_PERIOD_MS));

        if (xTaskNotifyWait(0, CMD_ALL, &cmd, 0) == pdTRUE)
        {
            handle_commands(cmd);
            if (state == TASK_DRV8313_STATE_IDLE || state == TASK_DRV8313_STATE_FAULT)
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
            align_elapsed_ms += TASK_DRV8313_PERIOD_MS;
            if (align_elapsed_ms >= motor.align_ms)
                enter_ramp();
            break;

        case TASK_DRV8313_STATE_RAMP:
            if (ramp_step())
            {
                log_msg("alvo atingido: %ld rpm, u = %ld/1000", to_long(rpm), to_long(comm_u * 1000.0f));
                set_state(TASK_DRV8313_STATE_RUN);
            }
            break;

        case TASK_DRV8313_STATE_RUN:
            /*
             * Malha aberta: segue o alvo com a mesma rampa. Futuro: malha fechada
             * de velocidade com o AS5600 (pid_control -> comm_u) e comutação pelo
             * ângulo do encoder em vez do timer.
             */
            (void)ramp_step();
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
    return m->pole_pairs > 0U &&
           m->start_rpm > 0.0f &&
           m->target_rpm >= m->start_rpm &&
           m->max_rpm >= m->target_rpm &&
           m->ramp_rpm_per_s > 0.0f &&
           m->align_u >= 0.0f && m->align_u <= 1.0f &&
           m->u_max > 0.0f && m->u_max <= 1.0f &&
           m->u_offset >= 0.0f &&
           m->u_per_rpm >= 0.0f;
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
    comm_running = false;
    comm_dir = m->direction;
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

float task_drv8313_get_rpm(void)
{
    return rpm;
}

float task_drv8313_get_target_rpm(void)
{
    return target_rpm;
}

float task_drv8313_get_u(void)
{
    return comm_u;
}

void task_drv8313_commutate_from_isr(void *context)
{
    (void)context;

    if (!comm_running || drv == NULL)
        return;

    const uint8_t sector = sixstep_next_sector(comm_sector, comm_dir);
    comm_sector = sector;

    sixstep_output_t out;
    (void)sixstep_commutate(sector, comm_u, &out);

    const drv8313_duty_t duty = {out.a, out.b, out.c};
    (void)drv8313_set_duty_cycle(drv, &duty);
}
