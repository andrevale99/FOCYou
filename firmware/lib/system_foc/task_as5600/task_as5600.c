#include <stdarg.h>
#include <stdio.h>

#include "task_as5600.h"

TaskHandle_t task_as5600_handle = NULL;

static as5600_t enc;
static i2c_t *bus;
static task_as5600_log_t log_fn;
static volatile task_as5600_sample_cb_t sample_cb;
static uint32_t period_us;

static volatile uint16_t raw_angle;
static volatile float velocity_rad_s;
static volatile bool ready;

/* Configuração do encoder: sempre ligado e com resposta rápida para o controle */
static const as5600_conf_t enc_conf = {
    .power_mode = AS5600_PM_NOM,
    .hysteresis = AS5600_HYST_OFF,
    .output_stage = AS5600_OUTS_ANALOG_FULL,
    .pwm_freq = AS5600_PWMF_115HZ,
    .slow_filter = AS5600_SF_2X,
    .fast_filter = AS5600_FTH_6LSB,
    .watchdog = false,
};

/* ------------------------------------------------------------------ */
/* Helpers internos                                                    */
/* ------------------------------------------------------------------ */

/** Formata a mensagem com o prefixo "[as5600] " e "\r\n" e envia para a função de log. */
static void log_msg(const char *fmt, ...)
{
    if (log_fn == NULL)
        return;

    char buf[TASK_AS5600_LOG_SIZE];
    int n = snprintf(buf, sizeof(buf), "[as5600] ");

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

static const char *magnet_str(as5600_err_t err)
{
    switch (err)
    {
    case AS5600_ERR_NO_MAGNET:
        return "ima nao detectado";
    case AS5600_ERR_MAGNET_TOO_WEAK:
        return "ima fraco (aproxime)";
    case AS5600_ERR_MAGNET_TOO_STRONG:
        return "ima forte (afaste)";
    default:
        return "falha ao ler STATUS";
    }
}

/** Procura o encoder no endereço primário e depois no secundário. */
static as5600_err_t encoder_detect(void)
{
    static const uint8_t addresses[] = {AS5600_I2C_ADDR, AS5600L_I2C_ADDR};

    as5600_err_t err = AS5600_ERR_NOT_FOUND;

    for (uint8_t i = 0; i < sizeof(addresses) / sizeof(addresses[0]); i++)
    {
        const as5600_config_t cfg = {
            .i2c = bus,
            .address = addresses[i],
            .delay_ms = NULL, /* sem comandos de OTP nesta task */
        };

        err = as5600_init(&enc, &cfg);
        if (err == AS5600_OK)
        {
            log_msg("encontrado em 0x%02X", addresses[i]);
            return AS5600_OK;
        }

        if (err == AS5600_ERR_NOT_FOUND)
            log_msg("sem resposta em 0x%02X", addresses[i]);
        else
            log_msg("erro %d no I2C em 0x%02X", as5600_get_i2c_error(&enc), addresses[i]);
    }

    return err;
}

/** Aplica a configuração e confere o ímã; falhas não bloqueiam a leitura. */
static void encoder_configure(void)
{
    if (as5600_set_conf(&enc, &enc_conf) == AS5600_OK)
        log_msg("CONF gravado (PM nominal, SF 2x, FTH 6 LSB)");
    else
        log_msg("falha ao escrever CONF (I2C %d)", as5600_get_i2c_error(&enc));

    const as5600_err_t err = as5600_check_magnet(&enc);
    if (err == AS5600_OK)
    {
        uint8_t agc = 0;
        if (as5600_read_agc(&enc, &agc) == AS5600_OK)
            log_msg("ima ok, AGC = %u", agc);
        else
            log_msg("ima ok");
    }
    else
    {
        log_msg("%s (%d)", magnet_str(err), err);
    }
}

/* ------------------------------------------------------------------ */
/* Task                                                                */
/* ------------------------------------------------------------------ */

static void vAs5600Task(void *pvParameters)
{
    (void)pvParameters;

    log_msg("task iniciada");

    vTaskDelay(pdMS_TO_TICKS(TASK_AS5600_POWERUP_MS));

    /* 1. Detecção: sem encoder, a task fica presa avisando a cada 1 s */
    if (encoder_detect() != AS5600_OK)
    {
        for (;;)
        {
            log_msg("nao encontrado em 0x%02X nem 0x%02X", AS5600_I2C_ADDR, AS5600L_I2C_ADDR);
            vTaskDelay(pdMS_TO_TICKS(TASK_AS5600_ERROR_MS));
        }
    }

    /* 2. Configuração */
    encoder_configure();

    /* 3. Leitura a cada notificação do timer */
    const float period_s = (float)period_us * 1e-6f;
    uint32_t errors_per_log = (TASK_AS5600_ERROR_MS * 1000U) / period_us; /* leituras em ~1 s */
    if (errors_per_log == 0U)
        errors_per_log = 1U;

    uint32_t consecutive_errors = 0;
    bool timer_lost = false;
    float dt_acc = 0.0f; /* tempo desde a última leitura bem-sucedida */

    /* Descarta as notificações acumuladas durante a detecção e a configuração */
    (void)ulTaskNotifyTake(pdTRUE, 0);

    for (;;)
    {
        /* pdTRUE: notificações acumuladas (task atrasada) valem por uma leitura só */
        const uint32_t ticks = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(TASK_AS5600_NOTIFY_TIMEOUT_MS));
        if (ticks == 0U)
        {
            log_msg("sem notificacao do timer ha %u ms", (unsigned)TASK_AS5600_NOTIFY_TIMEOUT_MS);
            timer_lost = true;
            continue;
        }

        if (timer_lost)
        {
            log_msg("notificacoes do timer restabelecidas");
            timer_lost = false;
        }

        dt_acc += (float)ticks * period_s;

        /* Lê o RAW ANGLE e atualiza voltas e velocidade */
        const as5600_err_t err = as5600_update(&enc, dt_acc);

        if (err == AS5600_OK)
        {
            const uint16_t raw = as5600_get_raw(&enc);

            if (!ready)
                log_msg("primeira leitura: raw = %u", raw);
            else if (consecutive_errors > 0U)
                log_msg("leitura restabelecida apos %lu erro(s)", (unsigned long)consecutive_errors);

            consecutive_errors = 0;
            raw_angle = raw;
            velocity_rad_s = as5600_get_velocity_rad_s(&enc);
            ready = true;

            /* Avisa a malha de controle logo após a leitura */
            const task_as5600_sample_cb_t cb = sample_cb;
            if (cb != NULL)
                cb(raw, velocity_rad_s, dt_acc);

            dt_acc = 0.0f;
            continue;
        }

        const driver_err_t i2c_err = as5600_get_i2c_error(&enc);

        /* Loga o primeiro erro e depois um resumo a cada TASK_AS5600_ERROR_MS, para não inundar a UART */
        if (consecutive_errors % errors_per_log == 0U)
            log_msg("erro de leitura %d (I2C %d), %lu seguido(s)", err, i2c_err,
                    (unsigned long)(consecutive_errors + 1U));
        consecutive_errors++;

        if (i2c_err == DRIVER_ERR_I2C_BUSY || i2c_err == DRIVER_ERR_I2C_TIMEOUT)
        {
            if (i2c_bus_recover(bus) != DRIVER_OK && consecutive_errors == 1U)
                log_msg("bus recover falhou: SDA preso em nivel baixo");
        }
    }
}

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */

BaseType_t task_as5600_create(i2c_t *i2c, uint32_t period, task_as5600_log_t log)
{
    if (i2c == NULL || period == 0U)
        return pdFAIL;

    bus = i2c;
    period_us = period;
    log_fn = log;
    velocity_rad_s = 0.0f;
    ready = false;
    raw_angle = 0;

    return xTaskCreate(vAs5600Task, "as5600", TASK_AS5600_STACK_SIZE, NULL,
                       TASK_AS5600_PRIORITY, &task_as5600_handle);
}

void task_as5600_notify_from_isr(void *context)
{
    (void)context;

    if (task_as5600_handle == NULL)
        return;

    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(task_as5600_handle, &woken);
    portYIELD_FROM_ISR(woken);
}

void task_as5600_set_sample_callback(task_as5600_sample_cb_t cb)
{
    sample_cb = cb;
}

float task_as5600_get_velocity_rpm(void)
{
    return velocity_rad_s * (60.0f / 6.28318530718f);
}

uint16_t task_as5600_get_raw(void)
{
    return raw_angle;
}

bool task_as5600_is_ready(void)
{
    return ready;
}
