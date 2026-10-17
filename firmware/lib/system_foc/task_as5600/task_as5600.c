#include <stdarg.h>
#include <stdio.h>

#include "task_as5600.h"

#define COUNTS ((int32_t)AS5600_COUNTS_PER_REV)
#define HALF_COUNTS (COUNTS / 2)

TaskHandle_t task_as5600_handle = NULL;

static task_as5600_config_t cfg;
static as5600_t enc; /* acompanhamento de voltas e velocidade; via I2C, também a configuração */
static task_as5600_log_t log_fn;
static volatile task_as5600_sample_cb_t sample_cb;

static volatile uint16_t raw_angle;
static volatile uint32_t position_counts; /* voltas · 4096 + ângulo */
static volatile uint16_t adc_last;
static volatile float velocity_rad_s;
static volatile bool ready;
static bool i2c_ok; /* sensor respondeu no I2C: habilita o diagnóstico */

/*
 * Janela da velocidade: posições (voltas · 4096 + ângulo) e tempos entre amostras aceitas.
 * As posições são uint32_t: o estouro dá a volta e a diferença continua certa.
 */
#define VEL_RING_SIZE (TASK_AS5600_VEL_MAX_SAMPLES + 1U)
static uint32_t vel_pos[VEL_RING_SIZE];
static float vel_dt[VEL_RING_SIZE];
static uint8_t vel_head;
static uint8_t vel_count;
static uint8_t vel_window; /* amostras na janela (0 = amostra a amostra) */

/* Configuração do encoder: sempre ligado, saída analógica de faixa cheia e resposta rápida */
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

/** Menor caminho entre dois ângulos, de -2048 a 2047 contagens. */
static int32_t wrap_delta(int32_t d)
{
    if (d >= HALF_COUNTS)
        d -= COUNTS;
    else if (d < -HALF_COUNTS)
        d += COUNTS;
    return d;
}

/** Converte a leitura do ADC em ângulo (0 a 4095) usando a faixa adc_min..adc_max. */
static int32_t adc_to_angle(uint16_t v)
{
    if (v <= cfg.adc_min)
        return 0;
    if (v >= cfg.adc_max)
        return COUNTS - 1;

    const int32_t span = (int32_t)cfg.adc_max - (int32_t)cfg.adc_min;
    return (((int32_t)v - (int32_t)cfg.adc_min) * (COUNTS - 1) + span / 2) / span;
}

/**
 * Faz @c oversample conversões e tira a média dos ângulos. A média é feita em
 * relação à primeira conversão, pelo menor caminho, para não errar perto de 0°/360°
 * (a média simples de 4095 e 0 daria 2048).
 */
static driver_err_t read_angle(uint16_t *angle)
{
    uint16_t v;
    driver_err_t err = adc_read(cfg.adc, cfg.adc_channel, &v);
    if (err != DRIVER_OK)
        return err;

    adc_last = v;
    const int32_t first = adc_to_angle(v);
    int32_t sum = 0;

    for (uint8_t i = 1; i < cfg.oversample; i++)
    {
        err = adc_read(cfg.adc, cfg.adc_channel, &v);
        if (err != DRIVER_OK)
            return err;

        sum += wrap_delta(adc_to_angle(v) - first);
    }

    int32_t a = first + sum / (int32_t)cfg.oversample;
    if (a < 0)
        a += COUNTS;
    else if (a >= COUNTS)
        a -= COUNTS;

    *angle = (uint16_t)a;
    return DRIVER_OK;
}

/**
 * Guarda a amostra e calcula a velocidade pela variação de posição na janela.
 * Com a janela em 0, usa a derivada amostra a amostra do as5600_track_raw().
 */
static float window_velocity(float dt)
{
    if (vel_window == 0U)
        return as5600_get_velocity_rad_s(&enc);

    vel_head = (uint8_t)((vel_head + 1U) % VEL_RING_SIZE);
    vel_pos[vel_head] = (uint32_t)as5600_get_turns(&enc) * (uint32_t)COUNTS + as5600_get_raw(&enc);
    vel_dt[vel_head] = dt;
    if (vel_count < VEL_RING_SIZE)
        vel_count++;

    /* Amostras disponíveis para trás (no começo, a janela ainda está enchendo) */
    const uint8_t m = (uint8_t)((vel_count - 1U < vel_window) ? vel_count - 1U : vel_window);
    if (m == 0U)
        return 0.0f;

    const uint8_t oldest = (uint8_t)((vel_head + VEL_RING_SIZE - m) % VEL_RING_SIZE);

    float t = 0.0f;
    for (uint8_t i = 0, k = vel_head; i < m; i++, k = (uint8_t)((k + VEL_RING_SIZE - 1U) % VEL_RING_SIZE))
        t += vel_dt[k];

    if (!(t > 0.0f))
        return 0.0f;

    const int32_t delta = (int32_t)(vel_pos[vel_head] - vel_pos[oldest]);
    return as5600_counts_to_rad(delta) / t;
}

/**
 * Lê o ângulo da fonte configurada. Retorna 0 em sucesso ou o código de erro
 * (driver_err_t do ADC ou do I2C) para o log.
 */
static int read_sample(uint16_t *angle)
{
    if (cfg.source == TASK_AS5600_SOURCE_ADC)
        return (int)read_angle(angle);

    if (as5600_read_raw_angle(&enc, angle) == AS5600_OK)
        return 0;

    const driver_err_t err = as5600_get_i2c_error(&enc);
    if (err == DRIVER_ERR_I2C_BUSY || err == DRIVER_ERR_I2C_TIMEOUT)
        (void)i2c_bus_recover(cfg.i2c);
    return (int)err;
}

/** Procura o encoder no I2C (primário e secundário), grava o CONF e confere o ímã. */
static void encoder_configure_i2c(void)
{
    static const uint8_t addresses[] = {AS5600_I2C_ADDR, AS5600L_I2C_ADDR};

    if (cfg.i2c == NULL)
    {
        log_msg("sem I2C: usa a configuracao de fabrica (saida analogica de faixa cheia)");
        return;
    }

    as5600_err_t err = AS5600_ERR_NOT_FOUND;

    for (uint8_t i = 0; i < sizeof(addresses) / sizeof(addresses[0]); i++)
    {
        const as5600_config_t ecfg = {
            .i2c = cfg.i2c,
            .address = addresses[i],
            .delay_ms = NULL, /* sem comandos de OTP nesta task */
        };

        err = as5600_init(&enc, &ecfg);
        if (err == AS5600_OK)
        {
            log_msg("encontrado no I2C em 0x%02X", addresses[i]);
            break;
        }
    }

    if (err != AS5600_OK)
    {
        if (cfg.source == TASK_AS5600_SOURCE_I2C)
        {
            /* Sem o sensor não há o que ler: a task fica presa avisando */
            for (;;)
            {
                log_msg("nao encontrado em 0x%02X nem 0x%02X", AS5600_I2C_ADDR, AS5600L_I2C_ADDR);
                vTaskDelay(pdMS_TO_TICKS(TASK_AS5600_ERROR_MS));
            }
        }

        log_msg("sem resposta no I2C em 0x%02X nem 0x%02X: usa a configuracao de fabrica",
                AS5600_I2C_ADDR, AS5600L_I2C_ADDR);
        return;
    }

    i2c_ok = true;

    if (as5600_set_conf(&enc, &enc_conf) == AS5600_OK)
        log_msg("CONF gravado (saida analogica 0-100%%, SF 2x, FTH 6 LSB)");
    else
        log_msg("falha ao escrever CONF (I2C %d)", as5600_get_i2c_error(&enc));

    /*
     * A saída OUT segue o ANGLE, que é limitado pela faixa ZPOS..MPOS (ou MANG).
     * Fora da faixa a tensão fica parada num extremo: zera os três na RAM para
     * a faixa cheia de 360° (não grava o OTP; vale até desligar o sensor).
     */
    /* Só a saída analógica depende da faixa; o RAW ANGLE do I2C não */
    if (cfg.source == TASK_AS5600_SOURCE_ADC)
    {
        uint8_t zmco = 0;
        uint16_t zpos = 0, mpos = 0, mang = 0;
        if (as5600_get_burn_count(&enc, &zmco) == AS5600_OK && as5600_get_zpos(&enc, &zpos) == AS5600_OK &&
            as5600_get_mpos(&enc, &mpos) == AS5600_OK && as5600_get_mang(&enc, &mang) == AS5600_OK)
            log_msg("faixa lida: ZMCO = %u, ZPOS = %u, MPOS = %u, MANG = %u", zmco, zpos, mpos, mang);
        else
            log_msg("falha ao ler ZPOS/MPOS/MANG (I2C %d)", as5600_get_i2c_error(&enc));

        if (as5600_set_zpos(&enc, 0) == AS5600_OK && as5600_set_mpos(&enc, 0) == AS5600_OK &&
            as5600_set_mang(&enc, 0) == AS5600_OK)
            log_msg("faixa ajustada para 360 graus (ZPOS = MPOS = MANG = 0)");
        else
            log_msg("falha ao zerar ZPOS/MPOS/MANG (I2C %d)", as5600_get_i2c_error(&enc));
    }

    err = as5600_check_magnet(&enc);
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

    if (cfg.source == TASK_AS5600_SOURCE_ADC)
        log_msg("task iniciada: ADC canal %u, faixa %u..%u, %u conversao(oes) por amostra",
                cfg.adc_channel, cfg.adc_min, cfg.adc_max, cfg.oversample);
    else
        log_msg("task iniciada: leitura do RAW ANGLE pelo I2C");
    log_msg("amostragem: %lu us, janela da velocidade: %u amostra(s)", (unsigned long)cfg.period_us,
            vel_window);

    vTaskDelay(pdMS_TO_TICKS(TASK_AS5600_POWERUP_MS));

    /* 1. Configuração pelo I2C (opcional) */
    encoder_configure_i2c();
    (void)as5600_reset_tracking(&enc);

    /* 2. Leitura a cada notificação do timer */
    const float period_s = (float)cfg.period_us * 1e-6f;
    uint32_t samples_per_log = (TASK_AS5600_ERROR_MS * 1000U) / cfg.period_us; /* amostras em ~1 s */
    if (samples_per_log == 0U)
        samples_per_log = 1U;

    /* Maior variação possível entre amostras, em contagens por segundo */
    const float max_counts_per_s = cfg.max_rpm / 60.0f * (float)COUNTS;

    uint32_t consecutive_errors = 0;
    uint32_t consecutive_rejects = 0;
    uint32_t rejected_total = 0;
    uint32_t log_counter = 0;
    bool timer_lost = false;
    float dt_acc = 0.0f; /* tempo desde a última leitura aceita */

    /* Descarta as notificações acumuladas durante a configuração */
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

#if TASK_AS5600_DIAG_MS > 0
        /* Diagnóstico: ângulo pelo I2C ao lado do ADC */
        static uint32_t diag_counter;
        if (cfg.source == TASK_AS5600_SOURCE_ADC && i2c_ok && ++diag_counter * cfg.period_us >= TASK_AS5600_DIAG_MS * 1000U)
        {
            diag_counter = 0;
            uint16_t i2c_raw = 0, i2c_angle = 0;
            const as5600_err_t e1 = as5600_read_raw_angle(&enc, &i2c_raw);
            const as5600_err_t e2 = as5600_read_angle(&enc, &i2c_angle);

            if (e1 == AS5600_OK && e2 == AS5600_OK)
                log_msg("diag: I2C raw %u, angle %u | ADC %u -> angulo %u", i2c_raw, i2c_angle, adc_last,
                        raw_angle);
            else
                log_msg("diag: falha no I2C (%d)", as5600_get_i2c_error(&enc));
        }
#endif

        /* Resumo das leituras descartadas a cada ~1 s */
        if (++log_counter >= samples_per_log)
        {
            log_counter = 0;
            if (rejected_total > 0U)
            {
                log_msg("%lu leitura(s) descartada(s) no ultimo segundo", (unsigned long)rejected_total);
                rejected_total = 0;
            }
        }

        uint16_t raw;
        const int err = read_sample(&raw);
        const char *src = (cfg.source == TASK_AS5600_SOURCE_ADC) ? "ADC" : "I2C";

        if (err != 0)
        {
            /* Loga o primeiro erro e depois um resumo a cada ~1 s */
            if (consecutive_errors % samples_per_log == 0U)
                log_msg("erro no %s %d, %lu seguido(s)", src, err, (unsigned long)(consecutive_errors + 1U));
            consecutive_errors++;
            continue;
        }

        if (consecutive_errors > 0U)
        {
            log_msg("%s restabelecido apos %lu erro(s)", src, (unsigned long)consecutive_errors);
            consecutive_errors = 0;
        }

        /* Descarta saltos impossíveis (conversão no meio da passagem por 0°/360°) */
        if (ready)
        {
            const int32_t delta = wrap_delta((int32_t)raw - (int32_t)raw_angle);
            const int32_t mag = (delta < 0) ? -delta : delta;
            const int32_t limit = (int32_t)(max_counts_per_s * dt_acc * 1.5f) + (int32_t)TASK_AS5600_NOISE_COUNTS;

            if (mag > limit && consecutive_rejects < TASK_AS5600_MAX_REJECT)
            {
                consecutive_rejects++;
                rejected_total++;
                continue; /* dt_acc continua acumulando até a próxima leitura aceita */
            }
        }
        consecutive_rejects = 0;

        /* Atualiza voltas e velocidade */
        (void)as5600_track_raw(&enc, raw, dt_acc);

        if (!ready)
        {
            if (cfg.source == TASK_AS5600_SOURCE_ADC)
                log_msg("primeira leitura: adc = %u, angulo = %u", adc_last, raw);
            else
                log_msg("primeira leitura: angulo = %u", raw);
        }

        taskENTER_CRITICAL(); /* ângulo e posição sempre do mesmo instante */
        raw_angle = raw;
        position_counts = (uint32_t)as5600_get_turns(&enc) * (uint32_t)COUNTS + raw;
        taskEXIT_CRITICAL();

        velocity_rad_s = window_velocity(dt_acc);
        ready = true;

        /* Avisa a malha de controle logo após a leitura */
        const task_as5600_sample_cb_t cb = sample_cb;
        if (cb != NULL)
            cb(raw, velocity_rad_s, dt_acc);

        dt_acc = 0.0f;
    }
}

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */

BaseType_t task_as5600_create(const task_as5600_config_t *c, task_as5600_log_t log)
{
    if (c == NULL || !(c->max_rpm > 0.0f) || c->period_us == 0U)
        return pdFAIL;

    if (c->source == TASK_AS5600_SOURCE_I2C)
    {
        if (c->i2c == NULL)
            return pdFAIL;
    }
    else if (c->source == TASK_AS5600_SOURCE_ADC)
    {
        if (c->adc == NULL || c->adc_channel > 18U ||
            c->adc_max <= c->adc_min || c->adc_max > ADC_MAX_VALUE ||
            c->oversample == 0U || c->oversample > 16U)
            return pdFAIL;
    }
    else
    {
        return pdFAIL;
    }

    cfg = *c;
    log_fn = log;

    /* Janela em amostras, arredondada e limitada ao tamanho do buffer */
    uint32_t w = (c->velocity_window_us + c->period_us / 2U) / c->period_us;
    if (w > TASK_AS5600_VEL_MAX_SAMPLES)
        w = TASK_AS5600_VEL_MAX_SAMPLES;
    vel_window = (uint8_t)w;
    vel_head = 0;
    vel_count = 0;

    velocity_rad_s = 0.0f;
    ready = false;
    raw_angle = 0;
    adc_last = 0;

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

void task_as5600_get_sample(uint16_t *raw, int32_t *position)
{
    taskENTER_CRITICAL();
    const uint16_t r = raw_angle;
    const uint32_t p = position_counts;
    taskEXIT_CRITICAL();

    if (raw != NULL)
        *raw = r;
    if (position != NULL)
        *position = (int32_t)p;
}

uint16_t task_as5600_get_adc(void)
{
    return adc_last;
}

bool task_as5600_is_ready(void)
{
    return ready;
}
