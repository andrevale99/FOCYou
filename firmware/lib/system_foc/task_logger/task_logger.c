#include "task_logger.h"

#include "message_buffer.h"
#include "semphr.h"

/* Maior linha do CSV: t_us (20 dígitos) + colunas (11 caracteres com sinal) + vírgulas + '\n' */
#define CSV_LINE_MAX (21U + TASK_LOGGER_MAX_COLUMNS * 12U + 1U)

/* Buffer do DMA: uma mensagem de texto + uma linha do CSV */
#define TX_BUF_SIZE (TASK_LOGGER_MSG_MAX + CSV_LINE_MAX)

TaskHandle_t task_logger_handle = NULL;

static task_logger_config_t cfg;
static MessageBufferHandle_t queue;
static SemaphoreHandle_t write_mutex;
static volatile uint32_t dropped_total;

/* Só é alterado com o DMA parado (uart_dma_busy() == false) */
static uint8_t tx_buf[TX_BUF_SIZE];

/* ------------------------------------------------------------------ */
/* Helpers internos                                                    */
/* ------------------------------------------------------------------ */

/** Escreve @p v em decimal em @p buf a partir de @p n; retorna o novo tamanho. */
static size_t append_u64(uint8_t *buf, size_t n, uint64_t v)
{
    char tmp[20];
    size_t k = 0;

    do
    {
        tmp[k++] = (char)('0' + (v % 10U));
        v /= 10U;
    } while (v != 0U);

    while (k > 0U)
        buf[n++] = (uint8_t)tmp[--k];

    return n;
}

static size_t append_i32(uint8_t *buf, size_t n, int32_t v)
{
    if (v < 0)
    {
        buf[n++] = '-';
        return append_u64(buf, n, (uint64_t)(-(int64_t)v));
    }
    return append_u64(buf, n, (uint64_t)v);
}

/** Liga o contador de ciclos do núcleo (DWT->CYCCNT). */
static void cycle_counter_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * Tempo em µs desde o início do contador. O CYCCNT dá a volta a cada ~86 s a
 * 50 MHz; como é lido pelo menos a cada period_ms, a extensão para 64 bits é segura.
 */
static uint64_t time_us(void)
{
    static uint32_t last;
    static uint64_t high;

    const uint32_t cyc = DWT->CYCCNT;
    if (cyc < last)
        high += 1ULL << 32;
    last = cyc;

    const uint32_t cycles_per_us = SystemCoreClock / 1000000U;
    return (high | cyc) / ((cycles_per_us != 0U) ? cycles_per_us : 1U);
}

/**
 * Copia @p msg para @p out pondo "# " no começo de cada linha e terminando cada
 * uma com "\r\n". Linhas vazias são ignoradas. Retorna o tamanho, sem o '\0'.
 */
static size_t prefix_lines(const char *msg, char *out, size_t cap)
{
    size_t n = 0;
    bool line_start = true;

    if (cap == 0U)
        return 0;
    cap--; /* reserva o '\0': o texto fica com no máximo cap - 1 bytes */

    for (const char *p = msg; *p != '\0'; p++)
    {
        const char c = *p;

        if (c == '\r' || c == '\n')
        {
            if (!line_start && n + 2U <= cap) /* o laço sempre deixa espaço para o "\r\n" */
            {
                out[n++] = '\r';
                out[n++] = '\n';
            }
            line_start = true;
            continue;
        }

        if (line_start)
        {
            if (n + 2U + 2U >= cap) /* "# " + "\r\n" */
                break;
            out[n++] = '#';
            out[n++] = ' ';
            line_start = false;
        }

        if (n + 2U >= cap) /* guarda espaço para o "\r\n" */
            break;
        out[n++] = c;
    }

    /* Última linha sem quebra (ou truncada): fecha com "\r\n" */
    if (!line_start)
    {
        out[n++] = '\r';
        out[n++] = '\n';
    }

    out[n] = '\0';
    return n;
}

/** Envia por DMA esperando o fim da transmissão anterior (só para o cabeçalho). */
static void send_blocking_dma(const uint8_t *data, uint16_t len)
{
    while (uart_dma_busy(cfg.usart))
        vTaskDelay(1);

    (void)uart_send_dma(cfg.usart, data, len);
}

/* ------------------------------------------------------------------ */
/* Task                                                                */
/* ------------------------------------------------------------------ */

static void vLoggerTask(void *pvParameters)
{
    (void)pvParameters;

    /* Cabeçalho do CSV */
    if (cfg.sample != NULL)
    {
        static const char t_col[] = "t_us,";
        size_t n = 0;

        for (const char *p = t_col; *p != '\0'; p++)
            tx_buf[n++] = (uint8_t)*p;
        for (const char *p = cfg.header; *p != '\0' && n < TX_BUF_SIZE - 1U; p++)
            tx_buf[n++] = (uint8_t)*p;
        tx_buf[n++] = '\n';

        send_blocking_dma(tx_buf, (uint16_t)n);
    }

    int32_t values[TASK_LOGGER_MAX_COLUMNS];
    uint32_t dropped_window = 0;
    uint32_t report_ms = 0;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(cfg.period_ms));

        /* Resumo das linhas descartadas */
        report_ms += cfg.period_ms;
        if (report_ms >= TASK_LOGGER_DROP_REPORT_MS)
        {
            report_ms = 0;
            if (dropped_window > 0U)
            {
                uint8_t msg[48];
                static const char txt1[] = "[logger] ";
                static const char txt2[] = " linha(s) descartada(s)";
                size_t k = 0;

                for (const char *p = txt1; *p != '\0'; p++)
                    msg[k++] = (uint8_t)*p;
                k = append_u64(msg, k, dropped_window);
                for (const char *p = txt2; *p != '\0'; p++)
                    msg[k++] = (uint8_t)*p;
                msg[k] = '\0';

                task_logger_write((const char *)msg);
                dropped_window = 0;
            }
        }

        /* DMA ainda enviando a linha anterior: descarta esta, sem esperar */
        if (uart_dma_busy(cfg.usart))
        {
            dropped_window++;
            dropped_total++;
            continue;
        }

        /* No máximo uma mensagem de texto por linha do CSV */
        size_t n = xMessageBufferReceive(queue, tx_buf, TASK_LOGGER_MSG_MAX, 0);

        if (cfg.sample != NULL)
        {
            const uint64_t t = time_us();
            cfg.sample(values);

            n = append_u64(tx_buf, n, t);
            for (uint8_t i = 0; i < cfg.n_columns; i++)
            {
                tx_buf[n++] = ',';
                n = append_i32(tx_buf, n, values[i]);
            }
            tx_buf[n++] = '\n';
        }

        if (n > 0U)
            (void)uart_send_dma(cfg.usart, tx_buf, (uint16_t)n);
    }
}

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */

BaseType_t task_logger_create(const task_logger_config_t *c)
{
    if (c == NULL || c->usart == NULL || c->period_ms == 0U)
        return pdFAIL;
    if (c->sample != NULL &&
        (c->header == NULL || c->n_columns == 0U || c->n_columns > TASK_LOGGER_MAX_COLUMNS))
        return pdFAIL;

    cfg = *c;

    if (uart_dma_init(cfg.usart) != DRIVER_OK)
        return pdFAIL;

    queue = xMessageBufferCreate(TASK_LOGGER_QUEUE_BYTES);
    write_mutex = xSemaphoreCreateMutex();
    if (queue == NULL || write_mutex == NULL)
        return pdFAIL;

    cycle_counter_init();

    return xTaskCreate(vLoggerTask, "logger", TASK_LOGGER_STACK_SIZE, NULL,
                       TASK_LOGGER_PRIORITY, &task_logger_handle);
}

void task_logger_write(const char *msg)
{
    if (msg == NULL || cfg.usart == NULL)
        return;

    char line[TASK_LOGGER_MSG_MAX + 1U];
    const size_t n = prefix_lines(msg, line, sizeof(line));
    if (n == 0U)
        return;

    /* Antes do scheduler: direto pela UART (o DMA ainda não está em uso) */
    if (queue == NULL || xTaskGetSchedulerState() != taskSCHEDULER_RUNNING)
    {
        (void)uart_send_string(cfg.usart, line);
        return;
    }

    /* Vários escritores: o message buffer só aceita um por vez */
    xSemaphoreTake(write_mutex, portMAX_DELAY);
    (void)xMessageBufferSend(queue, line, n, 0); /* fila cheia: a mensagem é descartada */
    xSemaphoreGive(write_mutex);
}

uint32_t task_logger_get_dropped(void)
{
    return dropped_total;
}
