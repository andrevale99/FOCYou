#include <stdio.h>

#include "env.h"

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"

#include "drivers/adc/adc.h"
#include "drivers/i2c/i2c.h"
#include "drivers/rcc/rcc.h"
#include "drivers/gptim/gptim.h"
#include "drivers/uart/uart.h"

#include "drv8313.h"

#include "task_as5600.h"
#include "task_drv8313.h"
#include "task_logger.h"

static i2c_t i2c;
static adc_t adc;
static uint8_t as5600_adc_channel;
static gptim_t as5600_tim;
static drv8313_t drv;
static gptim_t comm_tim;

/*
 * Logs de texto de todas as tasks: passam pelo logger, que põe "# " em cada
 * linha (o pandas ignora com comment="#") e os intercala com o CSV.
 */
static void log_write(const char *msg)
{
    task_logger_write(msg);
}

/* ---------- CSV de 1 ms ---------- */

#define LOG_HEADER "pwm_a,pwm_b,pwm_c,setor,pos,pos_unwrap,vel_rpm_x10"
#define LOG_COLUMNS 7U

/* Uma linha do CSV (o t_us é posto pelo logger) */
static void log_sample(int32_t *v)
{
    uint16_t raw;
    int32_t pos;
    task_as5600_get_sample(&raw, &pos);

    const float vel = task_drv8313_get_speed_rpm() * 10.0f;

    v[0] = (int32_t)ENV_DRV_TIM->CCR1; /* duty das fases, de 0 ao ARR do PWM */
    v[1] = (int32_t)ENV_DRV_TIM->CCR2;
    v[2] = (int32_t)ENV_DRV_TIM->CCR3;
    v[3] = task_drv8313_get_sector();
    v[4] = raw;                        /* 0 a 4095 */
    v[5] = pos;                        /* voltas · 4096 + ângulo */
    v[6] = (int32_t)(vel + ((vel >= 0.0f) ? 0.5f : -0.5f));
}

static BaseType_t logger_setup(void)
{
    const task_logger_config_t cfg = {
        .usart = ENV_UART,
        .header = LOG_HEADER,
        .n_columns = LOG_COLUMNS,
        .sample = log_sample,
        .period_ms = ENV_LOG_PERIOD_MS,
    };

    return task_logger_create(&cfg);
}

/* Falha de inicialização: para aqui (dá para ver com o debugger) */
static void init_error(void)
{
    for (;;)
    {
    }
}

/* SYSCLK de 50 MHz pelo PLL; os periféricos usam os clocks do env.h */
static driver_err_t clock_setup(void)
{
    rcc_config_t cfg = {
        .clock_source = RCC_PLL,
        .pll_conf = {
            .source = RCC_HSE,
            .m_factor = ENV_PLL_M,
            .n_factor = ENV_PLL_N,
            .p_factor = ENV_PLL_P,
        },
        .ahb_divider = RCC_AHB_DIV_0,
        .apb1_divider = RCC_APBx_DIV_0,
        .apb2_divider = RCC_APBx_DIV_0,
    };

    const driver_err_t err = rcc_init(&cfg);
    if (err != DRIVER_OK)
        return err;

    /* Confere com o env.h: os periféricos foram calculados para ENV_SYSCLK_HZ */
    return (SystemCoreClock == ENV_SYSCLK_HZ) ? DRIVER_OK : DRIVER_ERR_INVALID_CLOCK_SOURCE;
}

static driver_err_t uart_setup(void)
{
    const uart_config_t cfg = {
        .usart = ENV_UART,
        .clock_hz = ENV_PCLK2_HZ, /* USART1 fica no APB2 */
        .baudrate = ENV_UART_BAUDRATE,
        .gpio = ENV_UART_PORT,
        .tx_pin = ENV_UART_TX_PIN,
        .rx_pin = ENV_UART_RX_PIN,
        .alternate_function = ENV_UART_AF,
    };

    return uart_init(&cfg);
}

static driver_err_t i2c_setup(void)
{
    const i2c_config_t cfg = {
        .i2c = ENV_I2C,
        .pclk1_hz = ENV_PCLK1_HZ,
        .speed = ENV_I2C_SPEED,
        .scl = {ENV_I2C_SCL_PORT, ENV_I2C_SCL_PIN, ENV_I2C_AF},
        .sda = {ENV_I2C_SDA_PORT, ENV_I2C_SDA_PIN, ENV_I2C_AF},
        .timeout = 0, /* I2C_DEFAULT_TIMEOUT */
    };

    return i2c_init(&i2c, &cfg);
}

#if ENV_AS5600_USE_ADC
/* ADC1 com o pino OUT do AS5600 em modo analógico */
static driver_err_t adc_setup(void)
{
    const adc_config_t cfg = {
        .adc = ADC1,
        .pclk2_hz = ENV_PCLK2_HZ,
        .sample_time = ENV_AS5600_ADC_SAMPLE,
        .timeout = 0, /* ADC_DEFAULT_TIMEOUT */
    };

    driver_err_t err = adc_init(&adc, &cfg);
    if (err != DRIVER_OK)
        return err;

    return adc_config_pin(&adc, ENV_AS5600_ADC_PORT, ENV_AS5600_ADC_PIN, &as5600_adc_channel);
}
#endif

static BaseType_t as5600_task_setup(void)
{
    const task_as5600_config_t cfg = {
#if ENV_AS5600_USE_ADC
        .source = TASK_AS5600_SOURCE_ADC,
#else
        .source = TASK_AS5600_SOURCE_I2C,
#endif
        .adc = &adc,
        .adc_channel = as5600_adc_channel,
        .oversample = ENV_AS5600_ADC_OVERSAMPLE,
        .analog_range = ENV_AS5600_ANALOG_RANGE,
        .adc_min = ENV_AS5600_ADC_MIN,
        .adc_max = ENV_AS5600_ADC_MAX,
        .max_rpm = ENV_MOTOR_MAX_RPM,
        .period_us = ENV_AS5600_TIM_PERIOD_US,
        .velocity_window_us = ENV_AS5600_VEL_WINDOW_US,
#if ENV_AS5600_USE_ADC && !ENV_AS5600_USE_I2C
        .i2c = NULL,
#else
        .i2c = &i2c, /* obrigatório na fonte I2C */
#endif
    };

    return task_as5600_create(&cfg, log_write);
}

static driver_err_t as5600_tim_setup(void)
{
    const gptim_config_t cfg = {
        .timer = ENV_AS5600_TIM,
        .timer_clock_hz = ENV_AS5600_TIM_CLOCK_HZ,
        .period_us = ENV_AS5600_TIM_PERIOD_US,
        .irq_priority = ENV_AS5600_TIM_IRQ_PRIORITY,
        .callback = task_as5600_notify_from_isr, /* notifica a task do encoder */
        .context = NULL,
    };

    return gptim_init(&as5600_tim, &cfg);
}

/* Handler do timer do AS5600 (o nome vem do env.h) */
void ENV_AS5600_TIM_IRQHandler(void)
{
    gptim_irq_handler(&as5600_tim);
}

static drv8313_err_t drv_setup(void)
{
    const drv8313_config_t cfg = {
        .timer = ENV_DRV_TIM,
        .timer_clock_hz = ENV_DRV_TIM_CLOCK_HZ,
        .pwm_frequency_hz = ENV_DRV_PWM_HZ,
        .center_aligned = true,
        .in = {
            {ENV_DRV_IN1_PORT, ENV_DRV_IN1_PIN, ENV_DRV_IN_AF},
            {ENV_DRV_IN2_PORT, ENV_DRV_IN2_PIN, ENV_DRV_IN_AF},
            {ENV_DRV_IN3_PORT, ENV_DRV_IN3_PIN, ENV_DRV_IN_AF},
        },
        .en = {ENV_DRV_EN_PORT, ENV_DRV_EN_PIN},
        .n_fault = {ENV_DRV_NFAULT_PORT, ENV_DRV_NFAULT_PIN},
        .n_reset = {NULL, 0},
        .n_sleep = {NULL, 0},
        .delay_ms = NULL, /* só é obrigatório com nRESET ou nSLEEP */
    };

    return drv8313_init(&drv, &cfg);
}

static driver_err_t comm_tim_setup(void)
{
    const gptim_config_t cfg = {
        .timer = ENV_COMM_TIM,
        .timer_clock_hz = ENV_COMM_TIM_CLOCK_HZ,
        .period_us = 10000U, /* provisório: a task do motor define o período */
        .irq_priority = ENV_COMM_TIM_IRQ_PRIORITY,
        .callback = task_drv8313_commutate_from_isr,
        .context = NULL,
    };

    return gptim_init(&comm_tim, &cfg);
}

/* Handler do timer de comutação (o nome vem do env.h) */
void ENV_COMM_TIM_IRQHandler(void)
{
    gptim_irq_handler(&comm_tim);
}

static BaseType_t motor_task_setup(void)
{
    const task_drv8313_motor_cfg_t motor = {
        .pole_pairs = ENV_MOTOR_POLE_PAIRS,
        .direction = SIXSTEP_DIR_FORWARD,
        .align_u = ENV_MOTOR_ALIGN_U,
        .align_ms = ENV_MOTOR_ALIGN_MS,
        .start_rpm = ENV_MOTOR_START_RPM,
        .ramp_end_rpm = ENV_MOTOR_RAMP_END_RPM,
        .ramp_rpm_per_s = ENV_MOTOR_RAMP_RPM_PER_S,
        .u_offset = ENV_MOTOR_U_OFFSET,
        .u_per_rpm = ENV_MOTOR_U_PER_RPM,
        .u_max = ENV_MOTOR_U_MAX,
        .target_rpm = ENV_MOTOR_TARGET_RPM,
        .max_rpm = ENV_MOTOR_MAX_RPM,
        .closed_loop = ENV_MOTOR_CLOSED_LOOP,
        .kp = ENV_MOTOR_KP,
        .ki = ENV_MOTOR_KI,
        .kd = ENV_MOTOR_KD,
        .speed_filter_ms = ENV_MOTOR_SPEED_FILTER_MS,
        .comm_rate_hz = ENV_MOTOR_COMM_RATE_HZ,
        .switch_lag_deg = ENV_MOTOR_SWITCH_LAG_DEG,
    };

    return task_drv8313_create(&drv, &comm_tim, &motor, log_write);
}

int main(void)
{
    SystemCoreClockUpdate(); /* HSI 16 MHz após o reset */

    /* Antes de tudo: os periféricos são configurados para o clock final */
    const driver_err_t clk_err = clock_setup();

    if (uart_setup() != DRIVER_OK)
        init_error();

    /* Logo depois da UART: todos os logs passam por ele (antes do scheduler, direto) */
    const BaseType_t logger_ok = logger_setup();

    log_write("[main] FOCYou: UART ok\r\n");

    if (logger_ok != pdPASS)
    {
        log_write("[main] falha ao criar o logger (DMA ou heap)\r\n");
        init_error();
    }

    if (clk_err != DRIVER_OK)
    {
        /* Com o clock errado o baudrate também sai errado: este log pode sair ilegível */
        log_write("[main] falha ao configurar o clock de 50 MHz\r\n");
        init_error();
    }

    log_write("[main] SYSCLK = 50 MHz (PLL, HSE 25 MHz)\r\n");

    if (i2c_setup() != DRIVER_OK)
    {
        log_write("[main] falha em i2c_init\r\n");
        init_error();
    }

    log_write("[main] I2C ok\r\n");

#if ENV_AS5600_USE_ADC
    if (adc_setup() != DRIVER_OK)
    {
        log_write("[main] falha ao configurar o ADC do AS5600\r\n");
        init_error();
    }

    log_write("[main] ADC ok\r\n");
#endif

    if (as5600_task_setup() != pdPASS)
    {
        log_write("[main] falha ao criar a task do AS5600\r\n");
        init_error();
    }

    /* Depois da task: o callback do timer precisa do task_as5600_handle */
    if (as5600_tim_setup() != DRIVER_OK || gptim_start(&as5600_tim) != DRIVER_OK)
    {
        log_write("[main] falha ao configurar o timer do AS5600\r\n");
        init_error();
    }

    log_write("[main] timer do AS5600 ok\r\n");

    /* Motor: driver e timer de comutação antes da task (ela recebe os dois prontos) */
    if (drv_setup() != DRV8313_OK)
    {
        log_write("[main] falha em drv8313_init\r\n");
        init_error();
    }

    if (comm_tim_setup() != DRIVER_OK)
    {
        log_write("[main] falha ao configurar o timer de comutacao\r\n");
        init_error();
    }

    if (motor_task_setup() != pdPASS)
    {
        log_write("[main] falha ao criar a task do motor\r\n");
        init_error();
    }

    /* Cada amostra do encoder acorda a task do motor (PID logo após a leitura) */
    task_as5600_set_sample_callback(task_drv8313_encoder_sample);

    log_write("[main] DRV8313 ok\r\n");

#if ENV_MOTOR_AUTOSTART
    task_drv8313_start(); /* o comando fica pendente até a task rodar */
#endif

    log_write("[main] iniciando o scheduler\r\n");

    vTaskStartScheduler(); /* não retorna */

    /* Só chega aqui se faltar heap para a task idle */
    init_error();
    return 0;
}
