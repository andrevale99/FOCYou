#include <stdio.h>

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"

#include "inverter.h"

#include "lcd16x2.h"
#include "driver_lcd16x2.h"

/* LED da Black Pill: PC13 (ativo em nivel baixo) */
#define LED_PORT GPIOC
#define LED_PIN 13U

#define TIM_CLK_HZ 16000000UL /* HSI, APB2 sem prescaler */
#define PWM_FREQ_HZ 10000UL

/* Conversão ADC -> corrente (ajuste ao seu sensor; padrão: ACS712-20A, 100 mV/A, 3,3 V/12 bits) */
#define CURRENT_OFFSET_COUNTS 2048
#define CURRENT_MA_PER_COUNT 8

#define LCD_UPDATE_MS 1000U

/* ---------- dados compartilhados ISR -> task ---------- */

typedef struct
{
    uint16_t raw_a, raw_b, raw_c; /* contagens do ADC */
    float duty_a;                 /* 0.0 .. 1.0 */
} motor_data_t;

static inverter_t inv;
static volatile bool fault_flag;
static volatile motor_data_t motor;

/* Cópia atômica (as ISRs de ADC/break estão acima do limite do FreeRTOS) */
static motor_data_t motor_snapshot(void)
{
    motor_data_t d;
    const uint32_t primask = __get_PRIMASK();

    __disable_irq();
    d.raw_a = motor.raw_a;
    d.raw_b = motor.raw_b;
    d.raw_c = motor.raw_c;
    d.duty_a = motor.duty_a;
    __set_PRIMASK(primask);

    return d;
}

static void led_init(void)
{
    driver_gpio_enable_clock(LED_PORT);
    driver_gpio_set_mode(LED_PORT, LED_PIN, DRIVER_GPIO_OUTPUT);
    driver_gpio_set_pull(LED_PORT, LED_PIN, DRIVER_GPIO_PULL_NONE);
    driver_gpio_set_speed(LED_PORT, LED_PIN, DRIVER_GPIO_SPEED_LOW);
}

/* ---------- callbacks / ISRs (não usam API do FreeRTOS) ---------- */

static void on_break(void *ctx)
{
    (void)ctx;
    fault_flag = true; /* o hardware já desligou as saídas (MOE = 0) */
}

void TIM1_BRK_TIM9_IRQHandler(void)
{
    inverter_irq_break_handler(&inv);
}

/* Disparada pelo CC4 do TIM1: malha de controle a 10 kHz */
void ADC_IRQHandler(void)
{
    if (ADC1->SR & ADC_SR_JEOC)
    {
        ADC1->SR = ~ADC_SR_JEOC;

        motor.raw_a = (uint16_t)ADC1->JDR1;
        motor.raw_b = (uint16_t)ADC1->JDR2;
        motor.raw_c = (uint16_t)ADC1->JDR3;

        /* ... controle / modulação aqui ... */
        inverter_duty_norm_t duty = {0.5f, 0.5f, 0.5f};
        inverter_set_duty_cycle_normalized(&inv, &duty);
        motor.duty_a = duty.a;
    }
}

/* ---------- ADC1: 3 canais injetados (PA7, PB0, PB1), trigger TIM1_CC4 ---------- */

static void adc_init(void)
{
    driver_gpio_enable_clock(GPIOA);
    driver_gpio_enable_clock(GPIOB);
    GPIOA->MODER |= (3U << (7 * 2));                   /* PA7 analógico (IN7) */
    GPIOB->MODER |= (3U << (0 * 2)) | (3U << (1 * 2)); /* PB0, PB1 analógicos (IN8, IN9) */

    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    ADC1_COMMON->CCR &= ~ADC_CCR_ADCPRE; /* PCLK2/2 = 8 MHz */

    ADC1->SMPR2 = (3U << (7 * 3)) | (3U << (8 * 3)) | (3U << (9 * 3)); /* 56 ciclos */

    /* JL = 2 (3 conversões) -> JSQ2, JSQ3, JSQ4 resultam em JDR1..JDR3 */
    ADC1->JSQR = (2U << ADC_JSQR_JL_Pos) |
                 (7U << ADC_JSQR_JSQ2_Pos) |
                 (8U << ADC_JSQR_JSQ3_Pos) |
                 (9U << ADC_JSQR_JSQ4_Pos);

    ADC1->CR1 = ADC_CR1_SCAN | ADC_CR1_JEOCIE;
    ADC1->CR2 = ADC_CR2_ADON |
                (0U << ADC_CR2_JEXTSEL_Pos) | /* 0000 = TIM1_CC4 */
                (1U << ADC_CR2_JEXTEN_Pos);   /* borda de subida */

    NVIC_SetPriority(ADC_IRQn, 1);
    NVIC_EnableIRQ(ADC_IRQn);
}

/* ---------- inversor ---------- */

static bool inverter_setup(void)
{
    /* BKIN em PA6 (AF1), ativo em nível baixo */
    driver_gpio_enable_clock(GPIOA);
    driver_gpio_set_alternate_function(GPIOA, 6, 1);
    driver_gpio_set_pull(GPIOA, 6, DRIVER_GPIO_PULL_UP);

    inverter_config_t cfg = {
        .advanced_timer = INVERTER_TIMER,
        .timer_clock_hz = TIM_CLK_HZ,
        .deadtime = 0,
        .autorreload = 800, /* já corresponde a 10 kHz */
        .prescale = 0,
        .repetition_counter = 0,
        .alignment = INVERTER_ALIGN_CENTER_1, /* 1 evento CC4 por período */
        .polarity_high = INVERTER_POLARITY_ACTIVE_HIGH,
        .polarity_low = INVERTER_POLARITY_ACTIVE_HIGH,

        .gpioH = GPIOA, /* CH1..CH3   = PA8, PA9, PA10 (AF1)   */
        .gpioL = GPIOB, /* CH1N..CH3N = PB13, PB14, PB15 (AF1) */
        .config_gpioH = {{8, 1}, {9, 1}, {10, 1}},
        .config_gpioL = {{13, 1}, {14, 1}, {15, 1}},

        .idle = {
            .high_idle_level = {false, false, false},
            .low_idle_level = {false, false, false},
            .off_state_run = false,
            .off_state_idle = true,
        },
        .brk = {
            .enable = true,
            .active_high = false,
            .automatic_output = false,
        },
    };

    if (inverter_init(&inv, &cfg) != INVERTER_OK)
        return false;

    inverter_set_frequency(&inv, PWM_FREQ_HZ); /* confirma PSC = 0, ARR = 800 */
    inverter_set_deadtime_ns(&inv, 500);       /* 8 ticks de 62,5 ns */

    /* ADC dispara na descida, 8 ticks (0,5 us) após o pico da portadora */
    inverter_adc_trigger_config_t trig = {
        .enable = true,
        .compare_value = (uint16_t)(inv.autorreload - 8U),
        .use_trgo = false,
    };
    inverter_config_adc_trigger(&inv, &trig);

    inverter_callbacks_t cbs = {.on_update = NULL, .on_break = on_break, .context = NULL};
    inverter_set_callbacks(&inv, &cbs);

    NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 0); /* break acima do ADC */
    inverter_enable_break_irq(&inv, true);

    adc_init();

    return true;
}

/* ---------- LCD ---------- */

static void lcd_gpio_init(void)
{
    (void)driver_lcd16x2_init();
}

static void lcd_delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms)); /* só usar com o scheduler rodando */
}

static const lcd16x2_handle_t lcd = {
    .d4 = {write_d4},
    .d5 = {write_d5},
    .d6 = {write_d6},
    .d7 = {write_d7},
    .en = {write_en},
    .rs = {write_rs},
    .delay_ms = lcd_delay_ms,
};

/* Escreve "+1.2" (4 caracteres, sem '\0'); satura em +-99.9 */
static void fmt_tenths(char *dst, int32_t t)
{
    char sign = '+';

    if (t < 0)
    {
        sign = '-';
        t = -t;
    }
    if (t > 999)
        t = 999;

    dst[0] = sign;
    dst[1] = (char)('0' + (t / 100) % 10); /* limita a 1 dígito inteiro na tela */
    dst[2] = '.';
    dst[3] = (char)('0' + (t / 10) % 10);
}

static int32_t counts_to_tenths_a(uint16_t raw)
{
    return ((int32_t)raw - CURRENT_OFFSET_COUNTS) * CURRENT_MA_PER_COUNT / 100;
}

static void lcd_write_line(uint8_t cmd_addr, const char *line)
{
    lcd16x2_send_cmd(&lcd, cmd_addr);
    lcd16x2_write_string(&lcd, line, 16);
}

/*
 * Linha 1: "ON  D 50% 10.0k "   estado, duty da fase A, frequência de chaveamento
 * Linha 2: "I:+1.2 -0.3 +0.1"   correntes das fases A, B e C em A (saturadas em +-9.9)
 */
static void vLcdTask(void *pvParameters)
{
    (void)pvParameters;

    char line1[17];
    char line2[17];

    if (lcd16x2_init_4bits(&lcd, lcd_gpio_init) != LCD_OK)
        for (;;)
            vTaskDelay(portMAX_DELAY);

    TickType_t last_wake = xTaskGetTickCount();

    for (;;)
    {
        const motor_data_t d = motor_snapshot();

        const char *state = inverter_get_fault(&inv) ? "FLT"
                                                     : (inverter_get_state(&inv) == INVERTER_STATE_ON ? "ON " : "OFF");

        uint32_t freq_hz = 0;
        (void)inverter_get_frequency(&inv, &freq_hz);

        /* Valores limitados: o compilador prova que o texto cabe em 16 caracteres */
        const float duty_f = d.duty_a * 100.0f + 0.5f;
        const unsigned duty_pct = (duty_f <= 0.0f)     ? 0U
                                  : (duty_f >= 100.0f) ? 100U
                                                       : (unsigned)duty_f;

        const unsigned freq_khz = (unsigned)((freq_hz / 1000U) % 100U); /* 0..99 */
        const unsigned freq_dec = (unsigned)((freq_hz % 1000U) / 100U); /* 0..9  */

        snprintf(line1, sizeof(line1), "%-3.3s D%3u%% %2u.%uk ",
                 state, duty_pct, freq_khz, freq_dec);

        line2[0] = 'I';
        line2[1] = ':';
        fmt_tenths(&line2[2], counts_to_tenths_a(d.raw_a));
        line2[6] = ' ';
        fmt_tenths(&line2[7], counts_to_tenths_a(d.raw_b));
        line2[11] = ' ';
        fmt_tenths(&line2[12], counts_to_tenths_a(d.raw_c));
        line2[16] = '\0';

        lcd_write_line(SET_DDRAM | 0x00, line1);
        lcd_write_line(SECOND_LINE, line2);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LCD_UPDATE_MS));
    }
}

/* ---------- demais tasks ---------- */

static void vInverterTask(void *pvParameters)
{
    (void)pvParameters;

    inverter_start(&inv);
    inverter_set_state(&inv, INVERTER_STATE_ON);

    for (;;)
    {
        if (fault_flag)
        {
            /* só limpa quando a causa do break sumiu */
            if (inverter_clear_fault(&inv) == INVERTER_OK)
            {
                fault_flag = false;
                inverter_enable_break_irq(&inv, true);
                inverter_set_state(&inv, INVERTER_STATE_ON);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void vBlinkTask(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
        driver_gpio_toggle_pin(LED_PORT, LED_PIN);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

int main(void)
{
    SystemCoreClockUpdate(); /* HSI 16 MHz */
    led_init();

    if (!inverter_setup())
        for (;;)
        {
        } /* falha na configuração do inversor */

    xTaskCreate(vBlinkTask, "blink", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(vLcdTask, "lcd", configMINIMAL_STACK_SIZE * 4, NULL, 1, NULL);
    xTaskCreate(vInverterTask, "inv", configMINIMAL_STACK_SIZE * 2, NULL, 2, NULL);

    vTaskStartScheduler(); /* nao retorna */

    for (;;)
    {
    } /* so chega aqui se faltar heap */
}