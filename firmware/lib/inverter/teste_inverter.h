#ifndef INVERTER_H
#define INVERTER_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include <stm32f411xe.h>

#include "drivers/gpio/driver_gpio.h"

/* No STM32F411 o único timer advanced é o TIM1 */
#define INVERTER_TIMER TIM1
#define MAX_CHN_INVERTER 3

/* ------------------------------------------------------------------ */
/* Tipos                                                               */
/* ------------------------------------------------------------------ */

typedef enum
{
    INVERTER_OK = 0,
    INVERTER_ERROR_INVALID_ARGUMENT = -1,
    INVERTER_ERROR_NO_STATE = -2,
    INVERTER_ERROR_OUT_OF_RANGE = -3,
    INVERTER_ERROR_LOCKED = -4,
    INVERTER_ERROR_FAULT = -5,
    INVERTER_ERROR_NOT_INITIALIZED = -6,
} inverter_error_t;

typedef enum
{
    INVERTER_STATE_OFF = 0,
    INVERTER_STATE_ON = 1,
} inverter_state_t;

typedef enum
{
    INVERTER_ALIGN_EDGE = 0,
    INVERTER_ALIGN_CENTER_1 = 1,
    INVERTER_ALIGN_CENTER_2 = 2,
    INVERTER_ALIGN_CENTER_3 = 3,
} inverter_align_t;

typedef enum
{
    INVERTER_OUT_PWM1 = 0x6,
    INVERTER_OUT_PWM2 = 0x7,
    INVERTER_OUT_FORCE_INACTIVE = 0x4,
    INVERTER_OUT_FORCE_ACTIVE = 0x5,
} inverter_output_mode_t;

typedef enum
{
    INVERTER_POLARITY_ACTIVE_HIGH = 0,
    INVERTER_POLARITY_ACTIVE_LOW = 1,
} inverter_polarity_t;

typedef enum
{
    INVERTER_LOCK_OFF = 0,
    INVERTER_LOCK_LEVEL1 = 1, /* dead-time, break, polaridade e idle state */
    INVERTER_LOCK_LEVEL2 = 2,
    INVERTER_LOCK_LEVEL3 = 3,
} inverter_lock_level_t;

typedef enum
{
    INVERTER_CH_A = 0,
    INVERTER_CH_B = 1,
    INVERTER_CH_C = 2,
} inverter_channel_t;

typedef struct
{
    uint16_t pin;
    uint16_t alternate_function;
} inverter_gpio_config_t;

/* Nível dos pinos quando MOE = 0 (idle state) */
typedef struct
{
    bool high_idle_level[MAX_CHN_INVERTER]; /* OISx  */
    bool low_idle_level[MAX_CHN_INVERTER];  /* OISxN */
    bool off_state_run;                     /* OSSR  */
    bool off_state_idle;                    /* OSSI  */
} inverter_idle_config_t;

/* Entrada de break (BKIN) */
typedef struct
{
    bool enable;               /* BKE */
    bool active_high;          /* BKP */
    uint8_t filter;            /* BKF (0..15) */
    bool automatic_output;     /* AOE: religa MOE no próximo update */
} inverter_break_config_t;

/* Trigger do ADC (CH4 como evento de comparação) */
typedef struct
{
    bool enable;
    uint16_t compare_value; /* CCR4: posição do disparo no período (<= ARR) */
    bool use_trgo;          /* true: TRGO=OC4REF, false: só CH4 */
} inverter_adc_trigger_config_t;

/* Callbacks (chamados de dentro das ISRs) */
typedef void (*inverter_callback_t)(void *context);

typedef struct
{
    inverter_callback_t on_update; /* a cada update (período / RCR) */
    inverter_callback_t on_break;  /* evento de break / falha */
    void *context;
} inverter_callbacks_t;

/* Configuração de inicialização */
typedef struct
{
    TIM_TypeDef *advanced_timer;

    uint32_t timer_clock_hz; /* clock real do TIM1 (APB2 timer clock) */

    uint8_t deadtime;        /* valor bruto do DTG (codificação não linear) */
    uint16_t autorreload;
    uint16_t prescale;
    uint8_t repetition_counter; /* RCR */

    inverter_align_t alignment;
    inverter_polarity_t polarity_high;
    inverter_polarity_t polarity_low;

    GPIO_TypeDef *gpioH;
    GPIO_TypeDef *gpioL;

    inverter_gpio_config_t config_gpioH[MAX_CHN_INVERTER];
    inverter_gpio_config_t config_gpioL[MAX_CHN_INVERTER];

    inverter_idle_config_t idle;
    inverter_break_config_t brk;

} inverter_config_t;

/* Handle único usado por todas as funções (substitui inverter_duty_cycle_t como contexto) */
typedef struct
{
    TIM_TypeDef *timer;
    uint32_t timer_clock_hz;
    uint16_t autorreload;
    uint16_t prescale;
    uint8_t deadtime;
    inverter_align_t alignment;
    bool initialized;
    inverter_callbacks_t callbacks;
} inverter_t;

/* Duty cycles em contagens do timer (0..ARR) */
typedef struct
{
    uint16_t a;
    uint16_t b;
    uint16_t c;
} inverter_duty_t;

/* Duty cycles normalizados (0.0 .. 1.0) */
typedef struct
{
    float a;
    float b;
    float c;
} inverter_duty_norm_t;

/* ------------------------------------------------------------------ */
/* Ciclo de vida                                                       */
/* ------------------------------------------------------------------ */

/* Configura timer e GPIOs. Saídas permanecem DESLIGADAS (MOE = 0) e o timer parado. */
inverter_error_t inverter_init(inverter_t *inv, const inverter_config_t *config);
inverter_error_t inverter_deinit(inverter_t *inv);

inverter_error_t inverter_start(inverter_t *inv); /* CEN = 1 */
inverter_error_t inverter_stop(inverter_t *inv);  /* CEN = 0 e MOE = 0 */

/* ------------------------------------------------------------------ */
/* Estado das saídas (MOE)                                             */
/* ------------------------------------------------------------------ */

inverter_error_t inverter_set_state(inverter_t *inv, inverter_state_t state);
inverter_state_t inverter_get_state(const inverter_t *inv); /* NULL -> OFF */

/* ------------------------------------------------------------------ */
/* Frequência / dead-time / duty                                       */
/* ------------------------------------------------------------------ */

/* f = fclk / (2 * (PSC+1) * ARR) em center-aligned; fclk / ((PSC+1)*(ARR+1)) em edge */
inverter_error_t inverter_set_frequency(inverter_t *inv, uint32_t frequency_hz);
inverter_error_t inverter_get_frequency(const inverter_t *inv, uint32_t *frequency_hz);

inverter_error_t inverter_set_deadtime(inverter_t *inv, uint8_t dtg);
inverter_error_t inverter_set_deadtime_ns(inverter_t *inv, uint32_t deadtime_ns);

inverter_error_t inverter_set_duty_cycle(inverter_t *inv, const inverter_duty_t *duty);
inverter_error_t inverter_set_duty_cycle_normalized(inverter_t *inv, const inverter_duty_norm_t *duty);
inverter_error_t inverter_get_duty_cycle(const inverter_t *inv, inverter_duty_t *duty);

/* ------------------------------------------------------------------ */
/* Proteção                                                            */
/* ------------------------------------------------------------------ */

inverter_error_t inverter_config_break(inverter_t *inv, const inverter_break_config_t *cfg);
bool inverter_get_fault(const inverter_t *inv);            /* flag BIF */
inverter_error_t inverter_clear_fault(inverter_t *inv);    /* limpa BIF; MOE segue o AOE */
inverter_error_t inverter_set_idle_state(inverter_t *inv, const inverter_idle_config_t *cfg);
inverter_error_t inverter_lock_config(inverter_t *inv, inverter_lock_level_t level);

/* ------------------------------------------------------------------ */
/* Sincronismo com ADC / malha de controle                             */
/* ------------------------------------------------------------------ */

inverter_error_t inverter_config_adc_trigger(inverter_t *inv, const inverter_adc_trigger_config_t *cfg);
inverter_error_t inverter_set_repetition_counter(inverter_t *inv, uint8_t rcr);

inverter_error_t inverter_set_callbacks(inverter_t *inv, const inverter_callbacks_t *callbacks);
inverter_error_t inverter_enable_update_irq(inverter_t *inv, bool enable);
inverter_error_t inverter_enable_break_irq(inverter_t *inv, bool enable);

/* Chamar de dentro das ISRs (TIM1_UP_TIM10 e TIM1_BRK_TIM9) */
void inverter_irq_update_handler(inverter_t *inv);
void inverter_irq_break_handler(inverter_t *inv);

/* ------------------------------------------------------------------ */
/* Flexibilidade                                                       */
/* ------------------------------------------------------------------ */

inverter_error_t inverter_set_channel_enable(inverter_t *inv, inverter_channel_t ch, bool high, bool low);
inverter_error_t inverter_set_polarity(inverter_t *inv, inverter_polarity_t high, inverter_polarity_t low);
inverter_error_t inverter_set_alignment(inverter_t *inv, inverter_align_t alignment);
inverter_error_t inverter_set_output_mode(inverter_t *inv, inverter_channel_t ch, inverter_output_mode_t mode);
inverter_error_t inverter_force_output(inverter_t *inv, inverter_channel_t ch, bool active);

#endif