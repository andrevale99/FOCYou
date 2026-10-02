#ifndef INVERTER_H
#define INVERTER_H

#include <stddef.h>
#include <stdint.h>

#include <stm32f411xe.h>

#include "drivers/gpio/driver_gpio.h"

#define MAX_CHN_INVERTER 3

typedef enum
{
    INVERTER_OK = 0,
    INVERTER_ERROR_INVALID_ARGUMENT = -1,
    INVERTER_ERROR_NO_STATE = -2,
} inverter_error_t;

typedef enum
{
    INVERTER_STATE_OFF = 0,
    INVERTER_STATE_ON = 1,
} inverter_state_t;

typedef struct
{
    uint16_t pin;
    uint16_t alternate_function;
} inverter_gpio_config_t;

typedef struct
{
    TIM_TypeDef *const advanced_timer;

    uint8_t deadtime;
    uint16_t autorreload;
    uint16_t prescale;

    GPIO_TypeDef *const gpioH;
    GPIO_TypeDef *const gpioL;

    inverter_gpio_config_t config_gpioH[MAX_CHN_INVERTER];
    inverter_gpio_config_t config_gpioL[MAX_CHN_INVERTER];

} inverter_config_t;

typedef struct
{
    TIM_TypeDef *const advanced_timer;

    uint16_t duty_cycle_a;
    uint16_t duty_cycle_b;
    uint16_t duty_cycle_c;

} inverter_duty_cycle_t;

inverter_error_t inverter_init(inverter_config_t *config);
inverter_error_t inverter_set_state(inverter_duty_cycle_t *inverter,inverter_state_t state);
inverter_error_t inverter_set_duty_cycle(inverter_duty_cycle_t *duty_cycle);

inverter_state_t inverter_get_state(inverter_duty_cycle_t *inverter);

#endif