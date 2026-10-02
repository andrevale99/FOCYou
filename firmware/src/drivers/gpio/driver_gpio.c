#include "driver_gpio.h"

driver_gpio_err_t driver_gpio_enable_clock(GPIO_TypeDef *gpiox)
{
    if (gpiox == NULL)
        return DRIVER_ERR_NO_GPIO;

    if (gpiox == GPIOA)
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    else if (gpiox == GPIOB)
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    else if (gpiox == GPIOC)
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    else if (gpiox == GPIOD)
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

    else if (gpiox == GPIOE)
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

    else if (gpiox == GPIOH)
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;

    else
        return DRIVER_ERR_NO_CHANNEL_LOCATE;

    return DRIVER_GPIO_OK;
}

driver_gpio_err_t driver_gpio_set_mode(GPIO_TypeDef *gpiox, uint16_t pin,
                                       driver_gpio_moder_t mode)
{
    if (gpiox == NULL)
        return DRIVER_ERR_NO_GPIO;

    if (pin > 15 || mode > DRIVER_GPIO_ANALOG)
        return DRIVER_ERR_INVALID_PIN;

    if (mode < DRIVER_GPIO_INPUT || mode > DRIVER_GPIO_ANALOG)
        return DRIVER_ERR_INVALID_MODE;

    uint32_t shift = (uint32_t)pin << 1;

    gpiox->MODER &= ~(0x3UL << shift);
    gpiox->MODER |= ((uint32_t)mode << shift);

    return DRIVER_GPIO_OK;
}

driver_gpio_err_t driver_gpio_set_output_type(GPIO_TypeDef *gpiox, uint16_t pin,
                                              driver_gpio_otype_t type)
{
    if (gpiox == NULL)
        return DRIVER_ERR_NO_GPIO;

    if (pin > 15)
        return DRIVER_ERR_INVALID_PIN;

    if (type > DRIVER_GPIO_OTYPE_OPENDRAIN)
        return DRIVER_ERR_INVALID_MODE;

    gpiox->OTYPER &= ~(0x1UL << pin);
    gpiox->OTYPER |= ((uint32_t)type << pin);

    return DRIVER_GPIO_OK;
}

driver_gpio_err_t driver_gpio_set_speed(GPIO_TypeDef *gpiox, uint16_t pin,
                                        driver_gpio_speed_t speed)
{
    if (gpiox == NULL)
        return DRIVER_ERR_NO_GPIO;

    if (pin > 15)
        return DRIVER_ERR_INVALID_PIN;

    if (speed > DRIVER_GPIO_SPEED_VERY_HIGH)
        return DRIVER_ERR_INVALID_MODE;

    uint32_t shift = (uint32_t)pin << 1;

    gpiox->OSPEEDR &= ~(0x3UL << shift);
    gpiox->OSPEEDR |= ((uint32_t)speed << shift);

    return DRIVER_GPIO_OK;
}

driver_gpio_err_t driver_gpio_set_pull(GPIO_TypeDef *gpiox, uint16_t pin,
                                       driver_gpio_pull_t pull)
{
    if (gpiox == NULL)
        return DRIVER_ERR_NO_GPIO;

    if (pin > 15)
        return DRIVER_ERR_INVALID_PIN;

    if (pull > DRIVER_GPIO_PULL_DOWN)
        return DRIVER_ERR_INVALID_MODE;

    uint32_t shift = (uint32_t)pin << 1;

    gpiox->PUPDR &= ~(0x3UL << shift);
    gpiox->PUPDR |= ((uint32_t)pull << shift);

    return DRIVER_GPIO_OK;
}

driver_gpio_err_t driver_gpio_write_pin(GPIO_TypeDef *gpiox, uint16_t pin,
                                        driver_gpio_pin_state_t state)
{
    if (gpiox == NULL)
        return DRIVER_ERR_NO_GPIO;

    if (pin > 15)
        return DRIVER_ERR_INVALID_PIN;

    if (state > DRIVER_GPIO_PIN_SET)
        return DRIVER_ERR_INVALID_MODE;

    /* BSRR é atômico: bits [15:0] setam, bits [31:16] resetam. Sem read-modify-write. */
    if (state == DRIVER_GPIO_PIN_SET)
        gpiox->BSRR = (1UL << pin);
    else
        gpiox->BSRR = (1UL << (pin + 16U));

    return DRIVER_GPIO_OK;
}

driver_gpio_err_t driver_gpio_toggle_pin(GPIO_TypeDef *gpiox, uint16_t pin)
{
    if (gpiox == NULL)
        return DRIVER_ERR_NO_GPIO;

    if (pin > 15)
        return DRIVER_ERR_INVALID_PIN;

    gpiox->ODR ^= (1U << pin);

    return DRIVER_GPIO_OK;
}

driver_gpio_err_t driver_gpio_set_alternate_function(GPIO_TypeDef *gpiox,
                                                     uint16_t pin,
                                                     uint8_t alternate_mode)
{
    if (gpiox == NULL)
        return DRIVER_ERR_NO_GPIO;

    if (pin > 15)
        return DRIVER_ERR_INVALID_PIN;

    if (alternate_mode > 15)
        return DRIVER_ERR_INVALID_MODE;

    uint32_t idx = pin >> 3;
    uint32_t shift = (pin & 0x7UL) << 2;

    gpiox->MODER &= ~(0x3UL << (pin << 1));
    gpiox->MODER |= (0x2UL << (pin << 1));

    gpiox->AFR[idx] &= ~(0xFUL << shift);
    gpiox->AFR[idx] |= ((uint32_t)alternate_mode << shift);

    return DRIVER_GPIO_OK;
}