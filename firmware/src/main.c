#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"

#include "inverter.h"

#include "driver_lcd16x2.h"

/* LED da Black Pill: PC13 (ativo em nivel baixo) */
#define LED_PORT        GPIOC
#define LED_PIN         13U

static void led_init(void)
{
    driver_gpio_enable_clock(LED_PORT);
    driver_gpio_set_mode(LED_PORT, LED_PIN, DRIVER_GPIO_OUTPUT);
    driver_gpio_set_pull(LED_PORT, LED_PIN, DRIVER_GPIO_PULL_NONE);
    driver_gpio_set_speed(LED_PORT, LED_PIN, DRIVER_GPIO_SPEED_LOW);
}

static void vBlinkTask(void *pvParameters)
{
    (void)pvParameters;

    for (;;) {
        driver_gpio_toggle_pin(LED_PORT,LED_PIN);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

int main(void)
{
    SystemCoreClockUpdate();    /* HSI 16 MHz */
    led_init();

    xTaskCreate(vBlinkTask, "blink", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    vTaskStartScheduler();      /* nao retorna */

    for (;;) { }                /* so chega aqui se faltar heap */
}
