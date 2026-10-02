#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"

#include "inverter.h"

/* LED da Black Pill: PC13 (ativo em nivel baixo) */
#define LED_PORT        GPIOC
#define LED_PIN         13U
#define LED_CLK_EN      RCC_AHB1ENR_GPIOCEN

static void led_init(void)
{
    RCC->AHB1ENR |= LED_CLK_EN;
    (void)RCC->AHB1ENR;                             /* atraso para o clock estabilizar */

    LED_PORT->MODER   &= ~(3U << (LED_PIN * 2U));
    LED_PORT->MODER   |=  (1U << (LED_PIN * 2U));   /* saida */
    LED_PORT->OTYPER  &= ~(1U << LED_PIN);          /* push-pull */
    LED_PORT->OSPEEDR &= ~(3U << (LED_PIN * 2U));   /* baixa velocidade */
    LED_PORT->PUPDR   &= ~(3U << (LED_PIN * 2U));   /* sem pull */
}

static void vBlinkTask(void *pvParameters)
{
    (void)pvParameters;

    for (;;) {
        LED_PORT->ODR ^= (1U << LED_PIN);           /* alterna o LED */
        vTaskDelay(pdMS_TO_TICKS(200));
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
