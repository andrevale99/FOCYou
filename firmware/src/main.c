#include <stm32f411xe.h>

#include "rcc.h"

int main(void)
{
    /* Habilita acesso à FPU */
    SCB->CPACR |= (0xFU << 20);

    __DSB();
    __ISB();

    rcc_config_t rcc_config =
        {
            /*
             * PLL será utilizado como SYSCLK.
             */
            .clock_source = RCC_PLL,

            /*
             * Configuração do PLL:
             *
             * HSE = 25 MHz
             *
             * PLL input = 25 / 25 = 1 MHz
             * VCO        = 1 * 200 = 200 MHz
             * PLL output = 200 / 4 = 50 MHz
             */
            .pll_conf =
                {
                    .source = RCC_HSE,
                    .m_factor = 25U,
                    .n_factor = 200U,
                    .p_factor = RCC_PLL_P_DIV_4},

            /*
             * HCLK = SYSCLK / 1
             *      = 50 MHz
             */
            .ahb_divider = RCC_AHB_DIV_0,

            /*
             * PCLK1 = HCLK / 1
             *       = 50 MHz
             */
            .apb1_divider = RCC_APBx_DIV_0,

            /*
             * PCLK2 = HCLK / 1
             *       = 50 MHz
             */
            .apb2_divider = RCC_APBx_DIV_0};

    rcc_err_t err;

    err = rcc_init(&rcc_config);

    if (err != RCC_OK)
    {
        while (1)
            ;
    }
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // Enable GPIOC clock
    GPIOC->MODER |= GPIO_MODER_MODER13_0;
    GPIOC->BSRR |= GPIO_BSRR_BR13; // Clock source is HSE

    while (1)
    {
    }
}