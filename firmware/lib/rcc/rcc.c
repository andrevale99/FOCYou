#include "rcc.h"

/**
 * @brief Configura o número de estados de espera da memória Flash.
 *
 * Define a quantidade de wait states da memória Flash de acordo com
 * a frequência do clock do sistema. A configuração é necessária para
 * garantir o acesso adequado à memória Flash quando a frequência do
 * sistema é elevada.
 *
 * @param[in] SystemCoreClock
 *     Frequência atual do clock do núcleo, em Hz.
 *
 * @note
 *     Para frequências de até 30 MHz, nenhum wait state é utilizado.
 *
 * @note
 *     Para frequências entre 30 MHz e 60 MHz, é configurado 1 wait state.
 *
 * @note
 *     Para frequências entre 60 MHz e 90 MHz, são configurados 2 wait states.
 *
 * @note
 *     Para frequências superiores a 90 MHz, são configurados 3 wait states.
 *
 * @warning
 *     A configuração do número de wait states deve ser realizada antes
 *     do aumento da frequência do clock do sistema.
 */
static void set_wait_state_flash(uint32_t SystemCoreClock)
{
    if (SystemCoreClock <= 30000000U)
    {
        FLASH->ACR &= ~FLASH_ACR_LATENCY;
    }
    else if (SystemCoreClock <= 60000000U)
    {
        FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_1WS;
    }
    else if (SystemCoreClock <= 90000000U)
    {
        FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_2WS;
    }
    else
    {
        FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_3WS;
    }
}

rcc_err_t rcc_init(rcc_config_t *config)
{
    rcc_err_t err;

    /*
     * Configuração sem PLL
     */
    if (config->clock_source != RCC_PLL)
    {
        err = rcc_set_system_clock_source(config->clock_source);

        if (err != RCC_OK)
            return err;
    }

    /*
     * Configuração utilizando PLL
     */
    else
    {
        uint32_t pll_clock;

        if (config->pll_conf.source == RCC_HSE)
        {
            /*
             * Habilita HSE.
             */
            RCC->CR |= RCC_CR_HSEON;

            while (!(RCC->CR & RCC_CR_HSERDY))
                ;
        }
        else
        {
            /*
             * Habilita HSI.
             */
            RCC->CR |= RCC_CR_HSION;

            while (!(RCC->CR & RCC_CR_HSIRDY))
                ;
        }

        SystemCoreClockUpdate();

        /*
         * Calcula a frequência que será produzida pelo PLL.
         *
         * HSE_VALUE é definido pelo CMSIS, normalmente
         * em stm32f4xx.h.
         */
        pll_clock =
            ((SystemCoreClock / config->pll_conf.m_factor) *
             config->pll_conf.n_factor) /
            config->pll_conf.p_factor;

        /*
         * Configura Flash antes de aumentar a frequência
         * do sistema.
         */
        set_wait_state_flash(pll_clock);

        /*
         * Configura PLL.
         */
        err = rcc_configure_pll(&config->pll_conf);

        if (err != RCC_OK)
            return err;

        rcc_set_system_clock_source(config->clock_source);
    }

    /*
     * Atualiza SystemCoreClock depois que o SYSCLK
     * foi efetivamente alterado.
     */
    SystemCoreClockUpdate();

    /*
     * Configura AHB.
     */
    err = rcc_set_ahb_divider(config->ahb_divider);

    if (err != RCC_OK)
        return err;

    /*
     * Configura APB1.
     */
    err = rcc_set_apbx_divider(1, config->apb1_divider);

    if (err != RCC_OK)
        return err;

    /*
     * Configura APB2.
     */
    err = rcc_set_apbx_divider(2, config->apb2_divider);

    if (err != RCC_OK)
        return err;

    /*
     * Atualiza novamente após os prescalers.
     */
    SystemCoreClockUpdate();

    return RCC_OK;
}

rcc_err_t rcc_set_system_clock_source(rcc_clock_source_t clock_source)
{
    switch (clock_source)
    {
    case RCC_HSI:

        /*
         * Habilita HSI.
         */
        RCC->CR |= RCC_CR_HSION;

        /*
         * Aguarda HSI ficar pronto.
         */
        while (!(RCC->CR & RCC_CR_HSIRDY))
            ;

        /*
         * Seleciona HSI como SYSCLK.
         */
        RCC->CFGR &= ~RCC_CFGR_SW;
        RCC->CFGR |= RCC_CFGR_SW_HSI;

        /*
         * Aguarda confirmação da troca.
         */
        while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
            ;

        break;

    case RCC_HSE:

        /*
         * Habilita HSE.
         */
        RCC->CR |= RCC_CR_HSEON;

        /*
         * Aguarda HSE ficar pronto.
         */
        while (!(RCC->CR & RCC_CR_HSERDY))
            ;

        /*
         * Seleciona HSE como SYSCLK.
         */
        RCC->CFGR &= ~RCC_CFGR_SW;
        RCC->CFGR |= RCC_CFGR_SW_HSE;

        /*
         * Aguarda confirmação da troca.
         */
        while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSE)
            ;

        break;

    case RCC_PLL:

        /*
         * O PLL deve ter sido previamente configurado
         * em RCC->PLLCFGR.
         */

        /*
         * Habilita PLL.
         */
        RCC->CR |= RCC_CR_PLLON;

        /*
         * Aguarda PLL ficar pronto.
         */
        while (!(RCC->CR & RCC_CR_PLLRDY))
            ;

        /*
         * Seleciona PLL como SYSCLK.
         */
        RCC->CFGR &= ~RCC_CFGR_SW;
        RCC->CFGR |= RCC_CFGR_SW_PLL;

        /*
         * Aguarda confirmação da troca.
         */
        while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
            ;

        break;

    default:

        return RCC_ERR_INVALID_CLOCK_SOURCE;
    }

    return RCC_OK;
}

rcc_err_t rcc_configure_pll(rcc_pll_config_t *pll)
{
    uint32_t pll_p;
    uint32_t pll_source;

    /*
     * Seleciona a fonte do PLL.
     *
     * PLLSRC = 0 -> HSI
     * PLLSRC = 1 -> HSE
     */
    switch (pll->source)
    {
    case RCC_HSI:
        pll_source = 0U;
        break;

    case RCC_HSE:
        pll_source = RCC_PLLCFGR_PLLSRC;
        break;

    default:
        return RCC_ERR_INVALID_CLOCK_SOURCE;
    }

    /*
     * Verifica o fator M.
     *
     * No STM32F411, PLLM deve estar entre 2 e 63.
     */
    if ((pll->m_factor < 2U) || (pll->m_factor > 63U))
    {
        return RCC_ERR_INVALID_PLL_M_FACTOR;
    }

    /*
     * Verifica o fator N.
     *
     * No STM32F411, PLLN deve estar entre 50 e 432.
     */
    if ((pll->n_factor < 50U) || (pll->n_factor > 432U))
    {
        return RCC_ERR_INVALID_PLL_N_FACTOR;
    }

    /*
     * PLLP aceita somente os valores:
     *
     * 2 -> 00
     * 4 -> 01
     * 6 -> 10
     * 8 -> 11
     */
    switch (pll->p_factor)
    {
    case RCC_PLL_P_DIV_2:
        pll_p = 0U;
        break;

    case RCC_PLL_P_DIV_4:
        pll_p = 1U;
        break;

    case RCC_PLL_P_DIV_6:
        pll_p = 2U;
        break;

    case RCC_PLL_P_DIV_8:
        pll_p = 3U;
        break;

    default:
        return RCC_ERR_INVALID_PLL_P_FACTOR;
    }

    /*
     * Desabilita o PLL antes de alterar PLLCFGR.
     */
    RCC->CR &= ~RCC_CR_PLLON;

    /*
     * Aguarda o PLL ser desabilitado.
     */
    while (RCC->CR & RCC_CR_PLLRDY)
        ;

    /*
     * Configuração do PLL:
     *
     * PLL input  = source / M
     * VCO        = PLL input * N
     * PLL output = VCO / P
     */
    RCC->PLLCFGR =
        (pll->m_factor << RCC_PLLCFGR_PLLM_Pos) | (pll->n_factor << RCC_PLLCFGR_PLLN_Pos) | (pll_p << RCC_PLLCFGR_PLLP_Pos) | pll_source;

    /*
     * Habilita PLL.
     */
    RCC->CR |= RCC_CR_PLLON;

    /*
     * Aguarda o PLL estabilizar.
     */
    while (!(RCC->CR & RCC_CR_PLLRDY))
        ;

    return RCC_OK;
}

rcc_err_t rcc_set_apbx_divider(int apb_num, rcc_apb_divider_t divider)
{

    switch (divider)
    {
    case RCC_APBx_DIV_0:

        if (apb_num == 1)
        {
            RCC->CFGR &= ~RCC_CFGR_PPRE1;     // Clear PPRE1 bits
            RCC->CFGR |= RCC_CFGR_PPRE1_DIV1; // Set APB1 divider to 0
        }
        else if (apb_num == 2)
        {
            RCC->CFGR &= ~RCC_CFGR_PPRE2;     // Clear PPRE2 bits
            RCC->CFGR |= RCC_CFGR_PPRE2_DIV1; // Set APB2 divider to 0
        }
        else
            return RCC_ERR_INVALID_APB_NUM; // Error: Invalid APB number
        break;

    case RCC_APBx_DIV_2:

        if (apb_num == 1)
        {
            RCC->CFGR &= ~RCC_CFGR_PPRE1;     // Clear PPRE1 bits
            RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; // Set APB1 divider to 2
        }
        else if (apb_num == 2)
        {
            RCC->CFGR &= ~RCC_CFGR_PPRE2;     // Clear PPRE2 bits
            RCC->CFGR |= RCC_CFGR_PPRE2_DIV2; // Set APB2 divider to 2
        }
        else
            return RCC_ERR_INVALID_APB_NUM; // Error: Invalid APB number
        break;

    case RCC_APBx_DIV_4:

        if (apb_num == 1)
        {
            RCC->CFGR &= ~RCC_CFGR_PPRE1;     // Clear PPRE1 bits
            RCC->CFGR |= RCC_CFGR_PPRE1_DIV4; // Set APB1 divider to 4
        }
        else if (apb_num == 2)
        {
            RCC->CFGR &= ~RCC_CFGR_PPRE2;     // Clear PPRE2 bits
            RCC->CFGR |= RCC_CFGR_PPRE2_DIV4; // Set APB2 divider to 4
        }
        else
            return RCC_ERR_INVALID_APB_NUM; // Error: Invalid APB number
        break;

    case RCC_APBx_DIV_8:

        if (apb_num == 1)
        {
            RCC->CFGR &= ~RCC_CFGR_PPRE1;     // Clear PPRE1 bits
            RCC->CFGR |= RCC_CFGR_PPRE1_DIV8; // Set APB1 divider to 8
        }
        else if (apb_num == 2)
        {
            RCC->CFGR &= ~RCC_CFGR_PPRE2;     // Clear PPRE2 bits
            RCC->CFGR |= RCC_CFGR_PPRE2_DIV8; // Set APB2 divider to 8
        }
        else
            return RCC_ERR_INVALID_APB_NUM; // Error: Invalid APB number
        break;

    case RCC_APBx_DIV_16:

        if (apb_num == 1)
        {
            RCC->CFGR &= ~RCC_CFGR_PPRE1;      // Clear PPRE1 bits
            RCC->CFGR |= RCC_CFGR_PPRE1_DIV16; // Set APB1 divider to 16
        }
        else if (apb_num == 2)
        {
            RCC->CFGR &= ~RCC_CFGR_PPRE2;      // Clear PPRE2 bits
            RCC->CFGR |= RCC_CFGR_PPRE2_DIV16; // Set APB2 divider to 16
        }
        else
            return RCC_ERR_INVALID_APB_NUM; // Error: Invalid APB number
        break;

    default:
        return RCC_ERR_INVALID_APB_DIVIDER; // Error
    }

    return RCC_OK;
}

rcc_err_t rcc_set_ahb_divider(rcc_ahb_divider_t divider)
{
    switch (divider)
    {
    case RCC_AHB_DIV_0:
        RCC->CFGR &= ~RCC_CFGR_HPRE;     // Clear HPRE bits
        RCC->CFGR |= RCC_CFGR_HPRE_DIV1; // Set AHB divider
        break;

    case RCC_AHB_DIV_2:
        RCC->CFGR &= ~RCC_CFGR_HPRE;     // Clear HPRE bits
        RCC->CFGR |= RCC_CFGR_HPRE_DIV2; // Set AHB divider
        break;

    case RCC_AHB_DIV_4:
        RCC->CFGR &= ~RCC_CFGR_HPRE;     // Clear HPRE bits
        RCC->CFGR |= RCC_CFGR_HPRE_DIV4; // Set AHB divider
        break;

    case RCC_AHB_DIV_8:
        RCC->CFGR &= ~RCC_CFGR_HPRE;     // Clear HPRE bits
        RCC->CFGR |= RCC_CFGR_HPRE_DIV8; // Set AHB divider
        break;

    case RCC_AHB_DIV_16:
        RCC->CFGR &= ~RCC_CFGR_HPRE;      // Clear HPRE bits
        RCC->CFGR |= RCC_CFGR_HPRE_DIV16; // Set AHB divider
        break;

    case RCC_AHB_DIV_64:
        RCC->CFGR &= ~RCC_CFGR_HPRE;      // Clear HPRE
        RCC->CFGR |= RCC_CFGR_HPRE_DIV64; // Set AHB divider
        break;

    case RCC_AHB_DIV_128:
        RCC->CFGR &= ~RCC_CFGR_HPRE;       // Clear HPRE
        RCC->CFGR |= RCC_CFGR_HPRE_DIV128; // Set AHB divider
        break;

    case RCC_AHB_DIV_256:
        RCC->CFGR &= ~RCC_CFGR_HPRE;       // Clear HPRE
        RCC->CFGR |= RCC_CFGR_HPRE_DIV256; // Set AHB divider
        break;

    case RCC_AHB_DIV_512:
        RCC->CFGR &= ~RCC_CFGR_HPRE;       // Clear HPRE
        RCC->CFGR |= RCC_CFGR_HPRE_DIV512; // Set AHB divider
        break;

    default:
        return RCC_ERR_INVALID_APB_DIVIDER; // Error
        break;
    }

    return RCC_OK;
}

void rcc_get_clock_status(rcc_clock_status_t *status)
{

    // Get clock source
    uint32_t sw = RCC->CFGR & RCC_CFGR_SWS;
    if (sw == RCC_CFGR_SWS_HSI)
        status->clock_source = RCC_HSI;
    else if (sw == RCC_CFGR_SWS_HSE)
        status->clock_source = RCC_HSE;
    else if (sw == RCC_CFGR_SWS_PLL)
        status->clock_source = RCC_PLL;
    else
        status->clock_source = RCC_ERR_INVALID_CLOCK_SOURCE; // Invalid clock source

    // Get AHB divider
    uint32_t div = (RCC->CFGR & RCC_CFGR_HPRE) >> 4;
    status->ahb_divider = div;

    // Get APB1 divider
    div = (RCC->CFGR & RCC_CFGR_PPRE1) >> 10;
    status->apb1_divider = div;

    // Get APB2 divider
    div = (RCC->CFGR & RCC_CFGR_PPRE2) >> 13;
    status->apb2_divider = div;
}