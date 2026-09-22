#include "rcc.h"

rcc_err_t rcc_init(rcc_confg_t *config)
{
    rcc_err_t err;

    // Set clock source
    err = rcc_set_clock_source(config->clock_source);
    if (err != RCC_OK)
        return err;

    // Set AHB divider
    err = rcc_set_ahb_divider(config->ahb_divider);
    if (err != RCC_OK)
        return err;

    // Set APB1 divider
    err = rcc_set_apbx_divider(1, config->apb1_divider);
    if (err != RCC_OK)
        return err;

    // Set APB2 divider
    err = rcc_set_apbx_divider(2, config->apb2_divider);
    if (err != RCC_OK)
        return err;

    return RCC_OK;
}

rcc_err_t rcc_set_clock_source(rcc_clock_source_t clock_source)
{
    switch (clock_source)
    {
    case RCC_HSI:

        RCC->CR |= RCC_CR_HSION; // Enable HSI
        for (volatile int i = 0; i < 100000; i++)
            ; // Wait for HSI to stabilize
        if (RCC->CR & RCC_CR_HSIRDY)
        {
            RCC->CFGR &= ~RCC_CFGR_SW;    // Clear SW bits
            RCC->CFGR |= RCC_CFGR_SW_HSI; // Select HSI as system clock
        }
        else
            return RCC_ERR_SET_CLOCK_SOURCE_FAILED; // Error: HSI not ready
        break;

    case RCC_HSE:

        RCC->CR |= RCC_CR_HSEON; // Enable HSE
        for (volatile int i = 0; i < 100000; i++)
            ; // Wait for HSE to stabilize
        if (RCC->CR & RCC_CR_HSERDY)
        {
            RCC->CFGR &= ~RCC_CFGR_SW;    // Clear SW bits
            RCC->CFGR |= RCC_CFGR_SW_HSE; // Select HSE as system clock
        }
        else
            return RCC_ERR_SET_CLOCK_SOURCE_FAILED; // Error: HSE not ready
        break;

    case RCC_PLL:

        RCC->CR |= RCC_CR_PLLON; // Enable PLL
        for (volatile int i = 0; i < 100000; i++)
            ; // Wait for PLL to stabilize
        if (RCC->CR & RCC_CR_PLLRDY)
        {
            RCC->CFGR &= ~RCC_CFGR_SW;    // Clear SW bits
            RCC->CFGR |= RCC_CFGR_SW_PLL; // Select PLL as system clock
        }
        else
            return RCC_ERR_SET_CLOCK_SOURCE_FAILED; // Error: PLL not ready
        break;

    default:
        return RCC_ERR_INVALID_CLOCK_SOURCE; // Error
    }

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