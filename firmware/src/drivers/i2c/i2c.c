#include "i2c.h"

/* ------------------------------------------------------------------ */
/* Helpers internos                                                    */
/* ------------------------------------------------------------------ */

static driver_err_t check_ready(const i2c_t *h)
{
    if (h == NULL)
        return DRIVER_ERR_INVALID_ARG;
    if (!h->initialized)
        return DRIVER_ERR_I2C_NOT_INITIALIZED;
    return DRIVER_OK;
}

/** Liga (ou desliga) o clock da instância no APB1. */
static driver_err_t i2c_clock(const I2C_TypeDef *i2c, bool on)
{
    uint32_t bit;

    if (i2c == I2C1)
        bit = RCC_APB1ENR_I2C1EN;
    else if (i2c == I2C2)
        bit = RCC_APB1ENR_I2C2EN;
    else if (i2c == I2C3)
        bit = RCC_APB1ENR_I2C3EN;
    else
        return DRIVER_ERR_INVALID_ARG;

    if (on)
        RCC->APB1ENR |= bit;
    else
        RCC->APB1ENR &= ~bit;

    (void)RCC->APB1ENR; /* garante o clock antes do primeiro acesso */
    return DRIVER_OK;
}

/** Pulso de reset da instância pelo RCC (limpa qualquer estado travado). */
static void i2c_rcc_reset(const I2C_TypeDef *i2c)
{
    uint32_t bit = RCC_APB1RSTR_I2C1RST;

    if (i2c == I2C2)
        bit = RCC_APB1RSTR_I2C2RST;
    else if (i2c == I2C3)
        bit = RCC_APB1RSTR_I2C3RST;

    RCC->APB1RSTR |= bit;
    RCC->APB1RSTR &= ~bit;
}

/** Pino em alternate function open-drain, sem pull. */
static driver_err_t config_af_pin(const i2c_pin_t *p)
{
    driver_err_t err = driver_gpio_enable_clock(p->port);
    if (err != DRIVER_OK)
        return err;

    /* open-drain antes do AF: o pino nunca é acionado em push-pull */
    err = driver_gpio_set_output_type(p->port, p->pin, DRIVER_GPIO_OTYPE_OPENDRAIN);
    if (err != DRIVER_OK)
        return err;

    err = driver_gpio_set_pull(p->port, p->pin, DRIVER_GPIO_PULL_NONE);
    if (err != DRIVER_OK)
        return err;

    err = driver_gpio_set_speed(p->port, p->pin, DRIVER_GPIO_SPEED_HIGH);
    if (err != DRIVER_OK)
        return err;

    return driver_gpio_set_alternate_function(p->port, p->pin, p->alternate_function);
}

/** Pino em saída open-drain liberado (nível alto pelo pull-up), para a recuperação. */
static driver_err_t config_gpio_pin(const i2c_pin_t *p)
{
    driver_err_t err = driver_gpio_write_pin(p->port, p->pin, DRIVER_GPIO_PIN_SET);
    if (err != DRIVER_OK)
        return err;

    return driver_gpio_set_mode(p->port, p->pin, DRIVER_GPIO_OUTPUT);
}

/**
 * Calcula CR2.FREQ, CCR e TRISE (RM0383, seção do I2C_CCR).
 * Standard: Thigh = Tlow = CCR · Tpclk        -> CCR = pclk / (2 f)
 * Fast:     Tlow = 2 · Thigh = 2 · CCR · Tpclk -> CCR = pclk / (3 f)
 * Arredondado para cima para não passar da frequência pedida.
 */
static driver_err_t config_timing(i2c_t *h)
{
    const i2c_config_t *cfg = &h->cfg;
    I2C_TypeDef *const i2c = cfg->i2c;

    const uint32_t freq_mhz = cfg->pclk1_hz / 1000000U;
    uint32_t ccr;
    uint32_t trise;

    if (freq_mhz < 2U || freq_mhz > 50U)
        return DRIVER_ERR_I2C_INVALID_SPEED;

    if (cfg->speed == I2C_SPEED_STANDARD)
    {
        ccr = (cfg->pclk1_hz + 2U * cfg->speed - 1U) / (2U * cfg->speed);
        if (ccr < 4U)
            ccr = 4U;
        trise = freq_mhz + 1U; /* tr máx. 1000 ns */
    }
    else if (cfg->speed == I2C_SPEED_FAST)
    {
        if (freq_mhz < 4U)
            return DRIVER_ERR_I2C_INVALID_SPEED;

        ccr = (cfg->pclk1_hz + 3U * cfg->speed - 1U) / (3U * cfg->speed);
        if (ccr < 1U)
            ccr = 1U;
        trise = (freq_mhz * 300U) / 1000U + 1U; /* tr máx. 300 ns */
    }
    else
    {
        return DRIVER_ERR_I2C_INVALID_SPEED;
    }

    if (ccr > I2C_CCR_CCR) /* campo de 12 bits */
        return DRIVER_ERR_I2C_INVALID_SPEED;

    if (cfg->speed == I2C_SPEED_FAST)
        ccr |= I2C_CCR_FS; /* DUTY = 0 (Tlow/Thigh = 2) */

    i2c->CR1 = 0; /* PE = 0 durante a configuração */
    i2c->CR2 = freq_mhz;
    i2c->CCR = ccr;
    i2c->TRISE = trise;
    i2c->CR1 = I2C_CR1_PE;

    return DRIVER_OK;
}

/** Gera STOP e espera o hardware concluí-lo. */
static void send_stop(i2c_t *h)
{
    I2C_TypeDef *const i2c = h->cfg.i2c;

    i2c->CR1 |= I2C_CR1_STOP;

    for (uint32_t n = h->cfg.timeout; n > 0U; n--)
        if (!(i2c->CR1 & I2C_CR1_STOP))
            break;

    i2c->CR1 &= ~I2C_CR1_POS;
}

/**
 * Verifica flags de erro em SR1. Os flags são rc_w0: escrever 0 limpa só o bit.
 * Em NACK e BERR o mestre ainda controla o barramento: gera STOP.
 * Em ARLO o hardware já voltou ao modo escravo: não há STOP a gerar.
 */
static driver_err_t check_errors(i2c_t *h)
{
    I2C_TypeDef *const i2c = h->cfg.i2c;
    const uint32_t sr1 = i2c->SR1;

    if (sr1 & I2C_SR1_AF)
    {
        i2c->SR1 = ~I2C_SR1_AF;
        send_stop(h);
        return DRIVER_ERR_I2C_NACK;
    }
    if (sr1 & I2C_SR1_BERR)
    {
        i2c->SR1 = ~I2C_SR1_BERR;
        send_stop(h);
        return DRIVER_ERR_I2C_BUS_ERROR;
    }
    if (sr1 & I2C_SR1_ARLO)
    {
        i2c->SR1 = ~I2C_SR1_ARLO;
        return DRIVER_ERR_I2C_ARLO;
    }
    return DRIVER_OK;
}

/** Espera um flag de SR1, checando erros a cada iteração. */
static driver_err_t wait_sr1(i2c_t *h, uint32_t mask)
{
    I2C_TypeDef *const i2c = h->cfg.i2c;

    for (uint32_t n = h->cfg.timeout; n > 0U; n--)
    {
        if (i2c->SR1 & mask)
            return DRIVER_OK;

        const driver_err_t err = check_errors(h);
        if (err != DRIVER_OK)
            return err;
    }

    send_stop(h);
    return DRIVER_ERR_I2C_TIMEOUT;
}

/** Espera o barramento ficar livre antes de uma nova transação. */
static driver_err_t wait_not_busy(i2c_t *h)
{
    for (uint32_t n = h->cfg.timeout; n > 0U; n--)
        if (!(h->cfg.i2c->SR2 & I2C_SR2_BUSY))
            return DRIVER_OK;

    return DRIVER_ERR_I2C_BUSY;
}

/** ADDR só é limpo pela leitura de SR1 seguida de SR2. */
static void clear_addr(I2C_TypeDef *i2c)
{
    (void)i2c->SR1;
    (void)i2c->SR2;
}

/**
 * START (ou repeated start) + endereço. Retorna com ADDR setado e ainda não
 * limpo, porque a sequência de leitura precisa ajustar ACK/POS antes disso.
 */
static driver_err_t start_addr(i2c_t *h, uint8_t addr, bool read)
{
    I2C_TypeDef *const i2c = h->cfg.i2c;

    i2c->CR1 |= I2C_CR1_START;

    driver_err_t err = wait_sr1(h, I2C_SR1_SB);
    if (err != DRIVER_OK)
        return err;

    i2c->DR = (uint8_t)((addr << 1) | (read ? 1U : 0U)); /* leitura de SR1 + escrita em DR limpa SB */

    return wait_sr1(h, I2C_SR1_ADDR);
}

/** Envia bytes com ADDR já limpo; termina com BTF (último byte transmitido). */
static driver_err_t send_bytes(i2c_t *h, const uint8_t *data, uint16_t len)
{
    I2C_TypeDef *const i2c = h->cfg.i2c;

    for (uint16_t i = 0; i < len; i++)
    {
        driver_err_t err = wait_sr1(h, I2C_SR1_TXE);
        if (err != DRIVER_OK)
            return err;

        i2c->DR = data[i];
    }

    return wait_sr1(h, I2C_SR1_BTF);
}

/** Envia o endereço do registrador (1 ou 2 bytes, MSB primeiro). */
static driver_err_t send_reg(i2c_t *h, uint16_t reg, uint8_t reg_size)
{
    const uint8_t buf[2] = {(uint8_t)(reg >> 8), (uint8_t)reg};

    return (reg_size == 2U) ? send_bytes(h, buf, 2U) : send_bytes(h, &buf[1], 1U);
}

/**
 * Recebe @p len bytes após START + endereço de leitura (ADDR setado e não limpo).
 * Segue as sequências do RM0383 para 1, 2 e 3 ou mais bytes; os trechos entre
 * limpar o ADDR/ler DR e setar o STOP não podem ser interrompidos.
 */
static driver_err_t receive_bytes(i2c_t *h, uint8_t *data, uint16_t len)
{
    I2C_TypeDef *const i2c = h->cfg.i2c;
    driver_err_t err;
    uint32_t primask;

    if (len == 1U)
    {
        i2c->CR1 &= ~I2C_CR1_ACK;

        primask = __get_PRIMASK();
        __disable_irq();
        clear_addr(i2c);
        i2c->CR1 |= I2C_CR1_STOP;
        __set_PRIMASK(primask);

        err = wait_sr1(h, I2C_SR1_RXNE);
        if (err != DRIVER_OK)
            return err;

        data[0] = (uint8_t)i2c->DR;
    }
    else if (len == 2U)
    {
        i2c->CR1 &= ~I2C_CR1_ACK;
        i2c->CR1 |= I2C_CR1_POS; /* o NACK vale para o próximo byte (o 2º) */
        clear_addr(i2c);

        err = wait_sr1(h, I2C_SR1_BTF); /* byte 1 em DR, byte 2 no shift register */
        if (err != DRIVER_OK)
            return err;

        primask = __get_PRIMASK();
        __disable_irq();
        i2c->CR1 |= I2C_CR1_STOP;
        data[0] = (uint8_t)i2c->DR;
        __set_PRIMASK(primask);

        data[1] = (uint8_t)i2c->DR;
    }
    else
    {
        i2c->CR1 |= I2C_CR1_ACK;
        clear_addr(i2c);

        uint16_t i = 0;

        /* Todos menos os 3 últimos: leitura normal com ACK */
        while ((uint16_t)(len - i) > 3U)
        {
            err = wait_sr1(h, I2C_SR1_RXNE);
            if (err != DRIVER_OK)
                return err;

            data[i++] = (uint8_t)i2c->DR;
        }

        /* N-2 em DR, N-1 no shift register */
        err = wait_sr1(h, I2C_SR1_BTF);
        if (err != DRIVER_OK)
            return err;

        i2c->CR1 &= ~I2C_CR1_ACK; /* NACK no byte N */
        data[i++] = (uint8_t)i2c->DR;

        /* N-1 em DR, N no shift register */
        err = wait_sr1(h, I2C_SR1_BTF);
        if (err != DRIVER_OK)
            return err;

        primask = __get_PRIMASK();
        __disable_irq();
        i2c->CR1 |= I2C_CR1_STOP;
        data[i++] = (uint8_t)i2c->DR;
        __set_PRIMASK(primask);

        err = wait_sr1(h, I2C_SR1_RXNE);
        if (err != DRIVER_OK)
            return err;

        data[i] = (uint8_t)i2c->DR;
    }

    /* Espera o STOP terminar e restaura ACK/POS para a próxima transação */
    for (uint32_t n = h->cfg.timeout; n > 0U; n--)
        if (!(i2c->CR1 & I2C_CR1_STOP))
            break;

    i2c->CR1 &= ~I2C_CR1_POS;
    i2c->CR1 |= I2C_CR1_ACK;

    return DRIVER_OK;
}

/** Atraso curto (~meio período de SCL a 100 kHz) para a recuperação do barramento. */
static void recover_delay(void)
{
    for (volatile uint32_t i = SystemCoreClock / 1000000U * 2U; i > 0U; i--)
        ;
}

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */

driver_err_t i2c_init(i2c_t *h, const i2c_config_t *cfg)
{
    if (h == NULL || cfg == NULL || cfg->i2c == NULL)
        return DRIVER_ERR_INVALID_ARG;

    h->initialized = false;
    h->cfg = *cfg;
    if (h->cfg.timeout == 0U)
        h->cfg.timeout = I2C_DEFAULT_TIMEOUT;

    driver_err_t err = i2c_clock(cfg->i2c, true); /* também valida a instância */
    if (err != DRIVER_OK)
        return err;

    err = config_af_pin(&cfg->scl);
    if (err != DRIVER_OK)
        return err;

    err = config_af_pin(&cfg->sda);
    if (err != DRIVER_OK)
        return err;

    i2c_rcc_reset(cfg->i2c);

    err = config_timing(h);
    if (err != DRIVER_OK)
        return err;

    h->initialized = true;
    return DRIVER_OK;
}

driver_err_t i2c_deinit(i2c_t *h)
{
    driver_err_t err = check_ready(h);
    if (err != DRIVER_OK)
        return err;

    h->cfg.i2c->CR1 = 0;
    (void)i2c_clock(h->cfg.i2c, false);
    h->initialized = false;

    return DRIVER_OK;
}

driver_err_t i2c_write(i2c_t *h, uint8_t addr, const uint8_t *data, uint16_t len)
{
    driver_err_t err = check_ready(h);
    if (err != DRIVER_OK)
        return err;
    if (data == NULL && len > 0U)
        return DRIVER_ERR_INVALID_ARG;

    err = wait_not_busy(h);
    if (err != DRIVER_OK)
        return err;

    err = start_addr(h, addr, false);
    if (err != DRIVER_OK)
        return err;

    clear_addr(h->cfg.i2c);

    err = send_bytes(h, data, len);
    if (err != DRIVER_OK)
        return err;

    send_stop(h);
    return DRIVER_OK;
}

driver_err_t i2c_read(i2c_t *h, uint8_t addr, uint8_t *data, uint16_t len)
{
    driver_err_t err = check_ready(h);
    if (err != DRIVER_OK)
        return err;
    if (data == NULL || len == 0U)
        return DRIVER_ERR_INVALID_ARG;

    err = wait_not_busy(h);
    if (err != DRIVER_OK)
        return err;

    err = start_addr(h, addr, true);
    if (err != DRIVER_OK)
        return err;

    return receive_bytes(h, data, len);
}

driver_err_t i2c_mem_write(i2c_t *h, uint8_t addr, uint16_t reg, uint8_t reg_size,
                           const uint8_t *data, uint16_t len)
{
    driver_err_t err = check_ready(h);
    if (err != DRIVER_OK)
        return err;
    if ((data == NULL && len > 0U) || (reg_size != 1U && reg_size != 2U))
        return DRIVER_ERR_INVALID_ARG;

    err = wait_not_busy(h);
    if (err != DRIVER_OK)
        return err;

    err = start_addr(h, addr, false);
    if (err != DRIVER_OK)
        return err;

    clear_addr(h->cfg.i2c);

    err = send_reg(h, reg, reg_size);
    if (err != DRIVER_OK)
        return err;

    err = send_bytes(h, data, len);
    if (err != DRIVER_OK)
        return err;

    send_stop(h);
    return DRIVER_OK;
}

driver_err_t i2c_mem_read(i2c_t *h, uint8_t addr, uint16_t reg, uint8_t reg_size,
                          uint8_t *data, uint16_t len)
{
    driver_err_t err = check_ready(h);
    if (err != DRIVER_OK)
        return err;
    if (data == NULL || len == 0U || (reg_size != 1U && reg_size != 2U))
        return DRIVER_ERR_INVALID_ARG;

    err = wait_not_busy(h);
    if (err != DRIVER_OK)
        return err;

    err = start_addr(h, addr, false);
    if (err != DRIVER_OK)
        return err;

    clear_addr(h->cfg.i2c);

    err = send_reg(h, reg, reg_size);
    if (err != DRIVER_OK)
        return err;

    /* Repeated start: não há STOP entre a escrita do registrador e a leitura */
    err = start_addr(h, addr, true);
    if (err != DRIVER_OK)
        return err;

    return receive_bytes(h, data, len);
}

driver_err_t i2c_is_device_ready(i2c_t *h, uint8_t addr, uint8_t trials)
{
    driver_err_t err = check_ready(h);
    if (err != DRIVER_OK)
        return err;

    if (trials == 0U)
        trials = 1U;

    while (trials-- > 0U)
    {
        err = wait_not_busy(h);
        if (err != DRIVER_OK)
            return err;

        err = start_addr(h, addr, false);
        if (err == DRIVER_OK)
        {
            clear_addr(h->cfg.i2c);
            send_stop(h);
            return DRIVER_OK;
        }
        if (err != DRIVER_ERR_I2C_NACK)
            return err; /* NACK: tenta de novo; o STOP já foi gerado */
    }

    return DRIVER_ERR_I2C_NACK;
}

driver_err_t i2c_bus_recover(i2c_t *h)
{
    driver_err_t err = check_ready(h);
    if (err != DRIVER_OK)
        return err;

    const i2c_pin_t *scl = &h->cfg.scl;
    const i2c_pin_t *sda = &h->cfg.sda;

    h->cfg.i2c->CR1 = 0; /* PE = 0: os pinos passam a ser controlados pelo GPIO */

    err = config_gpio_pin(scl);
    if (err != DRIVER_OK)
        return err;

    err = config_gpio_pin(sda);
    if (err != DRIVER_OK)
        return err;

    /* Até 9 pulsos de clock: o escravo termina o byte em curso e solta o SDA */
    for (uint8_t i = 0; i < 9U; i++)
    {
        if (driver_gpio_read_pin(sda->port, sda->pin) == DRIVER_GPIO_PIN_SET)
            break;

        driver_gpio_write_pin(scl->port, scl->pin, DRIVER_GPIO_PIN_RESET);
        recover_delay();
        driver_gpio_write_pin(scl->port, scl->pin, DRIVER_GPIO_PIN_SET);
        recover_delay();
    }

    /* STOP manual: SDA sobe com SCL em nível alto */
    driver_gpio_write_pin(sda->port, sda->pin, DRIVER_GPIO_PIN_RESET);
    recover_delay();
    driver_gpio_write_pin(scl->port, scl->pin, DRIVER_GPIO_PIN_SET);
    recover_delay();
    driver_gpio_write_pin(sda->port, sda->pin, DRIVER_GPIO_PIN_SET);
    recover_delay();

    const bool sda_free = (driver_gpio_read_pin(sda->port, sda->pin) == DRIVER_GPIO_PIN_SET);

    err = config_af_pin(scl);
    if (err != DRIVER_OK)
        return err;

    err = config_af_pin(sda);
    if (err != DRIVER_OK)
        return err;

    /* Reset do periférico: limpa um BUSY travado */
    i2c_rcc_reset(h->cfg.i2c);

    err = config_timing(h);
    if (err != DRIVER_OK)
        return err;

    return sda_free ? DRIVER_OK : DRIVER_ERR_I2C_BUSY;
}
