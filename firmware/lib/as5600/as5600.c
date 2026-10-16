#include "as5600.h"

/* Bits do STATUS */
#define STATUS_MH (1U << 3)
#define STATUS_ML (1U << 4)
#define STATUS_MD (1U << 5)

/* Comandos do registrador BURN */
#define CMD_BURN_ANGLE 0x80U
#define CMD_BURN_SETTING 0x40U

/* CONF ocupa os bits 13:0; os bits 15:14 não são usados */
#define CONF_MASK 0x3FFFU

/* ------------------------------------------------------------------ */
/* Helpers internos                                                    */
/* ------------------------------------------------------------------ */

static as5600_err_t check_ready(const as5600_t *h)
{
    if (h == NULL)
        return AS5600_ERR_INVALID_ARG;
    if (!h->initialized)
        return AS5600_ERR_NOT_INITIALIZED;
    return AS5600_OK;
}

/** Guarda o código do driver I2C e converte para o erro da biblioteca. */
static as5600_err_t from_i2c(as5600_t *h, driver_err_t err)
{
    h->i2c_err = err;
    return (err == DRIVER_OK) ? AS5600_OK : AS5600_ERR_I2C;
}

static as5600_err_t read_regs(as5600_t *h, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return from_i2c(h, i2c_mem_read(h->cfg.i2c, h->cfg.address, reg, 1U, buf, len));
}

static as5600_err_t write_regs(as5600_t *h, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    return from_i2c(h, i2c_mem_write(h->cfg.i2c, h->cfg.address, reg, 1U, buf, len));
}

static as5600_err_t read_u8(as5600_t *h, uint8_t reg, uint8_t *value)
{
    return read_regs(h, reg, value, 1U);
}

static as5600_err_t write_u8(as5600_t *h, uint8_t reg, uint8_t value)
{
    return write_regs(h, reg, &value, 1U);
}

/** Lê um par de registradores, MSB no endereço menor. */
static as5600_err_t read_u16(as5600_t *h, uint8_t reg, uint16_t *value)
{
    uint8_t buf[2];

    const as5600_err_t err = read_regs(h, reg, buf, 2U);
    if (err != AS5600_OK)
        return err;

    *value = (uint16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    return AS5600_OK;
}

static as5600_err_t write_u16(as5600_t *h, uint8_t reg, uint16_t value)
{
    const uint8_t buf[2] = {(uint8_t)(value >> 8), (uint8_t)value};

    return write_regs(h, reg, buf, 2U);
}

/** Lê um registrador de 12 bits (os 4 bits altos do MSB não são usados). */
static as5600_err_t read_u12(as5600_t *h, uint8_t reg, uint16_t *value)
{
    const as5600_err_t err = read_u16(h, reg, value);
    if (err == AS5600_OK)
        *value &= AS5600_MAX_COUNT;
    return err;
}

/** Validação comum dos getters: handle pronto e ponteiro de saída válido. */
static as5600_err_t check_out(const as5600_t *h, const void *out)
{
    const as5600_err_t err = check_ready(h);
    if (err != AS5600_OK)
        return err;
    return (out == NULL) ? AS5600_ERR_INVALID_ARG : AS5600_OK;
}

static as5600_err_t get_u12(as5600_t *h, uint8_t reg, uint16_t *value)
{
    const as5600_err_t err = check_out(h, value);
    if (err != AS5600_OK)
        return err;
    return read_u12(h, reg, value);
}

static as5600_err_t set_u12(as5600_t *h, uint8_t reg, uint16_t value)
{
    const as5600_err_t err = check_ready(h);
    if (err != AS5600_OK)
        return err;
    if (value > AS5600_MAX_COUNT)
        return AS5600_ERR_OUT_OF_RANGE;
    return write_u16(h, reg, value);
}

/** Envia um comando ao registrador BURN e espera #AS5600_CMD_DELAY_MS. */
static as5600_err_t burn_cmd(as5600_t *h, uint8_t cmd)
{
    const as5600_err_t err = write_u8(h, AS5600_REG_BURN, cmd);
    if (err != AS5600_OK)
        return err;

    h->cfg.delay_ms(AS5600_CMD_DELAY_MS);
    return AS5600_OK;
}

/** Recarrega o conteúdo do OTP nos registradores (verificação após a gravação). */
static as5600_err_t load_otp(as5600_t *h)
{
    static const uint8_t cmds[] = {0x01U, 0x11U, 0x10U};

    for (size_t i = 0; i < sizeof(cmds); i++)
    {
        const as5600_err_t err = burn_cmd(h, cmds[i]);
        if (err != AS5600_OK)
            return err;
    }
    return AS5600_OK;
}

static uint16_t conf_pack(const as5600_conf_t *c)
{
    return (uint16_t)(((uint16_t)c->power_mode << 0) |
                      ((uint16_t)c->hysteresis << 2) |
                      ((uint16_t)c->output_stage << 4) |
                      ((uint16_t)c->pwm_freq << 6) |
                      ((uint16_t)c->slow_filter << 8) |
                      ((uint16_t)c->fast_filter << 10) |
                      ((c->watchdog ? 1U : 0U) << 13));
}

static void conf_unpack(uint16_t v, as5600_conf_t *c)
{
    c->power_mode = (as5600_power_mode_t)((v >> 0) & 0x3U);
    c->hysteresis = (as5600_hysteresis_t)((v >> 2) & 0x3U);
    c->output_stage = (as5600_output_stage_t)((v >> 4) & 0x3U);
    c->pwm_freq = (as5600_pwm_freq_t)((v >> 6) & 0x3U);
    c->slow_filter = (as5600_slow_filter_t)((v >> 8) & 0x3U);
    c->fast_filter = (as5600_fast_filter_t)((v >> 10) & 0x7U);
    c->watchdog = ((v >> 13) & 0x1U) != 0U;
}

static bool conf_valid(const as5600_conf_t *c)
{
    return (unsigned)c->power_mode <= AS5600_PM_LPM3 &&
           (unsigned)c->hysteresis <= AS5600_HYST_3LSB &&
           (unsigned)c->output_stage <= AS5600_OUTS_PWM &&
           (unsigned)c->pwm_freq <= AS5600_PWMF_920HZ &&
           (unsigned)c->slow_filter <= AS5600_SF_2X &&
           (unsigned)c->fast_filter <= AS5600_FTH_10LSB;
}

/* ------------------------------------------------------------------ */
/* Ciclo de vida                                                       */
/* ------------------------------------------------------------------ */

as5600_err_t as5600_init(as5600_t *h, const as5600_config_t *cfg)
{
    if (h == NULL || cfg == NULL || cfg->i2c == NULL)
        return AS5600_ERR_INVALID_ARG;

    h->cfg = *cfg;
    if (h->cfg.address == 0U)
        h->cfg.address = AS5600_I2C_ADDR;
    h->i2c_err = DRIVER_OK;
    h->initialized = false;
    (void)as5600_reset_tracking(h);

    const driver_err_t err = i2c_is_device_ready(cfg->i2c, h->cfg.address, 3U);
    h->i2c_err = err;
    if (err == DRIVER_ERR_I2C_NACK)
        return AS5600_ERR_NOT_FOUND;
    if (err != DRIVER_OK)
        return AS5600_ERR_I2C;

    h->initialized = true;
    return AS5600_OK;
}

driver_err_t as5600_get_i2c_error(const as5600_t *h)
{
    return (h == NULL) ? DRIVER_ERR_INVALID_ARG : h->i2c_err;
}

/* ------------------------------------------------------------------ */
/* Leitura de ângulo e diagnóstico                                     */
/* ------------------------------------------------------------------ */

as5600_err_t as5600_read_raw_angle(as5600_t *h, uint16_t *raw)
{
    return get_u12(h, AS5600_REG_RAW_ANGLE, raw);
}

as5600_err_t as5600_read_angle(as5600_t *h, uint16_t *angle)
{
    return get_u12(h, AS5600_REG_ANGLE, angle);
}

as5600_err_t as5600_read_status(as5600_t *h, as5600_status_t *status)
{
    as5600_err_t err = check_out(h, status);
    if (err != AS5600_OK)
        return err;

    uint8_t v;
    err = read_u8(h, AS5600_REG_STATUS, &v);
    if (err != AS5600_OK)
        return err;

    status->magnet_detected = (v & STATUS_MD) != 0U;
    status->magnet_too_weak = (v & STATUS_ML) != 0U;
    status->magnet_too_strong = (v & STATUS_MH) != 0U;
    return AS5600_OK;
}

as5600_err_t as5600_check_magnet(as5600_t *h)
{
    as5600_status_t s;

    const as5600_err_t err = as5600_read_status(h, &s);
    if (err != AS5600_OK)
        return err;

    if (!s.magnet_detected)
        return AS5600_ERR_NO_MAGNET;
    if (s.magnet_too_weak)
        return AS5600_ERR_MAGNET_TOO_WEAK;
    if (s.magnet_too_strong)
        return AS5600_ERR_MAGNET_TOO_STRONG;
    return AS5600_OK;
}

as5600_err_t as5600_read_agc(as5600_t *h, uint8_t *agc)
{
    const as5600_err_t err = check_out(h, agc);
    if (err != AS5600_OK)
        return err;
    return read_u8(h, AS5600_REG_AGC, agc);
}

as5600_err_t as5600_read_magnitude(as5600_t *h, uint16_t *magnitude)
{
    return get_u12(h, AS5600_REG_MAGNITUDE, magnitude);
}

/* ------------------------------------------------------------------ */
/* Configuração                                                        */
/* ------------------------------------------------------------------ */

as5600_err_t as5600_get_conf(as5600_t *h, as5600_conf_t *conf)
{
    as5600_err_t err = check_out(h, conf);
    if (err != AS5600_OK)
        return err;

    uint16_t v;
    err = read_u16(h, AS5600_REG_CONF, &v);
    if (err != AS5600_OK)
        return err;

    conf_unpack(v, conf);
    return AS5600_OK;
}

as5600_err_t as5600_set_conf(as5600_t *h, const as5600_conf_t *conf)
{
    as5600_err_t err = check_ready(h);
    if (err != AS5600_OK)
        return err;
    if (conf == NULL || !conf_valid(conf))
        return AS5600_ERR_INVALID_ARG;

    uint16_t v;
    err = read_u16(h, AS5600_REG_CONF, &v);
    if (err != AS5600_OK)
        return err;

    v = (uint16_t)((v & ~CONF_MASK) | conf_pack(conf));
    return write_u16(h, AS5600_REG_CONF, v);
}

as5600_err_t as5600_get_zpos(as5600_t *h, uint16_t *zpos)
{
    return get_u12(h, AS5600_REG_ZPOS, zpos);
}

as5600_err_t as5600_set_zpos(as5600_t *h, uint16_t zpos)
{
    return set_u12(h, AS5600_REG_ZPOS, zpos);
}

as5600_err_t as5600_get_mpos(as5600_t *h, uint16_t *mpos)
{
    return get_u12(h, AS5600_REG_MPOS, mpos);
}

as5600_err_t as5600_set_mpos(as5600_t *h, uint16_t mpos)
{
    return set_u12(h, AS5600_REG_MPOS, mpos);
}

as5600_err_t as5600_get_mang(as5600_t *h, uint16_t *mang)
{
    return get_u12(h, AS5600_REG_MANG, mang);
}

as5600_err_t as5600_set_mang(as5600_t *h, uint16_t mang)
{
    const as5600_err_t err = check_ready(h);
    if (err != AS5600_OK)
        return err;
    if (mang != 0U && mang < AS5600_MIN_RANGE)
        return AS5600_ERR_OUT_OF_RANGE;
    return set_u12(h, AS5600_REG_MANG, mang);
}

/* ------------------------------------------------------------------ */
/* OTP                                                                 */
/* ------------------------------------------------------------------ */

as5600_err_t as5600_get_burn_count(as5600_t *h, uint8_t *count)
{
    as5600_err_t err = check_out(h, count);
    if (err != AS5600_OK)
        return err;

    err = read_u8(h, AS5600_REG_ZMCO, count);
    if (err == AS5600_OK)
        *count &= 0x3U;
    return err;
}

as5600_err_t as5600_burn_angle(as5600_t *h)
{
    as5600_err_t err = check_ready(h);
    if (err != AS5600_OK)
        return err;
    if (h->cfg.delay_ms == NULL)
        return AS5600_ERR_NO_DELAY;

    as5600_status_t s;
    err = as5600_read_status(h, &s);
    if (err != AS5600_OK)
        return err;
    if (!s.magnet_detected)
        return AS5600_ERR_NO_MAGNET;

    uint8_t count;
    err = as5600_get_burn_count(h, &count);
    if (err != AS5600_OK)
        return err;
    if (count >= AS5600_MAX_BURN_ANGLE)
        return AS5600_ERR_BURN_LIMIT;

    uint16_t zpos, mpos;
    err = read_u12(h, AS5600_REG_ZPOS, &zpos);
    if (err != AS5600_OK)
        return err;
    err = read_u12(h, AS5600_REG_MPOS, &mpos);
    if (err != AS5600_OK)
        return err;

    err = burn_cmd(h, CMD_BURN_ANGLE);
    if (err != AS5600_OK)
        return err;

    err = load_otp(h);
    if (err != AS5600_OK)
        return err;

    uint16_t zpos_otp, mpos_otp;
    err = read_u12(h, AS5600_REG_ZPOS, &zpos_otp);
    if (err != AS5600_OK)
        return err;
    err = read_u12(h, AS5600_REG_MPOS, &mpos_otp);
    if (err != AS5600_OK)
        return err;

    return (zpos_otp == zpos && mpos_otp == mpos) ? AS5600_OK : AS5600_ERR_VERIFY;
}

as5600_err_t as5600_burn_setting(as5600_t *h)
{
    as5600_err_t err = check_ready(h);
    if (err != AS5600_OK)
        return err;
    if (h->cfg.delay_ms == NULL)
        return AS5600_ERR_NO_DELAY;

    uint8_t count;
    err = as5600_get_burn_count(h, &count);
    if (err != AS5600_OK)
        return err;
    if (count != 0U)
        return AS5600_ERR_BURN_LIMIT;

    uint16_t mang, conf;
    err = read_u12(h, AS5600_REG_MANG, &mang);
    if (err != AS5600_OK)
        return err;
    err = read_u16(h, AS5600_REG_CONF, &conf);
    if (err != AS5600_OK)
        return err;

    err = burn_cmd(h, CMD_BURN_SETTING);
    if (err != AS5600_OK)
        return err;

    err = load_otp(h);
    if (err != AS5600_OK)
        return err;

    uint16_t mang_otp, conf_otp;
    err = read_u12(h, AS5600_REG_MANG, &mang_otp);
    if (err != AS5600_OK)
        return err;
    err = read_u16(h, AS5600_REG_CONF, &conf_otp);
    if (err != AS5600_OK)
        return err;

    const bool ok = (mang_otp == mang) && ((conf_otp & CONF_MASK) == (conf & CONF_MASK));
    return ok ? AS5600_OK : AS5600_ERR_VERIFY;
}

/* ------------------------------------------------------------------ */
/* Acompanhamento de posição e velocidade                              */
/* ------------------------------------------------------------------ */

as5600_err_t as5600_update(as5600_t *h, float dt_s)
{
    uint16_t raw;

    const as5600_err_t err = as5600_read_raw_angle(h, &raw);
    if (err != AS5600_OK)
        return err;

    if (!h->tracking)
    {
        h->last_raw = raw;
        h->turns = 0;
        h->velocity = 0.0f;
        h->tracking = true;
        return AS5600_OK;
    }

    /* Menor caminho entre as leituras: |delta| <= meia volta */
    int32_t delta = (int32_t)raw - (int32_t)h->last_raw;
    const int32_t half = (int32_t)(AS5600_COUNTS_PER_REV / 2U);

    if (delta > half)
    {
        delta -= (int32_t)AS5600_COUNTS_PER_REV; /* passou por zero no sentido negativo */
        h->turns--;
    }
    else if (delta < -half)
    {
        delta += (int32_t)AS5600_COUNTS_PER_REV; /* passou por zero no sentido positivo */
        h->turns++;
    }

    h->last_raw = raw;

    if (dt_s > 0.0f)
        h->velocity = as5600_counts_to_rad(delta) / dt_s;

    return AS5600_OK;
}

as5600_err_t as5600_reset_tracking(as5600_t *h)
{
    if (h == NULL)
        return AS5600_ERR_INVALID_ARG;

    h->tracking = false;
    h->last_raw = 0;
    h->turns = 0;
    h->velocity = 0.0f;
    return AS5600_OK;
}

float as5600_get_angle_rad(const as5600_t *h)
{
    return (h == NULL) ? 0.0f : as5600_counts_to_rad(h->last_raw);
}

uint16_t as5600_get_raw(const as5600_t *h)
{
    return (h == NULL) ? 0U : h->last_raw;
}

float as5600_get_position_rad(const as5600_t *h)
{
    if (h == NULL)
        return 0.0f;
    return (float)h->turns * 6.28318530718f + as5600_counts_to_rad(h->last_raw);
}

int32_t as5600_get_turns(const as5600_t *h)
{
    return (h == NULL) ? 0 : h->turns;
}

float as5600_get_velocity_rad_s(const as5600_t *h)
{
    return (h == NULL) ? 0.0f : h->velocity;
}
