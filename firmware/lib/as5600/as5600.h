/**
 * @file as5600.h
 * @brief Driver do encoder magnético AS5600 (12 bits, I2C) para o STM32F411.
 *
 * Usa o driver I2C de @c src/drivers/i2c. O barramento é inicializado pela
 * aplicação e pode ser compartilhado com outros dispositivos; a biblioteca só
 * guarda o ponteiro para o handle.
 *
 * Características do sensor (datasheet ams AS5600 v1-00):
 * - endereço I2C fixo 0x36, até 1 MHz (o driver I2C vai até 400 kHz);
 * - sem clock stretching;
 * - 4096 contagens por volta; o sentido de contagem é definido pelo pino DIR
 *   (GND = cresce no sentido horário, VDD = anti-horário);
 * - power-up de até 10 ms: chame as5600_init() depois disso.
 *
 * Para o controle de motor, use as5600_update() a uma taxa fixa: ela lê o
 * RAW ANGLE, conta as voltas e calcula a velocidade.
 *
 * Exemplo (I2C1 já inicializado, laço de 1 kHz):
 * @code
 * static as5600_t enc;
 *
 * const as5600_config_t cfg = {
 *     .i2c = &i2c,
 *     .delay_ms = NULL,          // obrigatório só para os comandos de OTP
 * };
 *
 * if (as5600_init(&enc, &cfg) != AS5600_OK) { ... }
 * if (as5600_check_magnet(&enc) != AS5600_OK) { ... }   // ímã ausente ou fora da faixa
 *
 * for (;;)
 * {
 *     if (as5600_update(&enc, 0.001f) == AS5600_OK)
 *     {
 *         float pos = as5600_get_position_rad(&enc);   // várias voltas
 *         float vel = as5600_get_velocity_rad_s(&enc);
 *     }
 * }
 * @endcode
 */

#ifndef AS5600_H
#define AS5600_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "drivers/driver_err.h"
#include "drivers/i2c/i2c.h"

/** @brief Endereço I2C de 7 bits (fixo). */
#define AS5600_I2C_ADDR 0x36U

/** @brief Contagens por volta (12 bits). */
#define AS5600_COUNTS_PER_REV 4096U

/** @brief Maior valor de um registrador de 12 bits. */
#define AS5600_MAX_COUNT 0x0FFFU

/** @brief Faixa angular mínima (18°) em contagens: 18 / 360 · 4096 = 204,8. */
#define AS5600_MIN_RANGE 205U

/** @brief Quantas vezes o BURN_ANGLE pode ser executado. */
#define AS5600_MAX_BURN_ANGLE 3U

/** @brief Espera após cada comando de OTP, em ms (o datasheet pede pelo menos 1 ms). */
#ifndef AS5600_CMD_DELAY_MS
#define AS5600_CMD_DELAY_MS 2U
#endif

/** @brief Mapa de registradores. */
typedef enum
{
    AS5600_REG_ZMCO = 0x00,      /**< Quantas vezes ZPOS/MPOS foram gravados (2 bits). */
    AS5600_REG_ZPOS = 0x01,      /**< Posição inicial (12 bits, 0x01-0x02). */
    AS5600_REG_MPOS = 0x03,      /**< Posição final (12 bits, 0x03-0x04). */
    AS5600_REG_MANG = 0x05,      /**< Faixa angular máxima (12 bits, 0x05-0x06). */
    AS5600_REG_CONF = 0x07,      /**< Configuração (14 bits, 0x07-0x08). */
    AS5600_REG_STATUS = 0x0B,    /**< MD, ML, MH. */
    AS5600_REG_RAW_ANGLE = 0x0C, /**< Ângulo sem escala (12 bits, 0x0C-0x0D). */
    AS5600_REG_ANGLE = 0x0E,     /**< Ângulo com escala ZPOS/MPOS/MANG (12 bits, 0x0E-0x0F). */
    AS5600_REG_AGC = 0x1A,       /**< Ganho do AGC (8 bits). */
    AS5600_REG_MAGNITUDE = 0x1B, /**< Magnitude do CORDIC (12 bits, 0x1B-0x1C). */
    AS5600_REG_BURN = 0xFF,      /**< Comandos de OTP. */
} as5600_reg_t;

/** @brief Códigos de retorno do driver AS5600. */
typedef enum
{
    AS5600_OK = 0,                     /**< Operação executada com sucesso. */
    AS5600_ERR_INVALID_ARG = -1,       /**< Ponteiro NULL ou parâmetro inválido. */
    AS5600_ERR_NOT_INITIALIZED = -2,   /**< as5600_init() ainda não foi chamado. */
    AS5600_ERR_I2C = -3,               /**< Falha no barramento; detalhe em as5600_get_i2c_error(). */
    AS5600_ERR_NOT_FOUND = -4,         /**< Nenhum ACK no endereço 0x36. */
    AS5600_ERR_NO_MAGNET = -5,         /**< Ímã não detectado (MD = 0). */
    AS5600_ERR_MAGNET_TOO_WEAK = -6,   /**< AGC no ganho máximo (ML = 1): aproxime o ímã. */
    AS5600_ERR_MAGNET_TOO_STRONG = -7, /**< AGC no ganho mínimo (MH = 1): afaste o ímã. */
    AS5600_ERR_OUT_OF_RANGE = -8,      /**< Valor acima de 12 bits ou faixa menor que 18°. */
    AS5600_ERR_BURN_LIMIT = -9,        /**< OTP esgotado ou bloqueado (ZMCO). */
    AS5600_ERR_NO_DELAY = -10,         /**< delay_ms é NULL, mas a operação precisa dele. */
    AS5600_ERR_VERIFY = -11,           /**< O conteúdo do OTP difere do valor gravado. */
} as5600_err_t;

/** @brief Modo de consumo (CONF.PM). */
typedef enum
{
    AS5600_PM_NOM = 0,  /**< Sempre ligado (6,5 mA). */
    AS5600_PM_LPM1 = 1, /**< Polling de 5 ms (3,4 mA). */
    AS5600_PM_LPM2 = 2, /**< Polling de 20 ms (1,8 mA). */
    AS5600_PM_LPM3 = 3, /**< Polling de 100 ms (1,5 mA). */
} as5600_power_mode_t;

/** @brief Histerese da saída (CONF.HYST). */
typedef enum
{
    AS5600_HYST_OFF = 0,  /**< Sem histerese. */
    AS5600_HYST_1LSB = 1, /**< 1 LSB. */
    AS5600_HYST_2LSB = 2, /**< 2 LSBs. */
    AS5600_HYST_3LSB = 3, /**< 3 LSBs. */
} as5600_hysteresis_t;

/** @brief Estágio de saída do pino OUT (CONF.OUTS). */
typedef enum
{
    AS5600_OUTS_ANALOG_FULL = 0,    /**< Analógica, de 0% a 100% de VDD. */
    AS5600_OUTS_ANALOG_REDUCED = 1, /**< Analógica, de 10% a 90% de VDD. */
    AS5600_OUTS_PWM = 2,            /**< PWM digital (o DAC é desligado). */
} as5600_output_stage_t;

/** @brief Frequência do PWM do pino OUT (CONF.PWMF). */
typedef enum
{
    AS5600_PWMF_115HZ = 0, /**< 115 Hz. */
    AS5600_PWMF_230HZ = 1, /**< 230 Hz. */
    AS5600_PWMF_460HZ = 2, /**< 460 Hz. */
    AS5600_PWMF_920HZ = 3, /**< 920 Hz. */
} as5600_pwm_freq_t;

/** @brief Filtro lento (CONF.SF): atraso da resposta ao degrau x ruído RMS. */
typedef enum
{
    AS5600_SF_16X = 0, /**< 2,2 ms; 0,015°. Forçado nos modos LPM. */
    AS5600_SF_8X = 1,  /**< 1,1 ms; 0,021°. */
    AS5600_SF_4X = 2,  /**< 0,55 ms; 0,030°. */
    AS5600_SF_2X = 3,  /**< 0,286 ms; 0,043°. */
} as5600_slow_filter_t;

/** @brief Limiar do filtro rápido (CONF.FTH), em LSBs para passar do lento ao rápido. */
typedef enum
{
    AS5600_FTH_SLOW_ONLY = 0, /**< Só o filtro lento. */
    AS5600_FTH_6LSB = 1,      /**< 6 LSBs. */
    AS5600_FTH_7LSB = 2,      /**< 7 LSBs. */
    AS5600_FTH_9LSB = 3,      /**< 9 LSBs. */
    AS5600_FTH_18LSB = 4,     /**< 18 LSBs. */
    AS5600_FTH_21LSB = 5,     /**< 21 LSBs. */
    AS5600_FTH_24LSB = 6,     /**< 24 LSBs. */
    AS5600_FTH_10LSB = 7,     /**< 10 LSBs. */
} as5600_fast_filter_t;

/** @brief Campos do registrador CONF. */
typedef struct
{
    as5600_power_mode_t power_mode;     /**< PM. */
    as5600_hysteresis_t hysteresis;     /**< HYST. */
    as5600_output_stage_t output_stage; /**< OUTS. */
    as5600_pwm_freq_t pwm_freq;         /**< PWMF. */
    as5600_slow_filter_t slow_filter;   /**< SF. */
    as5600_fast_filter_t fast_filter;   /**< FTH. */
    bool watchdog;                      /**< WD: vai para LPM3 após 1 min parado (±4 LSB). */
} as5600_conf_t;

/** @brief Registrador STATUS. */
typedef struct
{
    bool magnet_detected;   /**< MD: ímã detectado. */
    bool magnet_too_weak;   /**< ML: AGC no ganho máximo. */
    bool magnet_too_strong; /**< MH: AGC no ganho mínimo. */
} as5600_status_t;

/** @brief Configuração do driver. */
typedef struct
{
    i2c_t *i2c;                 /**< Barramento já inicializado com i2c_init(). */
    void (*delay_ms)(uint32_t); /**< Atraso em ms; obrigatório só para os comandos de OTP. */
} as5600_config_t;

/**
 * @brief Handle do driver.
 * @note Os campos são de uso interno; não os modifique diretamente.
 */
typedef struct
{
    as5600_config_t cfg;  /**< Cópia da configuração. */
    driver_err_t i2c_err; /**< Último código do driver I2C. */
    bool initialized;     /**< true após as5600_init() bem-sucedido. */

    /* Acompanhamento de posição (as5600_update) */
    bool tracking;     /**< true após a primeira leitura de as5600_update(). */
    uint16_t last_raw; /**< Último RAW ANGLE lido. */
    int32_t turns;     /**< Voltas completas acumuladas. */
    float velocity;    /**< Velocidade da última atualização, em rad/s. */
} as5600_t;

/* ------------------------------------------------------------------ */
/* Ciclo de vida                                                       */
/* ------------------------------------------------------------------ */

/**
 * @brief Associa o handle ao barramento e verifica se o sensor responde.
 *
 * Não escreve nenhum registrador: o sensor continua com a configuração
 * que tinha (OTP ou valores padrão).
 *
 * @param[out] h   Handle a ser preenchido.
 * @param[in]  cfg Configuração desejada.
 *
 * @retval AS5600_OK             Sucesso.
 * @retval AS5600_ERR_INVALID_ARG NULL em @p h, @p cfg ou @c cfg->i2c.
 * @retval AS5600_ERR_NOT_FOUND  Sem ACK no endereço 0x36.
 * @retval AS5600_ERR_I2C        Outra falha no barramento.
 */
as5600_err_t as5600_init(as5600_t *h, const as5600_config_t *cfg);

/**
 * @brief Último código retornado pelo driver I2C (útil após #AS5600_ERR_I2C).
 * @param[in] h Handle do driver.
 * @return Código do driver I2C, ou DRIVER_ERR_INVALID_ARG se @p h for NULL.
 */
driver_err_t as5600_get_i2c_error(const as5600_t *h);

/* ------------------------------------------------------------------ */
/* Leitura de ângulo e diagnóstico                                     */
/* ------------------------------------------------------------------ */

/**
 * @brief Lê o RAW ANGLE: ângulo sem escala nem ZPOS/MPOS, de 0 a 4095.
 *
 * É o valor indicado para controle de motor, porque não depende da faixa programada.
 *
 * @param[in,out] h   Handle inicializado.
 * @param[out]    raw Ângulo em contagens.
 * @retval AS5600_OK Sucesso.
 * @return #AS5600_ERR_INVALID_ARG, #AS5600_ERR_NOT_INITIALIZED ou #AS5600_ERR_I2C.
 */
as5600_err_t as5600_read_raw_angle(as5600_t *h, uint16_t *raw);

/**
 * @brief Lê o ANGLE: ângulo com escala para a faixa ZPOS..MPOS (ou MANG), de 0 a 4095.
 *
 * Com a faixa padrão (360°) e ZPOS = 0, é igual ao RAW ANGLE, a menos de uma
 * histerese de 10 LSB perto de 0°/360°.
 *
 * @param[in,out] h     Handle inicializado.
 * @param[out]    angle Ângulo em contagens.
 * @retval AS5600_OK Sucesso.
 * @return #AS5600_ERR_INVALID_ARG, #AS5600_ERR_NOT_INITIALIZED ou #AS5600_ERR_I2C.
 */
as5600_err_t as5600_read_angle(as5600_t *h, uint16_t *angle);

/**
 * @brief Lê o registrador STATUS.
 * @param[in,out] h      Handle inicializado.
 * @param[out]    status Bits MD, ML e MH.
 * @retval AS5600_OK Sucesso.
 * @return #AS5600_ERR_INVALID_ARG, #AS5600_ERR_NOT_INITIALIZED ou #AS5600_ERR_I2C.
 */
as5600_err_t as5600_read_status(as5600_t *h, as5600_status_t *status);

/**
 * @brief Verifica se o ímã está presente e com intensidade adequada.
 * @param[in,out] h Handle inicializado.
 * @retval AS5600_OK                    Ímã detectado e dentro da faixa do AGC.
 * @retval AS5600_ERR_NO_MAGNET         MD = 0 (a saída é forçada a nível baixo).
 * @retval AS5600_ERR_MAGNET_TOO_WEAK   ML = 1.
 * @retval AS5600_ERR_MAGNET_TOO_STRONG MH = 1.
 * @return Também #AS5600_ERR_INVALID_ARG, #AS5600_ERR_NOT_INITIALIZED ou #AS5600_ERR_I2C.
 */
as5600_err_t as5600_check_magnet(as5600_t *h);

/**
 * @brief Lê o ganho do AGC (8 bits).
 *
 * Para a melhor robustez, ajuste a distância do ímã até o AGC ficar no meio da faixa.
 *
 * @param[in,out] h   Handle inicializado.
 * @param[out]    agc Ganho.
 * @retval AS5600_OK Sucesso.
 * @return #AS5600_ERR_INVALID_ARG, #AS5600_ERR_NOT_INITIALIZED ou #AS5600_ERR_I2C.
 */
as5600_err_t as5600_read_agc(as5600_t *h, uint8_t *agc);

/**
 * @brief Lê a magnitude do campo calculada pelo CORDIC (12 bits).
 * @param[in,out] h         Handle inicializado.
 * @param[out]    magnitude Magnitude.
 * @retval AS5600_OK Sucesso.
 * @return #AS5600_ERR_INVALID_ARG, #AS5600_ERR_NOT_INITIALIZED ou #AS5600_ERR_I2C.
 */
as5600_err_t as5600_read_magnitude(as5600_t *h, uint16_t *magnitude);

/* ------------------------------------------------------------------ */
/* Configuração (RAM; vale até o próximo power-up)                     */
/* ------------------------------------------------------------------ */

/**
 * @brief Lê o registrador CONF.
 * @param[in,out] h    Handle inicializado.
 * @param[out]    conf Campos decodificados.
 * @retval AS5600_OK Sucesso.
 * @return #AS5600_ERR_INVALID_ARG, #AS5600_ERR_NOT_INITIALIZED ou #AS5600_ERR_I2C.
 */
as5600_err_t as5600_get_conf(as5600_t *h, as5600_conf_t *conf);

/**
 * @brief Escreve o registrador CONF (preserva os bits 15:14, não usados).
 *
 * A nova configuração vale na saída pelo menos 1 ms depois.
 *
 * @param[in,out] h    Handle inicializado.
 * @param[in]     conf Campos desejados.
 * @retval AS5600_OK              Sucesso.
 * @retval AS5600_ERR_INVALID_ARG NULL ou campo fora da faixa.
 * @return Também #AS5600_ERR_NOT_INITIALIZED ou #AS5600_ERR_I2C.
 */
as5600_err_t as5600_set_conf(as5600_t *h, const as5600_conf_t *conf);

/**
 * @brief Lê ZPOS, a posição inicial da faixa.
 * @param[in,out] h    Handle inicializado.
 * @param[out]    zpos Posição em contagens do RAW ANGLE.
 * @retval AS5600_OK Sucesso.
 * @return #AS5600_ERR_INVALID_ARG, #AS5600_ERR_NOT_INITIALIZED ou #AS5600_ERR_I2C.
 */
as5600_err_t as5600_get_zpos(as5600_t *h, uint16_t *zpos);

/**
 * @brief Escreve ZPOS (normalmente o RAW ANGLE lido na posição inicial).
 * @param[in,out] h    Handle inicializado.
 * @param[in]     zpos 0 a 4095.
 * @retval AS5600_OK               Sucesso.
 * @retval AS5600_ERR_OUT_OF_RANGE @p zpos acima de 12 bits.
 * @return Também #AS5600_ERR_INVALID_ARG, #AS5600_ERR_NOT_INITIALIZED ou #AS5600_ERR_I2C.
 */
as5600_err_t as5600_set_zpos(as5600_t *h, uint16_t zpos);

/**
 * @brief Lê MPOS, a posição final da faixa.
 * @param[in,out] h    Handle inicializado.
 * @param[out]    mpos Posição em contagens do RAW ANGLE.
 * @retval AS5600_OK Sucesso.
 * @return #AS5600_ERR_INVALID_ARG, #AS5600_ERR_NOT_INITIALIZED ou #AS5600_ERR_I2C.
 */
as5600_err_t as5600_get_mpos(as5600_t *h, uint16_t *mpos);

/**
 * @brief Escreve MPOS (normalmente o RAW ANGLE lido na posição final).
 * @param[in,out] h    Handle inicializado.
 * @param[in]     mpos 0 a 4095; a distância a ZPOS deve ser maior que 18°.
 * @retval AS5600_OK               Sucesso.
 * @retval AS5600_ERR_OUT_OF_RANGE @p mpos acima de 12 bits.
 * @return Também #AS5600_ERR_INVALID_ARG, #AS5600_ERR_NOT_INITIALIZED ou #AS5600_ERR_I2C.
 */
as5600_err_t as5600_set_mpos(as5600_t *h, uint16_t mpos);

/**
 * @brief Lê MANG, a faixa angular máxima.
 * @param[in,out] h    Handle inicializado.
 * @param[out]    mang Faixa em contagens (0 = 360°).
 * @retval AS5600_OK Sucesso.
 * @return #AS5600_ERR_INVALID_ARG, #AS5600_ERR_NOT_INITIALIZED ou #AS5600_ERR_I2C.
 */
as5600_err_t as5600_get_mang(as5600_t *h, uint16_t *mang);

/**
 * @brief Escreve MANG (ex.: 90° = 0x400). Alternativa a MPOS.
 * @param[in,out] h    Handle inicializado.
 * @param[in]     mang #AS5600_MIN_RANGE a 4095, ou 0 para 360°.
 * @retval AS5600_OK               Sucesso.
 * @retval AS5600_ERR_OUT_OF_RANGE Acima de 12 bits ou abaixo de 18°.
 * @return Também #AS5600_ERR_INVALID_ARG, #AS5600_ERR_NOT_INITIALIZED ou #AS5600_ERR_I2C.
 */
as5600_err_t as5600_set_mang(as5600_t *h, uint16_t mang);

/* ------------------------------------------------------------------ */
/* Gravação permanente (OTP)                                           */
/* ------------------------------------------------------------------ */

/**
 * @brief Lê ZMCO: quantas vezes o BURN_ANGLE já foi executado (0 a 3).
 * @param[in,out] h     Handle inicializado.
 * @param[out]    count Contagem.
 * @retval AS5600_OK Sucesso.
 * @return #AS5600_ERR_INVALID_ARG, #AS5600_ERR_NOT_INITIALIZED ou #AS5600_ERR_I2C.
 */
as5600_err_t as5600_get_burn_count(as5600_t *h, uint8_t *count);

/**
 * @brief Grava ZPOS e MPOS no OTP (BURN_ANGLE, no máximo 3 vezes) e verifica.
 *
 * @warning Irreversível. Escreva ZPOS/MPOS antes com as5600_set_zpos() e
 * as5600_set_mpos(). Em 3,3 V a alimentação precisa estar entre 3,3 V e 3,5 V,
 * com 10 µF em VDD3V3, e a temperatura entre 20 °C e 30 °C.
 *
 * Sequência: confere MD = 1 e ZMCO < 3, grava, espera, recarrega o OTP
 * (comandos 0x01, 0x11, 0x10) e compara ZPOS/MPOS com o que foi escrito.
 * Confira de novo após um novo power-up.
 *
 * @param[in,out] h Handle inicializado (com @c delay_ms).
 * @retval AS5600_OK             Gravado e verificado.
 * @retval AS5600_ERR_NO_DELAY   @c delay_ms é NULL.
 * @retval AS5600_ERR_NO_MAGNET  MD = 0; nada foi gravado.
 * @retval AS5600_ERR_BURN_LIMIT ZMCO = 3; nada foi gravado.
 * @retval AS5600_ERR_VERIFY     O OTP lido difere do valor escrito.
 * @return Também #AS5600_ERR_INVALID_ARG, #AS5600_ERR_NOT_INITIALIZED ou #AS5600_ERR_I2C.
 */
as5600_err_t as5600_burn_angle(as5600_t *h);

/**
 * @brief Grava MANG e CONF no OTP (BURN_SETTING, uma única vez) e verifica.
 *
 * @warning Irreversível e só pode ser feito uma vez. As mesmas condições de
 * alimentação e temperatura de as5600_burn_angle() se aplicam.
 *
 * O datasheet só permite gravar MANG com ZMCO = 0; por isso a função recusa
 * a gravação se ZPOS/MPOS já foram gravados (siga a ordem da opção C:
 * primeiro BURN_SETTING, depois BURN_ANGLE).
 *
 * @param[in,out] h Handle inicializado (com @c delay_ms).
 * @retval AS5600_OK             Gravado e verificado.
 * @retval AS5600_ERR_NO_DELAY   @c delay_ms é NULL.
 * @retval AS5600_ERR_BURN_LIMIT ZMCO != 0; nada foi gravado.
 * @retval AS5600_ERR_VERIFY     O OTP lido difere do valor escrito.
 * @return Também #AS5600_ERR_INVALID_ARG, #AS5600_ERR_NOT_INITIALIZED ou #AS5600_ERR_I2C.
 */
as5600_err_t as5600_burn_setting(as5600_t *h);

/* ------------------------------------------------------------------ */
/* Acompanhamento de posição e velocidade                              */
/* ------------------------------------------------------------------ */

/**
 * @brief Lê o RAW ANGLE e atualiza voltas, posição e velocidade.
 *
 * Deve ser chamada a uma taxa em que o rotor gire menos de meia volta entre
 * duas chamadas (ex.: a 1 kHz, até 30.000 rpm). A primeira chamada só registra
 * o ângulo inicial (velocidade = 0).
 *
 * A velocidade é a diferença entre duas leituras dividida por @p dt_s, sem
 * filtro. Com 1 contagem = 1,53 mrad, a resolução a 1 kHz é de 1,53 rad/s:
 * filtre na aplicação, se precisar.
 *
 * @param[in,out] h    Handle inicializado.
 * @param[in]     dt_s Tempo desde a última chamada, em s (<= 0 mantém a velocidade anterior).
 * @retval AS5600_OK Sucesso.
 * @return #AS5600_ERR_INVALID_ARG, #AS5600_ERR_NOT_INITIALIZED ou #AS5600_ERR_I2C;
 *         em caso de erro, o estado não muda.
 */
as5600_err_t as5600_update(as5600_t *h, float dt_s);

/**
 * @brief Zera as voltas e a velocidade; a próxima as5600_update() reinicia o acompanhamento.
 * @param[in,out] h Handle do driver.
 * @retval AS5600_OK              Sucesso.
 * @retval AS5600_ERR_INVALID_ARG @p h é NULL.
 */
as5600_err_t as5600_reset_tracking(as5600_t *h);

/**
 * @brief Ângulo mecânico da última atualização, de 0 a 2π.
 * @param[in] h Handle do driver.
 * @return Ângulo em rad (0 se @p h for NULL ou sem atualização).
 */
float as5600_get_angle_rad(const as5600_t *h);

/**
 * @brief Posição acumulada (voltas · 2π + ângulo).
 *
 * Em @c float, a resolução de 1 contagem se perde depois de ~2000 voltas;
 * para posição absoluta longa, use as5600_get_turns() e as5600_get_angle_rad().
 *
 * @param[in] h Handle do driver.
 * @return Posição em rad (0 se @p h for NULL ou sem atualização).
 */
float as5600_get_position_rad(const as5600_t *h);

/**
 * @brief Voltas completas acumuladas desde a primeira atualização (com sinal).
 * @param[in] h Handle do driver.
 * @return Número de voltas (0 se @p h for NULL).
 */
int32_t as5600_get_turns(const as5600_t *h);

/**
 * @brief Velocidade da última atualização.
 * @param[in] h Handle do driver.
 * @return Velocidade em rad/s (0 se @p h for NULL).
 */
float as5600_get_velocity_rad_s(const as5600_t *h);

/* ------------------------------------------------------------------ */
/* Conversões                                                          */
/* ------------------------------------------------------------------ */

/** @brief Converte contagens em radianos. */
static inline float as5600_counts_to_rad(int32_t counts)
{
    return (float)counts * (6.28318530718f / (float)AS5600_COUNTS_PER_REV);
}

/** @brief Converte contagens em graus. */
static inline float as5600_counts_to_deg(int32_t counts)
{
    return (float)counts * (360.0f / (float)AS5600_COUNTS_PER_REV);
}

#endif /* AS5600_H */
