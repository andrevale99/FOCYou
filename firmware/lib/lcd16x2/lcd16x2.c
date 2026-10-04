#include "lcd16x2.h"

/**
 * @brief Valida o handle e todos os callbacks usados pelo driver.
 *
 * @param handle Ponteiro para a estrutura de controle do LCD.
 *
 * @retval LCD_OK                 Handle e callbacks válidos.
 * @retval LCD_ERR_NULL_HANDLE    @p handle é NULL.
 * @retval LCD_ERR_NULL_CALLBACK  Algum callback de pino ou @c delay_ms é NULL.
 */
static lcd16x2_err_t validate_handle(const lcd16x2_handle_t *handle)
{
    if (handle == NULL)
        return LCD_ERR_NULL_HANDLE;

    if (handle->d4.write == NULL || handle->d5.write == NULL ||
        handle->d6.write == NULL || handle->d7.write == NULL ||
        handle->en.write == NULL || handle->rs.write == NULL ||
        handle->delay_ms == NULL)
        return LCD_ERR_NULL_CALLBACK;

    return LCD_OK;
}

/**
 * @brief Gera um pulso no pino Enable do LCD.
 *
 * Esta função realiza o acionamento do sinal Enable (`EN`) do display,
 * permitindo que o LCD capture os dados presentes nas linhas de comunicação.
 * O pulso respeita pequenos atrasos para garantir a sincronização do
 * controlador LCD.
 *
 * @param handle Ponteiro (já validado) para a estrutura de controle do LCD.
 */
static void pulse_enable(const lcd16x2_handle_t *handle)
{
    handle->delay_ms(1);

    handle->en.write(1);
    handle->delay_ms(1);
    handle->en.write(0);

    handle->delay_ms(1);
}

/**
 * @brief Coloca um nibble (4 bits) nas linhas D4..D7.
 *
 * @param handle Ponteiro (já validado) para a estrutura de controle do LCD.
 * @param nibble Valor de 4 bits (bit 0 = D4 ... bit 3 = D7).
 */
static void write_nibble(const lcd16x2_handle_t *handle, uint8_t nibble)
{
    handle->d4.write((nibble >> 0) & 0x1);
    handle->d5.write((nibble >> 1) & 0x1);
    handle->d6.write((nibble >> 2) & 0x1);
    handle->d7.write((nibble >> 3) & 0x1);
}

/**
 * @brief Envia um byte em dois nibbles (mais significativo primeiro).
 *
 * @param handle Ponteiro (já validado) para a estrutura de controle do LCD.
 * @param is_data 1 = dado (RS = 1); 0 = comando (RS = 0).
 * @param value Byte a ser enviado.
 */
static void send_byte(const lcd16x2_handle_t *handle, uint8_t is_data, uint8_t value)
{
    handle->rs.write(is_data);

    write_nibble(handle, value >> 4);
    pulse_enable(handle);

    write_nibble(handle, value & 0x0F);
    pulse_enable(handle);

    handle->delay_ms(1);
}

lcd16x2_err_t lcd16x2_init_4bits(const lcd16x2_handle_t *handle, void (*init_func)(void))
{
    lcd16x2_err_t err = validate_handle(handle);
    if (err != LCD_OK)
        return err;

    if (init_func == NULL)
        return LCD_ERR_NULL_CALLBACK;

    init_func();

    handle->delay_ms(50);

    handle->rs.write(0);

    /* 0x3 */
    write_nibble(handle, 0x3);
    pulse_enable(handle);
    handle->delay_ms(1);

    /* 0x3 */
    pulse_enable(handle);
    handle->delay_ms(1);

    /* 0x3 */
    pulse_enable(handle);
    handle->delay_ms(1);

    /* 0x2 → 4 bits */
    write_nibble(handle, 0x2);
    pulse_enable(handle);
    handle->delay_ms(1);

    send_byte(handle, 0, BITS_4 | LINES_2);
    send_byte(handle, 0, DISPLAY_OFF);

    send_byte(handle, 0, CLEAR_DISPLAY);
    handle->delay_ms(1);

    send_byte(handle, 0, INCREMENT); // Entry mode set

    send_byte(handle, 0, DISPLAY_ON | BLINK_CURSOR);

    return LCD_OK;
}

lcd16x2_err_t lcd16x2_send_cmd(const lcd16x2_handle_t *handle, uint8_t cmd)
{
    lcd16x2_err_t err = validate_handle(handle);
    if (err != LCD_OK)
        return err;

    /* RS = 0 para comando */
    send_byte(handle, 0, cmd);

    return LCD_OK;
}

lcd16x2_err_t lcd16x2_send_data(const lcd16x2_handle_t *handle, uint8_t data)
{
    lcd16x2_err_t err = validate_handle(handle);
    if (err != LCD_OK)
        return err;

    /* RS = 1 para dado */
    send_byte(handle, 1, data);

    return LCD_OK;
}

lcd16x2_err_t lcd16x2_write_string(const lcd16x2_handle_t *handle, const char *str, uint8_t size)
{
    lcd16x2_err_t err = validate_handle(handle);
    if (err != LCD_OK)
        return err;

    if (str == NULL)
        return LCD_ERR_NULL_STRING;

    for (uint8_t idx = 0; idx < size; ++idx)
        send_byte(handle, 1, (uint8_t)str[idx]);

    return LCD_OK;
}