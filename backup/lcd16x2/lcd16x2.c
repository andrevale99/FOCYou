#include "lcd16x2.h"

/**
 * @brief Gera um pulso no pino Enable do LCD.
 *
 * Esta função realiza o acionamento do sinal Enable (`EN`) do display,
 * permitindo que o LCD capture os dados presentes nas linhas de comunicação.
 * O pulso respeita pequenos atrasos para garantir a sincronização do
 * controlador LCD.
 *
 * @param handle Ponteiro para a estrutura de controle do LCD.
 */
static void pulse_enable(const lcd16x2_handle *handle)
{
    handle->delay(1);

    handle->en.write(1);
    handle->delay(1);
    handle->en.write(0);

    handle->delay(1);
}

int8_t lcd16x2_init_4bits(const lcd16x2_handle *handle, void (*init_func)(void))
{
    if(!init_func)
        return -1;

    if (!(handle->delay) || !handle)
        return -2;

    init_func();

    handle->delay(100);

    handle->rs.write(0);

    /* 0x3 */
    handle->d4.write(1);
    handle->d5.write(1);
    handle->d6.write(0);
    handle->d7.write(0);
    pulse_enable(handle);
    handle->delay(1);

    /* 0x3 */
    pulse_enable(handle);
    handle->delay(1);

    /* 0x3 */
    pulse_enable(handle);
    handle->delay(1);

    /* 0x2 → 4 bits */
    handle->d4.write(0);
    handle->d5.write(1);
    handle->d6.write(0);
    handle->d7.write(0);
    pulse_enable(handle);
    handle->delay(1);

    lcd16x2_send_cmd(handle, BITS_4 | LINES_2);
    lcd16x2_send_cmd(handle, DISPLAY_OFF);

    lcd16x2_send_cmd(handle, CLEAR_DISPLAY);
    handle->delay(1);

    lcd16x2_send_cmd(handle, INCREMENT); // Entry mode set

    lcd16x2_send_cmd(handle, DISPLAY_ON | BLINK_CURSOR);

    return 0;
}

void lcd16x2_send_cmd(const lcd16x2_handle *handle, uint8_t cmd)
{
    /* RS = 0 para comando */
    handle->rs.write(0);

    handle->d4.write((cmd >> 4) & 0x1);
    handle->d5.write((cmd >> 5) & 0x1);
    handle->d6.write((cmd >> 6) & 0x1);
    handle->d7.write((cmd >> 7) & 0x1);

    pulse_enable(handle);

    handle->d4.write((cmd >> 0) & 0x1);
    handle->d5.write((cmd >> 1) & 0x1);
    handle->d6.write((cmd >> 2) & 0x1);
    handle->d7.write((cmd >> 3) & 0x1);

    pulse_enable(handle);

    handle->delay(1);
}

void lcd16x2_send_data(const lcd16x2_handle *handle, uint8_t data)
{
    /* RS = 1 para dado */
    handle->rs.write(1);

    handle->d4.write((data >> 4) & 0x1);
    handle->d5.write((data >> 5) & 0x1);
    handle->d6.write((data >> 6) & 0x1);
    handle->d7.write((data >> 7) & 0x1);

    pulse_enable(handle);

    handle->d4.write((data >> 0) & 0x1);
    handle->d5.write((data >> 1) & 0x1);
    handle->d6.write((data >> 2) & 0x1);
    handle->d7.write((data >> 3) & 0x1);

    pulse_enable(handle);

    handle->delay(1); // pode reduzir depois
}

void lcd16x2_write_string(const lcd16x2_handle *handle, const char *str, uint8_t size)
{

    for (uint8_t idx = 0; idx < size; ++idx)
        lcd16x2_send_data(handle, str[idx]);
}