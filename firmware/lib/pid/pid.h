#ifndef PID_H
#define PID_H

#include <stddef.h>
#include <stdint.h>

typedef enum
{
    PID_OK = 0,
    PID_ERR_INVALID_ARG = -1,
} pid_err_t;

typedef struct
{
    float kp;
    float ki;
    float kd;

    float P;
    float I;
    float D;

    float input_anterior; /* derivada calculada sobre a medida (sem derivative kick) */
    uint8_t has_prev;     /* 0 até a primeira amostra: evita pico do D no início */

    float saturation;     /* limite simétrico da saída e do integrador (>= 0) */

    /*
     * Limites assimétricos (opcionais): com out_max > out_min, substituem o
     * ±saturation na saída e no integrador. Com os dois em 0 (struct zerada),
     * vale a saturação simétrica.
     */
    float out_min;
    float out_max;

    float output;
} pid;

/**
 * @brief Zera o estado do controlador (P, I, D, saída e histórico).
 * @note kp, ki, kd, saturation, out_min e out_max são configuração e não são alterados.
 */
pid_err_t pid_clear(pid *_pid);

/**
 * @brief Executa um passo do controlador PID.
 * @param dt Período de amostragem em segundos (> 0).
 * @retval PID_ERR_INVALID_ARG ponteiro NULL, dt <= 0 ou, sem out_min/out_max, saturation < 0.
 */
pid_err_t pid_control(pid *_pid, float setpoint,
                      float input, float dt);

#endif