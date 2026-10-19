#include "pid.h"

pid_err_t pid_clear(pid *_pid)
{
    if (_pid == NULL)
        return PID_ERR_INVALID_ARG;

    _pid->P = 0.0f;
    _pid->I = 0.0f;
    _pid->D = 0.0f;

    _pid->input_anterior = 0.0f;
    _pid->has_prev = 0;

    _pid->output = 0.0f;

    /* saturation, kp, ki e kd são configuração: não são zerados */

    return PID_OK;
}

pid_err_t pid_control(pid *_pid, float setpoint,
                      float input, float dt)
{
    if (_pid == NULL || dt <= 0.0f)
        return PID_ERR_INVALID_ARG;

    /* Limites da saída e do integrador: assimétricos se configurados, senão ±saturation */
    float lo;
    float hi;
    if (_pid->out_max > _pid->out_min)
    {
        lo = _pid->out_min;
        hi = _pid->out_max;
    }
    else
    {
        if (_pid->saturation < 0.0f)
            return PID_ERR_INVALID_ARG;
        lo = -_pid->saturation;
        hi = _pid->saturation;
    }

    const float erro = setpoint - input;

    /* Primeira amostra: sem histórico, evita pico no termo derivativo */
    if (!_pid->has_prev)
    {
        _pid->input_anterior = input;
        _pid->has_prev = 1;
    }

    /*
     * Termo proporcional
     */
    _pid->P = _pid->kp * erro;

    /*
     * Termo derivativo sobre a medida (sem derivative kick)
     */
    _pid->D = -_pid->kd *
              (input - _pid->input_anterior) /
              dt;

    _pid->input_anterior = input;

    /*
     * Saída completa com o integrador candidato
     */
    float I_novo = _pid->I + _pid->ki * erro * dt;
    float out = _pid->P + I_novo + _pid->D;

    /*
     * Saturação com anti-windup: se a saída saturar no sentido do erro,
     * o integrador não avança.
     */
    if (out > hi)
    {
        out = hi;
        if (erro > 0.0f)
            I_novo = _pid->I;
    }
    else if (out < lo)
    {
        out = lo;
        if (erro < 0.0f)
            I_novo = _pid->I;
    }

    /*
     * Limita o próprio integrador
     */
    if (I_novo > hi)
        I_novo = hi;
    else if (I_novo < lo)
        I_novo = lo;

    _pid->I = I_novo;
    _pid->output = out;

    return PID_OK;
}