#include "pid.h"

pid_err_t pid_clear(pid *_pid)
{
    if (_pid == NULL)
        return PID_ERR_INVALID_ARG;

    _pid->P = 0.0f;
    _pid->I = 0.0f;
    _pid->D = 0.0f;

    _pid->erro_anterior = 0.0f;

    _pid->output = 0.0f;

    _pid->saturation = 0;

    return PID_OK;
}

pid_err_t pid_control(pid *_pid, float setpoint,
                      float input, float dt)
{
    if (_pid == NULL)
        return PID_ERR_INVALID_ARG;

    float erro = setpoint - input;

    /*
     * Termo proporcional
     */
    _pid->P = _pid->kp * erro;

    /*
     * Termo derivativo
     */
    _pid->D = _pid->kd *
             (erro - _pid->erro_anterior) /
             dt;

    /*
     * Atualiza o erro anterior
     */
    _pid->erro_anterior = erro;

    /*
     * Calcula uma saída sem o integrador
     */
    float output_sem_I = _pid->P + _pid->D;

    /*
     * Integra somente se a saída não estiver
     * saturada na direção do erro.
     */
    if ((output_sem_I < _pid->saturation && erro > 0.0f) ||
        (output_sem_I > -_pid->saturation && erro < 0.0f))
    {
        _pid->I += _pid->ki * erro * dt;
    }

    /*
     * Saída do _pid
     */
    _pid->output = _pid->P +
                  _pid->I +
                  _pid->D;

    /*
     * Saturação
     */
    if (_pid->output > _pid->saturation)
    {
        _pid->output = _pid->saturation;
    }
    else if (_pid->output < -_pid->saturation)
    {
        _pid->output = -_pid->saturation;
    }

    return PID_OK;
}