#include "pid.h"

float pid_calc(PID *pid, float measurement, float dt)
{
    float current_error = pid->target - measurement;
    pid->integral = current_error * dt;

    // 积分限幅
    if (pid->integral > 1000.0f)
        pid->integral = 1000.0f;
    if (pid->integral < -1000.0f)
        pid->integral = -1000.0f;
    
    float derivative = (current_error-pid->error)/dt;

    pid->output = pid->Kp*current_error + pid->Ki * pid->integral + pid->Kd * derivative;

    pid->error = current_error;

    return pid->output;
}