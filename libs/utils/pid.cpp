#include "pid.hpp"

float PID::operator()(float measurement, float dt) {
    float error =set_point - measurement;
    integral += error * dt;

    // 积分限幅，防止积分风up
    if(integral > 1000.0f) integral = 1000.0f;
    if(integral < -1000.0f) integral = -1000.0f;

    float derivative = (error - last_error) / dt;
    output = kp * error + ki * integral + kd * derivative;
    last_error = error;
    return output;
}
