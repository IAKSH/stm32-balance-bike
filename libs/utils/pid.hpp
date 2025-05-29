#pragma once

struct PID {
    float kp,ki,kd;
    float set_point;
    float last_error;
    float integral;
    float output;

    float operator()(float measurement,float dt);
};