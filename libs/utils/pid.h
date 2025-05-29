#pragma once

typedef struct {
    float Kp;        // 比例系数
    float Ki;        // 积分系数
    float Kd;        // 微分系数
    float setpoint;  // 目标值
    float lastError; // 上一次误差值
    float integral;  // 积分累积项
    float output;    // 当前输出值
} PID;

float pid_compute(PID *pid, float measurement, float dt);