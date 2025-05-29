#pragma once

typedef struct
{
    float Kp;       // 比例系数
    float Ki;       // 积分系数
    float Kd;       // 微分系数
    float target;   // 目标值
    float error;    // 误差
    float integral; // 积分累积
    float output;   // 输出
} PID;

float pid_calc(PID *pid,float measurement,float dt);