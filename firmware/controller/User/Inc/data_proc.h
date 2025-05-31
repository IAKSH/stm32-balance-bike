#pragma once

#include "main.h"
#include "wireless.h"

void gimbal_angle_calc(float *angle_buttom_offset,float *angle_top_offset);
void motor_speed_calc(int16_t *motor_left_speed,int16_t *motor_right_speed);
void data_packaing(CommandPacket *command);