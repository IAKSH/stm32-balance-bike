#include "data_proc.h"
#include "stdlib.h"
#include "stdio.h"
#include "wireless.h"

#define ADC_OFFSET(x) (((x) > 0) ? (10) : (-10))
#define GYRO_OFFSET(x)  (((x) > 0) ? (10) : (-10))

#define ADC_CENTER 2048
#define ADC_DEAD_ZONE 200 // 死区，避免轻微抖动

#define GYRO_DEAD_ZERO 5

#define MAX_SPEED 7200

extern int thb001p_adc_value[4];
extern float pitch, roll, yaw;

void motor_speed_calc(int *motor_left_speed,int *motor_right_speed)
{
    // 电机转速处理
    int motor_speed_x = thb001p_adc_value[0] - ADC_CENTER;
    int motor_speed_y = thb001p_adc_value[1] - ADC_CENTER;

    if (abs(motor_speed_x) < ADC_DEAD_ZONE)
        motor_speed_x = 0;
    if (abs(motor_speed_y) < ADC_DEAD_ZONE)
        motor_speed_y = 0;

    int speed = motor_speed_y * MAX_SPEED / (4095 / 2);
    int turn = motor_speed_x * MAX_SPEED / (4095 / 2);
    printf("speed : %d\n", speed);
    *motor_left_speed = speed + turn;
    *motor_right_speed = speed - turn;

    if (*motor_left_speed > MAX_SPEED)
        *motor_left_speed = MAX_SPEED;
    if (*motor_left_speed < -MAX_SPEED)
        *motor_left_speed = -MAX_SPEED;
    if (*motor_right_speed > MAX_SPEED)
        *motor_right_speed = MAX_SPEED;
    if (*motor_right_speed < -MAX_SPEED)
        *motor_right_speed = -MAX_SPEED;

    float norm_left = (float)*motor_left_speed / MAX_SPEED;
    float norm_right = (float)*motor_right_speed / MAX_SPEED;

    *motor_left_speed = (int)(norm_left * norm_left * norm_left * MAX_SPEED);
    *motor_right_speed = (int)(norm_right * norm_right * norm_right * MAX_SPEED);

    printf("motor_speed: %d  %d\n", *motor_left_speed, *motor_right_speed);
}

void gimbal_angle_calc(float *angle_buttom_offset,float *angle_top_offset)
{
    int gimbal_angle_x = thb001p_adc_value[2] - ADC_CENTER;
    int gimbal_angle_y = thb001p_adc_value[3] - ADC_CENTER;

    if (abs(gimbal_angle_x) < ADC_DEAD_ZONE)
        gimbal_angle_x = 0;
    if (abs(gimbal_angle_y) < ADC_DEAD_ZONE)
        gimbal_angle_y = 0;

    float gimbal_pitch = pitch;
    float gimbal_roll = roll;

    if (abs(gimbal_pitch) < GYRO_DEAD_ZERO)
        gimbal_pitch = 0;
    if (abs(gimbal_roll) < GYRO_DEAD_ZERO)
        gimbal_roll = roll;

    *angle_buttom_offset = ADC_OFFSET(gimbal_angle_x)+GYRO_OFFSET(gimbal_roll);
    *angle_top_offset = ADC_OFFSET(gimbal_angle_y)+GYRO_OFFSET(gimbal_pitch);
}

void data_packaing(CommandPacket *command)
{
    motor_speed_calc(&command->payload.move.speed[0], &command->payload.move.speed[1]);
    gimbal_angle_calc(&command->payload.cam_rotate.angle[0], &command->payload.cam_rotate.angle[1]);
}