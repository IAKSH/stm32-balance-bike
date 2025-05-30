#include "data_proc.h"
#include "stdlib.h"
#include "stdio.h"
#include "wireless.h"

#define ADC_OFFSET(x) (((x) > 0) ? (10) : (-10))
#define GYRO_OFFSET(x) (((x) > 0) ? (10) : (-10))

#define ADC_CENTER 2048
#define ADC_DEAD_ZONE 200 // 死区，避免轻微抖动

#define GYRO_DEAD_ZERO 5

#define MAX_SPEED 7200

extern int thb001p_adc_value[4];
extern float mpu6050_pitch,mpu_6050_roll,mpu_6050_yaw;

void motor_speed_calc(int *motor_left_speed, int *motor_right_speed)
{
    int speed_left,speed_right;

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
    speed_left = speed + turn;
    speed_right = speed - turn;

    if (speed_left > MAX_SPEED)
        speed_left = MAX_SPEED;
    if (speed_left < -MAX_SPEED)
        speed_left = -MAX_SPEED;
    if (speed_right > MAX_SPEED)
        speed_right = MAX_SPEED;
    if (speed_right < -MAX_SPEED)
        speed_right = -MAX_SPEED;

    float norm_left = (float)speed_left / MAX_SPEED;
    float norm_right = (float)speed_right / MAX_SPEED;

    speed_left = (int)(norm_left * norm_left * MAX_SPEED);
    speed_right = (int)(norm_right * norm_right * MAX_SPEED);

    printf("motor_speed: %d  %d\n", speed_left, speed_right);

    *motor_left_speed = speed_left;
    *motor_right_speed = speed_right;
}

void gimbal_angle_calc(float *angle_buttom_offset, float *angle_top_offset)
{
    int gimbal_angle_x = thb001p_adc_value[2] - ADC_CENTER;
    int gimbal_angle_y = thb001p_adc_value[3] - ADC_CENTER;

    if (abs(gimbal_angle_x) < ADC_DEAD_ZONE)
        gimbal_angle_x = 0;
    if (abs(gimbal_angle_y) < ADC_DEAD_ZONE)
        gimbal_angle_y = 0;

    float gimbal_pitch = mpu6050_pitch;
    float gimbal_roll = mpu_6050_roll;

    if (abs(gimbal_pitch) < GYRO_DEAD_ZERO)
        gimbal_pitch = 0;
    if (abs(gimbal_roll) < GYRO_DEAD_ZERO)
        gimbal_roll = mpu_6050_roll;

    *angle_buttom_offset = ADC_OFFSET(gimbal_angle_x) + GYRO_OFFSET(gimbal_roll);
    *angle_top_offset = ADC_OFFSET(gimbal_angle_y) + GYRO_OFFSET(gimbal_pitch);
}

void data_packaing(CommandPacket *command)
{
    switch (command->type)
    {
    case COMMAND_MOVE:
        motor_speed_calc(&command->move.speed[0], &command->move.speed[1]);
        break;
    
    case COMMAND_CAM_ROTATE:
        // gimbal_angle_calc(&command->cam_rotate.angle[0], &command->.cam_rotate.angle[1]);
        break;

    default:
        break;
    }
}