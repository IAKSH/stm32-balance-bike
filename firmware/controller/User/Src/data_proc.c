#include "data_proc.h"
#include "stdlib.h"
#include "stdio.h"

#define ADC_OFFSET(x) (((x) > 0) ? (10) : (-10))
#define GYRO_OFFSET(x)  (((x) > 0) ? (10) : (-10))

#define ADC_CENTER 2048
#define ADC_DEAD_ZONE 200 // 死区，避免轻微抖动

#define GYRO_DEAD_ZERO 5

#define MAX_SPEED 7200

extern int thb001p_adc_value[4];
extern float pitch, roll, yaw;

static void motor_speed_calc(int16_t *cmd_speed,int16_t *cmd_turn)
{
    // 电机转速处理
    int16_t rocker_x = thb001p_adc_value[0] - ADC_CENTER;
    int16_t rocker_y = thb001p_adc_value[1] - ADC_CENTER;

    if (abs(rocker_x) < ADC_DEAD_ZONE)
        rocker_x = 0;
    if (abs(rocker_y) < ADC_DEAD_ZONE)
        rocker_y = 0;

    int16_t speed = rocker_y * MAX_SPEED / (4095 / 2);
    int16_t turn = rocker_x * MAX_SPEED / (4095 / 2);

    float norm_speed = (float)speed / MAX_SPEED;
    float norm_turn = (float)turn / MAX_SPEED;

    *cmd_speed = (int16_t)(norm_speed * norm_speed * norm_speed * MAX_SPEED)/4;
    *cmd_turn = (int16_t)(norm_turn * norm_turn * norm_turn * MAX_SPEED);


    printf("motor_speed: %d  %d\n", *cmd_speed, *cmd_turn);
}

static void gimbal_angle_calc(float *angle_buttom_offset,float *angle_top_offset)
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
    switch (command->type)
    {
    case COMMAND_MOVE:
        motor_speed_calc(&command->payload.move.speed[0],&command->payload.move.speed[1]);
        break;
    
    default:
        break;
    }
    // gimbal_angle_calc(&command->payload.cam_rotate.angle[0], &command->payload.cam_rotate.angle[1]);
}