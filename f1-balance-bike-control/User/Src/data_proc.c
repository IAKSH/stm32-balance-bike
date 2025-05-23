#include "data_proc.h"
#include "stdlib.h"
#include "stdio.h"

#define SPEED_CALC(x) ((80.214 * (x) * (x)) + 4098.93 * (x) - 7200)

#define ADC_CENTER 2048
#define DEAD_ZONE 200 // 死区，避免轻微抖动

#define MAX_SPEED 7200

extern int thb001p_adc_value[4];
extern float pitch, roll, yaw;

void motor_speed_calc(void)
{
    // 电机转速处理
    int motor_speed_x = thb001p_adc_value[0] - ADC_CENTER;
    int motor_speed_y = thb001p_adc_value[1] - ADC_CENTER;

    printf("adc_val:%d\n", thb001p_adc_value[0]);

    if (abs(motor_speed_x) < DEAD_ZONE)
        motor_speed_x = 0;
    if (abs(motor_speed_y) < DEAD_ZONE)
        motor_speed_y = 0;

    int speed = motor_speed_y * MAX_SPEED / (4095 / 2);
    int turn = motor_speed_x * MAX_SPEED / (4095 / 2);
    printf("speed : %d\n", speed);
    int motor_left_speed = speed + turn;
    int motor_right_speed = speed - turn;

    if (motor_left_speed > MAX_SPEED) motor_left_speed = MAX_SPEED;
    if (motor_left_speed < -MAX_SPEED) motor_left_speed = -MAX_SPEED;
    if (motor_right_speed > MAX_SPEED) motor_right_speed = MAX_SPEED;
    if (motor_right_speed < -MAX_SPEED) motor_right_speed = -MAX_SPEED;

    float norm_left = (float)motor_left_speed / MAX_SPEED;
    float norm_right = (float)motor_right_speed / MAX_SPEED;

    motor_left_speed = (int)(norm_left * norm_left * norm_left * MAX_SPEED);
    motor_right_speed = (int)(norm_right * norm_right * norm_right * MAX_SPEED);


    printf("motor_speed: %d  %d\n", motor_left_speed, motor_right_speed);
}