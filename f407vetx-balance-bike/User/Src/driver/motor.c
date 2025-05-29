#include "motor.h"
#include "stdlib.h"

void motor_init(void)
{

    HAL_TIM_PWM_Start(&MOTOR_PWM_TIMER, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&MOTOR_PWM_TIMER, TIM_CHANNEL_4);

    HAL_TIM_Encoder_Start(&MOTOR_A_ENCODER_TIMER, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&MOTOR_B_ENCODER_TIMER, TIM_CHANNEL_ALL);

    HAL_TIM_Base_Start(&MOTOR_A_ENCODER_TIMER);
    HAL_TIM_Base_Start(&MOTOR_B_ENCODER_TIMER);
}

void motor_set_dirtect(MOTOR_ID id, MOTOR_DIRECTION direction)
{
    switch (id)
    {
    case MOTOR_A:
        HAL_GPIO_WritePin(MOTOR_CONTROL_A1_PORT, MOTOR_CONTROL_A1_PIN, direction == MOTOR_BACKWAD);
        HAL_GPIO_WritePin(MOTOR_CONTROL_A1_PORT, MOTOR_CONTROL_A2_PIN, direction == MOTOR_FORWARD);
        break;

    case MOTOR_B:
        HAL_GPIO_WritePin(MOTOR_CONTROL_B1_PORT, MOTOR_CONTROL_B1_PIN, direction == MOTOR_FORWARD);
        HAL_GPIO_WritePin(MOTOR_CONTROL_B2_PORT, MOTOR_CONTROL_B2_PIN, direction == MOTOR_BACKWAD);

    default:
        break;
    }
}

void motor_set_speed(MOTOR_ID id, int speed)
{
    if(speed>0) motor_set_dirtect(id,MOTOR_FORWARD);
    else if(speed<0) motor_set_dirtect(id,MOTOR_BACKWAD);
    else motor_set_dirtect(id,MOTOR_STOP);

    speed = abs(speed);
    switch (id)
    {
    case MOTOR_A:
        __HAL_TIM_SET_COMPARE(&MOTOR_PWM_TIMER, TIM_CHANNEL_3, speed);
        break;

    case MOTOR_B:
        __HAL_TIM_SET_COMPARE(&MOTOR_PWM_TIMER, TIM_CHANNEL_4, speed);
        break;

    default:
        break;
    }
}

void motor_update_speed(float *motorSpeedA, float *motorSpeedB)
{
    static uint16_t speedA,speedB;
    speedA = (uint16_t)__HAL_TIM_GET_COUNTER(&MOTOR_A_ENCODER_TIMER);
    speedB = (uint16_t)__HAL_TIM_GET_COUNTER(&MOTOR_B_ENCODER_TIMER);

    *motorSpeedA = (float)speedA;
    *motorSpeedB =(float)speedB;

    __HAL_TIM_SET_COUNTER(&MOTOR_A_ENCODER_TIMER,0);
    __HAL_TIM_SET_COUNTER(&MOTOR_B_ENCODER_TIMER,0);
    
}