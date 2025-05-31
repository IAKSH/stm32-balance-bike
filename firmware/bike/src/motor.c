#include "motor.h"

void motor_init(void) {
    HAL_TIM_PWM_Start(&MOTOR_PWM_TIMER,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&MOTOR_PWM_TIMER,TIM_CHANNEL_4);

    HAL_TIM_Encoder_Start(&MOTOR_A_ENCODER_TIMER,TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&MOTOR_B_ENCODER_TIMER,TIM_CHANNEL_ALL);
    HAL_TIM_Base_Start_IT(&MOTOR_A_ENCODER_TIMER);
    HAL_TIM_Base_Start_IT(&MOTOR_B_ENCODER_TIMER);
}

void motor_set_direct(MotorID id, MotorDirection direction) {
    switch(id) {
        case MOTOR_A:
            HAL_GPIO_WritePin(MOTOR_CONTROL_A1_PORT,MOTOR_CONTROL_A1_PIN,direction == MOTOR_BACKWARD);
            HAL_GPIO_WritePin(MOTOR_CONTROL_A2_PORT,MOTOR_CONTROL_A2_PIN,direction == MOTOR_FORWARD);
            break;
        case MOTOR_B:
            HAL_GPIO_WritePin(MOTOR_CONTROL_B1_PORT,MOTOR_CONTROL_B1_PIN,direction == MOTOR_FORWARD);
            HAL_GPIO_WritePin(MOTOR_CONTROL_B2_PORT,MOTOR_CONTROL_B2_PIN,direction == MOTOR_BACKWARD);
            break;
        default:
            break;
    }
}

void motor_set_speed(MotorID id,uint16_t speed) {
    switch(id) {
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

void motor_update_speed(float* speed_a,float* speed_b) { 
    static uint16_t speed[4];
    speed[0] = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    speed[1] = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    speed[2] = UINT16_MAX - speed[0];
    speed[3] = UINT16_MAX - speed[1];

    *speed_a = speed[0] < speed[2] ? speed[0] : -(float)speed[2];
    *speed_b = speed[1] < speed[3] ? speed[1] : -(float)speed[3];
    *speed_b = -*speed_b;
    
    __HAL_TIM_SET_COUNTER(&htim2,0);
    __HAL_TIM_SET_COUNTER(&htim4,0);
}   
    