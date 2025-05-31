#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

#define MOTOR_PWM_TIMER htim3
#define MOTOR_CONTROL_A1_PORT MOTOR_IO1_GPIO_Port
#define MOTOR_CONTROL_A1_PIN MOTOR_IO1_Pin
#define MOTOR_CONTROL_A2_PORT MOTOR_IO2_GPIO_Port
#define MOTOR_CONTROL_A2_PIN MOTOR_IO2_Pin
#define MOTOR_CONTROL_B1_PORT MOTOR_IO3_GPIO_Port
#define MOTOR_CONTROL_B1_PIN MOTOR_IO3_Pin
#define MOTOR_CONTROL_B2_PORT MOTOR_IO4_GPIO_Port
#define MOTOR_CONTROL_B2_PIN MOTOR_IO4_Pin

#define MOTOR_A_ENCODER_TIMER htim2
#define MOTOR_B_ENCODER_TIMER htim4

typedef enum {
    MOTOR_A,
    MOTOR_B
} MotorID;

typedef enum {
    MOTOR_FORWARD,
    MOTOR_BACKWARD,
    MOTOR_STOP
} MotorDirection;

void motor_init(void);
void motor_set_direct(MotorID id, MotorDirection direction);
void motor_set_speed(MotorID id,uint16_t speed);
void motor_update_speed(float* speed_a,float* speed_b);

#ifdef __cplusplus
}
#endif