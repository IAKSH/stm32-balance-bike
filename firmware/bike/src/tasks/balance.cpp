#include "cmsis_os2.h"
#include "motor.h"
#include "tasks.h"
#include <protocols/wireless.h>
#include <drivers/gyro/gyro.h>
#include <utils/pid.hpp>
#include <math.h>
#include <stdio.h>

#define ANGLE_KP 350.0f
#define ANGLE_KI 0.25f
#define ANGLE_KD 6.0f

#define SPEED_KP 1.6f
#define SPEED_KI 7.5f
#define SPEED_KD 0.0f

#define MAX_OUTPUT 7200
#define MIN_OUTPUT -7200
#define DEAD_ZONE 150

float gyro_pitch,gyro_roll,gyro_yaw;
float motor_speed_a,motor_speed_b;

extern CommandPacket command;
extern osEventFlagsId_t event;
extern osSemaphoreId_t gyro_ready_sem;

PID pid_angle = {.kp = ANGLE_KP, .ki = ANGLE_KI, .kd = ANGLE_KD};
PID pid_speed = {.kp = SPEED_KP, .ki = SPEED_KI, .kd = SPEED_KD};

static uint8_t i2c_read(uint16_t dev_addr,uint16_t reg_addr,uint16_t data_size,uint8_t *p_data) {
    osMutexAcquire(i2c_bus_mutex,osWaitForever);
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c1, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, p_data, data_size, 0xF);
    osMutexRelease(i2c_bus_mutex);
    return ret;
}

static uint8_t i2c_write(uint16_t dev_addr,uint16_t reg_addr,uint16_t data_size,uint8_t *p_data) {
    osMutexAcquire(i2c_bus_mutex,osWaitForever);
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(&hi2c1, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, p_data, data_size, 0xF);
    osMutexRelease(i2c_bus_mutex);
    return ret;
}

static void nop(void) {
    __NOP();
}

static inline int16_t clamp(int16_t val, int16_t min, int16_t max) {
    return val < min ? min : (val > max ? max : val);
}

void balance_task(void *arg) {
    GyroState state;
    gyro_create_state(&state);
    
    state.__impl.i2c_write = i2c_write;
    state.__impl.i2c_read = i2c_read;
    state.__impl.delay_ms = HAL_Delay;
    state.__impl.get_ms = HAL_GetTick;
    state.__impl.nop = nop;

    gyro_use_state(&state);

    while (gyro_init() != 0) {
        printf("MPU6050 init failed\n");
        osDelay(100);
    }
    osEventFlagsSet(event, EVENT_FLAG_GYRO_INITIALIZED);

    motor_init();
    motor_set_direct(MOTOR_A, MOTOR_FORWARD);
    motor_set_direct(MOTOR_B, MOTOR_FORWARD);


    while (1) {
        osSemaphoreAcquire(gyro_ready_sem, osWaitForever);
        gyro_get_data(&gyro_pitch, &gyro_roll, &gyro_yaw);
        motor_update_speed(&motor_speed_a, &motor_speed_b);

        float current_time = osKernelGetTickCount() * 0.001f;
        static float last_time = 0;
        float dt = current_time - last_time;
        last_time = current_time;
        if (dt <= 0)
            dt = 0.005f;

        // 检测防跌倒
        if (fabs(gyro_pitch) >= 40.0f) {
            motor_set_direct(MOTOR_A, MOTOR_STOP);
            motor_set_direct(MOTOR_B, MOTOR_STOP);
            osDelay(50); // 等待一会，防止频繁切换
            continue;
        }

        float target_angle=0.0f;
        float turn_offset =0.0f+ (float)command.payload.move.speed[1] / 3.0f;     // 转向差速
        

        // 外环 PID：角度控制
        pid_angle.set_point = target_angle;
        float angleCorrection = pid_angle(gyro_pitch, dt) + (float)command.payload.move.speed[0];
        
        angleCorrection=clamp(angleCorrection,MIN_OUTPUT,MAX_OUTPUT);

        // 内环 PID：速度控制（取左右轮平均速度）
        float avgSpeed = (motor_speed_a + motor_speed_b) / 2.0f ;
        pid_speed.set_point = angleCorrection;
        float speedCommand = pid_speed(avgSpeed, dt);

        // 生成两轮指令，加上转向偏差
        int16_t ctrlA = (int16_t)(speedCommand - turn_offset);
        int16_t ctrlB = (int16_t)(speedCommand + turn_offset);

        // 死区补偿
        if (ctrlA != 0)
            ctrlA += ctrlA > 0 ? DEAD_ZONE : -DEAD_ZONE;
        if (ctrlB != 0)
            ctrlB += ctrlB > 0 ? DEAD_ZONE : -DEAD_ZONE;

        // 限幅
        ctrlA = clamp(ctrlA, MIN_OUTPUT, MAX_OUTPUT);
        ctrlB = clamp(ctrlB, MIN_OUTPUT, MAX_OUTPUT);

        // 设置电机
        motor_set_direct(MOTOR_A, ctrlA >= 0 ? MOTOR_FORWARD : MOTOR_BACKWARD);
        motor_set_direct(MOTOR_B, ctrlB >= 0 ? MOTOR_FORWARD : MOTOR_BACKWARD);
        motor_set_speed(MOTOR_A, abs(ctrlA));
        motor_set_speed(MOTOR_B, abs(ctrlB));
    }
}
