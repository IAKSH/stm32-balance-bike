#include "cmsis_os2.h"
#include "motor.h"
#include "tasks.h"
#include <protocols/wireless.h>
#include <drivers/gyro/gyro.h>
#include <utils/pid.hpp>
#include <math.h>
#include <stdio.h>

float mpu6050_pitch,mpu_6050_roll,mpu_6050_yaw;
float motorSpeedA,motorSpeedB;

#define ANGLE_KP 350.0f
#define ANGLE_KI 0.25f
#define ANGLE_KD 6.0f

#define SPEED_KP 1.6f
#define SPEED_KI 7.5f
#define SPEED_KD 0.0f

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

#define MAX_OUTPUT 7200
#define MIN_OUTPUT -7200
#define DEAD_ZONE 150

float mpu6050_pitch, mpu6050_roll, mpu6050_yaw;
float motorSpeedA, motorSpeedB;

extern CommandPacket command;
extern osEventFlagsId_t event;
extern osSemaphoreId_t gyro_ready_sem;

PID pid_angle = {.kp = ANGLE_KP, .ki = ANGLE_KI, .kd = ANGLE_KD};
PID pid_speed = {.kp = SPEED_KP, .ki = SPEED_KI, .kd = SPEED_KD};

static inline int16_t clamp(int16_t val, int16_t min, int16_t max)
{
    return val < min ? min : (val > max ? max : val);
}

void balance_task(void *arg)
{
    // 初始化
    while (gyro_init() != 0)
    {
        printf("MPU6050 init failed\n");
        osDelay(100);
    }
    osEventFlagsSet(event, EVENT_FLAG_GYRO_INITIALIZED);

    motorInit();
    motorSetDirect(MOTOR_A, MOTOR_FORWARD);
    motorSetDirect(MOTOR_B, MOTOR_FORWARD);


    while (1)
    {
        osSemaphoreAcquire(gyro_ready_sem, osWaitForever);
        gyro_get_data(&mpu6050_pitch, &mpu6050_roll, &mpu6050_yaw);
        motorUpdateSpeed(&motorSpeedA, &motorSpeedB);

        float current_time = osKernelGetTickCount() * 0.001f;
        static float last_time = 0;
        float dt = current_time - last_time;
        last_time = current_time;
        if (dt <= 0)
            dt = 0.005f;

        // 检测防跌倒
        if (fabs(mpu6050_pitch) >= 40.0f)
        {

            motorSetDirect(MOTOR_A, MOTOR_STOP);
            motorSetDirect(MOTOR_B, MOTOR_STOP);
            osDelay(50); // 等待一会，防止频繁切换
            continue;
        }

        float target_angle=0.0f;
        float turn_offset =0.0f+ (float)command.payload.move.speed[1] / 3.0f;     // 转向差速
        

        // 外环 PID：角度控制
        pid_angle.set_point = target_angle;
        float angleCorrection = pid_angle(mpu6050_pitch, dt) + (float)command.payload.move.speed[0];
        
        angleCorrection=clamp(angleCorrection,MIN_OUTPUT,MAX_OUTPUT);

        // 内环 PID：速度控制（取左右轮平均速度）
        float avgSpeed = (motorSpeedA + motorSpeedB) / 2.0f ;
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
        motorSetDirect(MOTOR_A, ctrlA >= 0 ? MOTOR_FORWARD : MOTOR_BACKWARD);
        motorSetDirect(MOTOR_B, ctrlB >= 0 ? MOTOR_FORWARD : MOTOR_BACKWARD);
        motorSetSpeed(MOTOR_A, abs(ctrlA));
        motorSetSpeed(MOTOR_B, abs(ctrlB));
    }
}
