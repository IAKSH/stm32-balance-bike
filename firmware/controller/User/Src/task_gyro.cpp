#include "tasks.h"
#include "i2c.h"
#include <stdio.h>
#include <cmsis_os2.h>
#include <drivers/gyro/gyro.h>

float pitch,roll,yaw;
extern osSemaphoreId_t mpu6050DataReadySemaphore;

static uint8_t i2c_read(uint16_t dev_addr,uint16_t reg_addr,uint16_t data_size,uint8_t *p_data) {
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c1, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, p_data, data_size, 0xF);
    return ret;
}

static uint8_t i2c_write(uint16_t dev_addr,uint16_t reg_addr,uint16_t data_size,uint8_t *p_data) {
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(&hi2c1, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, p_data, data_size, 0xF);
    return ret;
}

static void nop(void) {
    __NOP();
}

static GyroState state;

static void setup() {
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
}

void mpu6050_task(void *argument) {
    setup();
    while (1) {
        osSemaphoreAcquire(mpu6050DataReadySemaphore,osWaitForever);
        gyro_get_data(&pitch,&roll,&yaw);
    }
}
