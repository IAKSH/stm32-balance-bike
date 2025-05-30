#include "main.h"
#include "cmsis_os.h"
#include "tasks.h"
#include "gyro.h"
#include "data_proc.h"
#include "stdio.h"

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

float mpu6050_pitch,mpu_6050_roll,mpu_6050_yaw;

void control_task(void *argument)
{
    GyroState state;
    gyro_create_state(&state);
    state.__impl.delay_ms = HAL_Delay;
    state.__impl.get_ms = HAL_GetTick;
    state.__impl.i2c_read = i2c_read;
    state.__impl.i2c_write = i2c_write;
    state.__impl.nop = nop;

    gyro_use_state(&state);

    int mpu6050_init_ret;
    while((mpu6050_init_ret = gyro_init()) != 0) {
        printf("MPU6050 init failed, ret = %d\n", mpu6050_init_ret);
        osDelay(100);
    }

    osEventFlagsSet(mpu6050_init_event,EVENT_FLAG_GYRO_INITIALIZED);

    while (1)
    {
        gyro_get_data(&mpu6050_pitch,&mpu_6050_roll,&mpu_6050_yaw);
        CommandPacket command = {
            .version = 0x00,
            .type = COMMAND_MOVE
        };

        data_packaing(&command);

        //wireless_send(&command, sizeof(command));
        osDelay(10);
    }
    
}
