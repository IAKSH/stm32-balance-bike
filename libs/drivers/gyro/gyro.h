#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define GYRO_ERROR_MPU_INIT      -1
#define GYRO_ERROR_SET_SENSOR    -2
#define GYRO_ERROR_CONFIG_FIFO   -3
#define GYRO_ERROR_SET_RATE      -4
#define GYRO_ERROR_LOAD_MOTION_DRIVER    -5 
#define GYRO_ERROR_SET_ORIENTATION       -6
#define GYRO_ERROR_ENABLE_FEATURE        -7
#define GYRO_ERROR_SET_FIFO_RATE         -8
#define GYRO_ERROR_SELF_TEST             -9
#define GYRO_ERROR_DMP_STATE             -10

#define DEFAULT_MPU_HZ  200
#define Q30  1073741824.0f

typedef struct {
    struct {
        uint8_t (*i2c_write)(uint16_t dev_addr,uint16_t reg_addr,uint16_t data_size,uint8_t *p_data);
        uint8_t (*i2c_read)(uint16_t dev_addr,uint16_t reg_addr,uint16_t data_size,uint8_t *p_data);
        void (*delay_ms)(uint32_t ms);
        uint32_t (*get_ms)(void);
        void (*nop)(void);
    } __impl;
} GyroState;

extern GyroState* __gyro_state;

void gyro_create_state(GyroState* s);
void gyro_use_state(GyroState* s);

int gyro_init(void);
int gyro_get_data(float *pitch, float *roll, float *yaw);

#ifdef __cplusplus
}
#endif