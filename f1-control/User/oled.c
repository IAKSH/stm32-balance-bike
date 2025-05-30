#include "tasks.h"
#include "ssd1306.h"
#include "main.h"
#include "cmsis_os2.h"
#include "stdio.h"
#include "math.h"

extern float mpu6050_pitch,mpu_6050_roll,mpu_6050_yaw;
extern double pitchPIDOutput;

#define I2C_PORT hi2c1
#define I2C_ADDR (0x3C<<1)

static void oled_wirte_cmd(uint8_t byte)
{
    osMutexAcquire(i2c_bus_mutex,osWaitForever);
    HAL_I2C_Mem_Write(&I2C_PORT,I2C_ADDR,0x00,1,&byte,1,HAL_MAX_DELAY);
    osMutexRelease(i2c_bus_mutex);
}

static void oled_write_data(uint8_t* buf,uint16_t len) {
    osMutexAcquire(i2c_bus_mutex,osWaitForever);
    HAL_I2C_Mem_Write(&I2C_PORT,I2C_ADDR,0x40,1,buf,len,HAL_MAX_DELAY);
    osMutexRelease(i2c_bus_mutex);
}

SSD1306State state;

void oled_task(void *argmuent)
{
    ssd1306_create_state(&state);

    state.__impl.delay_ms = HAL_Delay;
    state.__impl.write_cmd=oled_wirte_cmd;
    state.__impl.write_data=oled_write_data;

    state.font=&SSD1306Font_16x15;
    ssd1306_use_state(&state);
    ssd1306_init();

    printf("ssd1306 initialized!\n");

    state.cursor.x=state.cursor.y=0;

    ssd1306_write_string("init MPU6050...");
    ssd1306_flush();

    osEventFlagsWait(mpu6050_init_event,EVENT_FLAG_GYRO_INITIALIZED,osFlagsWaitAny,osWaitForever);
    ssd1306_fill(SSD1306_COLOR_BLACK);

    char buf[2][32];

    while (1)
    {
        snprintf(buf[0],sizeof(buf),"p= %d.%d%d r= %d.%d%d      ",
        (int)mpu6050_pitch,(int)fabs((int)(mpu6050_pitch * 10)) % 10,(int)fabs((int)(mpu6050_pitch * 100)) % 10,
        (int)mpu_6050_roll,(int)fabs((int)(mpu_6050_roll) * 10) % 10,(int)fabs((int)(mpu_6050_roll * 100)) % 10);

        state.cursor.x = 0;
        state.cursor.y = 0;
        ssd1306_write_string(buf[0]);
        state.cursor.x = 0;
        state.cursor.y = 12;
        ssd1306_write_string(buf[1]);

        ssd1306_flush();
        osDelay(10);
    }
    
}