#include "system.h"
#include "OLED.h"
#include "adc.h"

#include "mpu6050.h"

void system_init(void)
{
    OLED_Init();
    OLED_Clear();

    OLED_ShowString(0,0,"para:",OLED_6X8);

    OLED_Update();

    HAL_Delay(20);

    mpu6050_init();

    HAL_ADCEx_Calibration_Start(&hadc1);
}