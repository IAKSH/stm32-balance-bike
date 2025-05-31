#include "main.h"
#include "gpio.h"
#include "cmsis_os.h"
#include "stdio.h"
#include <protocols/wireless.h>

#define MPU6050_INT_PIN GPIO_PIN_5
#define EXIT7_WIRELESS_IRQ_Pin GPIO_PIN_4
extern osSemaphoreId_t mpu6050DataReadySemaphore;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin)
    {
    case  MPU6050_INT_PIN:
        osSemaphoreRelease(mpu6050DataReadySemaphore);
        break;
    case EXIT7_WIRELESS_IRQ_Pin:
        wireless_irq();
        break;
    default:
        break;
    }
}