#include "main.h"
#include "gpio.h"
#include "wireless.h"


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin)
    {
    // case  MPU6050_EXTI_Pin:
    //     //osSemaphoreRelease(mpu6050DataReadySemaphore);
    //     break;
    case IRQ_Pin:
        wireless_irq();
        break;
    default:
        break;
    }
}