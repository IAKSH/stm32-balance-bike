#include "main.h"
#include "gpio.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "wireless.h"

#define EXIT7_WIRELESS_IRQ_Pin GPIO_PIN_7

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin)
    {
    case  EXTI12_MPU_Pin:
        osSemaphoreRelease(mpu6050DataReadySemaphore);
        break;
    case EXIT7_WIRELESS_IRQ_Pin:
        wireless_irq();
        break;
    default:
        break;
    }
}