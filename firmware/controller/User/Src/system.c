#include "system.h"
#include "adc.h"
#include "stdio.h"

void system_init(void)
{
    HAL_ADCEx_Calibration_Start(&hadc1);
}