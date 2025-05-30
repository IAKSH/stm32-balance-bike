#include "THB001P.h"
#include "adc.h"
#include "cmsis_os.h"


int thb001p_adc_value[4];
extern osSemaphoreId_t thb001pDataReadySemaphore;

void TH8001P_read_data(void)
{
    for(uint8_t i=0;i<4;i++)
    {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        uint16_t adc_value = HAL_ADC_GetValue(&hadc1);
        // thb001p_adc_value[i]= (float)adc_value*3.3f/4096.0f;
        thb001p_adc_value[i]=adc_value;
    }
    HAL_ADC_Stop(&hadc1);
    //osSemaphoreRelease(thb001pDataReadySemaphore);
}





