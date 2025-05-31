#include "th8001p.h"
#include "adc.h"
#include "cmsis_os.h"

extern osSemaphoreId_t thb001pDataReadySemaphore;

void th8001p_init(void) {
    HAL_ADCEx_Calibration_Start(&hadc1);
}

void th8001p_read_data(th8001p_Data* data) {
    for(uint8_t i=0;i<4;i++) {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        uint16_t adc_value = HAL_ADC_GetValue(&hadc1);
        // thb001p_adc_value[i]= (float)adc_value*3.3f/4096.0f;
        data->adc_val[i] = adc_value;
    }
    HAL_ADC_Stop(&hadc1);
    osSemaphoreRelease(thb001pDataReadySemaphore);
}
