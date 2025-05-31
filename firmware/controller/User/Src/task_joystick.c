#include "adc.h"
#include "th8001p.h"
#include <cmsis_os2.h>

th8001p_Data joystick_data;

void joystick_task(void *argument) {
    th8001p_init();
    while (1) {
        th8001p_read_data(&joystick_data);
        osDelay(10);
    }
}