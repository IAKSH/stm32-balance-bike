#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void test_led_task(void* arg);
void oled_task(void* arg);
void balance_task(void* arg);
void control_task(void* arg);

#ifdef __cplusplus
}
#endif
