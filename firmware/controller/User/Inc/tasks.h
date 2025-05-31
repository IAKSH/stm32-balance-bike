#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void oled_task(void *argument);
void mpu6050_task(void *argument);
void wireless_send_task(void *argument);
void joystick_task(void *argument);

#ifdef __cplusplus
}
#endif