#include "tasks.h"
#include "th8001p.h"
#include "i2c.h"
#include <cstdio>
#include <string>
#include <type_traits>
#include <cmsis_os2.h>
#include <drivers/ssd1306/ssd1306.h>

static void write_cmd(uint8_t command) {
	HAL_I2C_Mem_Write(&hi2c1, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, &command, 1, HAL_MAX_DELAY);
}

static void write_data(uint8_t *buf, uint16_t len) {
	HAL_I2C_Mem_Write(&hi2c1, 0x78, 0x40, I2C_MEMADD_SIZE_8BIT, buf, len, HAL_MAX_DELAY);
}

extern th8001p_Data joystick_data;
extern osSemaphoreId_t thb001pDataReadySemaphore;
extern float pitch,roll,yaw;
extern osSemaphoreId_t mpu6050DataReadySemaphore;

static SSD1306State state;

static void setup(void) {
	ssd1306_create_state(&state);
	state.__impl.delay_ms = HAL_Delay;
	state.__impl.write_cmd = write_cmd;
	state.__impl.write_data = write_data;

	ssd1306_use_state(&state);
	ssd1306_init();
}

template <typename T>
static void draw_at(uint8_t x,uint8_t y,const T& t) {
	state.cursor.x = x;
	state.cursor.y = y;
	if constexpr (std::is_same_v<T, int>) {
		ssd1306_write_int(t);
	} else if constexpr (std::is_convertible_v<T, char*>) {
		ssd1306_write_string(t);
	} else if constexpr (std::is_same_v<T, float> || std::is_same_v<T, double> ) {
		// show float with 3 decimal digits
		ssd1306_write_float_with_precision(static_cast<float>(t), 3);
	} else {
		ssd1306_write_string("error");
	}
}

void oled_task(void *argument) {
	setup();
	while (1) {
		draw_at(0,8,"left_x:");
		draw_at(0,16,"left_y:");
		draw_at(0,24,"right_x:");
		draw_at(0,32,"right_y:");

		draw_at(60,8,joystick_data.adc_val[0]);
		draw_at(60,16,joystick_data.adc_val[1]);
		draw_at(60,24,joystick_data.adc_val[2]);
		draw_at(60,32,joystick_data.adc_val[3]);

		// draw_at(60,8,joystick_data.adc_val[0]);
		// draw_at(60,16,joystick_data.adc_val[1]);
		// draw_at(60,24,joystick_data.adc_val[2]);
		// draw_at(60,32,joystick_data.adc_val[3]);

		if(osSemaphoreAcquire(thb001pDataReadySemaphore,100)==osOK) {
			ssd1306_flush();
		}

		draw_at(0,40,"pitch:");
		draw_at(0,48,"roll:");
		draw_at(0,56,"yaw:");

		draw_at(42,40,pitch);
		draw_at(42,48,roll);
		draw_at(42,56,yaw);

		ssd1306_flush();
		
		osDelay(100);
	}
}
