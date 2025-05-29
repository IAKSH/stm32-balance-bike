#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#ifndef SSD1306_SCREEN_WIDTH
#define SSD1306_SCREEN_WIDTH 128
#endif

#ifndef SSD1306_SCREEN_HEIGHT
#define SSD1306_SCREEN_HEIGHT 32
#endif

#define SSD1306_FRAMEBUFFER_SIZE (SSD1306_SCREEN_WIDTH * (SSD1306_SCREEN_HEIGHT / 8))

#define SSD1306_MIRROR_VERT false
#define SSD1306_MIRROR_HORIZ false

#define SSD1306_ENABLE_FONT_6x8
#define SSD1306_ENABLE_FONT_7x10
#define SSD1306_ENABLE_FONT_11x18
#define SSD1306_ENABLE_FONT_16x15
#define SSD1306_ENABLE_FONT_16x24
#define SSD1306_ENABLE_FONT_16x26

typedef struct {
    const uint8_t w,h;
    const uint8_t *const char_width;
    const uint16_t *const data;
} __SSD1306Font;

#ifdef SSD1306_ENABLE_FONT_6x8
extern const __SSD1306Font SSD1306Font_6x8;
#endif

#ifdef SSD1306_ENABLE_FONT_7x10
extern const __SSD1306Font SSD1306Font_7x10;
#endif

#ifdef SSD1306_ENABLE_FONT_11x18
extern const __SSD1306Font SSD1306Font_11x18;
#endif

#ifdef SSD1306_ENABLE_FONT_16x15
extern const __SSD1306Font SSD1306Font_16x15;
#endif

#ifdef SSD1306_ENABLE_FONT_16x24
extern const __SSD1306Font SSD1306Font_16x24;
#endif

#ifdef SSD1306_ENABLE_FONT_16x26
extern const __SSD1306Font SSD1306Font_16x26;
#endif

typedef enum {
    SSD1306_COLOR_WHITE,
    SSD1306_COLOR_BLACK
} SSD1306Color;

typedef struct {
    struct {
        uint8_t x,y;
    } cursor;
    SSD1306Color color;
    const __SSD1306Font* font;
    uint8_t __framebuffer[SSD1306_FRAMEBUFFER_SIZE];
    bool on;

    struct {
        void (*delay_ms)(uint32_t ms);
        void (*write_cmd)(uint8_t byte);
        void (*write_data)(uint8_t* buf,uint16_t len);
    } __impl;
} SSD1306State;

void ssd1306_use_state(SSD1306State* s);

void ssd1306_init();
void ssd1306_reset();
void ssd1306_create_state(SSD1306State* s); 
void ssd1306_turn_display(bool on);

void ssd1306_fill(SSD1306Color color);
void ssd1306_flush(void);

void ssd1306_write_char(char c);
void ssd1306_write_string(const char* s);
void ssd1306_write_int(int i);
void ssd1306_write_float( float f);
void ssd1306_write_float_with_precision(float f,uint8_t precision);

void ssd1306_draw_pixel(void);
void ssd1306_draw_line(uint8_t x,uint8_t y);
void ssd1306_draw_arc(uint8_t radius,uint16_t start_angle,uint16_t sweep);
void ssd1306_draw_arc_with_radius_line(uint8_t radius,uint16_t start_angle,uint16_t sweep);

void ssd1306_draw_circle(uint8_t r);
void ssd1306_fill_circle(uint8_t r);
void ssd1306_draw_rect(uint8_t x,uint8_t y);
void ssd1306_fill_rect(uint8_t x,uint8_t y);

typedef struct {
    uint8_t x,y;
} SSD1306Vertex2D;

void ssd1306_draw_2d_polyline(const SSD1306Vertex2D* const vertices,uint16_t vertices_len);

void ssd1306_draw_bitmap(uint8_t w,uint8_t h,const uint8_t* bitmap);

#ifdef __cplusplus
}
#endif