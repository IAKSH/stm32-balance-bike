#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "ssd1306.h"

static SSD1306State* state;

void ssd1306_create_state(SSD1306State* s) {
    s->cursor.x = 0;
    s->cursor.y = 0;
    s->color = SSD1306_COLOR_WHITE;
    s->font = NULL;
    memset(s->__framebuffer, 0, SSD1306_FRAMEBUFFER_SIZE);
    s->on = true;
    
    memset(&s->__impl,0,sizeof(s->__impl));
}

void ssd1306_use_state(SSD1306State* s) {
    state = s;
}

static void ftoa(float f, char* buf, unsigned char precision) {
    char *p = buf;
    
    if (f < 0) {
        *p++ = '-';
        f = -f;
    }
    
    int ipart = (int)f;
    float fpart = f - (float)ipart;
    itoa(ipart, p, 10);
    while (*p)
        p++;
    
    *p++ = '.';

    int pow10 = 1;
    for (unsigned char i = 0; i < precision; i++) {
        pow10 *= 10;
    }

    int f_int = (int)(fpart * pow10 + 0.5f);
    
    char frac[10] = {0};
    itoa(f_int, frac, 10);
    
    int frac_len = 0;
    while (frac[frac_len]) {
        frac_len++;
    }
    for (unsigned char i = 0; i < precision - frac_len; i++) {
        *p++ = '0';
    }
    strcpy(p, frac);
}

static inline float deg2rad(float deg) {
    return deg * (M_PI / 180.0f);
}

static inline uint16_t normalize_angle(uint16_t angle) {
    return angle % 360;
}

static inline void draw_pixel(int x,int y,SSD1306Color color) {
    if ((unsigned)x >= SSD1306_SCREEN_WIDTH || (unsigned)y >= SSD1306_SCREEN_HEIGHT) return;
    uint16_t idx = x + (y / 8) * SSD1306_SCREEN_WIDTH;
    uint8_t bit = 1 << (y % 8);
    if (color == SSD1306_COLOR_WHITE) state->__framebuffer[idx] |= bit;
    else state->__framebuffer[idx] &= ~bit;
}

static void draw_line_raw(int x0, int y0, int x1, int y1, SSD1306Color color) {
    int dx = abs(x1 - x0), dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1, sy = (y0 < y1) ? 1 : -1, err = dx - dy;
    while (1) {
        draw_pixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 < dx) { err += dx; y0 += sy; }
    }
}

void ssd1306_reset() {

}

void ssd1306_turn_display(bool on) {
    state->on = on;
    state->__impl.write_cmd(on ? 0xAF : 0xAE);
}

void ssd1306_init() {
    ssd1306_reset();
    printf("delay 1\n");
    state->__impl.delay_ms(100);
    printf("delay 2\n");
    ssd1306_turn_display(false);

    state->__impl.write_cmd(0x20); // Set Memory Addressing Mode
    state->__impl.write_cmd(0x00); // Horizontal Addressing Mode

    // 可选：设置页起始地址
    state->__impl.write_cmd(0xB0);

    if (SSD1306_MIRROR_VERT)
        state->__impl.write_cmd(0xC0);
    else
        state->__impl.write_cmd(0xC8);

    state->__impl.write_cmd(0x00); // set low column address
    state->__impl.write_cmd(0x10); // set high column address

    // 设置start line address
    state->__impl.write_cmd(0x40 | 0x00); // 通常为0

    state->__impl.write_cmd(0x81); // set contrast
    state->__impl.write_cmd(0xFF);

    if (SSD1306_MIRROR_HORIZ)
        state->__impl.write_cmd(0xA0);
    else
        state->__impl.write_cmd(0xA1);

    state->__impl.write_cmd(0xA6); // normal display

    // 自动设置multiplex ratio
    state->__impl.write_cmd(0xA8);
    state->__impl.write_cmd(SSD1306_SCREEN_HEIGHT - 1);

    state->__impl.write_cmd(0xA4); // Output follows RAM content

    state->__impl.write_cmd(0xD3); // display offset
    state->__impl.write_cmd(0x00);

    state->__impl.write_cmd(0xD5); // display clock divide ratio/oscillator frequency
    state->__impl.write_cmd(0xF0);

    state->__impl.write_cmd(0xD9); // pre-charge period
    state->__impl.write_cmd(0x22);

    // 自动设置com pins hardware configuration
    state->__impl.write_cmd(0xDA);
    if (SSD1306_SCREEN_HEIGHT == 32)
        state->__impl.write_cmd(0x02);
    else
        state->__impl.write_cmd(0x12);

    state->__impl.write_cmd(0xDB); // vcomh
    state->__impl.write_cmd(0x20);

    state->__impl.write_cmd(0x8D); // DC-DC enable
    state->__impl.write_cmd(0x14);

    ssd1306_turn_display(true);
}

void ssd1306_flush(void) {
    for (uint8_t page = 0; page < (SSD1306_SCREEN_HEIGHT / 8); page++) {
        state->__impl.write_cmd(0xB0 | page); // 设置页地址
        state->__impl.write_cmd(0x00);        // 设置低列地址
        state->__impl.write_cmd(0x10);        // 设置高列地址
        uint16_t offset = page * SSD1306_SCREEN_WIDTH;
        state->__impl.write_data(&state->__framebuffer[offset], SSD1306_SCREEN_WIDTH);
    }
}

void ssd1306_write_char(char c) {
    if (c < 32 || c > 126)
        return;
#ifdef SSD1306_ENABLE_FONT_6x8
    const __SSD1306Font* defaultFont = &SSD1306Font_6x8;
#else
    const __SSD1306Font* defaultFont = NULL;
#endif
    const __SSD1306Font* font = state->font ? state->font : defaultFont;
    if (!font)
        return;
    uint8_t char_width = font->char_width ? font->char_width[c - 32] : font->w;
    for (uint8_t row = 0; row < font->h; row++) {
        uint16_t row_data = font->data[(c - 32) * font->h + row];
        for (uint8_t col = 0; col < char_width; col++) {
            if ((row_data << col) & 0x8000)
                draw_pixel(state->cursor.x + col, state->cursor.y + row, state->color);
            else {
                SSD1306Color bg = (state->color == SSD1306_COLOR_WHITE) ? SSD1306_COLOR_BLACK : SSD1306_COLOR_WHITE;
                draw_pixel(state->cursor.x + col, state->cursor.y + row, bg);
            }
        }
    }
    state->cursor.x += char_width;
}

void ssd1306_write_string(const char* s) {
    while (*s) {
        ssd1306_write_char(*s);
        s++;
    }
}

void ssd1306_write_int(int i) {
    char buf[12];
    itoa(i, buf, 10);
    ssd1306_write_string(buf);
}

void ssd1306_write_float(float f) {
    char buf[20];
    ftoa(f, buf, 2);
    ssd1306_write_string(buf);
}

void ssd1306_write_float_with_precision(float f,uint8_t precision) {
    char buf[20];
    ftoa(f, buf, precision);
    ssd1306_write_string(buf);
}

void ssd1306_draw_pixel(void) {
    draw_pixel(state->cursor.x, state->cursor.y, state->color);
}

void ssd1306_draw_line(uint8_t x, uint8_t y) {
    draw_line_raw(state->cursor.x, state->cursor.y, x, y, state->color);
    state->cursor.x = x; state->cursor.y = y;
}

void ssd1306_draw_arc(uint8_t radius, uint16_t start_angle, uint16_t sweep) {
    int cx = state->cursor.x, cy = state->cursor.y;
    uint16_t norm_start = normalize_angle(start_angle);
    uint16_t norm_sweep = normalize_angle(sweep);
    uint8_t segments = (norm_sweep * 36) / 360;
    if (segments < 1) segments = 1;
    float angle_step = (float)norm_sweep / segments;
    float angle = norm_start;
    int prev_x = cx + (int)(sin(deg2rad(angle)) * radius);
    int prev_y = cy + (int)(cos(deg2rad(angle)) * radius);
    for (uint8_t i = 1; i <= segments; i++) {
        angle = norm_start + i * angle_step;
        int curr_x = cx + (int)(sin(deg2rad(angle)) * radius);
        int curr_y = cy + (int)(cos(deg2rad(angle)) * radius);
        draw_line_raw(prev_x, prev_y, curr_x, curr_y, state->color);
        prev_x = curr_x;
        prev_y = curr_y;
    }
}

void ssd1306_draw_arc_with_radius_line(uint8_t radius, uint16_t start_angle, uint16_t sweep) {
    ssd1306_draw_arc(radius, start_angle, sweep);
    int cx = state->cursor.x, cy = state->cursor.y;
    uint16_t norm_start = normalize_angle(start_angle);
    uint16_t norm_end = normalize_angle(start_angle + sweep);
    int start_x = cx + (int)(sin(deg2rad(norm_start)) * radius);
    int start_y = cy + (int)(cos(deg2rad(norm_start)) * radius);
    int end_x   = cx + (int)(sin(deg2rad(norm_end)) * radius);
    int end_y   = cy + (int)(cos(deg2rad(norm_end)) * radius);
    draw_line_raw(cx, cy, start_x, start_y, state->color);
    draw_line_raw(cx, cy, end_x, end_y, state->color);
}

void ssd1306_draw_circle(uint8_t r) {
    int cx = state->cursor.x, cy = state->cursor.y, x = -r, y = 0, err = 2 - 2 * r;
    do {
        draw_pixel(cx - x, cy + y, state->color);
        draw_pixel(cx + x, cy + y, state->color);
        draw_pixel(cx + x, cy - y, state->color);
        draw_pixel(cx - x, cy - y, state->color);
        int e2 = err;
        if (e2 <= y) { y++; err += y * 2 + 1; }
        if (e2 > x) { x++; err += x * 2 + 1; }
    } while (x <= 0);
}

void ssd1306_fill_circle(uint8_t r) {
    int cx = state->cursor.x, cy = state->cursor.y;
    for (int dy = -r; dy <= r; dy++) {
        int dx = (int)sqrt(r * r - dy * dy);
        for (int x = cx - dx; x <= cx + dx; x++)
            draw_pixel(x, cy + dy, state->color);
    }
}

void ssd1306_draw_rect(uint8_t w, uint8_t h) {
    int x0 = state->cursor.x, y0 = state->cursor.y, x1 = x0 + w - 1, y1 = y0 + h - 1;
    for (int i = x0; i <= x1; i++) draw_pixel(i, y0, state->color);
    for (int i = x0; i <= x1; i++) draw_pixel(i, y1, state->color);
    for (int j = y0; j <= y1; j++) draw_pixel(x0, j, state->color);
    for (int j = y0; j <= y1; j++) draw_pixel(x1, j, state->color);
}

void ssd1306_fill_rect(uint8_t w, uint8_t h) {
    int x0 = state->cursor.x, y0 = state->cursor.y;
    for (int j = 0; j < h; j++)
        for (int i = 0; i < w; i++)
            draw_pixel(x0 + i, y0 + j, state->color);
}

void ssd1306_draw_2d_polyline(const SSD1306Vertex2D* v, uint16_t n) {
    if (!v || n < 2) return;
    for (uint16_t i = 1; i < n; i++)
        draw_line_raw(v[i - 1].x, v[i - 1].y, v[i].x, v[i].y, state->color);
}

void ssd1306_draw_bitmap(uint8_t w, uint8_t h, const uint8_t* bmp) {
    int byteWidth = (w + 7) / 8, x0 = state->cursor.x, y0 = state->cursor.y;
    for (uint8_t j = 0; j < h; j++)
        for (uint8_t i = 0; i < w; i++)
            if (bmp[j * byteWidth + i / 8] & (0x80 >> (i % 8)))
                draw_pixel(x0 + i, y0 + j, state->color);
}

void ssd1306_fill(SSD1306Color color) {
    memset(state->__framebuffer, color == SSD1306_COLOR_WHITE ? 0xFF : 0x00, SSD1306_FRAMEBUFFER_SIZE);
}