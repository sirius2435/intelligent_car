#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* RGB565 color helper used by the dashboard. */
#define LCD_COLOR(r8, g8, b8)                                              \
    ((uint16_t)((((uint16_t)(r8) & 0xF8U) << 8) |                          \
                (((uint16_t)(g8) & 0xFCU) << 3) |                          \
                ((uint16_t)(b8) >> 3)))

#define LCD_COLOR_BLACK    LCD_COLOR(0, 0, 0)
#define LCD_COLOR_WHITE    LCD_COLOR(255, 255, 255)
#define LCD_COLOR_GRAY     LCD_COLOR(128, 128, 128)
#define LCD_COLOR_RED      LCD_COLOR(255, 0, 0)
#define LCD_COLOR_GREEN    LCD_COLOR(0, 255, 0)
#define LCD_COLOR_BLUE     LCD_COLOR(0, 0, 255)
#define LCD_COLOR_YELLOW   LCD_COLOR(255, 255, 0)
#define LCD_COLOR_CYAN     LCD_COLOR(0, 255, 255)
#define LCD_COLOR_ORANGE   LCD_COLOR(255, 160, 0)

/* Initialize the SPI bus and the LQ_TFT18SPI V3.3 panel. Safe to call once;
 * subsequent calls return ESP_OK without touching the hardware again. */
esp_err_t lcd_init(void);
bool lcd_is_ready(void);
int lcd_width(void);
int lcd_height(void);

void lcd_fill_screen(uint16_t color);
void lcd_fill_rect(int x, int y, int w, int h, uint16_t color);

/* Render an ASCII string with the built-in 8x8 bitmap font.
 * scale 1 = 8x8 px per char, scale 2 = 16x16 px per char, ...
 * Characters outside the built-in subset render as blanks. The string is
 * clipped at the right screen edge. */
void lcd_draw_text(int x, int y, const char *text, int scale,
                   uint16_t fg, uint16_t bg);

#ifdef __cplusplus
}
#endif
