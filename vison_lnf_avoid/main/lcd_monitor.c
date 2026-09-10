/*
 * LCD dashboard: three wheel speeds (rpm) + ultrasonic distance.
 *
 * A dedicated FreeRTOS task samples the three PCNT encoders and the latest
 * HC-SR04 reading every LCD_MONITOR_PERIOD_MS (200 ms) and redraws only the
 * rows whose text changed, so the SPI bus stays quiet and the control loop
 * is untouched.
 *
 * Speed:  rpm = delta_counts * 60000 / (elapsed_ms * WHEEL_COUNTS_PER_REV)
 * Distance: median of the last LCD_MONITOR_DISTANCE_MEDIAN valid HC-SR04
 * readings. The median removes single-pulse outliers; the raw accuracy of
 * the ISR-based echo timing is ~1 mm, so the displayed value stays well
 * inside the required 5 cm error budget.
 */
#include "lcd_monitor.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "board_config.h"
#include "encoder.h"
#include "lcd.h"
#include "ultrasonic.h"

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LCD_MONITOR_TAG        "lcd_monitor"
#define LCD_MONITOR_ROWS       4
#define LCD_MONITOR_VALUE_COLS 9 /* " 1234 RPM" fits scale 2 on 128 px */

/* Row labels, top to bottom: the three wheel speeds then the filtered
 * distance. Row LCD_MONITOR_ROWS-1 is the distance row, not a wheel. */
static const char *const s_rows[LCD_MONITOR_ROWS] = {
    "LEFT", "RIGHT", "REAR", "DIST",
};

static bool s_started;
static int s_previous_counts[ENCODER_WHEEL_COUNT];
static TickType_t s_previous_ticks;
static uint32_t s_distance_window[LCD_MONITOR_DISTANCE_MEDIAN];
static size_t s_distance_window_count;
static size_t s_distance_window_head;

static uint32_t lcd_monitor_distance_median(void)
{
    uint32_t sorted[LCD_MONITOR_DISTANCE_MEDIAN];
    memcpy(sorted, s_distance_window,
           s_distance_window_count * sizeof(sorted[0]));
    /* Insertion sort; the window holds at most 5 values. */
    for (size_t i = 1; i < s_distance_window_count; ++i) {
        const uint32_t key = sorted[i];
        size_t j = i;
        while (j > 0 && sorted[j - 1] > key) {
            sorted[j] = sorted[j - 1];
            --j;
        }
        sorted[j] = key;
    }
    return sorted[s_distance_window_count / 2];
}

static void lcd_monitor_push_distance(uint32_t distance_mm)
{
    if (s_distance_window_count < LCD_MONITOR_DISTANCE_MEDIAN) {
        s_distance_window[s_distance_window_count++] = distance_mm;
        return;
    }
    s_distance_window[s_distance_window_head] = distance_mm;
    s_distance_window_head = (s_distance_window_head + 1) %
                             LCD_MONITOR_DISTANCE_MEDIAN;
}

static void lcd_monitor_draw_static_frame(void)
{
    lcd_fill_screen(LCD_COLOR_BLACK);
    lcd_draw_text(0, 0, "SMART CAR", 1, LCD_COLOR_CYAN, LCD_COLOR_BLACK);
    for (int i = 0; i < LCD_MONITOR_ROWS; ++i) {
        const int label_y = 12 + i * 32;
        if (label_y + 8 <= lcd_height()) {
            lcd_draw_text(0, label_y, s_rows[i], 1,
                          LCD_COLOR_WHITE, LCD_COLOR_BLACK);
        }
    }
    lcd_draw_text(0, lcd_height() - 16, "US:WAIT", 1,
                  LCD_COLOR_GRAY, LCD_COLOR_BLACK);
}

static void lcd_monitor_task(void *argument)
{
    (void)argument;
    char rendered[LCD_MONITOR_ROWS][LCD_MONITOR_VALUE_COLS];
    bool rendered_valid[LCD_MONITOR_ROWS] = { false, false, false, false };
    char rendered_status[16] = { 0 };
    uint16_t rendered_status_color = 0;

    lcd_monitor_draw_static_frame();

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(LCD_MONITOR_PERIOD_MS));

        const TickType_t now = xTaskGetTickCount();
        const uint32_t elapsed_ms =
            (uint32_t)((now - s_previous_ticks) * portTICK_PERIOD_MS);
        s_previous_ticks = now;
        if (elapsed_ms == 0) {
            continue;
        }

        /* --- three wheel speeds --- */
        int counts[ENCODER_WHEEL_COUNT];
        if (encoder_get_all(&counts[ENCODER_WHEEL_LEFT],
                            &counts[ENCODER_WHEEL_RIGHT],
                            &counts[ENCODER_WHEEL_REAR]) == ESP_OK) {
            for (int wheel = 0; wheel < ENCODER_WHEEL_COUNT; ++wheel) {
                const int delta =
                    counts[wheel] - s_previous_counts[wheel];
                s_previous_counts[wheel] = counts[wheel];
                const int rpm = (int)((int64_t)delta * 60000LL /
                                      ((int64_t)elapsed_ms *
                                       (int64_t)WHEEL_COUNTS_PER_REV));
                char text[LCD_MONITOR_VALUE_COLS];
                snprintf(text, sizeof(text), "%4d RPM", rpm);
                if (!rendered_valid[wheel] ||
                    strcmp(text, rendered[wheel]) != 0) {
                    const int value_y = 12 + wheel * 32 + 10;
                    if (value_y + 16 <= lcd_height()) {
                        lcd_draw_text(0, value_y, text, 2,
                                      LCD_COLOR_YELLOW, LCD_COLOR_BLACK);
                    }
                    memcpy(rendered[wheel], text, sizeof(text));
                    rendered_valid[wheel] = true;
                }
            }
        }

        /* --- ultrasonic distance (median filtered) --- */
        ultrasonic_reading_t reading;
        if (ultrasonic_get_latest(&reading) == ESP_OK) {
            char text[LCD_MONITOR_VALUE_COLS];
            if (reading.status == ULTRASONIC_READING_VALID) {
                lcd_monitor_push_distance(reading.distance_mm);
            }
            if (s_distance_window_count > 0) {
                /* 1 mm equals 0.1 cm, so millimetres are already cm * 10. */
                const uint32_t tenths = lcd_monitor_distance_median();
                if (tenths > 9999U) {
                    snprintf(text, sizeof(text), "999.9 CM");
                } else {
                    snprintf(text, sizeof(text), "%3lu.%lu CM",
                             (unsigned long)(tenths / 10U),
                             (unsigned long)(tenths % 10U));
                }
            } else {
                snprintf(text, sizeof(text), "--.- CM");
            }
            if (!rendered_valid[LCD_MONITOR_ROWS - 1] ||
                strcmp(text, rendered[LCD_MONITOR_ROWS - 1]) != 0) {
                const int value_y = 12 + 3 * 32 + 10;
                if (value_y + 16 <= lcd_height()) {
                    lcd_draw_text(0, value_y, text, 2,
                                  LCD_COLOR_GREEN, LCD_COLOR_BLACK);
                }
                memcpy(rendered[LCD_MONITOR_ROWS - 1], text, sizeof(text));
                rendered_valid[LCD_MONITOR_ROWS - 1] = true;
            }

            const char *status = "WAIT";
            uint16_t status_color = LCD_COLOR_GRAY;
            switch (reading.status) {
            case ULTRASONIC_READING_VALID:
                status = "US:OK";
                status_color = LCD_COLOR_GREEN;
                break;
            case ULTRASONIC_READING_NO_ECHO:
                status = "US:NO ECHO";
                status_color = LCD_COLOR_RED;
                break;
            case ULTRASONIC_READING_ERROR:
                status = "US:ERROR";
                status_color = LCD_COLOR_RED;
                break;
            default:
                break;
            }
            if (strcmp(status, rendered_status) != 0 ||
                status_color != rendered_status_color) {
                lcd_draw_text(0, lcd_height() - 16, status, 1,
                              status_color, LCD_COLOR_BLACK);
                snprintf(rendered_status, sizeof(rendered_status), "%s", status);
                rendered_status_color = status_color;
            }
        }
    }
}

esp_err_t lcd_monitor_start(void)
{
    if (s_started) {
        return ESP_OK;
    }
    esp_err_t result = lcd_init();
    if (result != ESP_OK) {
        return result;
    }
#if LCD_ROTATION != 0
    ESP_LOGW(LCD_MONITOR_TAG,
             "LCD_ROTATION=%d: the dashboard layout expects the native "
             "128x160 portrait orientation", LCD_ROTATION);
#endif
    int left_count = 0;
    int right_count = 0;
    int rear_count = 0;
    if (encoder_get_all(&left_count, &right_count, &rear_count) == ESP_OK) {
        s_previous_counts[ENCODER_WHEEL_LEFT] = left_count;
        s_previous_counts[ENCODER_WHEEL_RIGHT] = right_count;
        s_previous_counts[ENCODER_WHEEL_REAR] = rear_count;
    }
    s_previous_ticks = xTaskGetTickCount();
    if (xTaskCreate(lcd_monitor_task, "lcd_monitor", 4096, NULL, 3, NULL) !=
        pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    s_started = true;
    ESP_LOGI(LCD_MONITOR_TAG,
             "dashboard started: period=%dms counts_per_rev=%d distance median=%d",
             (int)LCD_MONITOR_PERIOD_MS, (int)WHEEL_COUNTS_PER_REV,
             (int)LCD_MONITOR_DISTANCE_MEDIAN);
    return ESP_OK;
}
