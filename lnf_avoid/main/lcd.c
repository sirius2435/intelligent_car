/*
 * Minimal 4-wire SPI driver for the LQ_TFT18SPI V3.3 1.8" TFT.
 *
 * Panel: ST7735S controller, 128x160 pixels, IPS (inversion ON).
 * Wiring (see main/board_config.h): CS=36 SCK=35 SDI/MOSI=45 D/C=21 RST=20.
 *
 * RST shares GPIO20 with the ESP32-S3 USB-Serial-JTAG secondary console;
 * gpio_reset_pin() reclaims the pin for ordinary GPIO use at init. The
 * primary console stays on UART0, so logging is unaffected.
 *
 * D/C is driven from the SPI pre-transaction hook: every transaction sets
 * user=0 (command byte) or user=1 (data bytes).
 */
#include "lcd.h"

#include <stddef.h>
#include <string.h>

#include "board_config.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LCD_TAG "lcd"

/* MIPI-DCS style commands understood by the ST7735S panel. */
#define LCD_CMD_SWRESET 0x01
#define LCD_CMD_SLPOUT  0x11
#define LCD_CMD_NORON   0x13
#define LCD_CMD_INVOFF  0x20
#define LCD_CMD_INVON   0x21
#define LCD_CMD_DISPON  0x29
#define LCD_CMD_CASET   0x2A
#define LCD_CMD_RASET   0x2B
#define LCD_CMD_RAMWR   0x2C
#define LCD_CMD_MADCTL  0x36
#define LCD_CMD_COLMOD  0x3A
#define LCD_CMD_FRMCTR1 0xB1
#define LCD_CMD_FRMCTR2 0xB2
#define LCD_CMD_FRMCTR3 0xB3
#define LCD_CMD_INVCTR  0xB4
#define LCD_CMD_PWCTR1  0xC0
#define LCD_CMD_PWCTR2  0xC1
#define LCD_CMD_PWCTR3  0xC2
#define LCD_CMD_PWCTR4  0xC3
#define LCD_CMD_PWCTR5  0xC4
#define LCD_CMD_VMCTR1  0xC5
#define LCD_CMD_GMCTRP1 0xE0
#define LCD_CMD_GMCTRN1 0xE1

#define LCD_MADCTL_MY  0x80
#define LCD_MADCTL_MX  0x40
#define LCD_MADCTL_MV  0x20
#define LCD_MADCTL_BGR 0x08

#define LCD_TX_CHUNK_BYTES    64 /* FIFO mode: SOC_SPI_MAXIMUM_BUFFER_SIZE */
#define LCD_ROW_BUFFER_BYTES 512

/*
 * Built-in 8x8 bitmap font, covering only the characters the dashboard
 * prints. Bit 7 of each byte is the LEFTMOST pixel column; bytes run from
 * the top row to the bottom row. Missing characters render as blanks.
 */
typedef struct {
    char ch;
    uint8_t bits[8];
} lcd_font_glyph_t;

static const lcd_font_glyph_t s_font_glyphs[] = {
    { ' ', { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
    { '-', { 0x00, 0x00, 0x00, 0x7E, 0x00, 0x00, 0x00, 0x00 } },
    { '.', { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18 } },
    { ':', { 0x00, 0x18, 0x18, 0x00, 0x18, 0x18, 0x00, 0x00 } },
    { '0', { 0x3C, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x3C } },
    { '1', { 0x18, 0x38, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3C } },
    { '2', { 0x3C, 0x66, 0x06, 0x0C, 0x18, 0x30, 0x60, 0x7E } },
    { '3', { 0x3C, 0x66, 0x06, 0x1C, 0x06, 0x06, 0x66, 0x3C } },
    { '4', { 0x0C, 0x1C, 0x3C, 0x6C, 0xCC, 0xFE, 0x0C, 0x0C } },
    { '5', { 0x7E, 0x60, 0x60, 0x7C, 0x06, 0x06, 0x66, 0x3C } },
    { '6', { 0x1C, 0x30, 0x60, 0x7C, 0x66, 0x66, 0x66, 0x3C } },
    { '7', { 0x7E, 0x06, 0x0C, 0x18, 0x18, 0x18, 0x18, 0x18 } },
    { '8', { 0x3C, 0x66, 0x66, 0x3C, 0x66, 0x66, 0x66, 0x3C } },
    { '9', { 0x3C, 0x66, 0x66, 0x66, 0x3E, 0x06, 0x0C, 0x38 } },
    { 'A', { 0x18, 0x3C, 0x66, 0x66, 0x7E, 0x66, 0x66, 0x66 } },
    { 'C', { 0x3C, 0x66, 0xC0, 0xC0, 0xC0, 0xC0, 0x66, 0x3C } },
    { 'D', { 0x7C, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x7C } },
    { 'E', { 0x7E, 0x60, 0x60, 0x7C, 0x60, 0x60, 0x60, 0x7E } },
    { 'F', { 0x7E, 0x60, 0x60, 0x7C, 0x60, 0x60, 0x60, 0x60 } },
    { 'G', { 0x3C, 0x66, 0x60, 0x6E, 0x66, 0x66, 0x66, 0x3C } },
    { 'H', { 0x66, 0x66, 0x66, 0x7E, 0x66, 0x66, 0x66, 0x66 } },
    { 'I', { 0x3C, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3C } },
    { 'K', { 0x66, 0x66, 0x6C, 0x78, 0x6C, 0x66, 0x66, 0x66 } },
    { 'L', { 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x7E } },
    { 'M', { 0xC6, 0xEE, 0xFE, 0xD6, 0xC6, 0xC6, 0xC6, 0xC6 } },
    { 'N', { 0xC6, 0xE6, 0xF6, 0xDE, 0xCE, 0xC6, 0xC6, 0xC6 } },
    { 'O', { 0x3C, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x3C } },
    { 'P', { 0x7C, 0x66, 0x66, 0x66, 0x7C, 0x60, 0x60, 0x60 } },
    { 'R', { 0x7C, 0x66, 0x66, 0x7C, 0x6C, 0x66, 0x66, 0x66 } },
    { 'S', { 0x3C, 0x66, 0x60, 0x3C, 0x06, 0x06, 0x66, 0x3C } },
    { 'T', { 0x7E, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18 } },
    { 'U', { 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x3C } },
    { 'W', { 0xC6, 0xC6, 0xC6, 0xD6, 0xD6, 0xFE, 0xEE, 0xC6 } },
};

static const uint8_t *lcd_font_lookup(char ch)
{
    for (size_t i = 0; i < sizeof(s_font_glyphs) / sizeof(s_font_glyphs[0]); ++i) {
        if (s_font_glyphs[i].ch == ch) {
            return s_font_glyphs[i].bits;
        }
    }
    return NULL;
}

static spi_device_handle_t s_spi;
static bool s_ready;
static int s_width;
static int s_height;
static uint8_t s_madctl;

/* 512 B chunk buffer for pixel streaming + one text row buffer. Both are
 * static so the display task needs no large stack. */
static uint8_t s_chunk[LCD_TX_CHUNK_BYTES];
static uint8_t s_row_buffer[LCD_ROW_BUFFER_BYTES];

static void lcd_dc_pre_cb(spi_transaction_t *transaction)
{
    gpio_set_level((gpio_num_t)LCD_DC_GPIO, (int)(intptr_t)transaction->user);
}

/* The panel expects big-endian RGB565 on the wire; SPI shifts out the
 * memory-order bytes, so store every color byte-swapped. */
static inline uint16_t lcd_wire_color(uint16_t color)
{
#if LCD_SWAP_RB
    const uint16_t red = (uint16_t)(color & 0xF800U);
    const uint16_t blue = (uint16_t)(color & 0x001FU);
    color = (uint16_t)((color & 0x07E0U) | (blue << 11) | (red >> 11));
#endif
    return (uint16_t)((color >> 8) | (color << 8));
}

static esp_err_t lcd_write_cmd(uint8_t command)
{
    spi_transaction_t transaction = {
        .length = 8,
        .tx_buffer = &command,
        .user = (void *)(uintptr_t)0, /* D/C low: command byte */
    };
    return spi_device_polling_transmit(s_spi, &transaction);
}

static esp_err_t lcd_write_data(const uint8_t *data, size_t length)
{
    while (length > 0) {
        const size_t chunk =
            length < LCD_TX_CHUNK_BYTES ? length : LCD_TX_CHUNK_BYTES;
        spi_transaction_t transaction = {
            .length = chunk * 8,
            .tx_buffer = data,
            .user = (void *)(uintptr_t)1, /* D/C high: data bytes */
        };
        ESP_RETURN_ON_ERROR(spi_device_polling_transmit(s_spi, &transaction),
                            LCD_TAG, "SPI data write failed");
        data += chunk;
        length -= chunk;
    }
    return ESP_OK;
}

static esp_err_t lcd_set_window(int x0, int y0, int x1, int y1)
{
    x0 += LCD_X_OFFSET;
    x1 += LCD_X_OFFSET;
    y0 += LCD_Y_OFFSET;
    y1 += LCD_Y_OFFSET;
    const uint8_t column_params[4] = {
        (uint8_t)(x0 >> 8), (uint8_t)(x0 & 0xFF),
        (uint8_t)(x1 >> 8), (uint8_t)(x1 & 0xFF),
    };
    const uint8_t row_params[4] = {
        (uint8_t)(y0 >> 8), (uint8_t)(y0 & 0xFF),
        (uint8_t)(y1 >> 8), (uint8_t)(y1 & 0xFF),
    };
    ESP_RETURN_ON_ERROR(lcd_write_cmd(LCD_CMD_CASET), LCD_TAG, "CASET failed");
    ESP_RETURN_ON_ERROR(lcd_write_data(column_params, sizeof(column_params)),
                        LCD_TAG, "CASET params failed");
    ESP_RETURN_ON_ERROR(lcd_write_cmd(LCD_CMD_RASET), LCD_TAG, "RASET failed");
    ESP_RETURN_ON_ERROR(lcd_write_data(row_params, sizeof(row_params)),
                        LCD_TAG, "RASET params failed");
    return lcd_write_cmd(LCD_CMD_RAMWR);
}

int lcd_height(void)
{
    return s_height;
}

void lcd_fill_screen(uint16_t color)
{
    lcd_fill_rect(0, 0, s_width, s_height, color);
}

void lcd_fill_rect(int x, int y, int w, int h, uint16_t color)
{
    if (!s_ready || w <= 0 || h <= 0) {
        return;
    }
    if (x < 0) {
        w += x;
        x = 0;
    }
    if (y < 0) {
        h += y;
        y = 0;
    }
    if (x + w > s_width) {
        w = s_width - x;
    }
    if (y + h > s_height) {
        h = s_height - y;
    }
    if (w <= 0 || h <= 0) {
        return;
    }
    if (lcd_set_window(x, y, x + w - 1, y + h - 1) != ESP_OK) {
        return;
    }
    const uint16_t wire = lcd_wire_color(color);
    uint16_t *words = (uint16_t *)s_chunk;
    for (size_t i = 0; i < LCD_TX_CHUNK_BYTES / 2; ++i) {
        words[i] = wire;
    }
    size_t remaining = (size_t)w * (size_t)h * 2U;
    while (remaining > 0) {
        const size_t chunk =
            remaining < LCD_TX_CHUNK_BYTES ? remaining : LCD_TX_CHUNK_BYTES;
        if (lcd_write_data(s_chunk, chunk) != ESP_OK) {
            return;
        }
        remaining -= chunk;
    }
}

void lcd_draw_text(int x, int y, const char *text, int scale,
                   uint16_t fg, uint16_t bg)
{
    if (!s_ready || text == NULL || scale < 1) {
        return;
    }
    size_t length = strlen(text);
    while (length > 0 && (x + (int)length * 8 * scale) > s_width) {
        --length; /* clip at the right screen edge */
    }
    if (length == 0 || y < 0 || y + 8 * scale > s_height) {
        return;
    }
    const int pixel_height = 8 * scale;
    const int pixel_width = (int)length * 8 * scale;
    const uint16_t fg_wire = lcd_wire_color(fg);
    const uint16_t bg_wire = lcd_wire_color(bg);
    if (lcd_set_window(x, y, x + pixel_width - 1, y + pixel_height - 1) != ESP_OK) {
        return;
    }
    const size_t row_pixels = (size_t)pixel_width;
    if (row_pixels * 2U > LCD_ROW_BUFFER_BYTES) {
        return; /* cannot happen with the dashboard strings; keep a guard */
    }
    for (int row = 0; row < pixel_height; ++row) {
        uint16_t *out = (uint16_t *)s_row_buffer;
        for (size_t i = 0; i < length; ++i) {
            const uint8_t *glyph = lcd_font_lookup(text[i]);
            const uint8_t bits = glyph != NULL ? glyph[row / scale] : 0U;
            for (int col = 0; col < 8 * scale; ++col) {
                const bool on = (bits >> (7 - (col / scale))) & 0x01U;
                *out++ = on ? fg_wire : bg_wire;
            }
        }
        if (lcd_write_data(s_row_buffer, row_pixels * 2U) != ESP_OK) {
            return;
        }
    }
}

static esp_err_t lcd_gpio_init(void)
{
    /* Reclaim RST from the USB-Serial-JTAG secondary console (GPIO20). */
    gpio_reset_pin((gpio_num_t)LCD_RST_GPIO);
    const gpio_config_t output_config = {
        .pin_bit_mask = (1ULL << LCD_DC_GPIO) | (1ULL << LCD_RST_GPIO)
#if LCD_BLK_GPIO >= 0
                        | (1ULL << LCD_BLK_GPIO)
#endif
        ,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&output_config), LCD_TAG,
                        "failed to configure LCD control GPIOs");
#if LCD_BLK_GPIO >= 0
    gpio_set_level((gpio_num_t)LCD_BLK_GPIO, 1); /* backlight on */
#endif
    return ESP_OK;
}

static void lcd_hardware_reset(void)
{
    gpio_set_level((gpio_num_t)LCD_RST_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level((gpio_num_t)LCD_RST_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level((gpio_num_t)LCD_RST_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(120));
}

static esp_err_t lcd_send_init_sequence(void)
{
    static const uint8_t gamma_positive[] = {
        0x02, 0x1C, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2D,
        0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10,
    };
    static const uint8_t gamma_negative[] = {
        0x03, 0x1D, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D,
        0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10,
    };

    ESP_RETURN_ON_ERROR(lcd_write_cmd(LCD_CMD_SWRESET), LCD_TAG, "SWRESET failed");
    vTaskDelay(pdMS_TO_TICKS(150));
    ESP_RETURN_ON_ERROR(lcd_write_cmd(LCD_CMD_SLPOUT), LCD_TAG, "SLPOUT failed");
    vTaskDelay(pdMS_TO_TICKS(120));

    const uint8_t frmctr1[] = { 0x05, 0x3C, 0x3C };
    const uint8_t frmctr2[] = { 0x05, 0x3C, 0x3C };
    const uint8_t frmctr3[] = { 0x05, 0x3C, 0x3C, 0x05, 0x3C, 0x3C };
    const uint8_t invctr[] = { 0x07 };
    const uint8_t pwctr1[] = { 0xA2, 0x02, 0x84 };
    const uint8_t pwctr2[] = { 0xC5 };
    const uint8_t pwctr3[] = { 0x0A, 0x00 };
    const uint8_t pwctr4[] = { 0x8A, 0x2A };
    const uint8_t pwctr5[] = { 0x8A, 0xEE };
    const uint8_t vmctr1[] = { 0x0E };
    const uint8_t colmod[] = { 0x05 }; /* 16 bit RGB565 */

    ESP_RETURN_ON_ERROR(lcd_write_cmd(LCD_CMD_FRMCTR1), LCD_TAG, "FRMCTR1 failed");
    ESP_RETURN_ON_ERROR(lcd_write_data(frmctr1, sizeof(frmctr1)), LCD_TAG, "FRMCTR1 data failed");
    ESP_RETURN_ON_ERROR(lcd_write_cmd(LCD_CMD_FRMCTR2), LCD_TAG, "FRMCTR2 failed");
    ESP_RETURN_ON_ERROR(lcd_write_data(frmctr2, sizeof(frmctr2)), LCD_TAG, "FRMCTR2 data failed");
    ESP_RETURN_ON_ERROR(lcd_write_cmd(LCD_CMD_FRMCTR3), LCD_TAG, "FRMCTR3 failed");
    ESP_RETURN_ON_ERROR(lcd_write_data(frmctr3, sizeof(frmctr3)), LCD_TAG, "FRMCTR3 data failed");
    ESP_RETURN_ON_ERROR(lcd_write_cmd(LCD_CMD_INVCTR), LCD_TAG, "INVCTR failed");
    ESP_RETURN_ON_ERROR(lcd_write_data(invctr, sizeof(invctr)), LCD_TAG, "INVCTR data failed");
    ESP_RETURN_ON_ERROR(lcd_write_cmd(LCD_CMD_PWCTR1), LCD_TAG, "PWCTR1 failed");
    ESP_RETURN_ON_ERROR(lcd_write_data(pwctr1, sizeof(pwctr1)), LCD_TAG, "PWCTR1 data failed");
    ESP_RETURN_ON_ERROR(lcd_write_cmd(LCD_CMD_PWCTR2), LCD_TAG, "PWCTR2 failed");
    ESP_RETURN_ON_ERROR(lcd_write_data(pwctr2, sizeof(pwctr2)), LCD_TAG, "PWCTR2 data failed");
    ESP_RETURN_ON_ERROR(lcd_write_cmd(LCD_CMD_PWCTR3), LCD_TAG, "PWCTR3 failed");
    ESP_RETURN_ON_ERROR(lcd_write_data(pwctr3, sizeof(pwctr3)), LCD_TAG, "PWCTR3 data failed");
    ESP_RETURN_ON_ERROR(lcd_write_cmd(LCD_CMD_PWCTR4), LCD_TAG, "PWCTR4 failed");
    ESP_RETURN_ON_ERROR(lcd_write_data(pwctr4, sizeof(pwctr4)), LCD_TAG, "PWCTR4 data failed");
    ESP_RETURN_ON_ERROR(lcd_write_cmd(LCD_CMD_PWCTR5), LCD_TAG, "PWCTR5 failed");
    ESP_RETURN_ON_ERROR(lcd_write_data(pwctr5, sizeof(pwctr5)), LCD_TAG, "PWCTR5 data failed");
    ESP_RETURN_ON_ERROR(lcd_write_cmd(LCD_CMD_VMCTR1), LCD_TAG, "VMCTR1 failed");
    ESP_RETURN_ON_ERROR(lcd_write_data(vmctr1, sizeof(vmctr1)), LCD_TAG, "VMCTR1 data failed");

    ESP_RETURN_ON_ERROR(lcd_write_cmd(LCD_CMD_MADCTL), LCD_TAG, "MADCTL failed");
    ESP_RETURN_ON_ERROR(lcd_write_data(&s_madctl, 1), LCD_TAG, "MADCTL data failed");
    ESP_RETURN_ON_ERROR(lcd_write_cmd(LCD_CMD_COLMOD), LCD_TAG, "COLMOD failed");
    ESP_RETURN_ON_ERROR(lcd_write_data(colmod, sizeof(colmod)), LCD_TAG, "COLMOD data failed");

    ESP_RETURN_ON_ERROR(lcd_write_cmd(LCD_CMD_GMCTRP1), LCD_TAG, "GMCTRP1 failed");
    ESP_RETURN_ON_ERROR(lcd_write_data(gamma_positive, sizeof(gamma_positive)), LCD_TAG, "GMCTRP1 data failed");
    ESP_RETURN_ON_ERROR(lcd_write_cmd(LCD_CMD_GMCTRN1), LCD_TAG, "GMCTRN1 failed");
    ESP_RETURN_ON_ERROR(lcd_write_data(gamma_negative, sizeof(gamma_negative)), LCD_TAG, "GMCTRN1 data failed");

    ESP_RETURN_ON_ERROR(
        lcd_write_cmd(LCD_INVERT_COLORS ? LCD_CMD_INVON : LCD_CMD_INVOFF),
        LCD_TAG, "inversion command failed");
    ESP_RETURN_ON_ERROR(lcd_write_cmd(LCD_CMD_NORON), LCD_TAG, "NORON failed");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(lcd_write_cmd(LCD_CMD_DISPON), LCD_TAG, "DISPON failed");
    vTaskDelay(pdMS_TO_TICKS(50));
    return ESP_OK;
}

esp_err_t lcd_init(void)
{
    if (s_ready) {
        return ESP_OK;
    }

#if LCD_ROTATION == 1 || LCD_ROTATION == 3
    s_width = LCD_PANEL_HEIGHT;
    s_height = LCD_PANEL_WIDTH;
#else
    s_width = LCD_PANEL_WIDTH;
    s_height = LCD_PANEL_HEIGHT;
#endif

    uint8_t rotation_bits;
    switch (LCD_ROTATION) {
    case 1:
        rotation_bits = LCD_MADCTL_MX | LCD_MADCTL_MV;
        break;
    case 2:
        rotation_bits = LCD_MADCTL_MX | LCD_MADCTL_MY;
        break;
    case 3:
        rotation_bits = LCD_MADCTL_MY | LCD_MADCTL_MV;
        break;
    default:
        rotation_bits = 0x00;
        break;
    }
    s_madctl = rotation_bits;
#if LCD_SWAP_RB
    s_madctl |= LCD_MADCTL_BGR;
#endif

    ESP_RETURN_ON_ERROR(lcd_gpio_init(), LCD_TAG, "GPIO init failed");

    const spi_bus_config_t bus_config = {
        .mosi_io_num = LCD_SDI_GPIO,
        .sclk_io_num = LCD_SCK_GPIO,
        .miso_io_num = -1, /* the panel is write-only */
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_TX_CHUNK_BYTES,
    };
    ESP_RETURN_ON_ERROR(
        spi_bus_initialize(LCD_SPI_HOST, &bus_config, SPI_DMA_DISABLED),
        LCD_TAG, "SPI bus init failed");
    const spi_device_interface_config_t device_config = {
        .clock_speed_hz = LCD_CLOCK_HZ,
        .mode = 0, /* ST7735S: clock idle low, sample on rising edge */
        .spics_io_num = LCD_CS_GPIO,
        .queue_size = 8,
        .pre_cb = lcd_dc_pre_cb,
    };
    ESP_RETURN_ON_ERROR(
        spi_bus_add_device(LCD_SPI_HOST, &device_config, &s_spi),
        LCD_TAG, "SPI device add failed");

    lcd_hardware_reset();
    esp_err_t result = lcd_send_init_sequence();
    if (result != ESP_OK) {
        ESP_LOGE(LCD_TAG, "panel init sequence failed: %s",
                 esp_err_to_name(result));
        return result;
    }

    lcd_fill_screen(LCD_COLOR_BLACK);
    s_ready = true;
    ESP_LOGI(LCD_TAG,
             "ST7735S %dx%d ready: CS=%d SCK=%d SDI=%d DC=%d RST=%d SPI=%d @%dHz",
             s_width, s_height, LCD_CS_GPIO, LCD_SCK_GPIO, LCD_SDI_GPIO,
             LCD_DC_GPIO, LCD_RST_GPIO, LCD_SPI_HOST, LCD_CLOCK_HZ);
    return ESP_OK;
}
