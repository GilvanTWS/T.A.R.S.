#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define W 320
#define H 240
#define ROWS 16
#define LCD_DC 2
#define LCD_BL 21
#define TOUCH_IRQ 36
#define TOUCH_PRESSURE_MIN 150
#define TOUCH_MIN 200
#define TOUCH_MAX 3900
#define TOUCH_SWAP_XY 0
#define TOUCH_INVERT_X 0
#define TOUCH_INVERT_Y 1

static spi_device_handle_t lcd, touch;
static uint16_t *pixels;
static float eye_x, eye_y;
static float openness = 1.0f;
static bool happy;
static float sleep_amount;
#define DROWSY_AFTER_MS 40000
#define ASLEEP_AFTER_MS 60000
#define WAKE_DURATION_MS 1200

static float smooth_progress(float value)
{
    value = fminf(1.0f, fmaxf(0.0f, value));
    return value * value * (3.0f - 2.0f * value);
}


static void command(uint8_t cmd, const uint8_t *data, int length)
{
    ESP_ERROR_CHECK(spi_device_acquire_bus(lcd, portMAX_DELAY));
    gpio_set_level(LCD_DC, 0);
    spi_transaction_t t = {
        .flags = SPI_TRANS_USE_TXDATA | (length ? SPI_TRANS_CS_KEEP_ACTIVE : 0),
        .length = 8, .tx_data = {cmd}
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(lcd, &t));
    if (length) {
        gpio_set_level(LCD_DC, 1);
        t = (spi_transaction_t){.length = length * 8};
        if (length <= 4) {
            t.flags = SPI_TRANS_USE_TXDATA;
            memcpy(t.tx_data, data, length);
        } else {
            t.tx_buffer = data;
        }
        ESP_ERROR_CHECK(spi_device_polling_transmit(lcd, &t));
    }
    spi_device_release_bus(lcd);
}

static void init_hardware(void)
{
    gpio_config_t outputs = {.pin_bit_mask = (1ULL << LCD_DC) | (1ULL << LCD_BL),
                             .mode = GPIO_MODE_OUTPUT};
    ESP_ERROR_CHECK(gpio_config(&outputs));
    gpio_set_level(LCD_BL, 0);
    spi_bus_config_t bus = {.mosi_io_num = 13, .miso_io_num = -1,
        .sclk_io_num = 14, .quadwp_io_num = -1, .quadhd_io_num = -1,
        .max_transfer_sz = W * ROWS * 2};
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));
    spi_device_interface_config_t device = {.clock_speed_hz = 10000000,
        .mode = 0, .spics_io_num = 15, .queue_size = 1};
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &device, &lcd));
    command(0x01, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(150));
    command(0x28, NULL, 0);

    command(0xCF, (uint8_t[]){0x00, 0xC1, 0x30}, 3);
    command(0xED, (uint8_t[]){0x64, 0x03, 0x12, 0x81}, 4);
    command(0xE8, (uint8_t[]){0x85, 0x00, 0x78}, 3);
    command(0xCB, (uint8_t[]){0x39, 0x2C, 0x00, 0x34, 0x02}, 5);
    command(0xF7, (uint8_t[]){0x20}, 1);
    command(0xEA, (uint8_t[]){0x00, 0x00}, 2);
    command(0xC0, (uint8_t[]){0x10}, 1);
    command(0xC1, (uint8_t[]){0x00}, 1);
    command(0xC5, (uint8_t[]){0x30, 0x30}, 2);
    command(0xC7, (uint8_t[]){0xB7}, 1);

    command(0x36, (uint8_t[]){0xE8}, 1);
    command(0x3A, (uint8_t[]){0x55}, 1);
    command(0xB1, (uint8_t[]){0x00, 0x1A}, 2);
    command(0xB6, (uint8_t[]){0x08, 0x82, 0x27}, 3);
    command(0xF2, (uint8_t[]){0x00}, 1);
    command(0x26, (uint8_t[]){0x01}, 1);
    command(0xE0, (uint8_t[]){0x0F,0x2A,0x28,0x08,0x0E,0x08,0x54,0xA9,
                            0x43,0x0A,0x0F,0x00,0x00,0x00,0x00}, 15);
    command(0xE1, (uint8_t[]){0x00,0x15,0x17,0x07,0x11,0x06,0x2B,0x56,
                            0x3C,0x05,0x10,0x0F,0x3F,0x3F,0x0F}, 15);
    command(0x21, NULL, 0);
    command(0x11, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(120));
    command(0x36, (uint8_t[]){0xE8}, 1);
    command(0x3A, (uint8_t[]){0x55}, 1);
    command(0x29, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    bus = (spi_bus_config_t){.mosi_io_num = 32, .miso_io_num = 39,
        .sclk_io_num = 25, .quadwp_io_num = -1, .quadhd_io_num = -1,
        .max_transfer_sz = 8};
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &bus, SPI_DMA_CH_AUTO));
    device = (spi_device_interface_config_t){.clock_speed_hz = 500000,
        .mode = 0, .spics_io_num = 33, .queue_size = 1};
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &device, &touch));
    gpio_config_t irq = {.pin_bit_mask = 1ULL << TOUCH_IRQ,
                        .mode = GPIO_MODE_INPUT};
    ESP_ERROR_CHECK(gpio_config(&irq));

}

static int adc(uint8_t cmd, bool keep_selected)
{
    spi_transaction_t t = {.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA |
            (keep_selected ? SPI_TRANS_CS_KEEP_ACTIVE : 0),
        .length = 24, .tx_data = {cmd, 0, 0}};
    ESP_ERROR_CHECK(spi_device_polling_transmit(touch, &t));
    return ((t.rx_data[1] << 8) | t.rx_data[2]) >> 3;
}

static int clamp(int n, int lo, int hi) { return n < lo ? lo : (n > hi ? hi : n); }

static int median(int a, int b, int c)
{
    if (a > b) { int t = a; a = b; b = t; }
    if (b > c) { b = c; }
    return a > b ? a : b;
}

static bool read_touch(int *x, int *y)
{


    ESP_ERROR_CHECK(spi_device_acquire_bus(touch, portMAX_DELAY));
    adc(0xB1, true);
    int z1 = adc(0xB1, true);
    adc(0xC1, true);
    int z2 = adc(0xC1, true);
    adc(0xD1, true);
    int ax = adc(0xD1, true), bx = adc(0xD1, true), cx = adc(0xD1, true);
    adc(0x91, true);
    int ay = adc(0x91, true), by = adc(0x91, true), cy = adc(0x91, true);
    adc(0x90, false);
    spi_device_release_bus(touch);
    int rx = median(ax, bx, cx), ry = median(ay, by, cy);
    int pressure = z1 + 4095 - z2;
    bool valid = z1 > 0 && z1 < 4095 && z2 > 0 && z2 < 4095 &&
                 rx > 0 && rx < 4095 && ry > 0 && ry < 4095;
    static int64_t next_log;
    int64_t now = esp_timer_get_time() / 1000;
    if (now >= next_log) {
        ESP_LOGI("touch", "irq=%d z1=%d z2=%d pressao=%d raw_x=%d raw_y=%d valido=%d",
                 gpio_get_level(TOUCH_IRQ), z1, z2, pressure, rx, ry, valid);
        next_log = now + 2000;
    }
    if (!valid || pressure < TOUCH_PRESSURE_MIN) return false;
    if (TOUCH_SWAP_XY) { int t = rx; rx = ry; ry = t; }
    *x = clamp((rx - TOUCH_MIN) * (W - 1) / (TOUCH_MAX - TOUCH_MIN), 0, W - 1);
    *y = clamp((ry - TOUCH_MIN) * (H - 1) / (TOUCH_MAX - TOUCH_MIN), 0, H - 1);
    if (TOUCH_INVERT_X) *x = W - 1 - *x;
    if (TOUCH_INVERT_Y) *y = H - 1 - *y;
    return true;
}

static bool eye_pixel(int x, int y, int center)
{
    float dx = x - (center + eye_x), dy = y - (110 + eye_y);
    if (happy) {
        float curve = -12 + dx * dx / 65;
        return fabsf(dx) < 32 && fabsf(dy - curve) < 4;
    }
    float half_h = 3 + 34 * openness;
    float qx = fmaxf(fabsf(dx) - 22, 0);
    float qy = fmaxf(fabsf(dy) - fmaxf(half_h - 10, 0), 0);
    float radius = fminf(10, half_h);
    return qx * qx + qy * qy <= radius * radius;
}

static void render(void)
{
    uint16_t color = ((uint16_t)(31 - 19 * sleep_amount) << 5) | (uint16_t)(31 - 15 * sleep_amount);
    uint16_t wire_color = (color >> 8) | (color << 8);
    for (int top = 0; top < H; top += ROWS) {
        for (int row = 0; row < ROWS; ++row) {
            for (int x = 0; x < W; ++x) {
                int y = top + row;
                bool lit = eye_pixel(x, y, 100) || eye_pixel(x, y, 220);
                if (happy && abs(x - 160) < 18) {
                    int curve = 169 - (x - 160) * (x - 160) / 50;
                    lit |= abs(y - curve) < 2;
                }
                pixels[row * W + x] = lit ? wire_color : 0;
            }
        }
        command(0x2A, (uint8_t[]){0, 0, 1, 63}, 4);
        command(0x2B, (uint8_t[]){0, top, 0, top + ROWS - 1}, 4);
        command(0x2C, (const uint8_t *)pixels, W * ROWS * 2);
    }
}

static void render_title(int visible)
{
    static const uint8_t glyphs[7][7] = {
        {31, 4, 4, 4, 4, 4, 4},
        {0, 0, 0, 0, 0, 12, 12},
        {14, 17, 17, 31, 17, 17, 17},
        {0, 0, 0, 0, 0, 12, 12},
        {30, 17, 17, 30, 20, 18, 17},
        {0, 0, 0, 0, 0, 12, 12},
        {15, 16, 16, 14, 1, 1, 30}
    };
    const int scale = 6, advance = 36;
    const int left = (W - (7 * advance - scale)) / 2;
    const int upper = (H - 7 * scale) / 2;
    for (int top = 0; top < H; top += ROWS) {
        memset(pixels, 0, W * ROWS * sizeof(*pixels));
        for (int row = 0; row < ROWS; ++row) {
            int gy = (top + row - upper);
            if (gy < 0 || gy >= 7 * scale) continue;
            for (int x = left; x < W; ++x) {
                int offset = x - left;
                int letter = offset / advance;
                int gx = (offset % advance) / scale;
                if (letter < visible && letter < 7 && gx < 5 &&
                    (glyphs[letter][gy / scale] & (1U << (4 - gx)))) {
                    pixels[row * W + x] = 0xFF03;
                }
            }
        }
        command(0x2A, (uint8_t[]){0, 0, 1, 63}, 4);
        command(0x2B, (uint8_t[]){0, top, 0, top + ROWS - 1}, 4);
        command(0x2C, (const uint8_t *)pixels, W * ROWS * 2);
    }
}

static void boot_animation(void)
{
    render_title(0);
    gpio_set_level(LCD_BL, 1);
    for (int visible = 1; visible <= 7; ++visible) {
        render_title(visible);
        vTaskDelay(pdMS_TO_TICKS(180));
    }
    vTaskDelay(pdMS_TO_TICKS(600));
}

void app_main(void)
{
    pixels = heap_caps_malloc(W * ROWS * 2, MALLOC_CAP_DMA);
    ESP_ERROR_CHECK(pixels ? ESP_OK : ESP_ERR_NO_MEM);
    init_hardware();
    boot_animation();
    int64_t last_touch = esp_timer_get_time() / 1000;
    int64_t happy_until = 0, blink_at = last_touch + 2500, gaze_at = 0;
    float target_x = 0, target_y = 0;
    int old_x = 0, old_y = 0, movement = 0;
    bool was_down = false;
    int64_t wake_started = -1, previous_frame = last_touch;
    float wake_from = 0;
    ESP_LOGI("deskpet", "Olhos e touch iniciados. Toque para acordar; deslize para carinho.");
    while (1) {
        int64_t now = esp_timer_get_time() / 1000;
        float dt = (now - previous_frame) / 1000.0f;
        previous_frame = now;
        int x, y;
        bool down = read_touch(&x, &y);
        if (down) {
            if (!was_down && sleep_amount > 0 && wake_started < 0) {
                wake_from = sleep_amount;
                wake_started = now;
                blink_at = now + WAKE_DURATION_MS + 1500;
            }
            last_touch = now;
            target_x = (x - W / 2) / 9.0f;
            target_y = (y - H / 2) / 12.0f;
            if (was_down) {
                int delta = abs(x - old_x) + abs(y - old_y);
                if (delta > 4 && delta < 100) movement += delta;
                if (movement > 65) { happy_until = now + 1800; movement = 0; }
            } else {
                ESP_LOGI("deskpet", "Toque: x=%d y=%d", x, y);
            }
            old_x = x; old_y = y;
        } else {
            movement = 0;
            if (now >= gaze_at) {
                target_x = (int)(esp_random() % 25) - 12;
                target_y = (int)(esp_random() % 13) - 6;
                gaze_at = now + 1500 + esp_random() % 2500;
            }
        }
        was_down = down;
        if (wake_started >= 0) {
            float progress = (float)(now - wake_started) / WAKE_DURATION_MS;
            sleep_amount = wake_from * (1.0f - smooth_progress(progress));
            if (progress >= 1) wake_started = -1;
        } else {
            sleep_amount = smooth_progress((float)(now - last_touch - DROWSY_AFTER_MS) /
                                          (ASLEEP_AFTER_MS - DROWSY_AFTER_MS));
        }
        happy = now < happy_until && wake_started < 0;
        float breath = sinf((now % 4000) * (6.2831853f / 4000.0f));
        float animated_x = target_x * (1.0f - sleep_amount);
        float animated_y = target_y * (1.0f - sleep_amount) + 2.5f * breath * sleep_amount;
        if (now > blink_at + 180) blink_at = now + 2200 + esp_random() % 3000;
        float goal = 1.0f - sleep_amount;
        if (sleep_amount < 0.8f && wake_started < 0 && now >= blink_at && now < blink_at + 180) goal = 0;
        openness += (goal - openness) * (1.0f - expf(-dt / 0.065f));
        eye_x += (animated_x - eye_x) * (1.0f - expf(-dt / 0.18f));
        eye_y += (animated_y - eye_y) * (1.0f - expf(-dt / 0.18f));
        render();
        gpio_set_level(LCD_BL, 1);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
