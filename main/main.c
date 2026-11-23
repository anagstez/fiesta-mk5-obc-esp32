/*
 * ST7789P3 2.25" 76x284 on ESP32-S3 — Ford Fiesta MK5 on-board computer, landscape 284x76
 *  data: MS-CAN (125 kbit/s) via the on-chip TWAI controller in listen-only mode (transmits only for a manual 'G' GEM read),
 *        IDs/bytes from github.com/roncapat/Ford-Fiesta-MK5-MS-CAN-bus; shows NO CAN without frames
 *  screens: DRIVE (rpm/speed/gauges/indicators), TRIP (6 trip stats), FUEL (fuel + volt); BOOT button (GPIO0), encoder (HW-040) or "N" over USB = next
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/usb_serial_jtag.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_heap_caps.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "mcp2515.h"
#include "car_icons.h"
#include "screens.h"

#define ICON_W 76
#define ICON_H 52
#define ICON_STRIDE 10

static const char *TAG = "obc";

#define PIN_SCK   12
#define PIN_MOSI  11
#define PIN_CS    10
#define PIN_DC     9
#define PIN_RST    8
#define PIN_BL     7
#define PIN_BTN    0                 /* BOOT button, active low */
#define PIN_CAN_TX 5                 /* to the transceiver TXD (unused in listen-only, but required) */
#define PIN_CAN_RX 4                 /* from the transceiver RXD */
#define PIN_ENC_CLK 1                /* HW-040 encoder, module has its own pull-ups, power it from 3V3 */
#define PIN_ENC_DT  2
#define PIN_ENC_SW  6                /* encoder push button, active low */
#define ENC_TRANS   2                /* quadrature transitions per detent (this encoder: 2, typical: 4) */
#define PIN_HS_SCK  14               /* MCP2515 (HS-CAN) on SPI3, 8 MHz crystal */
#define PIN_HS_MOSI 15
#define PIN_HS_MISO 16
#define PIN_HS_CS   17
#define PIN_HS_INT  18

/* landscape: logical 284x76; panel RAM is 240x320, so the window offsets are
 * 18 along the long axis and 82 along the short one (swap_xy exchanges them) */
#define LCD_W    284
#define LCD_H     76
#define GAP_X     18
#define GAP_Y     82
#define LCD_MIRROR_X true            /* flip these if the picture is mirrored/upside down */
#define LCD_MIRROR_Y false
#define LCD_HZ   (80 * 1000 * 1000)

#define FB_PIXELS (LCD_W * LCD_H)

#define STALE_MS  3000
#define FRAME_MS  50                 /* 20 fps redraw */

#define CONS_MIN_KMH 10              /* below this the consumption gauge shows L/h instead of L/100 km */

#define RPM_SEGS  16
#define RPM_MAX   7000

#define SCREEN_COUNT ((int)(sizeof(SCREENS) / sizeof(SCREENS[0])))   /* screens.h */
#define SCR_DRIVE 0                  /* SCREENS[0] */
#define GSER      3                  /* parameters per SK_GRAPH screen */
#define SCR_MENU  SCREEN_COUNT       /* after the configured screens, entered by a 5 s hold */
#define NCYCLE    SCREEN_COUNT       /* screens in the turn cycle */
#define LONG_MS   700                /* hold in the menu: back to DRIVE */
#define MENU_MS   5000               /* hold: open the settings menu */

static esp_lcd_panel_handle_t panel;
static SemaphoreHandle_t trans_done;
static uint16_t *fb;

typedef struct {
    int speed, rpm;                  /* km/h, rpm */
    int cool, volt10, gas;           /* degC, 0.1 V, pedal % */
    int flow10;                      /* fuel flow, 0.1 L/h; <0 = no source yet (needs HS-CAN/OBD data) */
    uint8_t arrows, lights, doors;   /* raw status bytes of 0x265 / 0x286 / 0x433 */
    bool reverse, locked, brake;
    int hh, mm;                      /* car clock (0x080) */
    int map;                         /* intake pressure, kPa (0x420 d7; checked against OBD PID 0B) */
    int fuel_lvl;                    /* tank level units (M 0x499 d4d5), ~15.2 per litre, zero point unknown */
} car_t;

/* status bits */
#define ARROW_L   0x20
#define ARROW_R   0x40
#define LIGHT_HB  0x40               /* high beams */
#define LIGHT_PB  0x10               /* handbrake */
#define DOOR_FL    0x80              /* 0x433 byte 0 (roncapat: front left/driver, front right, trunk) */
#define DOOR_FR    0x40
#define DOOR_TRUNK 0x08
#define DOOR_RL    0x20              /* ASSUMED: not in any source, verify with the 'D' dump (3-door/5-door!) */
#define DOOR_RR    0x10              /* ASSUMED, as above */
#define DOOR_ANY  (DOOR_FL | DOOR_FR | DOOR_RL | DOOR_RR | DOOR_TRUNK)
#define DOOR_POP_MS 4000             /* door popup stays this long after a door/trunk state change */

static car_t car;                    /* written from the TWAI rx ISR when live */

/* PCM values that exist only as OBD-II answers (nothing broadcasts them); each has its own timestamp, stale = "--" */
enum { Q_IAT, Q_TIMING, Q_STFT, Q_LTFT, Q_O2A, Q_LOAD, Q_ECT, Q_THR, Q_O2B, Q_FSYS, Q_DTC, NQ };
static const uint8_t obd_pids[NQ] = { 0x0F, 0x0E, 0x06, 0x07, 0x14, 0x04, 0x05, 0x11, 0x15, 0x03, 0x01 };
typedef struct { int val[NQ]; TickType_t ts[NQ]; } obd_t;
static volatile obd_t obd;
#define OBD_FRESH_MS 3000
static volatile bool can_seen;
static volatile TickType_t last_can;
static bool car_live;                /* false -> no fresh CAN frames */
static int screen;
static int bright = 100;             /* backlight, % (persisted in NVS, like the settings below) */
static int cc_step = 5;              /* km/h per encoder click when adjusting the cruise set point, 1..5 */
static int g_param, g_span;          /* menu override of the first graph screen: parameter (0 = screens.h), window in s (0 = screens.h) */
static bool al_on;                   /* AUTO LOCK as on a factory trip computer: GEM "lock at start of movement" (not applied yet) */
static float trip_m;                 /* metres */
static uint32_t trip_ms;
/* fuel: HS/MS 0x360 byte 7 is a free-running 8-bit counter of injected fuel; ISR sums its steps, 1 step = fuel_ul uL (calibrate by tank fill) */
static volatile uint32_t fuel_cnt;
static volatile bool fuel_seen;
static uint8_t fuel_prev;                                /* ISR only */
static int fuel_ul = 100;
static uint32_t fuel_last, trip_fuel, tank_cnt, tank_saved;
static float flow_ml_s;
static int max_speed, max_rpm;
static volatile TickType_t last_speed;   /* tick of the last 0x201 frame (speed source) */

/* advisory cruise: computes the throttle demand that would hold the set speed, drives nothing */
#define CC_MIN_KMH   15
#define CC_MAX_KMH   130
#define CC_DROP_KMH  15                  /* speed this far under the set point = braking/hill -> cancel */
#define CC_STALE_MS  300                 /* speed frame older than this -> cancel */
static bool cc_active;
static int cc_set;                       /* km/h */
static float cc_base, cc_i, cc_cmd;      /* pedal % at engage, integral, resulting demand % */
static TickType_t cc_evt_tick;           /* last ON/OFF event, drives the plaque flash / "CC OFF" notice */
static bool cc_evt_on, cc_evt_any;
#define CC_FLASH_MS  2000                /* ON plaque blinks this long */
#define CC_OFF_MS    2500                /* "CC OFF" notice stays this long */
static TickType_t door_pop_until;        /* popup visible until this tick */
static int ui_doors;                     /* 'O' over USB: fake door states for the popup (0 = real data) */
static int ui_preview;                   /* 'P' over USB: draw-only demo of the cruise display (1 on, 2 off-set, 3 "CC OFF"), drives nothing */
static TickType_t ui_preview_tick;
#define CC_DEV_KMH   5                   /* speed this far from the set point turns the digits amber */

/* 5x7 font, one byte per column, bit 0 = top row */
static const struct { char c; uint8_t col[5]; } font[] = {
    {'0', {0x3E, 0x51, 0x49, 0x45, 0x3E}}, {'1', {0x00, 0x42, 0x7F, 0x40, 0x00}},
    {'2', {0x42, 0x61, 0x51, 0x49, 0x46}}, {'3', {0x21, 0x41, 0x45, 0x4B, 0x31}},
    {'4', {0x18, 0x14, 0x12, 0x7F, 0x10}}, {'5', {0x27, 0x45, 0x45, 0x45, 0x39}},
    {'6', {0x3C, 0x4A, 0x49, 0x49, 0x30}}, {'7', {0x01, 0x71, 0x09, 0x05, 0x03}},
    {'8', {0x36, 0x49, 0x49, 0x49, 0x36}}, {'9', {0x06, 0x49, 0x49, 0x29, 0x1E}},
    {'.', {0x00, 0x60, 0x60, 0x00, 0x00}}, {'%', {0x23, 0x13, 0x08, 0x64, 0x62}},
    {'-', {0x08, 0x08, 0x08, 0x08, 0x08}}, {'A', {0x7E, 0x11, 0x11, 0x11, 0x7E}},
    {'B', {0x7F, 0x49, 0x49, 0x49, 0x36}}, {'C', {0x3E, 0x41, 0x41, 0x41, 0x22}},
    {'D', {0x7F, 0x41, 0x41, 0x22, 0x1C}}, {'K', {0x7F, 0x08, 0x14, 0x22, 0x41}},
    {'M', {0x7F, 0x02, 0x0C, 0x02, 0x7F}}, {'N', {0x7F, 0x04, 0x08, 0x10, 0x7F}},
    {'O', {0x3E, 0x41, 0x41, 0x41, 0x3E}}, {'P', {0x7F, 0x09, 0x09, 0x09, 0x06}},
    {'R', {0x7F, 0x09, 0x19, 0x29, 0x46}}, {'T', {0x01, 0x01, 0x7F, 0x01, 0x01}},
    {'U', {0x3F, 0x40, 0x40, 0x40, 0x3F}}, {'X', {0x63, 0x14, 0x08, 0x14, 0x63}},
    {'E', {0x7F, 0x49, 0x49, 0x49, 0x41}}, {'F', {0x7F, 0x09, 0x09, 0x09, 0x01}},
    {'G', {0x3E, 0x41, 0x49, 0x49, 0x7A}}, {'H', {0x7F, 0x08, 0x08, 0x08, 0x7F}},
    {'I', {0x00, 0x41, 0x7F, 0x41, 0x00}}, {'L', {0x7F, 0x40, 0x40, 0x40, 0x40}},
    {'S', {0x46, 0x49, 0x49, 0x49, 0x31}}, {'V', {0x1F, 0x20, 0x40, 0x20, 0x1F}},
    {'W', {0x3F, 0x40, 0x38, 0x40, 0x3F}}, {'Y', {0x07, 0x08, 0x70, 0x08, 0x07}},
    {':', {0x00, 0x36, 0x36, 0x00, 0x00}}, {'/', {0x20, 0x10, 0x08, 0x04, 0x02}},
};

/* RGB888 -> RGB565, byte-swapped for SPI */
static inline uint16_t rgb(uint8_t r, uint8_t g, uint8_t b)
{
    uint16_t c = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
    return (c >> 8) | (c << 8);
}

/* Ford-style palette: bright blue background, white instruments, dark bar tracks, amber warnings */
#define GRAY   rgb(205, 225, 250)
#define WHITE  rgb(255, 255, 255)
#define DIM    rgb(0, 28, 84)
#define OK_C   rgb(240, 248, 255)
#define ICE    rgb(150, 235, 255)
#define AMBER  rgb(255, 190, 0)
#define RED    rgb(255, 70, 70)
#define SER_B  rgb(110, 255, 130)           /* 2nd and 3rd line of a multi-parameter graph (1st is WHITE) */
#define SER_C  rgb(255, 150, 255)
#define FORD_BLUE rgb(0, 40, 125)
#define GREEN_ON  rgb(70, 235, 110)
#define CHIP_FG   rgb(0, 20, 60)
#define CHIP_OFF  rgb(70, 120, 190)

static bool IRAM_ATTR on_trans_done(esp_lcd_panel_io_handle_t io,
                                    esp_lcd_panel_io_event_data_t *ed, void *ctx)
{
    BaseType_t hp = pdFALSE;
    xSemaphoreGiveFromISR(trans_done, &hp);
    return hp == pdTRUE;
}

static void backlight_init(uint8_t duty)
{
    ledc_timer_config_t t = {
        .speed_mode = LEDC_LOW_SPEED_MODE, .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_8_BIT, .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK };
    ESP_ERROR_CHECK(ledc_timer_config(&t));
    ledc_channel_config_t c = {
        .gpio_num = PIN_BL, .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0, .timer_sel = LEDC_TIMER_0, .duty = duty,
        .flags.output_invert = 1 };      /* backlight is active-low: duty 0 must mean dark */
    ESP_ERROR_CHECK(ledc_channel_config(&c));
}

static int clampi(int v, int lo, int hi);

static void backlight_set(int pct)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pct * 255 / 100);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void settings_load(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) { ESP_LOGW(TAG, "NVS init failed, default settings"); return; }
    nvs_handle_t h;
    int8_t v;
    if (nvs_open("obc", NVS_READONLY, &h) == ESP_OK) {
        if (nvs_get_i8(h, "bright", &v) == ESP_OK) bright = clampi(v, 10, 100);
        if (nvs_get_i8(h, "ccstep", &v) == ESP_OK) cc_step = clampi(v, 1, 5);
        if (nvs_get_i8(h, "lock_on", &v) == ESP_OK) al_on = v != 0;
        if (nvs_get_i8(h, "g_param", &v) == ESP_OK) g_param = clampi(v, 0, NPARAM - 1);
        int16_t gs;
        if (nvs_get_i16(h, "g_span", &gs) == ESP_OK) g_span = clampi(gs, 0, 3600);
        int16_t u;
        if (nvs_get_i16(h, "fuel_ul", &u) == ESP_OK) fuel_ul = clampi(u, 20, 400);
        if (nvs_get_u32(h, "fuel_cnt", &tank_cnt) == ESP_OK) tank_saved = tank_cnt;
        nvs_close(h);
    }
}

static void settings_save(void)
{
    nvs_handle_t h;
    if (nvs_open("obc", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_i8(h, "bright", (int8_t)bright);
    nvs_set_i8(h, "ccstep", (int8_t)cc_step);
    nvs_set_i8(h, "lock_on", al_on);
    nvs_set_i8(h, "g_param", (int8_t)g_param);
    nvs_set_i16(h, "g_span", (int16_t)g_span);
    nvs_set_i16(h, "fuel_ul", (int16_t)fuel_ul);
    nvs_commit(h);
    nvs_close(h);
}

static void fuel_save(void)
{
    nvs_handle_t h;
    if (nvs_open("obc", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u32(h, "fuel_cnt", tank_cnt);
    nvs_commit(h);
    nvs_close(h);
    tank_saved = tank_cnt;
}

static void lcd_init(void)
{
    spi_bus_config_t bus = {
        .sclk_io_num = PIN_SCK, .mosi_io_num = PIN_MOSI, .miso_io_num = -1,
        .quadwp_io_num = -1, .quadhd_io_num = -1,
        .max_transfer_sz = FB_PIXELS * 2 };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t io;
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num = PIN_CS, .dc_gpio_num = PIN_DC,
        .spi_mode = 0, .pclk_hz = LCD_HZ,
        .trans_queue_depth = 10, .lcd_cmd_bits = 8, .lcd_param_bits = 8,
        .on_color_trans_done = on_trans_done };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_cfg, &io));

    esp_lcd_panel_dev_config_t pcfg = {
        .reset_gpio_num = PIN_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16 };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io, &pcfg, &panel));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel, false));  /* this panel is not inverted (true gave negative colours) */
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel, true));        /* landscape */
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel, LCD_MIRROR_X, LCD_MIRROR_Y));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel, GAP_X, GAP_Y));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));
}

static int clampi(int v, int lo, int hi) { return v < lo ? lo : v > hi ? hi : v; }

static void fill_rect(int x, int y, int w, int h, uint16_t c)
{
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > LCD_W) w = LCD_W - x;
    if (y + h > LCD_H) h = LCD_H - y;
    for (int j = 0; j < h; j++)
        for (int i = 0; i < w; i++)
            fb[(y + j) * LCD_W + x + i] = c;
}

static void fill_ellipse(int cx, int cy, int rx, int ry, uint16_t c)
{
    for (int dy = -ry; dy <= ry; dy++) {
        float k = 1.0f - (float)(dy * dy) / (float)(ry * ry);
        int hw = (int)(rx * sqrtf(k < 0 ? 0 : k));
        fill_rect(cx - hw, cy + dy, 2 * hw + 1, 1, c);
    }
}

static int text_w(const char *s, int scale)
{
    return (int)strlen(s) * 6 * scale - scale;
}

static void draw_text(int x, int y, int scale, uint16_t color, const char *s)
{
    for (; *s; s++, x += 6 * scale) {
        const uint8_t *g = NULL;
        for (size_t i = 0; i < sizeof(font) / sizeof(font[0]); i++)
            if (font[i].c == *s) { g = font[i].col; break; }
        if (!g) continue;
        for (int cx = 0; cx < 5; cx++)
            for (int cy = 0; cy < 7; cy++)
                if (g[cx] & (1 << cy))
                    fill_rect(x + cx * scale, y + cy * scale, scale, scale, color);
    }
}

/* blue oval with a white ring and the block-letter name; scale = text scale */
static void draw_badge(int cx, int cy, int rx, int ry, int ring, int scale)
{
    fill_ellipse(cx, cy, rx, ry, WHITE);
    fill_ellipse(cx, cy, rx - ring, ry - ring, FORD_BLUE);
    draw_text(cx - text_w("FORD", scale) / 2, cy - (7 * scale) / 2, scale, WHITE, "FORD");
}

/* horizontal bar, fill in permille */
static void draw_bar(int x, int y, int w, int h, int permille, uint16_t col)
{
    fill_rect(x, y, w, h, DIM);
    fill_rect(x, y, w * clampi(permille, 0, 1000) / 1000, h, col);
}

/* label (small) over value (x2) over bar, 84 px wide cell */
static void draw_gauge(int x, int y, const char *label, const char *val, int permille, uint16_t col)
{
    draw_text(x, y, 1, GRAY, label);
    draw_text(x, y + 9, 2, WHITE, val);
    draw_bar(x, y + 25, 80, 4, permille, col);
}

static void draw_chip(int x, int y, const char *t, bool on, uint16_t bg, uint16_t fg)
{
    fill_rect(x, y, 25, 12, on ? bg : DIM);
    draw_text(x + (25 - text_w(t, 1)) / 2, y + 3, 1, on ? fg : CHIP_OFF, t);
}

static uint8_t doors_mask(void)
{
    static const uint8_t sample[] = { 0, DOOR_FL | DOOR_FR, DOOR_FL, DOOR_RR | DOOR_TRUNK, DOOR_ANY, 0 };
    return ui_doors ? sample[ui_doors] : car.doors & DOOR_ANY;
}

static void draw_line(int x0, int y0, int x1, int y1, int t, uint16_t c)
{
    const int dx = abs(x1 - x0), dy = -abs(y1 - y0), sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    for (;;) {
        fill_rect(x0, y0, t, t, c);
        if (x0 == x1 && y0 == y1) break;
        const int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

/* convex-or-simple polygon, even-odd scanline; xy = x0,y0,x1,y1,... relative to (ox, oy) */
static void fill_poly(const int *xy, int n, int ox, int oy, uint16_t c)
{
    int y0 = xy[1], y1 = xy[1];
    for (int i = 1; i < n; i++) { if (xy[2 * i + 1] < y0) y0 = xy[2 * i + 1]; if (xy[2 * i + 1] > y1) y1 = xy[2 * i + 1]; }
    for (int y = y0; y <= y1; y++) {
        int xs[16], k = 0;
        for (int i = 0; i < n && k < 16; i++) {
            const int ax = xy[2 * i], ay = xy[2 * i + 1];
            const int bx = xy[2 * ((i + 1) % n)], by = xy[2 * ((i + 1) % n) + 1];
            if ((ay <= y && by > y) || (by <= y && ay > y)) xs[k++] = ax + (y - ay) * (bx - ax) / (by - ay);
        }
        for (int i = 1; i < k; i++) for (int j = i; j > 0 && xs[j] < xs[j - 1]; j--) { int t = xs[j]; xs[j] = xs[j - 1]; xs[j - 1] = t; }
        for (int i = 0; i + 1 < k; i += 2) fill_rect(ox + xs[i], oy + y, xs[i + 1] - xs[i] + 1, 1, c);
    }
}

/* 1 bpp layer 76x52, row = 10 bytes, MSB = left pixel; set pixels are painted, runs go out as one rect */
static void draw_icon(const uint8_t *ic, int ox, int oy, uint16_t c)
{
    for (int y = 0; y < ICON_H; y++) {
        const uint8_t *row = ic + y * ICON_STRIDE;
        for (int x = 0; x < ICON_W;) {
            if (!(row[x >> 3] & (0x80 >> (x & 7)))) { x++; continue; }
            int e = x;
            while (e < ICON_W && (row[e >> 3] & (0x80 >> (e & 7)))) e++;
            fill_rect(ox + x, oy + y, e - x, 1, c);
            x = e;
        }
    }
}

/* door/trunk popup: user-supplied layers (car_icons.h), nose to the LEFT, flush with the right and bottom screen edges
 * on the plain background; the consumption gauge and indicator chips are not drawn while it is up. Body is green when
 * everything is closed, open doors/trunk are drawn amber on top */
static void draw_door_panel(uint8_t m)
{
    const int ox = LCD_W - ICON_W, oy = LCD_H - ICON_H;
    draw_icon(icon_body, ox, oy, m ? OK_C : GREEN_ON);
    if (m & DOOR_FL)    draw_icon(icon_door_fl, ox, oy, AMBER);
    if (m & DOOR_FR)    draw_icon(icon_door_fr, ox, oy, AMBER);
    if (m & DOOR_RL)    draw_icon(icon_door_rl, ox, oy, AMBER);
    if (m & DOOR_RR)    draw_icon(icon_door_rr, ox, oy, AMBER);
    if (m & DOOR_TRUNK) draw_icon(icon_trunk, ox, oy, AMBER);
}

static void draw_drive(void)
{
    char b[24];

    int lit = clampi(car.rpm * RPM_SEGS / RPM_MAX, 0, RPM_SEGS);
    for (int i = 0; i < RPM_SEGS; i++) {
        uint16_t c = i >= lit ? DIM : i >= RPM_SEGS - 3 ? RED : i >= RPM_SEGS - 5 ? AMBER : i >= RPM_SEGS / 2 ? ICE : OK_C;
        fill_rect(2 + i * 14, 2, 13, 10, c);
    }
    draw_badge(257, 7, 25, 6, 1, 1);

    /* cruise display inputs: the real state, or the draw-only preview */
    bool act = cc_active, evt_on = cc_evt_on, off_notice;
    int set = cc_set, cmd = (int)cc_cmd;
    uint32_t evt_ms = cc_evt_any ? pdTICKS_TO_MS(xTaskGetTickCount() - cc_evt_tick) : UINT32_MAX;
    if (ui_preview) {
        act = ui_preview == 1 || ui_preview == 2;
        evt_on = true;
        set = ui_preview == 2 ? car.speed + 10 : car.speed;
        cmd = 40;
        evt_ms = ui_preview == 1 ? pdTICKS_TO_MS(xTaskGetTickCount() - ui_preview_tick) : UINT32_MAX;
    }
    off_notice = ui_preview ? ui_preview == 3 : (!cc_evt_on && evt_ms < CC_OFF_MS);
    uint16_t spd_col = WHITE;
    if (act) spd_col = abs(car.speed - set) > CC_DEV_KMH ? AMBER : GREEN_ON;
    snprintf(b, sizeof(b), "%d", car.speed);
    draw_text(4, 16, 6, spd_col, b);

    /* bottom-left: cruise plaque (green, blinks on engage; red "CC OFF" after a cancel), then KM/H and NO CAN */
    int x = 4;
    if (act || off_notice) {
        char t[12];
        uint16_t bg = GREEN_ON, fg = CHIP_FG;
        if (act) {
            snprintf(t, sizeof(t), "CC %d", set);
            if (evt_on && evt_ms < CC_FLASH_MS && (evt_ms / 250) % 2) bg = WHITE;
        } else {
            snprintf(t, sizeof(t), "CC OFF");
            bg = RED; fg = WHITE;
        }
        const int pw = text_w(t, 1) + 8;
        fill_rect(2, 62, pw, 12, bg);
        draw_text(6, 65, 1, fg, t);
        x = 2 + pw + 4;
    }
    draw_text(x, 65, 1, GRAY, "KM/H");
    if (!car_live) draw_text(x + text_w("KM/H", 1) + 6, 65, 1, RED, "NO CAN");

    fill_rect(106, 16, 1, 58, DIM);

    snprintf(b, sizeof(b), "%dC", car.cool);
    draw_gauge(114, 16, "COOL", b, (car.cool - 40) * 1000 / 80,
               car.cool < 70 ? ICE : car.cool > 105 ? RED : OK_C);
    const bool pop = (int32_t)(door_pop_until - xTaskGetTickCount()) > 0;   /* door popup replaces the right column */
    /* instantaneous consumption: L/100 km while moving, L/h when (almost) standing; "--" without a source */
    if (pop) {
        draw_door_panel(doors_mask());
    } else if (car.flow10 < 0) {
        draw_gauge(200, 16, car.speed >= CONS_MIN_KMH ? "L/100" : "L/H", "--", 0, OK_C);
    } else if (car.speed >= CONS_MIN_KMH) {
        const int c10 = car.flow10 * 100 / car.speed;    /* 0.1 L/100 km */
        snprintf(b, sizeof(b), "%d.%d", c10 / 10, c10 % 10);
        draw_gauge(200, 16, "L/100", b, c10 * 1000 / 200, c10 > 150 ? AMBER : OK_C);
    } else {
        snprintf(b, sizeof(b), "%d.%d", car.flow10 / 10, car.flow10 % 10);
        draw_gauge(200, 16, "L/H", b, car.flow10 * 1000 / 150, OK_C);
    }
    snprintf(b, sizeof(b), "%d%%", car.gas);
    draw_gauge(114, 46, "GAS", b, car.gas * 10, OK_C);
    if (act) {                                           /* demand marker on the GAS bar: amber, dark edges */
        const int mx = 114 + 77 * cmd / 100;
        fill_rect(mx - 1, 68, 5, 8, DIM);
        fill_rect(mx, 68, 3, 8, AMBER);
    }

    if (!pop) {
        /* indicators, 3 x 2 chips (lit = coloured, dim = off) */
        draw_chip(200, 46, "L",  car.arrows & ARROW_L,  GREEN_ON, CHIP_FG);
        draw_chip(227, 46, "R",  car.arrows & ARROW_R,  GREEN_ON, CHIP_FG);
        draw_chip(254, 46, "HB", car.lights & LIGHT_HB, ICE,      CHIP_FG);
        draw_chip(200, 60, "PB", car.lights & LIGHT_PB, RED,      WHITE);
        draw_chip(227, 60, "DR", car.doors & DOOR_ANY,  AMBER,    CHIP_FG);
        draw_chip(254, 60, "RV", car.reverse,           WHITE,    CHIP_FG);
    }
}

/* ---- configurable screens (layout in screens.h) ---- */

static const uint8_t param_dec[NPARAM] = {          /* decimals of the raw value, see the units in screens.h */
    [P_VOLT] = 1, [P_TRIP_KM] = 1, [P_FUEL_L] = 1, [P_L100] = 1, [P_LEFT_L] = 1, [P_LPH] = 1, [P_L100I] = 1,
    [P_TIMING] = 1, [P_STFT] = 1, [P_LTFT] = 1, [P_TRIM] = 1, [P_O2A] = 2, [P_O2B] = 2,
};

static bool obd_get(int q, int *v)
{
    const TickType_t t = obd.ts[q];
    if (!t || xTaskGetTickCount() - t >= pdMS_TO_TICKS(OBD_FRESH_MS)) return false;
    *v = obd.val[q];
    return true;
}

/* current value of a parameter in its raw unit; false = no data (shown as "--") */
static bool param_get(int id, int *v)
{
    const float km = trip_m / 1000.0f, hours = trip_ms / 3.6e6f, litres = trip_fuel * fuel_ul * 1e-6f;
    int a, b;
    switch (id) {
    case P_SPEED:     *v = car.speed;   return car_live;
    case P_RPM:       *v = car.rpm;     return car_live;
    case P_COOL:      *v = car.cool;    return car_live;
    case P_VOLT:      *v = car.volt10;  return car_live;
    case P_GAS:       *v = car.gas;     return car_live;
    case P_TRIP_KM:   *v = (int)(km * 10); return true;
    case P_TRIP_TIME: *v = (int)(trip_ms / 1000); return true;
    case P_AVG:       *v = hours > 0.0005f ? (int)(km / hours) : 0; return true;
    case P_MAX_SPEED: *v = max_speed;   return true;
    case P_MAX_RPM:   *v = max_rpm;     return true;
    case P_CLOCK:     *v = car.hh * 60 + car.mm; return true;
    case P_FUEL_L:    *v = (int)(litres * 10); return true;
    case P_L100:      *v = (int)(litres / km * 1000); return km >= 0.3f && litres > 0;
    case P_LEFT_L:    *v = car.fuel_lvl * 100 / 152; return car_live && car.fuel_lvl > 0;   /* ~15.2 units/L */
    case P_LPH:       *v = car.flow10;  return car_live && car.flow10 >= 0;
    case P_L100I:     *v = car.speed > 0 ? car.flow10 * 100 / car.speed : 0; return car_live && car.flow10 >= 0 && car.speed >= 10;
    case P_MAP:       *v = car.map;     return car_live;
    case P_IAT:       return obd_get(Q_IAT, v);
    case P_TIMING:    if (!obd_get(Q_TIMING, &a)) return false; *v = a * 5; return true;   /* half degrees -> 0.1 */
    case P_STFT:      return obd_get(Q_STFT, v);
    case P_LTFT:      return obd_get(Q_LTFT, v);
    case P_TRIM:      if (!obd_get(Q_STFT, &a) || !obd_get(Q_LTFT, &b)) return false; *v = a + b; return true;
    case P_O2A:       if (!obd_get(Q_O2A, &a)) return false; *v = a / 10; return true;      /* mV -> 0.01 V */
    case P_O2B:       if (!obd_get(Q_O2B, &a)) return false; *v = a / 10; return true;
    case P_LOAD:      return obd_get(Q_LOAD, v);
    case P_ECT:       return obd_get(Q_ECT, v);
    case P_THR:       return obd_get(Q_THR, v);
    case P_FSYS:      return obd_get(Q_FSYS, v);
    case P_DTC:       return obd_get(Q_DTC, v);
    default:          return false;
    }
}

static void param_text(int id, int v, char *b, size_t n)
{
    switch (id) {
    case P_TRIP_TIME: snprintf(b, n, "%d:%02d", v / 3600, v / 60 % 60); return;
    case P_CLOCK:     snprintf(b, n, "%02d:%02d", v / 60, v % 60); return;
    case P_FSYS:      snprintf(b, n, "%s", (v & 0x12) ? "CL" : "OL"); return;   /* 02/10 = closed loop */
    case P_DTC:       if (v & 0x100) snprintf(b, n, "%d MIL", v & 0x7F); else snprintf(b, n, "%d", v & 0x7F); return;
    default: break;
    }
    const int dec = param_dec[id];
    if (!dec) { snprintf(b, n, "%d", v); return; }
    const int p = dec == 1 ? 10 : 100, a = v < 0 ? -v : v;
    snprintf(b, n, "%s%d.%0*d", v < 0 ? "-" : "", a / p, dec, a % p);
}

static int param_level(int id, int v)                /* 0 ok, 1 warning (amber), 2 alarm (red) */
{
    const param_cfg_t *c = &PARAMS[id];
    if (id == P_DTC) v &= 0x7F;
    if (v < c->alarm_lo || v > c->alarm_hi) return 2;
    if (v < c->warn_lo || v > c->warn_hi) return 1;
    return 0;
}

/* the first SK_GRAPH screen can be re-pointed from the menu (item[0] and the window); the rest comes from screens.h */
static int graph_first(void)
{
    for (int i = 0; i < SCREEN_COUNT; i++) if (SCREENS[i].kind == SK_GRAPH) return i;
    return -1;
}

static int gitem(int scr, int k)
{
    return (k == 0 && g_param && scr == graph_first()) ? g_param : SCREENS[scr].item[k];
}

static int gspan(int scr)
{
    const int s = (g_span && scr == graph_first()) ? g_span : SCREENS[scr].span_s;
    return s ? s : 60;
}

/* bit mask over Q_* of the OBD answers the screen needs */
static uint16_t screen_pid_mask(int scr)
{
    uint16_t m = 0;
    if (scr < 0 || scr >= SCREEN_COUNT || SCREENS[scr].kind == SK_DRIVE) return 0;
    const int n = SCREENS[scr].kind == SK_GRAPH ? GSER : 6;
    for (int k = 0; k < n; k++) {
        switch (SCREENS[scr].kind == SK_GRAPH ? gitem(scr, k) : SCREENS[scr].item[k]) {
        case P_IAT:    m |= 1 << Q_IAT; break;
        case P_TIMING: m |= 1 << Q_TIMING; break;
        case P_STFT:   m |= 1 << Q_STFT; break;
        case P_LTFT:   m |= 1 << Q_LTFT; break;
        case P_TRIM:   m |= 1 << Q_STFT | 1 << Q_LTFT; break;
        case P_O2A:    m |= 1 << Q_O2A; break;
        case P_O2B:    m |= 1 << Q_O2B; break;
        case P_LOAD:   m |= 1 << Q_LOAD; break;
        case P_ECT:    m |= 1 << Q_ECT; break;
        case P_THR:    m |= 1 << Q_THR; break;
        case P_FSYS:   m |= 1 << Q_FSYS; break;
        case P_DTC:    m |= 1 << Q_DTC; break;
        default: break;
        }
    }
    return m;
}

static void draw_grid(const screen_cfg_t *sc)
{
    int cols = sc->cols ? sc->cols : 3, rows = sc->rows ? sc->rows : 1;
    while (cols * rows > 6) rows--;
    const int cell_w = LCD_W / cols, cell_h = LCD_H / rows;
    for (int k = 0; k < cols * rows; k++) {
        const int id = sc->item[k];
        if (id <= P_NONE || id >= NPARAM) continue;
        const int x = 4 + (k % cols) * cell_w, y = (k / cols) * cell_h;
        char b[16] = "--";
        int v = 0, lvl = 0;
        if (param_get(id, &v)) {
            param_text(id, v, b, sizeof(b));
            lvl = param_level(id, v);
        }
        int scale = sc->max_scale ? sc->max_scale : 3;
        while (scale > 1 && (text_w(b, scale) > cell_w - 8 || 7 + 4 + 7 * scale > cell_h - 2)) scale--;   /* must fit its cell */
        const int top = y + (cell_h - (7 + 4 + 7 * scale)) / 2;
        draw_text(x, top, 1, GRAY, PARAMS[id].label);
        draw_text(x, top + 7 + 4, scale, lvl == 2 ? RED : lvl == 1 ? AMBER : WHITE, b);
    }
    for (int i = 1; i < cols; i++) fill_rect(i * cell_w - 1, 2, 1, LCD_H - 4, DIM);
    for (int i = 1; i < rows; i++) fill_rect(4, i * cell_h, LCD_W - 8, 1, DIM);
    if (!car_live) draw_text(LCD_W - 4 - text_w("NO CAN", 1), 2, 1, RED, "NO CAN");
}

/* ---- graph screens (SK_GRAPH): ring of samples per screen ---- */
#define HIST_N   180                         /* plot width in px = samples in the window */
#define PLOT_X   100
#define NO_SAMPLE INT16_MIN
static int16_t hist[SCREEN_COUNT][GSER][HIST_N];   /* rings, newest at hist_head - 1 */
static uint16_t hist_head[SCREEN_COUNT];
static uint32_t hist_acc[SCREEN_COUNT];

static uint32_t graph_period_ms(int scr)
{
    return gspan(scr) * 1000u / HIST_N;
}

static void graph_clear(int scr)                             /* new parameter or window: the old curve means nothing */
{
    for (int j = 0; j < GSER; j++) for (int k = 0; k < HIST_N; k++) hist[scr][j][k] = NO_SAMPLE;
    hist_head[scr] = 0;
    hist_acc[scr] = 0;
}

/* called every frame with the frame time; records the shown parameter of every graph screen */
static void graph_step(uint32_t dt_ms)
{
    static bool inited;
    if (!inited) {                                           /* empty ring = no samples, not zeros */
        for (int i = 0; i < SCREEN_COUNT; i++) for (int j = 0; j < GSER; j++) for (int k = 0; k < HIST_N; k++) hist[i][j][k] = NO_SAMPLE;
        inited = true;
    }
    for (int i = 0; i < SCREEN_COUNT; i++) {
        if (SCREENS[i].kind != SK_GRAPH) continue;
        const uint32_t per = graph_period_ms(i);
        hist_acc[i] += dt_ms;
        while (hist_acc[i] >= per) {
            hist_acc[i] -= per;
            for (int j = 0; j < GSER; j++) {
                int v;
                const int id = gitem(i, j);
                hist[i][j][hist_head[i]] = (id > P_NONE && id < NPARAM && param_get(id, &v)) ? clampi(v, -32000, 32000) : NO_SAMPLE;
            }
            hist_head[i] = (hist_head[i] + 1) % HIST_N;
        }
    }
}

static void draw_graph1(int idx)
{
    const int id = gitem(idx, 0);
    const int py0 = 4, ph = LCD_H - 8;                       /* plot rows py0 .. py0+ph-1 */
    char b[16] = "--";
    int v = 0, lvl = 0;
    if (param_get(id, &v)) { param_text(id, v, b, sizeof(b)); lvl = param_level(id, v); }
    int scale = 3;
    while (scale > 1 && text_w(b, scale) > PLOT_X - 12) scale--;
    draw_text(4, 12, 1, GRAY, PARAMS[id].label);
    draw_text(4, 26, scale, lvl == 2 ? RED : lvl == 1 ? AMBER : WHITE, b);
    char sp[12];
    snprintf(sp, sizeof(sp), "%ds", gspan(idx));
    draw_text(4, LCD_H - 12, 1, GRAY, sp);

    int lo = INT_MAX, hi = INT_MIN;                          /* range of the window */
    for (int i = 0; i < HIST_N; i++) {
        const int s = hist[idx][0][i];
        if (s == NO_SAMPLE) continue;
        if (s < lo) lo = s;
        if (s > hi) hi = s;
    }
    fill_rect(PLOT_X - 4, py0, 1, ph, DIM);
    fill_rect(PLOT_X - 4, py0 + ph, LCD_W - PLOT_X, 1, DIM);
    if (lo > hi) {                                           /* nothing recorded yet */
        draw_text(PLOT_X + 8, LCD_H / 2 - 3, 1, GRAY, "NO DATA");
        if (!car_live) draw_text(4, 2, 1, RED, "NO CAN");
        return;
    }
    const int dec = param_dec[id], min_span = dec ? 20 : 10; /* do not blow noise up to full height */
    if (hi - lo < min_span) { const int mid = (hi + lo) / 2; lo = mid - min_span / 2; hi = lo + min_span; }
    const int pad = (hi - lo) / 10 + 1;
    lo -= pad; hi += pad;
    #define GY(val) (py0 + ph - 1 - ((val) - lo) * (ph - 1) / (hi - lo))
    const param_cfg_t *c = &PARAMS[id];
    const int lim[4] = { c->warn_lo, c->warn_hi, c->alarm_lo, c->alarm_hi };
    for (int k = 0; k < 4; k++) {                            /* dashed limit lines that fall inside the view */
        if (lim[k] == NO_LO || lim[k] == NO_HI || lim[k] <= lo || lim[k] >= hi) continue;
        const int y = GY(lim[k]);
        for (int x = PLOT_X; x < LCD_W - 2; x += 4) fill_rect(x, y, 2, 1, k >= 2 ? RED : AMBER);
    }
    int prev = NO_SAMPLE;
    for (int x = 0; x < HIST_N; x++) {                       /* oldest on the left */
        const int s = hist[idx][0][(hist_head[idx] + x) % HIST_N];
        if (s == NO_SAMPLE) { prev = NO_SAMPLE; continue; }
        const int y = GY(s), y0 = prev == NO_SAMPLE ? y : GY(prev);
        const int top = y < y0 ? y : y0, h = (y < y0 ? y0 - y : y - y0) + 1;
        fill_rect(PLOT_X + x, top, 1, h, param_level(id, s) == 2 ? RED : param_level(id, s) == 1 ? AMBER : WHITE);
        prev = s;
    }
    #undef GY
    char t[16];
    param_text(id, hi, t, sizeof(t));
    draw_text(LCD_W - 2 - text_w(t, 1), py0 + 1, 1, GRAY, t);
    param_text(id, lo, t, sizeof(t));
    draw_text(LCD_W - 2 - text_w(t, 1), py0 + ph - 8, 1, GRAY, t);
    if (!car_live) draw_text(4, 2, 1, RED, "NO CAN");
}

/* 2..3 parameters on one plot: each line is scaled to its own min..max (different units), colour = series */
static void draw_graph_multi(int idx, int n)
{
    const uint16_t col[GSER] = { WHITE, SER_B, SER_C };
    const int py0 = 4, ph = LCD_H - 8, row = LCD_H / n;
    for (int j = 0; j < n; j++) {                            /* left column: label + current value per series */
        const int id = gitem(idx, j);
        char b[16] = "--";
        int v = 0, lvl = 0;
        if (param_get(id, &v)) { param_text(id, v, b, sizeof(b)); lvl = param_level(id, v); }
        const int scale = n == 2 ? 3 : 2;
        const int top = j * row + (row - (7 + 3 + 7 * scale)) / 2;
        draw_text(4, top, 1, col[j], PARAMS[id].label);
        draw_text(4, top + 10, scale, lvl == 2 ? RED : lvl == 1 ? AMBER : col[j], b);
    }
    fill_rect(PLOT_X - 4, py0, 1, ph, DIM);
    fill_rect(PLOT_X - 4, py0 + ph, LCD_W - PLOT_X, 1, DIM);
    for (int j = 0; j < n; j++) {
        const int id = gitem(idx, j);
        int lo = INT_MAX, hi = INT_MIN;
        for (int i = 0; i < HIST_N; i++) {
            const int s = hist[idx][j][i];
            if (s == NO_SAMPLE) continue;
            if (s < lo) lo = s;
            if (s > hi) hi = s;
        }
        if (lo > hi) continue;
        const int min_span = param_dec[id] ? 20 : 10;
        if (hi - lo < min_span) { const int mid = (hi + lo) / 2; lo = mid - min_span / 2; hi = lo + min_span; }
        int prev = NO_SAMPLE;
        for (int x = 0; x < HIST_N; x++) {
            const int s = hist[idx][j][(hist_head[idx] + x) % HIST_N];
            if (s == NO_SAMPLE) { prev = NO_SAMPLE; continue; }
            const int y = py0 + ph - 1 - (s - lo) * (ph - 1) / (hi - lo);
            const int y0 = prev == NO_SAMPLE ? y : py0 + ph - 1 - (prev - lo) * (ph - 1) / (hi - lo);
            fill_rect(PLOT_X + x, y < y0 ? y : y0, 1, (y < y0 ? y0 - y : y - y0) + 1, col[j]);
            prev = s;
        }
    }
    char sp[12];
    snprintf(sp, sizeof(sp), "%ds", gspan(idx));
    fill_rect(LCD_W - 6 - text_w(sp, 1), 2, text_w(sp, 1) + 4, 9, DIM);      /* label over the lines */
    draw_text(LCD_W - 4 - text_w(sp, 1), 3, 1, GRAY, sp);
    if (!car_live) {
        fill_rect(LCD_W - 8 - text_w("NO CAN", 1), LCD_H - 12, text_w("NO CAN", 1) + 6, 9, DIM);
        draw_text(LCD_W - 5 - text_w("NO CAN", 1), LCD_H - 11, 1, RED, "NO CAN");
    }
}

static void draw_graph(int idx)
{
    int n = 0;                                               /* consecutive used slots */
    while (n < GSER && gitem(idx, n) > P_NONE && gitem(idx, n) < NPARAM) n++;
    if (n == 1) draw_graph1(idx);
    else if (n > 1) draw_graph_multi(idx, n);
}

static void draw_menu(void);
static volatile bool fb_dump;            /* 'F' over USB: print the next finished frame as hex (screenshot) */

static void fb_print(void)
{
    const uint8_t *m = (const uint8_t *)fb;              /* memory order = big-endian RGB565 (see rgb()) */
    printf("\nFBBEGIN %d %d\n", LCD_W, LCD_H);
    for (int y = 0; y < LCD_H; y++) {
        for (int i = 0; i < LCD_W * 2; i++) printf("%02x", m[y * LCD_W * 2 + i]);
        printf("\n");
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(5));                    /* let IDLE feed the watchdog while the USB FIFO drains */
    }
    printf("FBEND\n");
    fflush(stdout);
}

/* wait for the previous frame, draw the current screen, send */
static void render(void)
{
    xSemaphoreTake(trans_done, portMAX_DELAY);
    for (int y = 0; y < LCD_H; y++) {                    /* bright blue vertical gradient */
        uint16_t c = rgb(y * 5 / LCD_H, 45 + y * 50 / LCD_H, 130 + y * 80 / LCD_H);
        for (int x = 0; x < LCD_W; x++) fb[y * LCD_W + x] = c;
    }

    if (screen >= SCREEN_COUNT)                   draw_menu();
    else if (SCREENS[screen].kind == SK_GRID)     draw_grid(&SCREENS[screen]);
    else if (SCREENS[screen].kind == SK_GRAPH)    draw_graph(screen);
    else                                          draw_drive();

    if (fb_dump) { fb_dump = false; fb_print(); }
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel, 0, 0, LCD_W, LCD_H, fb));
}

static void car_step(uint32_t dt_ms)
{
    static uint8_t door_prev;
    const TickType_t now = xTaskGetTickCount();
    const uint8_t dm = doors_mask();
    if (dm != door_prev) { door_prev = dm; door_pop_until = now + pdMS_TO_TICKS(DOOR_POP_MS); }
    if (dm && car.speed >= 5) door_pop_until = now + pdMS_TO_TICKS(1000);   /* driving with something open: keep it up */
    if (!car_live) car = (car_t){ .hh = car.hh, .mm = car.mm };   /* no CAN: show nothing instead of stale values */
    const uint32_t fc = fuel_cnt, dc = fc - fuel_last;
    fuel_last = fc;
    if (car_live && fuel_seen) {
        trip_fuel += dc;
        tank_cnt += dc;
        const float inst = dc * fuel_ul * 1e-3f / (dt_ms * 1e-3f);   /* mL/s */
        flow_ml_s += (inst - flow_ml_s) * (dt_ms < 3000 ? dt_ms / 3000.0f : 1.0f);
        car.flow10 = (int)(flow_ml_s * 36.0f + 0.5f);    /* mL/s -> 0.1 L/h */
        static uint32_t save_ms;
        save_ms += dt_ms;
        if (save_ms >= 30000) { save_ms = 0; if (tank_cnt != tank_saved) fuel_save(); }
    } else {
        flow_ml_s = 0;
        car.flow10 = -1;
    }
    trip_ms += dt_ms;
    trip_m  += car.speed * (dt_ms / 3.6e6f) * 1000.0f;
    if (car.speed > max_speed) max_speed = car.speed;
    if (car.rpm > max_rpm) max_rpm = car.rpm;
}

static void cruise_cancel(const char *why)
{
    if (cc_active) {
        ESP_LOGI(TAG, "cruise OFF: %s", why);
        cc_evt_tick = xTaskGetTickCount(); cc_evt_on = false; cc_evt_any = true;
    }
    cc_active = false;
}

static void cruise_toggle(void)
{
    if (cc_active) { cruise_cancel("driver"); return; }
    if (!car_live) { ESP_LOGW(TAG, "cruise refused: no live CAN data"); return; }
    if (car.speed < CC_MIN_KMH || car.speed > CC_MAX_KMH) {
        ESP_LOGW(TAG, "cruise refused: speed %d outside %d..%d", car.speed, CC_MIN_KMH, CC_MAX_KMH);
        return;
    }
    cc_active = true;
    cc_evt_tick = xTaskGetTickCount(); cc_evt_on = true; cc_evt_any = true;
    cc_set = car.speed;
    cc_base = car.gas;                                   /* bumpless start from the current pedal */
    cc_i = 0;
    cc_cmd = cc_base;
    ESP_LOGI(TAG, "cruise ON: set %d km/h, base pedal %d%%", cc_set, car.gas);
}

static void cruise_adjust(int d)
{
    if (cc_active) cc_set = clampi(cc_set + d, CC_MIN_KMH, CC_MAX_KMH);
}

/* PI with anti-windup; every cancel condition is checked first, fail-safe = off */
static void cruise_step(float dt)
{
    if (!cc_active) return;
    if (!car_live) { cruise_cancel("CAN data lost"); return; }
    if (pdTICKS_TO_MS(xTaskGetTickCount() - last_speed) > CC_STALE_MS) { cruise_cancel("speed frame stale"); return; }
    if (car.brake) { cruise_cancel("brake pressed"); return; }
    if (car.speed < CC_MIN_KMH) { cruise_cancel("speed too low"); return; }
    if (car.speed > CC_MAX_KMH) { cruise_cancel("speed too high"); return; }
    if (car.speed < cc_set - CC_DROP_KMH) { cruise_cancel("speed dropped (brake/hill)"); return; }

    const float err = cc_set - car.speed;
    cc_i = cc_i + 0.5f * err * dt;
    if (cc_i > 30.0f) cc_i = 30.0f; else if (cc_i < -30.0f) cc_i = -30.0f;
    float cmd = cc_base + 2.0f * err + cc_i;
    cc_cmd = cmd < 0 ? 0 : cmd > 100 ? 100 : cmd;
}

/* raw frame dump for reverse engineering: 'D' over USB toggles; prints a frame only when its data changed */
typedef struct { uint8_t bus; uint8_t dlc; uint16_t id; uint8_t data[8]; } dump_msg_t;
static QueueHandle_t dump_q;
static volatile bool dump_on;

static void IRAM_ATTR dump_push_isr(char bus, uint32_t id, const uint8_t *d, int len)
{
    if (!dump_on) return;
    dump_msg_t m = { .bus = bus, .dlc = len, .id = id };
    memcpy(m.data, d, len);
    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(dump_q, &m, &woken);
    if (woken) portYIELD_FROM_ISR();
}

/* settings menu: turn = move (or change the value in edit mode), press = select/confirm, hold = back to DRIVE */
enum { MI_BRIGHT, MI_CC_STEP, MI_AUTOLOCK, MI_GRAPH_PARAM, MI_GRAPH_SPAN, MI_FUEL_CAL, MI_TANK, MI_TRIP_RESET, MI_DUMP, MI_EXIT, NMENU };
#define MENU_ROWS 4
static int menu_sel, menu_top;
static bool menu_edit;

static const int SPANS[] = { 0, 30, 60, 120, 300, 600 };   /* GR SPAN choices, s (0 = as in screens.h) */
#define NSPAN ((int)(sizeof(SPANS) / sizeof(SPANS[0])))

static void menu_leave(void)
{
    if (menu_edit) settings_save();
    menu_edit = false;
    screen = SCR_DRIVE;
    if (SCREENS[0].kind != SK_DRIVE) ESP_LOGE(TAG, "screens.h: SCREENS[0] must be the DRIVE screen");
}

static void menu_rotate(int steps)
{
    if (menu_edit) {                                     /* BRIGHT, CC STEP are editable */
        if (menu_sel == MI_BRIGHT) { bright = clampi(bright + 10 * steps, 10, 100); backlight_set(bright); }
        else if (menu_sel == MI_CC_STEP) cc_step = clampi(cc_step + steps, 1, 5);
        else if (menu_sel == MI_FUEL_CAL) fuel_ul = clampi(fuel_ul + 5 * steps, 20, 400);
        else if (menu_sel == MI_GRAPH_PARAM) { g_param = ((g_param + steps) % NPARAM + NPARAM) % NPARAM; graph_clear(graph_first()); }
        else if (menu_sel == MI_GRAPH_SPAN) {
            int i = 0;                                   /* position in the list, 0 = from screens.h */
            while (i < NSPAN && SPANS[i] != g_span) i++;
            i = i < NSPAN ? i : 0;
            g_span = SPANS[((i + steps) % NSPAN + NSPAN) % NSPAN];
            graph_clear(graph_first());
        }
    } else {
        menu_sel = ((menu_sel + steps) % NMENU + NMENU) % NMENU;
        if (menu_sel < menu_top) menu_top = menu_sel;
        if (menu_sel >= menu_top + MENU_ROWS) menu_top = menu_sel - MENU_ROWS + 1;
    }
}

static void menu_press(void)
{
    switch (menu_sel) {
    case MI_BRIGHT:
    case MI_CC_STEP:
    case MI_FUEL_CAL:
    case MI_GRAPH_PARAM:
    case MI_GRAPH_SPAN:
        if (graph_first() < 0 && (menu_sel == MI_GRAPH_PARAM || menu_sel == MI_GRAPH_SPAN)) break;   /* no graph screen */
        if (menu_edit) settings_save();
        menu_edit = !menu_edit;
        break;
    case MI_AUTOLOCK:
        al_on = !al_on;
        settings_save();
        break;
    case MI_TANK:
        tank_cnt = 0;
        fuel_save();
        break;
    case MI_TRIP_RESET:
        trip_fuel = 0;
        trip_m = 0; trip_ms = 0; max_speed = 0; max_rpm = 0;
        ESP_LOGI(TAG, "trip reset");
        break;
    case MI_DUMP:
        dump_on = !dump_on;
        break;
    case MI_EXIT:
        menu_leave();
        break;
    }
}

static void draw_menu(void)
{
    static const char *const label[NMENU] = { "BRIGHT", "CC STEP", "AUTO LOCK", "GR PARAM", "GR SPAN", "FUEL CAL", "TANK L", "TRIP RESET", "CAN DUMP", "EXIT" };
    char v[24];
    for (int r = 0; r < MENU_ROWS; r++) {
        const int i = menu_top + r;
        const int y = 2 + r * 18;
        const bool sel = i == menu_sel;
        if (sel) fill_rect(2, y, LCD_W - 14, 17, FORD_BLUE);
        draw_text(10, y + 2, 2, sel ? WHITE : GRAY, label[i]);
        v[0] = 0;
        if (i == MI_BRIGHT) snprintf(v, sizeof(v), "%d%%", bright);
        else if (i == MI_CC_STEP) snprintf(v, sizeof(v), "%d", cc_step);
        else if (i == MI_AUTOLOCK) snprintf(v, sizeof(v), "%s", al_on ? "ON" : "OFF");
        else if (i == MI_GRAPH_PARAM) snprintf(v, sizeof(v), "%s", graph_first() < 0 ? "-" : g_param ? PARAMS[g_param].label : "CFG");
        else if (i == MI_GRAPH_SPAN) { if (graph_first() < 0) snprintf(v, sizeof(v), "-"); else if (g_span) snprintf(v, sizeof(v), "%ds", g_span); else snprintf(v, sizeof(v), "CFG"); }
        else if (i == MI_FUEL_CAL) snprintf(v, sizeof(v), "%d.%03d", fuel_ul / 1000, fuel_ul % 1000);
        else if (i == MI_TANK) { const int t10 = (int)(tank_cnt * fuel_ul * 1e-5f); snprintf(v, sizeof(v), "%d.%d", t10 / 10, t10 % 10); }
        else if (i == MI_DUMP) snprintf(v, sizeof(v), "%s", dump_on ? "ON" : "OFF");
        draw_text(LCD_W - 18 - text_w(v, 2), y + 2, 2, sel && menu_edit ? AMBER : sel ? WHITE : GRAY, v);
    }
    const int th = 72 * MENU_ROWS / NMENU;               /* scroll thumb */
    fill_rect(LCD_W - 6, 2, 3, 72, DIM);
    fill_rect(LCD_W - 6, 2 + (72 - th) * menu_top / (NMENU - MENU_ROWS), 3, th, GRAY);
}

/* MS-CAN frame -> car state; runs in the TWAI rx ISR, so integer maths only (no FPU in ISRs) */
static void IRAM_ATTR can_decode(uint32_t id, const uint8_t *d, int len)
{
    switch (id) {
    case 0x201:                                          /* rpm, speed, gas pedal */
        if (len < 8) return;
        car.rpm   = d[0] << 8 | d[1];
        car.speed = (d[4] << 8 | d[5]) / 100;
        last_speed = xTaskGetTickCountFromISR();
        car.gas   = clampi((int)(((int64_t)(d[6] << 8 | d[7]) * 196294 - 25125632) / 100000000), 0, 100);
        break;
    case 0x360:
        if (len < 8) return;
        car.brake = d[5] & 0x08;                         /* d5: 60 released, 70 -> 78 pressed (checked on car) */
        if (fuel_seen) {
            const uint8_t step = (uint8_t)(d[7] - fuel_prev);
            if (step < 40) fuel_cnt += step;             /* a huge step = lost frames, not fuel */
        }
        fuel_prev = d[7];
        fuel_seen = true;
        break;
    case 0x499: if (len >= 6) car.fuel_lvl = d[4] << 8 | d[5]; break;
    case 0x420:
        if (len >= 1) car.cool = d[0] - 40;
        if (len >= 8) car.map = d[7];
        break;
    case 0x428: if (len >= 2) car.volt10 = d[1]; break;
    case 0x265: if (len >= 1) car.arrows = d[0]; break;
    case 0x286: if (len >= 1) car.lights = d[0]; break;
    case 0x433:
        if (len < 6) return;
        car.doors = d[0];
        car.reverse = d[3] & 0x02;
        car.locked = d[5] == 0x10;
        break;
    case 0x080:
        if (len >= 5) { car.hh = d[3]; car.mm = d[4]; }
        break;
    default:
        return;
    }
    can_seen = true;
    last_can = xTaskGetTickCountFromISR();
}

#define SEEN_N 256
static void dump_task(void *arg)
{
    static struct { uint8_t bus; uint16_t id; uint8_t dlc; uint8_t data[8]; bool used; } seen[SEEN_N];
    dump_msg_t m;
    while (1) {
        xQueueReceive(dump_q, &m, portMAX_DELAY);
        if (!dump_on) continue;
        unsigned h = (m.id * 31u + m.bus) % SEEN_N, i = h;
        while (seen[i].used && (seen[i].id != m.id || seen[i].bus != m.bus)) {
            i = (i + 1) % SEEN_N;
            if (i == h) break;                           /* table full: just print */
        }
        if (seen[i].used && seen[i].dlc == m.dlc && !memcmp(seen[i].data, m.data, m.dlc)) continue;
        seen[i].used = true; seen[i].bus = m.bus; seen[i].id = m.id; seen[i].dlc = m.dlc;
        memcpy(seen[i].data, m.data, 8);
        printf("%c %03X [%d]", m.bus, m.id, m.dlc);
        for (int k = 0; k < m.dlc; k++) printf(" %02X", m.data[k]);
        printf("\n");
    }
}

/* HS-CAN via MCP2515: dump only for now, no HS ids are decoded yet */
static void hs_task(void *arg)
{
    mcp_cfg_t cfg = { .host = SPI3_HOST, .sck = PIN_HS_SCK, .mosi = PIN_HS_MOSI, .miso = PIN_HS_MISO,
                      .cs = PIN_HS_CS, .intr = PIN_HS_INT, .bitrate = 500000, .tx = true };
    if (mcp2515_init(&cfg) != ESP_OK) {
        ESP_LOGW(TAG, "MCP2515 init failed, HS-CAN disabled");
        vTaskDelete(NULL);
    }
    uint32_t frames = 0, last_log = 0;
    mcp_frame_t f;
    /* OBD-II Mode 01 read-only polling (functional request 0x7DF, PCM answers 0x7E8), non-stop while CAN is alive:
       the PIDs of every configured screen (so graphs keep recording), those of the screen shown asked every other request */
    uint16_t all = 0;
    for (int i = 0; i < SCREEN_COUNT; i++) all |= screen_pid_mask(i);
    size_t pid_i = 0, bg_i = 0;
    bool turn_shown = true;
    TickType_t next_req = 0;
    int unanswered = 0;
    uint32_t n_sent = 0, n_ans = 0, n_busy = 0;
    while (1) {
        xSemaphoreTake(mcp2515_rx_sem(), pdMS_TO_TICKS(50));
        const TickType_t tn = xTaskGetTickCount();
        const uint16_t shown = screen_pid_mask(screen);
        if (all && car_live && (int32_t)(tn - next_req) >= 0) {
            const uint16_t mask = (turn_shown && shown) ? shown : all;
            size_t *idx = (turn_shown && shown) ? &pid_i : &bg_i;
            for (int t = 0; t < NQ && !(mask >> *idx & 1); t++) *idx = (*idx + 1) % NQ;
            const uint8_t rq[8] = { 0x02, 0x01, obd_pids[*idx], 0, 0, 0, 0, 0 };
            if (mcp2515_send(0x7DF, rq, 8)) {
                *idx = (*idx + 1) % NQ;
                turn_shown = !turn_shown;
                unanswered++;
                n_sent++;
            } else {
                n_busy++;
            }
            /* ~10 requests/s; a silent PCM (20 in a row without any answer) is polled once per 5 s only */
            next_req = tn + pdMS_TO_TICKS(unanswered > 20 ? 5000 : 100);
        }
        while (mcp2515_read(&f)) {
            frames++;
            if (f.id == 0x7E8 && !f.ext && !f.rtr && f.dlc >= 4 && f.data[1] == 0x41) {
                const int a = f.data[3];
                int q = 0;
                while (q < NQ && obd_pids[q] != f.data[2]) q++;
                if (q < NQ) {
                    switch (q) {
                    case Q_TIMING: obd.val[q] = a - 128; break;                   /* half degrees: a/2 - 64 */
                    case Q_STFT:
                    case Q_LTFT:   obd.val[q] = (a - 128) * 1000 / 128; break;    /* 0.1 % */
                    case Q_O2A:
                    case Q_O2B:    obd.val[q] = a * 5; break;                     /* mV, 5 per step */
                    case Q_LOAD:
                    case Q_THR:    obd.val[q] = a * 100 / 255; break;
                    case Q_IAT:
                    case Q_ECT:    obd.val[q] = a - 40; break;
                    case Q_DTC:    obd.val[q] = (a & 0x7F) | ((a & 0x80) ? 0x100 : 0); break;   /* count + MIL bit */
                    default:       obd.val[q] = a; break;                         /* fuel system status */
                    }
                    obd.ts[q] = tn;
                }
                unanswered = 0;
                n_ans++;
            }
            if (dump_on && !f.ext && !f.rtr && f.dlc) {
                dump_msg_t m = { .bus = 'H', .dlc = f.dlc, .id = f.id };
                memcpy(m.data, f.data, 8);
                xQueueSend(dump_q, &m, 0);
            }
        }
        uint32_t now = pdTICKS_TO_MS(xTaskGetTickCount());
        if (n_sent && now - last_log > 5000) {
            last_log = now;
            ESP_LOGI(TAG, "OBD sent=%lu answered=%lu txbusy=%lu unanswered=%d", (unsigned long)n_sent, (unsigned long)n_ans, (unsigned long)n_busy, unanswered);
        }
        if (dump_on && now - last_log > 5000) {
            last_log = now;
            ESP_LOGI(TAG, "HS frames=%lu overflows=%lu", (unsigned long)frames, (unsigned long)mcp2515_overflows());
        }
    }
}

#define GEM_TX_ID 0x726              /* GEM (body module) diagnostic request, reply on +8 */
#define GEM_RX_ID 0x72E
typedef struct { uint8_t d[8]; } gem_frame_t;
static QueueHandle_t gem_q;          /* GEM replies from the rx ISR to gem_task */
static twai_node_handle_t ms_node;

static bool IRAM_ATTR on_can_rx(twai_node_handle_t node, const twai_rx_done_event_data_t *ed, void *ctx)
{
    uint8_t d[8];
    twai_frame_t f = { .buffer = d, .buffer_len = sizeof(d) };
    if (twai_node_receive_from_isr(node, &f) == ESP_OK && !f.header.ide && !f.header.rtr)
    {
        int len = f.header.dlc > 8 ? 8 : f.header.dlc;
        can_decode(f.header.id, d, len);
        if (len) dump_push_isr('M', f.header.id, d, len);
        if (f.header.id == GEM_RX_ID && gem_q && len == 8) {
            gem_frame_t g;
            memcpy(g.d, d, 8);
            BaseType_t woken = pdFALSE;
            xQueueSendFromISR(gem_q, &g, &woken);
            return woken == pdTRUE;
        }
    }
    return false;
}

/* listen-only (default): no TX, no ACK — cannot disturb the car's bus.
 * Normal mode only for the duration of a manual 'G' (GEM read), see gem_task() */
static bool can_open(bool listen_only)
{
    twai_onchip_node_config_t cfg = {
        .io_cfg = { .tx = PIN_CAN_TX, .rx = PIN_CAN_RX, .quanta_clk_out = -1, .bus_off_indicator = -1 },
        .bit_timing = { .bitrate = 125000 },
        .fail_retry_cnt = listen_only ? 0 : 3,
        .tx_queue_depth = 2,
        .flags = { .enable_listen_only = listen_only } };
    twai_event_callbacks_t cbs = { .on_rx_done = on_can_rx };
    if (twai_new_node_onchip(&cfg, &ms_node) != ESP_OK ||
        twai_node_register_event_callbacks(ms_node, &cbs, NULL) != ESP_OK ||
        twai_node_enable(ms_node) != ESP_OK) {
        ESP_LOGW(TAG, "TWAI init failed, no MS-CAN data");
        return false;
    }
    return true;
}

static void can_init(void)
{
    can_open(true);
}

/* GEM As-Built READ ONLY (ISO 15765-2 / KWP2000, protocol notes in docs/): 10 87, then 21 01/02/03 on 0x726.
 * Never sends 3B (write), 11 (reset) or anything else. Started by 'G' over USB; refused unless the car is
 * standing with the handbrake on. The raw replies are logged and the first successful read is kept in NVS as a backup. */
static volatile bool gem_busy;

static bool gem_send(const uint8_t *d, int len)
{
    uint8_t b[8] = { 0 };
    memcpy(b, d, len);
    twai_frame_t f = { .header = { .id = GEM_TX_ID, .dlc = 8 }, .buffer = b, .buffer_len = 8 };
    return twai_node_transmit(ms_node, &f, 100) == ESP_OK && twai_node_transmit_wait_all_done(ms_node, 300) == ESP_OK;
}

/* one request, one (possibly multi-frame) answer; returns its length or -1 */
static int gem_request(const uint8_t *req, int rlen, uint8_t *out, int cap)
{
    uint8_t sf[8] = { (uint8_t)rlen };
    memcpy(sf + 1, req, rlen);
    xQueueReset(gem_q);
    if (!gem_send(sf, 1 + rlen)) return -1;

    int total = 0, have = 0, next_sn = 1;
    TickType_t end = xTaskGetTickCount() + pdMS_TO_TICKS(3000);
    gem_frame_t g;
    while (xTaskGetTickCount() < end) {
        if (xQueueReceive(gem_q, &g, pdMS_TO_TICKS(500)) != pdTRUE) continue;
        const int pci = g.d[0] >> 4;
        if (pci == 0 && total == 0) {                         /* single frame */
            const int n = g.d[0] & 0x0F;
            if (n >= 3 && g.d[1] == 0x7F && g.d[3] == 0x78) { end = xTaskGetTickCount() + pdMS_TO_TICKS(3000); continue; }   /* responsePending */
            if (n > cap || n > 7) return -1;
            memcpy(out, g.d + 1, n);
            return n;
        } else if (pci == 1 && total == 0) {                  /* first frame: ask for the rest */
            total = (g.d[0] & 0x0F) << 8 | g.d[1];
            if (total > cap) return -1;
            have = total < 6 ? total : 6;
            memcpy(out, g.d + 2, have);
            const uint8_t fc[3] = { 0x30, 0x00, 0x00 };
            if (!gem_send(fc, 3)) return -1;
        } else if (pci == 2 && total) {                       /* consecutive frame */
            if ((g.d[0] & 0x0F) != (next_sn & 0x0F)) return -1;
            next_sn++;
            const int n = total - have < 7 ? total - have : 7;
            memcpy(out + have, g.d + 1, n);
            have += n;
            if (have >= total) return total;
        }
    }
    return -1;
}

static void gem_hex(char *dst, size_t cap, const uint8_t *d, int n)
{
    dst[0] = 0;
    for (int i = 0; i < n && strlen(dst) + 4 < cap; i++) snprintf(dst + strlen(dst), cap - strlen(dst), "%02X ", d[i]);
}

static void gem_task(void *arg)
{
    const int mode = (int)(intptr_t)arg;        /* 0 'G' read only, 1 'W' enable lock-at-movement, 2 'E' ECUReset of the GEM */
    const bool write = mode == 1, reset = mode == 2;
    if (!car_live || car.speed != 0 || !(car.lights & LIGHT_PB)) {
        ESP_LOGW(TAG, "GEM read refused: need live CAN, speed 0 and the handbrake on");
        gem_busy = false;
        vTaskDelete(NULL);
    }
    ESP_LOGW(TAG, "GEM %s: MS-CAN switches to normal mode and TRANSMITS to 0x%X", write ? "WRITE" : reset ? "RESET" : "read", GEM_TX_ID);
    twai_node_disable(ms_node);
    twai_node_delete(ms_node);
    bool ok = can_open(false);

    uint8_t r[24];
    char hx[96];
    if (ok) {
        const uint8_t start[2] = { 0x10, 0x87 };
        int n = gem_request(start, 2, r, sizeof(r));
        gem_hex(hx, sizeof(hx), r, n > 0 ? n : 0);
        ESP_LOGI(TAG, "GEM 10 87 -> %s", n > 0 ? hx : "no answer");
        ok = n >= 2 && r[0] == 0x50;
    }
    if (write && ok) {
        /* LID02 = As-Built bytes 5..9, so byte 7 is payload index 2 (r[4]); mask 0x80 = lock at start of movement.
         * Only that one bit is changed, and only if the module still holds exactly the bytes saved as backup. */
        const uint8_t rq[2] = { 0x21, 0x02 };
        uint8_t bk[8] = { 0 };
        size_t bk_len = sizeof(bk);
        nvs_handle_t h;
        const bool have_bk = nvs_open("obc", NVS_READONLY, &h) == ESP_OK && nvs_get_blob(h, "gem2", bk, &bk_len) == ESP_OK;
        if (have_bk) nvs_close(h);
        int n = gem_request(rq, 2, r, sizeof(r));
        gem_hex(hx, sizeof(hx), r, n > 0 ? n : 0);
        ESP_LOGI(TAG, "GEM 21 02 -> %s", n > 0 ? hx : "no answer");
        if (n != 7 || r[0] != 0x61 || r[1] != 0x02) {
            ESP_LOGE(TAG, "GEM write aborted: unexpected LID02 answer");
        } else if (!have_bk || (int)bk_len != n || memcmp(bk, r, n) != 0) {
            ESP_LOGE(TAG, "GEM write aborted: LID02 differs from the NVS backup (or no backup), nothing written");
        } else if (r[4] & 0x80) {
            ESP_LOGI(TAG, "GEM: lock at start of movement is already ON, nothing to do");
        } else {
            uint8_t wr[7] = { 0x3B, 0x02, r[2], r[3], (uint8_t)(r[4] | 0x80), r[5], r[6] };
            gem_hex(hx, sizeof(hx), wr, 7);
            ESP_LOGW(TAG, "GEM WRITE: %s", hx);
            n = gem_request(wr, 7, r, sizeof(r));
            gem_hex(hx, sizeof(hx), r, n > 0 ? n : 0);
            ESP_LOGI(TAG, "GEM 3B 02 -> %s", n > 0 ? hx : "no answer");
            const bool acked = n >= 2 && r[0] == 0x7B && r[1] == 0x02;
            n = gem_request(rq, 2, r, sizeof(r));
            gem_hex(hx, sizeof(hx), r, n > 0 ? n : 0);
            ESP_LOGI(TAG, "GEM verify 21 02 -> %s", n > 0 ? hx : "no answer");
            if (acked && n == 7 && r[4] == (bk[4] | 0x80) && !memcmp(r + 2, wr + 2, 5)) {
                ESP_LOGW(TAG, "GEM write VERIFIED: lock at start of movement is ON");
            } else {
                ESP_LOGE(TAG, "GEM write NOT verified, restoring the backup");
                uint8_t rs[7] = { 0x3B, 0x02, bk[2], bk[3], bk[4], bk[5], bk[6] };
                n = gem_request(rs, 7, r, sizeof(r));
                gem_hex(hx, sizeof(hx), r, n > 0 ? n : 0);
                ESP_LOGW(TAG, "GEM restore 3B 02 -> %s", n > 0 ? hx : "no answer");
                ok = false;
            }
        }
    }
    if (reset && ok) {
        /* 11 01 = ECUReset, so the module re-reads its As-Built; it reboots in a few seconds */
        const uint8_t rst[2] = { 0x11, 0x01 };
        int n = gem_request(rst, 2, r, sizeof(r));
        gem_hex(hx, sizeof(hx), r, n > 0 ? n : 0);
        ESP_LOGW(TAG, "GEM 11 01 -> %s", n > 0 ? hx : "no answer");
        ok = n >= 2 && r[0] == 0x51;
        vTaskDelay(pdMS_TO_TICKS(4000));
        if (ok) {
            const uint8_t st[2] = { 0x10, 0x87 }, rq[2] = { 0x21, 0x02 };
            gem_request(st, 2, r, sizeof(r));
            n = gem_request(rq, 2, r, sizeof(r));
            gem_hex(hx, sizeof(hx), r, n > 0 ? n : 0);
            ESP_LOGI(TAG, "GEM after reset 21 02 -> %s", n > 0 ? hx : "no answer");
            ok = n == 7 && r[0] == 0x61;
        }
    }
    for (int lid = 1; ok && mode == 0 && lid <= 3; lid++) {
        const uint8_t rq[2] = { 0x21, (uint8_t)lid };
        int n = gem_request(rq, 2, r, sizeof(r));
        gem_hex(hx, sizeof(hx), r, n > 0 ? n : 0);
        ESP_LOGI(TAG, "GEM 21 %02X -> %s", lid, n > 0 ? hx : "no answer");
        if (n < 2 || r[0] != 0x61 || r[1] != lid) { ok = false; break; }
        char key[8];
        snprintf(key, sizeof(key), "gem%d", lid);
        nvs_handle_t h;
        size_t sz = 0;
        if (nvs_open("obc", NVS_READWRITE, &h) == ESP_OK) {
            if (nvs_get_blob(h, key, NULL, &sz) != ESP_OK) {   /* keep the FIRST read as the backup */
                nvs_set_blob(h, key, r, n);
                nvs_commit(h);
                ESP_LOGI(TAG, "GEM LID %02X saved to NVS as backup (obc/%s)", lid, key);
            }
            nvs_close(h);
        }
    }
    twai_node_disable(ms_node);
    twai_node_delete(ms_node);
    can_open(true);
    ESP_LOGW(TAG, "GEM read %s; MS-CAN back to listen-only", ok ? "done" : "FAILED");
    gem_busy = false;
    vTaskDelete(NULL);
}

/* HW-040 quadrature decoder: both edges of CLK/DT, see ENC_TRANS */
static volatile int enc_acc;

static void IRAM_ATTR on_enc(void *arg)
{
    static const int8_t tab[16] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 };
    static uint8_t st;
    st = ((st << 2) | (gpio_get_level(PIN_ENC_CLK) << 1) | gpio_get_level(PIN_ENC_DT)) & 0x0f;
    enc_acc += tab[st];
}

static void enc_init(void)
{
    gpio_config_t io = { .pin_bit_mask = (1ULL << PIN_ENC_CLK) | (1ULL << PIN_ENC_DT), .mode = GPIO_MODE_INPUT,
                         .pull_up_en = GPIO_PULLUP_ENABLE, .intr_type = GPIO_INTR_ANYEDGE };
    ESP_ERROR_CHECK(gpio_config(&io));
    gpio_config_t sw = { .pin_bit_mask = 1ULL << PIN_ENC_SW, .mode = GPIO_MODE_INPUT,
                         .pull_up_en = GPIO_PULLUP_ENABLE };
    ESP_ERROR_CHECK(gpio_config(&sw));
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_ENC_CLK, on_enc, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_ENC_DT, on_enc, NULL));
}

void app_main(void)
{
    trans_done = xSemaphoreCreateBinary();
    xSemaphoreGive(trans_done);

    fb = heap_caps_malloc(FB_PIXELS * 2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    assert(fb);
    car = (car_t){ .flow10 = -1 };

    gpio_config_t btn = { .pin_bit_mask = 1ULL << PIN_BTN, .mode = GPIO_MODE_INPUT,
                          .pull_up_en = GPIO_PULLUP_ENABLE };
    ESP_ERROR_CHECK(gpio_config(&btn));
    enc_init();

    usb_serial_jtag_driver_config_t usb_cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_cfg));

    settings_load();
    backlight_init(0);
    lcd_init();
    dump_q = xQueueCreate(128, sizeof(dump_msg_t));
    xTaskCreate(dump_task, "dump", 3072, NULL, 3, NULL);
    gem_q = xQueueCreate(8, sizeof(gem_frame_t));
    can_init();
    xTaskCreate(hs_task, "hs_can", 4096, NULL, 4, NULL);

    render();                                            /* first frame before the light comes up */
    for (int d = 0; d <= bright; d += 5) {               /* fade-in backlight up to the saved level */
        backlight_set(d);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    backlight_set(bright);
    ESP_LOGI(TAG, "ready; MS-CAN listen-only 125k on RX=%d, screens DRIVE/TRIP/FUEL, BOOT or 'N' = next, 'D' = toggle raw CAN dump, hold BOOT or 'S' = cruise set/cancel, '+'/'-' = set speed, encoder: turn = screen, click = cruise set/cancel, held + turn = set speed, hold 5 s = menu", PIN_CAN_RX);

    char line[16];
    int n = 0;
    bool btn_prev = false, long_done = false;
    TickType_t press_t = 0;
    TickType_t last_frame = xTaskGetTickCount();

    while (1) {
        uint8_t rx[32];
        int got = usb_serial_jtag_read_bytes(rx, sizeof(rx), pdMS_TO_TICKS(20));
        TickType_t now = xTaskGetTickCount();

        for (int i = 0; i < got; i++) {
            char c = (char)rx[i];
            if (c == '\n') {
                line[n] = 0;
                if (line[0] == 'N' && screen != SCR_MENU) screen = (screen + 1) % NCYCLE;
                else if (line[0] == 'P') { ui_preview = (ui_preview + 1) % 4; ui_preview_tick = now; }
                else if (line[0] == 'F') fb_dump = true;
                else if (line[0] == 'O') ui_doors = (ui_doors + 1) % 6;
                else if (line[0] == 'S') cruise_toggle();
                else if (line[0] == '+') cruise_adjust(cc_step);
                else if (line[0] == '-') cruise_adjust(-cc_step);
                else if ((line[0] == 'G' || line[0] == 'W' || line[0] == 'E') && !gem_busy) { gem_busy = true; xTaskCreate(gem_task, "gem", 5120, (void *)(intptr_t)(line[0] == 'W' ? 1 : line[0] == 'E' ? 2 : 0), 4, NULL); }
                else if (line[0] == 'D') {
                    dump_on = !dump_on;
                    ESP_LOGI(TAG, "dump %s", dump_on ? "ON (M=MS-CAN, H=HS-CAN, changes only)" : "OFF");
                }
                n = 0;
            } else if (c != '\r' && n < (int)sizeof(line) - 1) {
                line[n++] = c;
            }
        }

        int steps = enc_acc / ENC_TRANS;                 /* transitions per detent */
        if (steps) {
            enc_acc -= steps * ENC_TRANS;
            if (btn_prev) long_done = true;              /* turned while held: not a click, not a 5 s hold */
            if (screen == SCR_MENU) menu_rotate(steps);
            else if (btn_prev && cc_active) cruise_adjust(cc_step * steps);   /* held + turn: set speed */
            else screen = ((screen + steps) % NCYCLE + NCYCLE) % NCYCLE;
        }

        bool pressed = gpio_get_level(PIN_BTN) == 0 || gpio_get_level(PIN_ENC_SW) == 0;
        if (pressed && !btn_prev) { press_t = now; long_done = false; }
        uint32_t held = pressed ? pdTICKS_TO_MS(now - press_t) : 0;
        if (pressed && !long_done) {
            if (screen == SCR_MENU && held >= LONG_MS) {
                long_done = true;                        /* hold in the menu: back to DRIVE */
                menu_leave();
            } else if (screen != SCR_MENU && held >= MENU_MS) {
                long_done = true;                        /* 5 s hold: open the settings menu */
                menu_sel = 0;
                menu_edit = false;
                screen = SCR_MENU;
            }
        }
        if (!pressed && btn_prev && !long_done) {
            if (screen == SCR_MENU) menu_press();
            else cruise_toggle();                        /* click (released before 5 s): cruise set/cancel */
        }
        btn_prev = pressed;

        car_live = can_seen && (now - last_can) < pdMS_TO_TICKS(STALE_MS);

        uint32_t dt = pdTICKS_TO_MS(now - last_frame);
        if (dt >= FRAME_MS) {
            last_frame = now;
            car_step(dt);
            graph_step(dt);
            cruise_step(dt / 1000.0f);
            render();
        }
    }
}
