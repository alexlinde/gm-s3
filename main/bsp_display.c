#include "bsp_display.h"

#include <stdatomic.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lvgl.h"

#define BSP_LCD_SPI_HOST     SPI3_HOST
#define BSP_LCD_MOSI         GPIO_NUM_11
#define BSP_LCD_SCLK         GPIO_NUM_12
#define BSP_LCD_DC           GPIO_NUM_7
#define BSP_LCD_RST          GPIO_NUM_6
#define BSP_LCD_BL           GPIO_NUM_14
#define BSP_LCD_H_RES        240
#define BSP_LCD_V_RES        240
/* 40 MHz is the conservative ST7789 sweet spot for this board: signal
 * integrity over the flying-lead wiring is marginal at 80 MHz, and at
 * 80 MHz under sustained load (WiFi + Octal-PSRAM at 80 MHz + cJSON/TLS)
 * the panel IO `on_color_trans_done` ISR occasionally stops firing,
 * leaving LVGL's `wait_for_flushing` busy-spinning until the task
 * watchdog resets the device. 40 MHz is well within ST7789 spec and a
 * 240x240 RGB565 frame still pushes in ~6 ms, plenty fast for the
 * single-screen UX here. */
#define BSP_LCD_SPI_FREQ_HZ  (40 * 1000 * 1000)
#define BSP_LCD_CMD_BITS     8
#define BSP_LCD_PARAM_BITS   8
#define BSP_LCD_SPI_MODE     3

/* Backlight PWM. 10-bit resolution (1024 steps) for smooth low-end dimming;
 * the panel's backlight net is active-LOW, but we invert at the LEDC output
 * (flags.output_invert) so duty maps forward: 0 = off, max = full. */
#define BSP_BL_LEDC_MODE     LEDC_LOW_SPEED_MODE
#define BSP_BL_LEDC_TIMER    LEDC_TIMER_0
#define BSP_BL_LEDC_CHANNEL  LEDC_CHANNEL_0
#define BSP_BL_LEDC_RES      LEDC_TIMER_10_BIT
#define BSP_BL_DUTY_MAX      ((1u << 10) - 1u)  /* 1023 */
#define BSP_BL_FREQ_HZ       5000

static const char *TAG = "bsp_display";

/* ---------------------------------------------------------------------------
 * Flush-health instrumentation.
 *
 * A flush that never completes is the classic way this board wedges: LVGL
 * blocks in `wait_for_flushing()` until the panel IO ISR reports the SPI
 * transfer done, and with the stock esp_lvgl_port setup (no `flush_wait_cb`)
 * that wait is a `while(disp->flushing);` busy-spin which starves IDLE ->
 * task watchdog if the ISR never fires. We install a bounded wait_cb below,
 * but we still want visibility: these hooks expose how long the in-progress
 * wait has been pending, for any off-thread observer (nothing samples them
 * today; bsp_display_get_flush_stats is the hook).
 *
 * Atomics: flush events fire on the LVGL render thread; the sampler reads
 * them from an esp_timer task. uint32 wraparound is fine for counts; the
 * pending-us field uses a single-writer/multi-reader pattern with
 * memory_order_relaxed which is enough since we're only ever asking
 * "approximately how long has the current wait been outstanding". */

static _Atomic uint32_t s_flush_starts;
static _Atomic uint32_t s_flush_finishes;
static _Atomic uint32_t s_wait_starts;
static _Atomic uint32_t s_wait_finishes;
static _Atomic int64_t  s_wait_start_us;   /* 0 when not inside a wait */

static void on_flush_event(lv_event_t *e)
{
    switch (lv_event_get_code(e)) {
    case LV_EVENT_FLUSH_START:
        atomic_fetch_add_explicit(&s_flush_starts, 1, memory_order_relaxed);
        break;
    case LV_EVENT_FLUSH_FINISH:
        atomic_fetch_add_explicit(&s_flush_finishes, 1, memory_order_relaxed);
        break;
    case LV_EVENT_FLUSH_WAIT_START:
        atomic_store_explicit(&s_wait_start_us,
                              esp_timer_get_time(), memory_order_relaxed);
        atomic_fetch_add_explicit(&s_wait_starts, 1, memory_order_relaxed);
        break;
    case LV_EVENT_FLUSH_WAIT_FINISH:
        atomic_store_explicit(&s_wait_start_us, 0, memory_order_relaxed);
        atomic_fetch_add_explicit(&s_wait_finishes, 1, memory_order_relaxed);
        break;
    default:
        break;
    }
}

void bsp_display_get_flush_stats(bsp_display_flush_stats_t *out)
{
    if (!out) return;
    out->flush_starts   = atomic_load_explicit(&s_flush_starts,   memory_order_relaxed);
    out->flush_finishes = atomic_load_explicit(&s_flush_finishes, memory_order_relaxed);
    out->wait_starts    = atomic_load_explicit(&s_wait_starts,    memory_order_relaxed);
    out->wait_finishes  = atomic_load_explicit(&s_wait_finishes,  memory_order_relaxed);
    int64_t ws = atomic_load_explicit(&s_wait_start_us, memory_order_relaxed);
    out->pending_us = ws ? (esp_timer_get_time() - ws) : 0;
}

/* ---------------------------------------------------------------------------
 * SPI-tx-failure-tolerant flush waiter.
 *
 * The default esp_lvgl_port flow for SPI panel IO leaves LVGL's flush_wait_cb
 * unset and relies on the panel-IO `on_color_trans_done` ISR to clear
 * `disp->flushing` via lv_disp_flush_ready(). When `esp_lcd_panel_draw_bitmap`
 * fails (e.g. spi_master::setup_dma_priv_buffer returns ENOMEM while bouncing
 * a non-DMA draw buffer), the transaction is never queued, the ISR never
 * fires, and LVGL falls into the `while(disp->flushing);` busy-spin in
 * lv_refr.c -> task watchdog.
 *
 * We replace that with a semaphore handshake, split strictly in two:
 *
 *   - `on_panel_io_color_done` (ISR) ONLY gives the binary semaphore. It must
 *     NOT call lv_display_flush_ready(). lv_refr.c's wait_for_flushing() runs
 *     `if (disp->flushing) { flush_wait_cb(disp); disp->flushing = 0; }`, so
 *     clearing `flushing` from the ISR makes LVGL skip the wait for every
 *     flush that finished early, orphaning that flush's token. The next
 *     genuinely in-flight flush is then satisfied instantly by the stale
 *     token -- LVGL renders into a buffer GDMA is still reading -- and the
 *     one-token-ahead desync is permanent once it starts, which also silently
 *     defeats the timeout below.
 *   - `flush_wait_cb` (LVGL thread) takes exactly one token and returns;
 *     LVGL clears `disp->flushing` itself (lv_refr.c:1438).
 *
 * Give/take stays 1:1: draw_buf_flush() sets `flushing = 1` exactly once per
 * flush_cb and, since we are double-buffered, always runs wait_for_flushing()
 * before the next one; esp_lcd_panel_io_spi arms the done callback only on
 * the final chunk of a draw_bitmap, so one successful flush == one give ==
 * one take.
 *
 * The take is bounded by FLUSH_TIMEOUT_MS so a draw_bitmap that never queued
 * cannot wedge us: we count it and return, LVGL clears `flushing`, and the
 * next frame renders. We accept potential one-frame tearing in exchange for
 * not wedging the device.
 *
 * Ordering requirement: flush_wait_cb must be installed before this ISR
 * callback can fire for the first time, otherwise wait_for_flushing() takes
 * its no-wait_cb branch (`while(disp->flushing);`) with nothing left to clear
 * `flushing`. bsp_display_init() installs both under the LVGL lock.
 * ------------------------------------------------------------------------- */

#define FLUSH_TIMEOUT_MS 200

static SemaphoreHandle_t s_flush_done_sem;
static _Atomic uint32_t  s_flush_timeouts;
static _Atomic uint32_t  s_flush_late_arrivals;

static bool IRAM_ATTR on_panel_io_color_done(esp_lcd_panel_io_handle_t io,
                                              esp_lcd_panel_io_event_data_t *e,
                                              void *user_ctx)
{
    BaseType_t hpw = pdFALSE;
    if (s_flush_done_sem) {
        /* Give only -- never lv_display_flush_ready() from here; see the
         * contract above. Binary sem: a give while count==1 is a no-op, which
         * is exactly what we want if a previous wait_cb timed out and the ISR
         * finally fired after we'd already moved on. */
        xSemaphoreGiveFromISR(s_flush_done_sem, &hpw);
    }
    return hpw == pdTRUE;
}

static void flush_wait_cb(lv_display_t *disp)
{
    (void)disp;
    if (!s_flush_done_sem) return;
    if (xSemaphoreTake(s_flush_done_sem, pdMS_TO_TICKS(FLUSH_TIMEOUT_MS)) == pdTRUE) {
        return;
    }
    /* Timeout: the on_color_trans_done ISR didn't fire within FLUSH_TIMEOUT_MS.
     * Most likely the SPI transaction was never queued because
     * esp_lcd_panel_draw_bitmap returned an error; the ISR will never come.
     * We return anyway -- LVGL clears `disp->flushing` for us -- so the next
     * frame can render. Counter is exposed via bsp_display_flush_timeouts. */
    atomic_fetch_add_explicit(&s_flush_timeouts, 1, memory_order_relaxed);

    /* Best-effort late-arrival drain: if the ISR fires within the next tick
     * (race window), consume it now so the next flush starts clean. */
    if (xSemaphoreTake(s_flush_done_sem, 0) == pdTRUE) {
        atomic_fetch_add_explicit(&s_flush_late_arrivals, 1, memory_order_relaxed);
    }
}

uint32_t bsp_display_flush_timeouts(void)
{
    return atomic_load_explicit(&s_flush_timeouts, memory_order_relaxed);
}

uint32_t bsp_display_flush_late_arrivals(void)
{
    return atomic_load_explicit(&s_flush_late_arrivals, memory_order_relaxed);
}

static esp_err_t bsp_backlight_init(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = BSP_BL_LEDC_MODE,
        .duty_resolution = BSP_BL_LEDC_RES,
        .timer_num       = BSP_BL_LEDC_TIMER,
        .freq_hz         = BSP_BL_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_cfg), TAG, "LEDC timer config failed");

    ledc_channel_config_t ch_cfg = {
        .gpio_num   = BSP_LCD_BL,
        .speed_mode = BSP_BL_LEDC_MODE,
        .channel    = BSP_BL_LEDC_CHANNEL,
        .timer_sel  = BSP_BL_LEDC_TIMER,
        .duty       = 0,                 /* start off until panel is ready */
        .hpoint     = 0,
        .flags      = { .output_invert = 1 },
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ch_cfg), TAG, "LEDC channel config failed");
    return ESP_OK;
}

esp_err_t bsp_display_set_backlight_percent(int percent)
{
    if (percent < 0)   percent = 0;
    if (percent > 100) percent = 100;

    /* Forward map (thanks to output_invert above): 0% -> 0, 100% -> DUTY_MAX. */
    uint32_t duty = ((uint32_t)percent * BSP_BL_DUTY_MAX + 50) / 100;
    ESP_RETURN_ON_ERROR(
        ledc_set_duty(BSP_BL_LEDC_MODE, BSP_BL_LEDC_CHANNEL, duty),
        TAG, "Set duty failed");
    ESP_RETURN_ON_ERROR(
        ledc_update_duty(BSP_BL_LEDC_MODE, BSP_BL_LEDC_CHANNEL),
        TAG, "Update duty failed");
    return ESP_OK;
}

esp_err_t bsp_display_init(void)
{
    ESP_LOGI(TAG, "Backlight init (off during setup)...");
    ESP_RETURN_ON_ERROR(bsp_backlight_init(), TAG, "Backlight init failed");

    ESP_LOGI(TAG, "SPI bus init (SPI3_HOST, MOSI=%d, SCLK=%d)...",
             BSP_LCD_MOSI, BSP_LCD_SCLK);
    const spi_bus_config_t bus_cfg = {
        .sclk_io_num     = BSP_LCD_SCLK,
        .mosi_io_num     = BSP_LCD_MOSI,
        .miso_io_num     = -1,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = BSP_LCD_H_RES * 80 * sizeof(uint16_t),
    };
    ESP_RETURN_ON_ERROR(
        spi_bus_initialize(BSP_LCD_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO),
        TAG, "SPI bus init failed");

    ESP_LOGI(TAG, "Panel IO init (DC=%d, CS=-1, %d Hz, SPI Mode %d)...",
             BSP_LCD_DC, BSP_LCD_SPI_FREQ_HZ, BSP_LCD_SPI_MODE);
    esp_lcd_panel_io_handle_t io_handle = NULL;
    const esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num     = BSP_LCD_DC,
        .cs_gpio_num     = -1,
        .pclk_hz         = BSP_LCD_SPI_FREQ_HZ,
        .lcd_cmd_bits    = BSP_LCD_CMD_BITS,
        .lcd_param_bits  = BSP_LCD_PARAM_BITS,
        .spi_mode        = BSP_LCD_SPI_MODE,
        .trans_queue_depth = 10,
    };
    ESP_RETURN_ON_ERROR(
        esp_lcd_new_panel_io_spi(BSP_LCD_SPI_HOST, &io_cfg, &io_handle),
        TAG, "Panel IO init failed");

    ESP_LOGI(TAG, "ST7789 panel init (RST=%d)...", BSP_LCD_RST);
    esp_lcd_panel_handle_t panel = NULL;
    const esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num  = BSP_LCD_RST,
        .rgb_ele_order   = LCD_RGB_ELEMENT_ORDER_RGB,
        /* RGB565 in RAM is LE; SPI sends low byte first. Match ST7789 RAMCTRL. */
        .data_endian     = LCD_RGB_DATA_ENDIAN_LITTLE,
        .bits_per_pixel  = 16,
    };
    ESP_RETURN_ON_ERROR(
        esp_lcd_new_panel_st7789(io_handle, &panel_cfg, &panel),
        TAG, "Panel create failed");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(panel), TAG, "Panel reset failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(panel), TAG, "Panel init failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_invert_color(panel, true), TAG, "Invert failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(panel, true), TAG, "Disp on failed");
    /* Backlight stays off (LEDC channel was configured with duty=0); the app
     * is expected to call bsp_display_set_backlight_percent() once its first
     * frame is on the panel so the user never sees uninitialised VRAM. */

    ESP_LOGI(TAG, "LVGL port init...");
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    ESP_RETURN_ON_ERROR(lvgl_port_init(&lvgl_cfg), TAG, "LVGL port init failed");

    /* buffer_size is in PIXELS, not bytes (esp_lvgl_port_disp.h) -- the port
     * allocates buffer_size * lv_color_format_get_size(). 20 lines x 240 px =
     * 4800 px = 9600 B per buffer, two of them: 19,200 B of internal RAM total
     * for ~1/12 of the screen per buffer. (Passing bytes here silently doubled
     * both buffers to 40 lines / 19,200 B each.)
     *
     * buff_dma is load-bearing: without it the port allocates with
     * MALLOC_CAP_DEFAULT, which on this board (CONFIG_SPIRAM_USE_MALLOC with
     * SPIRAM_MALLOC_ALWAYSINTERNAL=4096) puts both buffers in PSRAM. esp_lcd's
     * SPI panel IO never sets SPI_TRANS_DMA_USE_PSRAM, so spi_master then hits
     * setup_dma_priv_buffer() and heap_caps_aligned_alloc()s an internal-RAM
     * bounce buffer the size of the whole flush on every single transfer --
     * the actual source of the ENOMEM that the flush watchdog above papers
     * over. MALLOC_CAP_DMA lands them in internal RAM instead, where the
     * ESP32-S3 AHB-GDMA TX alignment requirement is 1 byte, so the bounce path
     * is never taken. */
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle   = io_handle,
        .panel_handle = panel,
        .buffer_size  = BSP_LCD_H_RES * 20,   /* pixels */
        .double_buffer = true,
        .hres         = BSP_LCD_H_RES,
        .vres         = BSP_LCD_V_RES,
        .monochrome   = false,
        .color_format = LV_COLOR_FORMAT_RGB565,
        .rotation = {
            .swap_xy  = false,
            .mirror_x = false,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma = true,
        },
    };
    lv_display_t *disp = lvgl_port_add_disp(&disp_cfg);
    ESP_RETURN_ON_FALSE(disp, ESP_FAIL, TAG, "LVGL display add failed");

    /* Replace esp_lvgl_port's panel-IO trans-done callback with ours so a
     * failed esp_lcd_panel_draw_bitmap doesn't deadlock LVGL forever, and
     * install the matching flush_wait_cb that yields with a bounded timeout
     * instead of letting LVGL fall into its default busy-spin.
     *
     * Both must happen atomically w.r.t. the LVGL task, which is already
     * running (and at a higher priority than app_main) by the time
     * lvgl_port_add_disp returns: our ISR callback no longer clears
     * `disp->flushing`, so a flush dispatched with our ISR installed but the
     * wait_cb not yet installed would busy-spin forever. Holding the LVGL
     * lock keeps any refresh out of the window. A flush already in flight
     * across the swap is safe either way -- if esp_lvgl_port's old callback
     * already cleared `flushing`, LVGL skips the wait and no token is
     * consumed; if ours fires instead, `flushing` stays set and the token is
     * consumed by the wait_cb that is installed before we unlock. */
    s_flush_done_sem = xSemaphoreCreateBinary();
    ESP_RETURN_ON_FALSE(s_flush_done_sem, ESP_ERR_NO_MEM, TAG, "Flush sem alloc failed");
    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = on_panel_io_color_done,
    };
    ESP_RETURN_ON_FALSE(lvgl_port_lock(0), ESP_ERR_TIMEOUT, TAG, "LVGL lock failed");
    esp_err_t cb_err = esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs, disp);
    if (cb_err == ESP_OK) {
        /* Flush-health hooks. Registering each event individually instead of
         * LV_EVENT_ALL keeps our handler off the per-area redraw events that
         * fire dozens of times per frame. */
        lv_display_add_event_cb(disp, on_flush_event, LV_EVENT_FLUSH_START,       NULL);
        lv_display_add_event_cb(disp, on_flush_event, LV_EVENT_FLUSH_FINISH,      NULL);
        lv_display_add_event_cb(disp, on_flush_event, LV_EVENT_FLUSH_WAIT_START,  NULL);
        lv_display_add_event_cb(disp, on_flush_event, LV_EVENT_FLUSH_WAIT_FINISH, NULL);
        lv_display_set_flush_wait_cb(disp, flush_wait_cb);
    }
    lvgl_port_unlock();
    ESP_RETURN_ON_ERROR(cb_err, TAG, "Panel IO callback register failed");

    ESP_LOGI(TAG, "Display ready (%dx%d)", BSP_LCD_H_RES, BSP_LCD_V_RES);
    return ESP_OK;
}
