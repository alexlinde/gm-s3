#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t bsp_display_init(void);

/* Set the LCD backlight. percent is clamped to 0..100;
 * 0 = fully off, 100 = fully on. */
esp_err_t bsp_display_set_backlight_percent(int percent);

/* Snapshot of LVGL flush health. All counts are total since boot.
 *
 * LVGL blocks in wait_for_flushing() until the panel IO `on_color_trans_done`
 * ISR signals the SPI transfer is done; with the stock esp_lvgl_port setup
 * (no flush_wait_cb) that block is a busy-spin which starves IDLE0 -> task
 * watchdog reset if the ISR stops firing. bsp_display.c installs a bounded
 * wait_cb instead, and marks each FLUSH_WAIT_START / FLUSH_WAIT_FINISH so an
 * off-thread observer can spot a wait that has been outstanding too long.
 *
 * pending_us == 0 means we are not currently inside a wait. Otherwise it's
 * the duration (microseconds) of the in-progress wait at sample time. */
typedef struct {
    uint32_t  flush_starts;     /* LV_EVENT_FLUSH_START count */
    uint32_t  flush_finishes;   /* LV_EVENT_FLUSH_FINISH count */
    uint32_t  wait_starts;      /* LV_EVENT_FLUSH_WAIT_START count */
    uint32_t  wait_finishes;    /* LV_EVENT_FLUSH_WAIT_FINISH count */
    int64_t   pending_us;       /* duration of the currently-in-progress wait, or 0 */
} bsp_display_flush_stats_t;

void bsp_display_get_flush_stats(bsp_display_flush_stats_t *out);

/* Diagnostic counters for the bounded flush_wait_cb.
 *   timeouts         -> times the panel-IO ISR didn't fire within the
 *                       wait-cb timeout (typically because the SPI tx
 *                       was never queued, i.e. spi_master ENOMEM).
 *   late_arrivals    -> the ISR fired *after* wait_cb had timed out,
 *                       and we drained the stale signal so it didn't
 *                       confuse the next wait. */
uint32_t bsp_display_flush_timeouts(void);
uint32_t bsp_display_flush_late_arrivals(void);

#ifdef __cplusplus
}
#endif
