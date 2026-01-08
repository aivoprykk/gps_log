#include "log_private.h"
#include "ubx.h"
#include "ubx_msg.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "vfs.h"

#if defined(CONFIG_GPS_LOG_ENABLED)

static const char *TAG = "gps_log";

// #if (C_LOG_LEVEL == LOG_TRACE_NUM) && defined(SHOW_SAVED_FRAMES)
// #define CONFIG_GPS_TIMER_STATS_ENABLED 1
// #endif

#if defined(CONFIG_GPS_TIMER_STATS_ENABLED)
static uint32_t push_failed_count = 0, push_ok_count = 0, prev_push_failed_count = 0, prev_push_ok_count = 0;
static uint32_t local_nav_dop_count = 0, prev_local_nav_dop_count = 0;
static uint32_t prev_local_nav_pvt_count = 0;
static uint32_t prev_local_nav_sat_count = 0;
static uint32_t prev_millis = 0, prev_msg_count = 0, prev_err_count = 0;
static esp_timer_handle_t gps_periodic_timer = 0;
static size_t rx_buf_used_max = 0;
static size_t rx_buf_used_min = SIZE_MAX;

static void s(void* arg) {
    ubx_ctx_t *ubx_ctx = (ubx_ctx_t *)arg;
    if(!ubx_ctx) return;
    uint32_t millis = get_millis();
    uint32_t period = millis - prev_millis;
    uint16_t period_err_count = ubx_ctx->ubx_msg.count_err-prev_err_count;
    uint16_t period_msg_count = ubx_ctx->ubx_msg.count_msg - prev_msg_count;
    uint16_t period_saved_count = period_msg_count - period_err_count;
    uint16_t period_push_failed_count = push_failed_count - prev_push_failed_count;
    uint16_t period_push_ok_count = push_ok_count - prev_push_ok_count;
    uint16_t period_push_count = period_push_failed_count + period_push_ok_count;
    uint16_t period_local_nav_dop_count = local_nav_dop_count - prev_local_nav_dop_count;
    uint16_t period_local_nav_pvt_count = ubx_ctx->ubx_msg.count_nav_pvt - prev_local_nav_pvt_count;
    uint16_t period_local_nav_sat_count = ubx_ctx->ubx_msg.count_nav_sat - prev_local_nav_sat_count;
    prev_millis = millis;
    prev_msg_count = ubx_ctx->ubx_msg.count_msg;
    prev_err_count = ubx_ctx->ubx_msg.count_err;
    prev_push_failed_count = push_failed_count;
    prev_push_ok_count = push_ok_count;
    prev_local_nav_dop_count = local_nav_dop_count;
    prev_local_nav_pvt_count = ubx_ctx->ubx_msg.count_nav_pvt;
    prev_local_nav_sat_count = ubx_ctx->ubx_msg.count_nav_sat;

    // Throughput verification metrics
    float throughput = period > 0 ? (float)period_msg_count * 1000.0f / (float)period : 0.0f;
    float loss_pct = period_msg_count > 0 ? (float)period_err_count * 100.0f / (float)period_msg_count : 0.0f;
    uint8_t expected_hz = g_rtc_config.ubx.output_rate;

    // Snapshot RX ring buffer usage (non-blocking if mutex busy)
    size_t buf_size = ubx_ctx->rx_buf_size;
    size_t buf_avail = 0;
    size_t buf_free = 0;
    if (ubx_ctx->rx_buf_mutex && xSemaphoreTake(ubx_ctx->rx_buf_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        size_t head = ubx_ctx->rx_buf_head;
        size_t tail = ubx_ctx->rx_buf_tail;
        if (head >= tail) {
            buf_avail = head - tail;
        } else {
            buf_avail = buf_size - tail + head;
        }
        buf_free = buf_size > 0 ? buf_size - buf_avail - 1 : 0;
        xSemaphoreGive(ubx_ctx->rx_buf_mutex);
    }
    if (buf_avail > rx_buf_used_max) {
        rx_buf_used_max = buf_avail;
    }
    else if (buf_avail < rx_buf_used_min) {
        rx_buf_used_min = buf_avail;
    }
    float buf_used_pct = buf_size > 0 ? ((float)buf_avail * 100.0f) / (float)buf_size : 0.0f;
    float buf_used_max_pct = buf_size > 0 ? ((float)rx_buf_used_max * 100.0f) / (float)buf_size : 0.0f;

    printf("\n[%s] ========== THROUGHPUT VERIFICATION ==========\n", __FUNCTION__);
    printf("[%s] Period: %"PRIu32"ms, Config rate: %"PRIu8"Hz (expected ~%u msg/s)\n", 
        __FUNCTION__, period, expected_hz, (expected_hz*2+1));
    printf("[%s] RX Throughput: %.1f msg/s | Loss: %.1f%% (%"PRIu16" err / %"PRIu16" msg)\n", 
        __FUNCTION__, throughput, loss_pct, period_err_count, period_msg_count);
    printf("[%s] RX Buffer: size=%zu avail=%zu free=%zu used=%.1f%% [max_used=%zu (%.1f%%) min_used=%zu]\n",
        __FUNCTION__, buf_size, buf_avail, buf_free, buf_used_pct, rx_buf_used_max, buf_used_max_pct, rx_buf_used_min);
    printf("[%s] Message types (period): PVT=%"PRIu16" DOP=%"PRIu16" SAT=%"PRIu16"\n", 
        __FUNCTION__, period_local_nav_pvt_count, period_local_nav_dop_count, period_local_nav_sat_count);
    printf("[%s] Message distribution: PVT=%.0f%% DOP=%.0f%% SAT=%.0f%%\n", 
        __FUNCTION__, 
        period_msg_count > 0 ? (float)period_local_nav_pvt_count * 100.0f / (float)period_msg_count : 0.0f,
        period_msg_count > 0 ? (float)period_local_nav_dop_count * 100.0f / (float)period_msg_count : 0.0f,
        period_msg_count > 0 ? (float)period_local_nav_sat_count * 100.0f / (float)period_msg_count : 0.0f);
    printf("[%s] Push stats (period): ok=%"PRIu16" fail=%"PRIu16" (%.1f%% loss)\n", 
        __FUNCTION__, period_push_ok_count, period_push_failed_count,
        period_push_count > 0 ? (float)period_push_failed_count * 100.0f / (float)period_push_count : 0.0f);
    printf("[%s] Totals: msg=%"PRIu32" ok=%"PRIu32" err=%"PRIu32" | PVT=%"PRIu32" DOP=%"PRIu32" SAT=%"PRIu32"\n", 
        __FUNCTION__, ubx_ctx->ubx_msg.count_msg, ubx_ctx->ubx_msg.count_ok, ubx_ctx->ubx_msg.count_err,
        ubx_ctx->ubx_msg.count_nav_pvt, local_nav_dop_count, ubx_ctx->ubx_msg.count_nav_sat);
    printf("[%s] ========================================\n\n", __FUNCTION__);
}
#endif


typedef struct {
    uint16_t old_run_count;
    uint8_t GPS_delay;
    uint32_t old_nav_pvt_itow;
    uint32_t next_time_sync;
    bool ubx_restart_requested;
    int output_rate_swp;
    uint32_t last_flush_time;
    bool gps_task_is_running;
    TaskHandle_t gps_task_handle;
    uint16_t ubx_fail_count; // widen to avoid overflow when accumulating error penalties
    uint8_t gps_initialized;
    uint8_t gps_started;
    uint8_t gps_events_registered;
} log_context_t;

static log_context_t lctx = {.old_run_count = 0, .GPS_delay = 0, .old_nav_pvt_itow = 0, .next_time_sync = 0, .ubx_restart_requested = false, 
                               .output_rate_swp = 0, .last_flush_time = 0, .gps_task_is_running = false, .gps_task_handle = NULL, .ubx_fail_count = 0, .gps_initialized = 0, .gps_started = 0, .gps_events_registered = 0};

gps_context_t * gps = 0;
static bool gps_config_observer_registered = false;

ESP_EVENT_DEFINE_BASE(GPS_LOG_EVENT);        // declaration of the LOG_EVENT family

// Forward declarations for file handlers
static void gps_file_open_handler(void* handler_arg, esp_event_base_t base, int32_t id, void* event_data);
static void gps_file_flush_handler(void* handler_arg, esp_event_base_t base, int32_t id, void* event_data);


// Unified GPS->VFS processor (registered via vfs_register_work_interface)
static void gps_vfs_processor(vfs_work_type_t type, void *arg) {
    gps_context_t *g = (gps_context_t*)arg;
    if (!g) return;
    switch(type) {
        case VFS_WORK_OPEN_FILES:
            open_files(g);
            break;
        case VFS_WORK_CLOSE_FILES:
            gps_speed_metrics_save_session();
            close_files(g);
            break;
        case VFS_WORK_FLUSH_FILES:
            flush_files(g);
            break;
        case VFS_WORK_PARTITION_CHANGED:
            gps_speed_metrics_save_session();
            close_files(g);
            open_files(g);
            break;
        case VFS_WORK_SAVE_SESSION:
            gps_speed_metrics_save_session();
            break;
        default:
            break;
    }
}

#if (C_LOG_LEVEL <= LOG_INFO_NUM)
static const char * const _gps_log_event_strings[] = {
    GPS_LOG_EVENT_LIST(STRINGIFY)
};
const char * gps_log_event_strings(int id) {
    return id < lengthof(_gps_log_event_strings) ? _gps_log_event_strings[id] : "GPS_LOG_EVENT_UNKNOWN";
}
#else
const char * gps_log_event_strings(int id) {
    return "GPS_LOG_EVENT";
}
#endif

// ============================================================================
// TIME MANAGEMENT
// ============================================================================

static int8_t set_time(float time_offset) {
    FUNC_ENTRY_ARGSD(TAG, "offset:%.1fh", time_offset);
    struct ubx_ctx_s *ubx_ctx = gps->ubx_device;
    nav_pvt_t *pvt = &ubx_ctx->ubx_msg.navPvt;
    if (!pvt->numSV) return 0;
    if (pvt->year < 2023) { // no valid time
        WLOG(TAG, "[%s] wrong time: %hu-%hhu-%hhu", __func__, pvt->year, pvt->month, pvt->day);
    }
    uint32_t millis = get_millis();
    if (gps->time_set && millis < lctx.next_time_sync) { // time already set and not time to sync
        return 0;
    }
    //ILOG(TAG, "[%s] sats: %" PRIu8, __FUNCTION__, pvt->numSV);
    // time_t unix_timestamp = 0;  // a timestamp in seconds
#if defined(DLS)
    // summertime is on march 26 2023 2 AM, see
    // https://www.di-mgt.com.au/wclock/help/wclo_tzexplain.html
    setenv("TZ", "CET0CEST,M3.5.0/2,M10.5.0/3", 1);  // timezone UTC = CET, Daylightsaving ON :
                                                     // TZ=CET-1CEST,M3.5.0/2,M10.5.0/3
#else
    setenv("TZ", "UTC", 0);
#endif
    tzset();                            // this works for CET, but TZ string is different for every Land / continent....
    struct tm my_time = {
        .tm_sec = pvt->second,
        .tm_min = pvt->minute,
        .tm_hour = pvt->hour,
        .tm_mday = pvt->day,
        .tm_mon = pvt->month - 1,       // mktime needs months 0 - 11
        .tm_year = pvt->year - 1900,    // mktime needs years since 1900, so deduct 1900
        .tm_isdst = -1,                 // daylight saving time flag
    };
    int sync_period = 30;
    int ret = c_set_time(&my_time, NANO_TO_US_ROUND(pvt->nano), time_offset);  // set the time in the struct tm
    if (ret) {
        ELOG(TAG, "[%s] Failed to set time from gps", __FUNCTION__);
        sync_period = 5;
        goto done;
    }
    // unix_timestamp = mktime(&my_time);  // mktime returns local time, so TZ is important !!!
    // int64_t utc_ms = unix_timestamp * 1000LL + NANO_TO_MILLIS_ROUND(pvt->nano);
    // int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
    WLOG(TAG, "GPS pvt time: %d-%02d-%02d %02d:%02d:%02d %ld %hhu, offset: %f", pvt->year, pvt->month, pvt->day, pvt->hour, pvt->minute, pvt->second, pvt->nano, pvt->id, time_offset);
    // WLOG(TAG, "GPS tm time : %d-%02d-%02d %02d:%02d:%02d %ld", my_time.tm_year+1900, my_time.tm_mon+1, my_time.tm_mday, my_time.tm_hour, my_time.tm_min, my_time.tm_sec, NANO_TO_US_ROUND(pvt->nano));
#if (C_LOG_LEVEL <= LOG_INFO_NUM)
    struct tm tm;
    get_local_time(&tm);
    WLOG(TAG, "GPS time set: %d-%02d-%02d %02d:%02d:%02d", (tm.tm_year) + 1900, (tm.tm_mon) + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
#endif
    if (esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_GPS_TIME_SET, NULL,0, pdMS_TO_TICKS(10)) != ESP_OK) {
        WLOG(TAG, "EVT_FAIL: GPS_TIME_SET");
    }
    done:
    lctx.next_time_sync = millis + SEC_TO_MS(sync_period); // 1 minute
    gps->time_set = 1;
    return 1;
}

uint8_t gps_read_msg_timeout(uint8_t magnitude) {
    if(!gps || !gps->ubx_device) return 0;
    if(!magnitude) magnitude = 1;
    return (gps->time_set && (gps->ubx_device->ubx_msg.navPvt.iTOW - lctx.old_nav_pvt_itow) > (gps->time_out_gps_msg * (magnitude))) ? 1 : 0;
}

uint8_t gps_has_version_set() {
    if(gps && gps->ubx_device) {
        if(gps->ubx_device->ready && gps->ubx_device->ubx_msg.mon_ver.hwVersion[0]) return 2;
        if(gps->ubx_device->ready && gps->ubx_device->hw_type) return 1;
    }
    return 0;
}

static void gps_buffers_init(void) {
    FUNC_ENTRYD(TAG);
#if !defined(CONFIG_GPS_LOG_STATIC_S_BUFFER)
    gps_check_sec_buf(BUFFER_SEC_SIZE);
#endif
    gps_speed_metrics_init();
    refresh_gps_speeds_by_distance();
    /// alpha buffers depending on output rate, so allocation after ubx setup
}

#if !defined(CONFIG_GPS_LOG_STATIC_A_BUFFER) || !defined(CONFIG_GPS_LOG_STATIC_S_BUFFER)
static void gps_on_sample_rate_change(void* handler_arg, esp_event_base_t base, int32_t id, void* event_data) {
    FUNC_ENTRY_ARGSD(TAG, "new rate:%d", g_rtc_config.ubx.output_rate);
    /// reallocate alfa buffer depending on output rate
    if(xSemaphoreTake(log_p_lctx.xMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        gps_free_alfa_buf();
        gps_check_alfa_buf(ALPHA_BUFFER_SIZE(g_rtc_config.ubx.output_rate));
        xSemaphoreGive(log_p_lctx.xMutex);
    }
}

static void gps_on_ubx_config_changed(void* handler_arg, esp_event_base_t base, int32_t id, void* event_data) {
    FUNC_ENTRY(TAG);
    // Request UBX restart to apply new GNSS/rate configuration
    lctx.ubx_restart_requested = true;
}

static void gps_config_changed_cb(size_t group, size_t index) {
    if (group != SCFG_GROUP_GPS) {
        return;
    }

    // Keep local log config in sync with the persisted GPS config
    gps_config_fix_values();

#if defined(CONFIG_LOGGER_VFS_ENABLED)
    bool affects_files = false;
    switch (index) {
        case cfg_gps_log_txt:
        case cfg_gps_log_ubx:
        case cfg_gps_log_sbp:
        case cfg_gps_log_gpx:
#if defined(GPS_LOG_ENABLE_GPY)
        case cfg_gps_log_gpy:
#endif
        case cfg_gps_file_date_time:
        case cfg_gps_ubx_file:
            affects_files = true;
            break;
        default:
            break;
    }

    // If logging is active, re-open files via VFS worker to apply new config
    if (affects_files && gps && gps->files_opened) {
        vfs_post_work(VFS_WORK_PARTITION_CHANGED, gps); // closes and re-opens files with new config
    }
#endif
}

static void gps_on_ubx_nav_mode_changed(void* handler_arg, esp_event_base_t base, int32_t id, void* event_data) {
    FUNC_ENTRY_ARGS(TAG, "nav_mode:%d (confirmed by device)", g_rtc_config.ubx.nav_mode);
    // This handler fires AFTER u-blox confirms nav mode change (response event)
    // Do NOT call ubx_set_nav_mode() here - it already happened!
    // Only use this for logging/monitoring the confirmed change
}

static void gps_on_gps_log_nav_mode_changed(void* handler_arg, esp_event_base_t base, int32_t id, void* event_data) {
    FUNC_ENTRY_ARGS(TAG, "nav_mode:%d", g_rtc_config.ubx.nav_mode);
    // Actually apply the nav mode change to the u-blox device
    if(gps && gps->ubx_device) {
        ubx_set_nav_mode(gps->ubx_device, g_rtc_config.ubx.nav_mode);
    }
    // Handle GPS nav mode change file I/O operations in GPS task
    gps_log_nav_mode_change(gps, 1);
}

// Test function to simulate UBX config change for testing async reconfiguration
// void test_ubx_config_change(void) {
//     FUNC_ENTRY(TAG);
//     ILOG(TAG, "Testing async UBX config change - posting event");
//     esp_event_post(UBX_EVENT, UBX_EVENT_CONFIG_CHANGED, NULL, 0, portMAX_DELAY);
// }

static void gps_free_sec_buffers(void) {
    FUNC_ENTRYD(TAG);
#if !defined(CONFIG_GPS_LOG_STATIC_A_BUFFER)
    gps_free_alfa_buf();
#endif
#if !defined(CONFIG_GPS_LOG_STATIC_S_BUFFER)
    gps_free_sec_buf();
#endif
}

static void gps_buffers_free(void) {
    FUNC_ENTRYD(TAG);
    gps_free_sec_buffers();
    gps_speed_metrics_free();
}

static void gps_on_ubx_deinit(void* handler_arg, esp_event_base_t base, int32_t id, void* event_data) {
    FUNC_ENTRYD(TAG);
    gps_free_sec_buffers();
}

#endif // CONFIG_GPS_LOG_STATIC_A_BUFFER || CONFIG_GPS_LOG_STATIC_S_BUFFER

static void gpsTask(void *parameter) {
    FUNC_ENTRYD(TAG);
    uint32_t now = 0, loops = 0, mt = 0;
    ubx_ctx_t *ubx_ctx = gps->ubx_device;
    ubx_msg_byte_ctx_t ubx_packet = UBX_MSG_BYTE_CTX_DEFAULT(ubx_ctx->ubx_msg);
    ubx_packet.ctx = ubx_ctx;
    ubx_packet.msg_match_to_pos = false;
    ubx_packet.msg_ready_handler = ubx_msg_checksum_handler;
    ubx_msg_t * ubxMessage = &ubx_ctx->ubx_msg;
    struct nav_pvt_s * nav_pvt = &ubxMessage->navPvt;
    uint8_t try_setup_times = 5;
    while (lctx.gps_task_is_running) {
        now = get_millis();
        
        // Debug: verify loop is running
        if ((loops > 0) && (loops % 500) == 0) {
            WLOG(TAG, "LOOP_ALIVE: l=%lu now=%lu", loops, now);
        }
        
        if (!gps_has_version_set() || lctx.ubx_restart_requested) {
            mt = now - (ubx_ctx->ready ? ubx_ctx->ready_time : SEC_TO_MS(5));
            // ILOG(TAG, "[%s] Gps init ... (%lums)", __FUNCTION__, mt);
            if (mt > SEC_TO_MS(10)) { // 5 seconds
                if(ubx_ctx->ready){
                    ubx_off(ubx_ctx); // uart deinit
                    delay_ms(100);
                    lctx.ubx_fail_count++;
                }
                ubx_setup(ubx_ctx); // uart init and reset
            }
            if(lctx.ubx_fail_count>50) {
                if(!gps_has_version_set()) { // only when no hwVersion is received
                    if (esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_GPS_REQUEST_RESTART, NULL, 0, pdMS_TO_TICKS(100)) != ESP_OK) {
                        WLOG(TAG, "EVT_FAIL: GPS_REQUEST_RESTART");
                    }
                    ILOG(TAG, "[%s] Gps init failed, restart requested.", __FUNCTION__);
                }
                else
                    lctx.ubx_fail_count = 0;
            }
            lctx.ubx_restart_requested = 0;
            goto loop_tail;
        }
        else if(vfs_ctx.gps_log_part < VFS_PART_MAX) {
            if(!lctx.output_rate_swp && g_rtc_config.ubx.output_rate >= UBX_OUTPUT_5HZ && vfs_ctx.parts[vfs_ctx.gps_log_part].free_bytes < TO_K_UL(700)) {
                WLOG(TAG, "[%s] vfs log part %s free space too low: %llu!", __FUNCTION__, vfs_ctx.parts[vfs_ctx.gps_log_part].mount_point, vfs_ctx.parts[vfs_ctx.gps_log_part].free_bytes);
                lctx.output_rate_swp = g_rtc_config.ubx.output_rate;
                g_rtc_config.ubx.output_rate = UBX_OUTPUT_5HZ;
                ubx_set_gnss_and_rate(gps->ubx_device, g_rtc_config.ubx.gnss, g_rtc_config.ubx.output_rate);
                // set_gps_cfg_item(cfg_ubx_ubx_output_rate, 1);
            }
            else if(lctx.output_rate_swp && g_rtc_config.ubx.output_rate < UBX_OUTPUT_5HZ && vfs_ctx.parts[vfs_ctx.gps_log_part].free_bytes >= TO_K_UL(700)) {
                WLOG(TAG, "[%s] vfs log part %s free space ok again: %llu!", __FUNCTION__, vfs_ctx.parts[vfs_ctx.gps_log_part].mount_point, vfs_ctx.parts[vfs_ctx.gps_log_part].free_bytes);
                g_rtc_config.ubx.output_rate =(lctx.output_rate_swp == UBX_OUTPUT_5HZ) ? UBX_OUTPUT_10HZ :
                                        (lctx.output_rate_swp == UBX_OUTPUT_10HZ) ? UBX_OUTPUT_16HZ :
                                        (lctx.output_rate_swp == UBX_OUTPUT_16HZ) ? UBX_OUTPUT_20HZ :
                                        UBX_OUTPUT_1HZ; /// have to set 1 step above to calculate saved rate
                lctx.output_rate_swp = 0;
                ubx_set_gnss_and_rate(gps->ubx_device, g_rtc_config.ubx.gnss, g_rtc_config.ubx.output_rate);
                // set_gps_cfg_item(cfg_ubx_ubx_output_rate, 1);
            }
        }
        
        // Two-phase processing: FAST decode (drain buffer) → SLOW processing (file I/O)
        // Phase 1: Drain buffer by decoding ALL available messages (no heavy processing)
        // Timeout scales with GPS rate to minimize CPU waste while staying responsive:
        // 1-2Hz=100ms, 3-5Hz=50ms, 6-10Hz=20ms, 11-20Hz=10ms, 21-30Hz=5ms
        // At low rates (2Hz=500ms period), 100ms timeout = 5 wakeups/msg (vs 100 with 5ms)
        // At high rates (30Hz=33ms period), 5ms timeout = responsive without missing data
        // Recalculated each iteration to adapt to runtime rate changes
        uint32_t timeout_ms;
        if (!ubx_ctx->ready) {
            timeout_ms = 50;  // During init, use moderate timeout
        } else if (g_rtc_config.ubx.output_rate >= 21) {
            timeout_ms = 5;   // 21-30Hz: very responsive
        } else if (g_rtc_config.ubx.output_rate >= 11) {
            timeout_ms = 10;  // 11-20Hz: responsive
        } else if (g_rtc_config.ubx.output_rate >= 6) {
            timeout_ms = 20;  // 6-10Hz: balanced
        } else if (g_rtc_config.ubx.output_rate >= 3) {
            timeout_ms = 50;  // 3-5Hz: reduce wakeups
        } else {
            timeout_ms = 100; // 1-2Hz: minimize wakeups (still 5x per message at 2Hz)
        }
        BaseType_t signaled = xSemaphoreTake(ubx_ctx->msg_ready, pdMS_TO_TICKS(timeout_ms));
        
        if (!signaled && ubx_ctx->ready) {
            if ((loops % 1000) == 0) {
                WLOG(TAG, "NOSIG: timeout, no msgs at l=%lu", loops);
            }
            goto loop_tail;
        }
        
            bool had_nav_pvt = false;  // Track if we decoded a NAV_PVT (has speed data for movement detection)
            bool had_nav_dop = false;  // Track if we decoded a NAV_DOP (has DOP values)
            int decoded_count = 0;
            uint32_t phase1_start = get_millis();  // Track Phase 1 duration for diagnostics
            uint32_t phase2_start = phase1_start;
            // Phase 1: Decode loop - drain buffer but limit to prevent starvation
            // At GPS rates 1-30Hz, expect 1-4 messages per wake cycle
            // Break after decoding a reasonable batch to allow Phase 2 processing
            int phase1_iterations = 0;
            const int max_decode_batch = (g_rtc_config.ubx.output_rate >= 20) ? 6 : 
                                        (g_rtc_config.ubx.output_rate >= 10) ? 4 : 3;
        while (true) {
            // Exit after decoding batch to prevent backlog accumulation
            if (decoded_count >= max_decode_batch) {
                if ((loops % 500) == 0) {
                    WLOG(TAG, "P1BATCH: decoded %d msgs, breaking for Phase2", decoded_count);
                }
                break;
            }
            // Safety: prevent infinite loop
            if (++phase1_iterations > 50) {
                WLOG(TAG, "P1LIMIT: hit iteration limit, breaking (decoded=%d)", decoded_count);
                break;
            }
            esp_err_t ret = ubx_msg_handler(ubx_ctx, &ubx_packet);
            if ((loops % 100) == 0) {
                ILOG(TAG, "Phase1 ret=%d l=%lu", ret, loops);
            }
            if(!ret) {
                    decoded_count++;
                lctx.ubx_fail_count = 0;
                
                    // Only do FAST operations inline (update counters, store parsed data)
                    switch(ubx_packet.ubx_msg_type) {
                        case MT_NAV_PVT:
                            had_nav_pvt = true;  // Always trigger Phase 2 when NAV_PVT received (needed for set_time)
                            
                            // DEBUG: Log PVT reception
                            static uint32_t debug_pvt_decoded = 0;
                            // if ((++debug_pvt_decoded % g_rtc_config.ubx.output_rate) == 0) {
                            //     WLOG(TAG, "Decoded PVT #%lu: iTOW=%lu numSV=%d time_set=%d",
                            //          debug_pvt_decoded, ubxMessage->navPvt.iTOW, 
                            //          ubxMessage->navPvt.numSV, gps->time_set);
                            // }
                            if (ubxMessage->navPvt.iTOW > 0 && gps->time_set == 1) {
                                ubxMessage->count_nav_pvt++;
                                lctx.old_nav_pvt_itow = nav_pvt->iTOW;  // Update immediately to prevent false timeouts
                            }
                            break;
                        case MT_NAV_SAT:
                            ubxMessage->count_nav_sat++;
                            ubxMessage->nav_sat.iTOW = ubxMessage->nav_sat.iTOW - SEC_TO_MS(LEAP_UTC_OFFSET);
                            push_gps_sat_info(&gps->Ublox_Sat, &ubxMessage->nav_sat);
                            break;
                        case MT_NAV_DOP:
                            had_nav_dop = true;  // Optional DOP data
    #if defined(CONFIG_GPS_TIMER_STATS_ENABLED)
                            local_nav_dop_count++;
    #endif
                            break;
                        default:
                            break;
                    }
                
                // Keep draining: try to take msg_ready with zero timeout
                // Trust semaphore as primary indicator - only exit if both semaphore and buffer are empty
                if (xSemaphoreTake(ubx_ctx->msg_ready, 0) != pdTRUE) {
                    // No semaphore signal - buffer should be empty, but verify to be safe
                    // Single check without retries to minimize latency
                    if (!ubx_rx_has_complete_frame(ubx_ctx)) {
                        if ((loops % 500) == 0) {
                            uint32_t phase1_duration = get_millis() - phase1_start;
                            WLOG(TAG, "P1EXIT: decoded=%d duration=%lums at l=%lu", 
                                 decoded_count, phase1_duration, loops);
                        }
                        break;  // Buffer empty, proceed to Phase 2
                    }
                    else {
                        // Has frame - continue draining
                        if ((loops % 500) == 0) {
                            ILOG(TAG, "P1CONT: has frame without semaphore at l=%lu", loops);
                        }
                    }
                    // Has frame but no semaphore - UART task hasn't signaled yet, continue draining
                }
                // Has pending semaphore signal - continue draining
            } else {
                if((gps_has_version_set() && (ret == ESP_ERR_TIMEOUT && lctx.ubx_fail_count>3)) || lctx.ubx_fail_count>20) {
                    if(ubx_ctx->ready) {
                        ubx_ctx->ready = false;
                        ubxMessage->mon_ver.hwVersion[0] = 0;
                    }
                    lctx.ubx_fail_count++;
                    WLOG(TAG, "P1GOTO: critical fail at l=%lu", loops);
                    goto loop_tail;
                }
                lctx.ubx_fail_count+=5;
                if ((loops % 500) == 0) {
                    WLOG(TAG, "P1ERR: error ret=%d at l=%lu", ret, loops);
                }
                break;
            }
        }
        
            // DEBUG: Confirm we exited Phase 1 decode loop
            // WLOG(TAG, "POST-P1: had_pvt=%d had_dop=%d decoded=%d l=%lu",
            //      had_nav_pvt, had_nav_dop, decoded_count, loops);
        
            // Phase 2: Heavy processing AFTER buffer is drained
            // Do file I/O, calculations, events - buffer keeps filling but we're not blocking UART
            // Process NAV_PVT data (speed, movement, file operations)
            if (had_nav_pvt) {
                // DEBUG: Trace Phase 2 entry
                static uint32_t debug_phase2_count = 0;
                phase2_start = get_millis();
                debug_phase2_count++;
                if ((debug_phase2_count % 50) == 0) {
                    WLOG(TAG, "Phase2 #%lu: time_set=%d next_sync=%lu year=%d speed=%ld",
                         debug_phase2_count, gps->time_set, lctx.next_time_sync,
                         ubxMessage->navPvt.year, gps->gps_speed);
                }
                
                // Try to set time first (needed to bootstrap time_set flag)
                // Check if it's time to sync (now >= next_time_sync), avoiding uint32 wraparound issues
                if (now >= lctx.next_time_sync || lctx.next_time_sync == 0) {
                    set_time(g_rtc_config.gps.timezone);
                }
                
                // Only proceed with speed/movement processing if we have valid GPS time (iTOW > 0)
                if (nav_pvt->iTOW == 0) {
                    printf(".[].\n");
                    goto loop_tail;
                }
                
                // Check for first fix (needs DOP data, so only when had_nav_dop is true)
                if (had_nav_dop) {
                    if ((nav_pvt->numSV >= MIN_numSV_FIRST_FIX) && (MS_TO_SEC(nav_pvt->sAcc) < MAX_Sacc_FIRST_FIX) && 
                        (nav_pvt->valid >= 7) && (gps->signal_ok == false)) {
                        gps->signal_ok = true;
                        gps->first_fix = (now - ubx_ctx->ready_time);
                        WLOG(TAG, "[%s] First GPS Fix after %.01f sec.", __FUNCTION__, MS_TO_SEC(gps->first_fix));
                        // Non-blocking post
                        if (esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_GPS_FIRST_FIX, NULL, 0, 0) != ESP_OK) {
                            WLOG(TAG, "EVT_FAIL: GPS_FIRST_FIX");
                        }
                    }
                } else if (!gps->signal_ok && (loops % 200) == 0) {
                    // Log why first fix check was skipped (no DOP data)
                    WLOG(TAG, "FIX_SKIP: no_dop, sats=%d acc=%.1fm valid=0x%x",
                        nav_pvt->numSV, MS_TO_SEC(nav_pvt->sAcc), nav_pvt->valid);
                }
                
                if (gps->signal_ok && lctx.GPS_delay < UINT8_MAX) {
                    lctx.GPS_delay++;
                }

                if (!gps->files_opened && gps->signal_ok && 
                    (lctx.GPS_delay > (TIME_DELAY_FIRST_FIX * g_rtc_config.ubx.output_rate))) {
                    int32_t avg_speed = nav_pvt->gSpeed;
                    if (avg_speed > STANDSTILL_DETECTION_MAX && gps->time_set) {
                        gps->start_logging_millis = now;
                        WLOG(TAG, "[%s] Requesting file open: speed=%ldmm/s", __FUNCTION__, avg_speed);
                        // Non-blocking post (0ms) - file open is async, no need to wait
                        // Critical for high-rate GPS (20-30Hz) to prevent blocking
                        if (esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_REQUEST_FILE_OPEN, NULL, 0, 0) != ESP_OK) {
                            WLOG(TAG, "EVT_FAIL: REQUEST_FILE_OPEN (queue full?)");
                        }
                    }
                }
            
                // Process speed whenever we have valid NAV_PVT with iTOW (already checked above)
                // Update counter for tracking (but don't gate processing on it - GPS may lose/regain time lock)
                if (ubxMessage->navPvt.iTOW > 0 && gps->time_set == 1) {
                    ubxMessage->count_nav_pvt_prev = ubxMessage->count_nav_pvt;
                }
                
                if (gps->time_set && ubxMessage->count_nav_pvt > 10) {
                    gps->gps_speed = nav_pvt->gSpeed;
                    lctx.old_nav_pvt_itow = nav_pvt->iTOW;

                    if (gps->files_opened && (now - lctx.last_flush_time) > 60000) {
                        // Non-blocking post - flush is async and low priority
                        // If queue full at high rates (20-30Hz), skip and retry next period
                        if (esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_REQUEST_FILE_FLUSH, NULL, 0, 0) == ESP_OK) {
                            lctx.last_flush_time = now;
                        }
                        // Don't update last_flush_time on failure - will retry next period
                    }
                
                    uint32_t sacc_mm = nav_pvt->sAcc;
                    
                    if ((nav_pvt->numSV <= MIN_numSV_GPS_SPEED_OK) || 
                        (sacc_mm > MAX_Sacc_GPS_SPEED_OK * 1000) ||
                        (gps->gps_speed > MAX_GPS_SPEED_OK * 1000)) {
    #if (C_LOG_LEVEL <= LOG_INFO_NUM)
                        WLOG(TAG, "[%s] GPS REJECTED: sats=%hhu (need>%d), acc=%lumm (need<=%dm), speed=%ldmm/s",
                            __FUNCTION__, nav_pvt->numSV, MIN_numSV_GPS_SPEED_OK, sacc_mm, MAX_Sacc_GPS_SPEED_OK*1000, gps->gps_speed);
    #endif
                        gps->gps_speed = 0;
                        gps->Ublox.run_start_time = 0;
                    }
                
                    if (gps->gps_speed > STANDSTILL_DETECTION_MAX) {
                        if (gps->files_opened) {
                            log_to_file(gps);
                        }
                        if (!gps->gps_is_moving) {
                            gps->gps_is_moving = true;
                            // WLOG(TAG, "[%s] *** GPS IS MOVING *** speed=%ldmm/s", __FUNCTION__, gps->gps_speed);
                            // Non-blocking post for high-rate compatibility (1-30Hz)
                            if (esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_GPS_IS_MOVING, NULL, 0, 0) != ESP_OK) {
                                WLOG(TAG, "EVT_FAIL: GPS_IS_MOVING");
                            }
                            printf(".[M].\n");
                        }
                    }
                    else {
                        if (gps->gps_is_moving) {
                            // WLOG(TAG, "[%s] *** GPS IS STOPPING *** speed=%ldmm/s", __FUNCTION__, gps->gps_speed);
                             printf(".[S].\n");
                            gps->gps_is_moving = false;
                            // Non-blocking post
                            if (esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_GPS_IS_STOPPING, NULL, 0, 0) != ESP_OK) {
                                WLOG(TAG, "EVT_FAIL: GPS_IS_STOPPING");
                            }
                            log_p_lctx.standstill_start_millis = 0;
                        }
                        else if (!gps->skip_alfa_after_stop) {
                            if (log_p_lctx.standstill_start_millis == 0) {
                                log_p_lctx.standstill_start_millis = now;
                            }
                            else if ((now - log_p_lctx.standstill_start_millis) > SEC_TO_MS(5)) {
                                gps->skip_alfa_after_stop = 1;
                                DLOG(TAG, "[%s] Standstill 5s - skip alfa", __FUNCTION__);
                            }
                        }
                    }
                
                    esp_err_t ret = push_gps_data(gps, &gps->Ublox, FROM_10M(nav_pvt->lat), 
                                                 FROM_10M(nav_pvt->lon), gps->gps_speed);
                    if (!ret) {
                        new_run_detection(gps, FROM_100K(nav_pvt->heading), time_cur_speed(time_2s));
                        alfa_indicator(FROM_100K(nav_pvt->heading));
                    
                        if (gps->run_count != lctx.old_run_count) {
                            gps->Ublox.run_distance = 0;
                            if (MM_TO_M(gps->gps_speed) > STANDSTILL_DETECTION_MAX) {
                                gps->Ublox.run_start_time = now;
                                gps->record = 0;
                                // Non-blocking post
                                if (esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_GPS_NEW_RUN, NULL, 0, 0) != ESP_OK) {
                                    WLOG(TAG, "EVT_FAIL: GPS_NEW_RUN");
                                }
                            }
                            lctx.old_run_count = gps->run_count;
                        }
                        gps_speed_metrics_update();
                    }
#if defined(CONFIG_GPS_TIMER_STATS_ENABLED)
                    else ++push_failed_count;
#endif
                }
            }
        
            // Phase 2 timing diagnostics (only at moderate rates to avoid overhead)
            if (had_nav_pvt && g_rtc_config.ubx.output_rate <= 10) {
                static uint32_t phase2_count = 0;
                static uint32_t phase2_total_ms = 0;
                static uint32_t phase2_max_ms = 0;
                uint32_t phase2_duration = get_millis() - phase2_start;
                phase2_total_ms += phase2_duration;
                if (phase2_duration > phase2_max_ms) phase2_max_ms = phase2_duration;
                
                // Warn if Phase 2 takes too long (indicates file I/O blocking)
                uint32_t budget_ms = g_rtc_config.ubx.output_rate >= 10 ? 50 : 
                                    (g_rtc_config.ubx.output_rate >= 5 ? 100 : 200);
                if (phase2_duration > budget_ms) {
                    WLOG(TAG, "P2SLOW: %lums (budget=%lums) @%dHz", 
                         phase2_duration, budget_ms, g_rtc_config.ubx.output_rate);
                }
                
                if ((++phase2_count % 100) == 0) {
                    uint32_t avg_ms = phase2_total_ms / phase2_count;
                    WLOG(TAG, "P2STATS: avg=%lums max=%lums count=%lu (rate=%dHz)", 
                         avg_ms, phase2_max_ms, phase2_count, g_rtc_config.ubx.output_rate);
                }
            }
        
    loop_tail:
        loops++;
        if ((loops % 100) == 0) {
            WLOG(TAG, "BEAT: l=%lu r=%d t=%d s=%d p=%lu sv=%hhu",
                loops, ubx_ctx->ready, gps->time_set, gps->signal_ok, ubxMessage->count_nav_pvt, nav_pvt->numSV);
        }
        continue;
    }
    lctx.gps_task_handle = 0;
    vTaskDelete(NULL);
}

static void gps_task_start() {
    FUNC_ENTRYD(TAG);
    if(lctx.gps_task_is_running) {
        ELOG(TAG, "[%s] already running!", __func__);
        return;
    }
    if(!lctx.gps_task_is_running) {
        lctx.gps_task_is_running = true;
        xTaskCreatePinnedToCore(gpsTask,   /* Task function. */
                "gpsTask", /* String with name of task. */
                CONFIG_GPS_LOG_STACK_SIZE,  /* Stack size in bytes. */
                NULL,      /* Parameter passed as input of the task */
                10,         /* Priority of the task. Reduced from 19 to prevent task starvation */
                &lctx.gps_task_handle, 0);      /* Task handle. Core 0 for WiFi affinity */
#if defined(CONFIG_GPS_TIMER_STATS_ENABLED)
        if(gps_periodic_timer)
            esp_timer_start_periodic(gps_periodic_timer, SEC_TO_US(1));
#endif
    }
}

static void gps_task_stop() {
    FUNC_ENTRYD(TAG);
    if(lctx.gps_task_is_running) {
#if defined(CONFIG_GPS_TIMER_STATS_ENABLED)
        if(gps_periodic_timer)
            esp_timer_stop(gps_periodic_timer);
#endif
        lctx.gps_task_is_running = false;
        uint32_t deadline = get_millis() + SEC_TO_MS(5);
        while(lctx.gps_task_handle && (get_millis() < deadline)) delay_ms(10);
        if(lctx.gps_task_handle) {
            vTaskDelete(lctx.gps_task_handle);
            lctx.gps_task_handle = 0;
        }
    }
    DLOG(TAG, "[%s] done.", __func__);
}

void gps_init(gps_context_t * _gps) {
    if(lctx.gps_initialized) return;
    FUNC_ENTRY(TAG);
    gps = _gps;
    gps_config_fix_values();
    init_gps_context_fields(gps);
#if defined(CONFIG_GPS_TIMER_STATS_ENABLED)
    const esp_timer_create_args_t gps_periodic_timer_args = {
            .callback = &s,
            .name = "gps_periodic",
            .arg = _gps->ubx_device,
        };
        if(esp_timer_create(&gps_periodic_timer_args, &gps_periodic_timer)){
            ELOG(TAG, "[%s] Failed to create periodic timer", __func__);
        }
#endif
    if (!gps_config_observer_registered) {
        gps_config_observer_registered = config_observer_add(gps_config_changed_cb);
        if (!gps_config_observer_registered) {
            WLOG(TAG, "[%s] failed to register GPS config observer", __func__);
        }
    }
    lctx.gps_initialized = 1;
}

void gps_deinit(void) {
    if(!lctx.gps_initialized) return;
    FUNC_ENTRY(TAG);
#if defined(CONFIG_GPS_TIMER_STATS_ENABLED)
    esp_timer_delete(gps_periodic_timer);
#endif
    if (gps_config_observer_registered) {
        config_observer_remove(gps_config_changed_cb);
        gps_config_observer_registered = false;
    }
    deinit_gps_context_fields(gps);
    lctx.gps_initialized = 0;
}

int gps_start() {
    FUNC_ENTRYD(TAG);
    if(!gps) return 0;
    struct ubx_ctx_s *ubx_ctx = gps->ubx_device;
    if (!ubx_ctx) return 0;
    if (lctx.gps_started) return 0;
    int ret = 0;
#if !defined(CONFIG_GPS_LOG_STATIC_A_BUFFER)
    if(!lctx.gps_events_registered) {
        esp_event_handler_register(UBX_EVENT, UBX_EVENT_SAMPLE_RATE_CHANGED, &gps_on_sample_rate_change, gps->ubx_device);
        esp_event_handler_register(UBX_EVENT, UBX_EVENT_CONFIG_CHANGED, &gps_on_ubx_config_changed, gps->ubx_device);
        esp_event_handler_register(UBX_EVENT, UBX_EVENT_NAV_MODE_CHANGED, &gps_on_ubx_nav_mode_changed, gps->ubx_device);
        esp_event_handler_register(GPS_LOG_EVENT, GPS_LOG_EVENT_GPS_NAV_MODE_CHANGED, &gps_on_gps_log_nav_mode_changed, NULL);
        // Register async file handlers (enqueue work for VFS worker)
        esp_event_handler_register(GPS_LOG_EVENT, GPS_LOG_EVENT_REQUEST_FILE_OPEN, &gps_file_open_handler, gps);
        esp_event_handler_register(GPS_LOG_EVENT, GPS_LOG_EVENT_REQUEST_FILE_FLUSH, &gps_file_flush_handler, gps);

        // Register VFS GPS interface
#if defined(CONFIG_LOGGER_VFS_ENABLED)
        {
            vfs_work_if_t iface = {
                .process = gps_vfs_processor,
                .ctx = gps
            };
            vfs_register_work_interface(&iface);
        }
#endif
        lctx.gps_events_registered = 1;
    }
#endif
    gps_buffers_init();
    // Async writer now started in open_files() in gps_log_file.c
    gps_task_start();
    lctx.gps_started = 1;
    return ret;
}

int gps_shut_down() {
    FUNC_ENTRYD(TAG);
    if(!gps) return 0;
    struct ubx_ctx_s *ubx_ctx = gps->ubx_device;
    if (!ubx_ctx) return 0;
    if (!lctx.gps_started) return 0;
    int ret = 0;
    ubx_ctx->shutdown_requested = 1;
    if (ubx_ctx->ready) {
        gps->signal_ok = false;
        lctx.GPS_delay = 0;
    }
    if (gps->time_set) {  // Only safe to RTC memory if new GPS data is available !!
        if (esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_GPS_SAVE_FILES, NULL, 0, pdMS_TO_TICKS(100)) != ESP_OK) {
            WLOG(TAG, "EVT_FAIL: GPS_SAVE_FILES");
        }
        // next_screen = CUR_SCREEN_SAVE_SESSION;
        // gps_save_rtc();
        if (gps->files_opened) {
            gps_speed_metrics_save_session();
            close_files(gps);
        }
    }
    gps_task_stop();
    // Async writer now stopped in close_files() in gps_log_file.c
#if !defined(CONFIG_GPS_LOG_STATIC_A_BUFFER) || !defined(CONFIG_GPS_LOG_STATIC_S_BUFFER)
    if (lctx.gps_events_registered) {
        esp_event_handler_unregister(UBX_EVENT, UBX_EVENT_SAMPLE_RATE_CHANGED, &gps_on_sample_rate_change);
        esp_event_handler_unregister(UBX_EVENT, UBX_EVENT_CONFIG_CHANGED, &gps_on_ubx_config_changed);
        esp_event_handler_unregister(UBX_EVENT, UBX_EVENT_NAV_MODE_CHANGED, &gps_on_ubx_nav_mode_changed);
        esp_event_handler_unregister(GPS_LOG_EVENT, GPS_LOG_EVENT_GPS_NAV_MODE_CHANGED, &gps_on_gps_log_nav_mode_changed);
        // Unregister async file handlers
        esp_event_handler_unregister(GPS_LOG_EVENT, GPS_LOG_EVENT_REQUEST_FILE_OPEN, &gps_file_open_handler);
        esp_event_handler_unregister(GPS_LOG_EVENT, GPS_LOG_EVENT_REQUEST_FILE_FLUSH, &gps_file_flush_handler);

#if defined(CONFIG_LOGGER_VFS_ENABLED)
        // Unregister VFS work interface
        vfs_register_work_interface(NULL);
#endif
        lctx.gps_events_registered = 0;
    }
#endif
    ubx_off(ubx_ctx);
    gps_buffers_free();

    gps->time_set = 0;
    if(lctx.output_rate_swp) {
        g_rtc_config.ubx.output_rate = lctx.output_rate_swp;
        lctx.output_rate_swp = 0;
    }
    lctx.gps_started = 0;
    if (esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_GPS_SHUT_DOWN_DONE, NULL, 0, pdMS_TO_TICKS(100)) != ESP_OK) {
        WLOG(TAG, "EVT_FAIL: GPS_SHUT_DOWN_DONE");
    }
    // if (!no_sleep) {
    //     go_to_sleep(3);  // got to sleep after 5 s, this to prevent booting when
    //     // GPIO39 is still low !
    // }
    // if(next_screen == CUR_SCREEN_SAVE_SESSION) {
    //     next_screen = CUR_SCREEN_NONE;
    // }
    return ret;
}

// Async file open handler - enqueue work for the VFS worker (avoids blocking event loop)
static void gps_file_open_handler(void* handler_arg, esp_event_base_t base, int32_t id, void* event_data) {
    FUNC_ENTRY_ARGS(TAG, "async file open request");
    gps_context_t *gps = (gps_context_t*)handler_arg;
    if (gps && !gps->files_opened) {
        ILOG(TAG, "[%s] Requesting VFS worker to open files", __FUNCTION__);
        vfs_post_work(VFS_WORK_OPEN_FILES, gps);
    }
}

// Async file flush handler - enqueue flush work for the VFS worker
static void gps_file_flush_handler(void* handler_arg, esp_event_base_t base, int32_t id, void* event_data) {
    FUNC_ENTRY_ARGSD(TAG, "async file flush request");
    gps_context_t *gps = (gps_context_t*)handler_arg;
    if (gps && gps->files_opened) {
        DLOG(TAG, "[%s] Requesting VFS worker to flush files", __FUNCTION__);
        vfs_post_work(VFS_WORK_FLUSH_FILES, gps);
    }
}

#endif
