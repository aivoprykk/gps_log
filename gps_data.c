#include "log_private.h"
#include "gps_data.h"
#include <stdint.h>

#if (defined(CONFIG_UBLOX_ENABLED) && defined(CONFIG_GPS_LOG_ENABLED))

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/unistd.h>
#include <math.h>

#ifdef CONFIG_LOGGER_VFS_ENABLED
#include "vfs.h"
#endif
#include "gps_log_file.h"
#include "logger_common.h"
#include "numstr.h"
#include "ubx.h"
#include "gps_user_cfg.h"
#include "gps_log_events.h"

static const char * TAG = "gps_data";

gps_p_context_t log_p_lctx = GPS_P_CONTEXT_INIT;

void unalloc_buffer(void **buf) {
    if (buf && *buf) {
        heap_caps_free(*buf), *buf = NULL;
#if (C_LOG_LEVEL < 3)
        WLOG(TAG, "[%s] Unallocated buffer", __func__);
#endif
    }
}

// Universal buffer check and allocation function
bool check_and_alloc_buffer(void **buf, size_t required_count, size_t elem_size, uint16_t *current_count, uint32_t caps) {
    if (!buf) return false;
    if (*buf && current_count && *current_count >= required_count) return true;
    unalloc_buffer(buf);
    required_count = ROUND_UP_TO_8(required_count);
    *buf = heap_caps_malloc(required_count * elem_size, caps);
    if (*buf && current_count) {
        *current_count = required_count;
#if (C_LOG_LEVEL < 3)
        WLOG(TAG, "[%s] Allocated buffer of size %zu", __func__, required_count * elem_size);
#endif
        return true;
    } else {
        if (current_count) *current_count = 0;
        return false;
    }
}

#if !defined(CONFIG_GPS_LOG_STATIC_A_BUFFER)
void gps_free_alfa_buf() {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    if(log_p_lctx.alfa_buf_size){
        unalloc_buffer((void **)&log_p_lctx.buf_sec_speed);
        log_p_lctx.alfa_buf_size = 0;
    }
}

void gps_check_alfa_buf(size_t new_size) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    check_and_alloc_buffer((void **)&log_p_lctx.alfa_buf, new_size, sizeof(gps_point_t), &log_p_lctx.alfa_buf_size, MALLOC_CAP_DMA);
    if (log_p_lctx.alfa_buf) {
        memset(log_p_lctx.alfa_buf, 0, new_size);
    }
#if (C_LOG_LEVEL < 3)
    else {
        WLOG(TAG, "[%s] Failed to allocate alfa buffer of size %zu", __func__, new_size);
    }
#endif
}
#endif

#if !defined(CONFIG_GPS_LOG_STATIC_S_BUFFER)
void gps_free_sec_buf() {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s] Freeing sec speed buffer", __func__);
#endif
    if (log_p_lctx.buf_sec_speed_size) {
        unalloc_buffer((void **)&log_p_lctx.buf_sec_speed);
        log_p_lctx.buf_sec_speed_size = 0;
    }
}

void gps_check_sec_buf(size_t new_size) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    check_and_alloc_buffer((void **)&log_p_lctx.buf_sec_speed, new_size, sizeof(int16_t), &log_p_lctx.buf_sec_speed_size, MALLOC_CAP_DMA);
    // Ensure the buffer size is at least 48 elements (96 bytes) and not larger than UINT16_MAX
    // This is to ensure that the buffer can hold at least 48 speed values (for 1 second at 10Hz)
    // and not exceed the maximum size for uint16_t.
    // This is to ensure that the buffer can hold at least 48 speed values (for 1 second at 10Hz)
    // and not exceed the maximum size for uint16_t.

    if (log_p_lctx.buf_sec_speed) {
        memset(log_p_lctx.buf_sec_speed, 0, new_size * sizeof(int16_t));
    }
#if (C_LOG_LEVEL < 3)
    else {
        WLOG(TAG, "[%s] Failed to allocate sec speed buffer of size %zu", __func__, new_size);
    }
#endif
}
#endif

// #define GPS_TRACE_MSG_SPEED_BY_DIST 1
// #define GPS_TRACE_MSG_SPEED_BY_TIME 1
// #define GPS_TRACE_MSG_SPEED_ALPHA 1
// #if (C_LOG_LEVEL < 2)
// #define GPS_STATS 1
// #endif
#if defined(GPS_STATS)
static esp_timer_handle_t gps_timer = 0;
static esp_err_t gps_data_printf();
#endif

#if defined(GPS_STATS)
static void s(void*arg) {
    gps_context_t *me = (gps_context_t *)arg;
    printf("--- GPS periodic timer callback %lu ---\n", get_millis());
    gps_data_printf(&me->Ublox);
#if defined(GPS_MSG_CTX)
    gps_p_context_printf(&log_p_lctx);
#endif
#if defined(GPS_TRACE_MSG_SAT)
    gps_sat_info_printf(me);
#endif
#if defined(GPS_TRACE_MSG_SPEED_BY_DIST)
    // gps_speed_by_dist_printf(&me->M100);
    // gps_speed_by_dist_printf(&me->M250);
    gps_speed_by_dist_printf(&me->M[GPS_SPEED_BY_DIST_M500]);
    // gps_speed_by_dist_printf(&me->M1852);
#endif
#if defined(GPS_TRACE_MSG_SPEED_BY_TIME)
    // gps_speed_by_time_printf(&me->S2);
    gps_speed_by_time_printf(&me->S10);
    // gps_speed_by_time_printf(&me->S1800);
    // gps_speed_by_time_printf(&me->S3600);
#endif
#if defined(GPS_TRACE_MSG_SPEED_ALPHA)
    // gps_speed_by_alpha_printf(&me->A250);
    gps_speed_by_alpha_printf(&me->A500);
#endif
    printf("--- End of GPS periodic timer callback ---\n");
}
#endif

#define CONFIG_GPS_LOG_ENABLE_DEBUG 0
#if defined(GPS_STATS) && defined(GPS_MSG_CTX)
static esp_err_t gps_p_context_printf(const gps_p_context_t *me) {
    printf("gps_p_context:{\n");
    printf("sec_gSpeed: %ld, ", me->sec_gSpeed);
    printf("run_count: %lu, ", me->run_count);
    printf("alfa_count: %lu, ", me->alfa_count);
    printf("old_alfa_count: %lu\n", me->old_alfa_count);
    printf("heading: %.02f, ", me->heading);
    printf("old_heading: %.02f, ", me->old_heading);
    printf("delta_heading: %.02f, ", me->delta_heading);
    printf("delta_dist: %.02f, ", me->delta_dist);
    printf("straight_course: %d\n", me->straight_course);
    printf("P1: {lat: %.02f, long: %.02f}\n", me->alfa_p1.latitude, me->alfa_p1.longitude);
    printf("P2: {lat: %.02f, long: %.02f}\n", me->alfa_p2.latitude, me->alfa_p2.longitude);
    printf("index_gspeed: %lu, ", me->index_gspeed);
    printf("index_sec: %lu\n", me->index_sec);
    printf("buf_gspeed: ");
    uint8_t i, j=buf_index(me->index_gspeed);
    if(j<10) j=10;
    for (i = j-10; i < j; i++) printf(" %ld", me->buf_gspeed[i]);
    printf("\n");
    j=me->sec_buf_index(index_sec);
    if(j<10) j=10;
    printf("buf_sec_speed: ");
    for (i = j-10; i < j; i++) printf(" %hu", me->buf_sec_speed[i]);
    printf("\n");
    j=buf_index(me->index_gspeed);
    if(j<10) j=10;
    printf("buf_alfa: ");
    for (i = j-10; i < j; i++) printf("{%.02f, %.02f},", me->buf_alfa[i].latitude, me->buf_alfa[i].longitude);
    printf("\n");
    printf("velocity_0: %d, ", me->velocity_0);
    printf("velocity_5: %d, ", me->velocity_5);
    printf("dynamic_state: %d, ", me->dynamic_state);
    printf("delay_count_before_run: %lu\n", me->delay_count_before_run);
    printf("}\n");
    return ESP_OK;
}
#endif

int32_t gps_last_speed_smoothed(uint8_t window_size) {
    return smooth(log_p_lctx.buf_gspeed, log_p_lctx.index_gspeed, log_p_lctx.buf_gspeed_size, window_size);
}

int32_t gps_last_sec_speed_smoothed(uint8_t window_size) {
    return smooth((int32_t*)log_p_lctx.buf_sec_speed, log_p_lctx.index_sec, log_p_lctx.buf_sec_speed_size, window_size);
}


void init_gps_context_fields(gps_context_t *ctx) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    if(!ctx) return;
    init_gps_data(&ctx->Ublox);  // create an object storing GPS_data !
    init_gps_sat_info(&ctx->Ublox_Sat);  // create an object storing GPS_SAT info !
    if (ctx->ubx_device == NULL){
        ctx->ubx_device = ubx_config_new();
    }
    if (ctx->log_config == NULL)
        ctx->log_config = &log_config;
    if(log_p_lctx.xMutex == NULL)
        log_p_lctx.xMutex = xSemaphoreCreateMutex();
    gps_user_cfg_init();
#if defined(GPS_STATS)
    const esp_timer_create_args_t gps_timer_args = {
            .callback = &s,
            .name = "gps_periodic",
            .arg = ctx,
    };
    if(esp_timer_create(&gps_timer_args, &gps_timer)){
        ESP_LOGE(TAG, "[%s] Failed to create periodic timer", __func__);
    }
    else if(gps_timer) {
        esp_timer_start_periodic(gps_timer, SEC_TO_US(5));
    }
#endif
    ctx->Gps_fields_OK = 1;
}

void deinit_gps_context_fields(gps_context_t *ctx) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    if(!ctx) return;
    if (ctx->ubx_device != NULL){
        ubx_config_delete(ctx->ubx_device);
        ctx->ubx_device = NULL;
    }
    if (ctx->log_config != NULL){
        // log_config_delete(ctx->log_config);
        ctx->log_config = NULL;
    }
#if defined(GPS_STATS)
    if(gps_timer) {
        esp_timer_stop(gps_timer);
        esp_timer_delete(gps_timer);
    }
#endif
    if(log_p_lctx.xMutex != NULL){
        vSemaphoreDelete(log_p_lctx.xMutex);
        log_p_lctx.xMutex = NULL;
    }
#if !defined(CONFIG_GPS_LOG_STATIC_A_BUFFER)
    gps_free_alfa_buf();
#endif
#if !defined(CONFIG_GPS_LOG_STATIC_S_BUFFER)
    gps_free_sec_buf();
#endif
    gps_speed_metrics_free();
    gps_user_cfg_deinit();
    ctx->Gps_fields_OK = 0;
}

#if defined(GPS_STATS)
static esp_err_t gps_data_printf(struct gps_data_s *me) {
    printf("gps_data:{ ");
    printf("run_start_time: %lu, ", me->run_start_time);
    printf("run_distance: %.02f, ", me->run_distance);
    printf("run_distance_after_turn: %.02f, ", me->run_distance_after_turn);
    printf("total_distance: %.02f ", me->total_distance);
    printf("}\n");
    return ESP_OK;
}
#endif

#if (C_LOG_LEVEL < 3)
#include "strbf.h"
void gps_log_nav_mode_change(gps_context_t *context, uint8_t changed) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s] %hhu", __func__, changed);
#endif
    ubx_config_t *ubx = context->ubx_device;
    const char * bstr = " nav_mode changed ";
    if(changed) {
        strbf_t strbf;
        char buf[96] = {0};
        strbf_inits(&strbf, buf, sizeof(buf));
        strbf_putul(&strbf, (MS_TO_SEC(get_millis() - context->start_logging_millis)));
        strbf_puts(&strbf, bstr);
        strbf_putul(&strbf, ubx->rtc_conf->nav_mode);
        if(log_p_lctx.dynamic_state==0) {
            strbf_puts(&strbf, " back ");
            strbf_putul(&strbf, ubx->rtc_conf->nav_mode == UBX_MODE_PEDESTRIAN ? UBX_MODE_PEDESTRIAN : UBX_MODE_PORTABLE);
        }
        else {
            strbf_puts(&strbf, " to ");
            strbf_putul(&strbf, ubx->rtc_conf->nav_mode == UBX_MODE_PEDESTRIAN || ubx->rtc_conf->nav_mode == UBX_MODE_SEA ? UBX_MODE_PORTABLE : UBX_MODE_PEDESTRIAN);
        }
        strbf_puts(&strbf, ", speed: ");
        strbf_putf(&strbf, time_cur_speed(time_2s));
        strbf_puts(&strbf, "mm/s\n");
        *strbf.cur = 0;
        WRITETXT(strbf.start, strbf.cur-strbf.start);
    }
}
#else 
inline void gps_log_nav_mode_change(gps_context_t *context, uint8_t changed) {}
#endif

// rate counted speed buffer
static inline void update_speed_buffer(gps_point_t * p, int32_t gSpeed) {
    if(log_p_lctx.index_gspeed == UINT32_MAX) log_p_lctx.index_gspeed = 0;
    else log_p_lctx.index_gspeed++;
    log_p_lctx.buf_gspeed[buf_index(log_p_lctx.index_gspeed)] = gSpeed;
#if !defined(CONFIG_GPS_LOG_STATIC_A_BUFFER)
    if (!log_p_lctx.alfa_buf) gps_check_alfa_buf(BUFFER_ALFA);
#endif
#if !defined(CONFIG_GPS_LOG_STATIC_S_BUFFER)
    if (!log_p_lctx.buf_sec_speed) gps_check_sec_buf(BUFFER_SEC_SIZE);
#endif
    log_p_lctx.alfa_buf[al_buf_index(log_p_lctx.index_gspeed)].latitude = p->latitude;
    log_p_lctx.alfa_buf[al_buf_index(log_p_lctx.index_gspeed)].longitude = p->longitude;
}

static inline void update_sec_speed_buffer(int32_t gSpeed, uint8_t sample_rate) {
    if(log_p_lctx.index_sec == UINT32_MAX) log_p_lctx.index_sec = 0;
    else log_p_lctx.sec_gSpeed += gSpeed;
    if (log_p_lctx.index_gspeed % sample_rate == 0) {
        log_p_lctx.index_sec++;
        log_p_lctx.buf_sec_speed[sec_buf_index(log_p_lctx.index_sec)] = log_p_lctx.sec_gSpeed / sample_rate;
        log_p_lctx.sec_gSpeed = 0;
    }
}

int get_cur_nav_mode(int nav_mode) {
    return (log_p_lctx.dynamic_state == 1 && nav_mode == UBX_MODE_PEDESTRIAN) ? UBX_MODE_PORTABLE
        : log_p_lctx.dynamic_state == 1 && nav_mode == UBX_MODE_SEA ? UBX_MODE_PORTABLE 
        : log_p_lctx.dynamic_state == 1 && nav_mode == UBX_MODE_PORTABLE ? UBX_MODE_PEDESTRIAN
        : nav_mode;
}

// This function will always put 3 variables from the GPS into a global buffer:
// doppler speed, lat and long. A global buffer was chosen because this data
// must also be available in other classes (GPS_speed() and GPS_time). The last
// buffer position is also stored in a global variable, log_p_lctx.index_gspeed
esp_err_t push_gps_data(gps_context_t *context, struct gps_data_s *me, float latitude, float longitude, int32_t gSpeed) {  // gspeed in mm/s !!!
    if(!context) return ESP_ERR_INVALID_ARG;
    ubx_config_t *ubx = context->ubx_device;
    ubx_msg_t * ubxMessage = &ubx->ubx_msg;
    uint8_t sample_rate = ubx->rtc_conf->output_rate;
    // if(ubx->rtc_conf->nav_mode_auto) {
        bool changed = 0;
        const float spd2s = time_cur_speed(time_2s);  // speed in mm/s
        if(log_p_lctx.dynamic_state == 0) {
            /// switch to dynamic_model "portable" as "pedestrian" only works with speed < 30 m/s (108kmh)
            /// and "sea" only works with speed < 25 m/s (90kmh)
            if (((spd2s > 29500 && ubx->rtc_conf->nav_mode == UBX_MODE_PEDESTRIAN) /// change over 28ms 100kmh
                || (spd2s > 23750 && ubx->rtc_conf->nav_mode == UBX_MODE_SEA))) { /// change over 24ms 85kmh
                log_p_lctx.dynamic_state = 1;
                ubx_set_nav_mode(ubx, UBX_MODE_PORTABLE);
            }
            else if (spd2s < 28500 && ubx->rtc_conf->nav_mode == UBX_MODE_PORTABLE) { /// change under 28ms 100kmh
                /// switch to dynamic_model "pedestrian" below 30ms as "portable" is less accurate
                log_p_lctx.dynamic_state = 1;
                ubx_set_nav_mode(ubx, UBX_MODE_PEDESTRIAN);
            }
            if(log_p_lctx.dynamic_state==1) changed=1;
        }
        else if(log_p_lctx.dynamic_state == 1) {
            /// switch back to "pedestrian" below 27ms as "pedestrian" is more accurate
            if(spd2s < 27500 && (ubx->rtc_conf->nav_mode == UBX_MODE_PEDESTRIAN)) { /// change below 24ms 86kmh
                log_p_lctx.dynamic_state = 0;
                ubx_set_nav_mode(ubx, UBX_MODE_PEDESTRIAN);
            }
            else if(spd2s < 21750 && (ubx->rtc_conf->nav_mode == UBX_MODE_SEA)) { /// change below 24ms 86kmh
                log_p_lctx.dynamic_state = 0;
                ubx_set_nav_mode(ubx, UBX_MODE_SEA);
            }
            else if (spd2s > 29500 && ubx->rtc_conf->nav_mode == UBX_MODE_PORTABLE) { /// change above 28ms 100kmh
                log_p_lctx.dynamic_state = 0;
                ubx_set_nav_mode(ubx, UBX_MODE_PORTABLE);

            }
            if(log_p_lctx.dynamic_state==0) changed=1;
        }
        if(changed) {
            esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_GPS_NAV_MODE_CHANGED, 0, 0, portMAX_DELAY);
        }
    // }

    if(xSemaphoreTake(log_p_lctx.xMutex, 0) != pdTRUE)
        return ESP_FAIL;
    
    update_speed_buffer(&((gps_point_t){latitude, longitude}), gSpeed);
    // only add distance if reception is good, be careful sometimes sAcc<2
    // sAcc is the horizontal accuracy estimate in mm, so 1000mm = 1m
    if ((ubxMessage->navPvt.numSV >= FILTER_MIN_SATS) && (MM_TO_M(ubxMessage->navPvt.sAcc) < FILTER_MAX_sACC)) {
        log_p_lctx.delta_dist = (float)gSpeed / sample_rate;  // convert speed to distance !!!
        me->total_distance += log_p_lctx.delta_dist;
        me->run_distance += log_p_lctx.delta_dist;
        me->run_distance_after_turn += log_p_lctx.delta_dist;
    }
    // Store groundSpeed per second
    // !!******************************************************
    update_sec_speed_buffer(gSpeed, sample_rate);
    xSemaphoreGive(log_p_lctx.xMutex);
#if (C_LOG_LEVEL < 3)
    WLOG(TAG, "-- gSpeed: %lu, sAcc: %lu, numSv: %hhu --", gSpeed, ubxMessage->navPvt.sAcc, ubxMessage->navPvt.numSV);
#endif
    return ESP_OK;
}

// constructor for GPS_data
struct gps_data_s *init_gps_data(struct gps_data_s *me) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    memset(me, 0, sizeof(struct gps_data_s));
    log_p_lctx.index_gspeed = UINT32_MAX;  // start at 0 on first pass !!
    log_p_lctx.index_sec = UINT32_MAX;  // start at 0 on first pass !!
    return me;
}

#endif // UBLOX_ENABLED
