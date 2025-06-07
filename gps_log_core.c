#include "log_private.h"
#include "gps_data.h"

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
// #include "logger_config.h"
#include "logger_common.h"
#include "numstr.h"
#include "ubx.h"
#include "gps_user_cfg.h"
#include "gps_log_events.h"

static const char * TAG = "gps_data";
//extern struct UBXMessage ubxMessage;

#define NUM_OF_SPD_ARRAY_SIZE 10 // number of arrays in the speed by time and speed by distance
// static const char *TAG = "GPS_data";
typedef struct gps_p_context_s {
    int32_t buf_gSpeed[BUFFER_SIZE];   // speed buffer counted by gps rate
    int16_t buf_secSpeed[BUFFER_SIZE]; // speed buffer counted by sec

    int32_t index_GPS;                 // by rate counted speed buffer pos (-1), start at 0 on first pass !!
    int32_t index_sec;                 // by sec counted speed buffer pos (-1), start at 0 on first pass !!

    int32_t sec_gSpeed;      // for avg speed per second mm/s
    uint32_t delay_count_before_run;    // count loops to wait before incerment the run count
    uint16_t run_count;      // counter for runs
    uint16_t old_run_count;  // previous run counter
    uint16_t alfa_count;     // counter for alfa
    uint16_t old_alfa_count; // previous alfa counter

    bool velocity_0;      // min gemiddelde over 2 s < 1m/s
    bool velocity_5;      // min gemiddelde over 2 s = 1m/s
    bool dynamic_state;   // ubx setting dynamic model state: sea or portable here !!

    bool straight_course; // straight course or not
    gps_point_t buf_alfa[BUFFER_ALFA]; // alfa points buffer
    gps_point_t alfa_p1; // Point 1 for distance calculation
    gps_point_t alfa_p2; // Point 2 for distance calculation

    float delta_heading;
    float delta_dist;
    float heading;     // heding for alfa // = 0;
    float old_heading; // old heading for alfa // = 0
    float heading_mean;
    // Points for distance calculation

    SemaphoreHandle_t xMutex;
} gps_p_context_t;

#define GPS_P_CONTEXT_INIT { \
    .buf_gSpeed = {0}, \
    .buf_secSpeed = {0}, \
    .index_GPS = -1, \
    .index_sec = -1, \
    .sec_gSpeed = 0, \
    .delay_count_before_run = 0, \
    .run_count = 0, \
    .old_run_count = 0, \
    .alfa_count = 0, \
    .old_alfa_count = 0, \
    .velocity_0 = false, \
    .velocity_5 = false, \
    .straight_course = false, \
    .dynamic_state = 0, \
    .delta_heading = 0, \
    .heading_mean = 0, \
    .buf_alfa = {{0}}, \
    .delta_dist = 0, \
    .heading = 0, \
    .old_heading = 0, \
    .alfa_p1 = {0,0}, \
    .alfa_p2 = {0,0}, \
    .xMutex = NULL \
}
gps_p_context_t lctx = GPS_P_CONTEXT_INIT;

// #define GPS_TRACE_MSG_SPEED_BY_DIST 1
// #define GPS_TRACE_MSG_SPEED_BY_TIME 1
// #define GPS_TRACE_MSG_SPEED_ALPHA 1
// #if (C_LOG_LEVEL < 2)
// #define GPS_STATS 1
// #endif
#if defined(GPS_STATS)
static esp_timer_handle_t gps_timer = 0;
static esp_err_t gps_data_printf();
#if defined(GPS_MSG_CTX)
static esp_err_t gps_p_context_printf(const gps_p_context_t *me);
#endif
#if defined(GPS_TRACE_MSG_SAT)
static esp_err_t gps_sat_info_printf(const struct GPS_SAT_info *me);
#endif
#if defined(GPS_TRACE_MSG_SPEED_BY_DIST)
static esp_err_t gps_speed_by_dist_printf(const struct gps_speed_by_dist_s *me);
#endif
#if defined(GPS_TRACE_MSG_SPEED_BY_TIME)
static esp_err_t gps_speed_by_time_printf(const struct gps_speed_by_time_s *me);
#endif
#if defined(GPS_TRACE_MSG_SPEED_ALPHA)
esp_err_t gps_speed_by_alpha_printf(const struct gps_speed_alfa_s *me);
#endif
#endif

#if defined(GPS_STATS)
static void s(void*arg) {
    gps_context_t *me = (gps_context_t *)arg;
    printf("--- GPS periodic timer callback %lu ---\n", get_millis());
    gps_data_printf(&me->Ublox);
#if defined(GPS_MSG_CTX)
    gps_p_context_printf(&lctx);
#endif
#if defined(GPS_TRACE_MSG_SAT)
    gps_sat_info_printf(me);
#endif
#if defined(GPS_TRACE_MSG_SPEED_BY_DIST)
    // gps_speed_by_dist_printf(&me->M100);
    // gps_speed_by_dist_printf(&me->M250);
    gps_speed_by_dist_printf(&me->M500);
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

static inline int32_t al_buf_index(int32_t idx) {
    return (idx + BUFFER_ALFA) % BUFFER_ALFA;
}

static inline int32_t buf_index(int32_t idx) {
    return (idx + BUFFER_SIZE) % BUFFER_SIZE;
}

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
    printf("index_GPS: %ld, ", me->index_GPS);
    printf("index_sec: %ld\n", me->index_sec);
    printf("buf_gSpeed: ");
    uint8_t i, j=me->index_GPS%BUFFER_SIZE;
    if(j<10) j=10;
    for (i = j-10; i < j; i++) printf(" %ld", me->buf_gSpeed[i]);
    printf("\n");
    j=me->index_sec%BUFFER_SIZE;
    if(j<10) j=10;
    printf("buf_secSpeed: ");
    for (i = j-10; i < j; i++) printf(" %hu", me->buf_secSpeed[i]);
    printf("\n");
    j=me->index_GPS%BUFFER_SIZE;
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
    return smooth(lctx.buf_gSpeed, lctx.index_GPS, BUFFER_SIZE, window_size);
}

int32_t gps_last_sec_speed_smoothed(uint8_t window_size) {
    return smooth((int32_t*)lctx.buf_secSpeed, lctx.index_sec, BUFFER_SIZE, window_size);
}

void init_gps_context_fields(gps_context_t *ctx) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    assert(ctx);
    init_gps_data(&ctx->Ublox);  // create an object storing GPS_data !
    init_gps_sat_info(&ctx->Ublox_Sat);  // create an object storing GPS_SAT info !
    init_gps_speed(&ctx->M100, 100);
    init_gps_speed(&ctx->M250, 250);
    init_gps_speed(&ctx->M500, 500);
    init_gps_speed(&ctx->M1852, 1852);
    init_gps_time(&ctx->S2, 2);
    init_gps_time(&ctx->S10, 10);
    init_gps_time(&ctx->S1800, 1800);
    init_gps_time(&ctx->S3600, 3600);
    init_alfa_speed(&ctx->A250, 250);
    init_alfa_speed(&ctx->A500, 500);
#if defined(CONFIG_LOGGER_BUTTON_GPIO_1_ACTIVE)
    init_gps_time(&ctx->s2, 2);         // for  stats GPIO_12 screens, reset possible !!
    init_gps_time(&ctx->s10, 10);        // for  stats GPIO_12 screens, reset possible !!
    init_alfa_speed(&ctx->a500, 500);     // for  Alfa stats GPIO_12 screens, reset possible !!
#endif
    if (ctx->ubx_device == NULL){
        ctx->ubx_device = ubx_config_new();
    }
    if (ctx->log_config == NULL)
        ctx->log_config = &log_config;
    if(lctx.xMutex == NULL)
        lctx.xMutex = xSemaphoreCreateMutex();
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
    assert(ctx);
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
    if(lctx.xMutex != NULL){
        vSemaphoreDelete(lctx.xMutex);
        lctx.xMutex = NULL;
    }
    gps_user_cfg_deinit();
    ctx->Gps_fields_OK = 0;
}

#if defined(GPS_STATS)
static esp_err_t gps_data_printf(struct gps_data_s *me) {
    printf("gps_data:{ ");
    printf("run_start_time: %lu, ", me->run_start_time);
    printf("run_distance: %.02f, ", me->run_distance);
    printf("alfa_distance: %.02f, ", me->alfa_distance);
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
        if(lctx.dynamic_state==0) {
            strbf_puts(&strbf, " back ");
            strbf_putul(&strbf, ubx->rtc_conf->nav_mode == UBX_MODE_PEDESTRIAN ? UBX_MODE_PEDESTRIAN : UBX_MODE_PORTABLE);
        }
        else {
            strbf_puts(&strbf, " to ");
            strbf_putul(&strbf, ubx->rtc_conf->nav_mode == UBX_MODE_PEDESTRIAN || ubx->rtc_conf->nav_mode == UBX_MODE_SEA ? UBX_MODE_PORTABLE : UBX_MODE_PEDESTRIAN);
        }
        strbf_puts(&strbf, ", speed: ");
        strbf_putf(&strbf, context->S2.speed.speed);
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
    lctx.index_GPS++;
    lctx.buf_gSpeed[buf_index(lctx.index_GPS)] = gSpeed;
    lctx.buf_alfa[al_buf_index(lctx.index_GPS)].latitude = p->latitude;
    lctx.buf_alfa[al_buf_index(lctx.index_GPS)].longitude = p->longitude;
}

static inline void update_sec_speed_buffer(int32_t gSpeed, uint8_t sample_rate) {
    lctx.sec_gSpeed += gSpeed;
    if (lctx.index_GPS % sample_rate == 0) {
        lctx.index_sec++;
        lctx.buf_secSpeed[buf_index(lctx.index_sec)] = lctx.sec_gSpeed / sample_rate;
        lctx.sec_gSpeed = 0;
    }
}

static inline void store_time(gps_run_t *run) {
    struct tm tms;
    get_local_time(&tms);
    run->time.hour = tms.tm_hour;
    run->time.minute = tms.tm_min;
    run->time.second = tms.tm_sec;
}

// This function will always put 3 variables from the GPS into a global buffer:
// doppler speed, lat and long. A global buffer was chosen because this data
// must also be available in other classes (GPS_speed() and GPS_time). The last
// buffer position is also stored in a global variable, lctx.index_GPS
esp_err_t push_gps_data(gps_context_t *context, struct gps_data_s *me, float latitude, float longitude, int32_t gSpeed) {  // gspeed in mm/s !!!
    assert(context);
    // logger_config_t *config = context->config;
    // assert(config);
    ubx_config_t *ubx = context->ubx_device;
    ubx_msg_t * ubxMessage = &ubx->ubx_msg;
    uint8_t sample_rate = ubx->rtc_conf->output_rate;
    // if(ubx->rtc_conf->nav_mode_auto) {
        bool changed = 0;
        if(lctx.dynamic_state == 0) {
            /// switch to dynamic_model "portable" as "pedestrian" only works with speed < 30 m/s (108kmh)
            /// and "sea" only works with speed < 25 m/s (90kmh)
            if (((context->S2.speed.speed > 29500 && ubx->rtc_conf->nav_mode == UBX_MODE_PEDESTRIAN) /// change over 28ms 100kmh
                || (context->S2.speed.speed > 23750 && ubx->rtc_conf->nav_mode == UBX_MODE_SEA))) { /// change over 24ms 85kmh
                lctx.dynamic_state = 1;
                ubx_set_nav_mode(ubx, UBX_MODE_PORTABLE);
            }
            else if (context->S2.speed.speed < 28500 && ubx->rtc_conf->nav_mode == UBX_MODE_PORTABLE) { /// change under 28ms 100kmh
                /// switch to dynamic_model "pedestrian" below 30ms as "portable" is less accurate
                lctx.dynamic_state = 1;
                ubx_set_nav_mode(ubx, UBX_MODE_PEDESTRIAN);
            }
            if(lctx.dynamic_state==1) changed=1;
        }
        else if(lctx.dynamic_state == 1) {
            /// switch back to "pedestrian" below 27ms as "pedestrian" is more accurate
            if(context->S2.speed.speed < 27500 && (ubx->rtc_conf->nav_mode == UBX_MODE_PEDESTRIAN)) { /// change below 24ms 86kmh
                lctx.dynamic_state = 0;
                ubx_set_nav_mode(ubx, UBX_MODE_PEDESTRIAN);
            }
            else if (context->S2.speed.speed > 29500 && ubx->rtc_conf->nav_mode == UBX_MODE_PORTABLE) { /// change above 28ms 100kmh
                lctx.dynamic_state = 0;
                ubx_set_nav_mode(ubx, UBX_MODE_PORTABLE);

            }
            if(lctx.dynamic_state==0) changed=1;
        }
        if(changed) {
            esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_GPS_NAV_MODE_CHANGED, 0, 0, portMAX_DELAY);
        }
    // }

    if(xSemaphoreTake(lctx.xMutex, 0) != pdTRUE)
        return ESP_FAIL;
    
    update_speed_buffer(&((gps_point_t){latitude, longitude}), gSpeed);
    // only add distance if reception is good, be careful sometimes sAcc<2
    // sAcc is the horizontal accuracy estimate in mm, so 1000mm = 1m
    if ((ubxMessage->navPvt.numSV >= FILTER_MIN_SATS) && (MM_TO_M(ubxMessage->navPvt.sAcc) < FILTER_MAX_sACC)) {
        lctx.delta_dist = gSpeed / sample_rate;  // convert speed to distance !!!
        me->total_distance += lctx.delta_dist;
        me->run_distance += lctx.delta_dist;
        me->alfa_distance += lctx.delta_dist;
    }
// #if (C_LOG_LEVEL < 3)
//     WLOG(TAG, "-- sAcc: %lu, numSv: %hhu --", ubxMessage->navPvt.sAcc, ubxMessage->navPvt.numSV);
// #endif
    // Store groundSpeed per second
    // !!******************************************************
    update_sec_speed_buffer(gSpeed, sample_rate);

    xSemaphoreGive(lctx.xMutex);
    return ESP_OK;
}

// constructor for GPS_data
struct gps_data_s *init_gps_data(struct gps_data_s *me) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    memset(me, 0, sizeof(struct gps_data_s));
    lctx.index_GPS = 0;
    return me;
}

// constructor for SAT_info
struct GPS_SAT_info *init_gps_sat_info(struct GPS_SAT_info *me) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    memset(me, 0, sizeof(struct GPS_SAT_info));
    me->index_SAT_info = 0;
    return me;
}

#if defined(GPS_STATS) && defined(GPS_TRACE_MSG_SAT)
static esp_err_t gps_sat_info_printf(const struct GPS_SAT_info *me) {
    printf("GPS_SAT_info:{ ");
    printf("mean_cno: %hu, ", me->mean_cno);
    printf("min_cno: %hhu, ", me->min_cno);
    printf("max_cno: %hhu, ", me->max_cno);
    printf("nr_sats: %hhu, ", me->nr_sats);
    printf("index_SAT_info: %lu, ", me->index_SAT_info);
    printf("Mean_mean_cno: %hu, ", me->sat_info.Mean_mean_cno);
    printf("Mean_max_cno: %"PRIu8", ", me->sat_info.Mean_max_cno);
    printf("Mean_min_cno: %hhu, ", me->sat_info.Mean_min_cno);
    printf("Mean_numSV: %hhu, ", me->sat_info.Mean_numSV);
    printf("}\n");
    return ESP_OK;
}
#endif

// function to extract info out of NAV_SAT, and push it to array
// For every NAV_SAT frame, the Mean CNO, the Max cno, the Min cno and the nr of
// sats in the nav solution are stored Then, the means are calculated out of the
// last NAV_SAT_BUFFER frames (now 16 frames, @5Hz, this 0.5Hz NAV_SAT ca 32 s)
void push_gps_sat_info(struct GPS_SAT_info *me, struct nav_sat_s *nav_sat) {
    // #define NAV_SAT_BUFFER 10
    me->mean_cno = 0;
    me->min_cno = 0xFF;
    me->max_cno = 0;
    me->nr_sats = 0;
    for (uint8_t i = 0; i < nav_sat->numSvs; i++) {  // only evaluate nr of Sats that is in NAV_SAT
        if (nav_sat->sat[i].flags & 0x8) {  // only evaluate nr of Sats who are in nav solution, bit3 from X4
            me->mean_cno = me->mean_cno + nav_sat->sat[i].cno;
            if (nav_sat->sat[i].cno < me->min_cno)
                me->min_cno = nav_sat->sat[i].cno;
            if (nav_sat->sat[i].cno > me->max_cno)
                me->max_cno = nav_sat->sat[i].cno;
            me->nr_sats++;
        }
    }
    if (me->nr_sats) {  // protection divide int/0 !!
        me->mean_cno = me->mean_cno / me->nr_sats;
        uint32_t idx = me->index_SAT_info % NAV_SAT_BUFFER;
        me->sat_info.Mean_cno[idx] = me->mean_cno;
        me->sat_info.Max_cno[idx] = me->max_cno;
        me->sat_info.Min_cno[idx] = me->min_cno;
        me->sat_info.numSV[idx] = me->nr_sats;
        me->mean_cno = 0;
        me->min_cno = 0;
        me->max_cno = 0;
        me->nr_sats = 0;
        if (me->index_SAT_info > NAV_SAT_BUFFER) {
            for (uint8_t i = 0; i < NAV_SAT_BUFFER; i++) {
                idx = (me->index_SAT_info - NAV_SAT_BUFFER + i) % NAV_SAT_BUFFER;
                me->mean_cno = me->mean_cno + me->sat_info.Mean_cno[idx];
                me->max_cno = me->max_cno + me->sat_info.Max_cno[idx];
                me->min_cno = me->min_cno + me->sat_info.Min_cno[idx];
                me->nr_sats = me->nr_sats + me->sat_info.numSV[idx];
            }
            me->mean_cno = me->mean_cno / NAV_SAT_BUFFER;
            me->max_cno = me->max_cno / NAV_SAT_BUFFER;
            me->min_cno = me->min_cno / NAV_SAT_BUFFER;
            me->nr_sats = me->nr_sats / NAV_SAT_BUFFER;
            me->sat_info.Mean_mean_cno = me->mean_cno;
            me->sat_info.Mean_max_cno = me->max_cno;
            me->sat_info.Mean_min_cno = me->min_cno;
            me->sat_info.Mean_numSV = me->nr_sats;
        }
        me->index_SAT_info++;
    }
};

#if defined(GPS_STATS)
static void gps_run_printf(const struct gps_run_s * run) {
    printf("Run: {time: %02d:%02d.%02d, avg_speed: %.02f, nr_run: %hu}\n", run->time.hour, run->time.minute, run->time.second, run->avg_speed, run->nr_run);
}
#endif

// static void sort_run(gps_run_t runs[], uint16_t mean_cno[], uint8_t max_cno[], uint8_t min_cno[], uint8_t nr_sats[], uint8_t size) {
//     // printf("[%s]\n", __func__);
//     for (uint8_t i = 0; i < (size - 1); i++) {
//         for (uint8_t o = 0; o < (size - (i + 1)); o++) {
//             if (runs[o].avg_speed > runs[o + 1].avg_speed) {
//                 gps_run_t t = runs[o]; runs[o] = runs[o + 1]; runs[o + 1] = t;
//                 if(mean_cno) {uint8_t f = mean_cno[o];mean_cno[o] = mean_cno[o + 1];mean_cno[o + 1] = f;}
//                 if(max_cno) {uint8_t g = max_cno[o];max_cno[o] = max_cno[o + 1];max_cno[o + 1] = g;}
//                 if(min_cno) {uint8_t h = min_cno[o];min_cno[o] = min_cno[o + 1];min_cno[o + 1] = h;}
//                 if(nr_sats) {uint8_t j = nr_sats[o];nr_sats[o] = nr_sats[o + 1];nr_sats[o + 1] = j;}
//             }
//         }
//     }
// }

// static void sort_run_alfa(gps_run_t runs[], int32_t dist[], uint32_t nr_samples[], uint32_t message_no[], uint8_t size) {
//     // printf("[%s]\n", __func__);
//     for (uint8_t i = 0; i < (size - 1); i++) {
//         for (uint8_t o = 0; o < (size - (i + 1)); o++) {
//             if (runs[o].avg_speed > runs[o + 1].avg_speed) {
//                 gps_run_t t = runs[o]; runs[o] = runs[o + 1]; runs[o + 1] = t;
//                 if(dist) {uint32_t f = dist[o]; dist[o] = dist[o + 1]; dist[o + 1] = f;}
//                 if(nr_samples) {uint32_t g = nr_samples[o]; nr_samples[o] = nr_samples[o + 1]; nr_samples[o + 1] = g;}
//                 if(message_no) {uint32_t h = message_no[o]; message_no[o] = message_no[o + 1]; message_no[o + 1] = h;}
//             }
//         }
//     }
// }


// Helper for byte-wise swap
static inline void swap_bytes(void *a, void *b, size_t n) {
    uint8_t *pa = (uint8_t *)a;
    uint8_t *pb = (uint8_t *)b;
    for (uint8_t k = 0; k < n; ++k) {
        uint8_t tmp = pa[k]; pa[k] = pb[k];pb[k] = tmp;
    }
}

// Unified sort routine for gps_run_t and up to n parallel arrays
static void sort_runs(
    gps_run_t runs[],
    void *arrs[], size_t strides[], int num_arrays,
    uint8_t size
) {
    for (uint8_t i = 0; i < (size - 1); i++) {
        for (uint8_t o = 0; o < (size - (i + 1)); o++) {
            if (runs[o].avg_speed > runs[o + 1].avg_speed) {
                gps_run_t t = runs[o]; runs[o] = runs[o + 1]; runs[o + 1] = t;
                for (int arr_idx = 0; arr_idx < num_arrays; ++arr_idx) {
                    if (arrs[arr_idx]) {
                        swap_bytes(
                            (uint8_t*)arrs[arr_idx] + o * strides[arr_idx],
                            (uint8_t*)arrs[arr_idx] + (o + 1) * strides[arr_idx],
                            strides[arr_idx]
                        );
                    }
                }
            }
        }
#if defined(GPS_STATS)
        gps_run_printf(&runs[i]);
#endif
    }
}

static void sort_display(float a[], uint8_t size) {
    for (uint8_t i = 0; i < (size - 1); i++) {
        for (uint8_t o = 0; o < (size - (i + 1)); o++) {
            if (a[o] > a[o + 1]) {
                float t = a[o]; a[o] = a[o + 1]; a[o + 1] = t;
            }
        }
    }
}

#if defined(GPS_STATS)
static esp_err_t gps_display_printf(const gps_display_t * me) {
    uint8_t i, j=NUM_OF_SPD_ARRAY_SIZE;
    printf(" display:{ \n");
    printf("  max_speed: %.02f, ", me->display_max_speed);
    printf("last_max_speed: %.02f, ", me->display_last_run_max_speed);
    printf("record: %d\n", me->record);
    printf("  speed: ");
    for (i = 0; i < j; i++) printf("%.02f ", me->display_speed[i]);
    printf("\n }\n");
    return ESP_OK;
}

static esp_err_t gps_speed_printf(const gps_speed_t * me) {
    uint8_t i, j=NUM_OF_SPD_ARRAY_SIZE;
    printf(" == speed:{\n");
    printf(" speed: %.02f, ", me->speed);
    printf("speed_alfa: %.02f, ", me->speed_alfa);
    printf("max_speed: %.02f, ", me->max_speed);
    printf("avg_5runs: %.02f\n", me->avg_5runs);
    for (i = 0; i < j; i++) {
        printf(" %hhu ", i); gps_run_printf(&me->runs[i]);
    }
    gps_display_printf(&me->display);
    printf(" } ==\n");
    return ESP_OK;
}
#endif

#if defined(GPS_STATS) && defined(GPS_TRACE_MSG_SPEED_BY_DIST)
static esp_err_t gps_speed_by_dist_printf(const struct gps_speed_by_dist_s *me) {
    uint8_t i, j=NUM_OF_SPD_ARRAY_SIZE;
    printf("=== speed_by_dist: {\n");
    printf("m_set_dist: %hu, ", me->set_distance);
    printf("m_dist: %ld, ", me->distance);
    printf("m_dist_alfa: %ld, ", me->distance_alfa);
    printf("m_sample: %lu, ", me->m_sample);
    printf("m_index: %ld\n", me->m_index);
    gps_speed_printf(&me->speed);
    printf("dist: ");
    for (i = 0; i < j; i++) printf("%lu ", me->dist[i]);
    printf("\n");
    printf("nr_samples: ");
    for (i = 0; i < j; i++) printf("%lu ", me->nr_samples[i]);
    printf("\n");
    printf("message_nr: ");
    for (i = 0; i < j; i++) printf("%lu ", me->message_nr[i]);
    printf("\n");
    printf("} === \n");
    return ESP_OK;
}
#endif

static void record_last_run(gps_speed_t * speed, uint16_t actual_run) {
    // printf("[%s] %.01f %hu %hu\n", __func__, speed->max_speed, actual_run, speed->display.nr_display_last_run);
    if ((actual_run != speed->display.nr_display_last_run) && (speed->max_speed > 3000.0f)) { // 3m/s
        speed->display.nr_display_last_run = actual_run;
        speed->display.display_last_run_max_speed = 0;
    } 
    else if (speed->display.display_last_run_max_speed < speed->max_speed) {
        speed->display.display_last_run_max_speed = speed->max_speed;
    }
}

static inline void reset_runs_avg(gps_run_t runs[]) {
    // printf("[%s]\n", __func__);
    memset(runs, 0, NUM_OF_SPD_ARRAY_SIZE * sizeof(gps_run_t));
}

static inline void reset_display_speed(float * arr) {
    // printf("[%s]\n", __func__);
    memset(arr, 0, NUM_OF_SPD_ARRAY_SIZE * sizeof(float));
}

static void update_avg_5runs(gps_speed_t * speed, bool mode) {
    // printf("[%s]\n", __func__);
    speed->avg_5runs = speed->runs[(mode ? 0 : 5)].avg_speed;
    for(uint8_t i = 6, j = mode ? 9 : NUM_OF_SPD_ARRAY_SIZE; i < j; i++) speed->avg_5runs += speed->runs[i].avg_speed;
    speed->avg_5runs /= 5;
}

static inline void update_display_speed_array(gps_display_t * display, gps_run_t runs[], uint8_t start, uint8_t end) {
    // printf("[%s]\n", __func__);
    for (uint8_t i = start; i < end; i++) display->display_speed[i] = runs[i].avg_speed;
}

static inline void refresh_display_speeds(gps_display_t * display, gps_run_t runs[], float max_speed) {
    // printf("[%s]\n", __func__);
    display->display_speed[5] = max_speed;
    update_display_speed_array(display, runs, 6, NUM_OF_SPD_ARRAY_SIZE);
    sort_display(display->display_speed, NUM_OF_SPD_ARRAY_SIZE);
}

static uint8_t update_display_speeds(gps_speed_t * speed, uint8_t * record) {
    uint8_t ret = 0;
    if (speed->max_speed > speed->runs[5].avg_speed) {
        refresh_display_speeds(&speed->display, speed->runs, speed->max_speed);
        ret = 1;
    }
    if (speed->max_speed > speed->runs[9].avg_speed) {
        speed->display.display_max_speed = speed->max_speed;  // update on the fly, that's not correct here !!!
        speed->display.record = 1;
        *record = 1;
        ret = 2;
    }
    else
        speed->display.display_max_speed = speed->runs[9].avg_speed;
    return ret;
}

static void reset_gps_last_speed(gps_speed_t * speed) {
    update_avg_5runs(speed, false);  // calculate the average of the last 5 best runs
    speed->max_speed = 0;
    memset(&speed->runs[0], 0, sizeof(gps_run_t));
    speed->display.record = 0;
}

static bool store_max_speed(gps_speed_t * speed, uint16_t run_count) {
    if (speed->max_speed < speed->speed) {
        speed->max_speed = speed->speed;
        store_time(&speed->runs[0]);
        speed->runs[0].nr_run = run_count;
        speed->runs[0].avg_speed = speed->max_speed;
        return 1;
    }
    return 0;
}

/// Instance to determine the average speed over a certain distance, 
/// when a new run starts, save the highest speed of the previous run.
struct gps_speed_by_dist_s *init_gps_speed(struct gps_speed_by_dist_s *me, uint16_t dist) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    memset(me, 0, sizeof(struct gps_speed_by_dist_s));
    me->set_distance = dist;
    return me;
}

#if defined(CONFIG_LOGGER_BUTTON_GPIO_1_ACTIVE)
void reset_speed_stats(struct gps_speed_by_dist_s *me) {
    reset_runs_avg(me->speed.runs);
    reset_display_speed(me->speed.display.display_speed);
}
#endif

static inline bool store_speed_by_dist(struct gps_speed_by_dist_s *me, uint32_t set_distance) {
    // printf("[%s]\n", __func__);
    me->m_sample = lctx.index_GPS - me->m_index + 1;  // Check for the number of samples to avoid division by zero
    if (me->m_sample > 0) {
        // Calculate the distance in mm, so multiply by 1000 and consider the sample_rate !!
        me->speed.speed = me->distance / me->m_sample;  // 10 samples op 1s aan 10mm/s = 100/10 = 10 mm /s
    }
    if (lctx.index_GPS > me->m_index) {
        // Calculate the speed based on the distance and the number of samples
        me->speed.speed_alfa = me->distance_alfa / (lctx.index_GPS - me->m_index);
    }
    if (me->distance < set_distance) me->speed.speed = 0;  // This is to prevent incorrect speed if the distance has not been reached yet!
    if (me->m_sample >= BUFFER_SIZE) me->speed.speed = 0;  // This is to prevent incorrect speed if there is a BUFFER_SIZE overflow !!
    if (me->speed.speed == 0) me->speed.speed_alfa = 0;
    return me->speed.speed > 0;  // return the speed in mm/s
}

static inline void store_dist_data(gps_context_t *context, struct gps_speed_by_dist_s *me) {
    if (store_max_speed(&me->speed, context->run_count)) {  // store max speed of this run
        me->dist[0] = me->distance;
        me->nr_samples[0] = me->m_sample;
        me->message_nr[0] = context->ubx_device->ubx_msg.count_nav_pvt;
    }
    update_display_speeds(&me->speed, &context->record);
}

static inline void store_and_reset_dist_data_after_run(gps_context_t *context, struct gps_speed_by_dist_s *me) {
#if (C_LOG_LEVEL < 2)
    ILOG(TAG, "[%s]", __func__);
#endif
    // sort_run_alfa(me->speed.runs, me->dist, me->message_nr, (uint32_t*)me->nr_samples, 10);
    sort_runs(
        me->speed.runs,
        (void *[]){me->dist, me->nr_samples, me->message_nr},
        (size_t[]){sizeof(me->dist[0]), sizeof(me->nr_samples[0]), sizeof(me->message_nr[0])},
        3, NUM_OF_SPD_ARRAY_SIZE
    );
    update_display_speed_array(&me->speed.display, me->speed.runs, 0, NUM_OF_SPD_ARRAY_SIZE);
    reset_gps_last_speed(&me->speed);
}

static inline void move_distance_window(struct gps_speed_by_dist_s *me, uint32_t set_distance) {
    // printf("[%s]\n", __func__);
    // Determine buffer m_index for the desired distance
    if(me->distance > set_distance) {
        while (me->distance > set_distance && (lctx.index_GPS - me->m_index) < BUFFER_SIZE) {
            me->distance = me->distance - lctx.buf_gSpeed[me->m_index % BUFFER_SIZE];
            me->distance_alfa = me->distance;
            me->m_index++;
        }
        me->m_index--;
        me->distance = me->distance + lctx.buf_gSpeed[me->m_index % BUFFER_SIZE];
    }
}

float update_speed_by_distance(gps_context_t *context, struct gps_speed_by_dist_s *me) {
    // printf("[%s]\n", __func__);
    assert(context);
    uint32_t set_distance = M_TO_MM(me->set_distance) * context->ubx_device->rtc_conf->output_rate;  // Note that m_set_distance should now be in mm, so multiply by 1000 and consider the sample_rate !!
    me->distance = me->distance + lctx.buf_gSpeed[lctx.index_GPS % BUFFER_SIZE];  // the resolution of the distance is 0.1 mm
                                                                                  // the max int32  is 2,147,483,647 mm eq 214,748.3647 meters eq ~214 kilometers !!
    if ((lctx.index_GPS - me->m_index) >= BUFFER_SIZE) {  // controle buffer overflow
        me->distance = 0;
        me->m_index = lctx.index_GPS;
    }
    move_distance_window(me, set_distance);
    if(store_speed_by_dist(me, set_distance)) {  // store the speed if it is greater than 0
        store_dist_data(context, me);  // store the data in the speed struct
    }
    if ((context->run_count != me->speed.nr_prev_run) && (me->speed.runs[0].nr_run == me->speed.nr_prev_run)) {  // opslaan hoogste snelheid van run + sorteren
        store_and_reset_dist_data_after_run(context, me);
    }
    me->speed.nr_prev_run = context->run_count;
    record_last_run(&me->speed, context->run_count);
    return me->speed.max_speed;
}

/// Instance to determine the average speed over a certain time window
struct gps_speed_by_time_s *init_gps_time(struct gps_speed_by_time_s *me, uint16_t window) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    memset(me, 0, sizeof(struct gps_speed_by_time_s));
    me->time_window = window;
    return me;
}

#if defined(GPS_STATS) && defined(GPS_TRACE_MSG_SPEED_BY_TIME)
#if defined(SPEED_BAR_SETUP)
static void gps_speed_bar_data_printf(const struct gps_speed_bar_s *bar) {
    printf("bar_data: {bar_count: %hu, run_speeds: ", bar->bar_count);
    for (uint8_t i = 0; i < NR_OF_BAR; i++) {
        printf("%.02f ", bar->run_speed[i]);
    }
    printf("}\n");
}
#endif
static esp_err_t gps_speed_by_time_printf(const struct gps_speed_by_time_s *me) {
    printf("=== gps_speed_by_time: {\n");
    printf("time_window: %hu, ", me->time_window);
    gps_speed_printf(&me->speed);
    // printf("avg_s_sum: %ld\n", me->avg_s_sum);
    // printf("Mean_cno: ");
    // for (i = 0; i < j; i++)
    //     printf("%hu ", me->Mean_cno[i]);
    // printf("\n");
    // printf("Max_cno: ");
    // for (i = 0; i < j; i++)
    // printf("%hhu ", me->Max_cno[i]);
    // printf("\n");
    // printf("Min_cno: ");
    // for (i = 0; i < j; i++)
    //     printf("%hhu ", me->Min_cno[i]);
    // printf("\n");
    // printf("Mean_numSat: ");
    // for (i = 0; i < j; i++)
    //     printf("%hhu ", me->Mean_numSat[i]);
    // printf("\n");
    printf("} === \n");
    return ESP_OK;
}
#endif
#if defined(CONFIG_LOGGER_BUTTON_GPIO_1_ACTIVE)
void reset_time_stats(struct gps_speed_by_time_s *me) {
    reset_runs_avg(me->speed.runs);
    reset_display_speed(me->speed.display.display_speed);
}
#endif

#if defined(SPEED_BAR_SETUP)
static inline void store_speed_bar_data(struct gps_speed_by_time_s *me, uint16_t run_count) {
    if (me->speed.max_speed > 5000.0f) me->bar.bar_count++;  // min speed bar graph = 5 m/s
    me->bar.run_speed[run_count % NR_OF_BAR] = me->speed.runs[0].avg_speed;
}
#endif

static inline void store_speed_by_time_data(gps_context_t *context, struct gps_speed_by_time_s *me) {
    // printf("[%s]\n", __func__);
    if (store_max_speed(&me->speed, context->run_count)) {
        me->Mean_cno[0] = context->Ublox_Sat.sat_info.Mean_mean_cno;
        me->Max_cno[0] = context->Ublox_Sat.sat_info.Mean_max_cno;
        me->Min_cno[0] = context->Ublox_Sat.sat_info.Mean_min_cno;
        me->Mean_numSat[0] = context->Ublox_Sat.sat_info.Mean_numSV;
#if defined(SPEED_BAR_SETUP)
        me->bar.run_speed[context->run_count % NR_OF_BAR] = me->speed.speed;
#endif
    }
    if(update_display_speeds(&me->speed, &context->record)) {
        update_avg_5runs(&me->speed, true); // average of the runs 0 and 6-9
    }
}

static inline void store_and_reset_time_data_after_run(gps_context_t *context, struct gps_speed_by_time_s *me) {
#if (C_LOG_LEVEL < 2)
    ILOG(TAG, "[%s]", __func__);
#endif
    sort_runs(
        me->speed.runs,
        (void *[]){me->Mean_cno, me->Max_cno, me->Min_cno, me->Mean_numSat},
        (size_t[]){sizeof(me->Mean_cno[0]), sizeof(me->Max_cno[0]), sizeof(me->Min_cno[0]), sizeof(me->Mean_numSat[0])},
        4, NUM_OF_SPD_ARRAY_SIZE
    );
    update_display_speed_array(&me->speed.display, me->speed.runs, 0, NUM_OF_SPD_ARRAY_SIZE);  // update the runs array
#if defined(SPEED_BAR_SETUP)
    store_speed_bar_data(me, context->run_count);  // store the speed bar data
#endif
    reset_gps_last_speed(&me->speed);  // reset the speed for the next run
}

static inline bool store_avg_speed_by_time(struct gps_speed_by_time_s *me, uint32_t time_window_delta, uint8_t sample_rate) {
    // printf("[%s] %lu\n", __func__, time_window_delta);
    bool window_reached = false;
    if (time_window_delta < BUFFER_SIZE) {  // if time window is smaller than the sample_rate*BUFFER, use normal buffer
        me->avg_s_sum += lctx.buf_gSpeed[lctx.index_GPS % BUFFER_SIZE];  // always add gSpeed at every update
        if (lctx.index_GPS >= time_window_delta) { // once 10s is reached, subtract -10s from the sum again
            me->avg_s_sum = me->avg_s_sum - lctx.buf_gSpeed[(lctx.index_GPS - time_window_delta) % BUFFER_SIZE];
            window_reached = true;  // only if the time window is reached, we can calculate the speed
            me->speed.speed = me->avg_s_sum / me->time_window / sample_rate;
        }
    } else if (lctx.index_GPS % sample_rate == 0) {  // switch to seconds buffer, but only one update per second !!
         me->avg_s_sum += lctx.buf_secSpeed[lctx.index_sec % BUFFER_SIZE];  // lctx.buf_secSpeed[BUFFER_SIZE] and lctx.index_sec
        if (lctx.index_sec >= me->time_window) { // once 10s is reached, subtract -10s from the sum again
            me->avg_s_sum = me->avg_s_sum - lctx.buf_secSpeed[(lctx.index_sec - me->time_window) % BUFFER_SIZE];
            window_reached = true;  // only if the time window is reached, we can calculate the speed
            me->speed.speed = me->avg_s_sum / me->time_window;  // in the seconds array is the average of gspeed !!
        }
    }
    if(!window_reached && me->speed.speed > 0) {
        me->speed.speed = 0;  // if the time window is not reached, set the speed to 0
    }
    return window_reached;  // return true if the time window is reached, so we can calculate the speed
}

float update_speed_by_time(gps_context_t *context, struct gps_speed_by_time_s *me) {
    // printf("[%s]\n", __func__);
    assert(context);
    ubx_config_t *ubx = context->ubx_device;
    uint8_t sample_rate = ubx->rtc_conf->output_rate;
    // uint32_t actual_run = context->run_count;
    uint32_t time_window_delta = me->time_window * sample_rate;
    if(store_avg_speed_by_time(me, time_window_delta, sample_rate))
        store_speed_by_time_data(context, me);  // store the run data if the speed is higher than the previous run
    if ((context->run_count != me->speed.nr_prev_run) && (me->speed.runs[0].nr_run == me->speed.nr_prev_run)) {  // sorting only if new max during this run !!!
        store_and_reset_time_data_after_run(context, me);  // sort the runs and update the display speed}
    }
    me->speed.nr_prev_run = context->run_count;
    record_last_run(&me->speed, context->run_count); 
    return me->speed.max_speed;  // anders compiler waarschuwing control reaches end of non-void function [-Werror=return-type]
}

struct gps_speed_alfa_s *init_alfa_speed(struct gps_speed_alfa_s *me, uint16_t alfa_radius) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    memset(me, 0, sizeof(struct gps_speed_alfa_s));
    me->set_alfa_dist = alfa_radius;
    return me;
}

#if defined(CONFIG_LOGGER_BUTTON_GPIO_1_ACTIVE)
void reset_alfa_stats(struct gps_speed_alfa_s *me) {
    reset_runs_avg(me->speed.runs);
}
#endif

#if defined(GPS_STATS) && defined(GPS_TRACE_MSG_SPEED_ALPHA)
static esp_err_t gps_speed_by_alpha_printf(const struct gps_speed_alfa_s *me) {
    uint8_t i, j=NUM_OF_SPD_ARRAY_SIZE;
    printf("=== speed_by_alfa: {\n");
    printf("set_alfa_dist: %hu, ", me->set_alfa_dist);
    printf("straight_dist_square: %.03f\n", me->straight_dist_square);
    gps_speed_printf(&me->speed);
    printf("real_distance: ");
    for (i = 0; i < j; i++) printf("%ld ", me->real_distance[i]);
    printf("\n");
    printf("alfa_distance: ");
    for (i = 0; i < j; i++) printf("%lu ", me->alfa_distance[i]);
    printf("\n");
    printf("message_nr: ");
    for (i = 0; i < j; i++) printf("%lu ", me->message_nr[i]);
    printf("\n");
    printf("} === \n");
    return ESP_OK;
}
#endif

#define USE_HAVERSINE
#if defined(USE_HAVERSINE)
static inline double haversine(gps_point_t * p1, gps_point_t * p2) {
    float dlat = DEG2RAD * (p2->latitude -  p1->latitude);
    float dlon = DEG2RAD * (p2->longitude - p1->longitude);
    double a = sin(dlat/2) * sin(dlat/2) +
              cos(DEG2RAD *  p1->latitude) * cos(DEG2RAD *  p2->latitude) *
              sin(dlon/2) * sin(dlon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return EARTH_RADIUS_M * c;
}
#else
static inline double straight_dist_square(gps_point_t * p1, gps_point_t * p2) {
    // now calculate the absolute distance alfa_speed::alfa_update(GPS_speed M)
    // between the starting point and the end point of the 250m distance,
    // if < 50m this is an alfa !!! note, this is calculated in meters,
    // therefore alfa_circle also in m !! was (M.m_index-1), should be (M.m_index+1)
    float dlat = p1->latitude - p2->latitude;
    float px = cos(DEG2RAD * p1->latitude) * (p1->longitude - p2->longitude);
    return DEG_TO_METERS(POW_2(dlat) + POW_2(px));
}
#endif

static inline void store_alfa_data(gps_context_t *context, struct gps_speed_alfa_s *me, uint32_t dist) {
    // printf("[%s]\n", __func__);
    if (store_max_speed(&me->speed, context->run_count)) {
        me->message_nr[0] = context->ubx_device->ubx_msg.count_nav_pvt;
        me->real_distance[0] = (int32_t)me->straight_dist_square;
        me->alfa_distance[0] = dist;
    }
    update_display_speeds(&me->speed, &context->record);
}

static inline void store_and_reset_alfa_data_after_run(struct gps_speed_alfa_s *me) { 
#if (C_LOG_LEVEL < 2)
    ILOG(TAG, "[%s]", __func__);
#endif
    // sort_run_alfa(me->speed.runs, me->real_distance, me->message_nr, me->alfa_distance, 10);
    sort_runs(
        me->speed.runs,
        (void *[]){me->real_distance, me->message_nr, me->alfa_distance},
        (size_t[]){sizeof(me->real_distance[0]), sizeof(me->message_nr[0]), sizeof(me->alfa_distance[0])},
        3, NUM_OF_SPD_ARRAY_SIZE
    );
    reset_gps_last_speed(&me->speed);  // reset the speed for the next run
}

// Attention, here the distance traveled must be less than 500 m! Therefore, 
// an extra variable, m_speed_alfa, is provided in GPS_speed!!!
float update_alfa_speed(gps_context_t *context, struct gps_speed_alfa_s *me, struct gps_speed_by_dist_s *M) {
    // printf("[%s]\n", __func__);
    assert(context);
#if defined(USE_HAVERSINE)
    me->straight_dist_square = haversine(
        &lctx.buf_alfa[al_buf_index(lctx.index_GPS)], &lctx.buf_alfa[al_buf_index(M->m_index + 1)]
    );
#else
    me->straight_dist_square = straight_dist_square(&lctx.buf_alfa[al_buf_index(lctx.index_GPS)], &lctx.buf_alfa[al_buf_index(M->m_index + 1)]);
#endif
// #if (C_LOG_LEVEL < 2)
//     WLOG(TAG, "A%hu straight_dist: %.02f of 50, straight_dist_square: %.02f of %d", me->set_alfa_dist, straight_dist, me->straight_dist_square, POW_2(me->set_alfa_dist));
// #endif
#if defined(USE_HAVERSINE)
    if (me->straight_dist_square < 50.0f) {
#else
    if (me->straight_dist_square < POW_2(me->set_alfa_dist)) {
#endif
        me->speed.speed = M->speed.speed_alfa;
        if (M->m_sample >= BUFFER_ALFA) me->speed.speed = 0;  // avoid overflow at low speeds
        store_alfa_data(context, me, M->distance_alfa / context->ubx_device->rtc_conf->output_rate);
    }
    // if((alfa_speed_max>0.0f)&(straight_dist_square>(alfa_circle_square*1.4))){ //alfa max only resets to 0 if 500 m after the jibe, straight distance after the jibe
    if (context->run_count != me->speed.nr_prev_run) {
        store_and_reset_alfa_data_after_run(me);
    }
    me->speed.nr_prev_run = context->run_count;
    record_last_run(&me->speed, context->run_count); 
    return me->speed.max_speed;
}

static inline float unwrap_heading(float actual_heading, float *old_heading, float *delta_heading) {
    // printf("[%s]\n", __func__);
    if ((actual_heading - *old_heading) >  300.0f) *delta_heading -= 360.0f;
    if ((actual_heading - *old_heading) < -300.0f) *delta_heading += 360.0f;
    *old_heading = actual_heading;
    return actual_heading + *delta_heading;
}

static inline void detect_run_start_end(float S2_speed) {
    /// detection stand still, more then 2s with velocity < 1m/s 
    if (S2_speed > SPEED_DETECTION_MIN) lctx.velocity_5 = 1;  // 2s with speed over 4m/s
    if ((S2_speed < STANDSTILL_DETECTION_MAX) && (lctx.velocity_5)) lctx.velocity_0 = 1; // stopping detected, 2s with speed < 1 m/s
    else lctx.velocity_0 = 0;
    /// New run detected due to standstill
    if (lctx.velocity_0) {
        lctx.velocity_5 = 0;
        lctx.delay_count_before_run = 0;
    }
}

/* Calculation of the average heading over the last 10 seconds 
************************************************************************/
static inline bool is_straight_course(float heading_diff) {
    return (heading_diff < STRAIGHT_COURSE_MAX_DEV);
}

static inline bool detect_jibe(float heading_diff) {
    return heading_diff > JIBE_COURSE_DEVIATION_MIN && lctx.straight_course;
}

uint32_t new_run_detection(gps_context_t *context, float actual_heading, float S2_speed) {
    // printf("[%s]\n", __func__);
    assert(context);
    ubx_config_t *ubx = context->ubx_device;
    uint8_t sample_rate = ubx->rtc_conf->output_rate;

    lctx.heading = unwrap_heading(actual_heading, &lctx.old_heading, &lctx.delta_heading);    
    /// detect heading change over 15s is more than 40°, a new run is started
    uint16_t mean_heading_delta_time = MEAN_HEADING_TIME * sample_rate;
    lctx.heading_mean = lctx.heading_mean * (mean_heading_delta_time - 1) / (mean_heading_delta_time) + lctx.heading / (mean_heading_delta_time);
    
    /// detection stand still, more then 2s with velocity < 1m/s 
    detect_run_start_end(S2_speed);

    /// New run detected due to heading change
    float heading_diff = fabs(lctx.heading_mean - lctx.heading);
    if(is_straight_course(heading_diff) && lctx.velocity_5) {
        // printf("Straight course detected, heading_diff: %.02f\n", heading_diff);
        lctx.straight_course = 1;  // straight course detected, set straight_course to true
    }
    if(detect_jibe(heading_diff)) {
#if (C_LOG_LEVEL < 2)
        printf("Jibe detected, heading_diff: %.02f\n", heading_diff);
#endif
        lctx.straight_course = 0;  // jibe detected, straight course is false
        lctx.delay_count_before_run = 0;
        lctx.alfa_count++;  // jibe detection for alfa_indicator ....
    }
    lctx.delay_count_before_run++;
    if (lctx.delay_count_before_run == (TIME_DELAY_NEW_RUN * sample_rate)) {
        lctx.run_count = ++context->run_count;
// #if (C_LOG_LEVEL < 2)
//         WLOG(TAG, "=== Run finished, count changed to %hu ===", context->run_count);
// #endif
    }
// #if (C_LOG_LEVEL < 2)
//     printf("delay_count_before_run: %lu, start_cond: %u ...\n", lctx.delay_count_before_run, (TIME_DELAY_NEW_RUN * sample_rate));
//     printf("run_count: %hu, lctx.alfa_count: %hu, lctx.straight_course: %d\n", context->run_count, lctx.alfa_count, lctx.straight_course);
// #endif
    return context->run_count;
}

// #define USE_LONG_POINTS  // define this to use the long points for jibe detection, otherwise the short points are used
static inline void update_jibe_reference_points(int32_t m_1, int32_t m_2, gps_point_t * buf, gps_point_t *p1, gps_point_t *p2) {
    // printf("[%s]\n", __func__);
    // this is the point at -250 m from the current position (speed extrapolation from -500m)
    p1->latitude  = buf[m_1].latitude;
    p1->longitude = buf[m_1].longitude;
    // this is the point at -100 m from the current position (speed extrapolation from -250m)
    p2->latitude  = buf[m_2].latitude;
    p2->longitude = buf[m_2].longitude;
}

/*
/// previous code for dist_point_line, not used anymore, but kept for reference
float dist_point_line(float long_act, float lat_act, float long_1, float lat_1, float long_2, float lat_2) {
    float corr_lat=111120;
    float corr_long=111120*cos(DEG2RAD*buf_alfa[lctx.index_GPS%BUFFER_ALFA]);
    lambda_T=(lctx.P2_lat-lctx.P1_lat)*(P_lat-lctx.P1_lat)*corr_lat*corr_lat+(lctx.P2_long-lctx.P1_long)*(P_long-lctx.P1_long)*corr_long*corr_long;
    lambda_N=pow((lctx.P2_lat-lctx.P1_lat)*corr_lat,2)+pow((lctx.P2_long-lctx.P1_long)*corr_long,2);
    lambda=lambda_T/lambda_N;
    alfa_dist=sqrt(
        pow((P_lat-lctx.P1_lat-lambda*(lctx.P2_lat-lctx.P1_lat))*corr_lat,2)+
        pow((P_long-lctx.P1_long-lambda*(lctx.P2_long-lctx.P1_long))*corr_long,2));
}
*/

/// Calculates distance from point with corr lat/long to line which passes points lat_1/long_1 and lat_2/long_2
float dist_point_line(gps_point_t * act,  gps_point_t * p1, gps_point_t * p2) {
    // printf("[%s]\n", __func__);
    float corr_lat = METERS_PER_LATITUDE_DEGREE;
    float corr_long = corr_lat * cos(DEG2RAD * act->latitude);  // meters per longitude degree, this is a function of latitude !
    float dlat = p2->latitude - p1->latitude;
    float dlong = p2->longitude - p1->longitude;
    float dlat_act = act->latitude - p1->latitude;
    float dlong_act = act->longitude - p1->longitude;
    float lambda_T = ((dlat * dlat_act) * POW_2(corr_lat)) + ((dlong * dlong_act) * POW_2(corr_long));
    float lambda_N = POW_2(dlat * corr_lat) + POW_2(dlong * corr_long);
    // float lambda = 0.0f;
    // if (lambda_N != 0.0f) {
    float lambda = lambda_T / lambda_N;
    // } else {
    //     // Avoid division by zero; return a large value or 0 as appropriate
    //     return 0.0f;
    // }
    float alfa_distance = sqrt(POW_2((dlat_act - lambda * dlat) * corr_lat) + POW_2((dlong_act - lambda * dlong) * corr_long));
// #if (C_LOG_LEVEL < 2)
//     printf("lambda_T: %.2f, lambda_N: %.2f, lambda: %.2f, adist: %.02f\n", lambda_T, lambda_N, lambda, alfa_distance);
// #endif
    return alfa_distance;
}

/* Here the current "alfa distance" is calculated based on 2 points for the jibe: 
 P1 = 250m and P2 = 100m for the jibe. These points determine an imaginary line, 
 the perpendicular distance to the current position must be less than 50m/s
 when point P1 is passed.
 */
float alfa_indicator(gps_context_t *context, float actual_heading) {
    // printf("[%s]\n", __func__);
    assert(context);
    ubx_config_t *ubx = context->ubx_device;
    // Update jibe reference points if needed
    if (lctx.alfa_count != lctx.old_alfa_count) {
        // the distance traveled since the jibe detection 10*100.000/10.000=100 samples ?
        context->Ublox.alfa_distance = 0;
        update_jibe_reference_points(
// #define USE_LONG_POINTS 1
#if defined(USE_LONG_POINTS)
            al_buf_index(context->M500.m_index), 
            al_buf_index(context->M250.m_index), 
#else
            al_buf_index(context->M250.m_index), 
            al_buf_index(context->M100.m_index), 
#endif
            lctx.buf_alfa, 
            &lctx.alfa_p1, 
            &lctx.alfa_p2
        );
        lctx.old_alfa_count = lctx.alfa_count;
    }
    // Current position
    int32_t idx_cur = al_buf_index(lctx.index_GPS);
    gps_point_t cur = { lctx.buf_alfa[idx_cur].latitude, lctx.buf_alfa[idx_cur].longitude};

    // Position 2 seconds ago
    int32_t idx_prev = al_buf_index(lctx.index_GPS - (2 * ubx->rtc_conf->output_rate));
    gps_point_t prev = {lctx.buf_alfa[idx_prev].latitude, lctx.buf_alfa[idx_prev].longitude};

    // -2s position lat
    // was previously sin, calculate extra point for heading, calculate using distance/degree length!!
    // cos(ubxMessage.navPvt.heading*PI/180.0f/100000.0f) * 111120 + P_lat;
    // -2s position long
    // calculate using distance/degree length!!
    // sin(ubxMessage.navPvt.heading*PI/180.0f/100000.0f) * 111120 * cos(DEG2RAD*P_lat) + P_long;
    // m500 cur pos dist from line( current pos, pos 2s ago ), this is the point at 225-275 m from the current position
    context->alfa_exit = dist_point_line(&lctx.alfa_p1, &cur, &prev);
    // pos 2s ago dist from line( current pos, m250 pos )
    context->alfa_window = dist_point_line(&cur, &lctx.alfa_p1, &lctx.alfa_p2);
// #if (C_LOG_LEVEL < 2)
//     ILOG(TAG, "[%s] alfa_exit: %.02f, alfa_window: %.2f", __func__, context->alfa_exit, context->alfa_window);
// #endif
    return context->alfa_window;  // current perpendicular distance relative to the line P2-P1, may be max 50m for a valid alfa !!
}


/*Heading tov reference***********************************************************************************
// Calculate reference heading (in degrees) from P2 to P1.
// This is tested and works correctly, but is +180° compared to the actual direction.
ref_heading = atan2(((P1_long - P2_long) * corr_long), ((P1_lat - P2_lat) * corr_lat)) * 180 / PI;
 if(ref_heading<0)ref_heading=ref_heading+360; // atan2 returns a value between -PI and +PI radians.
 delta_heading=(int)(actual_heading-ref_heading*180/PI)%360; // due to P1-P2, this is the opposite direction from travelling ! if(delta_heading>180)
 delta_heading=delta_heading-360; if(delta_heading<-180)
 delta_heading=delta_heading+360;
 */

#endif // UBLOX_ENABLED
