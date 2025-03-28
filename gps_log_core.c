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

#define NR_OF_BAR 42 // aantal bar in de bar_graph
// #define GPS_TRACE_MSG_SAT 1

// static const char *TAG = "GPS_data";
typedef struct gps_p_context_s {
    int32_t buf_gSpeed[BUFFER_SIZE];   // speed buffer counted by gps rate
    int16_t buf_secSpeed[BUFFER_SIZE]; // speed buffer counted by sec

    int32_t index_GPS;                 // by rate counted speed buffer pos (-1), start at 0 on first pass !!
    int32_t index_sec;                 // by sec counted speed buffer pos (-1), start at 0 on first pass !!

    int32_t sec_gSpeed;      // for avg speed per second mm/s
    uint32_t delay_count_before_run;    // count loops to wait before incerment the run count
    uint32_t run_count;      // counter for runs
    uint32_t old_run_count;  // previous run counter
    uint32_t alfa_count;     // counter for alfa
    uint32_t old_alfa_count; // previous alfa counter

    bool velocity_0;      // min gemiddelde over 2 s < 1m/s
    bool velocity_5;      // min gemiddelde over 2 s = 1m/s
    bool straight_course; // straight course or not
    bool dynamic_state;   // ubx setting dynamic model state: sea or portable here !!

    double delta_heading;
    double ref_heading;
    
    float buf_lat[BUFFER_ALFA];  // lat buffer counted by alfa
    float buf_long[BUFFER_ALFA]; // long buffer counted by alfa

    float delta_dist;
    float heading;     // heding for alfa // = 0;
    float old_heading; // old heading for alfa // = 0, 
    float P1_lat;
    float P1_long;
    float P2_lat;
    float P2_long;
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
    .ref_heading = 0, \
    .buf_lat = {0}, \
    .buf_long = {0}, \
    .delta_dist = 0, \
    .heading = 0, \
    .old_heading = 0, \
    .P1_lat = 0, \
    .P1_long = 0, \
    .P2_lat = 0, \
    .P2_long = 0, \
    .xMutex = NULL \
}
gps_p_context_t lctx = GPS_P_CONTEXT_INIT;

#define CONFIG_GPS_LOG_ENABLE_DEBUG 0
#if defined(CONFIG_GPS_LOG_LEVEL_TRACE)
esp_err_t gps_p_context_printf(const gps_p_context_t *me) {
    printf("gps_p_context:{\n");
    printf("dynamic_state: %d\n", me->dynamic_state);
    printf("sec_gSpeed: %ld\n", me->sec_gSpeed);
    printf("delay_count_before_run: %lu\n", me->delay_count_before_run);
    printf("run_count: %lu\n", me->run_count);
    printf("old_run_count: %lu\n", me->old_run_count);
    printf("alfa_count: %lu\n", me->alfa_count);
    printf("old_alfa_count: %lu\n", me->old_alfa_count);
    printf("velocity_0: %d\n", me->velocity_0);
    printf("velocity_5: %d\n", me->velocity_5);
    printf("straight_course: %d\n", me->straight_course);
    printf("old_heading: %.03f\n", me->old_heading);
    printf("heading: %.03f\n", me->heading);
    printf("delta_heading: %.03f\n", me->delta_heading);
    printf("ref_heading: %.03f\n", me->ref_heading);
    printf("delta_dist: %.03f\n", me->delta_dist);
    printf("P1_lat: %.03f\n", me->P1_lat);
    printf("P1_long: %.03f\n", me->P1_long);
    printf("P2_lat: %.03f\n", me->P2_lat);
    printf("P2_long: %.03f\n", me->P2_long);
    printf("index_GPS: %ld\n", me->index_GPS);
    printf("index_sec: %ld\n", me->index_sec);
    printf("_lat: ");
    uint8_t i, j=me->index_GPS%BUFFER_SIZE;
    if(j<20) j=20;
    for (i = j-20; i < j; i++)
        printf(" %.03f", me->buf_lat[i]);
    printf("\n");
    printf("_long: ");
    for (i = j-20; i < j; i++)
        printf(" %.03f", me->buf_long[i]);
    printf("\n");
    printf("_gSpeed: ");
    for (i = j-20; i < j; i++)
        printf(" %ld", me->buf_gSpeed[i]);
    printf("\n");
    j=me->index_sec%BUFFER_SIZE;
    if(j<20) j=20;
    printf("\nbuf_secSpeed: ");
    for (i = j-20; i < j; i++)
        printf(" %hu", me->buf_secSpeed[i]);
    printf("\n");
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
    ILOG(TAG, "[%s]", __func__);
    assert(ctx);
    init_gps_data(&ctx->Ublox);  // create an object storing GPS_data !
    init_gps_sat_info(&ctx->Ublox_Sat);  // create an object storing GPS_SAT info !
    init_gps_speed(&ctx->M100, 100);
    init_gps_speed(&ctx->M250, 250);
    init_gps_speed(&ctx->M500, 500);
    init_gps_speed(&ctx->M1852, 1852);
    //init_gps_speed(&ctx->M1852, 1852);
    init_gps_time(&ctx->S2, 2);
    init_gps_time(&ctx->s2, 2);
    init_gps_time(&ctx->S10, 10);
    init_gps_time(&ctx->s10, 10);  // for  stats GPIO_12 screens, reset possible !!
    init_gps_time(&ctx->S1800, 1800);
    init_gps_time(&ctx->S3600, 3600);
    init_alfa_speed(&ctx->A250, 250);
    init_alfa_speed(&ctx->A500, 500);
    init_alfa_speed(&ctx->a500, 500);
    if (ctx->ubx_device == NULL)
        ctx->ubx_device = ubx_config_new();
    if (ctx->log_config == NULL)
        ctx->log_config = &log_config;
    if(lctx.xMutex == NULL)
        lctx.xMutex = xSemaphoreCreateMutex();
    gps_user_cfg_init();
    ctx->Gps_fields_OK = 1;
}

void deinit_gps_context_fields(gps_context_t *ctx) {
    ILOG(TAG, "[%s]", __func__);
    assert(ctx);
    if (ctx->ubx_device != NULL){
        ubx_config_delete(ctx->ubx_device);
        ctx->ubx_device = NULL;
    }
    if (ctx->log_config != NULL){
        // log_config_delete(ctx->log_config);
        ctx->log_config = NULL;
    }
    if(lctx.xMutex != NULL){
        vSemaphoreDelete(lctx.xMutex);
        lctx.xMutex = NULL;
    }
    gps_user_cfg_deinit();
    ctx->Gps_fields_OK = 0;
}

#if defined(CONFIG_GPS_LOG_LEVEL_TRACE)
esp_err_t gps_data_printf(const struct gps_data_s *me) {
    printf("gps_data:{ ");
    printf("id: %ld(%ld), ", lctx.index_GPS, lctx.index_GPS % BUFFER_SIZE);
    printf("run_start_time: %lu, ", me->run_start_time);
    printf("run_distance: %.03f, ", me->run_distance);
    printf("alfa_distance: %.03f, ", me->alfa_distance);
    printf("total_distance: %.03f ", me->total_distance);
    printf("}\n");
    return ESP_OK;
}
#endif

#if (CONFIG_GPS_LOG_LEVEL < 3) 
#include "strbf.h"
void gps_log_nav_mode_change(gps_context_t *context, uint8_t changed) {
    ILOG(TAG, "[%s] %hhu", __func__, changed);
    ubx_config_t *ubx = context->ubx_device;
    const char * bstr = " nav_mode changed ";
    if(changed) {
        strbf_t strbf;
        char buf[96] = {0};
        strbf_inits(&strbf, buf, sizeof(buf));
        strbf_putul(&strbf, ((get_millis() - context->start_logging_millis) / 1000));
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
        strbf_putf(&strbf, context->S2.avg_s);
        strbf_puts(&strbf, "mm/s\n");
        *strbf.cur = 0;
        WRITETXT(strbf.start, strbf.cur-strbf.start);
    }
}
#endif

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
    if(ubx->rtc_conf->nav_mode_auto) {
        uint8_t changed = 0;
        if(lctx.dynamic_state == 0) {
            /// switch to dynamic_model "portable", as "pedestrian" only works with speed < 30 m/s (108kmh)
            /// and "sea" only works with speed < 25 m/s (90kmh)
            if (((context->S2.avg_s > 28000 && ubx->rtc_conf->nav_mode == UBX_MODE_PEDESTRIAN) /// change over 28ms 100kmh
                || (context->S2.avg_s > 23500 && ubx->rtc_conf->nav_mode == UBX_MODE_SEA))) { /// change over 23.5ms 85kmh
                lctx.dynamic_state = 1;
                ubx_set_nav_mode(ubx, UBX_MODE_PORTABLE);
            }
            else if (context->S2.avg_s < 29000 && ubx->rtc_conf->nav_mode == UBX_MODE_PORTABLE) { /// change under 28ms 100kmh
                /// switch to dynamic_model "pedestrian" below 30ms as "portable" is less accurate
                lctx.dynamic_state = 1;
                ubx_set_nav_mode(ubx, UBX_MODE_PEDESTRIAN);
            }
            if(lctx.dynamic_state==1) changed=1;
        }
        else if(lctx.dynamic_state == 1) {
            /// switch back to "pedestrian" below 27ms as "pedestrian" is more accurate
            if(context->S2.avg_s < 24000 && (ubx->rtc_conf->nav_mode == UBX_MODE_PEDESTRIAN)) { /// change below 24ms 86kmh
                lctx.dynamic_state = 0;
                ubx_set_nav_mode(ubx, UBX_MODE_PEDESTRIAN);
            }
            else if (context->S2.avg_s > 28000 && ubx->rtc_conf->nav_mode == UBX_MODE_PORTABLE) { /// change under 28ms 100kmh
                lctx.dynamic_state = 0;
                ubx_set_nav_mode(ubx, UBX_MODE_PORTABLE);

            }
            if(lctx.dynamic_state==0) changed=1;
        }
        if(changed) {
            esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_GPS_NAV_MODE_CHANGED, &lctx.dynamic_state, sizeof(uint8_t*), portMAX_DELAY);
        }
    }

    if(xSemaphoreTake(lctx.xMutex, 0) != pdTRUE)
        return ESP_FAIL;
    
    lctx.index_GPS++;  // always increment index after updating all instances
    lctx.buf_gSpeed[lctx.index_GPS % BUFFER_SIZE] = gSpeed;  // always store gSpeed in array range !
    lctx.buf_lat[lctx.index_GPS % BUFFER_ALFA] = latitude;
    lctx.buf_long[lctx.index_GPS % BUFFER_ALFA] = longitude;
    // only add distance if reception is good, be careful sometimes sAcc<2
    // !!!****************************************************
    if ((ubxMessage->navPvt.numSV >= FILTER_MIN_SATS) && ((ubxMessage->navPvt.sAcc / 1000.0f) < FILTER_MAX_sACC)) {
        lctx.delta_dist = gSpeed / sample_rate;  // convert speed to distance !!!
        me->total_distance += lctx.delta_dist;
        me->run_distance += lctx.delta_dist;
        me->alfa_distance += lctx.delta_dist;
    }
    // Store groundSpeed per second
    // !!******************************************************
    lctx.sec_gSpeed += gSpeed;  // store at seconds interval for 30 min / 60 min average speed
    if (lctx.index_GPS % sample_rate == 0) {  // modulus of index%sample rate
        lctx.index_sec++;   // also lctx.index_sec may only be updated after instance update
        lctx.buf_secSpeed[lctx.index_sec % BUFFER_SIZE] = lctx.sec_gSpeed / sample_rate; // otherwise overflow because lctx.buf_secSpeed[] is only up to 65535 (uint16) !!!!
        lctx.sec_gSpeed = 0; // reset sec_gSpeed for calc next second average speed
    }
    xSemaphoreGive(lctx.xMutex);
#if defined(CONFIG_GPS_LOG_LEVEL_TRACE)
    if (lctx.index_GPS % sample_rate == 0 && gSpeed > STANDSTILL_DETECTION_MAX) {
        gps_data_printf(me);
        gps_p_context_printf(&lctx);
    }
#endif
    //printf("Speed upadated gspeed: %"PRIu32" avgSpeed:%"PRIu32"\n", gSpeed, lctx.avg_gSpeed);
    return ESP_OK;
}

// constructor for GPS_data
struct gps_data_s *init_gps_data(struct gps_data_s *me) {
    ILOG(TAG, "[%s]", __func__);
    memset(me, 0, sizeof(struct gps_data_s));
    lctx.index_GPS = 0;
    return me;
}

// esp_err_t gps_data_serialize_json(const struct gps_data_s *data, cJSON *root) {
//     cJSON *gps_data = cJSON_CreateObject();
//     if (gps_data == NULL) {
//         return ESP_ERR_NO_MEM;
//     }
//     cJSON_AddNumberToObject(gps_data, "delta_dist", data->delta_dist);
//     cJSON_AddNumberToObject(gps_data, "total_distance", data->total_distance);
//     cJSON_AddNumberToObject(gps_data, "run_distance", data->run_distance);
//     cJSON_AddNumberToObject(gps_data, "alfa_distance", data->alfa_distance);
//     cJSON_AddItemToObject(root, "gps_data", gps_data);
//     return ESP_OK;
// }

// constructor for SAT_info
struct GPS_SAT_info *init_gps_sat_info(struct GPS_SAT_info *me) {
    ILOG(TAG, "[%s]", __func__);
    memset(me, 0, sizeof(struct GPS_SAT_info));
    me->index_SAT_info = 0;
    return me;
}
#if defined(CONFIG_GPS_LOG_LEVEL_TRACE)  && defined(GPS_TRACE_MSG_SAT)
esp_err_t gps_sat_info_printf(const struct GPS_SAT_info *me) {
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
        me->sat_info.Mean_cno[me->index_SAT_info % NAV_SAT_BUFFER] = me->mean_cno;
        me->sat_info.Max_cno[me->index_SAT_info % NAV_SAT_BUFFER] = me->max_cno;
        me->sat_info.Min_cno[me->index_SAT_info % NAV_SAT_BUFFER] = me->min_cno;
        me->sat_info.numSV[me->index_SAT_info % NAV_SAT_BUFFER] = me->nr_sats;
        me->mean_cno = 0;
        me->min_cno = 0;
        me->max_cno = 0;
        me->nr_sats = 0;
        if (me->index_SAT_info > NAV_SAT_BUFFER) {
            for (uint8_t i = 0; i < NAV_SAT_BUFFER; i++) {
                me->mean_cno = me->mean_cno + me->sat_info.Mean_cno[(me->index_SAT_info - NAV_SAT_BUFFER + i) % NAV_SAT_BUFFER];
                me->max_cno = me->max_cno + me->sat_info.Max_cno[(me->index_SAT_info - NAV_SAT_BUFFER + i) % NAV_SAT_BUFFER];
                me->min_cno = me->min_cno + me->sat_info.Min_cno[(me->index_SAT_info - NAV_SAT_BUFFER + i) % NAV_SAT_BUFFER];
                me->nr_sats = me->nr_sats + me->sat_info.numSV[(me->index_SAT_info - NAV_SAT_BUFFER + i) % NAV_SAT_BUFFER];
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
#if defined(CONFIG_GPS_LOG_LEVEL_TRACE) && defined(GPS_TRACE_MSG_SAT)
        gps_sat_info_printf(me);
#endif
    }
};

void sort_display(double a[], uint8_t size) {
    for (uint8_t i = 0; i < (size - 1); i++) {
        for (uint8_t o = 0; o < (size - (i + 1)); o++) {
            if (a[o] > a[o + 1]) {
                double t = a[o];
                a[o] = a[o + 1];
                a[o + 1] = t;
            }
        }
    }
}

void sort_run(double a[], uint8_t hour[], uint8_t minute[], uint8_t seconde[], uint16_t mean_cno[], uint8_t max_cno[], uint8_t min_cno[], uint8_t nrSats[], uint32_t runs[], uint8_t size) {
    for (uint8_t i = 0; i < (size - 1); i++) {
        for (uint8_t o = 0; o < (size - (i + 1)); o++) {
            if (a[o] > a[o + 1]) {
                double t = a[o];
                uint8_t b = hour[o];
                uint8_t c = minute[o];
                uint8_t d = seconde[o];
                uint32_t e = runs[o];
                uint16_t f = mean_cno[o];
                uint8_t g = max_cno[o];
                uint8_t h = min_cno[o];
                uint8_t j = nrSats[o];
                a[o] = a[o + 1];
                hour[o] = hour[o + 1];
                minute[o] = minute[o + 1];
                seconde[o] = seconde[o + 1];
                runs[o] = runs[o + 1];
                mean_cno[o] = mean_cno[o + 1];
                max_cno[o] = max_cno[o + 1];
                min_cno[o] = min_cno[o + 1];
                nrSats[o] = nrSats[o + 1];
                a[o + 1] = t;
                hour[o + 1] = b;
                minute[o + 1] = c;
                seconde[o + 1] = d;
                runs[o + 1] = e;
                mean_cno[o + 1] = f;
                max_cno[o + 1] = g;
                min_cno[o + 1] = h;
                nrSats[o + 1] = j;
            }
        }
    }
}
void sort_run_alfa(double a[], int32_t dis[], uint32_t message[], uint8_t hour[], uint8_t minute[], uint8_t seconde[], uint32_t runs[], uint32_t samples[], uint8_t size) {
    for (uint8_t i = 0; i < (size - 1); i++) {
        for (uint8_t o = 0; o < (size - (i + 1)); o++) {
            if (a[o] > a[o + 1]) {
                double t = a[o];
                uint32_t v = dis[o];
                uint32_t x = message[o];
                uint8_t b = hour[o];
                uint8_t c = minute[o];
                uint8_t d = seconde[o];
                uint32_t e = runs[o];
                uint32_t f = samples[o];
                a[o] = a[o + 1];
                dis[o] = dis[o + 1];
                message[o] = message[o + 1];
                hour[o] = hour[o + 1];
                minute[o] = minute[o + 1];
                seconde[o] = seconde[o + 1];
                runs[o] = runs[o + 1];
                samples[o] = samples[o + 1];
                a[o + 1] = t;
                dis[o + 1] = v;
                message[o + 1] = x;
                hour[o + 1] = b;
                minute[o + 1] = c;
                seconde[o + 1] = d;
                runs[o + 1] = e;
                samples[o + 1] = f;
            }
        }
    }
}

/*Instantie om gemiddelde snelheid over een bepaalde afstand te bepalen, bij een
 * nieuwe run opslaan hoogste snelheid van de vorige run*****************/
struct gps_speed_by_dist_s *init_gps_speed(struct gps_speed_by_dist_s *me, uint16_t afstand) {
    ILOG(TAG, "[%s]", __func__);
    memset(me, 0, sizeof(struct gps_speed_by_dist_s));
    me->m_set_distance = afstand;
    return me;
}

#if defined(CONFIG_GPS_LOG_LEVEL_TRACE)  && defined(GPS_TRACE_MSG_SPEED)
// esp_err_t gps_speed_serialize_json(struct gps_speed_by_dist_s *me, cJSON * root) {
//     cJSON *gps_speed = cJSON_CreateObject();
//     if (gps_speed == NULL) {
//         return ESP_ERR_NO_MEM;
//     }
//     cJSON_AddNumberToObject(gps_speed, "m_set_distance", me->m_set_distance);
//     cJSON_AddNumberToObject(gps_speed, "m_distance", me->m_distance);
//     cJSON_AddNumberToObject(gps_speed, "m_distance_alfa", me->m_distance_alfa);
//     cJSON_AddNumberToObject(gps_speed, "m_sample", me->m_sample);
//     cJSON_AddNumberToObject(gps_speed, "m_speed", me->m_speed);
//     cJSON_AddNumberToObject(gps_speed, "m_speed_alfa", me->m_speed_alfa);
//     cJSON_AddNumberToObject(gps_speed, "m_max_speed", me->m_max_speed);
//     cJSON_AddNumberToObject(gps_speed, "display_max_speed", me->display_max_speed);
//     cJSON_AddNumberToObject(gps_speed, "old_run", me->old_run);
//     cJSON_AddNumberToObject(gps_speed, "m_index", me->m_index);
//     cJSON_AddNumberToObject(gps_speed, "m_Set_Distance", me->m_Set_Distance);
//     cJSON * t = cJSON_CreateArray();
//     cJSON_AddItemToObject(gps_speed, "time_hour", t);
//     for (uint8_t i = 0; i < 10; i++)
//         cJSON_AddItemToArray(t, cJSON_CreateNumber(me->time_hour[i]));
//     t = cJSON_CreateArray();
//     cJSON_AddItemToObject(gps_speed, "time_min", t);
//     for (uint8_t i = 0; i < 10; i++)
//         cJSON_AddItemToArray(t, cJSON_CreateNumber(me->time_min[i]));
//     t = cJSON_CreateArray();
//     cJSON_AddItemToObject(gps_speed, "time_sec", t);
//     for (uint8_t i = 0; i < 10; i++)
//         cJSON_AddItemToArray(t, cJSON_CreateNumber(me->time_sec[i]));
//     t = cJSON_CreateArray();
//     cJSON_AddItemToObject(gps_speed, "this_run", t);
//     for (uint8_t i = 0; i < 10; i++)
//         cJSON_AddItemToArray(t, cJSON_CreateNumber(me->this_run[i]));
//     t = cJSON_CreateArray();
//     cJSON_AddItemToObject(gps_speed, "avg_speed", t);
//     for (uint8_t i = 0; i < 10; i++)
//         cJSON_AddItemToArray(t, cJSON_CreateNumber(me->avg_speed[i]));
//     t = cJSON_CreateArray();
//     cJSON_AddItemToObject(gps_speed, "m_Distance", t);
//     for (uint8_t i = 0; i < 10; i++)
//         cJSON_AddItemToArray(t, cJSON_CreateNumber(me->m_Distance[i]));
//     t = cJSON_CreateArray();
//     cJSON_AddItemToObject(gps_speed, "nr_samples", t);
//     for (uint8_t i = 0; i < 10; i++)
//         cJSON_AddItemToArray(t, cJSON_CreateNumber(me->nr_samples[i]));
//     t = cJSON_CreateArray();
//     cJSON_AddItemToObject(gps_speed, "message_nr", t);
//     for (uint8_t i = 0; i < 10; i++)
//         cJSON_AddItemToArray(t, cJSON_CreateNumber(me->message_nr[i]));
//     return ESP_OK;    
// }

esp_err_t gps_speed_str(const struct gps_speed_by_dist_s *me, char *buf, size_t size) {
    snprintf(buf, size, "GPS_speed:{m_set_distance: %hu, m_distance: %ld, m_distance_alfa: %ld, m_sample: %lu, m_speed: %.03f, m_speed_alfa: %.03f, m_max_speed: %.03f, display_max_speed: %.03f, old_run: %lu, m_index: %ld, m_Set_Distance: %lu}",
        me->m_set_distance, me->m_distance, me->m_distance_alfa, me->m_sample, me->m_speed, me->m_speed_alfa, me->m_max_speed, me->display_max_speed, me->old_run, me->m_index, me->m_Set_Distance);
    snprintf(buf, size, "time_hour: {");
    uint8_t i, j=10;
    for (i = 0; i < j; i++)
        snprintf(buf, size, "%hhu ", me->time_hour[i]);
    sniprintf(buf, size, "}\n");
    snprintf(buf, size, "time_min: {");
    for (i = 0; i < j; i++)
        snprintf(buf, size, "%hhu ", me->time_min[i]);
    sniprintf(buf, size, "}\n");
    snprintf(buf, size, "time_sec: {");
    for (i = 0; i < j; i++)
        snprintf(buf, size, "%hhu ", me->time_sec[i]);
    sniprintf(buf, size, "}\n");
    snprintf(buf, size, "this_run: {");
    for (i = 0; i < j; i++)
        snprintf(buf, size, "%lu ", me->this_run[i]);
    sniprintf(buf, size, "}\n");
    snprintf(buf, size, "avg_speed: {");
    for (i = 0; i < j; i++)
        snprintf(buf, size, "%.03f ", me->avg_speed[i]);
    snprintf(buf, size, "m_Distance: {");
    for (i = 0; i < j; i++)
        snprintf(buf, size, "%lu ", me->m_Distance[i]);
    sniprintf(buf, size, "}\n");
    snprintf(buf, size, "nr_samples: {");
    for (i = 0; i < j; i++)
        snprintf(buf, size, "%lu ", me->nr_samples[i]);
    snprintf(buf, size, "}\n");
    snprintf(buf, size, "message_nr: {");
    for (uint8_t i = 0; i < 10; i++)
        snprintf(buf, size, "%lu ", me->message_nr[i]);
    snprintf(buf, size, "}\n");
    snprintf(buf, size, "}");
    return ESP_OK;
}

esp_err_t gps_speed_printf(const struct gps_speed_by_dist_s *me) {
    printf("GPS_speed:{\n");
    printf("m_set_distance: %hu\n", me->m_set_distance);
    printf("m_distance: %ld\n", me->m_distance);
    printf("m_distance_alfa: %ld\n", me->m_distance_alfa);
    printf("m_sample: %lu\n", me->m_sample);
    printf("m_speed: %.03f\n", me->m_speed);
    printf("m_speed_alfa: %.03f\n", me->m_speed_alfa);
    printf("m_max_speed: %.03f\n", me->m_max_speed);
    printf("display_max_speed: %.03f\n", me->display_max_speed);
    printf("old_run: %lu\n", me->old_run);
    printf("m_index: %ld\n", me->m_index);
    printf("m_Set_Distance: %lu\n", me->m_Set_Distance);
    printf("time_hour: ");
    uint8_t i, j=10;
    for (i = 0; i < j; i++)
        printf("%hhu ", me->time_hour[i]);
    printf("\n");
    printf("time_min: ");
    for (i = 0; i < j; i++)
        printf("%hhu ", me->time_min[i]);
    printf("\n");
    printf("time_sec: ");
    for (i = 0; i < j; i++)
        printf("%hhu ", me->time_sec[i]);
    printf("\n");
    printf("this_run: ");
    for (i = 0; i < j; i++)
        printf("%lu ", me->this_run[i]);
    printf("\n");
    printf("avg_speed: ");
    for (i = 0; i < j; i++)
        printf("%.03f ", me->avg_speed[i]);
    printf("\n");
    printf("m_Distance: ");
    for (i = 0; i < j; i++)
        printf("%lu ", me->m_Distance[i]);
    printf("\n");
    printf("nr_samples: ");
    for (i = 0; i < j; i++)
        printf("%lu ", me->nr_samples[i]);
    printf("\n");
    printf("message_nr: ");
    for (uint8_t i = 0; i <
        10; i++)
    printf("%lu ", me->message_nr[i]);
    printf("\n");
    printf("}\n");
    return ESP_OK;
}
#endif

void reset_distance_stats(struct gps_speed_by_dist_s *me) {
    for (int i = 0; i < 10; i++) {
        me->avg_speed[i] = 0;
        me->display_speed[i] = 0;
    }
}

double update_distance(gps_context_t *context, struct gps_speed_by_dist_s *me) {
    assert(context);
    // logger_config_t *config = context->config;
    // assert(config);
    ubx_config_t *ubx = context->ubx_device;
    uint8_t sample_rate = ubx->rtc_conf->output_rate, i;
    uint32_t actual_run = context->run_count;
    me->m_Set_Distance = me->m_set_distance * 1000 * sample_rate;  // Note that m_set_distance should now be in mm, so multiply by 1000 and consider the sample_rate !!
    me->m_distance = me->m_distance + lctx.buf_gSpeed[lctx.index_GPS % BUFFER_SIZE];  // the resolution of the distance is 0.1 mm
                                                                                  // the max int32  is 2,147,483,647 mm eq 214,748.3647 meters eq ~214 kilometers !!
    if ((lctx.index_GPS - me->m_index) >= BUFFER_SIZE) {  // controle buffer overflow
        me->m_distance = 0;
        me->m_index = lctx.index_GPS;
    }
    if (me->m_distance > me->m_Set_Distance) {
        // Determine buffer m_index for the desired distance
        while (me->m_distance > me->m_Set_Distance && (lctx.index_GPS - me->m_index) < BUFFER_SIZE) {
            me->m_distance = me->m_distance - lctx.buf_gSpeed[me->m_index % BUFFER_SIZE];
            me->m_distance_alfa = me->m_distance;
            me->m_index++;
        }
        me->m_index--;
        me->m_distance = me->m_distance + lctx.buf_gSpeed[me->m_index % BUFFER_SIZE];
    }
    me->m_sample = lctx.index_GPS - me->m_index + 1;  // Check for the number of samples to avoid division by zero
    if (lctx.index_GPS - me->m_index + 1) {
        me->m_speed = (double)me->m_distance / me->m_sample;  // 10 samples op 1s aan 10mm/s = 100/10 = 10 mm /s
    }
    if (lctx.index_GPS - me->m_index) {
        me->m_speed_alfa = (double)me->m_distance_alfa / (lctx.index_GPS - me->m_index);
    }
    if (me->m_distance < me->m_Set_Distance)
        me->m_speed = 0;  // This is to prevent incorrect speed if the distance has not been reached yet!
    if (me->m_sample >= BUFFER_SIZE)
        me->m_speed = 0;  // This is to prevent incorrect speed if there is a BUFFER_SIZE overflow !!
    if (me->m_speed == 0)
        me->m_speed_alfa = 0;
    if (me->m_max_speed < me->m_speed) {
        me->m_max_speed = me->m_speed;
        struct tm tms;
        getLocalTime(&tms, 0);
        me->time_hour[0] = tms.tm_hour;
        me->time_min[0] = tms.tm_min;
        me->time_sec[0] = tms.tm_sec;
        me->this_run[0] = actual_run;  // om berekening te checken
        me->avg_speed[0] = me->m_max_speed;
        me->m_Distance[0] = me->m_distance;
        me->nr_samples[0] = me->m_sample;
        me->message_nr[0] = ubx->ubx_msg.count_nav_pvt;
        if (me->m_max_speed > me->avg_speed[5]) {
            me->display_speed[5] = me->m_max_speed;  // current run is faster than run[5] !!
            for (i = 9; i > 5; i--) {  // copy other runs
                me->display_speed[i] = me->avg_speed[i];
            }
            sort_display(me->display_speed, 10);
        }
    }
    if (me->m_max_speed > me->avg_speed[9]){
        me->display_max_speed = me->m_max_speed;  // update on the fly, dat klopt hier niet !!!
        me->record = 1;
        context->record = 1;
    }
    else
        me->display_max_speed = me->avg_speed[9];
    if ((actual_run != me->old_run) && (me->this_run[0] == me->old_run)) {  // opslaan hoogste snelheid van run + sorteren
        sort_run_alfa(me->avg_speed, me->m_Distance, me->message_nr, me->time_hour, me->time_min, me->time_sec, me->this_run, me->nr_samples, 10);
        for (i = 0; i < 10; i++) {
            me->display_speed[i] = me->avg_speed[i];  // to make a direct update on the screen possible
        }
        me->avg_speed[0] = 0;
        me->m_max_speed = 0;
        me->record = 0;
    }
    me->old_run = actual_run;
#if defined(CONFIG_GPS_LOG_LEVEL_TRACE)  && defined(GPS_TRACE_MSG_SPEED)
    gps_speed_printf(me);
#endif
    return me->m_max_speed;
}

/*Instantie om gemiddelde snelheid over een bepaald tijdvenster te
 * bepalen*******************************************/
struct gps_speed_by_time_s *init_gps_time(struct gps_speed_by_time_s *me, uint16_t tijdvenster) {
    ILOG(TAG, "[%s]", __func__);
    memset(me, 0, sizeof(struct gps_speed_by_time_s));
    me->time_window = tijdvenster;
    return me;
}
#if defined(CONFIG_GPS_LOG_LEVEL_TRACE)  && defined(GPS_TRACE_MSG_TIME)
esp_err_t gps_time_printf(const struct gps_speed_by_time_s *me) {
    printf("GPS_time:{\n");
    printf("time_window: %hu\n", me->time_window);
    printf("avg_s: %.03f\n", me->avg_s);
    printf("avg_s_sum: %ld\n", me->avg_s_sum);
    printf("s_max_speed: %.03f\n", me->s_max_speed);
    printf("display_max_speed: %.03f\n", me->display_max_speed);
    printf("speed_bar_run_counter: %lu\n", me->speed_bar_run_counter);
    printf("old_run: %lu\n", me->old_run);
    printf( "reset_display_last_run: %lu\n", me->reset_display_last_run);
    printf("avg_5runs: %.03f\n", me->avg_5runs);
    printf("time_hour: ");
    uint8_t i, j=10;
    for (i = 0; i < j; i++)
        printf("%hhu ", me->time_hour[i]);
    printf("\n");
    printf("time_min: ");
    for (i = 0; i < j; i++)
        printf("%hhu ", me->time_min[i]);
    printf("\n");
    printf("time_sec: ");
    for (i = 0; i < j; i++)
        printf("%hhu ", me->time_sec[i]);
    printf("\n");
    printf("this_run: ");
    for (i = 0; i < j; i++)
        printf("%lu ", me->this_run[i]);
    printf("\n");
    printf("avg_speed: ");
    for (i = 0; i < j; i++)
        printf("%.03f ", me->avg_speed[i]);
    printf("\n");
    printf("display_speed: ");
    for (i = 0; i < j; i++)
        printf("%.03f ", me->display_speed[i]);
    printf("\n");
    printf("speed_bar_run: ");
    for (i = 0; i < j; i++)
        printf("%.03f ", me->speed_bar_run[i]);
    printf("\n");
    printf("Mean_cno: ");
    for (i = 0; i < j; i++)
        printf("%hu ", me->Mean_cno[i]);
    printf("\n");
    printf("Max_cno: ");
    for (i = 0; i < j; i++)
    printf("%hhu ", me->Max_cno[i]);
    printf("\n");
    printf("Min_cno: ");
    for (i = 0; i < j; i++)
        printf("%hhu ", me->Min_cno[i]);
    printf("\n");
    printf("Mean_numSat: ");
    for (i = 0; i < j; i++)
        printf("%hhu ", me->Mean_numSat[i]);
    printf("\n");
    printf("}\n");
    return ESP_OK;
}
#endif
void reset_time_stats(struct gps_speed_by_time_s *me) {
    for (int i = 0; i < 10; i++) {
        me->avg_speed[i] = 0;
        me->display_speed[i] = 0;
    }
    me->avg_5runs = 0;
}

float update_speed(gps_context_t *context, struct gps_speed_by_time_s *me) {
    assert(context);
    ubx_config_t *ubx = context->ubx_device;
    uint8_t sample_rate = ubx->rtc_conf->output_rate, i;
    uint32_t actual_run = context->run_count, time_window_delta = me->time_window * sample_rate;
    if (time_window_delta < BUFFER_SIZE) {  // if time window is smaller than the sample_rate*BUFFER, use normal buffer
        me->avg_s_sum += lctx.buf_gSpeed[lctx.index_GPS % BUFFER_SIZE];  // always add gSpeed at every update
        if (lctx.index_GPS >= time_window_delta) {
            me->avg_s_sum = me->avg_s_sum - lctx.buf_gSpeed[(lctx.index_GPS - time_window_delta) % BUFFER_SIZE];  // once 10s is reached, subtract -10s from the sum again
        }
        me->avg_s = (double)me->avg_s_sum / me->time_window / sample_rate;
        if (me->s_max_speed < me->avg_s) {
            me->s_max_speed = me->avg_s;
            me->speed_bar_run[actual_run % NR_OF_BAR] = me->avg_s;
            struct tm tms;
            getLocalTime(&tms, 0);
            me->time_hour[0] = tms.tm_hour;
            me->time_min[0] = tms.tm_min;
            me->time_sec[0] = tms.tm_sec;
            me->this_run[0] = actual_run;
            me->avg_speed[0] = me->s_max_speed;
            me->Mean_cno[0] = context->Ublox_Sat.sat_info.Mean_mean_cno;
            me->Max_cno[0] = context->Ublox_Sat.sat_info.Mean_max_cno;
            me->Min_cno[0] = context->Ublox_Sat.sat_info.Mean_min_cno;
            me->Mean_numSat[0] = context->Ublox_Sat.sat_info.Mean_numSV;
            // To update the average during the run, calculate the average of the unsorted array!
            if (me->s_max_speed > me->avg_speed[5]) {
                me->avg_5runs = 0;
                for (i = 6; i < 10; i++) {
                    me->avg_5runs = me->avg_5runs + me->avg_speed[i];
                }
                me->avg_5runs = me->avg_5runs + me->avg_speed[0];
                me->avg_5runs = me->avg_5runs / 5;
                me->display_speed[5] = me->s_max_speed;  // current run is faster than run[5] !!
                for (i = 9; i > 5; i--) {  // copy other runs
                    me->display_speed[i] = me->avg_speed[i];
                }
                sort_display(me->display_speed, 10);
            }
            if (me->s_max_speed > me->avg_speed[9]) {
                me->display_max_speed = me->s_max_speed;  // update on the fly, that doesn't make sense here !!!
                me->record = 1;
                context->record = 1;
            }
            else
                me->display_max_speed = me->avg_speed[9];
        }
        if ((actual_run != me->old_run) && (me->this_run[0] == me->old_run)) {  // sorting only if new max during this run !!!
            sort_run(me->avg_speed, me->time_hour, me->time_min, me->time_sec, me->Mean_cno, me->Max_cno, me->Min_cno, me->Mean_numSat, me->this_run, 10);
            if (me->s_max_speed > 5000)
                me->speed_bar_run_counter++;  // changes SW5.51 min speed bar graph = 5 m/s
            for (i = 0; i < 10; i++) {
                me->display_speed[i] = me->avg_speed[i];  // to make a direct update on the screen possible
            }
            me->speed_bar_run[actual_run % NR_OF_BAR] = me->avg_speed[0];  // SW 5.5
            me->avg_speed[0] = 0;
            me->s_max_speed = 0;
            me->avg_5runs = 0;
            me->record = 0;
            for (i = 5; i < 10; i++) {
                me->avg_5runs = me->avg_5runs + me->avg_speed[i];
            }
            me->avg_5runs = me->avg_5runs / 5;
        }
        if ((actual_run != me->reset_display_last_run) && (me->avg_s > 3000)) {
            me->reset_display_last_run = actual_run;
            me->display_last_run = 0;
        } 
        else if (me->display_last_run < me->s_max_speed) {
            me->display_last_run = me->s_max_speed;
        }
        me->old_run = actual_run;
        //return me->s_max_speed;
    } else if (lctx.index_GPS % sample_rate == 0) {  // switch to seconds buffer, but only one update per second !!
        me->avg_s_sum += lctx.buf_secSpeed[lctx.index_sec % BUFFER_SIZE];  // lctx.buf_secSpeed[BUFFER_SIZE] and lctx.index_sec
        if (lctx.index_sec >= me->time_window) {
            me->avg_s_sum = me->avg_s_sum - lctx.buf_secSpeed[(lctx.index_sec - me->time_window) % BUFFER_SIZE];  // vanaf 10s bereikt, terug -10s aftrekken van som
        }
        me->avg_s = (double)me->avg_s_sum / me->time_window;  // in de seconden array staat de gemiddelde van gspeed !!
        // ESP_LOGI(TAG, "avg_s ");Serial.println(avg_s);
        if (me->s_max_speed < me->avg_s) {
            me->s_max_speed = me->avg_s;
            struct tm tms;
            getLocalTime(&tms, 0);
            me->time_hour[0] = tms.tm_hour;
            me->time_min[0] = tms.tm_min;
            me->time_sec[0] = tms.tm_sec;
            me->this_run[0] = actual_run;
            me->avg_speed[0] = me->s_max_speed;  // s_max_speed niet resetten bij elke run !!!
        }
        if (me->s_max_speed > me->avg_speed[9])
            me->display_max_speed = me->s_max_speed;  // update on the fly voor S1800 / S3600
        else
            me->display_max_speed = me->avg_speed[9];
        if ((actual_run != me->old_run) && (me->this_run[0] == me->old_run)) {  // sorting only if new max during this run !!! 
            // sort_run(avg_speed,time_hour,time_min,time_sec,this_run,10);
            sort_run(me->avg_speed, me->time_hour, me->time_min, me->time_sec, me->Mean_cno, me->Max_cno, me->Min_cno, me->Mean_numSat, me->this_run, 10);
            me->avg_speed[0] = 0;
            me->s_max_speed = 0;
            me->avg_5runs = 0;
            for (i = 5; i < 10; i++) {
                me->avg_5runs = me->avg_5runs + me->avg_speed[i];
            }
            me->avg_5runs = me->avg_5runs / 5;
        }
        me->old_run = actual_run;
        //return me->s_max_speed;
    }
    //}
#if defined(CONFIG_GPS_LOG_LEVEL_TRACE)  && defined(GPS_TRACE_MSG_TIME)
    gps_time_printf(me);
#endif
    return me->s_max_speed;  // anders compiler waarschuwing control reaches end of non-void function [-Werror=return-type]
}

struct gps_speed_alfa_s *init_alfa_speed(struct gps_speed_alfa_s *me, int alfa_radius) {
    ILOG(TAG, "[%s]", __func__);
    memset(me, 0, sizeof(struct gps_speed_alfa_s));
    me->alfa_circle_square = alfa_radius * alfa_radius;  // to avoid sqrt calculation !!
    return me;
}
    
#if defined(CONFIG_GPS_LOG_LEVEL_TRACE)  && defined(GPS_TRACE_MSG_ALPHA)
esp_err_t alfa_speed_printf(const struct gps_speed_alfa_s *me) {
    printf("alfa_speed:{\n");
    printf("alfa_circle_square: %.03f\n", me->alfa_circle_square);
    printf("straight_dist_square: %.03f\n", me->straight_dist_square);
    printf("alfa_speed: %.03f\n", me->alfa_speed);
    printf("alfa_speed_max: %.03f\n", me->alfa_speed_max);
    printf("display_max_speed: %.03f\n", me->display_max_speed);
    printf("old_alfa_count: %lu\n", me->old_alfa_count);
    printf("real_distance: ");
    uint8_t i, j=10;
    for (i = 0; i < j; i++)
        printf("%ld ", me->real_distance[i]);
    printf("\n");
    printf("time_hour: ");
    for (i = 0; i < j; i++)
        printf("%hhu ", me->time_hour[i]);
    printf("\n");
    printf("time_min: ");
    for (i = 0; i < j; i++)
        printf("%hhu ", me->time_min[i]);
    printf("\n");
    printf("time_sec: ");
    for (i = 0; i < j; i++)
        printf("%hhu ", me->time_sec[i]);
    printf("\n");
    printf("this_run: ");
    for (i = 0; i < j; i++)
        printf("%lu ", me->this_run[i]);
    printf("\n");
    printf("avg_speed: ");
    for (i = 0; i < j; i++)
        printf("%.03f ", me->avg_speed[i]);
    printf("\n");
    printf("message_nr: ");
    for (i = 0; i < j; i++)
        printf("%lu ", me->message_nr[i]);
    printf("\n");
    printf("alfa_distance: ");
    for (i = 0; i < j; i++)
        printf("%lu ", me->alfa_distance[i]);
    printf("\n");
    printf("}\n");
    return ESP_OK;
}
#endif

// Attention, here the distance traveled must be less than 500 m! Therefore, 
// an extra variable, m_speed_alfa, is provided in GPS_speed!!!
float update_alfa_speed(gps_context_t *context, struct gps_speed_alfa_s *me, struct gps_speed_by_dist_s *M) {
    assert(context);
    ubx_config_t *ubx = context->ubx_device;
    uint8_t sample_rate = ubx->rtc_conf->output_rate;

    // now calculate the absolute distance alfa_speed::alfa_update(GPS_speed M)
    // between the starting point and the end point of the 250m distance,
    // if < 50m this is an alfa !!! note, this is calculated in meters,
    // therefore alfa_circle also in m !! was (M.m_index-1), should be (M.m_index+1)

    me->straight_dist_square =
        (pow((lctx.buf_lat[lctx.index_GPS % BUFFER_ALFA] - lctx.buf_lat[(M->m_index + 1) % BUFFER_ALFA]), 2) +
         pow(cos(DEG2RAD * lctx.buf_lat[lctx.index_GPS % BUFFER_ALFA]) * (lctx.buf_long[lctx.index_GPS % BUFFER_ALFA] - lctx.buf_long[(M->m_index + 1) % BUFFER_ALFA]), 2)
         ) * 111195 * 111195;  // was 111120
    if (me->straight_dist_square < me->alfa_circle_square) {
        me->alfa_speed = M->m_speed_alfa;
        if (M->m_sample >= BUFFER_ALFA)
            me->alfa_speed = 0;  // overflow vermijden bij lage snelheden
        if (me->alfa_speed > me->alfa_speed_max) {
            me->alfa_speed_max = me->alfa_speed;
            me->real_distance[0] = (int32_t)me->straight_dist_square;
            struct tm tms; 
            getLocalTime(&tms, 0);
            me->time_hour[0] = tms.tm_hour;
            me->time_min[0] = tms.tm_min;
            me->time_sec[0] = tms.tm_sec;
            me->this_run[0] = lctx.alfa_count;  // was alfa_count
            me->avg_speed[0] = me->alfa_speed_max;
            me->message_nr[0] = ubx->ubx_msg.count_nav_pvt;
            me->alfa_distance[0] = M->m_distance_alfa / sample_rate;
        }
    }
    // if((alfa_speed_max>0.0f)&(straight_dist_square>(alfa_circle_square*1.4))){ //alfa max gaat pas op 0 indien 500 m na de gijp, rechte afstand na de gijp
    if (context->run_count != me->old_alfa_count) {
        sort_run_alfa(me->avg_speed, me->real_distance, me->message_nr, me->time_hour, me->time_min, me->time_sec, me->alfa_distance, me->this_run, 10);
        /* char tekst[20] = "";
        char message[255] = "";
        strcat(message, " alfa_speed ");
        xdtostrf(M->m_set_distance, 3, 0, tekst);
        strcat(message, "m ");
        xdtostrf(me->alfa_speed_max * (context->rtc->RTC_calibration_speed), 4, 2, tekst);
        strcat(message, tekst);
        strcat(message, "\n"); */
        // logERR(message);
        me->alfa_speed = 0;
        me->alfa_speed_max = 0;
        me->record = 0;
    }
    me->old_alfa_count = context->run_count;
    if (me->alfa_speed_max > me->avg_speed[9]) {
        me->display_max_speed = me->alfa_speed_max;  // update on the fly, that's not correct here !!!
        me->record = 1;
        context->record = 1;
    }
    else
        me->display_max_speed = me->avg_speed[9];
#if defined(CONFIG_GPS_LOG_LEVEL_TRACE)  && defined(GPS_TRACE_MSG_ALPHA)
    alfa_speed_printf(me);
#endif
    return me->alfa_speed_max;
}

void reset_alfa_stats(struct gps_speed_alfa_s *me) {
    for (int i = 0; i < 10; i++) {
        me->avg_speed[i] = 0;
    }
}

/* Calculation of the average heading over the last 10 seconds 
************************************************************************/
#define MEAN_HEADING_TIME 15  // time in sec for calculating average heading
#define STRAIGHT_COURSE_MAX_DEV 10  // max angle deviation for straight ahead recognition (degrees)
#define JIBE_COURSE_DEVIATION_MIN 50  // min angle deviation for jibe detection (degrees)
#define TIME_DELAY_NEW_RUN 10U // uint time_delay_new_run
uint32_t new_run_detection(gps_context_t *context, float actual_heading, float S2_speed) {
    assert(context);
    ubx_config_t *ubx = context->ubx_device;
    uint8_t sample_rate = ubx->rtc_conf->output_rate;
    uint16_t speed_detection_min = SPEED_DETECTION_MIN;  // minimum snelheid 4m/s (14 km/h)voor snelheid display
    //uint16_t standstill_detection_max = STANDSTILL_DETECTION_MAX;  // maximum snelheid 1 m/s (3.6 km/h) voor stilstand herkenning, was 1.5 m/s change SW5.51
    // float headAcc=ubxMessage.navPvt.headingAcc/100000.0f;  // heading Accuracy wordt niet gebruikt ???
    // actual_heading=ubxMessage.navPvt.heading/100000.0f;
    if ((actual_heading - lctx.old_heading) > 300.0f)
        lctx.delta_heading = lctx.delta_heading - 360.0f;
    if ((actual_heading - lctx.old_heading) < -300.0f)
        lctx.delta_heading = lctx.delta_heading + 360.0f;
    lctx.old_heading = actual_heading;
    lctx.heading = actual_heading + lctx.delta_heading;
    /* detect heading change over 15s is more than 40°, a new run is started !! ***************************************************************************/
    //uint8_t mean_heading_time = MEAN_HEADING_TIME;  // tijd in s voor berekening gemiddelde heading
    //uint8_t straight_course_max = STRAIGHT_COURSE_MAX_DEV;  // max hoek afwijking voor rechtdoor herkenning
    //uint8_t course_deviation_min = JIBE_COURSE_DEVIATION_MIN;  // min hoek afwijking om gijp te detecteren, was 40
    //context->heading_SD = lctx.heading;
    uint16_t mean_heading_delta_time = MEAN_HEADING_TIME * sample_rate;
    context->Mean_heading = context->Mean_heading * (mean_heading_delta_time - 1) / (mean_heading_delta_time) + lctx.heading / (mean_heading_delta_time);
    /* detection stand still, more then 2s with velocity<1m/s **************************************************************************************************/
    if (S2_speed > speed_detection_min)
        lctx.velocity_5 = 1;  // min gemiddelde over 2 s = 1m/s
    if ((S2_speed < STANDSTILL_DETECTION_MAX) && (lctx.velocity_5))
        lctx.velocity_0 = 1;
    else
        lctx.velocity_0 = 0;
    /* Nieuwe run gedetecteerd omwille stilstand *****************************************************************************************************************/
    if (lctx.velocity_0) {
        lctx.velocity_5 = 0;
        lctx.delay_count_before_run = 0;
    }
    /* Nieuwe run gedetecteerd omwille heading  change *****************************************************************************************************************/
    // if(abs(Mean_heading-heading)<straight_course_max){straight_course=true;} // stabiele koers terug bereikt
    if ((fabs(context->Mean_heading - lctx.heading) < STRAIGHT_COURSE_MAX_DEV) && (S2_speed > speed_detection_min)) {
        lctx.straight_course = 1;
    }  // stabiele koers terug bereikt, added min_speed SW5.51
    if (((fabs(context->Mean_heading - lctx.heading) > JIBE_COURSE_DEVIATION_MIN) && (lctx.straight_course))) {
        lctx.straight_course = 0;
        lctx.delay_count_before_run = 0;
        lctx.alfa_count++;  // jibe detection for alfa_indicator ....
    }
    lctx.delay_count_before_run++;
    if (lctx.delay_count_before_run == (TIME_DELAY_NEW_RUN * sample_rate))
        lctx.run_count++;
    return lctx.run_count;
}

/* Here the current "alfa distance" is calculated based on 2 points for the jibe: 
 P1 = 250m and P2 = 100m for the jibe. These points determine an imaginary line, 
 the perpendicular distance to the current position must be less than 50m/s
 when point P1 is passed.
 */
float alfa_indicator(gps_context_t *context, float actual_heading) {
    assert(context);
    ubx_config_t *ubx = context->ubx_device;
    uint8_t sample_rate = ubx->rtc_conf->output_rate;
    struct gps_speed_by_dist_s M250 = context->M250;
    struct gps_speed_by_dist_s M100 = context->M100;
    float P_lat, P_long, P_lat_heading, P_long_heading;
    //,lambda_T,lambda_N,lambda,
    float alfa_afstand;
    if (lctx.alfa_count != lctx.old_alfa_count) {
        context->Ublox.alfa_distance = 0;  // the distance traveled since the jibe detection 10*100.000/10.000=100 samples ?
        lctx.P1_lat = lctx.buf_lat[M250.m_index % BUFFER_ALFA];  // this is the point at -250 m from the current position
        lctx.P1_long = lctx.buf_long[M250.m_index % BUFFER_ALFA];
        lctx.P2_lat = lctx.buf_lat[M100.m_index % BUFFER_ALFA];  // this is the point at -100 m from the current position (speed extrapolation from -250m)
        lctx.P2_long = lctx.buf_long[M100.m_index % BUFFER_ALFA];
    }
    lctx.old_alfa_count = lctx.alfa_count;
    P_lat = lctx.buf_lat[lctx.index_GPS % BUFFER_ALFA];    // actuele positie lat
    P_long = lctx.buf_long[lctx.index_GPS % BUFFER_ALFA];  // actuele positie long
    /*
    float corr_lat=111120;
    float corr_long=111120*cos(DEG2RAD*buf_lat[lctx.index_GPS%BUFFER_ALFA]);
    lambda_T=(lctx.P2_lat-lctx.P1_lat)*(P_lat-lctx.P1_lat)*corr_lat*corr_lat+(lctx.P2_long-lctx.P1_long)*(P_long-lctx.P1_long)*corr_long*corr_long;
    lambda_N=
    pow((lctx.P2_lat-lctx.P1_lat)*corr_lat,2)+pow((lctx.P2_long-lctx.P1_long)*corr_long,2);
    lambda=lambda_T/lambda_N;
    alfa_afstand=sqrt(pow((P_lat-lctx.P1_lat-lambda*(lctx.P2_lat-lctx.P1_lat))*corr_lat,2)+pow((P_long-lctx.P1_long-lambda*(lctx.P2_long-lctx.P1_long))*corr_long,2));
    */
    P_lat_heading = lctx.buf_lat[(lctx.index_GPS - 2 * sample_rate) % BUFFER_ALFA];  //-2s positie lat cos(ubxMessage.navPvt.heading*PI/180.0f/100000.0f)*111120+P_lat;
                            //was eerst sin,extra punt berekenen heading, berekenen met afstand/lengte graad !!
    P_long_heading = lctx.buf_long[(lctx.index_GPS - 2 * sample_rate) % BUFFER_ALFA];  //-2s positie long sin(ubxMessage.navPvt.heading*PI/180.0f/100000.0f)*111120*cos(DEG2RAD*P_lat)+P_long;
                            //berekenen met afstand/lengte graad!!
    context->alfa_exit = dis_point_line(lctx.P1_long, lctx.P1_lat, P_long, P_lat, P_long_heading, P_lat_heading);  //
    alfa_afstand = dis_point_line(P_long, P_lat, lctx.P1_long, lctx.P1_lat, lctx.P2_long, lctx.P2_lat);
    return alfa_afstand;  // current perpendicular distance relative to the line P2-P1, may be max 50m for a valid alfa !!
}

/* Calculates distance from point with corr lat/long to line which passes points lat_1/long_1 and lat_2/long_2 **************************************/
float dis_point_line(float long_act, float lat_act, float long_1, float lat_1, float long_2, float lat_2) {
    float corr_lat = 111195;  // meters per latitude degree
    float corr_long = 111195 * cos(DEG2RAD * lat_act);  // meters per longitude degree, this is a function of latitude !
    float lambda_T, lambda_N, lambda, alfa_distance;
    lambda_T = (lat_2 - lat_1) * (lat_act - lat_1) * corr_lat * corr_lat +
               (long_2 - long_1) * (long_act - long_1) * corr_long * corr_long;
    lambda_N = pow((lat_2 - lat_1) * corr_lat, 2) + pow((long_2 - long_1) * corr_long, 2);
    lambda = lambda_T / lambda_N;
    alfa_distance = sqrt(
        pow((lat_act - lat_1 - lambda * (lat_2 - lat_1)) * corr_lat, 2) +
        pow((long_act - long_1 - lambda * (long_2 - long_1)) * corr_long, 2));
    return alfa_distance;
}
/*Heading tov reference***********************************************************************************

 ref_heading=atan2(((P1_long-P2_long)*corr_long),((P1_lat-P2_lat)*corr_lat))*180/PI;//dit
 is getest en werkt correct, wel +180° tov werkelijke richting
 if(ref_heading<0)ref_heading=ref_heading+360;//atan2 geeft een waarde terug
 tussen -PI en +PI radialen !
 delta_heading=(int)(actual_heading-ref_heading*180/PI)%360;//due to P1-P2, this
 is the opposite direction from travelling ! if(delta_heading>180)
 delta_heading=delta_heading-360; if(delta_heading<-180)
 delta_heading=delta_heading+360;
 */

#endif