#ifndef DB99F2E7_B596_4059_B6AF_FAD2A14CD6A0
#define DB99F2E7_B596_4059_B6AF_FAD2A14CD6A0

#ifdef __cplusplus
extern "C" {
#endif
#include "sdkconfig.h"
#include "gps_log.h"
#include "gps_log_file.h"
#include "gps_data.h"
#include "config_observer.h"
#include "unified_config.h"
#include "gps_log_events.h"
#include "dstat_screens.h"

struct gps_context_s;

size_t log_write(const struct gps_context_s * context, uint8_t file, const void * msg, size_t len);
int log_close(const struct gps_context_s * context, uint8_t file);
int log_fsync(const struct gps_context_s * context, uint8_t file);
void printFile(const char *filename);

struct gps_context_s;
extern struct gps_context_s * gps;
extern uint32_t buffer_caps;
#define NOGPX (log_get_fd((context), sd_log_gpx)==-1)
#if defined (CONFIG_GPS_LOG_GPY)
#define NOGPY (log_get_fd((context), sd_log_gpy)==-1)
#endif
#define NOUBX (log_get_fd((context), sd_log_ubx)==-1)
#define NOSBP (log_get_fd((context), sd_log_sbp)==-1)
#define NOTXT (log_get_fd((context), sd_log_txt)==-1)

#define WRITEGPX(msg, len) log_write((gps), sd_log_gpx, (msg), (len))
#if defined (CONFIG_GPS_LOG_GPY)
#define WRITEGPY(msg, len) log_write((gps), sd_log_gpy, (msg), (len))
#endif
#define WRITEUBX(msg, len) log_write((gps), sd_log_ubx, (msg), (len))
#define WRITESBP(msg, len) log_write((gps), sd_log_sbp, (msg), (len))
#define WRITETXT(msg, len) log_write((gps), sd_log_txt, (msg), (len))
#define GET_FD(f) (gps->log_config->filefds[f])

inline int log_get_fd(const struct gps_context_s * context, uint8_t file) {
    return GET_FD(file);
}

#include "sdkconfig.h"
#if (defined(CONFIG_LOGGER_USE_GLOBAL_LOG_LEVEL) && CONFIG_LOGGER_GLOBAL_LOG_LEVEL < CONFIG_GPS_LOG_LEVEL)
#define C_LOG_LEVEL CONFIG_LOGGER_GLOBAL_LOG_LEVEL
#else
#define C_LOG_LEVEL CONFIG_GPS_LOG_LEVEL
#endif
#include "common_log.h"

#define ALFA_DISTANCE_MAX 501.0f
#define ALPHA_BUFFER_SIZE(sample_rate, speed) (uint32_t)(sample_rate * (ALFA_DISTANCE_MAX / speed) + 1.5f)
#define ALFA_THRESHOLDS_MS {2.2f, 2.8f, 3.3f, 4.2f, 5.0f, 5.6f} // m/s for 8,10,12,15,18,20 km/h
#define ALFA_THRESHOLD_IDX_TABLE {0,0,0,0,0,0,1,1,1,1,1,2,2,2,2,2,2} // index for speed thresholds for alfa logging based on the configured rate (1-20Hz)
extern const float speed_thresholds_for_alfa[];
/// known rates are 1, 2, 5, 10, 16 and 20Hz but we want to support any rate between 1 and 20, 
/// so we use a lookup table for the thresholds for alfa logging based on the configured rate
extern const uint8_t speed_threshold_index[17];
inline float spd_threshold_for_alfa(int rate) {
    if (rate > 16) return speed_thresholds_for_alfa[3];
    if (rate < 0) return speed_thresholds_for_alfa[0];
    return speed_thresholds_for_alfa[speed_threshold_index[rate]];
}
#define MIN_numSV_FIRST_FIX 5      // before start logging, changed from 4 to 5 7.1/2023
#define MAX_Sacc_FIRST_FIX 2       // before start logging
#define MIN_numSV_GPS_SPEED_OK  4  // minimum number of satellites for calculating speed, otherwise
#define MAX_Sacc_GPS_SPEED_OK  1   // max Sacc value for calculating speed, otherwise 0
#define MAX_GPS_SPEED_OK  60       // max speed in m/s for calculating speed, otherwise 0 - [60 m/s = 216 km/h]

#define TIME_DELAY_FIRST_FIX 10        // 10 navpvt messages before start logging

#define SPEED_DETECTION_MIN 4000       // min average speed over 2s for new run detection (mm/s)
#define STANDSTILL_DETECTION_MAX  1000 // max average speed over 2s voor stand still detection (mm/s)

#define MEAN_HEADING_TIME 15         // time in sec for calculating average heading
#define STRAIGHT_COURSE_MAX_DEV 10   // max angle deviation for straight ahead recognition (degrees)
#define JIBE_COURSE_DEVIATION_MIN 50 // min angle deviation for jibe detection (degrees)
#define TIME_DELAY_NEW_RUN 10U       // uint time_delay_new_run

typedef struct gps_p_context_s {
    int32_t buf_gspeed[BUFFER_SIZE];   // speed buffer counted by gps rate
    uint16_t buf_gspeed_size; // size of the speed buffer counted by gps rate
#if defined(CONFIG_GPS_LOG_STATIC_S_BUFFER)
    int16_t buf_sec_speed[BUFFER_SEC_SIZE]; // speed buffer counted by sec
#else
    int16_t *buf_sec_speed;             // speed buffer counted by sec
#endif
    uint16_t buf_sec_speed_size;        // size of the speed buffer counted by sec

    uint32_t index_gspeed;              // by rate counted speed buffer pos (-1), start at 0 on first pass !!
    uint32_t index_sec;                 // by sec counted speed buffer pos (-1), start at 0 on first pass !!

    int32_t sec_gSpeed;      // for avg speed per second mm/s
    uint16_t delay_count_before_run;    // count loops to wait before incerment the run count
    uint16_t old_alfa_count; // previous alfa counter

    bool velocity_0;      // min gemiddelde over 2 s < 1m/s
    bool velocity_5;      // min gemiddelde over 2 s = 1m/s

    bool straight_course; // straight course or not
#if defined(CONFIG_GPS_LOG_STATIC_A_BUFFER)
    gps_point_t alfa_buf[BUFFER_ALFA]; // buffer for gps points
#else
    gps_point_t *alfa_buf;
#endif
    uint16_t alfa_buf_size;
    gps_point_t alfa_p1; // Point 1 for distance calculation
    gps_point_t alfa_p2; // Point 2 for distance calculation

    float delta_heading;
    float delta_dist;
    float heading;     // heding for alfa // = 0;
    float old_heading; // old heading for alfa // = 0
    float heading_mean;
    uint32_t standstill_start_millis;
    SemaphoreHandle_t xMutex;
    uint32_t count_nav_pvt;
} gps_p_context_t;

#define AA .buf_gspeed = {0}, .buf_gspeed_size = BUFFER_SIZE

#if defined(CONFIG_GPS_LOG_STATIC_A_BUFFER)
#define AI .alfa_buf = {0}, .alfa_buf_size = BUFFER_ALFA
#else
#define AI .alfa_buf = NULL, .alfa_buf_size = 0
#endif

#if defined(CONFIG_GPS_LOG_STATIC_S_BUFFER)
#define SSI .buf_sec_speed = {0}, .buf_sec_speed_size = BUFFER_SEC_SIZE
#else
#define SSI .buf_sec_speed = NULL, .buf_sec_speed_size = 0
#endif

#define GPS_P_CONTEXT_INIT { \
    AA, \
    SSI, \
    .index_gspeed = UINT32_MAX, \
    .index_sec = UINT32_MAX, \
    .sec_gSpeed = 0, \
    .delay_count_before_run = 0, \
    .old_alfa_count = 0, \
    .velocity_0 = false, \
    .velocity_5 = false, \
    .straight_course = false, \
    .delta_heading = 0, \
    .heading_mean = 0, \
    AI, \
    .delta_dist = 0, \
    .heading = 0, \
    .old_heading = 0, \
    .alfa_p1 = {0,0}, \
    .alfa_p2 = {0,0}, \
    .standstill_start_millis = 0, \
    .xMutex = NULL, \
    .count_nav_pvt = 0 \
}

extern gps_p_context_t log_p_lctx;

inline bool gps_log_file_bits_check(cfg_gps_log_enables_t *enables) {
    return (
            enables->bits.log_ubx
            || enables->bits.log_sbp
#if defined (GPS_LOG_ENABLE_GPY)
            || enables->bits.log_gpy
#endif
            || enables->bits.log_gpx
        );
}

extern float get_spd(float b);

extern float get_avg5(const float *arr, float (*conv)(float), int start_index);

inline float get_display_avg(const gps_display_t *b) {
    return b ? get_avg5(b->display_speed, get_spd, IDX_OF_SPD_ARRAY_MIN_SPD) : 0.0f;
}

static inline int32_t al_buf_index(uint32_t idx) {
    return log_p_lctx.alfa_buf_size ? (idx + log_p_lctx.alfa_buf_size) % log_p_lctx.alfa_buf_size : 0;
}

static inline int32_t buf_index(uint32_t idx) {
    return log_p_lctx.buf_gspeed_size ? (idx + log_p_lctx.buf_gspeed_size) % log_p_lctx.buf_gspeed_size : 0;
}

static inline int32_t sec_buf_index(uint32_t idx) {
    return log_p_lctx.buf_sec_speed_size ? (idx + log_p_lctx.buf_sec_speed_size) % log_p_lctx.buf_sec_speed_size : 0;
}

#if !defined(CONFIG_GPS_LOG_STATIC_A_BUFFER)
void gps_check_alfa_buf(size_t new_size);
void gps_free_alfa_buf(void);
#endif
#if !defined(CONFIG_GPS_LOG_STATIC_S_BUFFER)
void gps_check_sec_buf(size_t new_size);
void gps_free_sec_buf(void);
#endif

void refresh_gps_speeds_by_distance(void);
struct gps_speed_by_dist_s *init_gps_speed_by_distance(struct gps_speed_by_dist_s *me, uint16_t);
float update_speed_by_distance(struct gps_speed_by_dist_s *);
struct gps_speed_alfa_s * init_gps_speed_by_alfa(struct gps_speed_by_dist_s*); //constructor
float update_speed_by_alfa(struct gps_speed_by_dist_s*);


struct gps_speed_by_time_s * init_gps_speed_by_time(struct gps_speed_by_time_s*, uint16_t); // description of the constructor
float update_speed_by_time(struct gps_speed_by_time_s*);

void gps_speed_metrics_update(void);
void gps_speed_metrics_save_session(void);

void init_gps_context_fields(struct gps_context_s * ctx);
void deinit_gps_context_fields(struct gps_context_s *ctx);

#if defined(GPS_STATS)
#if defined(GPS_MSG_CTX)
static esp_err_t gps_p_context_printf(const gps_p_context_t *me);
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

bool check_and_alloc_buffer(void **buf, size_t required_count, size_t elem_size, uint16_t *current_count, uint32_t caps);
void unalloc_buffer(void **buf);
esp_err_t log_gps_timeout(const gps_context_t *context, uint32_t period_ms, const char *tag);

#ifdef __cplusplus
}
#endif
#endif /* DB99F2E7_B596_4059_B6AF_FAD2A14CD6A0 */
