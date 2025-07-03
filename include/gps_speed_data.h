#ifndef A320F35D_1092_466D_AFF5_7C8A11B524A2
#define A320F35D_1092_466D_AFF5_7C8A11B524A2

#ifdef __cplusplus
extern "C" {
#endif

#include "sdkconfig.h"
#include <stdint.h>
#include <stdbool.h>
#include "logger_common.h"

#define NUM_OF_SPD_ARRAY_SIZE 10 // number of arrays in the speed by time and speed by distance
#define IDX_OF_SPD_ARRAY_MAX_SPD 9
#define IDX_OF_SPD_ARRAY_MIN_SPD 5

typedef struct gps_tm_s {
    uint8_t hour;      // Hour of the day (0-23)
    uint8_t minute;    // Minute of the hour (0-59)
    uint8_t second;    // Second of the minute (0-59)
} gps_tm_t; // struct size is 3 bytes


typedef struct {
    uint32_t message_nr;
    int32_t dist;
    int32_t real_distance;
} gps_run_alfa_data_t; // struct size is 12 bytes

typedef struct {
    uint32_t message_nr;
    int32_t dist;
    int32_t nr_samples;
} gps_run_dist_data_t; // struct size is 12 bytes

typedef struct {
    uint16_t Mean_cno;
    uint8_t Max_cno;
    uint8_t Min_cno;
    uint8_t Mean_numSat;
} gps_run_time_data_t; // struct size is 6 bytes

typedef struct gps_run_s {
    struct gps_tm_s time;
    float avg_speed;
    uint16_t nr;
    union {
        gps_run_alfa_data_t alfa; // for speed by alfa
        gps_run_dist_data_t dist; // for speed by distance
        gps_run_time_data_t time; // for speed by time
    } data; // union size is 12 bytes
} gps_run_t; // struct size is 32 bytes

#define GPS_RUN_DEFAULT_CONFIG() { \
    .time = {0, 0, 0}, \
    .avg_speed = 0, \
    .nr = 0, \
    .data = {{0}} \
}

typedef struct gps_display_s {
    float display_speed[NUM_OF_SPD_ARRAY_SIZE];
    float display_max_speed; // to update on the fly on display
    float display_last_run_max_speed; // to update on the fly on display
    uint16_t nr_display_last_run;
    uint8_t record;
} gps_display_t; // struct size is 48 bytes

#define GPS_DISPLAY_DEFAULT_CONFIG() { \
    .display_speed = {0}, \
    .display_max_speed = 0, \
    .display_last_run_max_speed = 0, \
    .nr_display_last_run = 0, \
    .record = 0 \
}
typedef struct gps_speed_s {
    gps_run_t runs[NUM_OF_SPD_ARRAY_SIZE];
#if defined(MUTABLE_RUNS)
    gps_run_t runs_mutable[NUM_OF_SPD_ARRAY_SIZE]; // mutable runs for speed calculation
#endif
    gps_display_t display; // display speed for the last 10 runs
    float cur_speed;           // speed over the desired distance
    float max_speed;      // maximum speed of the last run
    uint16_t nr_prev_run;
    uint8_t flags;
} gps_speed_t; // struct size is 320 bytes

typedef struct gps_speed_op_s {
    float(*run_avg_speed)(int, uint8_t, int);
    gps_tm_t*(*run_time)(int, uint8_t, int);
    float(*cur_speed)(int, uint8_t);
    float(*max_speed)(int, uint8_t);
    float(*display_max_speed)(int, uint8_t);
    float(*display_last_run_max_speed)(int, uint8_t);
    float(*display_speed)(int, uint8_t, int);
    bool(*display_record)(int, uint8_t);
    gps_display_t * (*get_time_display)(int);
    gps_display_t * (*get_alfa_display)(int);
} gps_speed_op_t;

#if defined(MUTABLE_RUNS)
#define MRUN .runs = {GPS_RUN_DEFAULT_CONFIG()}, .runs_mutable = {GPS_RUN_DEFAULT_CONFIG()}}
#define RUNS_FOR_DISPLAY runs_mutable
#else
#define MRUN .runs = {GPS_RUN_DEFAULT_CONFIG()},
#define RUNS_FOR_DISPLAY runs
#endif
#define GPS_SPEED_DEFAULT_CONFIG() { \
    MRUN \
    .speed = 0, \
    .max_speed = 0, \
    .display = GPS_DISPLAY_DEFAULT_CONFIG(), \
    .nr_prev_run = 0, \
    .flags = 0, \
}

// Calculation of the alpha speed, instance of gps_speed_by_dist_s 250 /500 can be used + circle diameter (normally 50 m)
typedef struct gps_speed_alfa_s {
    uint16_t distance_window;
    gps_speed_t speed;        // speed over the desired distance
    double straight_dist_square;
    struct gps_speed_by_dist_s *base; // pointer to the base speed instance, if used
} gps_speed_by_alfa_t; // struct size is 32 bytes

#define GPS_SPEED_ALFA_DEFAULT_CONFIG() { \
    .distance_window = 0, \
    .speed = GPS_SPEED_DEFAULT_CONFIG(), \
    .straight_dist_square = 0, \
    .base = NULL, \
}
typedef struct gps_speed_by_dist_s {
    uint16_t distance_window;  // here the instance distance is set, e.g. 100m, 200m, 500m....
    uint32_t distance_window_raw; // raw value of the set distance, e.g. 100m = sample_rate * 100
    gps_speed_t speed;      // speed over the desired distance
    int32_t distance;       // current distance, the next larger distance than "distance", e.g. 100m
    int32_t m_index;
    int32_t m_sample;
    struct gps_speed_alfa_s *alfa; // pointer to the alfa speed instance, if used
} gps_speed_by_dist_t; // struct size is 64 bytes

#define GPS_SPEED_BY_DIST_DEFAULT_CONFIG() { \
    .distance_window = 0, \
    .distance_window_raw = 0, \
    .speed = GPS_SPEED_DEFAULT_CONFIG(), \
    .distance = 0, \
    .m_index = 0, \
    .m_sample = 0, \
    .alfa = NULL, \
}

// #define SPEED_BAR_SETUP 1
#if defined(SPEED_BAR_SETUP)
#define NR_OF_BAR 42 // number of bars in the bar_graph
struct gps_speed_bar_s {
    float run_speed[NR_OF_BAR]; // for bar_graph
    uint16_t bar_count;
};

#define GPS_SPEED_BAR_DEFAULT_CONFIG { \
    .run_speed = {0}, \
    .bar_count = 0 \
},
#else
#define GPS_SPEED_BAR_DEFAULT_CONFIG
#endif

// calculation of average speed over a time window (2s, 10s, 1800s...)
typedef struct gps_speed_by_time_s {
    uint16_t time_window;     // time window in seconds, e.g. 2s, 10s, 1800s...
    gps_speed_t speed;        // speed over the desired time window
    int32_t avg_s_sum;
#if defined(SPEED_BAR_SETUP)
    struct gps_speed_bar_s bar;
#endif
} gps_speed_by_time_t; // struct size is 64 bytes

#define GPS_SPEED_BY_TIME_DEFAULT_CONFIG() { \
    .time_window = 0, \
    .speed = GPS_SPEED_DEFAULT_CONFIG(), \
    .avg_s_sum = 0, \
    GPS_SPEED_BAR_DEFAULT_CONFIG \
}

#define SPEED_TYPE_MASK   0x03      // 2 bits for speed type (00 = time, 01 = dist, 10 = alfa, 11 = other)
#define GPS_SPEED_TYPE_TIME   0x00
#define GPS_SPEED_TYPE_DIST   0x01
#define GPS_SPEED_TYPE_ALFA   0x02
#define GPS_SPEED_TYPE_OTHER  0x03

typedef struct gps_speed_metrics_desc_s {
    uint8_t type;
    int window;
    union {
        gps_speed_by_time_t *time;
        gps_speed_by_dist_t *dist;
    } handle;
} gps_speed_metrics_desc_t; // struct size is 8 bytes

typedef struct gps_speed_metrics_cfg_s {
    uint8_t type;
    int window;
} gps_speed_metrics_cfg_t;

#define GPS_SPEED_BY_TIME_SET(l) l(time_2s) l(time_10s) l(time_1800s) l(time_3600s)
#define GPS_SPEED_BY_DIST_SET(l) l(dist_100m) l(dist_250m) l(dist_500m) l(dist_1852m)
enum gps_speed_metrics_e {
    GPS_SPEED_BY_TIME_SET(ENUM)
    GPS_SPEED_BY_DIST_SET(ENUM)
};
#define alfa_500m dist_500m

extern gps_speed_op_t speed_ops;
#define time_run_avg_speed(a,b) speed_ops.run_avg_speed(a, GPS_SPEED_TYPE_TIME, b)
#define time_cur_speed(a) speed_ops.cur_speed(a, GPS_SPEED_TYPE_TIME)
#define time_run_time(a,b) speed_ops.run_time(a, GPS_SPEED_TYPE_TIME, b)
#define time_max_speed(a) speed_ops.max_speed(a, GPS_SPEED_TYPE_TIME)
#define time_display_max_speed(a) speed_ops.display_max_speed(a, GPS_SPEED_TYPE_TIME)
#define time_display_last_run_max_speed(a) speed_ops.display_last_run_max_speed(a, GPS_SPEED_TYPE_TIME)
#define time_display_speed(a,b) speed_ops.display_speed(a, GPS_SPEED_TYPE_TIME, b)
#define time_display_record(a) speed_ops.display_record(a, GPS_SPEED_TYPE_TIME)
#define dist_run_avg_speed(a,b) speed_ops.run_avg_speed(a, GPS_SPEED_TYPE_DIST, b)
#define dist_cur_speed(a) speed_ops.cur_speed(a, GPS_SPEED_TYPE_DIST)
#define dist_run_time(a,b) speed_ops.run_time(a, GPS_SPEED_TYPE_DIST, b)
#define dist_max_speed(a) speed_ops.max_speed(a, GPS_SPEED_TYPE_DIST)
#define dist_display_max_speed(a) speed_ops.display_max_speed(a, GPS_SPEED_TYPE_DIST)
#define dist_display_last_run_max_speed(a) speed_ops.display_last_run_max_speed(a, GPS_SPEED_TYPE_DIST)
#define dist_display_speed(a,b) speed_ops.display_speed(a, GPS_SPEED_TYPE_DIST, b)
#define dist_display_record(a) speed_ops.display_record(a, GPS_SPEED_TYPE_DIST)
#define alfa_run_avg_speed(a,b) speed_ops.run_avg_speed(a, GPS_SPEED_TYPE_ALFA, b)
#define alfa_cur_speed(a) speed_ops.cur_speed(a, GPS_SPEED_TYPE_ALFA)
#define alfa_run_time(a,b) speed_ops.run_time(a, GPS_SPEED_TYPE_ALFA, b)
#define alfa_max_speed(a) speed_ops.max_speed(a, GPS_SPEED_TYPE_ALFA)
#define alfa_display_max_speed(a) speed_ops.display_max_speed(a, GPS_SPEED_TYPE_ALFA)
#define alfa_display_last_run_max_speed(a) speed_ops.display_last_run_max_speed(a, GPS_SPEED_TYPE_ALFA)
#define alfa_display_speed(a,b) speed_ops.display_speed(a, GPS_SPEED_TYPE_ALFA, b)
#define alfa_display_record(a) speed_ops.display_record(a, GPS_SPEED_TYPE_ALFA)
#define alfa_display(a) speed_ops.get_alfa_display(a)
#define time_display(a) speed_ops.get_time_display(a)

void refresh_gps_speeds_by_distance(void);
void reset_distance_stats(struct gps_speed_by_dist_s *);
void reset_time_stats(struct gps_speed_by_time_s*);
void reset_alfa_stats(struct gps_speed_alfa_s*);
float alfa_indicator(float actual_heading);
void gps_update_max_speed(void);

int gps_speed_metrics_add(const gps_speed_metrics_cfg_t *new_set, int pos);
void gps_speed_metrics_check(const gps_speed_metrics_cfg_t *cfg, size_t num_sets);
void gps_speed_metrics_init(void);
void gps_speed_metrics_free(void);

#ifdef __cplusplus
}
#endif

#endif /* A320F35D_1092_466D_AFF5_7C8A11B524A2 */
