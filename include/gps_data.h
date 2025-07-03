#ifndef GPS_DATA_H
#define GPS_DATA_H


#ifdef __cplusplus
extern "C" {
#endif

#include "sdkconfig.h"
#include <stdint.h>
#include <stdbool.h>
#include "logger_common.h"
#include "gps_satellite_data.h"
#include "gps_speed_data.h"

#define DEG2RAD 0.0174532925f  // is PI/180 !!!
#define BUFFER_SIZE CONFIG_GPS_BUFFER_SIZE
#define BUFFER_SEC_SIZE 3632
#define BUFFER_ALFA CONFIG_GPS_ALFA_BUFFER_SIZE
#define NR_OF_BARS 42         // number of bars in the bar graph

typedef struct gps_point_s {
    float latitude;  // Latitude in degrees
    float longitude; // Longitude in degrees
} gps_point_t; // struct size is 8 bytes

struct gps_data_s {
    float total_distance;
    float run_distance;
    float run_distance_after_turn;
    uint32_t run_start_time;
}; // struct size is 16 bytes

#define GPS_DATA_DEFAULT_CONFIG() { \
    .total_distance = 0, \
    .run_distance = 0, \
    .run_distance_after_turn = 0, \
    .run_start_time = 0 \
}

/**
 * @brief GPS context structure
*/
typedef struct gps_context_s {
    struct gps_data_s Ublox;          // create an object storing GPS_data !
    struct gps_sat_info_s Ublox_Sat;    // create an object storing GPS_SAT info !
    gps_speed_metrics_desc_t *speed_metrics;
    uint16_t num_speed_metrics;
    bool Gps_fields_OK;

    uint16_t run_count;
    uint16_t alfa_count;     // counter for alfa

    uint32_t time_out_gps_msg;
    
    //uint32_t last_gps_msg;
    
    uint8_t next_gpy_full_frame;
    
    int32_t gps_speed;
    uint32_t start_logging_millis;

    float alfa_window;
    float alfa_exit;
    //float heading_SD;

    bool files_opened;
    const uint8_t * mac_address;
    struct ubx_config_s *ubx_device;
    struct gps_log_file_config_s * log_config;
    const char * SW_version;
    uint8_t record;
    uint16_t lost_frames;
        
    bool time_set;
    bool signal_ok;
    uint32_t first_fix;
    // float calibration_speed;
    bool gps_is_moving;
    uint8_t skip_alfa_after_stop;
    gps_run_t max_speed;
} gps_context_t;

#define CONTEXT_GPS_DEFAULT_CONFIG() { \
    .Ublox = GPS_DATA_DEFAULT_CONFIG(), \
    .Ublox_Sat = GPS_SAT_INFO_DEFAULT_CONFIG(), \
    .speed_metrics = NULL, \
    .num_speed_metrics = 0, \
    .Gps_fields_OK = false, \
    .run_count = 0, \
    .alfa_count = 0, \
    .time_out_gps_msg = 0, \
    .next_gpy_full_frame = 0, \
    .gps_speed = 0, \
    .alfa_window = 0, \
    .alfa_exit = 0, \
    .ubx_device = NULL, \
    .files_opened = false,   \
    .mac_address = 0, \
    .start_logging_millis = 0, \
    .log_config = NULL, \
    .SW_version = 0, \
    .record = 0, \
    .lost_frames = 0, \
    .time_set = false, \
    .signal_ok = false, \
    .first_fix = 0, \
    .gps_is_moving = false, \
    .skip_alfa_after_stop = 0, \
    .max_speed = GPS_RUN_DEFAULT_CONFIG(), \
}

extern gps_context_t * gps;

struct gps_data_s * init_gps_data(struct gps_data_s*);

int push_gps_data(struct gps_context_s * context, struct gps_data_s*, float latitude, float longitude, int32_t gSpeed); // hier wordt de gps data in de buffer geplaatst

uint32_t new_run_detection(struct gps_context_s * context, float actual_heading, float S2_speed);

void gps_log_nav_mode_change(gps_context_t *context, uint8_t changed);

int get_cur_nav_mode(int nav_mode);

int32_t gps_last_speed_smoothed(uint8_t average_records);
int32_t gps_last_sec_speed_smoothed(uint8_t average_records);

#ifdef __cplusplus
}
#endif
#endif
