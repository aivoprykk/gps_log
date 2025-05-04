#ifndef GPS_DATA_H
#define GPS_DATA_H


#ifdef __cplusplus
extern "C" {
#endif

#include "sdkconfig.h"
#include <stdint.h>
#include <stdbool.h>

#define DEG2RAD 0.0174532925  // is PI/180 !!!
#define BUFFER_SIZE CONFIG_GPS_BUFFER_SIZE
#define BUFFER_ALFA CONFIG_GPS_ALFA_BUFFER_SIZE
#define NAV_SAT_BUFFER CONFIG_GPS_NAV_SAT_BUFFER_SIZE
#define FILTER_MIN_SATS 5
#define FILTER_MAX_sACC 2   
#define NR_OF_BARS 42         // number of bars in the bar graph


// Description of the GPS data processing class
struct gps_data_s {
    float total_distance;
    float run_distance;
    float alfa_distance;
    uint32_t run_start_time;
};

#define GPS_DATA_DEFAULT_CONFIG() { \
    .total_distance = 0, \
    .run_distance = 0, \
    .alfa_distance = 0, \
    .run_start_time = 0 \
}

/**
 * @brief GPS satellite information structure
*/
struct SAT_info {
      uint16_t Mean_cno[NAV_SAT_BUFFER]; // Mean cno value from all sats used in nav
      uint8_t Max_cno[NAV_SAT_BUFFER];   // Max cno value from all sats used in nav      
      uint8_t Min_cno[NAV_SAT_BUFFER];   // Min cno value from all sats used in nav
      uint8_t numSV[NAV_SAT_BUFFER];     // Nr of sats used in nav 
      uint16_t Mean_mean_cno ;           // Mean over last x NAV_SAT messages
      uint8_t Mean_max_cno ;             // Mean over last x NAV_SAT messages
      uint8_t Mean_min_cno ;             // Mean over last x NAV_SAT messages
      uint8_t Mean_numSV ;               // Mean over last x NAV_SAT messages
 };

 #define SAT_INFO_DEFAULT_CONFIG() { \
    .Mean_cno = {0}, \
    .Max_cno = {0}, \
    .Min_cno = {0}, \
    .numSV = {0}, \
    .Mean_mean_cno = 0, \
    .Mean_max_cno = 0, \
    .Mean_min_cno = 0, \
    .Mean_numSV = 0 \
}

/**
 * @brief GPS satellite information structure
 */
struct GPS_SAT_info {
    struct SAT_info sat_info;
    uint32_t index_SAT_info;
    uint16_t mean_cno;
    uint8_t max_cno;
    uint8_t min_cno;
    uint8_t nr_sats;
};

#define GPS_SAT_INFO_DEFAULT_CONFIG() { \
    .sat_info = SAT_INFO_DEFAULT_CONFIG(), \
    .index_SAT_info = 0, \
    .mean_cno = 0, \
    .max_cno = 0, \
    .min_cno = 0, \
    .nr_sats = 0 \
}

struct nav_sat_s;

/**
 * @brief GPS average speed over a certain distance configuration structure 
*/
// Calculating the average speed over a certain distance, sample rate has no influence !!****************/
struct gps_speed_by_dist_s {
    float m_speed;           // speed over the desired distance
    float m_speed_alfa;      // alpha speed over the desired distance
    double m_max_speed;      // maximum speed of the last run
    float display_max_speed; // to update on the fly on display
    double avg_speed[10];
    double display_speed[10]; // speed for update on the fly on display
    int32_t m_Distance[10];
    uint8_t time_hour[10];
    uint8_t time_min[10];
    uint8_t time_sec[10];
    uint32_t this_run[10];
    uint32_t nr_samples[10];
    uint32_t message_nr[10];
    int32_t m_index;
    int32_t m_distance;       // current distance, the next larger distance than "distance", e.g. 100m
    uint32_t m_distance_alfa; // the next smaller distance than "distance", used for alpha calculation
    uint16_t m_set_distance;  // here the instance distance is set, e.g. 100m, 200m, 500m....
    uint32_t m_Set_Distance;  // here in the correct resolution, multiplied by 1000 and sample_rate !!
    uint32_t m_sample;
    //private
    uint32_t old_run;
    uint8_t record;
};

#define GPS_SPEED_DEFAULT_CONFIG() { \
    .m_speed = 0, \
    .m_speed_alfa = 0, \
    .m_max_speed = 0, \
    .display_max_speed = 0, \
    .m_Distance = {0}, \
    .time_hour = {0}, \
    .time_min = {0}, \
    .time_sec = {0}, \
    .this_run = {0}, \
    .nr_samples = {0}, \
    .message_nr = {0}, \
    .m_index = 0, \
    .m_distance = 0, \
    .m_distance_alfa = 0, \
    .m_set_distance = 0, \
    .m_Set_Distance = 0, \
    .m_sample = 0, \
    .old_run = 0, \
    .record = 0 \
}

/**
 * @brief GPS average speed over a time window configuration structure
*/
// calculation of average speed over a time window (2s, 10s, 1800s...)
// ****************************************************************
struct gps_speed_by_time_s {
    double avg_s;             // speed over the desired time window
    int32_t avg_s_sum;
    double s_max_speed;       // maximum speed of the last run
    float display_max_speed;  // to update on the fly on display
    float display_last_run;
    double avg_speed[10];
    double display_speed[10]; // speed for update on the fly on display
    double avg_5runs;         // speed average for 5 runs over the desired time window
    uint8_t time_hour[10];
    uint8_t time_min[10];
    uint8_t time_sec[10];
    uint32_t this_run[10];
    uint16_t time_window; 
    uint32_t speed_bar_run_counter;
    double speed_bar_run[50];       //for bar_graph
    uint16_t Mean_cno[10];
    uint8_t Max_cno[10];
    uint8_t Min_cno[10];
    uint8_t Mean_numSat[10];
    //private
    uint32_t old_run;
    uint32_t reset_display_last_run;
    uint8_t record;
}; 

#define GPS_TIME_DEFAULT_CONFIG { \
    .avg_s = 0, \
    .avg_s_sum = 0, \
    .s_max_speed = 0, \
    .display_max_speed = 0, \
    .display_last_run = 0, \
    .avg_speed = {0}, \
    .display_speed = {0}, \
    .avg_5runs = 0, \
    .time_hour = {0}, \
    .time_min = {0}, \
    .time_sec = {0}, \
    .this_run = {0}, \
    .time_window = 0, \
    .speed_bar_run_counter = 0, \
    .speed_bar_run = {0}, \
    .Mean_cno = {0}, \
    .Max_cno = {0}, \
    .Min_cno = {0}, \
    .Mean_numSat = {0}, \
    .old_run = 0, \
    .reset_display_last_run = 0, \
    .record = 0 \
}

/**
 * @brief GPS Calculation of the alpha speed configuration structure 
*/
// Calculation of the alpha speed, instance of gps_speed_by_dist_s 250 /500 can be used + circle diameter (normally 50 m)
struct gps_speed_alfa_s {
    double straight_dist_square;
    double alfa_speed;
    double alfa_speed_max;
    float display_max_speed;   // to update on the fly on display
    double alfa_circle_square;
    double avg_speed[10];
    int32_t real_distance[10];
    uint8_t time_hour[10];
    uint8_t time_min[10];
    uint8_t time_sec[10];
    uint32_t this_run[10];
    uint32_t message_nr[10];
    uint32_t alfa_distance[10];
    //private
    uint32_t alfa_count;
    uint32_t old_alfa_count;
    uint8_t record;
};

#define ALFA_SPEED_DEFAULT_CONFIG() { \
    .straight_dist_square = 0, \
    .alfa_speed = 0, \
    .alfa_speed_max = 0, \
    .display_max_speed = 0, \
    .alfa_circle_square = 0, \
    .avg_speed = {0}, \
    .real_distance = {0}, \
    .time_hour = {0}, \
    .time_min = {0}, \
    .time_sec = {0}, \
    .this_run = {0}, \
    .message_nr = {0}, \
    .alfa_distance = {0}, \
    .alfa_count = 0, \
    .old_alfa_count = 0, \
    .record = 0 \
}

/**
 * @brief GPS context structure
*/
typedef struct gps_context_s {
    struct gps_data_s Ublox;          // create an object storing GPS_data !
    struct GPS_SAT_info Ublox_Sat;    // create an object storing GPS_SAT info !
    struct gps_speed_by_dist_s M100;
    struct gps_speed_by_dist_s M250;
    struct gps_speed_by_dist_s M500;
    struct gps_speed_by_dist_s M1852;
    struct gps_speed_by_time_s S2;
    struct gps_speed_by_time_s s2;         // for  stats GPIO_12 screens, reset possible !!
    struct gps_speed_by_time_s S10;
    struct gps_speed_by_time_s s10;        // for  stats GPIO_12 screens, reset possible !!
    struct gps_speed_by_time_s S1800;
    struct gps_speed_by_time_s S3600;
    struct gps_speed_alfa_s A250;
    struct gps_speed_alfa_s A500;
    struct gps_speed_alfa_s a500;     // for  Alfa stats GPIO_12 screens, reset possible !!
    
    bool Gps_fields_OK;
    
    uint32_t run_count;
    uint32_t old_run_count;
    uint8_t GPS_delay;

    uint32_t time_out_gps_msg;
    uint32_t interval_gps_msg;

    //uint32_t last_gps_msg;
    uint32_t old_nav_pvt_itow;
    
    uint8_t next_gpy_full_frame;
    
    int32_t gps_speed;
    uint32_t start_logging_millis;

    float alfa_window;
    float alfa_exit;
    float Mean_heading;
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
    uint32_t next_time_sync;
    // float calibration_speed;
    bool ubx_restart_requested;
} gps_context_t;

#define CONTEXT_GPS_DEFAULT_CONFIG { \
    .Ublox = GPS_DATA_DEFAULT_CONFIG(), \
    .Ublox_Sat = GPS_SAT_INFO_DEFAULT_CONFIG(), \
    .M100 = GPS_SPEED_DEFAULT_CONFIG(), \
    .M250 = GPS_SPEED_DEFAULT_CONFIG(), \
    .M500 = GPS_SPEED_DEFAULT_CONFIG(), \
    .M1852 = GPS_SPEED_DEFAULT_CONFIG(), \
    .S2 = GPS_TIME_DEFAULT_CONFIG, \
    .s2 = GPS_TIME_DEFAULT_CONFIG, \
    .S10 = GPS_TIME_DEFAULT_CONFIG, \
    .s10 = GPS_TIME_DEFAULT_CONFIG, \
    .S1800 = GPS_TIME_DEFAULT_CONFIG, \
    .S3600 = GPS_TIME_DEFAULT_CONFIG, \
    .A250 = ALFA_SPEED_DEFAULT_CONFIG(), \
    .A500 = ALFA_SPEED_DEFAULT_CONFIG(), \
    .a500 = ALFA_SPEED_DEFAULT_CONFIG(), \
    .Gps_fields_OK = false, \
    .run_count = 0, \
    .old_run_count = 0, \
    .GPS_delay = 0, \
    .time_out_gps_msg = 0, \
    .interval_gps_msg = 0, \
    .old_nav_pvt_itow = 0, \
    .next_gpy_full_frame = 0, \
    .gps_speed = 0, \
    .alfa_window = 0, \
    .alfa_exit = 0, \
    .Mean_heading = 0, \
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
    .next_time_sync = 0, \
    .ubx_restart_requested = false \
}

/** 
 * @brief Initialize the GPS data structure
 * 
 * @param gps_data_s* Pointer to the GPS data structure
 * @return struct gps_data_s* Pointer to the GPS data structure
*/
struct gps_data_s * init_gps_data(struct gps_data_s*);

/** 
 * @brief Initialize the GPS speed by time structure
 * 
 * @param gps_speed_by_time_s* Pointer to the GPS speed by time structure
 * @param uint16_t tijdvenster Time window in seconds
 * @return struct gps_speed_by_time_s* Pointer to the GPS speed by time structure
*/
struct gps_speed_by_time_s * init_gps_time(struct gps_speed_by_time_s*, uint16_t tijdvenster); // description of the constructor

/** 
 * @brief Update the speed by time structure
 * 
 * @param struct gps_context_s * Pointer to the GPS context structure
 * @param struct gps_speed_by_time_s* Pointer to the GPS speed by time structure
 * @return float Updated speed
*/
float update_speed(struct gps_context_s * context, struct gps_speed_by_time_s*);//update function

/** 
 * @brief Reset the time statistics
 * 
 * @param struct gps_speed_by_time_s* Pointer to the GPS speed by time structure
*/
void reset_time_stats(struct gps_speed_by_time_s*); //reset all stats to 0

/** 
 * @brief Initialize the GPS satellite information structure
 * 
 * @param struct GPS_SAT_info* Pointer to the GPS satellite information structure
 * @return struct GPS_SAT_info* Pointer to the GPS satellite information structure
*/
struct GPS_SAT_info* init_gps_sat_info(struct GPS_SAT_info*);

/** 
 * @brief Push the GPS satellite information
 * 
 * @param struct GPS_SAT_info* Pointer to the GPS satellite information structure
 * @param struct nav_sat_s * Pointer to the NAV SAT structure
*/
void push_gps_sat_info(struct GPS_SAT_info*, struct nav_sat_s * nav_sat);

/** 
 * @brief sort the run data
*/
void sort_run(double a[], uint8_t hour[], uint8_t minute[], uint8_t seconde[], uint16_t mean_cno[], uint8_t max_cno[], uint8_t min_cno[], uint8_t nrSats[], uint32_t runs[], uint8_t size);

/** 
 * @brief sort the run data for alfa
*/
void sort_run_alfa(double a[], int32_t dis[], uint32_t message[], uint8_t hour[], uint8_t minute[], uint8_t seconde[], uint32_t runs[], uint32_t samples[], uint8_t size);

/** 
 * @brief Initialize the GPS speed by distance structure
 * 
 * @param struct gps_speed_by_dist_s* Pointer to the GPS speed by distance structure
 * @param uint16_t afstand Length in m where avg speed is calculated
 * @return struct gps_speed_by_dist_s* Pointer to the GPS speed by distance structure
*/
struct gps_speed_by_dist_s * init_gps_speed(struct gps_speed_by_dist_s *, uint16_t afstand); // description of the constructor, length in m where avg speed is calculated

/** 
 * @brief Push the GPS data
*/
int push_gps_data(struct gps_context_s * context, struct gps_data_s*, float latitude, float longitude, int32_t gSpeed); // hier wordt de gps data in de buffer geplaatst


/** 
 * @brief New run detection
*/
uint32_t new_run_detection(struct gps_context_s * context, float actual_heading, float S2_speed);

/** 
 * @brief Reset the distance statistics
*/
void reset_distance_stats(struct gps_speed_by_dist_s *me);

/** 
 * @brief Update the distance statistics
*/
double update_distance(struct gps_context_s *context, struct gps_speed_by_dist_s *);

/** 
 * @brief Initialize the alpha speed structure
 * 
 * @param struct gps_speed_alfa_s* Pointer to the alpha speed structure
 * @param int alfa_radius Radius of the circle in m
 * @return struct gps_speed_alfa_s* Pointer to the alpha speed structure
 */
struct gps_speed_alfa_s * init_alfa_speed(struct gps_speed_alfa_s*,int alfa_radius);//constructor

/** 
 * @brief Update the alpha speed
 * 
 * @param struct gps_context_s * Pointer to the GPS context structure
 * @param struct gps_speed_alfa_s* Pointer to the alpha speed structure
 * @param struct gps_speed_by_dist_s * Pointer to the GPS speed by distance structure
 * @return float Updated alpha speed
 */
float update_alfa_speed(struct gps_context_s * context, struct gps_speed_alfa_s*, struct gps_speed_by_dist_s * M);   //update function every GPS-sample

/** 
 * @brief Reset the alpha statistics
 * 
 * @param struct gps_speed_alfa_s* Pointer to the alpha speed structure
 */
void reset_alfa_stats(struct gps_speed_alfa_s*); //reset all stats to 0

/** 
 * @brief Calculate the distance between two points
 * 
 * @param float long_act Longitude of the actual position
 * @param float lat_act Latitude of the actual position
 * @param float long_1 Longitude of the first point
 * @param float lat_1 Latitude of the first point
 * @param float long_2 Longitude of the second point
 * @param float lat_2 Latitude of the second point
 * @return float Distance between the two points
 */
float dis_point_line(float long_act,float lat_act,float long_1,float lat_1,float long_2,float lat_2);

/** 
 * @brief Calculate the alpha indicator
 * 
 * @param struct gps_context_s * Pointer to the GPS context structure
 * @param float actual_heading Actual heading
 * @return float Alpha indicator
 */
float alfa_indicator(struct gps_context_s * context, float actual_heading);

/** 
 * @brief Initialize the GPS context fields
 * 
 * @param struct gps_context_s * Pointer to the GPS context structure
 */
void init_gps_context_fields(struct gps_context_s * ctx);

/*
@returns last speed in mm/s if average_records is 0, the last speed is returned
 */
int32_t gps_last_speed_smoothed(uint8_t average_records);

/**
 *@brief returns the last speed in mm/s over average_records
@returns last speed in mm/s over average_records if average_records is 0, the last sec speed is returned
 */
int32_t gps_last_sec_speed_smoothed(uint8_t average_records);

void gps_log_nav_mode_change(gps_context_t *context, uint8_t changed);

#ifdef __cplusplus
}
#endif
#endif
