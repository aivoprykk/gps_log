#ifndef C814F046_3E50_4544_8BE1_20BE96FD60CD
#define C814F046_3E50_4544_8BE1_20BE96FD60CD

#ifdef __cplusplus
extern "C" {
#endif

#include "sdkconfig.h"
#include <stdint.h>
#include <stdbool.h>

#define NAV_SAT_BUFFER CONFIG_GPS_NAV_SAT_BUFFER_SIZE
#define FILTER_MIN_SATS 5
#define FILTER_MAX_sACC 2

/**
 * @brief GPS satellite information structure
*/
struct sat_info_s {
      uint16_t Mean_cno[NAV_SAT_BUFFER]; // Mean cno value from all sats used in nav
      uint8_t Max_cno[NAV_SAT_BUFFER];   // Max cno value from all sats used in nav      
      uint8_t Min_cno[NAV_SAT_BUFFER];   // Min cno value from all sats used in nav
      uint8_t numSV[NAV_SAT_BUFFER];     // Nr of sats used in nav 
      uint16_t Mean_mean_cno ;           // Mean over last x NAV_SAT messages
      uint8_t Mean_max_cno ;             // Mean over last x NAV_SAT messages
      uint8_t Mean_min_cno ;             // Mean over last x NAV_SAT messages
      uint8_t Mean_numSV ;               // Mean over last x NAV_SAT messages
 }; // struct size is 40 bytes

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
struct gps_sat_info_s {
    struct sat_info_s sat_info;
    uint32_t index_sat_info;
    uint16_t mean_cno;
    uint8_t max_cno;
    uint8_t min_cno;
    uint8_t nr_sats;
}; // struct size is 56 bytes

#define GPS_SAT_INFO_DEFAULT_CONFIG() { \
    .sat_info = SAT_INFO_DEFAULT_CONFIG(), \
    .index_sat_info = 0, \
    .mean_cno = 0, \
    .max_cno = 0, \
    .min_cno = 0, \
    .nr_sats = 0 \
}

/** 
 * @brief Initialize the GPS satellite information structure
 * 
 * @param struct gps_sat_info* Pointer to the GPS satellite information structure
 * @return struct gps_sat_info* Pointer to the GPS satellite information structure
*/
struct gps_sat_info_s* init_gps_sat_info(struct gps_sat_info_s*);

struct nav_sat_s;

/** 
 * @brief Push the GPS satellite information
 * 
 * @param struct gps_sat_info* Pointer to the GPS satellite information structure
 * @param struct nav_sat_s * Pointer to the NAV SAT structure
*/
void push_gps_sat_info(struct gps_sat_info_s*, struct nav_sat_s *);

#ifdef __cplusplus
}
#endif

#endif /* C814F046_3E50_4544_8BE1_20BE96FD60CD */
