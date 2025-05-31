#ifndef DB99F2E7_B596_4059_B6AF_FAD2A14CD6A0
#define DB99F2E7_B596_4059_B6AF_FAD2A14CD6A0

#ifdef __cplusplus
extern "C" {
#endif
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

struct gps_context_s;

int log_get_fd(const struct gps_context_s * context, uint8_t file);
size_t log_write(const struct gps_context_s * context, uint8_t file, const void * msg, size_t len);
int log_close(const struct gps_context_s * context, uint8_t file);
int log_fsync(const struct gps_context_s * context, uint8_t file);
void printFile(const char *filename);

struct gps_context_s;
extern struct gps_context_s * gps;

#define NOGPY (log_get_fd((context), SD_GPY)==-1)
#define NOGPX (log_get_fd((context), SD_GPX)==-1)
#define NOSBP (log_get_fd((context), SD_SBP)==-1)
#define NOTXT (log_get_fd((context), SD_TXT)==-1)

#define WRITEGPX(msg, len) log_write((context), SD_GPX, (msg), (len))
#define WRITEGPY(msg, len) log_write((context), SD_GPY, (msg), (len))
#define WRITEUBX(msg, len) log_write((context), SD_UBX, (msg), (len))
#define WRITESBP(msg, len) log_write((context), SD_SBP, (msg), (len))
#define WRITETXT(msg, len) log_write((context), SD_TXT, (msg), (len))

#define GET_FD(f) (context->log_config->filefds[f])

#include "sdkconfig.h"
#if (defined(CONFIG_LOGGER_USE_GLOBAL_LOG_LEVEL) && CONFIG_LOGGER_GLOBAL_LOG_LEVEL < CONFIG_GPS_LOG_LEVEL)
#define C_LOG_LEVEL CONFIG_LOGGER_GLOBAL_LOG_LEVEL
#else
#define C_LOG_LEVEL CONFIG_GPS_LOG_LEVEL
#endif
#include "common_log.h"

#define MIN_numSV_FIRST_FIX 5      // before start logging, changed from 4 to 5 7.1/2023
#define MAX_Sacc_FIRST_FIX 2       // before start logging
#define MIN_numSV_GPS_SPEED_OK  4  // minimum number of satellites for calculating speed, otherwise
#define MAX_Sacc_GPS_SPEED_OK  1   // max Sacc value for calculating speed, otherwise 0
#define MAX_GPS_SPEED_OK  60       // max speed in m/s for calculating speed, otherwise 0 - [60 m/s = 216 km/h]

#define TIME_DELAY_FIRST_FIX 10       // 10 navpvt messages before start logging

#define SPEED_DETECTION_MIN 4000  // min average speed over 2s for new run detection (mm/s)
#define STANDSTILL_DETECTION_MAX  1000  // max average speed over 2s voor stand still detection (mm/s)

struct gps_point_s;
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
float dist_point_line(struct gps_point_s * act,  struct gps_point_s * p1, struct gps_point_s * p2);

#ifdef __cplusplus
}
#endif
#endif /* DB99F2E7_B596_4059_B6AF_FAD2A14CD6A0 */
