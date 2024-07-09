#ifndef A2745060_E1EF_4AED_A6C1_CC005166CAAB
#define A2745060_E1EF_4AED_A6C1_CC005166CAAB

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PATH_MAX_CHAR_SIZE 64
#define MAC2STR2(a) *(a), *((a)+1), *((a)+2), *((a)+3), *((a)+4), *((a)+5)

typedef enum {
    SD_TXT = 0,
    SD_SBP,
    SD_UBX,
    SD_GPX,
    SD_GPY,
    SD_FD_END
} sd_fd_t;

struct logger_config_s;

typedef struct gps_log_file_config_s {
    
    // int ubxfile;                               // 32
    // int errorfile;                             // 64
    // int gpyfile;                               // 96
    // int sbpfile;                               // 128
    // int gpxfile;                               // 160
    
    char filename_NO_EXT[PATH_MAX_CHAR_SIZE];  // 64
    // char filenameERR[PATH_MAX_CHAR_SIZE];      // 128
    // char filenameUBX[PATH_MAX_CHAR_SIZE];      // 196
    // char filenameGPY[PATH_MAX_CHAR_SIZE];      // 254
    // char filenameSBP[PATH_MAX_CHAR_SIZE];      // 318
    // char filenameGPX[PATH_MAX_CHAR_SIZE];      // 392

    char filenames[SD_FD_END][PATH_MAX_CHAR_SIZE];     // 512
    int filefds[SD_FD_END];                     // 544
    uint8_t log_file_bits;
    uint8_t log_file_open_bits;

    struct logger_config_s *config;            // 400
    // 412
} gps_log_file_config_t;

#define GPS_LOG_DEFAULT_CONFIG() (gps_log_file_config_t){ \
    .filename_NO_EXT = {0}, \
    .filenames = { {0}, {0}, {0}, {0}, {0} }, \
    .filefds = { -1, -1, -1, -1, -1 }, \
    .log_file_bits = 0, \
    .log_file_open_bits = 0, \
    .config = NULL \
}

struct tm;
struct gps_data_s;
struct gps_speed_by_dist_s;
struct gps_speed_by_time_s;
struct gps_speed_alfa_s;
struct gps_context_s;
struct ubx_msg_s;

void log_err(const struct gps_context_s * context, const char *message);
void open_files(struct gps_context_s * context);
void close_files(struct gps_context_s *context);
void flush_files(const struct gps_context_s *context);
void log_to_file(struct gps_context_s * context); 

void model_info(const struct gps_context_s * context, int model);
void session_info(const struct gps_context_s *context, struct gps_data_s * G);
void session_results_m(const struct gps_context_s *context, struct gps_speed_by_dist_s * M, float calibration_speed);
void session_results_s(const struct gps_context_s *context, struct gps_speed_by_time_s * S, float calibration_speed);
void session_results_alfa(const struct gps_context_s *context, struct gps_speed_alfa_s * A, struct gps_speed_by_dist_s * M, float calibration_speed);

esp_err_t log_config_add_config(gps_log_file_config_t * log, struct logger_config_s *config);
gps_log_file_config_t *log_config_new();
void log_config_delete(gps_log_file_config_t *log);

void log_ubx(struct gps_context_s *context, struct ubx_msg_s * ubxMessage, bool log_ubx_nav_sat);
esp_err_t save_log_file_bits(struct gps_context_s *config, uint8_t *log_file_bits);
#ifdef __cplusplus
}
#endif

#endif /* A2745060_E1EF_4AED_A6C1_CC005166CAAB */
