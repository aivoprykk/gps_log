#ifndef A2745060_E1EF_4AED_A6C1_CC005166CAAB
#define A2745060_E1EF_4AED_A6C1_CC005166CAAB


#ifdef __cplusplus
extern "C" {
#endif

#include "sdkconfig.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include <esp_err.h>
#include "esp_vfs.h"

#include "logger_common.h"
#include "config_gps.h"

#define PATH_MAX_CHAR_SIZE 64
#define MAC2STR2(a) *(a), *((a)+1), *((a)+2), *((a)+3), *((a)+4), *((a)+5)

#define VAL_A(a) {0},
#define VAL_B(a) -1,
#define ENUM_A(a) sd_ ## a,

/// copy from config_gps.h
#if defined (CONFIG_GPS_LOG_GPY)
#define CFG_GPS_USER_FILE_ITEMS_LL(l) l(log_gpy)
#else
#define CFG_GPS_USER_FILE_ITEMS_LL(l)
#endif
#define CFG_GPS_FILE_ITEMS(l) l(log_txt) l(log_sbp) l(log_ubx) l(log_gpx) CFG_GPS_USER_FILE_ITEMS_LL(l)
/// end copy from config_gps.h

#define SD_FILE_DEFAULT_CONFIG { CFG_GPS_FILE_ITEMS(VAL_A) }
#define SD_FILE_DEFAULT_FDS { CFG_GPS_FILE_ITEMS(VAL_B) }

typedef enum {
    CFG_GPS_FILE_ITEMS(ENUM_A)
    sd_log_end
} sd_fd_t;

struct logger_config_s;

typedef struct gps_log_file_config_s {    
    char filename_base[PATH_MAX_CHAR_SIZE];  // 64

    char filenames[sd_log_end][PATH_MAX_CHAR_SIZE];     // 512
    int filefds[sd_log_end];                     // 544

    char base_path[ESP_VFS_PATH_MAX + 1];

    // 412
} gps_log_file_config_t;

#define GPS_LOG_DEFAULT_CONFIG() (gps_log_file_config_t){ \
    .filename_base = {0}, \
    .filenames = SD_FILE_DEFAULT_CONFIG, \
    .filefds = SD_FILE_DEFAULT_FDS, \
}

extern gps_log_file_config_t log_config;

struct tm;
struct gps_data_s;
struct gps_context_s;
struct ubx_msg_s;

// Check if GPS log partition is available and ready for logging
bool gps_log_partition_is_available(void);

// Register/unregister VFS event handlers to track partition availability
void gps_log_register_vfs_handler(void);
void gps_log_unregister_vfs_handler(void);

gps_log_file_config_t *gps_log_config_init(void);
void log_err(const struct gps_context_s * context, const char *message);
void open_files(struct gps_context_s * context);
void close_files(struct gps_context_s *context);
void flush_files(const struct gps_context_s *context);
void log_to_file(struct gps_context_s * context); 
bool log_files_opened(struct gps_context_s * context);

// esp_err_t log_config_add_config(gps_log_file_config_t * log, struct logger_config_s *config);
// gps_log_file_config_t *log_config_new();
// void log_config_delete(gps_log_file_config_t *log);

void log_ubx(struct gps_context_s *context, struct ubx_msg_s * ubxMessage, bool log_ubx_nav_sat);
// esp_err_t save_log_file_bits(struct gps_context_s *config, uint8_t *log_file_bits);

#ifdef __cplusplus
}
#endif

#endif /* A2745060_E1EF_4AED_A6C1_CC005166CAAB */
