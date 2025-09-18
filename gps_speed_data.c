#include "log_private.h"
#include "gps_speed_data.h"
#include "ubx.h"
#include <math.h>
#include <stdarg.h>
#include "logger_buffer_pool.h"

static const char *TAG = "gps_speed";

// GPS metrics error logging configuration
#ifdef CONFIG_GPS_SPEED_ERROR_LOGGING
#define GPS_SPEED_ERROR_LOGGING_ENABLED 1
#else
#define GPS_SPEED_ERROR_LOGGING_ENABLED 0
#endif

// GPS error type definitions (always available for type safety)
typedef enum {
    GPS_ERROR_MEMORY_ALLOC_FAIL = 0,
    GPS_ERROR_NULL_HANDLE,
    GPS_ERROR_INIT_VALIDATION_FAIL,
    GPS_ERROR_CRITICAL_INIT_FAIL,
    GPS_ERROR_CRITICAL_BOUNDS_ERROR,
    GPS_ERROR_CRITICAL_INIT_SUMMARY,
    GPS_ERROR_TYPE_COUNT
} gps_error_type_t;

typedef enum {
    GPS_METRIC_TIME = 0,
    GPS_METRIC_DISTANCE,
    GPS_METRIC_ALFA,
    GPS_METRIC_SPEED_METRICS_ARRAY,
    GPS_METRIC_GPS_SPEED_SYSTEM,
    GPS_METRIC_COUNT
} gps_metric_name_t;

#if GPS_SPEED_ERROR_LOGGING_ENABLED

// Error type lookup table to reduce string duplication
static const char* const gps_error_type_strings[GPS_ERROR_TYPE_COUNT] = {
    "MEMORY_ALLOC_FAIL",
    "NULL_HANDLE_ERROR", 
    "INIT_VALIDATION_FAIL",
    "CRITICAL_INIT_FAIL",
    "CRITICAL_BOUNDS_ERROR",
    "CRITICAL_INIT_SUMMARY"
};

// Metric name lookup table
static const char* const gps_metric_name_strings[GPS_METRIC_COUNT] = {
    "TIME_METRIC",
    "DISTANCE_METRIC",
    "ALFA_METRIC", 
    "SPEED_METRICS_ARRAY",
    "GPS_SPEED_SYSTEM"
};

// Helper function to format error details using buffer pool with lookup tables
static void log_gps_error_with_details(gps_error_type_t error_type, gps_metric_name_t metric_name, const char* format, ...) {
    // Only log to file if GPS context and TXT file are available
    extern struct gps_context_s *gps;
    if (!gps || !gps->files_opened || !GETBIT(gps->log_config->log_file_bits, SD_TXT) || GET_FD(SD_TXT) <= 0) {
        return;
    }
    
    // Bounds check for lookup tables
    if (error_type >= GPS_ERROR_TYPE_COUNT || metric_name >= GPS_METRIC_COUNT) {
        ELOG(TAG, "Invalid error type or metric name in GPS error logging");
        return;
    }
    
    // Use buffer pool for error details formatting
    logger_buffer_handle_t detail_buffer = {0};
    if (!logger_buffer_pool_is_initialized() || 
        logger_buffer_pool_alloc(LOGGER_BUFFER_SMALL, LOGGER_BUFFER_USAGE_GPS_DATA, &detail_buffer, pdMS_TO_TICKS(100)) != ESP_OK) {
        // Fallback to simple error logging without details
        log_gps_metrics_error_to_file(gps_error_type_strings[error_type], gps_metric_name_strings[metric_name], "Buffer pool unavailable for error details");
        return;
    }
    
    char *details_str = (char*)detail_buffer.buffer;
    va_list args;
    va_start(args, format);
    vsnprintf(details_str, detail_buffer.size, format, args);
    va_end(args);
    
    log_gps_metrics_error_to_file(gps_error_type_strings[error_type], gps_metric_name_strings[metric_name], details_str);
    
    // Release the buffer
    logger_buffer_pool_free(&detail_buffer);
}

// Simple error logging using lookup tables
static void log_gps_error_simple(gps_error_type_t error_type, gps_metric_name_t metric_name, const char* details) {
    if (error_type >= GPS_ERROR_TYPE_COUNT || metric_name >= GPS_METRIC_COUNT) {
        ELOG(TAG, "Invalid error type or metric name in GPS error logging");
        return;
    }
    log_gps_metrics_error_to_file(gps_error_type_strings[error_type], gps_metric_name_strings[metric_name], details);
}

// Error logging function to write GPS metrics errors to TXT file using buffer pool
static void log_gps_metrics_error_to_file(const char* error_type, const char* metric_name, const char* details) {
    // Only log to file if GPS context and TXT file are available
    extern struct gps_context_s *gps;
    if (!gps || !gps->files_opened || !GETBIT(gps->log_config->log_file_bits, SD_TXT) || GET_FD(SD_TXT) <= 0) {
        return;
    }
    
    // Use buffer pool for error message - much more efficient than static global buffer
    logger_buffer_handle_t error_buffer = {0};
    if (!logger_buffer_pool_is_initialized() || 
        logger_buffer_pool_alloc(LOGGER_BUFFER_SMALL, LOGGER_BUFFER_USAGE_GPS_DATA, &error_buffer, pdMS_TO_TICKS(100)) != ESP_OK) {
        ELOG(TAG, "Failed to get buffer for GPS error logging");
        return;
    }
    
    char *error_msg = (char*)error_buffer.buffer;
    uint64_t timestamp = esp_timer_get_time() / 1000; // Convert to milliseconds
    
    int len = snprintf(error_msg, error_buffer.size, 
        "[GPS_METRICS_ERROR] Time:%llu Type:%s Metric:%s Details:%s\n",
        timestamp, error_type, metric_name ? metric_name : "UNKNOWN", details ? details : "No details");
    
    if (len > 0 && len < error_buffer.size) {
        WRITETXT(error_msg, len);
    }
    ELOG(TAG, "%s", error_msg);
    
    // Release the buffer back to the pool
    logger_buffer_pool_free(&error_buffer);
}
#else
// Dummy function when logging is disabled
static inline void log_gps_metrics_error_to_file(const char* error_type, const char* metric_name, const char* details) {
    (void)error_type; (void)metric_name; (void)details; // Suppress unused warnings
}
static inline void log_gps_error_with_details(gps_error_type_t error_type, gps_metric_name_t metric_name, const char* format, ...) {
    (void)error_type; (void)metric_name; (void)format; // Suppress unused warnings
}
static inline void log_gps_error_simple(gps_error_type_t error_type, gps_metric_name_t metric_name, const char* details) {
    (void)error_type; (void)metric_name; (void)details; // Suppress unused warnings
}
#endif

// Pre-calculated constants for performance optimization
static const float DEG2RAD_CONST = M_PI / 180.0f;
static const float RAD2DEG_CONST = 180.0f / M_PI;
static const float EARTH_RADIUS_M_CONST = 6371000.0f;
static const float SPEED_THRESHOLD_MIN = 3000.0f; // 3 m/s in mm/s
static const float SPEED_THRESHOLD_BAR = 5000.0f; // 5 m/s in mm/s
static const float ALFA_THRESHOLD = 50.0f; // 50 meters
static const uint32_t TIME_WINDOW_SAMPLES_2S = 2; // Will be multiplied by sample_rate
static const uint32_t TIME_WINDOW_SAMPLES_15S = 15;

static const gps_speed_metrics_cfg_t initial_speed_metrics_sets[] = {
    { GPS_SPEED_TYPE_TIME, 2,    },
    { GPS_SPEED_TYPE_TIME, 10,   },
    { GPS_SPEED_TYPE_TIME, 1800, },
    { GPS_SPEED_TYPE_TIME, 3600, },
    { GPS_SPEED_TYPE_DIST, 100,  },
    { GPS_SPEED_TYPE_DIST|GPS_SPEED_TYPE_ALFA, 250,  },
    { GPS_SPEED_TYPE_DIST|GPS_SPEED_TYPE_ALFA, 500,  },
    { GPS_SPEED_TYPE_DIST, 1852, },
};

static inline uint32_t convert_distance_to_mm(int distance) {
    // Use original macro logic to ensure identical behavior
    return M_TO_MM(distance) * gps->ubx_device->rtc_conf->output_rate;
}

static inline gps_speed_t * gps_select_speed_instance(int num, uint8_t flags) {
    if(!gps->speed_metrics || num < 0 || num >= gps->num_speed_metrics) return NULL;
    gps_speed_metrics_desc_t *spd = &gps->speed_metrics[num];
    if ((flags & GPS_SPEED_TYPE_ALFA) && spd->handle.dist && spd->handle.dist->alfa)
        return &spd->handle.dist->alfa->speed;
    else if ((flags & GPS_SPEED_TYPE_DIST) && spd->handle.dist)
        return &spd->handle.dist->speed;
    else if ((flags == GPS_SPEED_TYPE_TIME) && spd->handle.time)
        return &spd->handle.time->speed;
    return NULL;
}

static gps_display_t * gps_get_time_display_struct(int set) {
    gps_speed_t *spd = gps_select_speed_instance(set, GPS_SPEED_TYPE_TIME);
    if(!spd) {
        WLOG(TAG, "[%s] Time speed instance %d not found", __func__, set);
        return NULL;
    }
    return &spd->display;
}
static gps_display_t * gps_get_alfa_display_struct(int set) {
    gps_speed_t *spd = gps_select_speed_instance(set, GPS_SPEED_TYPE_ALFA);
    if(!spd) {
        WLOG(TAG, "[%s] Alfa speed instance %d not found", __func__, set);
        return NULL;
    }
    return &spd->display;
}
static float gps_get_run_average_speed(int set, uint8_t type, int num) {
    gps_speed_t *spd = gps_select_speed_instance(set, type);
    return spd ? spd->runs[num].avg_speed : 0.0f;
}

static gps_tm_t * gps_get_run_time_struct(int set, uint8_t type, int num) {
    gps_speed_t *spd = gps_select_speed_instance(set, type);
    return spd ? &spd->runs[num].time : 0;
}

static float gps_get_cur_speed(int set, uint8_t type) {
    gps_speed_t *spd = gps_select_speed_instance(set, type);
    return spd ? spd->cur_speed : 0.0f;
}

static float gps_get_max_speed_value(int set, uint8_t type) {
    gps_speed_t *spd = gps_select_speed_instance(set, type);
    return spd ? spd->max_speed : 0.0f;
}

static float get_display_max_speed(int set, uint8_t type) {
    gps_speed_t *spd = gps_select_speed_instance(set, type);
    return spd ? spd->display.display_max_speed : 0.0f;
}

static float get_display_last_run_max_speed(int set, uint8_t type) {
    gps_speed_t *spd = gps_select_speed_instance(set, type);
    return spd ? spd->display.display_last_run_max_speed : 0.0f;
}

static float get_display_display_speed(int set, uint8_t type, int num) {
    gps_speed_t *spd = gps_select_speed_instance(set, type);
    return spd ? spd->display.display_speed[num] : 0.0f;
}

static bool get_display_record(int set, uint8_t type) {
    gps_speed_t *spd = gps_select_speed_instance(set, type);
    return spd ? spd->display.record : false;
}

gps_speed_op_t speed_ops = {
    &gps_get_run_average_speed,
    &gps_get_run_time_struct,
    &gps_get_cur_speed,
    &gps_get_max_speed_value,
    &get_display_max_speed,
    &get_display_last_run_max_speed,
    &get_display_display_speed,
    &get_display_record,
    &gps_get_time_display_struct,
    &gps_get_alfa_display_struct
};

esp_err_t gps_speed_metrics_add(const gps_speed_metrics_cfg_t *cfg, int pos) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s] idx: %d type: %d window: %d, max_metrics: %hu", __func__, pos, cfg->type, cfg->window, gps->num_speed_metrics);
#endif
    if (!gps->speed_metrics || pos < 0 || pos >= gps->num_speed_metrics) {
        ELOG(TAG, "[%s] Invalid speed metrics or position %d (max: %hu)", __func__, pos, gps->num_speed_metrics);
        return ESP_ERR_INVALID_ARG;
    }
    
    gps_speed_metrics_desc_t *desc = &gps->speed_metrics[pos];
#if (C_LOG_LEVEL < 3)
    if(desc->handle.time || desc->handle.dist) {
        WLOG(TAG, "[%s] Speed set %d already exists, skipping.", __func__, pos);
        return ESP_OK; // Already exists
    }
#endif
    desc->type = cfg->type;
    desc->window = cfg->window;
    uint16_t size = 0;
    
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s] Initializing metric %d: type=%d, window=%d", __func__, pos, cfg->type, cfg->window);
#endif

    // Optimize allocation with proper error checking
    if ((cfg->type & SPEED_TYPE_MASK) == GPS_SPEED_TYPE_TIME) { // time bit set
        if(!desc->handle.time) {
            if (!check_and_alloc_buffer((void **)&desc->handle.time, 1, sizeof(gps_speed_by_time_t), &size, MALLOC_CAP_DMA)) {
                ELOG(TAG, "[%s] Failed to allocate time buffer for metric %d", __func__, pos);
#if GPS_SPEED_ERROR_LOGGING_ENABLED
                log_gps_error_with_details(GPS_ERROR_MEMORY_ALLOC_FAIL, GPS_METRIC_TIME, 
                    "Time metric %d alloc failed, size=%zu", pos, sizeof(gps_speed_by_time_t));
#endif
                return ESP_ERR_NO_MEM;
            }
        }
        if (desc->handle.time) {
            init_gps_speed_by_time(desc->handle.time, cfg->window);
            desc->handle.time->speed.flags = GPS_SPEED_TYPE_TIME;
#if (C_LOG_LEVEL < 3)
            ILOG(TAG, "[%s] Time metric %d initialized successfully", __func__, pos);
#endif
        } else {
            ELOG(TAG, "[%s] Time handle is null after allocation for metric %d", __func__, pos);
#if GPS_SPEED_ERROR_LOGGING_ENABLED
            log_gps_error_with_details(GPS_ERROR_NULL_HANDLE, GPS_METRIC_TIME, 
                "Time handle null after alloc, metric %d", pos);
#endif
            return ESP_ERR_NO_MEM;
        }
    }
    else if ((cfg->type & (GPS_SPEED_TYPE_DIST | GPS_SPEED_TYPE_ALFA))) { // dist or alfa bit set
        if(!desc->handle.dist) {
            if (!check_and_alloc_buffer((void **)&desc->handle.dist, 1, sizeof(gps_speed_by_dist_t), &size, MALLOC_CAP_DMA)) {
                ELOG(TAG, "[%s] Failed to allocate distance buffer for metric %d", __func__, pos);
#if GPS_SPEED_ERROR_LOGGING_ENABLED
                log_gps_error_with_details(GPS_ERROR_MEMORY_ALLOC_FAIL, GPS_METRIC_DISTANCE, 
                    "Distance metric %d alloc failed, size=%zu", pos, sizeof(gps_speed_by_dist_t));
#endif
                return ESP_ERR_NO_MEM;
            }
        }
        if (desc->handle.dist) {
            init_gps_speed_by_distance(desc->handle.dist, cfg->window);
            desc->handle.dist->speed.flags = GPS_SPEED_TYPE_DIST;
            if ((cfg->type & GPS_SPEED_TYPE_ALFA)) { // check if alfa bit set
                if(!desc->handle.dist->alfa) {
                    // Allocate alfa speed instance if not already allocated
                    if (!check_and_alloc_buffer((void **)&desc->handle.dist->alfa, 1, sizeof(gps_speed_by_alfa_t), &size, MALLOC_CAP_DMA)) {
                        ELOG(TAG, "[%s] Failed to allocate alfa buffer for metric %d", __func__, pos);
#if GPS_SPEED_ERROR_LOGGING_ENABLED
                        log_gps_error_with_details(GPS_ERROR_MEMORY_ALLOC_FAIL, GPS_METRIC_ALFA, 
                            "Alfa metric %d alloc failed, size=%zu", pos, sizeof(gps_speed_by_alfa_t));
#endif
                        return ESP_ERR_NO_MEM;
                    }
                }
                if (desc->handle.dist->alfa) {
                    init_gps_speed_by_alfa(desc->handle.dist);
                    desc->handle.dist->speed.flags |= GPS_SPEED_TYPE_ALFA;
#if (C_LOG_LEVEL < 3)
                    ILOG(TAG, "[%s] Distance+Alfa metric %d initialized successfully", __func__, pos);
#endif
                } else {
                    ELOG(TAG, "[%s] Alfa handle is null after allocation for metric %d", __func__, pos);
                    return ESP_ERR_NO_MEM;
                }
            } else {
#if (C_LOG_LEVEL < 3)
                ILOG(TAG, "[%s] Distance metric %d initialized successfully", __func__, pos);
#endif
            }
        } else {
            ELOG(TAG, "[%s] Distance handle is null after allocation for metric %d", __func__, pos);
            return ESP_ERR_NO_MEM;
        }
    }
    return ESP_OK;
}

void gps_speed_metrics_check(const gps_speed_metrics_cfg_t *cfg, size_t num_sets) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s] num_sets: %zu, current: %hu", __func__, num_sets, gps->num_speed_metrics);
#endif
    if(num_sets > gps->num_speed_metrics){
        if (!check_and_alloc_buffer((void **)&gps->speed_metrics, num_sets, sizeof(gps_speed_metrics_desc_t), &gps->num_speed_metrics, MALLOC_CAP_DMA)) {
            ELOG(TAG, "[%s] Failed to allocate speed metrics array for %zu sets", __func__, num_sets);
#if GPS_SPEED_ERROR_LOGGING_ENABLED
            log_gps_error_with_details(GPS_ERROR_MEMORY_ALLOC_FAIL, GPS_METRIC_SPEED_METRICS_ARRAY, 
                "Failed to allocate main speed metrics array, num_sets=%zu, element_size=%zu", 
                num_sets, sizeof(gps_speed_metrics_desc_t));
#endif
            return;
        }
        if (gps->speed_metrics) {
#if (C_LOG_LEVEL < 3)
            ILOG(TAG, "[%s] Allocated %hu speed metrics, initializing %zu", __func__, gps->num_speed_metrics, num_sets);
#endif
            memset(gps->speed_metrics, 0, num_sets * sizeof(gps_speed_metrics_desc_t));
            for (int i = 0; i < num_sets; ++i) {
                esp_err_t err = gps_speed_metrics_add(&cfg[i], i);
                if (err != ESP_OK) {
                    ELOG(TAG, "[%s] Failed to add speed metric %d: %s", __func__, i, esp_err_to_name(err));
                    // Continue with other metrics even if one fails
                }
            }
        } else {
            ELOG(TAG, "[%s] Speed metrics array is null after allocation", __func__);
        }
    } else {
#if (C_LOG_LEVEL < 3)
        ILOG(TAG, "[%s] Speed metrics already initialized (%hu >= %zu)", __func__, gps->num_speed_metrics, num_sets);
#endif
    }
}

void gps_speed_metrics_init() {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    gps_speed_metrics_check(&initial_speed_metrics_sets[0], lengthof(initial_speed_metrics_sets));
    
    // Validate all metrics are properly initialized
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s] Validating %d speed metrics initialization...", __func__, lengthof(initial_speed_metrics_sets));
#endif
    bool initialization_failed = false;
    
    for (int i = 0; i < lengthof(initial_speed_metrics_sets); i++) {
        const gps_speed_metrics_cfg_t *cfg = &initial_speed_metrics_sets[i];
        if (i < gps->num_speed_metrics && gps->speed_metrics) {
            gps_speed_metrics_desc_t *desc = &gps->speed_metrics[i];
            bool valid = false;
            
            if ((cfg->type & SPEED_TYPE_MASK) == GPS_SPEED_TYPE_TIME) {
                valid = (desc->handle.time != NULL);
#if (C_LOG_LEVEL < 3)
                ILOG(TAG, "  Metric %d (TIME, %d): %s", i, cfg->window, valid ? "OK" : "FAILED");
#endif
                if (!valid) {
                    log_gps_error_simple(GPS_ERROR_INIT_VALIDATION_FAIL, GPS_METRIC_TIME, 
                        "Time metric handle is NULL after initialization");
                    initialization_failed = true;
                }
            } else if (cfg->type & (GPS_SPEED_TYPE_DIST | GPS_SPEED_TYPE_ALFA)) {
                valid = (desc->handle.dist != NULL);
                if (valid && (cfg->type & GPS_SPEED_TYPE_ALFA)) {
                    valid = (desc->handle.dist->alfa != NULL);
#if (C_LOG_LEVEL < 3)
                    ILOG(TAG, "  Metric %d (DIST+ALFA, %d): %s", i, cfg->window, valid ? "OK" : "FAILED");
#endif
                    if (!valid) {
                        log_gps_error_simple(GPS_ERROR_INIT_VALIDATION_FAIL, GPS_METRIC_ALFA, 
                            "Alfa handle is NULL in distance metric");
                        initialization_failed = true;
                    }
                } else if (valid) {
#if (C_LOG_LEVEL < 3)
                    ILOG(TAG, "  Metric %d (DIST, %d): %s", i, cfg->window, "OK");
#endif
                } else {
#if (C_LOG_LEVEL < 3)
                    ILOG(TAG, "  Metric %d (DIST, %d): %s", i, cfg->window, "FAILED");
#endif
                    log_gps_error_simple(GPS_ERROR_INIT_VALIDATION_FAIL, GPS_METRIC_DISTANCE, 
                        "Distance metric handle is NULL after initialization");
                    initialization_failed = true;
                }
            }
            
            if (!valid) {
                ELOG(TAG, "CRITICAL: Speed metric %d failed to initialize properly!", i);
#if GPS_SPEED_ERROR_LOGGING_ENABLED
                log_gps_error_with_details(GPS_ERROR_CRITICAL_INIT_FAIL, GPS_METRIC_SPEED_METRICS_ARRAY, 
                    "Metric index %d failed validation", i);
#endif
            }
        } else {
            ELOG(TAG, "CRITICAL: Speed metric %d is out of bounds or array is null!", i);
#if GPS_SPEED_ERROR_LOGGING_ENABLED
            log_gps_error_with_details(GPS_ERROR_CRITICAL_BOUNDS_ERROR, GPS_METRIC_SPEED_METRICS_ARRAY, 
                "Metric index %d out of bounds", i);
#endif
            initialization_failed = true;
        }
    }
    
    if (initialization_failed) {
        ELOG(TAG, "GPS SPEED METRICS INITIALIZATION FAILED - ERRORS LOGGED TO TXT FILE");
#if GPS_SPEED_ERROR_LOGGING_ENABLED
        log_gps_error_simple(GPS_ERROR_CRITICAL_INIT_SUMMARY, GPS_METRIC_GPS_SPEED_SYSTEM, 
            "One or more GPS speed metrics failed to initialize properly");
#endif
    } else {
#if (C_LOG_LEVEL < 3)
        ILOG(TAG, "All GPS speed metrics successfully validated and initialized");
#endif
    }
}

void gps_speed_metrics_free(void) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    if (!gps->speed_metrics || gps->num_speed_metrics == 0) {
        return;
    }
    
    for (int i = 0; i < gps->num_speed_metrics; ++i) {
        if (gps->speed_metrics[i].type == GPS_SPEED_TYPE_TIME) {
            if (gps->speed_metrics[i].handle.time) {
                unalloc_buffer((void **)&gps->speed_metrics[i].handle.time);
            }
        } else if (gps->speed_metrics[i].type & (GPS_SPEED_TYPE_DIST | GPS_SPEED_TYPE_ALFA)){
            if (gps->speed_metrics[i].handle.dist) {
                if ((gps->speed_metrics[i].type & GPS_SPEED_TYPE_ALFA) && gps->speed_metrics[i].handle.dist->alfa) {
                    unalloc_buffer((void **)&gps->speed_metrics[i].handle.dist->alfa);
                }
                unalloc_buffer((void **)&gps->speed_metrics[i].handle.dist);
            }
        }
    }
    unalloc_buffer((void **)&gps->speed_metrics);
    gps->speed_metrics = NULL;
    gps->num_speed_metrics = 0;
}

void gps_speed_metrics_update(void) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s] sets: %hu", __func__, gps->num_speed_metrics);
#endif
    if (!gps->speed_metrics || gps->num_speed_metrics == 0) {
        return;
    }
    
    // Cache the metrics count to avoid repeated memory access
    const uint8_t num_metrics = gps->num_speed_metrics;
    gps_speed_metrics_desc_t *metrics = gps->speed_metrics;
    
    // Process all metrics in a single loop for better cache locality
    for(uint8_t i = 0; i < num_metrics; i++) {
        const uint8_t type = metrics[i].type;
        if (type == GPS_SPEED_TYPE_TIME && metrics[i].handle.time) {
            update_speed_by_time(metrics[i].handle.time);
        } else if ((type & (GPS_SPEED_TYPE_DIST | GPS_SPEED_TYPE_ALFA)) && metrics[i].handle.dist) {
            update_speed_by_distance(metrics[i].handle.dist);
            update_speed_by_alfa(metrics[i].handle.dist);
        }
    }
    gps_update_max_speed();
}

void refresh_gps_speeds_by_distance(void) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    for(uint8_t i = 0, j = gps->num_speed_metrics; i < j; i++) {
        if (!(gps->speed_metrics[i].type  & (GPS_SPEED_TYPE_DIST))) continue;
        if (gps->speed_metrics[i].window == 0) continue; // skip zero window sets
        struct gps_speed_by_dist_s *spd = gps->speed_metrics[i].handle.dist;
        if(spd)
            spd->distance_window_raw = convert_distance_to_mm(spd->distance_window);
    }
}

static inline void store_time(gps_run_t *run) {
    struct tm tms;
    get_local_time(&tms);
    run->time.hour = (uint8_t)tms.tm_hour;
    run->time.minute = (uint8_t)tms.tm_min;
    run->time.second = (uint8_t)tms.tm_sec;
}

void gps_update_max_speed(void) {
    if(gps->gps_speed > gps->max_speed.avg_speed) {
        gps->max_speed.avg_speed = gps->gps_speed;
        store_time(&gps->max_speed);
        gps->max_speed.nr = gps->run_count;
    }
}

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
    // printf("speed_alfa: %.02f, ", me->speed_alfa);
    printf("max_speed: %.02f, ", me->max_speed);
    // printf("avg_5runs: %.02f\n", me->avg_5runs);
    for (i = 0; i < j; i++) {
        printf(" %hhu ", i); gps_run_printf(&me->runs[i]);
    }
    gps_display_printf(&me->display);
    printf(" } ==\n");
    return ESP_OK;
}
#endif


static void record_last_run(gps_speed_t * speed, uint16_t actual_run) {
    // printf("[%s] %.01f %hu %hu\n", __func__, speed->max_speed, actual_run, speed->display.nr_display_last_run);
    if ((actual_run != speed->display.nr_display_last_run) && (speed->max_speed > SPEED_THRESHOLD_MIN)) { // 3m/s in mm/s
        speed->display.nr_display_last_run = actual_run;
        speed->display.display_last_run_max_speed = 0;
    } 
    else if (speed->display.display_last_run_max_speed < speed->max_speed) {
        speed->display.display_last_run_max_speed = speed->max_speed;
    }
}

#if defined(MUTABLE_RUNS)
static inline void reset_runs_avg(gps_run_t runs[]) {
    // printf("[%s]\n", __func__);
    memset(runs, 0, NUM_OF_SPD_ARRAY_SIZE * sizeof(gps_run_t));
}
#endif

static inline void reset_display_speed(float * arr) {
    // printf("[%s]\n", __func__);
    memset(arr, 0, NUM_OF_SPD_ARRAY_SIZE * sizeof(float));
}

// static void update_avg_5runs(gps_speed_t * speed, bool mode) {
//     // printf("[%s]\n", __func__);
//     speed->avg_5runs = speed->runs[(mode ? 0 : 5)].avg_speed;
//     for(uint8_t i = 6, j = mode ? 9 : NUM_OF_SPD_ARRAY_SIZE; i < j; i++) speed->avg_5runs += speed->runs[i].avg_speed;
//     speed->avg_5runs /= 5;
// }

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
    if (speed->max_speed > speed->RUNS_FOR_DISPLAY[IDX_OF_SPD_ARRAY_MIN_SPD].avg_speed) {
        refresh_display_speeds(&speed->display, speed->RUNS_FOR_DISPLAY, speed->max_speed);
        ret = 1;
    }
    if (speed->max_speed > speed->RUNS_FOR_DISPLAY[IDX_OF_SPD_ARRAY_MAX_SPD].avg_speed) {
        speed->display.display_max_speed = speed->max_speed;  // update on the fly, that's not correct here !!!
        speed->display.record = 1;
        *record = 1;
        ret = 2;
    }
    else
        speed->display.display_max_speed = speed->RUNS_FOR_DISPLAY[IDX_OF_SPD_ARRAY_MAX_SPD].avg_speed;
    return ret;
}

static void reset_last_run_speeds(gps_speed_t * speed) {
    // update_avg_5runs(speed, false);  // calculate the average of the last 5 best runs
    speed->max_speed = 0;
    memset(&speed->runs[0], 0, sizeof(gps_run_t));
#if defined(MUTABLE_RUNS)
    memset(&speed->runs_mutable[0], 0, sizeof(gps_run_t));
#endif
    speed->display.record = 0;
}

static bool store_run_max_speed(gps_speed_t * speed, uint16_t run_count) {
    if (speed->max_speed < speed->cur_speed) {
        speed->max_speed = speed->cur_speed;
        store_time(&speed->runs[0]);
        speed->runs[0].nr = run_count;
        speed->runs[0].avg_speed = speed->max_speed;
#if defined(MUTABLE_RUNS)
        memcpy(&speed->runs_mutable[0], &speed->runs[0], sizeof(gps_run_t));
#endif
        return 1;
    }
    return 0;
}

#if defined(GPS_STATS)
static void gps_run_printf(const struct gps_run_s * run) {
    printf("Run: {time: %02d:%02d.%02d, avg_speed: %.02f, nr: %hu}\n", run->time.hour, run->time.minute, run->time.second, run->avg_speed, run->nr);
}
#endif

#if defined(GPS_STATS) && defined(GPS_TRACE_MSG_SPEED_BY_DIST)
static esp_err_t gps_speed_by_dist_printf(const struct gps_speed_by_dist_s *me) {
    uint8_t i, j=NUM_OF_SPD_ARRAY_SIZE;
    printf("=== speed_by_dist: {\n");
    printf("m_set_dist: %hu, ", me->distance_window);
    printf("m_dist: %ld, ", me->distance);
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

/// Instance to determine the average speed over a certain distance, 
/// when a new run starts, save the highest speed of the previous run.
struct gps_speed_by_dist_s *init_gps_speed_by_distance(struct gps_speed_by_dist_s *me, uint16_t dist) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    memset(me, 0, sizeof(struct gps_speed_by_dist_s));
    me->distance_window = dist;
    return me;
}

#if defined(MUTABLE_RUNS)
void reset_speed_stats(struct gps_speed_by_dist_s *me) {
    reset_runs_avg(me->speed.RUNS_FOR_DISPLAY);
    reset_display_speed(me->speed.display.display_speed);
}
#endif

static inline bool store_speed_by_dist(struct gps_speed_by_dist_s *me) {
    // printf("[%s]\n", __func__);
    if (me->distance >= me->distance_window_raw) {
        me->m_sample = log_p_lctx.index_gspeed - me->m_index + 1;  // Check for the number of samples to avoid division by zero
        if (me->m_sample > 0) {
            // Calculate the distance in mm, so multiply by 1000 and consider the sample_rate !!
            me->speed.cur_speed = (float)me->distance / me->m_sample;  // 10 samples op 1s aan 10mm/s = 100/10 = 10 mm /s
        }
        // if (me->m_sample > 1) {
        //     // Calculate the speed based on the distance and the number of samples
        //     me->speed_alfa = me->distance / me->m_sample - 1;
        // }
    } else if(me->speed.cur_speed > 0) {
        me->speed.cur_speed = 0;
        // me->speed_alfa = 0;
    }
    return me->speed.cur_speed > 0;  // return the speed in mm/s
}

static inline void store_dist_data(struct gps_speed_by_dist_s *me) {
    if (store_run_max_speed(&me->speed, gps->run_count)) {  // store max speed of this run
        me->speed.runs[0].data.dist.dist = me->distance;
        me->speed.runs[0].data.dist.nr_samples = me->m_sample;
        me->speed.runs[0].data.dist.message_nr = gps->ubx_device->ubx_msg.count_nav_pvt;
    }
    update_display_speeds(&me->speed, &gps->record);
}

static inline void store_and_reset_dist_data_after_run(struct gps_speed_by_dist_s *me) {
#if (C_LOG_LEVEL < 2)
    ILOG(TAG, "[%s]", __func__);
#endif
    // sort_run_alfa(me->speed.runs, me->dist, me->message_nr, (uint32_t*)me->nr_samples, 10);
    sort_runs(
        me->speed.runs,
        (void *[]){0},
        (size_t[]){0},
        0, NUM_OF_SPD_ARRAY_SIZE
    );
#if defined(MUTABLE_RUNS)
    sort_runs(
        me->speed.RUNS_FOR_DISPLAY,
        (void *[]){0},
        (size_t[]){0},
        0, NUM_OF_SPD_ARRAY_SIZE
    );
#endif
    update_display_speed_array(&me->speed.display, me->speed.RUNS_FOR_DISPLAY, 0, NUM_OF_SPD_ARRAY_SIZE);
    reset_last_run_speeds(&me->speed);
}

static inline void move_distance_window(struct gps_speed_by_dist_s *me) {
    // printf("[%s]\n", __func__);
    // Determine buffer m_index for the desired distance
    if(me->distance > me->distance_window_raw) {
        while (me->distance > me->distance_window_raw && (log_p_lctx.index_gspeed - me->m_index) < log_p_lctx.buf_gspeed_size) {
            me->distance -= log_p_lctx.buf_gspeed[buf_index(me->m_index++)];
        }
        me->distance = me->distance + log_p_lctx.buf_gspeed[buf_index(--me->m_index)];
    }
}

float update_speed_by_distance(struct gps_speed_by_dist_s *me) {
    // printf("[%s]\n", __func__);
    if(!me) return 0.0f;
    // uint32_t distance_window = convert_distance_to_mm(me->distance_window, gps->ubx_device->rtc_conf->output_rate);  // Note that m_distance_window should now be in mm, so multiply by 1000 and consider the sample_rate !!
    me->distance = me->distance + log_p_lctx.buf_gspeed[buf_index(log_p_lctx.index_gspeed)];  // the resolution of the distance is 0.1 mm
                                                                                  // the max int32  is 2,147,483,647 mm eq 214,748.3647 meters eq ~214 kilometers !!
    if ((log_p_lctx.index_gspeed - me->m_index) >= log_p_lctx.buf_gspeed_size) {  // controle buffer overflow
        printf("[%s] buffer overflow, resetting index_gspeed\n", __func__);
        me->distance = 0;
        me->m_index = log_p_lctx.index_gspeed;
    }
    move_distance_window(me);
    // printf("[%s] dist: %.1f, set: %hu spd: %.1f, max: %0.1f\n", __func__, get_distance_m(me->distance, gps->ubx_device->rtc_conf->output_rate), me->distance_window, me->speed.runs[0].avg_speed, me->speed.max_speed);
    if(store_speed_by_dist(me)) {  // store the speed if it is greater than 0
        store_dist_data(me);  // store the data in the speed struct
    }
    if ((gps->run_count != me->speed.nr_prev_run) && (me->speed.runs[0].nr == me->speed.nr_prev_run)) {  // opslaan hoogste snelheid van run + sorteren
        store_and_reset_dist_data_after_run(me);
    }
    me->speed.nr_prev_run = gps->run_count;
    record_last_run(&me->speed, gps->run_count);
    return me->speed.max_speed;
}


/// Instance to determine the average speed over a certain time window
struct gps_speed_by_time_s *init_gps_speed_by_time(struct gps_speed_by_time_s *me, uint16_t window) {
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
#if defined(MUTABLE_RUNS)
void reset_time_stats(struct gps_speed_by_time_s *me) {
    reset_runs_avg(me->speed.RUNS_FOR_DISPLAY);
    reset_display_speed(me->speed.display.display_speed);
}
#endif

#if defined(SPEED_BAR_SETUP)
static inline void store_speed_bar_data(struct gps_speed_by_time_s *me, uint16_t run_count) {
    if (me->speed.max_speed > SPEED_THRESHOLD_BAR) me->bar.bar_count++;  // min speed bar graph = 5 m/s
    me->bar.run_speed[run_count % NR_OF_BAR] = me->speed.runs[0].avg_speed;
}
#endif

static inline void store_speed_by_time_data(struct gps_speed_by_time_s *me) {
    // printf("[%s]\n", __func__);
    if (store_run_max_speed(&me->speed, gps->run_count)) {
        me->speed.runs[0].data.time.Mean_cno = gps->Ublox_Sat.sat_info.Mean_mean_cno;
        me->speed.runs[0].data.time.Max_cno = gps->Ublox_Sat.sat_info.Mean_max_cno;
        me->speed.runs[0].data.time.Min_cno = gps->Ublox_Sat.sat_info.Mean_min_cno;
        me->speed.runs[0].data.time.Mean_numSat = gps->Ublox_Sat.sat_info.Mean_numSV;
#if defined(SPEED_BAR_SETUP)
        me->bar.run_speed[gps->run_count % NR_OF_BAR] = me->speed.cur_speed;
#endif
    }
    if(update_display_speeds(&me->speed, &gps->record)) {
        // update_avg_5runs(&me->speed, true); // average of the runs 0 and 6-9
    }
}

static inline void store_and_reset_time_data_after_run(struct gps_speed_by_time_s *me) {
#if (C_LOG_LEVEL < 2)
    ILOG(TAG, "[%s]", __func__);
#endif
    sort_runs(
        me->speed.runs,
        (void *[]){0},
        (size_t[]){0},
        0, NUM_OF_SPD_ARRAY_SIZE
    );
#if defined(MUTABLE_RUNS)
    sort_runs(
        me->speed.RUNS_FOR_DISPLAY,
        (void *[]){0},
        (size_t[]){0},
        0, NUM_OF_SPD_ARRAY_SIZE
    );
#endif
    update_display_speed_array(&me->speed.display, me->speed.RUNS_FOR_DISPLAY, 0, NUM_OF_SPD_ARRAY_SIZE);  // update the runs array
#if defined(SPEED_BAR_SETUP)
    store_speed_bar_data(me, context->run_count);  // store the speed bar data
#endif
    reset_last_run_speeds(&me->speed);  // reset the speed for the next run
}

static inline bool store_avg_speed_by_time_optimized(struct gps_speed_by_time_s *me, uint32_t time_window_delta, uint8_t sample_rate) {
    // printf("[%s] %lu\n", __func__, time_window_delta);
    bool window_reached = false;
    const uint32_t current_speed = log_p_lctx.buf_gspeed[buf_index(log_p_lctx.index_gspeed)];
    
    if (time_window_delta < log_p_lctx.buf_gspeed_size) {  // if time window is smaller than the sample_rate*BUFFER, use normal buffer
        me->avg_s_sum += current_speed;  // always add gSpeed at every update
        if (log_p_lctx.index_gspeed >= time_window_delta) { // once window is reached, subtract old value from the sum
            me->avg_s_sum -= log_p_lctx.buf_gspeed[buf_index(log_p_lctx.index_gspeed - time_window_delta)];
            window_reached = true;  // only if the time window is reached, we can calculate the speed
            // Pre-calculate division factor to avoid repeated division
            const float inv_time_samples = 1.0f / (me->time_window * sample_rate);
            me->speed.cur_speed = (float)me->avg_s_sum * inv_time_samples;
        }
    } else if (log_p_lctx.index_gspeed % sample_rate == 0) {  // switch to seconds buffer, but only one update per second !!
        me->avg_s_sum += log_p_lctx.buf_sec_speed[sec_buf_index(log_p_lctx.index_sec)];  // log_p_lctx.buf_sec_speed[SEC_BUFFER_SIZE] and log_p_lctx.index_sec
        if (log_p_lctx.index_sec >= me->time_window) { // once window is reached, subtract old value
            me->avg_s_sum -= log_p_lctx.buf_sec_speed[sec_buf_index(log_p_lctx.index_sec - me->time_window)];
            window_reached = true;  // only if the time window is reached, we can calculate the speed
            const float inv_time_window = 1.0f / me->time_window;
            me->speed.cur_speed = (float)me->avg_s_sum * inv_time_window;  // in the seconds array is the average of gspeed !!
        }
    }
    if(!window_reached && me->speed.cur_speed > 0) {
        me->speed.cur_speed = 0;  // if the time window is not reached, set the speed to 0
    }
    return window_reached;  // return true if the time window is reached, so we can calculate the speed
}

float update_speed_by_time(struct gps_speed_by_time_s *me) {
    if(!me) return 0.0f;
    const uint8_t sample_rate = gps->ubx_device->rtc_conf->output_rate;
    // uint32_t actual_run = gps->run_count;
    const uint32_t time_window_delta = me->time_window * sample_rate;
    if(store_avg_speed_by_time_optimized(me, time_window_delta, sample_rate))
        store_speed_by_time_data(me);  // store the run data if the speed is higher than the previous run
    if ((gps->run_count != me->speed.nr_prev_run) && (me->speed.runs[0].nr == me->speed.nr_prev_run)) {  // sorting only if new max during this run !!!
        store_and_reset_time_data_after_run(me);  // sort the runs and update the display speed}
    }
    me->speed.nr_prev_run = gps->run_count;
    record_last_run(&me->speed, gps->run_count); 
    return me->speed.max_speed;  // anders compiler waarschuwing control reaches end of non-void function [-Werror=return-type]
}

struct gps_speed_alfa_s *init_gps_speed_by_alfa(struct gps_speed_by_dist_s *m) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    if (!m || !m->alfa) return NULL;  // protection against NULL pointer
    memset(m->alfa, 0, sizeof(struct gps_speed_alfa_s));
    m->alfa->distance_window = m->distance_window;
    m->alfa->base = m;
    return m->alfa;
}

#if defined(MUTABLE_RUNS)
void reset_alfa_stats(struct gps_speed_alfa_s *me) {
    reset_runs_avg(me->speed.RUNS_FOR_DISPLAY);
}
#endif

#if defined(GPS_STATS) && defined(GPS_TRACE_MSG_SPEED_ALPHA)
static esp_err_t gps_speed_by_alpha_printf(const struct gps_speed_alfa_s *me) {
    uint8_t i, j=NUM_OF_SPD_ARRAY_SIZE;
    printf("=== speed_by_alfa: {\n");
    printf("set_alfa_dist: %hu, ", me->distance_window);
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

#if defined(USE_HAVERSINE_DIST_TO_LINE) || defined(USE_HAVERSINE)
// Optimized coordinate conversion with pre-calculated constants
static inline void latlon_to_xyz_optimized(const gps_point_t *pt, double *x, double *y, double *z) {
    const double lat_rad = pt->latitude * DEG2RAD_CONST;
    const double lon_rad = pt->longitude * DEG2RAD_CONST;
    const double cos_lat = cos(lat_rad);
    *x = cos_lat * cos(lon_rad);
    *y = cos_lat * sin(lon_rad);
    *z = sin(lat_rad);
}
#endif

#if defined(USE_HAVERSINE)

/**
 * @brief Calculates the Haversine distance between two GPS points.
 *
 * This function computes the distance between two points on the Earth's surface using the Haversine formula.
 * The function uses the mean radius of the Earth (6,371 km) to calculate the distance.
 * The Haversine formula is used to calculate the distance between two points on the surface of a sphere.
 * The result is a double value representing the distance in meters.
 *
 * @note The function uses the constant EARTH_RADIUS_M, which is defined as 6,371,000 meters (mean radius of the Earth).
 * @note The function uses the constant DEG2RAD to convert degrees to radians.
 *
 * @param p1 Pointer to the gps_point_t structure representing the first GPS point (latitude and longitude).
 * @param p2 Pointer to the gps_point_t structure representing the second GPS point (latitude and longitude).
 * @return The Haversine distance between the two GPS points in meters.
 */
// static inline double haversine_previous(gps_point_t * p1, gps_point_t * p2) {
//     float dlat = DEG2RAD * (p2->latitude -  p1->latitude);
//     float dlon = DEG2RAD * (p2->longitude - p1->longitude);
//     double a = POW_2(sin(dlat/2)) + cos(DEG2RAD * p1->latitude) * cos(DEG2RAD *  p2->latitude) * POW_2(sin(dlon/2));
//     double c = 2 * atan2(sqrt(a), sqrt(1-a));
//     return EARTH_RADIUS_M * c;
// }

// Optimized haversine calculation with minimal function calls
static inline double straight_dist_haversine_optimized(const gps_point_t *p1, const gps_point_t *p2) {
    double x1, y1, z1, x2, y2, z2;
    latlon_to_xyz_optimized(p1, &x1, &y1, &z1);
    latlon_to_xyz_optimized(p2, &x2, &y2, &z2);

    // Dot product of unit vectors with bounds checking optimized
    double dot = x1 * x2 + y1 * y2 + z1 * z2;
    // Fast clamping without branches for common case
    dot = (dot > 1.0) ? 1.0 : ((dot < -1.0) ? -1.0 : dot);

    return EARTH_RADIUS_M_CONST * acos(dot); // distance in meters
}

#else

/// @brief Calculates the straight-line distance squared between two GPS points.
/// This function computes the squared distance between two points on the Earth's surface using a simplified formula.
/// The formula is based on the difference in latitude and longitude, adjusted for the curvature of the Earth.
/// The result is a double value representing the squared distance in meters.
/// This function is used to determine if the distance between two points is less than a certain threshold.
/// @note The function uses the constant DEG2RAD to convert degrees to radians.
/// @note The function uses the constant DEG_TO_METERS to convert degrees to meters.
/// @param p1 Pointer to the gps_point_t structure representing the first GPS point (latitude and longitude).
/// @param p2 Pointer to the gps_point_t structure representing the second GPS point (latitude and longitude).
/// @return The squared straight-line distance between the two GPS points in meters.
static inline double straight_dist_square(gps_point_t * p1, gps_point_t * p2) {
    // now calculate the absolute distance alfa_speed::alfa_update(GPS_speed M)
    // between the starting point and the end point of the 250m distance,
    // if < 50m this is an alfa !!! note, this is calculated in meters,
    // therefore alfa_circle also in m !! was (M.m_index-1), should be (M.m_index+1)
    float dlat = p1->latitude - p2->latitude;
    float px = cos(DEG2RAD_CONST * p1->latitude) * (p1->longitude - p2->longitude);
    return DEG_TO_METERS(POW_2(dlat) + POW_2(px));
}
#endif

#if defined(USE_HAVERSINE_DIST_TO_LINE)
/**
 * @brief Haversine-based perpendicular distance from a point to a great-circle defined by two points.
 * @param act Pointer to the point to measure from.
 * @param p1 Pointer to the first point defining the line.
 * @param p2 Pointer to the second point defining the line.
 * @return The shortest distance from act to the great-circle through p1 and p2, in meters.
 */
static double point_to_line_distance(const gps_point_t *act, const gps_point_t *p1, const gps_point_t *p2) {
    // Convert all points to 3D Cartesian coordinates
    double x0, y0, z0, x1, y1, z1, x2, y2, z2;
    latlon_to_xyz(act, &x0, &y0, &z0);
    latlon_to_xyz(p1, &x1, &y1, &z1);
    latlon_to_xyz(p2, &x2, &y2, &z2);

    // Compute normal vector of the plane (great circle) defined by p1 and p2
    double nx = y1 * z2 - z1 * y2;
    double ny = z1 * x2 - x1 * z2;
    double nz = x1 * y2 - y1 * x2;
    double norm = sqrt(nx * nx + ny * ny + nz * nz);
    nx /= norm; ny /= norm; nz /= norm;

    // The perpendicular (angular) distance is arcsin(dot product of act and normal))
    double dot = x0 * nx + y0 * ny + z0 * nz;
    double angle = fabs(asin(dot)); // radians

    return EARTH_RADIUS_M * angle; // meters
}
#else
/*
/// previous code for point_to_line_distance, not used anymore, but kept for reference
float point_to_line_distance(float long_act, float lat_act, float long_1, float lat_1, float long_2, float lat_2) {
    float corr_lat=111120;
    float corr_long=111120*cos(DEG2RAD*buf_alfa[log_p_lctx.index_gspeed%BUFFER_ALFA]);
    lambda_T=(log_p_lctx.P2_lat-log_p_lctx.P1_lat)*(P_lat-log_p_lctx.P1_lat)*corr_lat*corr_lat+(log_p_lctx.P2_long-log_p_lctx.P1_long)*(P_long-log_p_lctx.P1_long)*corr_long*corr_long;
    lambda_N=pow((log_p_lctx.P2_lat-log_p_lctx.P1_lat)*corr_lat,2)+pow((log_p_lctx.P2_long-log_p_lctx.P1_long)*corr_long,2);
    lambda=lambda_T/lambda_N;
    alfa_dist=sqrt(
        pow((P_lat-log_p_lctx.P1_lat-lambda*(log_p_lctx.P2_lat-log_p_lctx.P1_lat))*corr_lat,2)+
        pow((P_long-log_p_lctx.P1_long-lambda*(log_p_lctx.P2_long-log_p_lctx.P1_long))*corr_long,2));
}
*/
/// Calculates distance from point with act lat/long to line which passes points lat_1/long_1 and lat_2/long_2
/// The result is the perpendicular distance from the point to the line in meters.
/// This function uses the curvature of the Earth to calculate the distance.
/// @note The function uses the constant METERS_PER_LATITUDE_DEGREE to convert latitude degrees to meters.
/// @note The function uses the constant DEG2RAD to convert degrees to radians.
/// @param act Pointer to the gps_point_t structure representing the current position (latitude and longitude).
/// @param p1 Pointer to the gps_point_t structure representing the first point on the line (latitude and longitude).
/// @param p2 Pointer to the gps_point_t structure representing the second point on the line (latitude and longitude).
/// @return The perpendicular distance from the point to the line in meters.
// Optimized point to line distance with pre-calculated latitude correction
static float point_to_line_distance_optimized(const gps_point_t * act, const gps_point_t * p1, const gps_point_t * p2) {
    // Pre-calculate corrections to avoid repeated calculations
    const float corr_lat = METERS_PER_LATITUDE_DEGREE;
    const float corr_long = corr_lat * cos(act->latitude * DEG2RAD_CONST);
    
    // Cache differences
    const float dlat = p2->latitude - p1->latitude;
    const float dlong = p2->longitude - p1->longitude;
    const float dlat_act = act->latitude - p1->latitude;
    const float dlong_act = act->longitude - p1->longitude;
    
    // Pre-calculate squared corrections
    const float corr_lat_sq = corr_lat * corr_lat;
    const float corr_long_sq = corr_long * corr_long;
    
    const float lambda_T = (dlat * dlat_act * corr_lat_sq) + (dlong * dlong_act * corr_long_sq);
    const float lambda_N = (dlat * dlat * corr_lat_sq) + (dlong * dlong * corr_long_sq);
    
    const float lambda = lambda_T / lambda_N;
    
    // Optimized final calculation
    const float dx = (dlat_act - lambda * dlat) * corr_lat;
    const float dy = (dlong_act - lambda * dlong) * corr_long;
    
    return sqrtf(dx * dx + dy * dy); // Use sqrtf for float precision
}

#endif

static inline void store_alfa_data(struct gps_speed_alfa_s *me, uint32_t dist) {
    // printf("[%s]\n", __func__);
    if (store_run_max_speed(&me->speed, gps->run_count)) {
        me->speed.runs[0].data.alfa.message_nr = gps->ubx_device->ubx_msg.count_nav_pvt;
        me->speed.runs[0].data.alfa.real_distance = (int32_t)me->straight_dist_square;
        me->speed.runs[0].data.alfa.dist = dist;
    }
    update_display_speeds(&me->speed, &gps->record);
}

static inline void store_and_reset_alfa_data_after_run(struct gps_speed_alfa_s *me) { 
#if (C_LOG_LEVEL < 2)
    ILOG(TAG, "[%s]", __func__);
#endif
    // sort_run_alfa(me->speed.runs, me->real_distance, me->message_nr, me->dist, 10);
    sort_runs(
        me->speed.runs,
        (void *[]){0},
        (size_t[]){0},
        0, NUM_OF_SPD_ARRAY_SIZE
    );
#if defined(MUTABLE_RUNS)
    sort_runs(
        me->speed.RUNS_FOR_DISPLAY,
        (void *[]){0},
        (size_t[]){0},
        0, NUM_OF_SPD_ARRAY_SIZE
    );
#endif
    reset_last_run_speeds(&me->speed);  // reset the speed for the next run
}

// Attention, here the distance traveled must be less than 500 m! Therefore, 
// an extra variable, m_speed_alfa, is provided in GPS_speed!!!
float update_speed_by_alfa(struct gps_speed_by_dist_s *m) {
    if(!m || !m->alfa) return 0.0f;
    struct gps_speed_alfa_s *me = m->alfa;
    // if (gps->Ublox.run_distance_after_turn < 375000.0f) {
#if defined(USE_HAVERSINE)
        me->straight_dist_square = straight_dist_haversine_optimized(
#else
        me->straight_dist_square = straight_dist_square(
#endif
            &log_p_lctx.alfa_buf[al_buf_index(log_p_lctx.index_gspeed)], 
            &log_p_lctx.alfa_buf[al_buf_index(m->m_index + 1)]
        );
#if defined(USE_HAVERSINE)
        if (me->straight_dist_square < ALFA_THRESHOLD) {
#else
        if (me->straight_dist_square < POW_2(me->set_alfa_dist)) {
#endif
            me->speed.cur_speed = m->speed.cur_speed; // current speed in mm/s
            if (m->m_sample >= log_p_lctx.alfa_buf_size) {
                printf("Warning: m_sample %ld >= al_buf_size %hd, setting speed to 0\n", m->m_sample, log_p_lctx.alfa_buf_size);
                me->speed.cur_speed = 0;  // avoid overflow at low speeds
            }
            store_alfa_data(me, m->distance);
        }
        // printf("[%s] dist: %.1f, set: %hu spd: %.1f, max: %0.1f\n", __func__, get_distance_m(m->distance, gps->ubx_device->rtc_conf->output_rate), me->base->distance_window, me->speed.runs[0].avg_speed, me->speed.max_speed);
    // }
    // if((alfa_speed_max>0.0f)&(straight_dist_square>(alfa_circle_square*1.4))){ //alfa max only resets to 0 if 500 m after the jibe, straight distance after the jibe
    if (gps->run_count != me->speed.nr_prev_run) {
        store_and_reset_alfa_data_after_run(me);
    }
    me->speed.nr_prev_run = gps->run_count;
    record_last_run(&me->speed, gps->run_count); 
    return me->speed.max_speed;
}

// Optimized heading unwrap with pre-calculated thresholds
static inline float unwrap_heading_optimized(float actual_heading, float *old_heading, float *delta_heading) {
    const float diff = actual_heading - *old_heading;
    // Use pre-calculated thresholds for faster comparison
    if (diff > 300.0f) *delta_heading -= 360.0f;
    else if (diff < -300.0f) *delta_heading += 360.0f;
    *old_heading = actual_heading;
    return actual_heading + *delta_heading;
}

static inline void detect_run_start_end(float S2_speed) {
    /// detection stand still, more then 2s with velocity < 1m/s 
    if (S2_speed > SPEED_DETECTION_MIN) log_p_lctx.velocity_5 = 1;  // 2s with speed over 4m/s
    if ((S2_speed < STANDSTILL_DETECTION_MAX) && (log_p_lctx.velocity_5)) log_p_lctx.velocity_0 = 1; // stopping detected, 2s with speed < 1 m/s
    else log_p_lctx.velocity_0 = 0;
    /// New run detected due to standstill
    if (log_p_lctx.velocity_0) {
        log_p_lctx.velocity_5 = 0;
        log_p_lctx.delay_count_before_run = 0;
    }
}

/* Calculation of the average heading over the last 10 seconds 
************************************************************************/
static inline bool is_straight_course(float heading_diff) {
    return (heading_diff < STRAIGHT_COURSE_MAX_DEV);
}

static inline bool detect_jibe(float heading_diff) {
    return heading_diff > JIBE_COURSE_DEVIATION_MIN && log_p_lctx.straight_course;
}

uint32_t new_run_detection(gps_context_t *context, float actual_heading, float S2_speed) {
    // printf("[%s]\n", __func__);
    if(!context) return 0;  // return 0 if context is NULL
    ubx_config_t *ubx = context->ubx_device;
    const uint8_t sample_rate = ubx->rtc_conf->output_rate;

    log_p_lctx.heading = unwrap_heading_optimized(actual_heading, &log_p_lctx.old_heading, &log_p_lctx.delta_heading);    
    
    /// detect heading change over 15s is more than 40°, a new run is started
    const uint16_t mean_heading_delta_time = TIME_WINDOW_SAMPLES_15S * sample_rate;
    const float inv_mean_heading_time = 1.0f / mean_heading_delta_time;
    log_p_lctx.heading_mean = log_p_lctx.heading_mean * (1.0f - inv_mean_heading_time) + log_p_lctx.heading * inv_mean_heading_time;
    
    /// detection stand still, more then 2s with velocity < 1m/s 
    detect_run_start_end(S2_speed);

    /// New run detected due to heading change
    const float heading_diff = fabsf(log_p_lctx.heading_mean - log_p_lctx.heading);
    if(is_straight_course(heading_diff) && log_p_lctx.velocity_5) {
        // printf("Straight course detected, heading_diff: %.02f\n", heading_diff);
        if(!log_p_lctx.straight_course) log_p_lctx.straight_course = 1;  // straight course detected, set straight_course to true
    }
    if(detect_jibe(heading_diff)) {
// #if (C_LOG_LEVEL < 2)
        printf("Jibe detected, heading_diff: %.02f\n", heading_diff);
// #endif
        if(gps->skip_alfa_after_stop) gps->skip_alfa_after_stop = 0;  // reset the skip alfa after stop counter
        if(log_p_lctx.straight_course) log_p_lctx.straight_course = 0;  // jibe detected, straight course is false
        log_p_lctx.delay_count_before_run = 0;
        context->alfa_count++;  // jibe detection for alfa_indicator ....
    }
    log_p_lctx.delay_count_before_run++;
    const uint32_t time_delay_samples = TIME_DELAY_NEW_RUN * sample_rate;
    if (log_p_lctx.delay_count_before_run == time_delay_samples) {
        ++context->run_count;
// #if (C_LOG_LEVEL < 2)
//         WLOG(TAG, "=== Run finished, count changed to %hu ===", context->run_count);
// #endif
    }
// #if (C_LOG_LEVEL < 2)
//     printf("delay_count_before_run: %lu, start_cond: %u ...\n", log_p_lctx.delay_count_before_run, time_delay_samples);
//     printf("run_count: %hu, context->alfa_count: %hu, log_p_lctx.straight_course: %d\n", context->run_count, context->alfa_count, log_p_lctx.straight_course);
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

/* Here the current "alfa distance" is calculated based on 2 points for the jibe: 
 P1 = 250m and P2 = 100m for the jibe. These points determine an imaginary line, 
 the perpendicular distance to the current position must be less than 50m/s
 when point P1 is passed.
 */
float alfa_indicator(float actual_heading) {
    // Update jibe reference points if needed
    if (gps->alfa_count != log_p_lctx.old_alfa_count) {
        // the distance traveled since the jibe detection 10*100.000/10.000=100 samples ?
        gps->Ublox.run_distance_after_turn = 0;
        update_jibe_reference_points(
            al_buf_index(gps->speed_metrics[dist_250m].handle.dist->m_index),
            al_buf_index(gps->speed_metrics[dist_100m].handle.dist->m_index),
            log_p_lctx.alfa_buf,
            &log_p_lctx.alfa_p1,
            &log_p_lctx.alfa_p2
        );
        log_p_lctx.old_alfa_count = gps->alfa_count;
    }
    // Current position
    int32_t idx_cur = al_buf_index(log_p_lctx.index_gspeed);
    gps_point_t cur = { log_p_lctx.alfa_buf[idx_cur].latitude, log_p_lctx.alfa_buf[idx_cur].longitude};

    // Position 2 seconds ago
    int32_t idx_prev = al_buf_index(log_p_lctx.index_gspeed - (2 * gps->ubx_device->rtc_conf->output_rate));
    gps_point_t prev = {log_p_lctx.alfa_buf[idx_prev].latitude, log_p_lctx.alfa_buf[idx_prev].longitude};

    gps->alfa_exit = point_to_line_distance_optimized(&log_p_lctx.alfa_p1, &cur, &prev); // turn-250m point distance to line {cur,cur-2sec}
    gps->alfa_window = point_to_line_distance_optimized(&cur, &log_p_lctx.alfa_p1, &log_p_lctx.alfa_p2); // cur point distance to line {turn-m250,turn-m100}
#if (C_LOG_LEVEL < 3)
    printf("[%s] run: %hu, exit: %.1f, window: %.1f, atdist: %.1f\n", __func__, 
            gps->run_count, gps->alfa_exit, 
            gps->alfa_window, 
            (float)(MM_TO_M(gps->Ublox.run_distance_after_turn)));
#endif
    return gps->alfa_window;  // current perpendicular distance relative to the line P2-P1, may be max 50m for a valid alfa !!
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
