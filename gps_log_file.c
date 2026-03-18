#include "log_private.h"

#if (defined(CONFIG_UBLOX_ENABLED) && defined(CONFIG_GPS_LOG_ENABLED))

#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/unistd.h>
#include <math.h>
#include <stdio.h>
#include <stdatomic.h>

#include <esp_mac.h>
#include <esp_heap_caps.h>

#include "gps_log.h"
// #include "esp_log.h"
#include "logger_buffer_pool.h"
#include "logger_fixed_pool.h"

#include "strbf.h"
#include "numstr.h"
#include "ubx.h"
#include "gpx.h"
#include "sbp.h"
#ifdef GPS_LOG_ENABLE_GPY
#include "gpy.h"
#endif
#include "vfs.h"
#include "vfs_events.h"
#include "context.h"

// #include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "gps_log";

// ============================================================================
// ASYNC WRITER - FAT-optimized file I/O with sector-aligned buffering
// ============================================================================

// Buffer size matches FAT sector size from sdkconfig for optimal write performance
// CONFIG_FATFS_PER_FILE_CACHE provides FatFs-level caching (inside VFS layer)
// Our async writer provides additional benefits:
// - Non-blocking writes from GPS task (offload to dedicated writer task)
// - Batch multiple small writes into single sector-aligned write
// - Reduce write() syscall overhead (40 writes/sec -> ~1 flush/sec)
#if defined(CONFIG_FATFS_SECTOR_4096)
    #define ASYNC_WRITER_BUFFER_SIZE    4096   // 4KB sectors
#elif defined(CONFIG_FATFS_SECTOR_512)
    #define ASYNC_WRITER_BUFFER_SIZE    512    // 512B sectors
#else
    #define ASYNC_WRITER_BUFFER_SIZE    4096   // Default to 4KB for modern cards
#endif

#define ASYNC_WRITER_QUEUE_DEPTH    16     // Queue depth for 1x20Hz = 20 writes/sec
#define ASYNC_WRITER_FLUSH_TIMEOUT_MS 1000 // Flush buffer after 1 second if not full

typedef enum {
    ASYNC_WRITE_REQUEST_DATA,     // Write data to buffer
    ASYNC_WRITE_REQUEST_FLUSH,    // Force buffer flush
    ASYNC_WRITE_REQUEST_CLOSE,    // Close file and cleanup
} async_write_request_type_t;

// ============================================================================
// ASYNC WRITE POOL - Pre-allocated fixed-size blocks eliminate hot-path malloc.
// GPS frames: UBX/SBP typically <128B, GPX text <400B; 512B covers all cases.
// 16 blocks x 512B = 8KB static DRAM; zero alloc/free on GPS hot path.
// ============================================================================
#define ASYNC_POOL_BLOCK_SIZE   512   // Max GPS frame (covers GPX text ~400B)
#define ASYNC_POOL_BLOCK_COUNT  16    // Matches QUEUE_DEPTH; total pool = 8KB

typedef struct {
    uint8_t data[ASYNC_POOL_BLOCK_SIZE];
    size_t  len;                  // Actual data length in this block
} async_pool_block_t;

typedef struct {
    async_write_request_type_t type;
    uint8_t file_index;           // sd_log_ubx, sd_log_sbp, etc.
    async_pool_block_t *block;    // DATA: borrowed pool block; NULL for FLUSH/CLOSE
} async_write_request_t;

// Per-file write buffer state
typedef struct {
    uint8_t *buffer;              // 4KB buffer (malloc'd)
    size_t buffer_used;           // Bytes used in buffer
    TickType_t last_write_tick;   // Last write timestamp for timeout flush
} file_write_buffer_t;

static QueueHandle_t async_writer_queue = NULL;
static TaskHandle_t async_writer_task_handle = NULL;
static file_write_buffer_t file_buffers[sd_log_end] = {0}; // Max 5 file types (UBX, SBP, GPX, GPY, TXT)
static atomic_bool async_writer_running = false;

// Pool storage - dynamically allocated on init, freed on stop; lifecycle managed by logger_fixed_pool
static async_pool_block_t *async_pool_storage = NULL;
static logger_fixed_pool_t async_pool = {0};

// Forward declarations
static void async_writer_task(void *arg);
static esp_err_t async_writer_flush_buffer(uint8_t file_index);
static esp_err_t async_writer_write_buffered(uint8_t file_index, const uint8_t *data, size_t len);

// Definitions for functions declared in log_private.h
float get_spd(float b) {
    return convert_speed(b, g_rtc_config.gps.speed_unit);
}

float get_avg5(const float *arr, float (*conv)(float), int start_index) {
    float sum = 0.0f;
    for (int i = start_index, end_index = start_index + 5; i < end_index; ++i) sum += arr[i];
    return conv ? conv(sum / 5.0f) : (sum / 5.0f);
}

RTC_DATA_ATTR gps_log_file_config_t log_config = GPS_LOG_DEFAULT_CONFIG();

// Buffer helpers for centralized buffer pool
// Helper function to get GPS message buffer (512 bytes)
static esp_err_t get_gps_message_buffer(logger_buffer_handle_t *handle) {
    if (!logger_buffer_pool_is_initialized()) {
        ELOG(TAG, "Buffer pool not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    return logger_buffer_pool_alloc(LOGGER_BUFFER_MEDIUM,
        LOGGER_BUFFER_USAGE_GPS_MESSAGE, handle, pdMS_TO_TICKS(5000));
}

// Helper function to get GPS scratch buffer (256 bytes)
static esp_err_t get_gps_scratch_buffer(logger_buffer_handle_t *handle) {
    if (!logger_buffer_pool_is_initialized()) {
        ELOG(TAG, "Buffer pool not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    return logger_buffer_pool_alloc(LOGGER_BUFFER_SMALL,
        LOGGER_BUFFER_USAGE_GPS_DATA, handle, pdMS_TO_TICKS(5000));
}

// Sync log_file_bits with current runtime config (g_rtc_config.gps)
static void gps_log_sync_bits_from_config(gps_log_file_config_t *cfg) {
    // Ensure at least one format is enabled; fall back to SBP
    if (!gps_log_file_bits_check(&g_rtc_config.gps.log_enables)) {
        g_rtc_config.gps.log_enables.bits.log_sbp = 1;
    }
}

void gps_config_fix_values(void) {
    FUNC_ENTRY(TAG);
    gps_log_sync_bits_from_config(&log_config);
}

// Helper function to release any GPS buffer
static void release_gps_buffer(logger_buffer_handle_t *handle) {
    if (handle && handle->buffer) {
        logger_buffer_pool_free(handle);
    }
}

// static const char * const speed_units[] = {
//     "m/s", "km/h","knots"
// };

//static gps_log_file_config_t log_config = GPS_LOG_DEFAULT_CONFIG();


static float get_avg(const gps_run_t *b) {
    return (float) get_avg5((const float[]){b[5].avg_speed,b[6].avg_speed,b[7].avg_speed,b[8].avg_speed,b[9].avg_speed}, get_spd, 0);
}

// ============================================================================
// ASYNC WRITER IMPLEMENTATION - FAT-optimized buffered I/O
// ============================================================================

/**
 * @brief Flush write buffer to file
 * Must be called from async writer task only
 */
static esp_err_t async_writer_flush_buffer(uint8_t file_index) {
    if (file_index >= 5) return ESP_ERR_INVALID_ARG;

    file_write_buffer_t *fb = &file_buffers[file_index];
    if (!fb->buffer || fb->buffer_used == 0) {
        return ESP_OK; // Nothing to flush
    }

    int fd = GET_FD(file_index);
    if (fd < 0) {
        ELOG(TAG, "Invalid FD for file %"PRIu8, file_index);
        return ESP_ERR_INVALID_STATE;
    }

    // Perform synchronous write (we're in dedicated writer task)
    ssize_t written = write(fd, fb->buffer, fb->buffer_used);
    if (written != fb->buffer_used) {
        ELOG(TAG, "Flush failed for file %"PRIu8": wrote %zu of %zu bytes (%s)", 
             file_index, written, fb->buffer_used, strerror(errno));
        return ESP_FAIL;
    }

    DLOG(TAG, "Flushed %zu bytes to file %"PRIu8, fb->buffer_used, file_index);
    fb->buffer_used = 0;
    fb->last_write_tick = xTaskGetTickCount();

    return ESP_OK;
}

/**
 * @brief Write data to buffer, flushing if full
 * Must be called from async writer task only
 */
static esp_err_t async_writer_write_buffered(uint8_t file_index, const uint8_t *data, size_t len) {
    if (file_index >= 5 || !data || len == 0) return ESP_ERR_INVALID_ARG;

    file_write_buffer_t *fb = &file_buffers[file_index];

    // Allocate buffer on first write
    if (!fb->buffer) {
        /* Prefer internal DRAM; fall back to DEFAULT (may include SPIRAM) */
        fb->buffer = (uint8_t *)heap_caps_malloc(ASYNC_WRITER_BUFFER_SIZE, buffer_caps);
        if (!fb->buffer) {
            ELOG(TAG, "Failed to allocate %dB buffer for file %"PRIu8, ASYNC_WRITER_BUFFER_SIZE, file_index);
            return ESP_ERR_NO_MEM;
        }
        fb->buffer_used = 0;
        fb->last_write_tick = xTaskGetTickCount();
        ILOG(TAG, "Allocated %dB write buffer for file %"PRIu8, ASYNC_WRITER_BUFFER_SIZE, file_index);
    }

    size_t bytes_written = 0;
    while (bytes_written < len) {
        size_t space_available = ASYNC_WRITER_BUFFER_SIZE - fb->buffer_used;
        size_t bytes_to_copy = (len - bytes_written) < space_available ? 
                               (len - bytes_written) : space_available;

        memcpy(fb->buffer + fb->buffer_used, data + bytes_written, bytes_to_copy);
        fb->buffer_used += bytes_to_copy;
        bytes_written += bytes_to_copy;

        // Flush if buffer full
        if (fb->buffer_used >= ASYNC_WRITER_BUFFER_SIZE) {
            esp_err_t err = async_writer_flush_buffer(file_index);
            if (err != ESP_OK) {
                return err;
            }
        }
    }

    fb->last_write_tick = xTaskGetTickCount();
    return ESP_OK;
}

/**
 * @brief Async writer task - processes write requests from queue
 */
static void async_writer_task(void *arg) {
    ILOG(TAG, "Async writer task started");
    async_write_request_t req;
    TickType_t last_timeout_check = xTaskGetTickCount();

    while (async_writer_running) {
        // Wait for request with timeout for periodic flush check
        if (xQueueReceive(async_writer_queue, &req, pdMS_TO_TICKS(100)) == pdTRUE) {
            switch (req.type) {
                case ASYNC_WRITE_REQUEST_DATA:
                    if (req.block) {
                        if (req.block->len > 0) {
                            async_writer_write_buffered(req.file_index,
                                                        req.block->data, req.block->len);
                        }
                        logger_fixed_pool_free(&async_pool, req.block);  // Return block to pool (O(1))
                        req.block = NULL;
                    }
                    break;

                case ASYNC_WRITE_REQUEST_FLUSH:
                    async_writer_flush_buffer(req.file_index);
                    break;

                case ASYNC_WRITE_REQUEST_CLOSE:
                    // Flush before closing
                    async_writer_flush_buffer(req.file_index);
                    // Free buffer
                    if (file_buffers[req.file_index].buffer) {
                        heap_caps_free(file_buffers[req.file_index].buffer);
                        file_buffers[req.file_index].buffer = NULL;
                        file_buffers[req.file_index].buffer_used = 0;
                    }
                    break;
            }
        }

        // Periodic timeout flush check (every 100ms)
        TickType_t now = xTaskGetTickCount();
        if ((now - last_timeout_check) >= pdMS_TO_TICKS(100)) {
            for (uint8_t i = 0; i < sd_log_end; i++) {
                file_write_buffer_t *fb = &file_buffers[i];
                if (fb->buffer && fb->buffer_used > 0) {
                    // Flush if data older than timeout threshold
                    if ((now - fb->last_write_tick) >= pdMS_TO_TICKS(ASYNC_WRITER_FLUSH_TIMEOUT_MS)) {
                        DLOG(TAG, "Timeout flush for file %d (%zu bytes)", i, fb->buffer_used);
                        async_writer_flush_buffer(i);
                    }
                }
            }
            last_timeout_check = now;
        }
    }

    // Cleanup on exit: flush all buffers
    for (uint8_t i = 0; i < sd_log_end; i++) {
        if (file_buffers[i].buffer) {
            async_writer_flush_buffer(i);
            heap_caps_free(file_buffers[i].buffer);
            file_buffers[i].buffer = NULL;
            file_buffers[i].buffer_used = 0;
        }
    }

    ILOG(TAG, "Async writer task stopped");
    async_writer_task_handle = NULL;
    vTaskDelete(NULL);
}
#define ASYNC_WRITER_TASK_STACK_SIZE 2560
/**
 * @brief Start async writer subsystem
 */
esp_err_t async_writer_start(void) {
    if (async_writer_running) {
        WLOG(TAG, "Async writer already running");
        return ESP_OK;
    }
    task_memory_info(__func__);
    mem_info();
    // Allocate pool storage dynamically (eliminates static memory usage)
    FUNC_ENTRY_ARGS(TAG, "Starting async writer with pool block size %d bytes, count %d, total %zu bytes", 
              ASYNC_POOL_BLOCK_SIZE, ASYNC_POOL_BLOCK_COUNT, ASYNC_POOL_BLOCK_COUNT * sizeof(async_pool_block_t));
    async_pool_storage = (async_pool_block_t *)heap_caps_malloc(
        ASYNC_POOL_BLOCK_COUNT * sizeof(async_pool_block_t), buffer_caps);
    if (!async_pool_storage) {
        ELOG(TAG, "Failed to allocate async write pool (%zu bytes)", 
             ASYNC_POOL_BLOCK_COUNT * sizeof(async_pool_block_t));
        return ESP_ERR_NO_MEM;
    }

    // Initialize pre-allocated block pool (eliminates hot-path malloc)
    esp_err_t pool_err = logger_fixed_pool_init(&async_pool, async_pool_storage,
                                                 sizeof(async_pool_block_t),
                                                 ASYNC_POOL_BLOCK_COUNT);
    if (pool_err != ESP_OK) {
        ELOG(TAG, "Failed to initialize async write pool");
        heap_caps_free(async_pool_storage);
        async_pool_storage = NULL;
        return pool_err;
    }

    // Create queue for write requests
    async_writer_queue = xQueueCreate(ASYNC_WRITER_QUEUE_DEPTH, sizeof(async_write_request_t));
    if (!async_writer_queue) {
        ELOG(TAG, "Failed to create async writer queue");
        logger_fixed_pool_deinit(&async_pool);
        return ESP_ERR_NO_MEM;
    }

    // Pre-allocate sector buffers for currently open files.
    // Avoids first-write allocation failure mid-recording; non-fatal if heap tight.
    for (uint8_t i = 0; i < sd_log_end; i++) {
        if (GET_FD(i) >= 0 && !file_buffers[i].buffer) {
            FUNC_ENTRY_ARGS(TAG, "Pre-allocating buffer for open file %d (%dB)", i, ASYNC_WRITER_BUFFER_SIZE);
            file_buffers[i].buffer = (uint8_t *)heap_caps_malloc(ASYNC_WRITER_BUFFER_SIZE, buffer_caps);
            if (!file_buffers[i].buffer) {
                WLOG(TAG, "Pre-alloc failed for file %d (%dB), will retry on first write",
                     i, ASYNC_WRITER_BUFFER_SIZE);
            } else {
                file_buffers[i].buffer_used = 0;
                file_buffers[i].last_write_tick = xTaskGetTickCount();
            }
        }
    }

    task_memory_info(__func__);
    mem_info();
    
    // Create writer task (priority 5, below GPS task priority 10)
    async_writer_running = true;
    BaseType_t ret = xTaskCreatePinnedToCore(
        async_writer_task,
        "async_writer",
        ASYNC_WRITER_TASK_STACK_SIZE,
        NULL,
        5,                              // Priority 5 (lower than GPS task)
        &async_writer_task_handle,
        tskNO_AFFINITY                  // No CPU affinity
    );

    if (ret != pdPASS) {
        ELOG(TAG, "Failed to create async writer task");
        vQueueDelete(async_writer_queue);
        async_writer_queue = NULL;
        async_writer_running = false;
        logger_fixed_pool_deinit(&async_pool);
        return ESP_FAIL;
    }

    ILOG(TAG, "Async writer started (%dB sector-aligned buffering, %dms flush timeout)", 
         ASYNC_WRITER_BUFFER_SIZE, ASYNC_WRITER_FLUSH_TIMEOUT_MS);
    task_memory_info(__func__);
    mem_info();
    return ESP_OK;
}

/**
 * @brief Stop async writer subsystem
 */
void async_writer_stop(void) {
    if (!async_writer_running) {
        return;
    }

    ILOG(TAG, "Stopping async writer...");
    async_writer_running = false;

    // Wait for task to finish (max 2 seconds)
    for (int i = 0; i < 20 && async_writer_task_handle != NULL; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    // Grace period: task sets handle NULL then calls vTaskDelete(NULL);
    // a brief yield ensures the scheduler has cleaned up the TCB before
    // we delete the queue that the task may still reference in that window.
    vTaskDelay(pdMS_TO_TICKS(10));

    if (async_writer_queue) {
        vQueueDelete(async_writer_queue);
        async_writer_queue = NULL;
    }

    logger_fixed_pool_deinit(&async_pool);
    if (async_pool_storage) {
        heap_caps_free(async_pool_storage);
        async_pool_storage = NULL;
    }
    async_writer_task_handle = NULL;
    ILOG(TAG, "Async writer stopped");
}

/**
 * @brief Queue async write request (non-blocking)
 * @param file_index File index (sd_log_ubx, sd_log_sbp, etc.)
 * @param data Data to write (will be copied and freed by writer task)
 * @param len Length of data
 * @return ESP_OK on success, error otherwise
 */
static esp_err_t queue_async_write(uint8_t file_index, const uint8_t *data, size_t len) {
    if (!async_writer_running || !async_writer_queue) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Reject frames too large for the pool (caller falls back to sync write)
    if (len > ASYNC_POOL_BLOCK_SIZE) {
        WLOG(TAG, "Frame too large for pool (%zu > %d), using sync write", len, ASYNC_POOL_BLOCK_SIZE);
        return ESP_ERR_NO_MEM;
    }

    // Borrow a pre-allocated block from pool (O(1), zero heap pressure)
    async_pool_block_t *blk = (async_pool_block_t *)logger_fixed_pool_alloc(&async_pool);
    if (!blk) {
        WLOG(TAG, "Write pool exhausted for file %"PRIu8" (%zu B), using sync write",
             file_index, len);
        return ESP_ERR_NO_MEM;
    }
    memcpy(blk->data, data, len);
    blk->len = len;

    async_write_request_t req = {
        .type = ASYNC_WRITE_REQUEST_DATA,
        .file_index = file_index,
        .block = blk,
    };

    // Non-blocking send (0 timeout)
    if (xQueueSend(async_writer_queue, &req, 0) != pdTRUE) {
        logger_fixed_pool_free(&async_pool, blk);
        ELOG(TAG, "Async writer queue full (dropped %zu bytes for file %"PRIu8")", len, file_index);
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

/**
 * @brief Queue async flush request
 */
static esp_err_t queue_async_flush(uint8_t file_index) {
    if (!async_writer_running || !async_writer_queue) {
        return ESP_ERR_INVALID_STATE;
    }

    async_write_request_t req = {
        .type = ASYNC_WRITE_REQUEST_FLUSH,
        .file_index = file_index,
        .block = NULL,
    };

    if (xQueueSend(async_writer_queue, &req, 0) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

/**
 * @brief Queue async close request
 */
static esp_err_t queue_async_close(uint8_t file_index) {
    if (!async_writer_running || !async_writer_queue) {
        return ESP_ERR_INVALID_STATE;
    }

    async_write_request_t req = {
        .type = ASYNC_WRITE_REQUEST_CLOSE,
        .file_index = file_index,
        .block = NULL,
    };

    if (xQueueSend(async_writer_queue, &req, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

// ============================================================================
// VFS EVENT HANDLERS - Reinitialize logging config when partition available
// ============================================================================

/**
 * @brief Handle VFS partition discovery - reinitialize log config when partition becomes available
 * This ensures logging can start as soon as storage is discovered by VFS
 */
static void vfs_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base != VFS_EVENT) {
        return;
    }

    // Detect mount-like events that may make storage available
    bool is_mount_event = false;
    bool is_unmount_event = false;

    switch (event_id) {
        case VFS_EVENT_SDCARD_MOUNTED:
        case VFS_EVENT_FAT_PARTITION_MOUNTED:
#if defined(CONFIG_LITTLEFS_ENABLED)
        case VFS_EVENT_LITTEFS_PARTITION_MOUNTED:
#endif
#if defined(CONFIG_SPIFFS_ENABLED)
        case VFS_EVENT_SPIFFS_PARTITION_MOUNTED:
#endif
        case VFS_EVENT_LOG_PARTITION_CHANGED:
            is_mount_event = true;
            break;
        case VFS_EVENT_SDCARD_UNMOUNTED:
        case VFS_EVENT_FAT_PARTITION_UNMOUNTED:
#if defined(CONFIG_LITTLEFS_ENABLED)
        case VFS_EVENT_LITTEFS_PARTITION_UNMOUNTED:
#endif
#if defined(CONFIG_SPIFFS_ENABLED)
        case VFS_EVENT_SPIFFS_PARTITION_UNMOUNTED:
#endif
        case VFS_EVENT_SDCARD_MOUNT_FAILED:
        case VFS_EVENT_FAT_PARTITION_MOUNT_FAILED:
#if defined(CONFIG_LITTLEFS_ENABLED)
        case VFS_EVENT_LITTEFS_PARTITION_MOUNT_FAILED:
#endif
#if defined(CONFIG_SPIFFS_ENABLED)
        case VFS_EVENT_SPIFFS_PARTITION_MOUNT_FAILED:
#endif
        case VFS_EVENT_SDCARD_WRITE_FAILED:
        case VFS_EVENT_FAT_PARTITION_WRITE_FAILED:
#if defined(CONFIG_LITTLEFS_ENABLED)
        case VFS_EVENT_LITTEFS_PARTITION_WRITE_FAILED:
#endif
#if defined(CONFIG_SPIFFS_ENABLED)
        case VFS_EVENT_SPIFFS_PARTITION_WRITE_FAILED:
#endif
            is_unmount_event = true;
            break;
        default:
            break;
    }

    const bool partition_available = gps_log_partition_is_available();

    if (partition_available && is_mount_event) {
        ILOG(TAG, "VFS partition available - refreshing GPS log config and scheduling file reopen");
        gps_log_config_init();

        // Notify UI/other components that config refreshed and ask VFS worker to close+open
        if (esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_CONFIG_REFRESHED,
                          NULL, 0, pdMS_TO_TICKS(100)) != ESP_OK) {
            WLOG(TAG, "EVT_FAIL: CONFIG_REFRESHED");
        }

        if (gps) {
            vfs_post_work(VFS_WORK_PARTITION_CHANGED, gps);
        }
    } else if (!partition_available && (is_unmount_event || is_mount_event)) {
        WLOG(TAG, "GPS log partition unavailable - closing log files and notifying UI");
        if (gps && gps->files_opened) {
            vfs_post_work(VFS_WORK_CLOSE_FILES, gps);
        }
        if (esp_event_post(GPS_LOG_EVENT,
                          GPS_LOG_EVENT_LOG_FILES_OPEN_FAILED, NULL, 0,
                          pdMS_TO_TICKS(100)) != ESP_OK) {
            WLOG(TAG, "EVT_FAIL: LOG_FILES_OPEN_FAILED");
        }
    }
}

/**
 * @brief Register VFS event handler for GPS logging config updates
 * Call this during GPS initialization to track partition availability
 */
void gps_log_register_vfs_handler(void) {
    esp_event_handler_register(VFS_EVENT, ESP_EVENT_ANY_ID, vfs_event_handler, NULL);
    DLOG(TAG, "Registered VFS event handler for GPS logging");
}

/**
 * @brief Unregister VFS event handler when GPS logging is stopped
 */
void gps_log_unregister_vfs_handler(void) {
    esp_event_handler_unregister(VFS_EVENT, ESP_EVENT_ANY_ID, vfs_event_handler);
    DLOG(TAG, "Unregistered VFS event handler for GPS logging");
}

// ============================================================================
// FILE OPERATIONS - now use async writer
// ============================================================================

/**
 * @brief Write data to file (async buffered)
 * Replaces synchronous write with async buffered write for 20Hz+ performance
 */
size_t log_write(const struct gps_context_s * context, uint8_t file, const void *msg, size_t len) {
    int fd = GET_FD(file);
    if (fd < 0)
        return 0;

    // Use async writer if running
    if (async_writer_running) {
        esp_err_t err = queue_async_write(file, (const uint8_t *)msg, len);
        if (err == ESP_OK) {
            return len; // Return requested length on successful queue
        }
        // Fall through to synchronous write on error
        WLOG(TAG, "Async write failed, falling back to sync");
    }

    // Fallback: synchronous write
    ssize_t result = write(fd, msg, len);
    if (result < 0) {
        ELOG(TAG, "Failed to write (%s) %" PRIu8, strerror(errno), file);
    }
    return result;
}

/**
 * @brief Close file (async close + flush)
 */
int log_close(const struct gps_context_s * context, uint8_t file) {
    int fd = GET_FD(file);
    if (fd < 0)
        return fd;

    // Queue async close request (flushes buffer automatically)
    if (async_writer_running) {
        queue_async_close(file);
        // Wait for queue to drain (bounded, up to 500ms)
        for (int i = 0; i < 50
            && uxQueueMessagesWaiting(async_writer_queue) > 0; i++) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    log_fsync(context, file);
    int result = close(fd);
    if (result < 0) {
        ELOG(TAG, "Failed to close (%s) fd: %" PRIu8, strerror(errno), file);
    }
    return result;
}

/**
 * @brief Sync file (async flush)
 */
int log_fsync(const struct gps_context_s * context, uint8_t file) {
    int fd = GET_FD(file);
    if (fd < 0)
        return fd;

    // Queue async flush if writer running
    if (async_writer_running) {
        queue_async_flush(file);
        // Wait for queue to drain (bounded, up to 250ms)
        for (int i = 0; i < 25
            && uxQueueMessagesWaiting(async_writer_queue) > 0; i++) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    int result = fsync(fd);
    if (result < 0) {
        ELOG(TAG, "Failed to sync (%s) %" PRIu8, strerror(errno), file);
    }
    return result;
}

void log_err(const gps_context_t *context, const char *message) {
    if(!context) return;
    if (g_rtc_config.gps.log_enables.bits.log_txt) {
        WRITETXT(message, strlen(message));
    }
}

// ============================================================================
// GPS LOG CONFIG GATING - Check partition availability before logging
// ============================================================================

/**
 * @brief Check if GPS log partition is available and ready for logging
 * @return true if partition is mounted and has valid mount point, false otherwise
 */
bool gps_log_partition_is_available(void) {
    // Check if partition index is valid
    if (vfs_ctx.gps_log_part == VFS_PART_MAX) {
        DLOG(TAG, "GPS log partition not assigned (VFS_PART_MAX)");
        return false;
    }

    // Check partition bounds
    if (vfs_ctx.gps_log_part >= VFS_MAX_PARTS) {
        ELOG(TAG, "GPS log partition index out of bounds: %" PRIu8 " >= %" PRIu8, vfs_ctx.gps_log_part, VFS_MAX_PARTS);
        return false;
    }

    // Check if partition is mounted
    vfs_config_t *part = &vfs_ctx.parts[vfs_ctx.gps_log_part];
    if (!part->is_mounted) {
        DLOG(TAG, "GPS log partition not mounted: %" PRIu8, vfs_ctx.gps_log_part);
        return false;
    }

    // Check mount point is valid
    if (!part->mount_point || strlen(part->mount_point) == 0) {
        ELOG(TAG, "GPS log partition has invalid mount point");
        return false;
    }

    // All checks passed
    return true;
}

/**
 * @brief Initialize GPS log config with VFS partition info
 * Only proceeds if partition is available; otherwise returns early with warning
 */
gps_log_file_config_t *gps_log_config_init(void) {
    FUNC_ENTRY_ARGS(TAG, "gps_log_part: %" PRIu8 " (MAX=%" PRIu8 ")", vfs_ctx.gps_log_part, VFS_PART_MAX);

    // GATE: Check if partition is available before initializing path
    if (!gps_log_partition_is_available()) {
        WLOG(TAG, "GPS log partition not available - logging disabled until partition found");
        log_config.base_path[0] = '\0';  // Clear path to prevent invalid operations
        // Notify UI/other modules early so the user sees missing storage state
        if (esp_event_post(GPS_LOG_EVENT,
                          GPS_LOG_EVENT_LOG_FILES_OPEN_FAILED, NULL, 0,
                          pdMS_TO_TICKS(100)) != ESP_OK) {
            WLOG(TAG, "EVT_FAIL: LOG_FILES_OPEN_FAILED");
        }
        return &log_config;
    }

    // Partition is available - initialize path safely
    strbf_t buf;
    strbf_inits(&buf, log_config.base_path, ESP_VFS_PATH_MAX + 1);

    vfs_config_t *part = &vfs_ctx.parts[vfs_ctx.gps_log_part];
    strbf_put_path(&buf, part->mount_point);
    strbf_finish(&buf);

    FUNC_ENTRY_ARGSD(TAG, "log path initialized: %s", &log_config.base_path[0]);
    return &log_config;
}

#ifndef FILE_APPEND
#define FILE_APPEND "a"
#endif

// esp_err_t save_log_file_bits(gps_context_t *context, uint8_t *log_file_bits) {
//     assert(context && context->log_config);
//     logger_config_t *config = context->log_config->config;
//     if (!config)
//         return ESP_ERR_INVALID_ARG;
//     if(c_gps_cfg.log_ubx)
//         SETBIT(*log_file_bits, sd_log_ubx);
//     if(c_gps_cfg.log_gpy)
//         SETBIT(*log_file_bits, sd_og_gpy);
//     if(c_gps_cfg.log_sbp)
//         SETBIT(*log_file_bits, sd_log_sbp);
//     if(c_gps_cfg.log_gpx)
//         SETBIT(*log_file_bits, sd_log_gpx);
//     if(c_gps_cfg.log_txt) 
//         SETBIT(*log_file_bits, sd_log_txt);
//     return ESP_OK;
// }

bool log_files_opened(gps_context_t *context) {
    if(!context) return false;
    return context->files_opened;
}

void open_files(gps_context_t *context) {
    FUNC_ENTRY(TAG);
    if(!context) return;
    if (context->files_opened)
        return;

    // GATE: Check if partition is available before attempting to open files
    if (!gps_log_partition_is_available()) {
        WLOG(TAG, "Cannot open log files - GPS log partition not available");
        if (esp_event_post(GPS_LOG_EVENT,
                          GPS_LOG_EVENT_LOG_FILES_OPEN_FAILED, NULL, 0,
                          pdMS_TO_TICKS(100)) != ESP_OK) {
            WLOG(TAG, "EVT_FAIL: LOG_FILES_OPEN_FAILED");
        }
        return;
    }

    // Partition is available - safe to proceed with file operations
    gps_log_file_config_t *config = context->log_config;
    // logger_config_t *cfg = config->config;
    // save_log_file_bits(context, log_config.log_file_bits);
    // Use direct config access instead of old sconfig API
    char str[32] = {0}, *filename = str;
    strncpy(filename, g_rtc_config.gps.ubx_file, sizeof(str) - 1);
    if (strlen(filename) == 0) {
        strcpy(filename, "gps");
    }
    while (*filename == '/' || *filename == ' ')
        ++filename;
    strbf_t sb;
    uint8_t open_failed = 0;
    strbf_inits(&sb, config->filename_base, PATH_MAX_CHAR_SIZE);
    // Use direct config access for file_date_time
    uint8_t value = g_rtc_config.gps.file_date_time;
    if (value != 0) {
        // struct tm tmstruct;
        // char extension[16] = ".txt";  //
        char timestamp[16];
        strbf_t tsb;
        struct tm tms;
        strbf_inits(&tsb, timestamp, 16);
        get_local_time(&tms);
        strbf_putl(&tsb, tms.tm_year + 1900);
        if (tms.tm_mon < 9)
            strbf_putc(&tsb, '0');
        strbf_putl(&tsb, tms.tm_mon + 1);
        if (tms.tm_mday < 10)
            strbf_putc(&tsb, '0');
        strbf_putl(&tsb, tms.tm_mday);
        if (tms.tm_hour < 10)
            strbf_putc(&tsb, '0');
        strbf_putl(&tsb, tms.tm_hour);
        if (tms.tm_min < 10)
            strbf_putc(&tsb, '0');
        strbf_putl(&tsb, tms.tm_min);
        // sprintf(timestamp, "%zu%02u%02u%02u%02u", (tms.tm_year) + 1900, (tms.tm_mon) + 1, tms.tm_mday, tms.tm_hour, tms.tm_min);
        if (value == 1) {
            strbf_puts(&sb, filename);  // copy filename from config
            strbf_putc(&sb, '_');
            strbf_puts(&sb, timestamp);  // add timestamp
        }
        if (value == 2) {
            strbf_puts(&sb, timestamp);  // add timestamp
            strbf_putc(&sb, '_');
            strbf_puts(&sb, filename);  // copy filename from config
        }
        // strbf_puts(&sb, extension);  // add extension.txt
    } else {
        // char txt[16] = "000.txt";
        char macAddr[24];
        sprintf(macAddr, MACSTR, MAC2STR(context->mac_address));
        strbf_puts(&sb, filename);  // copy filename from config
        strbf_putc(&sb, '_');
        strbf_puts(&sb, macAddr);
        int filenameSize = sb.cur - sb.start;  // dit is dan 7 + NULL = 8
        strbf_puts(&sb, "000");                 // dit wordt dan /BN280A000.txt
        for (uint16_t i = 0; i < 1000; i++) {
            config->filename_base[filenameSize + 2] = '0' + i % 10;
            config->filename_base[filenameSize + 1] = '0' + ((i / 10) % 10);
            config->filename_base[filenameSize] = '0' + ((i / 100) % 10);
            // create if does not exist, do not open existing, write, sync after
            // write
#if defined(CONFIG_LOGGER_VFS_ENABLED)
            if (!s_xfile_exists(config->filename_base)) {
                break;
            }
#endif
        }
    }
        // Refresh selection from runtime config just before open
        gps_log_sync_bits_from_config(config);
    const char * fn = 0;
    cfg_gps_log_enables_t *enables = &g_rtc_config.gps.log_enables;
    if(!gps_log_file_bits_check(enables)) // at least one file must be opened
        enables->bits.log_sbp = 1;
    for(uint8_t i = 0; i < sd_log_end; i++) {
        FUNC_ENTRY_ARGSD(TAG, "opening %s", config->filenames[i]);
        if (enables->value & (1 << i)) {
            fn = 
#if defined(GPS_LOG_ENABLE_GPY)
            i == sd_log_gpy ? ".gpy" : 
#endif
            i == sd_log_ubx ? ".ubx" : i == sd_log_sbp ? ".sbp" : i == sd_log_gpx ? ".gpx" : ".txt";
            strcpy(config->filenames[i], config->filename_base);
            strcat(config->filenames[i], fn);
            FUNC_ENTRY_ARGSD(TAG, "opening %s", config->filenames[i]);
#if defined(CONFIG_LOGGER_VFS_ENABLED)
            GET_FD(i) = s_open(config->filenames[i], config->base_path, FILE_APPEND);
            if(GET_FD(i) < 0) {
                open_failed++;
            }
#endif
        }
    }
    if (esp_event_post(GPS_LOG_EVENT,
                      open_failed ? GPS_LOG_EVENT_LOG_FILES_OPEN_FAILED
                                  : GPS_LOG_EVENT_LOG_FILES_OPENED,
                      NULL, 0, pdMS_TO_TICKS(100)) != ESP_OK) {
        WLOG(TAG, "EVT_FAIL: LOG_FILES open/fail event");
    }
    // Only mark files as opened if at least one fd was successfully opened
    context->files_opened = 0;
    for (int _fi = 0; _fi < sd_log_end; _fi++) {
        if (GET_FD(_fi) >= 0) { context->files_opened = 1; break; }
    }

    // Start async writer after files opened (only if partition is still available)
    if (!open_failed && gps_log_partition_is_available()) {
        esp_err_t err = async_writer_start();
        if (err != ESP_OK) {
            WLOG(TAG, "Failed to start async writer, using synchronous writes");
        }
    }
}

void close_files(gps_context_t *context) {
    FUNC_ENTRY(TAG);
    if(!context) return;
    if (!context->files_opened) {
        return;
    }

    // Stop async writer before closing files (flushes all buffers)
    async_writer_stop();

    gps_log_file_config_t *config = context->log_config;
    for (int i = 0, j = sd_log_end; i < j; ++i) {
        if (GET_FD(i) >= 0) {
            if (i == sd_log_gpx) {
                log_GPX(context, GPX_END);
            }
            log_close(context, i);
            GET_FD(i) = -1;
        }
    }
    context->files_opened = 0;
    if (esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_LOG_FILES_CLOSED,
                      NULL, 0, pdMS_TO_TICKS(100)) != ESP_OK) {
        WLOG(TAG, "EVT_FAIL: LOG_FILES_CLOSED");
    }
}

static int load_balance = 0;

void flush_files(const gps_context_t *context) {
    if(!context) return;
    if (!context->files_opened) {
        return;
    }
    if (g_rtc_config.ubx.output_rate <= 10) {
        if (load_balance == 0) {
            log_fsync(context, sd_log_ubx);
        }
        if (load_balance == 1) {
            log_fsync(context, sd_log_txt);
        }
#ifdef GPS_LOG_ENABLE_GPY
        if (load_balance == 2) {
            log_fsync(context, sd_og_gpy);
        }
#endif
        if (load_balance == 3) {
            log_fsync(context, sd_log_sbp);
        }
        if (load_balance == 4) {
            log_fsync(context, sd_log_gpx);
            load_balance = -1;
        }
        load_balance++;
    }
}

esp_err_t log_gps_timeout(const gps_context_t *context, uint32_t time_diff_ms, const char *tag) {
    logger_buffer_handle_t scratch_handle = {0};
    if (get_gps_scratch_buffer(&scratch_handle) == ESP_OK) {
        strbf_t sb;
        char *datastr = (char*)scratch_handle.buffer;
        char *buffer = datastr + 200;
        strbf_inits(&sb, datastr, 200);
        time_to_char_hms(context->ubx_device->ubx_msg.navPvt.hour, 
            context->ubx_device->ubx_msg.navPvt.minute, 
            context->ubx_device->ubx_msg.navPvt.second, 
            buffer);
        strbf_puts(&sb, buffer);
        strbf_putc(&sb, ':');
        strbf_putl(&sb, log_p_lctx.count_nav_pvt);
        strbf_putc(&sb, ':');
        strbf_putl(&sb, context->run_count);
        strbf_putc(&sb, ':');
        strbf_putl(&sb, time_diff_ms);
        strbf_putc(&sb, ':');
        strbf_puts(&sb, tag);
        strbf_putc(&sb, '\n');
        if (GET_FD(sd_log_txt) > 0) {
            WRITETXT(strbf_finish(&sb), sb.cur - sb.start);
        }
        release_gps_buffer(&scratch_handle);
    }
    return ESP_OK;
}

// ============================================================================
// GPS LOG TO FILE - Main entry point called from GPS task
// Formats GPS data and writes to enabled file formats via async writer
// ============================================================================

void log_to_file(gps_context_t *context) {
    if(!context || !context->files_opened || context->time_set != 1) return;

    ubx_ctx_t *ubx = context->ubx_device;
    if (!ubx) return;  // Safety check

    struct nav_pvt_s * nav_pvt = &ubx->ubx_msg.navPvt;

    // Log data in all enabled formats (consolidated bit checks)
    cfg_gps_log_enables_t enables = g_rtc_config.gps.log_enables;
    if (enables.bits.log_ubx) {
        log_ubx(context, &ubx->ubx_msg, g_rtc_config.ubx.msgout_sat);
    }
#ifdef GPS_LOG_ENABLE_GPY
    if (enables.bits.log_gpy) {
        log_GPY(context);
    }
#endif
    if (enables.bits.log_sbp) {
        log_SBP(context);
    }
    if (enables.bits.log_gpx) {
        log_GPX(context, GPX_FRAME);
    }
}

// Prints the content of a file to the Serial
// static void printFile(gps_log_file_config_t *log, const char *filename) {
//     // Open file for reading
//     char *f = s_read_from_file(filename, log->base_path);
//     if (f) {
//         printf("%s", f);
//         free(f);
//     }
// }

static void session_info(const gps_context_t *context, struct gps_data_s *G) {
    FUNC_ENTRY(TAG);
    if(!context) return;
    if (!context->files_opened) {
        ELOG(TAG, "[%s] files not open", __FUNCTION__);
        return;
    }
    // logger_config_t *config = context->log_config->config;
    const ubx_ctx_t *ubx = context->ubx_device;
    const ubx_msg_t * ubxMessage = &ubx->ubx_msg;
    int32_t millis = get_millis();
    strbf_t sb;

    // Get buffers from pool
    logger_buffer_handle_t msg_handle = {0}, scratch_handle = {0};
    if (get_gps_message_buffer(&msg_handle) != ESP_OK || get_gps_scratch_buffer(&scratch_handle) != ESP_OK) {
        ELOG(TAG, "Failed to get GPS buffers for session info");
        release_gps_buffer(&msg_handle);
        release_gps_buffer(&scratch_handle);
        return;
    }

    char *message = (char*)msg_handle.buffer;
    char *tekst = (char*)scratch_handle.buffer;

    strbf_inits(&sb, message, msg_handle.size);
    strbf_puts(&sb, "T5 MAC adress: ");
    sprintf(tekst, MACSTR, MAC2STR(gps->mac_address));
    strbf_puts(&sb, tekst);
    strbf_puts(&sb, "\n");
    strbf_puts(&sb, "GPS Logger: ");
    semVerStr(tekst, 0);
    strbf_puts(&sb, tekst);
    strbf_puts(&sb, "\n");
    strbf_puts(&sb, "First fix : ");
    strbf_putn(&sb, MS_TO_SEC(gps->first_fix));
    strbf_puts(&sb, " s\n");
    strbf_puts(&sb, "Total time : ");
    strbf_putn(&sb, MS_TO_SEC(millis - gps->start_logging_millis));
    strbf_puts(&sb, " s\n");

    strbf_puts(&sb, "Total distance : ");
    strbf_putn(&sb, MM_TO_M(G->total_distance));
    strbf_puts(&sb, " m\n");
    strbf_puts(&sb, "Sample rate : ");
    strbf_putn(&sb, g_rtc_config.ubx.output_rate);
    strbf_puts(&sb, " Hz\n");
    strbf_puts(&sb, "Speed units: ");
    strbf_puts(&sb, get_speed_unit_str(g_rtc_config.gps.speed_unit));
    strbf_puts(&sb, " \n");
    strbf_puts(&sb, "Timezone : ");
    strbf_putn(&sb, g_rtc_config.gps.timezone);
    strbf_puts(&sb, " h\n");
    strbf_puts(&sb, "Dynamic model: ");
    strbf_puts(&sb, dynamic_models[g_rtc_config.ubx.nav_mode==0 ? 0 : g_rtc_config.ubx.nav_mode-2]);
    strbf_puts(&sb, " \n");
    strbf_puts(&sb, "GNSS = GPS + ");
    if (ubxMessage->monGNSS.enabled_Gnss == 3)
        strbf_puts(&sb, "GLONAS");
    if (ubxMessage->monGNSS.enabled_Gnss == 9)
        strbf_puts(&sb, "GALILEO");
    if (ubxMessage->monGNSS.enabled_Gnss == 11)
        strbf_puts(&sb, "GLONAS + GALILEO");
    if (ubxMessage->monGNSS.enabled_Gnss == 13)
        strbf_puts(&sb, "GLONAS + BEIDOU");
    if (ubxMessage->monGNSS.enabled_Gnss == 15)
        strbf_puts(&sb, "GLONAS + GALILEO + BEIDOU");
    strbf_puts(&sb, " \n");
    strbf_puts(&sb, "Ublox SW-version : ");
    strbf_puts(&sb, ubxMessage->mon_ver.swVersion);
    strbf_puts(&sb, " \n");
    strbf_puts(&sb, "Ublox HW-version : ");
    strbf_puts(&sb, ubxMessage->mon_ver.hwVersion);
    strbf_puts(&sb, " \n");
    strbf_puts(&sb, "Ublox ");
    strbf_puts(&sb, ubx_get_dev_str());
    strbf_puts(&sb, " ID = ");
    strbf_sprintf(&sb,"%02x%02x%02x%02x%02x", &ubxMessage->ubxId.ubx_id_1, &ubxMessage->ubxId.ubx_id_2, &ubxMessage->ubxId.ubx_id_3, &ubxMessage->ubxId.ubx_id_4, &ubxMessage->ubxId.ubx_id_5);
    if (ubx->hw_type > UBX_TYPE_M8)
        strbf_sprintf(&sb, "%02x", ubxMessage->ubxId.ubx_id_6);
    strbf_puts(&sb, "\n\n*** Results ***\n");
    WRITETXT(strbf_finish(&sb), sb.cur - sb.start);
    FUNC_ENTRY_ARGSD(TAG, "%s", sb.start);

    // Release buffers
    release_gps_buffer(&msg_handle);
    release_gps_buffer(&scratch_handle);
}

const char * strings[] = {
    "==="
};

static void result_speed_avg(gps_speed_t *speed, strbf_t *sb, const char * units, const char * unit, uint16_t window, char *tekst) {
    f3_to_char(get_avg(&speed->runs[0]), tekst);
    strbf_puts(sb, strings[0]);
    strbf_putc(sb, ' ');
    strbf_puts(sb, unit);
    strbf_putl(sb, window);
    strbf_puts(sb, " avg_5: ");
    strbf_puts(sb, tekst);
    strbf_puts(sb, units);
    strbf_puts(sb, ", best_runs ");
    strbf_puts(sb, strings[0]);
    strbf_putc(sb, '\n');
}

// ============================================================================
// GPS METRICS RENDER HELPER - shared buffer-acquire/release pattern
// ============================================================================

/**
 * @brief Context passed to every gps_metrics format callback.
 *        The strbf is embedded so the render helper owns it; callbacks
 *        only need this single pointer.
 */
typedef struct {
    strbf_t  sb;            // String builder (maps onto message buffer)
    char    *message;       // Points into msg_handle.buffer
    size_t   message_size;  // msg_handle.size
    char    *tekst;         // Points into scratch_handle.buffer (temp conversions)
} gps_metrics_ctx_t;

/** Signature for content-specific formatting callbacks. */
typedef void (*gps_metrics_format_fn)(gps_metrics_ctx_t *ctx, void *arg);

/**
 * @brief Shared render helper for all gps_metrics_result_* functions.
 *
 * Acquires two buffer-pool buffers, initialises the strbf, calls @p fn,
 * then releases the buffers.  The files_opened guard is centralised here.
 *
 * @param caller  __FUNCTION__ string for error messages
 * @param fn      Content-specific format callback
 * @param arg     Arbitrary pointer forwarded to @p fn (the metrics struct)
 */
static void gps_metrics_render(const char *caller, gps_metrics_format_fn fn, void *arg) {
    if (!gps->files_opened) {
        ELOG(TAG, "[%s] files not open", caller);
        return;
    }

    logger_buffer_handle_t msg_handle = {0}, scratch_handle = {0};
    if (get_gps_message_buffer(&msg_handle) != ESP_OK ||
        get_gps_scratch_buffer(&scratch_handle) != ESP_OK) {
        ELOG(TAG, "Failed to get GPS buffers for %s", caller);
        release_gps_buffer(&msg_handle);
        release_gps_buffer(&scratch_handle);
        return;
    }

    gps_metrics_ctx_t ctx = {
        .message      = (char *)msg_handle.buffer,
        .message_size = msg_handle.size,
        .tekst        = (char *)scratch_handle.buffer,
    };
    strbf_inits(&ctx.sb, ctx.message, ctx.message_size);

    fn(&ctx, arg);

    release_gps_buffer(&msg_handle);
    release_gps_buffer(&scratch_handle);
}

// ============================================================================
// Format callbacks - one per metric type; contain only the content logic
// ============================================================================

static void fmt_result_dist(gps_metrics_ctx_t *ctx, void *arg) {
    struct gps_speed_by_dist_s *me = (struct gps_speed_by_dist_s *)arg;
    strbf_t *sb = &ctx->sb;
    char *tekst = ctx->tekst;
    const char *units = get_speed_unit_str(g_rtc_config.gps.speed_unit);
    const char *unit = " M";
    *tekst = 0;
    result_speed_avg(&me->speed, sb, units, unit, me->distance_window, tekst);
    for (int i = 9; i > 4; i--) {
        gps_run_t *run = &me->speed.runs[i];
        f3_to_char(get_spd(run->avg_speed), tekst);
        strbf_puts(sb, tekst);
        strbf_puts(sb, units);
        strbf_putc(sb, ' ');
        time_to_char_hms((int)run->time.hour, (int)run->time.minute, (int)run->time.second, tekst);
        strbf_puts(sb, tekst);
        strbf_puts(sb, " Distance: ");
        f2_to_char(get_distance_m(run->data.dist.dist, g_rtc_config.ubx.output_rate), tekst);
        strbf_puts(sb, tekst);
        strbf_puts(sb, " Msg_nr: ");
        strbf_putl(sb, run->data.dist.message_nr);
        strbf_puts(sb, " Samples: ");
        strbf_putl(sb, run->data.dist.nr_samples);
        strbf_puts(sb, " Run: ");
        strbf_putl(sb, run->nr);
        strbf_puts(sb, unit);
        strbf_putl(sb, me->distance_window);
        strbf_puts(sb, "\n");
        WRITETXT(strbf_finish(sb), sb->cur - sb->start);
        FUNC_ENTRY_ARGSD(TAG, "%s", sb->start);
        strbf_reset(sb);
    }
}

static void fmt_result_time(gps_metrics_ctx_t *ctx, void *arg) {
    struct gps_speed_by_time_s *me = (struct gps_speed_by_time_s *)arg;
    strbf_t *sb = &ctx->sb;
    char *tekst = ctx->tekst;
    const char *units = get_speed_unit_str(g_rtc_config.gps.speed_unit);
    const char *unit = " S";
    *tekst = 0;
    result_speed_avg(&me->speed, sb, units, unit, me->time_window, tekst);
    for (int i = 9; i > 4; i--) {
        gps_run_t *run = &me->speed.runs[i];
        f3_to_char(get_spd(run->avg_speed), tekst);
        strbf_puts(sb, tekst);
        strbf_puts(sb, units);
        strbf_putc(sb, ' ');
        time_to_char_hms((int)run->time.hour, (int)run->time.minute, (int)run->time.second, tekst);
        strbf_puts(sb, tekst);
        strbf_puts(sb, " Run:");
        strbf_putl(sb, run->nr);
        strbf_puts(sb, unit);
        strbf_putl(sb, me->time_window);
        if (g_rtc_config.ubx.msgout_sat) {
            strbf_sprintf(sb, " CNO Max: %zu Avg: %zu Min: %zu nr Sat: %zu",
                run->data.time.Max_cno, run->data.time.Mean_cno,
                run->data.time.Min_cno, run->data.time.Mean_numSat);
        }
        strbf_puts(sb, "\n");
        WRITETXT(ctx->message, sb->cur - sb->start);
        FUNC_ENTRY_ARGSD(TAG, "%s", sb->start);
        strbf_reset(sb);
    }
}

static void fmt_result_alfa(gps_metrics_ctx_t *ctx, void *arg) {
    struct gps_speed_by_dist_s *me = (struct gps_speed_by_dist_s *)arg;
    gps_speed_by_alfa_t *A = me->alfa;
    strbf_t *sb = &ctx->sb;
    char *tekst = ctx->tekst;
    const char *units = get_speed_unit_str(g_rtc_config.gps.speed_unit);
    const char *unit = " A";
    result_speed_avg(&A->speed, sb, units, unit, me->distance_window, tekst);
    for (int i = 9; i > 4; i--) {
        gps_run_t *run = &A->speed.runs[i];
        f3_to_char(get_spd(run->avg_speed), tekst);
        strbf_puts(sb, tekst);
        strbf_puts(sb, units);
        strbf_putc(sb, ' ');
        f2_to_char(sqrt((float)run->data.alfa.real_distance), tekst);
        strbf_puts(sb, tekst);
        strbf_puts(sb, " m ");
        strbf_putl(sb, get_distance_m(run->data.alfa.dist, g_rtc_config.ubx.output_rate));
        strbf_puts(sb, " m ");
        time_to_char_hms((int)run->time.hour, (int)run->time.minute, (int)run->time.second, tekst);
        strbf_puts(sb, tekst);
        strbf_puts(sb, " Run: ");
        strbf_putl(sb, run->nr);
        strbf_puts(sb, " Msg_nr: ");
        strbf_putl(sb, run->data.alfa.message_nr);
        strbf_puts(sb, unit);
        strbf_putl(sb, A->distance_window);
        strbf_puts(sb, "\n");
        WRITETXT(strbf_finish(sb), sb->cur - sb->start);
        FUNC_ENTRY_ARGSD(TAG, "%s", sb->start);
        strbf_reset(sb);
    }
}

static void fmt_result_max(gps_metrics_ctx_t *ctx, void *arg) {
    (void)arg;
    strbf_t *sb = &ctx->sb;
    char *tekst = ctx->tekst;
    gps_run_t *run = &gps->max_speed;
    strbf_puts(sb, strings[0]);
    strbf_puts(sb, " Max speed ");
    f3_to_char(get_spd(run->avg_speed), tekst);
    strbf_puts(sb, tekst);
    strbf_puts(sb, get_speed_unit_str(g_rtc_config.gps.speed_unit));
    strbf_putc(sb, ' ');
    time_to_char_hms((int)run->time.hour, (int)run->time.minute, (int)run->time.second, tekst);
    strbf_puts(sb, tekst);
    strbf_puts(sb, " Run: ");
    strbf_putl(sb, run->nr);
    strbf_putc(sb, ' ');
    strbf_puts(sb, strings[0]);
    strbf_puts(sb, "\n");
    WRITETXT(strbf_finish(sb), sb->cur - sb->start);
    FUNC_ENTRY_ARGSD(TAG, "%s", sb->start);
}

// ============================================================================
// Public entry points - thin wrappers that validate input then delegate
// ============================================================================

static void gps_metrics_result_dist(struct gps_speed_by_dist_s *me) {
    FUNC_ENTRY(TAG);
    if (!me) return;
    gps_metrics_render(__FUNCTION__, fmt_result_dist, me);
}

static void gps_metrics_result_time(struct gps_speed_by_time_s *me) {
    FUNC_ENTRY(TAG);
    if (!me) return;
    gps_metrics_render(__FUNCTION__, fmt_result_time, me);
}

static void gps_metrics_result_alfa(struct gps_speed_by_dist_s *me) {
    FUNC_ENTRY(TAG);
    if (!me || !me->alfa) return;
    gps_metrics_render(__FUNCTION__, fmt_result_alfa, me);
}

static void gps_metrics_result_max(void) {
    FUNC_ENTRY(TAG);
    gps_metrics_render(__FUNCTION__, fmt_result_max, NULL);
}

void gps_speed_metrics_save_session(void) {
    FUNC_ENTRY(TAG);
    if (g_rtc_config.gps.log_enables.bits.log_txt && gps->log_config->filefds[sd_log_txt] > 0) {
        session_info(gps, &gps->Ublox);
        gps_metrics_result_max();
        for(uint8_t i = 0, j = gps->num_speed_metrics; i < j; i++) {
            if (gps->speed_metrics[i].type == GPS_SPEED_TYPE_TIME) {
                gps_metrics_result_time(gps->speed_metrics[i].handle.time);
            } else if (gps->speed_metrics[i].type & (GPS_SPEED_TYPE_DIST | GPS_SPEED_TYPE_ALFA)) {
                gps_metrics_result_dist(gps->speed_metrics[i].handle.dist);
                if (gps->speed_metrics[i].type & (GPS_SPEED_TYPE_ALFA)) {
                    gps_metrics_result_alfa(gps->speed_metrics[i].handle.dist);
                }
            }
        }
    }
}

#endif
