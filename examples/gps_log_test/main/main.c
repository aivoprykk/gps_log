/**
 * GPS Log Integration Test - Real Pipeline Stress Test
 * 
 * Architecture:
 *   Mock: UART input stream (UBX bytes with realistic GPS track)
 *   Real: gpsTask → UBX parser → push_gps_data() → speed calculations → file I/O
 * 
 * This test uses the REAL gps_log module pipeline end-to-end.
 * Only the UART GPS input is mocked - everything else is production code.
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <sys/stat.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_event.h"

#include "config.h"
// #include "common.h"
#include "gps_log.h"         // Real gpsTask, gps_init, gps_start
#include "gps_user_cfg.h"     // For c_gps_cfg
#include "gps_data.h"         // For gps_data_s (Ublox struct)
#include "gps_speed_data.h"   // For speed_metrics enums (time_10s, dist_500m, etc.)
#include "context.h"
#include "vfs.h"
#include "gps_track_generator.h"
#include "ubx_nav_pvt_generator.h"
#include "ubx.h"
#include "driver/uart.h"
#include <math.h>

#define C_LOG_LEVEL 2
#include "common_log.h"
#include "unified_config.h"
#include "config_lock.h"

static const char *TAG = "gps_test";

// GPS track state for mock UART
static gps_track_state_t track_state;

// Global GPS context (used by real gps_log module)
extern gps_context_t *gps;

// Statistics
typedef struct {
    uint32_t start_time_ms;
    uint32_t last_stats_ms;
    uint32_t samples_generated;
    uint32_t test_duration_ms;
} test_stats_t;

static test_stats_t stats = {0};

struct main_ctx_s {
    int app_mode;
} main_ctx_t;

struct main_ctx_s m_app_ctx = {1}; // Required by vfs module

// Initialize filesystem using production VFS
static esp_err_t init_filesystem(void) {
    ILOG(TAG, "Initializing VFS (production filesystem handling)");
    
    // Use real vfs_init() which handles all filesystem types
    // Note: Expect "Partition not found" errors for partitions not in test partition table
    // (e.g., FATFS, SPIFFS). Only SD card is required for this test.
    int ret = vfs_init();
    if (ret != ESP_OK) {
        ELOG(TAG, "VFS initialization failed: %d", ret);
        return ESP_FAIL;
    }
    
    // VFS context is now initialized with mounted filesystems
    // Check what got mounted
    for (int i = 0; i < VFS_MAX_PARTS; i++) {
        if (vfs_ctx.parts[i].mount_point) {
            ILOG(TAG, "Mounted: %s (type=%d, total=%llu, free=%llu)",
                 vfs_ctx.parts[i].mount_point,
                 vfs_ctx.parts[i].part_type,
                 vfs_ctx.parts[i].total_bytes,
                 vfs_ctx.parts[i].free_bytes);
        }
    }
    
    // GPS log will use the configured partition (typically FATFS or SD card)
    if (!vfs_ctx.parts[vfs_ctx.gps_log_part].mount_point) {
        ELOG(TAG, "No filesystem available for GPS logging");
        return ESP_FAIL;
    }
    
    ILOG(TAG, "GPS will log to: %s", vfs_ctx.parts[vfs_ctx.gps_log_part].mount_point);
    
    return ESP_OK;
}

// Print test statistics
static void print_stats(const char *phase_name, uint8_t num_satellites) {
    uint32_t now_ms = get_millis();
    uint32_t elapsed_ms = now_ms - stats.start_time_ms;
    uint32_t period_ms = now_ms - stats.last_stats_ms;
    
    printf("\n========== TEST STATISTICS ==========\n");
    printf("Test Phase: %s\n", phase_name);
    printf("Test Rate: %d Hz | Elapsed: %d s / %d s\n",
           TEST_GPS_RATE, (int)(elapsed_ms/1000), (int)(TOTAL_TEST_DURATION_MS/1000));
    printf("Satellites: %d | Track Phase: %s\n", num_satellites, gps_track_phase_name(&track_state));
    printf("Track: lat=%.6f lon=%.6f speed=%.1f m/s heading=%.1f°\n",
           track_state.lat, track_state.lon, track_state.speed_ms, track_state.heading_deg);
    printf("Samples Generated: %lu (%.1f samples/s)\n",
           stats.samples_generated,
           period_ms > 0 ? (float)stats.samples_generated * 1000.0f / (float)elapsed_ms : 0.0f);
    
    // GPS module stats (from real gpsTask)
    if (gps && gps->ubx_device) {
        ubx_ctx_t *ubx = gps->ubx_device;
        printf("\nREAL GPS PIPELINE STATS:\n");
        printf("  UBX Messages: total=%" PRIu32 " pvt=%" PRIu32 " sat=%" PRIu32 " errors=%" PRIu32 "\n",
               ubx->ubx_msg.count_msg,
               log_p_lctx.count_nav_pvt,
               ubx->ubx_msg.count_nav_sat,
               ubx->ubx_msg.count_err);
        printf("  GPS Context: time_set=%d signal_ok=%d files_open=%d run_count=%d\n",
               gps->time_set, gps->signal_ok, gps->files_opened, gps->run_count);
        
        // Access speed metrics via convenience macros
        float m500 = dist_display_max_speed(dist_500m);
        float s10 = time_display_max_speed(time_10s);
        float alfa = alfa_display_max_speed(alfa_500m);
        
        printf("  GPS Data: total_dist=%.1f M500=%.1f S10=%.1f alfa=%.1f\n",
               gps->Ublox.total_distance, m500, s10, alfa);
    }
    
    printf("=====================================\n\n");
    
    // Print detailed three-tier pipeline statistics
#if defined(CONFIG_GPS_TIMER_STATS_ENABLED)
    if (gps && gps->ubx_device) {
        gps_log_print_all_stats((void*)gps->ubx_device);
    }
#endif
    
    stats.last_stats_ms = now_ms;
}

// Helper to inject UBX message bytes into GPS module's UART handler
// This simulates UART data reception by directly posting to the UART event queue
static void inject_ubx_message(const uint8_t *data, size_t len) {
    if (!gps || !gps->ubx_device || !gps->ubx_device->uart_event_queue) {
        return;
    }
    
    // Post UART_DATA event to the GPS module's event queue
    uart_event_t event = {
        .type = UART_DATA,
        .size = len,
    };
    
    // Write data to UART buffer (simulated)
    // The real UART handler would read from hardware, we inject directly
    uart_write_bytes(gps->ubx_device->uart_num, (const char*)data, len);
    
    // Notify the UART event task
    xQueueSend(gps->ubx_device->uart_event_queue, &event, portMAX_DELAY);
}

// Mock UART data generator task
// Generates UBX NAV-PVT messages based on GPS track
static void mock_uart_task(void *arg) {
    ILOG(TAG, "Mock UART task started");
    
    TickType_t last_wake = xTaskGetTickCount();
    TickType_t interval_ticks = pdMS_TO_TICKS(MOCK_DATA_INTERVAL_MS);
    
    double lat = TRACK_START_LAT;
    double lon = TRACK_START_LON;
    float speed_ms = 0.0f;
    float heading = 0.0f;
    uint32_t itow = 0;
    uint8_t num_sv = INIT_START_SATELLITES;
    uint8_t msg_counter = 0;
    
    // Test phase tracking
    enum {
        PHASE_INIT,          // GPS initialization: acquiring satellites
        PHASE_WAIT_FIX,      // Waiting for first fix (5+ satellites)
        PHASE_STATIONARY,    // Stationary after fix (GPS_delay increment)
        PHASE_SPEEDSURFING   // Active speedsurfing test
    } test_phase = INIT_PHASE_ENABLED ? PHASE_INIT : PHASE_SPEEDSURFING;
    
    uint32_t phase_start_ms = get_millis();
    uint32_t last_sat_increment_ms = phase_start_ms;
    
    // Buffers for UBX messages
    uint8_t pvt_msg[100];
    uint8_t dop_msg[26];
    uint8_t sat_msg[200];
    
    while (get_millis() - stats.start_time_ms < TOTAL_TEST_DURATION_MS) {
        uint32_t now_ms = get_millis();
        uint32_t phase_elapsed_ms = now_ms - phase_start_ms;
        
        // === PHASE MANAGEMENT ===
        switch (test_phase) {
            case PHASE_INIT:
                // Gradually acquire satellites
                if (now_ms - last_sat_increment_ms >= INIT_SAT_INTERVAL_MS) {
                    if (num_sv < INIT_TARGET_SATELLITES) {
                        num_sv += INIT_SAT_INCREMENT;
                        ILOG(TAG, "Satellites acquired: %d/%d", num_sv, INIT_TARGET_SATELLITES);
                        last_sat_increment_ms = now_ms;
                    }
                    if (num_sv >= INIT_MIN_SATS_FOR_FIX) {
                        ILOG(TAG, "=== FIRST FIX ACHIEVED (%d satellites) ===", num_sv);
                        test_phase = PHASE_WAIT_FIX;
                        phase_start_ms = now_ms;
                    }
                }
                // Stay stationary during initialization
                speed_ms = 0.0f;
                break;
                
            case PHASE_WAIT_FIX:
                // GPS module sets time_set, signal_ok flags
                // Wait a moment for state transitions
                if (phase_elapsed_ms >= 3000) {  // 3 seconds after fix
                    ILOG(TAG, "=== ENTERING STATIONARY PHASE ===");
                    test_phase = PHASE_STATIONARY;
                    phase_start_ms = now_ms;
                }
                speed_ms = 0.0f;
                break;
                
            case PHASE_STATIONARY:
                // Stay stationary to allow GPS_delay counter to increment
                // This triggers file opening in real GPS module
                if (phase_elapsed_ms >= INIT_STATIONARY_TIME_MS) {
                    ILOG(TAG, "=== STARTING SPEEDSURFING TEST ===");
                    ILOG(TAG, "Track: 600m loop, %d Hz", TEST_GPS_RATE);
                    test_phase = PHASE_SPEEDSURFING;
                    phase_start_ms = now_ms;
                    gps_track_reset(&track_state);  // Reset track to start
                }
                speed_ms = 0.0f;
                break;
                
            case PHASE_SPEEDSURFING:
                // Generate speedsurfing track
                if (!gps_track_next_sample(&track_state, &lat, &lon, &speed_ms, &heading)) {
                    ILOG(TAG, "Track complete - resetting for another lap");
                    gps_track_reset(&track_state);
                }
                break;
        }
        
        
        // Generate UBX NAV-PVT message with current GPS data
        size_t pvt_len = generate_ubx_nav_pvt(pvt_msg, lat, lon, speed_ms, heading, itow, num_sv);
        if (pvt_len > 0) {
            inject_ubx_message(pvt_msg, pvt_len);
            
            // Debug: Log key info for troubleshooting
            static uint32_t pvt_count = 0;
            if ((pvt_count++ % 50) == 0) {  // Log every 50th message
                ILOG(TAG, "Injected PVT #%lu: class=0x%02X id=0x%02X numSV=%d lat=%.6f speed=%.2f",
                     pvt_count, pvt_msg[2], pvt_msg[3], num_sv, lat, speed_ms);
            }
        }
        
        // Every 5th message, also send NAV-DOP
        // if ((msg_counter % 5) == 0) {
            size_t dop_len = generate_ubx_nav_dop(dop_msg, itow);
            if (dop_len > 0) {
                inject_ubx_message(dop_msg, dop_len);
            }
        // }
        
        // Every 10th message, send NAV-SAT
        if ((msg_counter % 10) == 0 && num_sv > 0) {
            size_t sat_len = generate_ubx_nav_sat(sat_msg, itow, num_sv);
            if (sat_len > 0) {
                inject_ubx_message(sat_msg, sat_len);
            }
        }
        
        stats.samples_generated++;
        msg_counter++;
        itow += (1000 / TEST_GPS_RATE);  // Increment iTOW by sample interval
        
        // Print stats periodically
        if ((get_millis() - stats.last_stats_ms) >= STATS_INTERVAL_MS) {
            const char *phase_str;
            switch (test_phase) {
                case PHASE_INIT: phase_str = "GPS INITIALIZATION"; break;
                case PHASE_WAIT_FIX: phase_str = "WAITING FOR FIX"; break;
                case PHASE_STATIONARY: phase_str = "STATIONARY (GPS_delay increment)"; break;
                case PHASE_SPEEDSURFING: phase_str = "SPEEDSURFING TEST"; break;
                default: phase_str = "UNKNOWN"; break;
            }
            print_stats(phase_str, num_sv);
        }
        
        // Wait for next interval
        vTaskDelayUntil(&last_wake, interval_ticks);
    }
    
    ILOG(TAG, "Test duration reached");
    
    // Give GPS pipeline time to process remaining messages
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    print_stats("TEST COMPLETE", num_sv);  // Final stats
    
    // Print test results summary
    printf("\n========== TEST RESULTS ==========\n");
    uint32_t total_time = get_millis() - stats.start_time_ms;
    float throughput = (float)stats.samples_generated * 1000.0f / (float)total_time;
    printf("Total samples generated: %lu\n", stats.samples_generated);
    printf("Test duration: %lu ms (%.1f s)\n", total_time, total_time / 1000.0f);
    printf("Average throughput: %.1f samples/sec\n", throughput);
    
    if (gps && gps->ubx_device) {
        float msg_success_rate = 0.0f;
        if (gps->ubx_device->ubx_msg.count_msg > 0) {
            msg_success_rate = (float)(gps->ubx_device->ubx_msg.count_nav_pvt) * 100.0f / 
                              (float)(gps->ubx_device->ubx_msg.count_msg);
        }
        printf("Message success rate: %.1f%%\n", msg_success_rate);
        printf("Parse errors: %lu\n", gps->ubx_device->ubx_msg.count_err);
    }
    printf("==================================\n");
    
    // Shutdown GPS module
    ILOG(TAG, "Shutting down GPS module");
    gps_shut_down();
    
    ILOG(TAG, "Test complete - SUCCESS");
    vTaskDelete(NULL);
}

void app_main(void) {
    ILOG(TAG, "=== GPS Log Integration Test ===");
    ILOG(TAG, "");
    
    // Print test configuration
    if (INIT_PHASE_ENABLED) {
        ILOG(TAG, "TEST PHASES:");
        ILOG(TAG, "  1. Initialization: %d satellites acquisition (0→%d, %dms interval)", 
             INIT_TARGET_SATELLITES, INIT_TARGET_SATELLITES, INIT_SAT_INTERVAL_MS);
        ILOG(TAG, "  2. First Fix: %d+ satellites, GPS time set", INIT_MIN_SATS_FOR_FIX);
        ILOG(TAG, "  3. Stationary: %ds for GPS_delay increment & file opening", 
             INIT_STATIONARY_TIME_MS/1000);
        ILOG(TAG, "  4. Speedsurfing: %ds at %d Hz, 600m loop", 
             SPEEDSURFING_DURATION_MS/1000, TEST_GPS_RATE);
        ILOG(TAG, "");
        ILOG(TAG, "Total Duration: ~%d seconds", TOTAL_TEST_DURATION_MS/1000);
    } else {
        ILOG(TAG, "Speedsurfing Test Only: %d Hz, %d seconds", 
             TEST_GPS_RATE, SPEEDSURFING_DURATION_MS/1000);
    }
    
    ILOG(TAG, "Save Frequency: Every %d seconds", TEST_GPS_RATE);
    ILOG(TAG, "Track: 600m loop with speed profile 0→70→20→70→0 km/h");
    ILOG(TAG, "");
    
    // Initialize config lock (required by config system)
    config_lock_init();
    
    // Initialize event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Initialize filesystem
    if (init_filesystem() != ESP_OK) {
        ELOG(TAG, "Filesystem init failed");
        return;
    }
    
    // Initialize GPS context
    if (!gps) {
        gps = (gps_context_t*)calloc(1, sizeof(gps_context_t));
        if (!gps) {
            ELOG(TAG, "Failed to allocate GPS context");
            return;
        }
    }
    
    // Initialize real GPS module
    ILOG(TAG, "Initializing GPS module");
    gps_init(gps);
    
    // Configure GPS logging via gps_user_cfg
    g_rtc_config.gps.log_ubx = 1;
    g_rtc_config.gps.log_txt = 1;
    g_rtc_config.ubx.output_rate = TEST_UBX_OUTPUT_RATE;  // Configure GPS update rate
    strncpy(g_rtc_config.gps.ubx_file, "test", sizeof(g_rtc_config.gps.ubx_file) - 1);
    gps_config_fix_values();
    
    // Initialize GPS track generator
    gps_track_init(&track_state, (float)TEST_GPS_RATE, TRACK_START_LAT, TRACK_START_LON);
    
    // Initialize test stats
    stats.start_time_ms = get_millis();
    stats.last_stats_ms = stats.start_time_ms;
    stats.samples_generated = 0;
    stats.test_duration_ms = TOTAL_TEST_DURATION_MS;
    
    // Start mock UART task FIRST to pre-fill UART buffer
    xTaskCreate(mock_uart_task, "mock_uart", 4096, NULL, 5, NULL);
    
    // Give mock task time to generate initial data
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Now start GPS processing (ubx_initial_read will have data available)
    ILOG(TAG, "Starting GPS processing task");
    if (gps_start() != 0) {
        ELOG(TAG, "Failed to start GPS");
        free(gps);
        return;
    }
    
    ILOG(TAG, "Test running... (will auto-stop after %d seconds)", TOTAL_TEST_DURATION_MS/1000);
    
    // Main task done - mock_uart_task will handle the test
}
