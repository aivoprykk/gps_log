#include "log_private.h"
#include "ubx.h"
#include "ubx_msg.h"
// #include "esp_mac.h"
#include "esp_timer.h"
#include "vfs.h"

#if defined(CONFIG_GPS_LOG_ENABLED)

static const char *TAG = "gps_log";

#define GPS_TASK_DEBUG 1
#if defined(CONFIG_GPS_TIMER_STATS_ENABLED)
static ubx_msg_stats_t cur_msg_stats = {0};
static ubx_msg_stats_t prev_msg_stats = {0};
static ubx_msg_stats_t period_msg_stats = {0};

static uint32_t prev_millis = 0;
static esp_timer_handle_t gps_periodic_timer = 0;

void gps_log_print_stats(uint32_t period_ms, uint8_t expected_hz) {

	// Calculate period statistics and store in period_msg_stats
	period_msg_stats.count_nav_pvt =
		cur_msg_stats.count_nav_pvt -
		prev_msg_stats.count_nav_pvt; // Valid PVTs processed
	period_msg_stats.count_nav_dop =
		cur_msg_stats.count_nav_dop -
		prev_msg_stats.count_nav_dop; // DOPs processed
	period_msg_stats.count_ok =
		cur_msg_stats.count_ok - prev_msg_stats.count_ok; // File push successes
	period_msg_stats.count_err = cur_msg_stats.count_err -
								 prev_msg_stats.count_err; // File push failures
	period_msg_stats.count =
		cur_msg_stats.count_nav_pvt +
		cur_msg_stats.count_nav_dop; // Total valid messages processed

	float period_s = (float)period_ms / 1000.0f;
	float throughput =
		period_s > 0.0f ? (float)period_msg_stats.count / period_s : 0.0f;
	float expected_count = (expected_hz * (float)period_s) * 2 +
						   1; // Total messages expected in this period
	float loss_pct =
		expected_count > 0.0f
			? (1.0f - (float)period_msg_stats.count / expected_count) * 100.0f
			: 0.0f;
	if (loss_pct < 0.0f)
		loss_pct = 0.0f; // Clamp negative loss (happens when rate > expected)

	printf("[GPS] ========== GPS LOG STATS ==========\n");
	printf("[GPS] GPS messages: %.1f msg/s (expected:  %.1f msg/s at %" PRIu8
		   " Hz, loss: %.1f%%)\n",
		   throughput, expected_count, expected_hz, loss_pct);
	printf("[GPS] Valid messages (period): PVT=%" PRIu32 " DOP=%" PRIu32
		   " (passed GPS validity checks)\n",
		   period_msg_stats.count_nav_pvt, period_msg_stats.count_nav_dop);
	printf("[GPS] File I/O (period): ok=%" PRIu32 " fail=%" PRIu32
		   " (%.1f%% loss)\n",
		   period_msg_stats.count_ok, period_msg_stats.count_err,
		   (period_msg_stats.count_ok + period_msg_stats.count_err) > 0
			   ? (float)period_msg_stats.count_err * 100.0f /
					 (float)(period_msg_stats.count_ok +
							 period_msg_stats.count_err)
			   : 0.0f);
	printf("[GPS] Totals: Valid PVT=%" PRIu32 " DOP=%" PRIu32
		   " | File ok=%" PRIu32 " err=%" PRIu32 "\n",
		   cur_msg_stats.count_nav_pvt, cur_msg_stats.count_nav_dop,
		   cur_msg_stats.count_ok, cur_msg_stats.count_err);
	printf("[GPS] ========================================\n");
}

void gps_log_print_all_stats(void *arg) {
	uint32_t millis = get_millis();
	uint32_t period = millis - prev_millis;
	// Update previous snapshot
	prev_millis = millis;
	prev_msg_stats = cur_msg_stats;

	uint8_t expected_hz = g_rtc_config.ubx.output_rate;

	// Print stats in order: UART RX → UBX Protocol → GPS Application
	printf("\n");
	if (arg)
		ubx_uart_print_buffer_stats(
			(struct ubx_ctx_s *)arg);		   // UART buffer health
	ubx_uart_print_stats(period, expected_hz); // Layer 1: UART message stats
	ubx_print_stats(period, expected_hz);
	gps_log_print_stats(period, expected_hz); // Layer 2: UBX decoded messages
	printf("\n");
}

#else
void gps_log_print_all_stats(void *arg) {}
void gps_log_print_stats(uint32_t period_ms, uint8_t expected_hz) {}
#endif

typedef struct {
	uint16_t old_run_count;
	uint8_t gps_log_delay;
	uint32_t old_nav_pvt_itow;
	uint32_t next_time_sync;
	bool ubx_restart_requested;
	int output_rate_swp;
	uint32_t last_flush_time;
	bool gps_task_is_running;
	TaskHandle_t gps_task_handle;
	uint16_t ubx_fail_count; // widen to avoid overflow when accumulating error
							 // penalties
	uint8_t gps_initialized;
	uint8_t gps_started;
	uint8_t gps_events_registered;
} log_context_t;

static log_context_t lctx = {.old_run_count = 0,
							 .gps_log_delay = 0,
							 .old_nav_pvt_itow = 0,
							 .next_time_sync = 0,
							 .ubx_restart_requested = false,
							 .output_rate_swp = 0,
							 .last_flush_time = 0,
							 .gps_task_is_running = false,
							 .gps_task_handle = NULL,
							 .ubx_fail_count = 0,
							 .gps_initialized = 0,
							 .gps_started = 0,
							 .gps_events_registered = 0};

gps_context_t *gps = 0;
static bool gps_config_observer_registered = false;

ESP_EVENT_DEFINE_BASE(GPS_LOG_EVENT); // declaration of the LOG_EVENT family

// Forward declarations for file handlers
static void gps_file_open_handler(void *handler_arg, esp_event_base_t base,
								  int32_t id, void *event_data);
static void gps_file_flush_handler(void *handler_arg, esp_event_base_t base,
								   int32_t id, void *event_data);

// Adjust GPS output rate based on available partition space (event-driven from
// partition change, not hot loop) Called when: partition changes, UBX device
// initializes, or partition space is updated
static void gps_adjust_rate_for_partition_space(void) {
	// Check if partition is available
	if (vfs_ctx.gps_log_part >= VFS_PART_MAX || !gps) {
		return;
	}

	uint64_t free_bytes = vfs_ctx.parts[vfs_ctx.gps_log_part].free_bytes;

	// Drop rate if space too low (< 700 KB) and rate is high (>= 5 Hz)
	if (!lctx.output_rate_swp &&
		g_rtc_config.ubx.output_rate >= UBX_OUTPUT_5HZ &&
		free_bytes < TO_K_UL(700)) {
		WLOG(TAG,
			 "[%s] vfs log part %s free space too low: %llu! Dropping rate.",
			 __FUNCTION__, vfs_ctx.parts[vfs_ctx.gps_log_part].mount_point,
			 free_bytes);
		lctx.output_rate_swp =
			g_rtc_config.ubx.output_rate;			   // Save current rate
		g_rtc_config.ubx.output_rate = UBX_OUTPUT_5HZ; // Drop to 5 Hz
		ubx_set_gnss_and_rate(gps->ubx_device, g_rtc_config.ubx.gnss,
							  g_rtc_config.ubx.output_rate);
		// Notify UI about low storage condition
		esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_LOW_STORAGE, NULL, 0, 0);
	}
	// Restore rate when space recovers (>= 700 KB)
	else if (lctx.output_rate_swp &&
			 g_rtc_config.ubx.output_rate < UBX_OUTPUT_5HZ &&
			 free_bytes >= TO_K_UL(700)) {
		WLOG(TAG,
			 "[%s] vfs log part %s free space ok again: %llu! Restoring rate.",
			 __FUNCTION__, vfs_ctx.parts[vfs_ctx.gps_log_part].mount_point,
			 free_bytes);
		// Restore to one step above saved rate (hysteresis to avoid
		// oscillation)
		g_rtc_config.ubx.output_rate =
			(lctx.output_rate_swp == UBX_OUTPUT_5HZ)	? UBX_OUTPUT_10HZ
			: (lctx.output_rate_swp == UBX_OUTPUT_10HZ) ? UBX_OUTPUT_16HZ
			: (lctx.output_rate_swp == UBX_OUTPUT_16HZ) ? UBX_OUTPUT_20HZ
														: UBX_OUTPUT_1HZ;
		lctx.output_rate_swp = 0;
		ubx_set_gnss_and_rate(gps->ubx_device, g_rtc_config.ubx.gnss,
							  g_rtc_config.ubx.output_rate);
	}
}

// Unified GPS->VFS processor (registered via vfs_register_work_interface)
static void gps_vfs_processor(vfs_work_type_t type, void *arg) {
	gps_context_t *g = (gps_context_t *)arg;
	if (!g)
		return;
	switch (type) {
	case VFS_WORK_OPEN_FILES:
		open_files(g);
		break;
	case VFS_WORK_CLOSE_FILES:
		gps_speed_metrics_save_session();
		close_files(g);
		break;
	case VFS_WORK_FLUSH_FILES:
		flush_files(g);
		break;
	case VFS_WORK_PARTITION_CHANGED:
		// Partition changed: evaluate rate adjustment based on new partition's
		// free space
		gps_adjust_rate_for_partition_space();
		gps_speed_metrics_save_session();
		close_files(g);
		open_files(g);
		break;
	case VFS_WORK_SAVE_SESSION:
		gps_speed_metrics_save_session();
		break;
	default:
		break;
	}
}

#if (C_LOG_LEVEL <= LOG_INFO_NUM)
static const char *const _gps_log_event_strings[] = {
	GPS_LOG_EVENT_LIST(STRINGIFY)};
const char *gps_log_event_strings(int id) {
	return id < lengthof(_gps_log_event_strings) ? _gps_log_event_strings[id]
												 : "GPS_LOG_EVENT_UNKNOWN";
}
#else
const char *gps_log_event_strings(int id) { return "GPS_LOG_EVENT"; }
#endif

// ============================================================================
// TIME MANAGEMENT
// ============================================================================

static esp_err_t set_time(nav_pvt_t *nav_pvt, float time_offset) {
	FUNC_ENTRY_ARGSD(TAG, "offset:%.1fh", time_offset);
	// Data validity checks (timing already checked by caller)
	if (!nav_pvt || !nav_pvt->numSV)
		return ESP_FAIL;
	if (nav_pvt->year < 2023) {
		WLOG(TAG, "[%s] invalid time data: %" PRIu16 "-%" PRIu8 "-%" PRIu8 "",
			 __func__, nav_pvt->year, nav_pvt->month, nav_pvt->day);
		return ESP_FAIL;
	}
	// ILOG(TAG, "[%s] sats: %" PRIu8, __FUNCTION__, nav_pvt->numSV);
	//  time_t unix_timestamp = 0;  // a timestamp in seconds
#if defined(DLS)
	// summertime is on march 26 2023 2 AM, see
	// https://www.di-mgt.com.au/wclock/help/wclo_tzexplain.html
	setenv("TZ", "CET0CEST,M3.5.0/2,M10.5.0/3",
		   1); // timezone UTC = CET, Daylightsaving ON :
			   // TZ=CET-1CEST,M3.5.0/2,M10.5.0/3
#else
	setenv("TZ", "UTC", 0);
#endif
	tzset(); // this works for CET, but TZ string is different for every Land /
			 // continent....
	struct tm my_time = {
		.tm_sec = nav_pvt->second,
		.tm_min = nav_pvt->minute,
		.tm_hour = nav_pvt->hour,
		.tm_mday = nav_pvt->day,
		.tm_mon = nav_pvt->month - 1, // mktime needs months 0 - 11
		.tm_year = nav_pvt->year -
				   1900, // mktime needs years since 1900, so deduct 1900
		.tm_isdst = -1,	 // daylight saving time flag
	};
	int ret = c_set_time(&my_time, NANO_TO_US_ROUND(nav_pvt->nano),
						 time_offset); // set the time in the struct tm
	if (ret) {
		ELOG(TAG, "[%s] Failed to set time from gps", __FUNCTION__);
		return ESP_FAIL;
	}
	// unix_timestamp = mktime(&my_time);  // mktime returns local time, so TZ
	// is important !!! int64_t utc_ms = unix_timestamp * 1000LL +
	// NANO_TO_MILLIS_ROUND(nav_pvt->nano); int64_t time_us =
	// (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
	FUNC_ENTRY_ARGSD(TAG,
					 "GPS pvt time: %d-%02d-%02d %02d:%02d:%02d %" PRId32
					 " %" PRIu8 ", offset: %f",
					 nav_pvt->year, nav_pvt->month, nav_pvt->day, nav_pvt->hour,
					 nav_pvt->minute, nav_pvt->second, nav_pvt->nano,
					 nav_pvt->id, time_offset);
	// WLOG(TAG, "GPS tm time : %d-%02d-%02d %02d:%02d:%02d %" PRId32 "",
	// my_time.tm_year+1900, my_time.tm_mon+1, my_time.tm_mday, my_time.tm_hour,
	// my_time.tm_min, my_time.tm_sec, NANO_TO_US_ROUND(nav_pvt->nano));
#if (C_LOG_LEVEL <= LOG_INFO_NUM)
	struct tm tm;
	get_local_time(&tm);
	FUNC_ENTRY_ARGS(TAG, "GPS time set: %d-%02d-%02d %02d:%02d:%02d",
					(tm.tm_year) + 1900, (tm.tm_mon) + 1, tm.tm_mday,
					tm.tm_hour, tm.tm_min, tm.tm_sec);
#endif
	if (esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_GPS_TIME_SET, NULL, 0,
					   pdMS_TO_TICKS(10)) != ESP_OK) {
		WLOG(TAG, "EVT_FAIL: GPS_TIME_SET");
	}
	gps->time_set = 1;
	return ESP_OK;
}

uint8_t gps_read_msg_timeout(uint8_t magnitude) {
	if (!gps || !gps->ubx_device)
		return 0;
	if (!magnitude)
		magnitude = 1;
	return (gps->time_set &&
			(gps->ubx_device->ubx_msg.navPvt.iTOW - lctx.old_nav_pvt_itow) >
				(gps->time_out_gps_msg * (magnitude)))
			   ? 1
			   : 0;
}

uint8_t gps_has_version_set() {
	if (gps && gps->ubx_device) {
		if (gps->ubx_device->ready &&
			gps->ubx_device->ubx_msg.mon_ver.hwVersion[0])
			return 2;
		if (gps->ubx_device->ready && gps->ubx_device->hw_type)
			return 1;
	}
	return 0;
}

static void gps_buffers_init(void) {
	FUNC_ENTRYD(TAG);
#if !defined(CONFIG_GPS_LOG_STATIC_S_BUFFER)
	gps_check_sec_buf(BUFFER_SEC_SIZE);
#endif
	gps_speed_metrics_init();
	refresh_gps_speeds_by_distance();
	/// alpha buffers depending on output rate, so allocation after ubx setup
}

#if !defined(CONFIG_GPS_LOG_STATIC_A_BUFFER) ||                                \
	!defined(CONFIG_GPS_LOG_STATIC_S_BUFFER)
static void gps_on_sample_rate_change(void *handler_arg, esp_event_base_t base,
									  int32_t id, void *event_data) {
	uint8_t new_rate = g_rtc_config.ubx.output_rate;
	FUNC_ENTRY_ARGS(TAG, "new rate:%d", new_rate);
	// Resize alfa buffer if needed - check_and_alloc_buffer will reuse existing
	// buffer if it's already large enough (e.g., 20Hz->5Hz keeps 20Hz buffer)
	if (xSemaphoreTake(log_p_lctx.xMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
		gps_check_alfa_buf(ALPHA_BUFFER_SIZE(new_rate, spd_threshold_for_alfa(new_rate)));
		xSemaphoreGive(log_p_lctx.xMutex);
	}
}

static void gps_on_ubx_config_changed(void *handler_arg, esp_event_base_t base,
									  int32_t id, void *event_data) {
	FUNC_ENTRY(TAG);
	// Request UBX restart to apply new GNSS/rate configuration
	lctx.ubx_restart_requested = true;
	// Signal in-progress setup to abort so new config is applied immediately
	ubx_ctx_t *ubx = (ubx_ctx_t *)handler_arg;
	if (ubx) {
		ubx->reconfig_requested = true;
	}
}

static void gps_config_changed_cb(size_t group, size_t index) {
	if (group != config_gps_handle()) {
		return;
	}

	// Keep local log config in sync with the persisted GPS config
	gps_config_fix_values();

#if defined(CONFIG_LOGGER_VFS_ENABLED)
	bool affects_files = false;
	switch (index) {
	case cfg_gps_log_txt:
	case cfg_gps_log_format:
	case cfg_gps_file_date_time:
	case cfg_gps_ubx_file:
		affects_files = true;
		break;
	default:
		break;
	}

	// If logging is active, re-open files via VFS worker to apply new config
	if (affects_files && gps && gps->files_opened) {
		vfs_post_work(VFS_WORK_PARTITION_CHANGED,
					  gps); // closes and re-opens files with new config
	}
#endif
}

static void gps_on_ubx_nav_mode_changed(void *handler_arg,
										esp_event_base_t base, int32_t id,
										void *event_data) {
	FUNC_ENTRY_ARGS(TAG, "nav_mode:%d (confirmed by device)",
					g_rtc_config.ubx.nav_mode);
	// This handler fires AFTER u-blox confirms nav mode change (response event)
	// Do NOT call ubx_set_nav_mode() here - it already happened!
	// Only use this for logging/monitoring the confirmed change
}

static void gps_on_gps_log_nav_mode_changed(void *handler_arg,
											esp_event_base_t base, int32_t id,
											void *event_data) {
	FUNC_ENTRY_ARGS(TAG, "nav_mode:%d", g_rtc_config.ubx.nav_mode);
	// Actually apply the nav mode change to the u-blox device
	if (gps && gps->ubx_device) {
		ubx_set_nav_mode(gps->ubx_device, g_rtc_config.ubx.nav_mode);
	}
	// Handle GPS nav mode change file I/O operations in GPS task
	gps_log_nav_mode_change(gps, 1);
}

// Test function to simulate UBX config change for testing async reconfiguration
// void test_ubx_config_change(void) {
//     FUNC_ENTRY(TAG);
//     ILOG(TAG, "Testing async UBX config change - posting event");
//     esp_event_post(UBX_EVENT, UBX_EVENT_CONFIG_CHANGED, NULL, 0,
//     portMAX_DELAY);
// }

static void gps_free_sec_buffers(void) {
	FUNC_ENTRYD(TAG);
#if !defined(CONFIG_GPS_LOG_STATIC_A_BUFFER)
	gps_free_alfa_buf();
#endif
#if !defined(CONFIG_GPS_LOG_STATIC_S_BUFFER)
	gps_free_sec_buf();
#endif
}

static void gps_buffers_free(void) {
	FUNC_ENTRYD(TAG);
	gps_free_sec_buffers();
	gps_speed_metrics_free();
}

static void gps_on_ubx_deinit(void *handler_arg, esp_event_base_t base,
							  int32_t id, void *event_data) {
	FUNC_ENTRYD(TAG);
	gps_free_sec_buffers();
}

#endif // CONFIG_GPS_LOG_STATIC_A_BUFFER || CONFIG_GPS_LOG_STATIC_S_BUFFER

static bool ubx_init_rate_adjusted = false;

static void gpsTask(void *parameter) {
	FUNC_ENTRYD(TAG);
	uint32_t now = 0, mt = 0;
	ubx_ctx_t *ubx_ctx = gps->ubx_device;
	ubx_msg_byte_ctx_t ubx_packet = UBX_MSG_BYTE_CTX_DEFAULT(ubx_ctx->ubx_msg);
	ubx_packet.ctx = ubx_ctx;
	ubx_packet.msg_match_to_pos = false;
	ubx_packet.msg_ready_handler = ubx_msg_checksum_handler;
	ubx_msg_t *ubxMessage = &ubx_ctx->ubx_msg;
	uint8_t try_setup_times = 5;
#if (C_LOG_LEVEL <= LOG_DEBUG_NUM || defined(GPS_TASK_DEBUG))
	uint32_t loops = 0;
#endif
	ubx_init_rate_adjusted = false;
	while (lctx.gps_task_is_running) {
		now = get_millis();
		uint8_t has_decoded_count =
			0; // Track messages decoded this iteration for adaptive yielding
#if (C_LOG_LEVEL <= LOG_DEBUG_NUM || GPS_TASK_DEBUG > 1)
		// Debug: verify loop is running
		if ((loops > 0) && (loops % 500) == 0) {
			DLOG(TAG, "LOOP_ALIVE: l=%" PRIu32 " now=%" PRIu32 "", loops, now);
		}
#endif
		if (!gps_has_version_set() || lctx.ubx_restart_requested) {
			bool is_reconfig = lctx.ubx_restart_requested && ubx_ctx->ready;
			if (is_reconfig) {
				// Explicit reconfig: skip cooldown, restart immediately
				ILOG(TAG, "[%s] Reconfig requested, restarting UBX",
					 __FUNCTION__);
				ubx_off(ubx_ctx); // clears reconfig_requested
				vTaskDelay(pdMS_TO_TICKS(100));
				ubx_setup(ubx_ctx);
			} else {
				// Initial boot or retry: use 10s cooldown guard
				mt = now
					 - (ubx_ctx->ready ? ubx_ctx->ready_time : SEC_TO_MS(5));
				if (mt > SEC_TO_MS(10)) {
					if (ubx_ctx->ready) {
						ubx_off(ubx_ctx); // uart deinit
						vTaskDelay(pdMS_TO_TICKS(100));
						lctx.ubx_fail_count++;
					}
					ubx_setup(ubx_ctx); // uart init and reset
				}
			}
			if (lctx.ubx_fail_count > 50) {
				if (!gps_has_version_set()) {
					if (esp_event_post(GPS_LOG_EVENT,
									   GPS_LOG_EVENT_GPS_REQUEST_RESTART, NULL,
									   0, pdMS_TO_TICKS(100)) != ESP_OK) {
						WLOG(TAG, "EVT_FAIL: GPS_REQUEST_RESTART");
					}
					WLOG(TAG, "[%s] Gps init failed, restart requested.",
						 __FUNCTION__);
				} else
					lctx.ubx_fail_count = 0;
			}
			lctx.ubx_restart_requested = 0;
#if (C_LOG_LEVEL <= LOG_DEBUG_NUM || defined(GPS_TASK_DEBUG))
			if ((loops % 100) == 0) {
				WLOG(TAG, "gps not ready, next loop l=%" PRIu32 "", loops);
			}
#endif
			goto loop_tail;
		} else {
			// UBX device is ready: evaluate rate adjustment when partition is
			// available
			if (!ubx_init_rate_adjusted && gps_has_version_set()) {
				// First time UBX becomes ready: check rate constraints based on
				// partition space
				gps_adjust_rate_for_partition_space();
				ubx_init_rate_adjusted = true;
#if defined(CONFIG_GPS_TIMER_STATS_ENABLED)
				if (gps_periodic_timer)
					esp_timer_start_periodic(gps_periodic_timer, SEC_TO_US(10));
#endif
			}
		}

		// Two-phase processing: FAST decode (drain buffer) → SLOW processing
		// (file I/O) Phase 1: Drain buffer by decoding ALL available messages
		// (no heavy processing) Timeout scales with GPS rate to minimize CPU
		// waste while staying responsive: 1-2Hz=100ms, 3-5Hz=50ms, 6-10Hz=20ms,
		// 11-20Hz=10ms, 21-30Hz=5ms At low rates (2Hz=500ms period), 100ms
		// timeout = 5 wakeups/msg (vs 100 with 5ms) At high rates (30Hz=33ms
		// period), 5ms timeout = responsive without missing data Recalculated
		// each iteration to adapt to runtime rate changes
		uint32_t timeout_ms;
		if (!ubx_ctx->ready) {
			timeout_ms = 50; // During init, use moderate timeout
		} else if (g_rtc_config.ubx.output_rate >= 21) {
			timeout_ms = 5; // 21-30Hz: very responsive
		} else if (g_rtc_config.ubx.output_rate >= 11) {
			timeout_ms = 10; // 11-20Hz: responsive
		} else if (g_rtc_config.ubx.output_rate >= 6) {
			timeout_ms = 20; // 6-10Hz: balanced
		} else if (g_rtc_config.ubx.output_rate >= 3) {
			timeout_ms = 50; // 3-5Hz: reduce wakeups
		} else {
			timeout_ms =
				100; // 1-2Hz: minimize wakeups (still 5x per message at 2Hz)
		}
		BaseType_t signaled =
			xSemaphoreTake(ubx_ctx->msg_ready, pdMS_TO_TICKS(timeout_ms));

		if (!signaled && ubx_ctx->ready) {
#if (C_LOG_LEVEL <= LOG_DEBUG_NUM || defined(GPS_TASK_DEBUG))
			if ((loops % 100) == 0) {
				WLOG(TAG, "NOSIG: timeout, no msgs at l=%" PRIu32 "", loops);
			}
#endif
			goto loop_tail;
		}

		// Event-driven processing: Decode → Process immediately → Next event
		// CRITICAL: Copy NAV_PVT data immediately to protect from buffer
		// overwrites The shared buffer (ubx_ctx->ubx_msg) is reused for every
		// decoded message
		nav_pvt_t pvt_snapshot; // Local copy of current NAV_PVT (protects from
								// overwrites)
		bool had_nav_dop = false; // Track NAV_DOP for first fix detection
		int iterations = 0;

		// Process all pending messages (event-driven: decode → process →
		// complete)
		while (true) {
			// Safety: prevent infinite loop, allow other tasks to run
			if (++iterations > 50) {
#if (C_LOG_LEVEL <= LOG_DEBUG_NUM || defined(GPS_TASK_DEBUG))
				WLOG(TAG, "ITER_LIMIT: breaking at %d iterations", iterations);
#endif
				break;
			}

			// Yield every 5 iterations to prevent task starvation
			if ((iterations % 5) == 0) {
				vTaskDelay(pdMS_TO_TICKS(1)); // 1ms for scheduler
			}

			esp_err_t ret = ubx_msg_handler(ubx_ctx, &ubx_packet);

			if (!ret) {
				if (!has_decoded_count)
					has_decoded_count++;
				lctx.ubx_fail_count = 0;

				// Event-driven: Process each message type immediately
				switch (ubx_packet.ubx_msg_type) {
				case MT_NAV_PVT:
					// CRITICAL: Copy NAV_PVT data FIRST to protect from
					// subsequent overwrites ubx_msg_handler() writes directly
					// into shared buffer, so next decode will overwrite this!
					memcpy(&pvt_snapshot, &ubxMessage->navPvt,
						   sizeof(nav_pvt_t));

					// Update counters immediately
					if (pvt_snapshot.iTOW > 0 && gps->time_set == 1) {
						log_p_lctx.count_nav_pvt++;
					}

					// Process speed/movement immediately while data is fresh
					// Only proceed if we have valid GPS time (iTOW > 0)
					if (pvt_snapshot.iTOW == 0) {
						break; // Skip processing invalid data
					}

					// Check for first fix (needs DOP data from previous
					// message)
					if (had_nav_dop) {
						if ((pvt_snapshot.numSV >= MIN_numSV_FIRST_FIX) &&
							(MS_TO_SEC(pvt_snapshot.sAcc) <
							 MAX_Sacc_FIRST_FIX) &&
							(pvt_snapshot.valid >= 7) &&
							(!gps->signal_ok && !ubx_ctx->shutdown_requested)) {
							gps->signal_ok = true;
							gps->first_fix = (now - ubx_ctx->ready_time);
							WLOG(TAG, "[%s] First GPS Fix after %.01f sec.",
								 __FUNCTION__, MS_TO_SEC(gps->first_fix));
							if (esp_event_post(GPS_LOG_EVENT,
											   GPS_LOG_EVENT_GPS_FIRST_FIX,
											   NULL, 0, 0) != ESP_OK) {
								WLOG(TAG, "EVT_FAIL: GPS_FIRST_FIX");
							}
						}
					}

					if (gps->signal_ok && lctx.gps_log_delay < UINT8_MAX) {
						lctx.gps_log_delay++;
					}

					// Request file open when conditions met
					if (!gps->files_opened && gps->signal_ok &&
						(lctx.gps_log_delay > (TIME_DELAY_FIRST_FIX *
											   g_rtc_config.ubx.output_rate))) {
						int32_t avg_speed = pvt_snapshot.gSpeed;
						if (avg_speed > STANDSTILL_DETECTION_MAX &&
							gps->time_set) {
							gps->start_logging_millis = now;
							if (esp_event_post(GPS_LOG_EVENT,
											   GPS_LOG_EVENT_REQUEST_FILE_OPEN,
											   NULL, 0, 0) != ESP_OK) {
								WLOG(TAG, "EVT_FAIL: REQUEST_FILE_OPEN");
							}
						}
					}

					// Process speed/movement immediately using LOCAL snapshot
					// (safe from overwrites)
					if (gps->time_set && log_p_lctx.count_nav_pvt > 10) {
						gps->gps_speed = pvt_snapshot.gSpeed;

						// Process satellite data if available
						if (ubxMessage->count_nav_sat > 0) {
							push_gps_sat_info(&gps->Ublox_Sat,
											  &ubxMessage->nav_sat);
						}

						// Request file flush if needed
						if (gps->files_opened &&
							(now - lctx.last_flush_time) > 60000) {
							if (esp_event_post(GPS_LOG_EVENT,
											   GPS_LOG_EVENT_REQUEST_FILE_FLUSH,
											   NULL, 0, 0) == ESP_OK) {
								lctx.last_flush_time = now;
							}
						}

						// Validate speed data
						uint32_t sacc_mm = pvt_snapshot.sAcc;
						if ((pvt_snapshot.numSV <= MIN_numSV_GPS_SPEED_OK) ||
							(sacc_mm > MAX_Sacc_GPS_SPEED_OK * 1000) ||
							(gps->gps_speed > MAX_GPS_SPEED_OK * 1000)) {
#if (C_LOG_LEVEL <= LOG_INFO_NUM || defined(GPS_TASK_DEBUG))
							FUNC_ENTRY_ARGW(
								TAG,
								"GPS REJECTED: sats=%" PRIu8 " acc=%" PRIu32
								"mm speed=%" PRId32 "mm/s",
								pvt_snapshot.numSV, sacc_mm, gps->gps_speed);
#endif
							gps->gps_speed = 0;
							gps->Ublox.run_start_time = 0;
						}
#if defined(CONFIG_GPS_TIMER_STATS_ENABLED)
						else {
							// Count only PVTs that passed GPS validity checks
							// (not rejected)
							++cur_msg_stats.count_nav_pvt;
						}
#endif

						// Movement detection and logging (immediate while data
						// fresh)
						if (gps->gps_speed > STANDSTILL_DETECTION_MAX) {
							// // CRITICAL: Update nav_pvt pointer to local
							// snapshot before logging
							// // log_to_file() and other functions access
							// nav_pvt via pointer struct nav_pvt_s *orig_pvt =
							// nav_pvt; nav_pvt = &pvt_snapshot;

							// Log to file immediately using protected snapshot
							if (gps->files_opened) {
								log_to_file(gps);
								uint32_t time_diff_ms =
									pvt_snapshot.iTOW -
									lctx.old_nav_pvt_itow; // Correct order
								uint32_t threshold_ms =
									HZ_TO_MS(g_rtc_config.ubx
												 .output_rate); // Frame length
								if (gps->gps_speed >
									2000) { // only check timeouts when speed >
											// 2 m/s

									if (time_diff_ms >
										(threshold_ms *
										 10)) { // 10 x frame length
										gps->gps_timeout_flag++;
										gps->lost_frames++;
										if (gps->gps_timeout_flag == 1)
											log_gps_timeout(gps, time_diff_ms,
															"Timeout");
									} else if (time_diff_ms > threshold_ms) {
										gps->frame_lost_flag++;
										log_gps_timeout(gps, time_diff_ms,
														"Lost frame");
									}
								}
							}

							// nav_pvt = orig_pvt;  // Restore pointer

							if (!gps->gps_is_moving) {
								gps->gps_is_moving = true;
#if (C_LOG_LEVEL <= LOG_INFO_NUM || defined(GPS_TASK_DEBUG))
								FUNC_ENTRY_ARGW(
									TAG,
									"*** GPS IS MOVING *** speed=%" PRId32
									"mm/s",
									gps->gps_speed);
#endif
								if (esp_event_post(GPS_LOG_EVENT,
												   GPS_LOG_EVENT_GPS_IS_MOVING,
												   NULL, 0, 0) != ESP_OK) {
									WLOG(TAG, "EVT_FAIL: GPS_IS_MOVING");
								}
							}
						} else {
							if (gps->gps_is_moving) {
#if (C_LOG_LEVEL <= LOG_INFO_NUM || defined(GPS_TASK_DEBUG))
								FUNC_ENTRY_ARGW(
									TAG,
									"*** GPS IS STOPPING *** speed=%" PRId32
									"mm/s",
									gps->gps_speed);
#endif
								gps->gps_is_moving = false;
								if (esp_event_post(
										GPS_LOG_EVENT,
										GPS_LOG_EVENT_GPS_IS_STOPPING, NULL, 0,
										0) != ESP_OK) {
									WLOG(TAG, "EVT_FAIL: GPS_IS_STOPPING");
								}
								log_p_lctx.standstill_start_millis = 0;
							} else if (!gps->skip_alfa_after_stop) {
								if (log_p_lctx.standstill_start_millis == 0) {
									log_p_lctx.standstill_start_millis = now;
								} else if ((now -
											log_p_lctx
												.standstill_start_millis) >
										   SEC_TO_MS(5)) {
									gps->skip_alfa_after_stop = 1;
								}
							}
						}

						// Speed metrics and run detection using protected
						// snapshot
						esp_err_t ret = push_gps_data(
							gps, &gps->Ublox, FROM_10M(pvt_snapshot.lat),
							FROM_10M(pvt_snapshot.lon), gps->gps_speed);
						if (!ret) {
							gps->pvt_seq++;  // Increment sequence counter for NAV-PVT data updates
							new_run_detection(gps,
											  FROM_100K(pvt_snapshot.heading),
											  time_cur_speed(time_2s));
							alfa_indicator(FROM_100K(pvt_snapshot.heading));

							if (gps->run_count != lctx.old_run_count) {
								gps->Ublox.run_distance = 0;
								if (MM_TO_M(gps->gps_speed) >
									STANDSTILL_DETECTION_MAX) {
									gps->Ublox.run_start_time = now;
									gps->record = 0;
									if (esp_event_post(
											GPS_LOG_EVENT,
											GPS_LOG_EVENT_GPS_NEW_RUN, NULL, 0,
											0) != ESP_OK) {
										WLOG(TAG, "EVT_FAIL: GPS_NEW_RUN");
									}
								}
								lctx.old_run_count = gps->run_count;
#if (C_LOG_LEVEL <= LOG_INFO_NUM || defined(GPS_TASK_DEBUG))
								FUNC_ENTRY_ARGW(TAG,
												"*** New run *** speed=%" PRId32
												"mm/s",
												gps->gps_speed);
#endif
							}
							gps_speed_metrics_update();
							gps->stats_seq++;  // Increment sequence counter for speed metrics updates
#if defined(CONFIG_GPS_TIMER_STATS_ENABLED)
							++cur_msg_stats.count_ok;
#endif
						}
#if defined(CONFIG_GPS_TIMER_STATS_ENABLED)
						else
							++cur_msg_stats.count_err;
#endif
					}
					lctx.old_nav_pvt_itow = pvt_snapshot.iTOW;
					break;

				case MT_NAV_SAT:
					ubxMessage->nav_sat.iTOW =
						ubxMessage->nav_sat.iTOW - SEC_TO_MS(LEAP_UTC_OFFSET);
					// Satellite processing happens in NAV_PVT case above
					break;

				case MT_NAV_DOP:
					had_nav_dop = true;
#if defined(CONFIG_GPS_TIMER_STATS_ENABLED)
					cur_msg_stats.count_nav_dop++;
#endif
					break;

				default:
					break;
				}

				// Check for more messages (drain semaphore queue)
				if (xSemaphoreTake(ubx_ctx->msg_ready, 0) != pdTRUE) {
					// No more pending messages
					if (!ubx_rx_has_complete_frame(ubx_ctx)) {
						break; // Buffer empty, done processing
					}
				}
			} else {
				// Decode error
				if ((gps_has_version_set() &&
					 (ret == ESP_ERR_TIMEOUT && lctx.ubx_fail_count > 3)) ||
					lctx.ubx_fail_count > 20) {
					if (ubx_ctx->ready) {
						ubx_ctx->ready = false;
						ubxMessage->mon_ver.hwVersion[0] = 0;
					}
					lctx.ubx_fail_count++;
					goto loop_tail;
				}
				lctx.ubx_fail_count += 5;
				break;
			}
		}

		// After processing all messages: handle time sync if needed (check
		// timing once here)
		if (now >= lctx.next_time_sync || lctx.next_time_sync == 0) {
			if (set_time(&pvt_snapshot, g_rtc_config.gps.timezone) == ESP_OK) {
				lctx.next_time_sync =
					now + SEC_TO_MS(30); // Sync again in 30 seconds
			} else {
				lctx.next_time_sync =
					now + SEC_TO_MS(5); // Retry in 5 seconds on failure
			}
		}

		// Yield after processing messages
		if (has_decoded_count) {
			vTaskDelay(pdMS_TO_TICKS(1)); // Let other tasks run
		} else {
			vTaskDelay(0); // Cooperative yield on timeout
		}
	loop_tail:
		// Loop-tail yield: cooperative only; main delay is applied after Phase
		// 1 when messages are decoded
		vTaskDelay(0);

#if (C_LOG_LEVEL <= LOG_DEBUG_NUM || defined(GPS_TASK_DEBUG))
		loops++;
		if ((loops % 100) == 0) {
			ILOG(TAG,
				 "BEAT: l=%" PRIu32 " r=%d t=%d s=%d p=%" PRIu32 " sv=%" PRIu8
				 "",
				 loops, ubx_ctx->ready, gps->time_set, gps->signal_ok,
				 log_p_lctx.count_nav_pvt, pvt_snapshot.numSV);
		}
#endif
		continue;
	}
	lctx.gps_task_handle = 0;
	vTaskDelete(NULL);
}

static void gps_task_start() {
	FUNC_ENTRYD(TAG);
	if (lctx.gps_task_is_running) {
		ELOG(TAG, "[%s] already running!", __func__);
		return;
	}
	if (!lctx.gps_task_is_running) {
		lctx.gps_task_is_running = true;
		xTaskCreatePinnedToCore(
			gpsTask,				   /* Task function. */
			"gpsTask",				   /* String with name of task. */
			CONFIG_GPS_LOG_STACK_SIZE, /* Stack size in bytes. */
			NULL, /* Parameter passed as input of the task */
			10,	  /* Priority of the task. Reduced from 19 to prevent task
					 starvation */
			&lctx.gps_task_handle,
			0); /* Task handle. Core 0 for WiFi affinity */
	}
}

static void gps_task_stop() {
	FUNC_ENTRYD(TAG);
	if (lctx.gps_task_is_running) {
#if defined(CONFIG_GPS_TIMER_STATS_ENABLED)
		if (gps_periodic_timer)
			esp_timer_stop(gps_periodic_timer);
#endif
		lctx.gps_task_is_running = false;
		uint32_t deadline = get_millis() + SEC_TO_MS(2);
		while (lctx.gps_task_handle && (get_millis() < deadline))
			delay_ms(10);
		if (lctx.gps_task_handle) {
			vTaskDelete(lctx.gps_task_handle);
			lctx.gps_task_handle = 0;
		}
	}
	DLOG(TAG, "[%s] done.", __func__);
}

void gps_init(gps_context_t *_gps) {
	if (lctx.gps_initialized)
		return;
	FUNC_ENTRY(TAG);
	gps = _gps;
	gps_config_fix_values();
	init_gps_context_fields(gps);
#if defined(CONFIG_GPS_TIMER_STATS_ENABLED)
	const esp_timer_create_args_t gps_periodic_timer_args = {
		.callback = &gps_log_print_all_stats,
		.name = "gps_periodic",
		.arg = _gps->ubx_device,
	};
	if (esp_timer_create(&gps_periodic_timer_args, &gps_periodic_timer)) {
		ELOG(TAG, "[%s] Failed to create periodic timer", __func__);
	}
#endif
	if (!gps_config_observer_registered) {
		gps_config_observer_registered =
			config_observer_add(gps_config_changed_cb);
		if (!gps_config_observer_registered) {
			WLOG(TAG, "[%s] failed to register GPS config observer", __func__);
		}
	}

	// Register VFS event handler to track partition availability for logging
	gps_log_register_vfs_handler();

	lctx.gps_initialized = 1;
}

void gps_deinit(void) {
	if (!lctx.gps_initialized)
		return;
	FUNC_ENTRY(TAG);
#if defined(CONFIG_GPS_TIMER_STATS_ENABLED)
	esp_timer_delete(gps_periodic_timer);
#endif
	if (gps_config_observer_registered) {
		config_observer_remove(gps_config_changed_cb);
		gps_config_observer_registered = false;
	}

	// Unregister VFS event handler when GPS is shut down
	gps_log_unregister_vfs_handler();

	deinit_gps_context_fields(gps);
	lctx.gps_initialized = 0;
}

int gps_start() {
	FUNC_ENTRYD(TAG);
	if (!gps)
		return 0;
	struct ubx_ctx_s *ubx_ctx = gps->ubx_device;
	if (!ubx_ctx)
		return 0;
	if (lctx.gps_started)
		return 0;
	int ret = 0;
#if !defined(CONFIG_GPS_LOG_STATIC_A_BUFFER)
	if (!lctx.gps_events_registered) {
		esp_event_handler_register(UBX_EVENT, UBX_EVENT_SAMPLE_RATE_CHANGED,
								   &gps_on_sample_rate_change, gps->ubx_device);
		esp_event_handler_register(UBX_EVENT, UBX_EVENT_CONFIG_CHANGED,
								   &gps_on_ubx_config_changed, gps->ubx_device);
		esp_event_handler_register(UBX_EVENT, UBX_EVENT_NAV_MODE_CHANGED,
								   &gps_on_ubx_nav_mode_changed,
								   gps->ubx_device);
		esp_event_handler_register(GPS_LOG_EVENT,
								   GPS_LOG_EVENT_GPS_NAV_MODE_CHANGED,
								   &gps_on_gps_log_nav_mode_changed, NULL);
		// Register async file handlers (enqueue work for VFS worker)
		esp_event_handler_register(GPS_LOG_EVENT,
								   GPS_LOG_EVENT_REQUEST_FILE_OPEN,
								   &gps_file_open_handler, gps);
		esp_event_handler_register(GPS_LOG_EVENT,
								   GPS_LOG_EVENT_REQUEST_FILE_FLUSH,
								   &gps_file_flush_handler, gps);

		// Register VFS GPS interface
#if defined(CONFIG_LOGGER_VFS_ENABLED)
		{
			vfs_work_if_t iface = {.process = gps_vfs_processor, .ctx = gps};
			vfs_register_work_interface(&iface);
		}
#endif
		lctx.gps_events_registered = 1;
	}
#endif
	gps_buffers_init();
	// Async writer now started in open_files() in gps_log_file.c
	gps_task_start();
	lctx.gps_started = 1;
	return ret;
}

int gps_shut_down() {
	FUNC_ENTRYD(TAG);
	if (!gps)
		return 0;
	struct ubx_ctx_s *ubx_ctx = gps->ubx_device;
	if (!ubx_ctx)
		return 0;
	if (!lctx.gps_started)
		return 0;
	int ret = 0;
	ubx_ctx->shutdown_requested = 1;
	if (ubx_ctx->ready) {
		gps->signal_ok = false;
		lctx.gps_log_delay = 0;
	}
	// Stop GPS task FIRST before any async writer teardown.
	// The GPS task writes to async_writer_queue and async_pool at up to 20Hz.
	// close_files() deletes both those queues inside async_writer_stop(), so if
	// the GPS task is mid-xQueueSend or mid-logger_fixed_pool_alloc when those
	// handles are freed, FreeRTOS state is corrupted and the device hangs
	// forever (no logs, no sleep reached, reproducible on both cores / rates).
	gps_task_stop();
	if (gps->time_set) { // Only safe to RTC memory if new GPS data is available
						 // !!
		if (esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_GPS_SAVE_FILES, NULL, 0,
						   pdMS_TO_TICKS(100)) != ESP_OK) {
			WLOG(TAG, "EVT_FAIL: GPS_SAVE_FILES");
		}
		// next_screen = CUR_SCREEN_SAVE_SESSION;
		// gps_save_rtc();
		if (gps->files_opened) {
			gps_speed_metrics_save_session();
			close_files(gps);
			// Async writer stopped inside close_files() in gps_log_file.c
		}
	}
#if !defined(CONFIG_GPS_LOG_STATIC_A_BUFFER) ||                                \
	!defined(CONFIG_GPS_LOG_STATIC_S_BUFFER)
	if (lctx.gps_events_registered) {
		esp_event_handler_unregister(UBX_EVENT, UBX_EVENT_SAMPLE_RATE_CHANGED,
									 &gps_on_sample_rate_change);
		esp_event_handler_unregister(UBX_EVENT, UBX_EVENT_CONFIG_CHANGED,
									 &gps_on_ubx_config_changed);
		esp_event_handler_unregister(UBX_EVENT, UBX_EVENT_NAV_MODE_CHANGED,
									 &gps_on_ubx_nav_mode_changed);
		esp_event_handler_unregister(GPS_LOG_EVENT,
									 GPS_LOG_EVENT_GPS_NAV_MODE_CHANGED,
									 &gps_on_gps_log_nav_mode_changed);
		// Unregister async file handlers
		esp_event_handler_unregister(GPS_LOG_EVENT,
									 GPS_LOG_EVENT_REQUEST_FILE_OPEN,
									 &gps_file_open_handler);
		esp_event_handler_unregister(GPS_LOG_EVENT,
									 GPS_LOG_EVENT_REQUEST_FILE_FLUSH,
									 &gps_file_flush_handler);

#if defined(CONFIG_LOGGER_VFS_ENABLED)
		// Unregister VFS work interface
		vfs_register_work_interface(NULL);
#endif
		lctx.gps_events_registered = 0;
	}
#endif
	ubx_off(ubx_ctx);
	gps_buffers_free();

	gps->time_set = 0;
	if (lctx.output_rate_swp) {
		g_rtc_config.ubx.output_rate = lctx.output_rate_swp;
		lctx.output_rate_swp = 0;
	}
	lctx.gps_started = 0;
	if (esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_GPS_SHUT_DOWN_DONE, NULL, 0,
					   pdMS_TO_TICKS(100)) != ESP_OK) {
		WLOG(TAG, "EVT_FAIL: GPS_SHUT_DOWN_DONE");
	}
	// if (!no_sleep) {
	//     go_to_sleep(3);  // got to sleep after 5 s, this to prevent booting
	//     when
	//     // GPIO39 is still low !
	// }
	// if(next_screen == CUR_SCREEN_SAVE_SESSION) {
	//     next_screen = CUR_SCREEN_NONE;
	// }
	return ret;
}

// Async file open handler - enqueue work for the VFS worker (avoids blocking
// event loop)
static void gps_file_open_handler(void *handler_arg, esp_event_base_t base,
								  int32_t id, void *event_data) {
	FUNC_ENTRY_ARGS(TAG, "async file open request");
	gps_context_t *gps = (gps_context_t *)handler_arg;
	if (gps && !gps->files_opened) {
		ILOG(TAG, "[%s] Requesting VFS worker to open files", __FUNCTION__);
		vfs_post_work(VFS_WORK_OPEN_FILES, gps);
	}
}

// Async file flush handler - enqueue flush work for the VFS worker
static void gps_file_flush_handler(void *handler_arg, esp_event_base_t base,
								   int32_t id, void *event_data) {
	FUNC_ENTRY_ARGSD(TAG, "async file flush request");
	gps_context_t *gps = (gps_context_t *)handler_arg;
	if (gps && gps->files_opened) {
		DLOG(TAG, "[%s] Requesting VFS worker to flush files", __FUNCTION__);
		vfs_post_work(VFS_WORK_FLUSH_FILES, gps);
	}
}

#endif
