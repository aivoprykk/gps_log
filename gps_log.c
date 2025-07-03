#include "gps_log.h"
#include "log_private.h"
#include "gps_data.h"
#include "gps_log_file.h"
#include "gps_log_events.h"
#include "ubx.h"
#include "ubx_msg.h"
#include "logger_common.h"
#include "dstat_screens.h"
#include "gps_user_cfg.h"

#include "esp_mac.h"

static const char *TAG = "gps_log";

#if (C_LOG_LEVEL < 1) && defined(SHOW_SAVED_FRAMES)
#define GPS_TIMER_STATS 1
#endif

#if defined(GPS_TIMER_STATS)
static uint32_t push_failed_count = 0, push_ok_count = 0, prev_push_failed_count = 0, prev_push_ok_count = 0;
static uint32_t local_nav_dop_count = 0, prev_local_nav_dop_count = 0;
static uint32_t prev_local_nav_pvt_count = 0;
static uint32_t prev_local_nav_sat_count = 0;
static uint32_t prev_millis = 0, prev_msg_count = 0, prev_err_count = 0;
static esp_timer_handle_t gps_periodic_timer = 0;

static void s(void* arg) {
    ubx_config_t *ubx_dev = (ubx_config_t *)arg;
    if(!ubx_dev) return;
    uint32_t millis = get_millis();
    uint32_t period = millis - prev_millis;
    uint16_t period_err_count = ubx_dev->ubx_msg.count_err-prev_err_count;
    uint16_t period_msg_count = ubx_dev->ubx_msg.count_msg - prev_msg_count;
    uint16_t period_saved_count = period_msg_count - period_err_count;
    uint16_t period_push_failed_count = push_failed_count - prev_push_failed_count;
    uint16_t period_push_ok_count = push_ok_count - prev_push_ok_count;
    uint16_t period_push_count = period_push_failed_count + period_push_ok_count;
    uint16_t period_local_nav_dop_count = local_nav_dop_count - prev_local_nav_dop_count;
    uint16_t period_local_nav_pvt_count = ubx_dev->ubx_msg.count_nav_pvt - prev_local_nav_pvt_count;
    uint16_t period_local_nav_sat_count = ubx_dev->ubx_msg.count_nav_sat - prev_local_nav_sat_count;
    prev_millis = millis;
    prev_msg_count = ubx_dev->ubx_msg.count_msg;
    prev_err_count = ubx_dev->ubx_msg.count_err;
    prev_push_failed_count = push_failed_count;
    prev_push_ok_count = push_ok_count;
    prev_local_nav_dop_count = local_nav_dop_count;
    prev_local_nav_pvt_count = ubx_dev->ubx_msg.count_nav_pvt;
    prev_local_nav_sat_count = ubx_dev->ubx_msg.count_nav_sat;
    printf("\n[%s] * p:%"PRIu32"ms, r:%"PRIu8"Hz\n", __FUNCTION__, period, ubx_dev->rtc_conf->output_rate);
    printf("[%s] >>>> period  msgcount: %6"PRIu16"  ok: %6"PRIu16" fail: %6"PRIu16" >>>>\n", __FUNCTION__, period_msg_count, period_saved_count, period_err_count);
    printf("[%s] >>>> total   msgcount: %6"PRIu32", ok: %6"PRIu32" fail: %6"PRIu32" >>>>\n", __FUNCTION__, ubx_dev->ubx_msg.count_msg, ubx_dev->ubx_msg.count_ok, ubx_dev->ubx_msg.count_err);
    printf("[%s] >>>> period pushcount: %6"PRIu16", ok: %6"PRIu16" fail: %6"PRIu16" >>>>\n", __FUNCTION__, period_push_count, period_push_ok_count, period_push_failed_count);
    printf("[%s] >>>> total  pushcount: %6"PRIu32", ok: %6"PRIu32" fail: %6"PRIu32" >>>>\n\n", __FUNCTION__, push_ok_count+push_failed_count, push_ok_count, push_failed_count);
    printf("[%s] >>>> period nav_dop_count: %6"PRIu16" >>>>\n", __FUNCTION__, period_local_nav_dop_count);
    printf("[%s] >>>> total  nav_dop_count: %6"PRIu32" >>>>\n\n", __FUNCTION__, local_nav_dop_count);
    printf("[%s] >>>> period nav_pvt_count: %6"PRIu16" >>>>\n", __FUNCTION__, period_local_nav_pvt_count);
    printf("[%s] >>>> total  nav_pvt_count: %6"PRIu32" >>>>\n\n", __FUNCTION__, ubx_dev->ubx_msg.count_nav_pvt);
    printf("[%s] >>>> period nav_sat_count: %6"PRIu16" >>>>\n", __FUNCTION__, period_local_nav_sat_count);
    printf("[%s] >>>> total  nav_sat_count: %6"PRIu32" >>>>\n\n", __FUNCTION__, ubx_dev->ubx_msg.count_nav_sat);
}
#endif


typedef struct {
    uint16_t old_run_count;
    uint8_t GPS_delay;
    uint32_t old_nav_pvt_itow;
    uint32_t next_time_sync;
    bool ubx_restart_requested;
    int output_rate_swp;
    uint32_t last_flush_time;
    bool gps_task_is_running;
    TaskHandle_t gps_task_handle;
    uint8_t ubx_fail_count;
} log_context_t;

static log_context_t lctx = {.old_run_count = 0, .GPS_delay = 0, .old_nav_pvt_itow = 0, .next_time_sync = 0, .ubx_restart_requested = false, 
                               .output_rate_swp = 0, .last_flush_time = 0, .gps_task_is_running = false, .gps_task_handle = NULL, .ubx_fail_count = 0};

gps_context_t * gps = 0;

ESP_EVENT_DEFINE_BASE(GPS_LOG_EVENT);        // declaration of the LOG_EVENT family

#if (C_LOG_LEVEL < 2)
const char * const _gps_log_event_strings[] = {
    GPS_LOG_EVENT_LIST(STRINGIFY)
};
const char * gps_log_event_strings(int id) {
    return _gps_log_event_strings[id];
}
#else
const char * gps_log_event_strings(int id) {
    return "GPS_LOG_EVENT";
}
#endif

static int8_t set_time(float time_offset) {
#if (C_LOG_LEVEL < 1)
    DLOG(TAG, "[%s]\n", __FUNCTION__);
#endif
    struct ubx_config_s *ubx_dev = gps->ubx_device;
    nav_pvt_t *pvt = &ubx_dev->ubx_msg.navPvt;
    if (!pvt->numSV) return 0;
    if (pvt->year < 2023) { // no valid time
        WLOG(TAG, "[%s] wrong time: %hu-%hhu-%hhu", __func__, pvt->year, pvt->month, pvt->day);
    }
    uint32_t millis = get_millis();
    if (gps->time_set && millis < lctx.next_time_sync) { // time already set and not time to sync
        return 0;
    }
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s] time_offset: %f", __FUNCTION__, time_offset);
#endif
    //ESP_LOGI(TAG, "[%s] sats: %" PRIu8, __FUNCTION__, pvt->numSV);
    // time_t unix_timestamp = 0;  // a timestamp in seconds
#if defined(DLS)
    // summertime is on march 26 2023 2 AM, see
    // https://www.di-mgt.com.au/wclock/help/wclo_tzexplain.html
    setenv("TZ", "CET0CEST,M3.5.0/2,M10.5.0/3", 1);  // timezone UTC = CET, Daylightsaving ON :
                                                     // TZ=CET-1CEST,M3.5.0/2,M10.5.0/3
#else
    setenv("TZ", "UTC", 0);
#endif
    tzset();                            // this works for CET, but TZ string is different for every Land / continent....
    struct tm my_time = {
        .tm_sec = pvt->second,
        .tm_min = pvt->minute,
        .tm_hour = pvt->hour,
        .tm_mday = pvt->day,
        .tm_mon = pvt->month - 1,       // mktime needs months 0 - 11
        .tm_year = pvt->year - 1900,    // mktime needs years since 1900, so deduct 1900
        .tm_isdst = -1,                 // daylight saving time flag
    };
    int sync_period = 30;
    int ret = c_set_time(&my_time, NANO_TO_US_ROUND(pvt->nano), time_offset);  // set the time in the struct tm
    if (ret) {
        ELOG(TAG, "[%s] Failed to set time from gps", __FUNCTION__);
        sync_period = 5;
        goto done;
    }
#if (C_LOG_LEVEL < 3)
    // unix_timestamp = mktime(&my_time);  // mktime returns local time, so TZ is important !!!
    // int64_t utc_ms = unix_timestamp * 1000LL + NANO_TO_MILLIS_ROUND(pvt->nano);
    // int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
    WLOG(TAG, "GPS pvt time: %d-%02d-%02d %02d:%02d:%02d %ld %hhu", pvt->year, pvt->month, pvt->day, pvt->hour, pvt->minute, pvt->second, pvt->nano, pvt->id);
    // WLOG(TAG, "GPS tm time : %d-%02d-%02d %02d:%02d:%02d %ld", my_time.tm_year+1900, my_time.tm_mon+1, my_time.tm_mday, my_time.tm_hour, my_time.tm_min, my_time.tm_sec, NANO_TO_US_ROUND(pvt->nano));
#endif
#if (C_LOG_LEVEL < 4)
    struct tm tm;
    get_local_time(&tm);
    WLOG(TAG, "GPS time set: %d-%02d-%02d %02d:%02d:%02d", (tm.tm_year) + 1900, (tm.tm_mon) + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
#endif
    esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_GPS_TIME_SET, NULL,0, portMAX_DELAY);
    done:
    lctx.next_time_sync = millis + SEC_TO_MS(sync_period); // 1 minute
    gps->time_set = 1;
    return 1;
}

uint8_t gps_read_msg_timeout(uint8_t magnitude) {
    if(!gps || !gps->ubx_device) return 0;
    if(!magnitude) magnitude = 1;
    return (gps->time_set && (gps->ubx_device->ubx_msg.navPvt.iTOW - lctx.old_nav_pvt_itow) > (gps->time_out_gps_msg * (magnitude))) ? 1 : 0;
}

uint8_t gps_has_version_set() {
    if(gps && gps->ubx_device) {
        if(gps->ubx_device->ready && gps->ubx_device->ubx_msg.mon_ver.hwVersion[0]) return 2;
        if(gps->ubx_device->ready && gps->ubx_device->rtc_conf->hw_type) return 1;
    }
    return 0;
}

static esp_err_t ubx_msg_do(const ubx_msg_byte_ctx_t *ubx_packet) {
    struct ubx_config_s *ubx_dev = gps->ubx_device;
    struct gps_data_s *gps_data = &gps->Ublox;
    struct ubx_msg_s * ubxMessage = &ubx_dev->ubx_msg;
    const struct nav_pvt_s * nav_pvt = &ubxMessage->navPvt;
    const uint32_t now = get_millis();
    esp_err_t ret = ESP_OK;
    switch(ubx_packet->ubx_msg_type) {
            case MT_NAV_DOP:
                if (nav_pvt->iTOW > 0) {  // Logging after NAV_DOP, ublox sends first NAV_PVT, and then NAV_DOP.
                #if defined(GPS_TIMER_STATS)
                    local_nav_dop_count++;
                #endif
                    if ((nav_pvt->numSV >= MIN_numSV_FIRST_FIX) && (MS_TO_SEC(nav_pvt->sAcc) < MAX_Sacc_FIRST_FIX) && (nav_pvt->valid >= 7) && (gps->signal_ok == false)) {
                        gps->signal_ok = true;
                        gps->first_fix = (now - ubx_dev->ready_time);
                        WLOG(TAG, "[%s] First GPS Fix after %.01f sec.", __FUNCTION__, MS_TO_SEC(gps->first_fix));
                        esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_GPS_FIRST_FIX, NULL, 0, portMAX_DELAY);
                        // if(!m_context_rtc.RTC_screen_auto_refresh) {
                        //     lcd_ui_task_resume_for_times(1, -1, -1, true);
                        // }
                    }
                    if (gps->signal_ok && lctx.GPS_delay < UINT8_MAX) {
                        lctx.GPS_delay++; // delay max is 255 ubx messages for now
                    }

                    set_time(c_gps_cfg.timezone);
                    // printf(" msg:[%lu, %hu-%hhu-%hhu %hhu:%hhu:%hhu]", nav_pvt->iTOW, nav_pvt->year, nav_pvt->month, nav_pvt->day, nav_pvt->hour, nav_pvt->minute, nav_pvt->second);

                    if (!gps->files_opened && gps->signal_ok && (lctx.GPS_delay > (TIME_DELAY_FIRST_FIX * ubx_dev->rtc_conf->output_rate))) {  // vertraging Gps_time_set is nu 10 s!!
                        int32_t avg_speed = 0;
                        avg_speed = (avg_speed + nav_pvt->gSpeed * 19) / 20;  // FIR filter average speed of last 20 measurements in mm/s
                        if (avg_speed > STANDSTILL_DETECTION_MAX) { // 1 m/s == 3.6 km/h
                            if (gps->time_set) {
                                gps->start_logging_millis = now;
                                open_files(gps);  // only start logging if GPS signal is OK
                                esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_LOG_FILES_OPENED, NULL, 0, portMAX_DELAY);
                            } else {
                                ELOG(TAG, "[%s] gps time not set!", __FUNCTION__);
                            }
                        }  // Only speed > 0 if speed is greater than 1m/s + sACC < 1 + sat < 5 + speed > 35 m/s !!!
                    }
                    //printf("need to be for speed: %x %x %"PRIu32" %"PRIu32"\n", m_context.sdOK, gps->time_set, ubxMessage->count_nav_pvt, ubxMessage->count_nav_pvt_prev);
                    if ((gps->time_set) && (ubxMessage->count_nav_pvt > 10) && (ubxMessage->count_nav_pvt != ubxMessage->count_nav_pvt_prev)) {
                        ubxMessage->count_nav_pvt_prev = ubxMessage->count_nav_pvt;
                        // float sAcc=nav_pvt->sAcc/1000;
                        gps->gps_speed = nav_pvt->gSpeed;  // hier alles naar mm/s !!
                        //printf("[%s] GPS nav_pvt gSpeed: %"PRId32"\n", __FUNCTION__, gps->gps_speed);
                        
                        // gps msg interval used by sd card logging and run detection and show trouble screen when no gps signal
                        // gps->interval_gps_msg = nav_pvt->iTOW - lctx.old_nav_pvt_itow;
                        lctx.old_nav_pvt_itow = nav_pvt->iTOW;
        
                        if (gps->files_opened && (now - lctx.last_flush_time) > 60000) {
                            flush_files(gps);
                            lctx.last_flush_time = now;
                        }
                        // init run if speed is 0 or sAcc > 1 or speed > 35 m/s
                        if ((nav_pvt->numSV <= MIN_numSV_GPS_SPEED_OK) || (MM_TO_M(nav_pvt->sAcc) > MAX_Sacc_GPS_SPEED_OK) || (MM_TO_M(nav_pvt->gSpeed) > MAX_GPS_SPEED_OK)) {
#if (C_LOG_LEVEL < 2)
                            const char * c = " GPS run cancelled ";
                            if(nav_pvt->numSV <= MIN_numSV_GPS_SPEED_OK) WLOG(TAG, "[%s]%s %hhu <= %d MIN_numSV_GPS_SPEED_OK", __FUNCTION__, c, nav_pvt->numSV, (int)MIN_numSV_GPS_SPEED_OK);
                            if(MM_TO_M(nav_pvt->sAcc) > MAX_Sacc_GPS_SPEED_OK) WLOG(TAG, "[%s]%s%.01f > %d MAX_Sacc_GPS_SPEED_OK", __FUNCTION__, c, MM_TO_M(nav_pvt->sAcc), (int)MAX_Sacc_GPS_SPEED_OK);
                            if(MM_TO_M(nav_pvt->gSpeed) > MAX_GPS_SPEED_OK) WLOG(TAG, "[%s]%s%.01f > %d MAX_GPS_SPEED_OK", __FUNCTION__, c, MM_TO_M(nav_pvt->gSpeed), (int)MAX_GPS_SPEED_OK);
#endif
                            gps->gps_speed = 0;
                            gps_data->run_start_time = 0;
                        }
                        //saved_count++;
                        if (gps->gps_speed > STANDSTILL_DETECTION_MAX) { // log only when speed is above 1 m/s == 3.6 km/h
                            log_to_file(gps);  // here it is also printed to serial !!
                            if(!gps->gps_is_moving) {
#if (C_LOG_LEVEL < 2)
                                ILOG(TAG, "[%s] GPS moving detected (%ld ms), run start event sent.\n", __FUNCTION__, gps->gps_speed);
#endif
                                gps->gps_is_moving = true;
                                esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_GPS_IS_MOVING, NULL, 0, portMAX_DELAY);
                            }
                            // if(!m_context_rtc.RTC_screen_auto_refresh && lcd_ui_task_is_paused()) {
                            //     lcd_ui_task_resume();
                            // }
                        }
                        else {
                            if(gps->gps_is_moving) {
#if (C_LOG_LEVEL < 2)
                                ILOG(TAG,"[%s] GPS stop run detected (%ld ms), stop event sent.\n", __FUNCTION__, gps->gps_speed);
#endif
                                gps->gps_is_moving = false;
                                esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_GPS_IS_STOPPING, NULL, 0, portMAX_DELAY);
                                log_p_lctx.standstill_start_millis = 0;
                            }
                            else if (!gps->skip_alfa_after_stop) {
                                if (log_p_lctx.standstill_start_millis == 0) {
                                    log_p_lctx.standstill_start_millis = now;
                                }
                                else if ((now - log_p_lctx.standstill_start_millis) > SEC_TO_MS(5)) { // 5 seconds of standstill
                                    if (!gps->skip_alfa_after_stop) {
                                        printf("[%s] GPS run stopped, alfa detection skipped.\n", __FUNCTION__);
                                        // esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_GPS_STANDSTILL, NULL, 0, portMAX_DELAY);
                                        gps->skip_alfa_after_stop = 1; // reset skip alfa detection
                                    }
                                }
                            }
                            // if(!m_context_rtc.RTC_screen_auto_refresh) {
                            // if(!lcd_ui_task_is_paused()) {   
                            //     lcd_ui_task_pause();
                            //     goto showscr;
                            // }
                            // if(record_done < 240) {
                            //     showscr:
                            //     lcd_ui_task_resume_for_times(1, -1, -1, true);
                            // }
                            // }
                        }
                        ret = push_gps_data(gps, gps_data, FROM_10M(nav_pvt->lat), FROM_10M(nav_pvt->lon), gps->gps_speed);
                        if(ret){
                        #if defined(GPS_TIMER_STATS)
                            ++push_failed_count;
                        #endif
#if (C_LOG_LEVEL < 2)
                            ESP_LOGE(TAG, "[%s] push msg failed, timer: %lu, msg_count: %lu ...\n", __func__, now, ubxMessage->count_nav_pvt);
#endif
                            goto end;
                        }
                        #if defined(GPS_TIMER_STATS)
                        else
                            ++push_ok_count;
                        #endif
                        new_run_detection(gps, FROM_100K(nav_pvt->heading), time_cur_speed(time_2s));
                        alfa_indicator(FROM_100K(nav_pvt->heading));
                        // new run detected, reset run distance
                        if (gps->run_count != lctx.old_run_count) {
                            gps_data->run_distance = 0;
                            // gps_data->run_distance_after_turn = 0;
                            if (MM_TO_M(gps->gps_speed) > STANDSTILL_DETECTION_MAX) {
#if (C_LOG_LEVEL < 2)
                            ILOG(TAG, "[%s] GPS new run detected (%ld ms).\n", __FUNCTION__, gps->gps_speed);
#endif
                                gps_data->run_start_time = now;
                                gps->record = 0;
                                esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_GPS_NEW_RUN, NULL, 0, portMAX_DELAY);
                            }
                        }
                        lctx.old_run_count = gps->run_count;
                        gps_speed_metrics_update();
                    }
                }
                break;
            case MT_NAV_PVT:
                if (nav_pvt->iTOW > 0) {
                    if (gps->time_set == 1) {
                        ubxMessage->count_nav_pvt++;
                    }
                }
                break;
            case MT_NAV_SAT:
                ubxMessage->count_nav_sat++;
                ubxMessage->nav_sat.iTOW = ubxMessage->nav_sat.iTOW - SEC_TO_MS(LEAP_UTC_OFFSET); //to match 18s diff UTC nav pvt & GPS nav sat !!!
                push_gps_sat_info(&gps->Ublox_Sat, &ubxMessage->nav_sat);
                break;
            default:
                break;
        }
        end:
        return ret;
}

static void gpsTask(void *parameter) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    uint32_t now = 0, loops = 0, mt = 0;
    ubx_config_t *ubx_dev = gps->ubx_device;
    ubx_msg_byte_ctx_t ubx_packet = UBX_MSG_BYTE_CTX_DEFAULT(ubx_dev->ubx_msg);
    ubx_packet.msg_match_to_pos = false;
    ubx_packet.msg_ready_handler = ubx_msg_checksum_handler;
    ubx_msg_t * ubxMessage = &ubx_dev->ubx_msg;
    struct nav_pvt_s * nav_pvt = &ubxMessage->navPvt;
    uint8_t try_setup_times = 5;
    while (lctx.gps_task_is_running) {
// #if (C_LOG_LEVEL < 2 || defined(DEBUG))
//         if (loops++ > 500) {
//             task_memory_info(__func__);
//             loops = 0;
//         }
// #endif
        now = get_millis();
        if (!gps_has_version_set() || lctx.ubx_restart_requested) {
            mt = now - (ubx_dev->ready ? ubx_dev->ready_time : SEC_TO_MS(5));
            // ILOG(TAG, "[%s] Gps init ... (%lums)", __FUNCTION__, mt);
            if (mt > SEC_TO_MS(10)) { // 5 seconds
                if(ubx_dev->ready){
                    ubx_off(ubx_dev);
                    delay_ms(100);
                    lctx.ubx_fail_count++;
                }
                ubx_setup(ubx_dev);
            }
            if(lctx.ubx_fail_count>50) {
                if(!gps_has_version_set()) { // only when no hwVersion is received
                    esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_GPS_REQUEST_RESTART, NULL, 0, portMAX_DELAY);
                    //m_context.request_restart = true;
                    ILOG(TAG, "[%s] Gps init failed, restart requested.", __FUNCTION__);
                }
                else
                    lctx.ubx_fail_count = 0;
            }
            lctx.ubx_restart_requested = 0;
            goto loop_tail;
        }
        else if(vfs_ctx.gps_log_part < VFS_PART_MAX) {
            if(!lctx.output_rate_swp && rtc_config.output_rate >= UBX_OUTPUT_5HZ && vfs_ctx.parts[vfs_ctx.gps_log_part].free_bytes < TO_K_UL(700)) {
                WLOG(TAG, "[%s] vfs log part %s free space too low: %llu!", __FUNCTION__, vfs_ctx.parts[vfs_ctx.gps_log_part].mount_point, vfs_ctx.parts[vfs_ctx.gps_log_part].free_bytes);
                lctx.output_rate_swp = rtc_config.output_rate;
                rtc_config.output_rate = UBX_OUTPUT_5HZ;
                set_gps_cfg_item(gps_cfg_sample_rate, 1);
            }
            else if(lctx.output_rate_swp && rtc_config.output_rate < UBX_OUTPUT_5HZ && vfs_ctx.parts[vfs_ctx.gps_log_part].free_bytes >= TO_K_UL(700)) {
                WLOG(TAG, "[%s] vfs log part %s free space ok again: %llu!", __FUNCTION__, vfs_ctx.parts[vfs_ctx.gps_log_part].mount_point, vfs_ctx.parts[vfs_ctx.gps_log_part].free_bytes);
                rtc_config.output_rate =(lctx.output_rate_swp == UBX_OUTPUT_5HZ) ? UBX_OUTPUT_10HZ :
                                        (lctx.output_rate_swp == UBX_OUTPUT_10HZ) ? UBX_OUTPUT_16HZ :
                                        (lctx.output_rate_swp == UBX_OUTPUT_16HZ) ? UBX_OUTPUT_20HZ :
                                        UBX_OUTPUT_1HZ; /// have to set 1 step above to calculate saved rate
                lctx.output_rate_swp = 0;
                set_gps_cfg_item(gps_cfg_sample_rate, 1);
            }
        }
        esp_err_t ret = ubx_msg_handler(ubx_dev, &ubx_packet);
        if(!ret) { // only decoding if no Wifi connection}
            ubx_msg_do(&ubx_packet);
            lctx.ubx_fail_count = 0;
        } else {
            if((gps_has_version_set() && (ret == ESP_ERR_TIMEOUT && lctx.ubx_fail_count>3)) || lctx.ubx_fail_count>20) {
                if(ubx_dev->ready) {
                    ubx_dev->ready = false;
                    ubxMessage->mon_ver.hwVersion[0] = 0;
                }
                lctx.ubx_fail_count++;
                goto loop_tail;
            }
            lctx.ubx_fail_count+=5;
        }
        
    loop_tail:
        delay_ms(10);
    }
    lctx.gps_task_handle = 0;
    vTaskDelete(NULL);
}

#if !defined(CONFIG_GPS_LOG_STATIC_A_BUFFER) || !defined(CONFIG_GPS_LOG_STATIC_S_BUFFER)
static void gps_on_sample_rate_change(void* handler_arg, esp_event_base_t base, int32_t id, void* event_data) {
    struct ubx_config_s *ubx = (struct ubx_config_s *)handler_arg;
    if (ubx) {
#if !defined(CONFIG_GPS_LOG_STATIC_A_BUFFER)
        gps_check_alfa_buf(ALPHA_BUFFER_SIZE(ubx->rtc_conf->output_rate));
#endif
#if !defined(CONFIG_GPS_LOG_STATIC_S_BUFFER)
        gps_check_sec_buf(BUFFER_SEC_SIZE);
#endif
        gps_speed_metrics_init();
        refresh_gps_speeds_by_distance();
    }
}

static void gps_on_ubx_deinit(void* handler_arg, esp_event_base_t base, int32_t id, void* event_data) {
#if !defined(CONFIG_GPS_LOG_STATIC_A_BUFFER)
    gps_free_alfa_buf();
#endif
#if !defined(CONFIG_GPS_LOG_STATIC_S_BUFFER)
    gps_free_sec_buf();
#endif
}
#endif

void gps_task_start() {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
#if !defined(CONFIG_GPS_LOG_STATIC_A_BUFFER)
    esp_event_handler_register(UBX_EVENT, UBX_EVENT_SAMPLE_RATE_CHANGED, &gps_on_sample_rate_change, gps->ubx_device);
    esp_event_handler_register(UBX_EVENT, UBX_EVENT_UART_DEINIT_DONE, &gps_on_ubx_deinit, NULL);
#endif
    if(!lctx.gps_task_is_running) {
        lctx.gps_task_is_running = true;
        xTaskCreatePinnedToCore(gpsTask,   /* Task function. */
                "gpsTask", /* String with name of task. */
                CONFIG_GPS_LOG_STACK_SIZE,  /* Stack size in bytes. */
                NULL,      /* Parameter passed as input of the task */
                19,         /* Priority of the task. */
                &lctx.gps_task_handle, 1);      /* Task handle. */
#if defined(GPS_TIMER_STATS)
        if(gps_periodic_timer)
            esp_timer_start_periodic(gps_periodic_timer, SEC_TO_US(1));
#endif
    }
}

void gps_task_stop() {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    if(lctx.gps_task_is_running) {
#if defined(GPS_TIMER_STATS)
        if(gps_periodic_timer)
            esp_timer_stop(gps_periodic_timer);
#endif
        lctx.gps_task_is_running = false;
        uint32_t now = get_millis() + SEC_TO_MS(5);
        while(lctx.gps_task_handle && (get_millis() < now)) delay_ms(10);
        if(lctx.gps_task_handle) {
            vTaskDelete(lctx.gps_task_handle);
            lctx.gps_task_handle = 0;
        }
    }
#if !defined(CONFIG_GPS_LOG_STATIC_A_BUFFER) || !defined(CONFIG_GPS_LOG_STATIC_S_BUFFER)
    esp_event_handler_unregister(UBX_EVENT, ESP_EVENT_ANY_ID, &gps_on_sample_rate_change);
    esp_event_handler_unregister(UBX_EVENT, ESP_EVENT_ANY_ID, &gps_on_ubx_deinit);
#endif
#if !defined(CONFIG_GPS_LOG_STATIC_A_BUFFER)
    gps_free_alfa_buf();
#endif
#if !defined(CONFIG_GPS_LOG_STATIC_S_BUFFER)
    gps_free_sec_buf();
#endif
    gps_speed_metrics_free();
#if (C_LOG_LEVEL < 2)
    ILOG(TAG, "[%s] done.", __func__);
#endif
}

void gps_init(gps_context_t * _gps) {
    gps = _gps;
    init_gps_context_fields(gps);
#if defined(GPS_TIMER_STATS)
    const esp_timer_create_args_t gps_periodic_timer_args = {
            .callback = &s,
            .name = "gps_periodic",
            .arg = _gps->ubx_device,
        };
        if(esp_timer_create(&gps_periodic_timer_args, &gps_periodic_timer)){
            ESP_LOGE(TAG, "[%s] Failed to create periodic timer", __func__);
        }
#endif
}

void gps_deinit(void) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
#if defined(GPS_TIMER_STATS)
    esp_timer_delete(gps_periodic_timer);
#endif
    deinit_gps_context_fields(gps);
}

int gps_shut_down() {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    if(!gps) return 0;
    struct ubx_config_s *ubx_dev = gps->ubx_device;
    if (!ubx_dev) return 0;
    int ret = 0;
    if(!ubx_dev)
        goto end;
    ubx_dev->shutdown_requested = 1;
    if (ubx_dev->ready) {
        gps->signal_ok = false;
        lctx.GPS_delay = 0;
    }
    if (gps->time_set) {  // Only safe to RTC memory if new GPS data is available !!
        esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_GPS_SAVE_FILES, NULL, 0, portMAX_DELAY);
        // next_screen = CUR_SCREEN_SAVE_SESSION;
        // gps_save_rtc();
        if (gps->files_opened) {
            gps_speed_metrics_save_session();
            close_files(gps);
        }
    }
    gps_task_stop();
    ubx_off(ubx_dev);
    gps->time_set = 0;
    if(lctx.output_rate_swp) {
        rtc_config.output_rate = lctx.output_rate_swp;
        lctx.output_rate_swp = 0;
    }
    end:
    esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_GPS_SHUT_DOWN_DONE, NULL, 0, portMAX_DELAY);
    // if (!no_sleep) {
    //     go_to_sleep(3);  // got to sleep after 5 s, this to prevent booting when
    //     // GPIO39 is still low !
    // }
    // if(next_screen == CUR_SCREEN_SAVE_SESSION) {
    //     next_screen = CUR_SCREEN_NONE;
    // }
    return ret;
}
