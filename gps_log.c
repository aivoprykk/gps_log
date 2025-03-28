#include "log_private.h"
#include "gps_data.h"
#include "gps_log_file.h"
#include "gps_log_events.h"
#include "ubx.h"
#include "ubx_msg.h"
#include "logger_common.h"
#include "dstat_screens.h"

#include "esp_mac.h"

static const char *TAG = "gps_log";

#if defined(CONFIG_GPS_LOG_LEVEL_TRACE) && defined(SHOW_SAVED_FRAMES)
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
    assert(ubx_dev);
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

static uint32_t last_flush_time = 0;
static bool gps_is_moving = false;
bool gps_task_is_running = false;
TaskHandle_t gps_task_handle = 0;
static uint8_t ubx_fail_count = 0;

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
    struct ubx_config_s *ubx_dev = gps->ubx_device;
    nav_pvt_t *pvt = &ubx_dev->ubx_msg.navPvt;
    if (!pvt->numSV || pvt->year < 2023) { // no sats or no valid time
        return 0;
    }
    uint32_t millis = get_millis();
    if (gps->time_set && millis < gps->next_time_sync) { // time already set and not time to sync
        return 0;
    }
    //ESP_LOGI(TAG, "[%s] sats: %" PRIu8, __FUNCTION__, pvt->numSV);
    struct tm my_time={0};      // time elements structure
    time_t unix_timestamp = 0;  // a timestamp in seconds
#if defined(DLS)
    // summertime is on march 26 2023 2 AM, see
    // https://www.di-mgt.com.au/wclock/help/wclo_tzexplain.html
    setenv("TZ", "CET0CEST,M3.5.0/2,M10.5.0/3", 1);  // timezone UTC = CET, Daylightsaving ON :
                                                     // TZ=CET-1CEST,M3.5.0/2,M10.5.0/3
    tzset();                                         // this works for CET, but TZ string is different for every Land /
                                                     // continent....
#else
    setenv("TZ", "UTC", 0);
    tzset();
#endif
    my_time.tm_sec = pvt->second;
    my_time.tm_hour = pvt->hour;
    my_time.tm_min = pvt->minute;
    my_time.tm_mday = pvt->day;
    my_time.tm_mon = pvt->month - 1;     // mktime needs months 0 - 11
    my_time.tm_year = pvt->year - 1900;  // mktime needs years since 1900, so deduct 1900
    unix_timestamp = mktime(&my_time);                // mktime returns local time, so TZ is important !!!
    int64_t utc_ms = unix_timestamp * 1000LL + (pvt->nano + 500000) / 1000000LL;
    ESP_LOGW(TAG, "GPS raw time: %d-%02d-%02d %02d:%02d:%02d %" PRId64, pvt->year, pvt->month, pvt->day, pvt->hour, pvt->minute, pvt->second, utc_ms);
    struct timeval tv = {
        .tv_sec = (time_t)(unix_timestamp + (time_offset * 3600)),
        .tv_usec = 0};  // clean utc time !!
    settimeofday(&tv, NULL);
    struct tm tms;
    localtime_r(&unix_timestamp, &tms);
    ESP_LOGW(TAG, "GPS time set: %d-%02d-%02d %02d:%02d:%02d", (tms.tm_year) + 1900, (tms.tm_mon) + 1, tms.tm_mday, tms.tm_hour, tms.tm_min, tms.tm_sec);
    if (tms.tm_year < 123) {
        ESP_LOGW(TAG, "GPS Reported year not plausible (<2023)!");
        return 0;
    }
    ESP_ERROR_CHECK(esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_GPS_TIME_SET, NULL,0, portMAX_DELAY));
    gps->next_time_sync = millis + (60000); // 1 minute
    gps->time_set = 1;
    return 1;
}


static  esp_err_t ubx_msg_do(ubx_msg_byte_ctx_t *ubx_packet) {
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
                    if ((nav_pvt->numSV >= MIN_numSV_FIRST_FIX) && ((nav_pvt->sAcc / 1000.0f) < MAX_Sacc_FIRST_FIX) && (nav_pvt->valid >= 7) && (gps->signal_ok == false)) {
                        gps->signal_ok = true;
                        gps->first_fix = (now - ubx_dev->ready_time) / 1000;
                        ESP_LOGI(TAG, "[%s] First GPS Fix after %"PRIu16" sec.", __FUNCTION__, gps->first_fix);
                        esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_GPS_FIRST_FIX, NULL, 0, portMAX_DELAY);
                        // if(!m_context_rtc.RTC_screen_auto_refresh) {
                        //     lcd_ui_task_resume_for_times(1, -1, -1, true);
                        // }
                    }
                    if (gps->signal_ok && gps->GPS_delay < UINT8_MAX) {
                        gps->GPS_delay++; // delay max is 255 ubx messages for now
                    }

                    set_time(log_get_tz(gps));

                    if (!gps->files_opened && gps->signal_ok && (gps->GPS_delay > (TIME_DELAY_FIRST_FIX * ubx_dev->rtc_conf->output_rate))) {  // vertraging Gps_time_set is nu 10 s!!
                        int32_t avg_speed = 0;
                        avg_speed = (avg_speed + nav_pvt->gSpeed * 19) / 20;  // FIR filter average speed of last 20 measurements in mm/s
                        if (avg_speed > STANDSTILL_DETECTION_MAX) { // 1 m/s == 3.6 km/h
                            if (gps->time_set) {
                                gps->start_logging_millis = now;
                                open_files(gps);  // only start logging if GPS signal is OK
                                esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_LOG_FILES_OPENED, NULL, 0, portMAX_DELAY);
                            } else {
                                ESP_LOGE(TAG, "[%s] gps time not set!", __FUNCTION__);
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
                        gps->interval_gps_msg = nav_pvt->iTOW - gps->old_nav_pvt_itow;
                        gps->old_nav_pvt_itow = nav_pvt->iTOW;
        
                        if (gps->files_opened && (now - last_flush_time) > 60000) {
                            flush_files(gps);
                            last_flush_time = now;
                        }
                        // init run if speed is 0 or sAcc > 1 or speed > 35 m/s
                        if ((nav_pvt->numSV <= MIN_numSV_GPS_SPEED_OK) || ((nav_pvt->sAcc / 1000.0f) > MAX_Sacc_GPS_SPEED_OK) || (nav_pvt->gSpeed / 1000.0f > MAX_GPS_SPEED_OK)) {
#if (CONFIG_LOGGER_COMMON_LOG_LEVEL < 3)
                            const char * c = " GPS run cancelled ";
                            if(nav_pvt->numSV <= MIN_numSV_GPS_SPEED_OK) ESP_LOGE(TAG, "[%s]%s %hhu <= %d MIN_numSV_GPS_SPEED_OK", __FUNCTION__, c, nav_pvt->numSV, (int)MIN_numSV_GPS_SPEED_OK);
                            if((nav_pvt->sAcc / 1000.0f) > MAX_Sacc_GPS_SPEED_OK) ESP_LOGE(TAG, "[%s]%s%lu > %d MAX_Sacc_GPS_SPEED_OK", __FUNCTION__, c, nav_pvt->sAcc/1000, (int)MAX_Sacc_GPS_SPEED_OK);
                            if(nav_pvt->gSpeed / 1000.0f > MAX_GPS_SPEED_OK) ESP_LOGE(TAG, "[%s]%s%ld > %d MAX_GPS_SPEED_OK", __FUNCTION__, c, nav_pvt->gSpeed, (int)MAX_GPS_SPEED_OK);
#endif
                            gps->gps_speed = 0;
                            gps_data->run_start_time = 0;
                        }
                        //saved_count++;
                        if (gps->gps_speed > STANDSTILL_DETECTION_MAX) { // log only when speed is above 1 m/s == 3.6 km/h
                            log_to_file(gps);  // here it is also printed to serial !!
                            if(!gps_is_moving) {
#if (CONFIG_LOGGER_COMMON_LOG_LEVEL < 1)
                                    ILOG(TAG, "[%s] GPS moving detected (%ld ms), run start event sent.\n", __FUNCTION__, gps->gps_speed);
#endif
                                gps_is_moving = true;
                                esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_GPS_IS_MOVING, NULL, 0, portMAX_DELAY);
                            }
                            // if(!m_context_rtc.RTC_screen_auto_refresh && lcd_ui_task_is_paused()) {
                            //     lcd_ui_task_resume();
                            // }
                        }
                        else {
                            if(gps_is_moving) {
#if (CONFIG_LOGGER_COMMON_LOG_LEVEL < 1)
                                    ILOG(TAG,"[%s] GPS stop run detected (%ld ms), stop event sent.\n", __FUNCTION__, gps->gps_speed);
#endif
                               gps_is_moving = false;
                                esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_GPS_IS_STOPPING, NULL, 0, portMAX_DELAY);
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
                        ret = push_gps_data(gps, gps_data, nav_pvt->lat / 10000000.0f, nav_pvt->lon / 10000000.0f, gps->gps_speed);
                        if(ret){
                        #if defined(GPS_TIMER_STATS)
                            ++push_failed_count;
                        #endif
                        #if (CONFIG_LOGGER_COMMON_LOG_LEVEL < 3)
                            ESP_LOGE(TAG, "[%s] push msg failed, timer: %lu, msg_count: %lu ...\n", __func__, now, ubxMessage->count_nav_pvt);
                        #endif
                            goto end;
                        }
                        #if defined(GPS_TIMER_STATS)
                        else
                            ++push_ok_count;
                        #endif
                        gps->run_count = new_run_detection(gps, nav_pvt->heading / 100000.0f, gps->S2.avg_s);
                        gps->alfa_window = alfa_indicator(gps, nav_pvt->heading / 100000.0f);
                        // new run detected, reset run distance
                        if (gps->run_count != gps->old_run_count) {
                            gps_data->run_distance = 0;
                            if (gps->gps_speed / 1000.0f > STANDSTILL_DETECTION_MAX) {
#if (CONFIG_LOGGER_COMMON_LOG_LEVEL < 1)
                            ILOG(TAG, "[%s] GPS new run detected (%ld ms).\n", __FUNCTION__, gps->gps_speed);
#endif
                                gps_data->run_start_time = now;
                                gps->record = 0;
                                esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_GPS_NEW_RUN, NULL, 0, portMAX_DELAY);
                                // cancel_lcd_ui_delay();
                            }
                        }
                        gps->old_run_count = gps->run_count;
                        update_distance(gps, &gps->M100);
                        update_distance(gps, &gps->M250);
                        update_distance(gps, &gps->M500);
                        update_distance(gps, &gps->M1852);
                        update_speed(gps, &gps->S2);
                        update_speed(gps, &gps->s2);
                        update_speed(gps, &gps->S10);
                        update_speed(gps, &gps->s10);
                        update_speed(gps, &gps->S1800);
                        update_speed(gps, &gps->S3600);
                        update_alfa_speed(gps, &gps->A250, &gps->M250);
                        update_alfa_speed(gps, &gps->A500, &gps->M500);
                        update_alfa_speed(gps, &gps->a500, &gps->M500);
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
                // ubxMessage->nav_sat.iTOW=ubxMessage->nav_sat.iTOW-18*1000; //to match 18s diff UTC nav pvt & GPS nav sat !!!
                push_gps_sat_info(&gps->Ublox_Sat, &ubxMessage->nav_sat);
                break;
            default:
                break;
        }
        end:
        return ret;
}

static void gpsTask(void *parameter) {
    ILOG(TAG, "[%s]", __func__);
    uint32_t now = 0, loops = 0, mt = 0;
    ubx_config_t *ubx_dev = gps->ubx_device;
    ubx_msg_byte_ctx_t ubx_packet = UBX_MSG_BYTE_CTX_DEFAULT(ubx_dev->ubx_msg);
    ubx_packet.msg_match_to_pos = false;
    ubx_packet.msg_ready_handler = ubx_msg_checksum_handler;
    ubx_msg_t * ubxMessage = &ubx_dev->ubx_msg;
    struct nav_pvt_s * nav_pvt = &ubxMessage->navPvt;
    uint8_t try_setup_times = 5;
    while (gps_task_is_running) {
// #if (C_LOG_LEVEL < 2 || defined(DEBUG))
//         if (loops++ > 500) {
//             task_memory_info(__func__);
//             loops = 0;
//         }
// #endif
        now = get_millis();
        if (!ubx_dev->ready || !ubxMessage->mon_ver.hwVersion[0] || gps->ubx_restart_requested) {
            mt = now - (ubx_dev->ready ? ubx_dev->ready_time : 5000);
            // ILOG(TAG, "[%s] Gps init ... (%lums)", __FUNCTION__, mt);
            if (mt > 10000) { // 5 seconds
                if(ubx_dev->ready){
                    ubx_off(ubx_dev);
                    delay_ms(100);
                    ubx_fail_count++;
                }
                ubx_setup(ubx_dev);
            }
            if(ubx_fail_count>50) {
                if(!ubxMessage->mon_ver.hwVersion[0]) { // only when no hwVersion is received
                    esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_GPS_REQUEST_RESTART, NULL, 0, portMAX_DELAY);
                    //m_context.request_restart = true;
                    ILOG(TAG, "[%s] Gps init failed, restart requested.", __FUNCTION__);
                }
                else
                    ubx_fail_count = 0;
            }
            gps->ubx_restart_requested = 0;
            goto loop_tail;
        }
        esp_err_t ret = ubx_msg_handler(ubx_dev, &ubx_packet);
        if(!ret) { // only decoding if no Wifi connection}
            ubx_msg_do(&ubx_packet);
            ubx_fail_count = 0;
        } else {
            if((ubx_dev->ready && ubxMessage->mon_ver.hwVersion[0] && (ret == ESP_ERR_TIMEOUT && ubx_fail_count>3)) || ubx_fail_count>20) {
                if(ubx_dev->ready) {
                    ubx_dev->ready = false;
                    ubxMessage->mon_ver.hwVersion[0] = 0;
                }
                ubx_fail_count++;
                goto loop_tail;
            }
            ubx_fail_count+=5;
        }
        
    loop_tail:
        delay_ms(10);
    }
    gps_task_handle = 0;
    vTaskDelete(NULL);
}

void gps_task_start() {
    ILOG(TAG, "[%s]", __func__);
    if(!gps_task_is_running) {
        gps_task_is_running = true;
        xTaskCreatePinnedToCore(gpsTask,   /* Task function. */
                "gpsTask", /* String with name of task. */
                CONFIG_GPS_LOG_STACK_SIZE,  /* Stack size in bytes. */
                NULL,      /* Parameter passed as input of the task */
                19,         /* Priority of the task. */
                &gps_task_handle, 1);      /* Task handle. */
#if defined(GPS_TIMER_STATS)
        if(gps_periodic_timer)
            ESP_ERROR_CHECK(esp_timer_start_periodic(gps_periodic_timer, 1000000));
#endif
    }
}

void gps_task_stop() {
    ILOG(TAG, "[%s]", __func__);
    if(gps_task_is_running) {
#if defined(GPS_TIMER_STATS)
        if(gps_periodic_timer)
            ESP_ERROR_CHECK(esp_timer_stop(gps_periodic_timer));
#endif
        gps_task_is_running = false;
        uint32_t now = get_millis();
        while(gps_task_handle && (get_millis() - now) < 5000) delay_ms(10);
        if(gps_task_handle) {
            vTaskDelete(gps_task_handle);
            gps_task_handle = 0;
        }
    }
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
        ESP_ERROR_CHECK(esp_timer_create(&gps_periodic_timer_args, &gps_periodic_timer));
#endif
}

void gps_deinit(void) {
    ILOG(TAG, "[%s]", __func__);
#if defined(GPS_TIMER_STATS)
    ESP_ERROR_CHECK(esp_timer_delete(gps_periodic_timer));
#endif
}

int gps_shut_down() {
    ILOG(TAG, "[%s]", __func__);
    if(!gps) return 0;
    struct ubx_config_s *ubx_dev = gps->ubx_device;
    if ((!ubx_dev || !ubx_dev->uart_setup_ok)) return 0;
    int ret = 0;
    if(!ubx_dev)
        goto end;
    if (ubx_dev->ready) {
        gps->signal_ok = false;
        gps->GPS_delay = 0;
    }
    if (gps->time_set) {  // Only safe to RTC memory if new GPS data is available !!
        esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_GPS_SAVE_FILES, NULL, 0, portMAX_DELAY);
        // next_screen = CUR_SCREEN_SAVE_SESSION;
        // gps_save_rtc();
        if (gps->files_opened) {
            if (GETBIT(gps->log_config->log_file_bits, SD_TXT)) {
                session_info(gps, &gps->Ublox);
                session_results_s(gps, &gps->S2);
                session_results_s(gps, &gps->S10);
                session_results_s(gps, &gps->S1800);
                session_results_s(gps, &gps->S3600);
                session_results_m(gps, &gps->M100);
                session_results_m(gps, &gps->M500);
                session_results_m(gps, &gps->M1852);
                session_results_alfa(gps, &gps->A250, &gps->M250);
                session_results_alfa(gps, &gps->A500, &gps->M500);
            }
            close_files(gps);
        }
    }
    gps_task_stop();
    ubx_off(ubx_dev);
    gps->time_set = 0;
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