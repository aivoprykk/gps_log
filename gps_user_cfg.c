#include "log_private.h"
#include "gps_user_cfg.h"
#include "gps_data.h"

#include "strbf.h"
#if defined(CONFIG_GPS_LOG_USE_CJSON)
#include "cJSON.h"
#else
#include "json.h"
#endif
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "ubx.h"
#include "gps_log_file.h"
#include "gps_log_events.h"
#include "common_log.h"
#include "common_cfg.h"

static const char * TAG = "gps_user_cfg"; // for logging

// Optimized string constants to reduce memory usage
static const char str_on[] = "on";
static const char str_off[] = "off";
static const char str_not_set[] = "not set";
static const char str_ubx[] = ".ubx";
static const char str_sbp[] = ".sbp";
static const char str_txt[] = ".txt";
static const char str_gpx[] = ".gpx";
static const char str_gpy[] = ".gpy";

// Pre-calculated timeout for common semaphore operations
static const TickType_t timeout_max = portMAX_DELAY;

#define CFG_TO_BASE(l) (l-CFG_GPS_ITEM_BASE)
#define SPEED_UNIT_ITEM_LIST(l) l(m/s) l(km/h) l(knots) l(m/ph)
#define SAMPLE_RATE_ITEM_LIST(l) l(1 Hz) l(2 Hz) l(5 Hz) l(10 Hz) l(16 Hz) l(20 Hz)
/// Portable -  Applications with low acceleration, e.g. portable devices. Suitable for most situations; max velocity is 310ms deviation medium
/// Sea - Recommended for applications at sea, with zero vertical velocity. Zero vertical velocity assumed. Sea level assumed; max velocity is 25ms deviation medium
/// Automotive - Used for applications with equivalent dynamics to those of a passenger car. Low vertical acceleration assumed; max velocity 100ms deviation medium
/// Pedestrian Applications with low acceleration and speed, e.g. how a pedestrian would move. Low acceleration assumed; max velocity 30ms, deviation small
/// 0 Portable, 2 Stationary, 3 Pedestrian, 4 Automotive, 5 Sea
#define DYNAMIC_MODEL_ITEM_LIST(l) l(Portable) l(Pedestrian) l(Automotive) l(Sea)
#define GSS_DESC_ITEM_LIST(l) l(G + E + B + R) l(G + B + R) l(G + E + R) l(G + E + B) l(G + R) l(G + B) l(G + E)
#define GNSS_DESC_VAL_LIST(l) l(111) l(107) l(103) l(47) l(99) l(43) l(39)
#define TIMEZONE_ITEM_LIST(l) l(UTC) l(UTC+1) l(UTC+2) l(UTC+3)
#define FILE_DATE_TIME_ITEM_LIST(l) l(name_mac_index) l(name_date_time) l(date_time_name)
#define L(x) x,
static const char * const config_gps_items[] = { CFG_GPS_USER_CFG_ITEM_LIST(STRINGIFY_V) };
const size_t gps_user_cfg_item_count = sizeof(config_gps_items) / sizeof(config_gps_items[0]);
const char * const speed_units[] = {SPEED_UNIT_ITEM_LIST(STRINGIFY)};
static const char * const sample_rates[] = {SAMPLE_RATE_ITEM_LIST(STRINGIFY)};
const char * const config_stat_screen_items[] = { STAT_SCREEN_ITEM_LIST(STRINGIFY) };
const size_t gps_stat_screen_item_count = sizeof(config_stat_screen_items) / sizeof(config_stat_screen_items[0]);

const char * const dynamic_models[] = {DYNAMIC_MODEL_ITEM_LIST(STRINGIFY)};
static const char * const gnss_desc[] = {GSS_DESC_ITEM_LIST(STRINGIFY)};
static const uint8_t gnss_desc_val[] = {GNSS_DESC_VAL_LIST(L)};
static const char * const timezone_items[] = {TIMEZONE_ITEM_LIST(STRINGIFY)};
static const char * const file_date_time_items[] = {FILE_DATE_TIME_ITEM_LIST(STRINGIFY)};

static SemaphoreHandle_t c_sem_lock = 0;
// global unit for gps user config.
RTC_DATA_ATTR struct gps_user_cfg_s c_gps_cfg = GPS_USER_CFG_DEFAULTS();
extern struct gps_context_s *gps;
extern gps_log_file_config_t log_config;

typedef struct gps_user_cfg_stat_screens_s {
    STAT_SCREEN_ITEM_LIST(GPS_STAT_S)
} gps_user_cfg_stat_screens_t;

#define GPS_CFG_STAT_SCREENS_DEFAULTS() { \
    STAT_SCREEN_ITEM_LIST(GPS_STAT_D) \
}

void gps_user_cfg_init(void) {
    if(!c_sem_lock) {
        c_sem_lock = xSemaphoreCreateMutex();
    }
    if(!c_sem_lock) {
        ELOG(TAG, "Failed to create semaphore");
    }
}

void gps_user_cfg_deinit(void) {
    if(c_sem_lock) {
        vSemaphoreDelete(c_sem_lock);
        c_sem_lock = 0;
    }
}

static bool gps_cfg_lock(int timeout) {
    if (!c_sem_lock) return false;
    const TickType_t timeout_ticks = (timeout == -1) ? timeout_max : pdMS_TO_TICKS(timeout);
    return  xSemaphoreTake(c_sem_lock, timeout_ticks) == pdTRUE;
}

static void gps_cfg_unlock() {
    if (c_sem_lock) {
        xSemaphoreGive(c_sem_lock);
    }
}

struct m_config_item_s * get_stat_screen_cfg_item(int num, struct m_config_item_s *item) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s] num:%d", __func__, num);
#endif
    if(!item) return 0;
    if(gps_cfg_lock(timeout_max) == pdTRUE) {
        if(num>=0 && num<gps_stat_screen_item_count) {
            item->name = config_stat_screen_items[num];
            item->pos = num;
            item->value = GETBIT(c_gps_cfg.stat_screens,num);
            item->desc = item->value ? str_on : str_off;
        }
        gps_cfg_unlock();
    }
    esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_CFG_GET, &num, sizeof(num), timeout_max);
    return item;
}

int set_stat_screen_cfg_item(int num) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s] num:%d", __func__, num);
#endif
    if(num>=gps_stat_screen_item_count) return 0;
    if(gps_cfg_lock(timeout_max) == pdTRUE) {
        uint16_t val = c_gps_cfg.stat_screens;
        if(num>=0 && num<gps_stat_screen_item_count) {
            val ^= (BIT(num));
        }
        if(val!=c_gps_cfg.stat_screens) {
            c_gps_cfg.stat_screens = val;
        }
        gps_cfg_unlock();
    }
    esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_CFG_SET, &num, sizeof(num), timeout_max);
    return 1;
}

struct m_config_item_s * get_gps_cfg_item(int num, struct m_config_item_s *item) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s] num:%d", __func__, num);
#endif
    if(!item) return 0;
    if(num < CFG_GPS_ITEM_BASE || num > CFG_GPS_ITEM_BASE + gps_user_cfg_item_count) num = CFG_GPS_ITEM_BASE;
    item->name = config_gps_items[CFG_TO_BASE(num)];
    item->pos = num;
    if(gps_cfg_lock(timeout_max) == pdTRUE) {
        switch (num)  {
            case gps_cfg_gnss:
                item->value = rtc_config.gnss;
                for (int i = 0; i < sizeof(gnss_desc_val) / sizeof(gnss_desc_val[0]); i++) {
                    if (rtc_config.gnss == gnss_desc_val[i]) {
                        item->desc = gnss_desc[i];
                        break;
                    }
                }
                if (!item->desc) {
                    item->desc = str_not_set;
                }
                break;
            case gps_cfg_sample_rate:
                item->value = rtc_config.output_rate;
                switch(rtc_config.output_rate) {
                    case UBX_OUTPUT_1HZ: item->desc = sample_rates[0]; break;
                    case UBX_OUTPUT_2HZ: item->desc = sample_rates[1]; break;
                    case UBX_OUTPUT_5HZ: item->desc = sample_rates[2]; break;
                    case UBX_OUTPUT_10HZ: item->desc = sample_rates[3]; break;
                    case UBX_OUTPUT_16HZ: item->desc = sample_rates[4]; break;
                    case UBX_OUTPUT_20HZ: item->desc = sample_rates[5]; break;
                    default: item->desc = str_not_set; break;
                }
                break;
            case gps_cfg_timezone:
                item->value = c_gps_cfg.timezone;
                for(int i = 0; i < lengthof(timezone_items); i++) {
                    if(c_gps_cfg.timezone == i) {
                        item->desc = timezone_items[i];
                        break;
                    }
                }
                break;
            case gps_cfg_speed_unit:
                item->value = c_gps_cfg.speed_unit;
                item->desc = speed_units[c_gps_cfg.speed_unit];
                break;
            case gps_cfg_log_txt:
                item->value = GETBIT(log_config.log_file_bits, SD_TXT) ? 1 : 0;
                item->desc = item->value ? str_on : str_off;
                break;
            case gps_cfg_log_ubx:
                item->value = GETBIT(log_config.log_file_bits, SD_UBX) ? 1 : 0;
                item->desc = item->value ? str_on : str_off;
                break;
            case gps_cfg_log_sbp:
                item->value = GETBIT(log_config.log_file_bits, SD_SBP) ? 1 : 0;
                item->desc = item->value ? str_on : str_off;
                break;
            case gps_cfg_log_gpx:
                item->value = GETBIT(log_config.log_file_bits, SD_GPX) ? 1 : 0;
                item->desc = item->value ? str_on : str_off;
                break;
#ifdef GPS_LOG_ENABLE_GPY
            case gps_cfg_log_gpy:
                item->value = GETBIT(log_config.log_file_bits, SD_GPY) ? 1 : 0;
                item->desc = item->value ? str_on : str_off;
                break;
#endif
            case gps_cfg_log_ubx_nav_sat:
                item->value = rtc_config.msgout_sat ? 1 : 0;
                item->desc = rtc_config.msgout_sat ? str_on : str_off;
                break;
            // case gps_cfg_dynamic_model_auto:
            //     item->value = rtc_config.nav_mode_auto ? 1 : 0;
            //     item->desc = rtc_config.nav_mode_auto ? "on" : "off";
            //     break;
            case gps_cfg_dynamic_model:
                item->value = rtc_config.nav_mode;
                switch(rtc_config.nav_mode) {
                    case UBX_MODE_PORTABLE: item->desc = dynamic_models[0]; break;
                    case UBX_MODE_PEDESTRIAN: item->desc = dynamic_models[1]; break;
                    case UBX_MODE_AUTOMOTIVE: item->desc = dynamic_models[2]; break;
                    case UBX_MODE_SEA: item->desc = dynamic_models[3]; break;
                    default: item->desc = str_not_set; break;
                }
                break;
            case gps_cfg_file_date_time:
                item->value = c_gps_cfg.file_date_time;
                item->desc = file_date_time_items[c_gps_cfg.file_date_time];    
                break;
            case gps_cfg_ubx_file:
                item->value = 0;
                item->desc = c_gps_cfg.ubx_file;
                break;
            default:
                item->desc = str_not_set;
                break;
        }
        gps_cfg_unlock();
    }
    esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_CFG_GET, &item, sizeof(item), timeout_max);
    return item;
}

void gps_config_fix_values(void) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    if(!gps_log_file_bits_check(log_config.log_file_bits))
        SETBIT(log_config.log_file_bits, SD_SBP);
    if(rtc_config.gnss == 0) {
        rtc_config.gnss = 111; // default to GPS + GLONASS + BEIDOU + GALILEO
    }
}

static void set_gps_time_out_msg(void) {
#if C_LOG_LEVEL < 3
    ILOG(TAG, "[%s]", __func__);
#endif
    gps->time_out_gps_msg = HZ_TO_MS(rtc_config.output_rate) + 75;  // max time out = 175 ms
}

int set_gps_cfg_item(int num, bool skip_done_msg) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s] num:%d", __func__, num);
#endif
    if(num < CFG_GPS_ITEM_BASE || num > CFG_GPS_ITEM_BASE + gps_user_cfg_item_count) return 255;
    const char *name = config_gps_items[CFG_TO_BASE(num)];
    struct gps_user_cfg_evt_data_s evt_data = {num, 0};
    if(gps_cfg_lock(timeout_max) == pdTRUE){
        switch(num) {
            case gps_cfg_gnss:
                switch(rtc_config.gnss) {
                    case 111: rtc_config.gnss = 107; break;
                    case 107: rtc_config.gnss = 103; break;
                    case 103: rtc_config.gnss = 47; break;
                    case 47: rtc_config.gnss = 99; break;
                    case 99: rtc_config.gnss = 43; break;
                    case 43: rtc_config.gnss = 39; break;
                    case 39: rtc_config.gnss = 111; break;
                    default: rtc_config.gnss = 111; break;
                }
                break;
            case gps_cfg_sample_rate:
                switch(rtc_config.output_rate) {
                    case UBX_OUTPUT_20HZ: rtc_config.output_rate = UBX_OUTPUT_16HZ; break;
                    case UBX_OUTPUT_16HZ: rtc_config.output_rate = UBX_OUTPUT_10HZ; break;
                    case UBX_OUTPUT_10HZ: rtc_config.output_rate = UBX_OUTPUT_5HZ; break;
                    case UBX_OUTPUT_5HZ: rtc_config.output_rate = UBX_OUTPUT_2HZ; break;
                    case UBX_OUTPUT_2HZ: rtc_config.output_rate = UBX_OUTPUT_1HZ; break;
                    default: rtc_config.output_rate = UBX_OUTPUT_20HZ; break;
                }
                break;
            case gps_cfg_timezone:
                evt_data.value = c_gps_cfg.timezone;
                if(c_gps_cfg.timezone == 0) c_gps_cfg.timezone = 1;
                else if(c_gps_cfg.timezone == 1) c_gps_cfg.timezone = 2;
                else if(c_gps_cfg.timezone == 2) c_gps_cfg.timezone = 3;
                else c_gps_cfg.timezone = 0;
                break;
            case gps_cfg_speed_unit:
                if(c_gps_cfg.speed_unit == 1) c_gps_cfg.speed_unit = 0;
                else if(c_gps_cfg.speed_unit == 2) c_gps_cfg.speed_unit = 1;
                else  c_gps_cfg.speed_unit = 2;
                break;
            case gps_cfg_log_txt:
                if(GETBIT(log_config.log_file_bits, SD_TXT) == 1) {
                    CLRBIT(log_config.log_file_bits, SD_TXT);
                }
                else {
                    SETBIT(log_config.log_file_bits, SD_TXT);
                }
                break;
            case gps_cfg_log_ubx:
                if(GETBIT(log_config.log_file_bits, SD_UBX) == 1) {
                    CLRBIT(log_config.log_file_bits, SD_UBX);
                }
                else {
                    SETBIT(log_config.log_file_bits, SD_UBX);
                }
                break;
            case gps_cfg_log_sbp:
                if(GETBIT(log_config.log_file_bits, SD_SBP) == 1) {
                    CLRBIT(log_config.log_file_bits, SD_SBP);
                }
                else {
                    SETBIT(log_config.log_file_bits, SD_SBP);
                }
                break;
            case gps_cfg_log_gpx:
                if(GETBIT(log_config.log_file_bits, SD_GPX) == 1) {
                    CLRBIT(log_config.log_file_bits, SD_GPX);
                }
                else {
                    SETBIT(log_config.log_file_bits, SD_GPX);
                }
                break;
#ifdef GPS_LOG_ENABLE_GPY
            case gps_cfg_log_gpy:
                if(GETBIT(log_config.log_file_bits, SD_GPY) == 1) {
                    CLRBIT(log_config.log_file_bits, SD_GPY);
                }
                else {
                    SETBIT(log_config.log_file_bits, SD_GPY);
                }
                break;
#endif
            case gps_cfg_log_ubx_nav_sat:
                rtc_config.msgout_sat = rtc_config.msgout_sat ? 0 : 1;
                break;
            // case gps_cfg_dynamic_model_auto:
            //     rtc_config.nav_mode_auto = rtc_config.nav_mode_auto ? 0 : 1;
            //     break;
            case gps_cfg_dynamic_model:
                switch(rtc_config.nav_mode) {
                    case UBX_MODE_SEA: rtc_config.nav_mode = UBX_MODE_AUTOMOTIVE; break;
                    case UBX_MODE_AUTOMOTIVE: rtc_config.nav_mode = UBX_MODE_PEDESTRIAN; break;
                    case UBX_MODE_PEDESTRIAN: rtc_config.nav_mode = UBX_MODE_PORTABLE; break;
                    default: rtc_config.nav_mode = UBX_MODE_SEA; break;
                }
                break;
            case gps_cfg_file_date_time:
                if(c_gps_cfg.file_date_time == 0) c_gps_cfg.file_date_time = 1;
                else if(c_gps_cfg.file_date_time == 1) c_gps_cfg.file_date_time = 2;
                else c_gps_cfg.file_date_time = 0;
                if (c_gps_cfg.file_date_time == 0 && GETBIT(log_config.log_file_bits, SD_TXT) == 0)
                    SETBIT(log_config.log_file_bits, SD_TXT);  // because txt file is needed for generating new file count !!
                break;
            default:
                break;
        }
        // config_save_json(config, ublox_hw);
        if((num == gps_cfg_gnss || num == gps_cfg_sample_rate || num == gps_cfg_log_ubx_nav_sat || num == gps_cfg_dynamic_model) && gps && gps->ubx_device && gps->ubx_device->initialized) {
            if(num == gps_cfg_dynamic_model)
                ubx_set_nav_mode(gps->ubx_device, rtc_config.nav_mode);
            else {
                if(num == gps_cfg_sample_rate)
                    set_gps_time_out_msg();
                ubx_set_ggnss_and_rate(gps->ubx_device, rtc_config.gnss, rtc_config.output_rate);
#if C_LOG_LEVEL < 3
                ILOG(TAG, "[%s] set gnss: %d, rate: %d", __func__, rtc_config.gnss, rtc_config.output_rate);
#endif
            }
        }
        gps_cfg_unlock();
    }
    if(!skip_done_msg) /// if set, no message is sent and no cfg save requested
        esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_CFG_SET, &evt_data, sizeof(evt_data), timeout_max);
    return num;
}

uint8_t gps_cfg_get_pos(const char *str) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s] str: %s", __func__, str ? str : "-");
#endif
    if(!str) {
        return 254;
    }
    for (uint8_t i = 0, j = gps_user_cfg_item_count; i < j; i++) {
        if (!strcmp(str, config_gps_items[i])) {
            return i + CFG_GPS_ITEM_BASE;
        }
    }
    if(!strcmp(str, "Stat_screens")) {
        return gps_cfg_stat_screens;
    }
    if(!strcmp(str, "logTXT")) {
        return gps_cfg_log_txt;
    }
    if(!strcmp(str, "logUBX")) {
        return gps_cfg_log_ubx;
    }
    if(!strcmp(str, "logUBX_nav_sat")) {
        return gps_cfg_log_ubx_nav_sat;
    }
    if(!strcmp(str, "logSBP")) {
        return gps_cfg_log_sbp;
    }
    if(!strcmp(str, "logGPX")) {
        return gps_cfg_log_gpx;
    }
#ifdef GPS_LOG_ENABLE_GPY
    if(!strcmp(str, "logGPY")) {
        return gps_cfg_log_gpy;
    }
#endif
    if(!strcmp(str, "UBXfile")) {
        return gps_cfg_ubx_file;
    }
    return 255;
}

uint8_t gps_cnf_set_item(uint8_t pos, void * el, uint8_t force) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s] pos: %hhu", __func__, pos);
#endif
    if (!el) {
        return 254;
    }
    if(pos < CFG_GPS_ITEM_BASE || pos >= CFG_GPS_ITEM_BASE + gps_user_cfg_item_count) {
        return 255;
    }
    uint8_t changed = 255, ret = 0;
#if defined(CONFIG_GPS_LOG_USE_CJSON)
    cJSON *item = (cJSON *)el;
#else
    JsonNode *item = (JsonNode *)el;
#endif
    if(gps_cfg_lock(timeout_max) == pdTRUE) {
        switch (pos) {
            case gps_cfg_file_date_time:
                ret = set_hhu(item, &c_gps_cfg.file_date_time, 0);
                if(!ret) changed = gps_cfg_file_date_time;
                break;
            case gps_cfg_timezone:  // choice for timedifference in hours with UTC, for Belgium 1, 2, 3 for estonia (summer time)
                ret = set_f(item, &c_gps_cfg.timezone, 0);
                if(!ret) changed = gps_cfg_timezone;
                break;
            case gps_cfg_stat_screens: // choice for stats field when no speed, here stat_screen 1, 2 and 3 will be active
                ret = set_u(item, &c_gps_cfg.stat_screens, 0);
                if(!ret) changed = gps_cfg_stat_screens;
                break;
            case gps_cfg_speed_unit:  // conversion m/s to km/h, for knots use 1.944
                ret = set_hhu(item, (uint8_t*)&c_gps_cfg.speed_unit, 0);
                if(!ret) changed = gps_cfg_speed_unit;
                break;
            case gps_cfg_sample_rate:  // gps_rate in Hz, 1, 2, 5 or 10Hz !!!
                ret = set_hhu(item, (uint8_t*)&rtc_config.output_rate, 0);
                if(!ret) changed = gps_cfg_sample_rate;
                set_gps_time_out_msg();
                break;
            case gps_cfg_gnss:  // default setting 2 GNSS, GPS & GLONAS
                ret = set_hhu(item, &rtc_config.gnss, 0);
                if(!ret) changed = gps_cfg_gnss;
                break;
            // case gps_cfg_dynamic_model_auto: /// wether device can switch mode automatically
            //     ret = set_hhu(item, (uint8_t*)&rtc_config.nav_mode_auto, 0);
            //     if(!ret) changed = gps_cfg_dynamic_model_auto;
            //     break;
            case gps_cfg_dynamic_model:  // sea, portable, automotive, pedestrian
                ret = set_hhu(item, (uint8_t*)&rtc_config.nav_mode, 0);
                if(!ret) changed = gps_cfg_dynamic_model;
                break;
            case gps_cfg_log_txt: // || !strcmp(var, "logTXT")) {  // switchinf off .txt files
                ret = set_bit(item, &log_config.log_file_bits, SD_TXT, 0);
                if(!ret) changed = gps_cfg_log_txt;
                goto test_if_log_set; // if no log file selected, set SBP as default
            case gps_cfg_log_ubx: // || !strcmp(var, "logUBX")) {  // log to .ubx
                ret = set_bit(item, &log_config.log_file_bits, SD_UBX, 0);
                if(!ret) changed = gps_cfg_log_ubx;
                goto test_if_log_set; // if no log file selected, set SBP as default
            case gps_cfg_log_ubx_nav_sat: // || !strcmp(var, "logUBX_nav_sat")) {  // log nav sat msg to .ubx
                ret = set_hhu(item, (uint8_t*)&rtc_config.msgout_sat, 0);
                if(!ret) changed = gps_cfg_log_ubx_nav_sat;
                goto test_if_log_set; // if no log file selected, set SBP as default
            case gps_cfg_log_sbp: // || !strcmp(var, "logSBP")) {  // log to .sbp
                ret = set_bit(item, &log_config.log_file_bits, SD_SBP, 0);
                if(!ret) changed = gps_cfg_log_sbp;
                test_if_log_set:
                if(!gps_log_file_bits_check(log_config.log_file_bits))
                    SETBIT(log_config.log_file_bits, SD_SBP); // set SBP if none selected
                break;
            case gps_cfg_log_gpx: //  || !strcmp(var, "logGPX")) {
                ret = set_bit(item, &log_config.log_file_bits, SD_GPX, 0);
                if(!ret) changed = gps_cfg_log_gpx;
                goto test_if_log_set; // if no log file selected, set SBP as default
#ifdef GPS_LOG_ENABLE_GPY
            case gps_cfg_log_gpy: // || !strcmp(var, "logGPY")) {
                ret = set_bit(item, &log_config.log_file_bits, SD_GPY, 0);
                if(!ret) changed = gps_cfg_log_gpy;
                goto test_if_log_set; // if no log file selected, set SBP as default
#endif
            case gps_cfg_ubx_file: // ]) || !strcmp(var, "UBXfile")) {
                ret = set_c(item, &c_gps_cfg.ubx_file[0], 0);
                if(!ret) changed = gps_cfg_ubx_file;
                break;
            default:
                changed = 253;
                break;
        }
        gps_cfg_unlock();
    }
    if(changed < 253){
        esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_CFG_CHANGED, &changed, sizeof(changed), timeout_max);
    }
    return changed;
}

int gps_config_set(const char *str, void *root, uint8_t force) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG,"[%s] name: %s",__func__, str ? str : "-");
#endif
    if (!root)  return 254;
    const char *var = 0;
    uint8_t changed = 255;
#if defined(CONFIG_GPS_LOG_USE_CJSON)
    cJSON *name = 0, *value = 0;
#else
    JsonNode *name = 0, *value = 0;
#endif
    if(str) {
        var = str;
#if defined(CONFIG_GPS_LOG_USE_CJSON)
        value = cJSON_GetObjectItemCaseSensitive(root, str);
#else
        value = json_find_member((JsonNode*)root, str);
#endif
    }
    else {
#if defined(CONFIG_GPS_LOG_USE_CJSON)
        value = cJSON_GetObjectItemCaseSensitive(root, "value");
        name = cJSON_GetObjectItemCaseSensitive(root, "name");
        var = name->valuestring;
#else
        value = json_find_member((JsonNode*)root, "value");
        name = json_find_member((JsonNode*)root, "name");
        var = name->data.string_;
#endif
    }
    uint8_t pos = gps_cfg_get_pos(var);
    if (pos >= 254) {
        DLOG(TAG, "[%s] ! var", __func__);
        changed = 255;
        goto err;
    }
    changed = gps_cnf_set_item(pos, value, force);
err:    
    return changed;
}

esp_err_t gps_config_decode(const char *json) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG,"[%s]",__func__);
#endif
    int ret = ESP_OK;
#if defined(CONFIG_GPS_LOG_USE_CJSON)
    cJSON *root = cJSON_Parse(json);
#else
    JsonNode *root = json_decode(json);
#endif
    if (!root) {
        return ESP_FAIL;
    }
#if defined(CONFIG_GPS_LOG_USE_CJSON)
    cJSON * gps = cJSON_GetObjectItemCaseSensitive(root, "gps"), *item;
#else
    JsonNode * gps = json_find_member(root, "gps"), *item;
#endif
    if(!gps) gps = root;
    for (uint8_t i = 0, j = gps_user_cfg_item_count; i < j; i++) {
#if defined(CONFIG_GPS_LOG_USE_CJSON)
        item = cJSON_GetObjectItemCaseSensitive(gps, config_gps_items[i]);
#else
        item = json_find_member(gps, config_gps_items[i]);
#endif
        if (item) {
            gps_cnf_set_item(i+CFG_GPS_ITEM_BASE, item, 0);
        }
        else {
            if(i+CFG_GPS_ITEM_BASE == gps_cfg_stat_screens) {
#if defined(CONFIG_GPS_LOG_USE_CJSON)
                item = cJSON_GetObjectItemCaseSensitive(gps, "statScreens");
#else
                item = json_find_member(gps, "statScreens");
#endif
            }
            else if(i+CFG_GPS_ITEM_BASE == gps_cfg_log_gpx) {
#if defined(CONFIG_GPS_LOG_USE_CJSON)
                item = cJSON_GetObjectItemCaseSensitive(gps, "logGPX");
#else
                item = json_find_member(gps, "logGPX");
#endif
            }
#ifdef GPS_LOG_ENABLE_GPY
            else if(i+CFG_GPS_ITEM_BASE == gps_cfg_log_gpy) {
#if defined(CONFIG_GPS_LOG_USE_CJSON)
                item = cJSON_GetObjectItemCaseSensitive(gps, "logGPY");
#else
                item = json_find_member(gps, "logGPY");
#endif
            }
#endif
            else if(i+CFG_GPS_ITEM_BASE == gps_cfg_log_sbp) {
#if defined(CONFIG_GPS_LOG_USE_CJSON)
                item = cJSON_GetObjectItemCaseSensitive(gps, "logSBP");
#else
                item = json_find_member(gps, "logSBP");
#endif
            }
            else if(i+CFG_GPS_ITEM_BASE == gps_cfg_log_ubx) {
#if defined(CONFIG_GPS_LOG_USE_CJSON)
                item = cJSON_GetObjectItemCaseSensitive(gps, "logUBX");
#else
                item = json_find_member(gps, "logUBX");
#endif
            }
            else if(i+CFG_GPS_ITEM_BASE == gps_cfg_log_txt) {
#if defined(CONFIG_GPS_LOG_USE_CJSON)
                item = cJSON_GetObjectItemCaseSensitive(gps, "logTXT");
#else
                item = json_find_member(gps, "logTXT");
#endif
            }
            else if(i+CFG_GPS_ITEM_BASE == gps_cfg_log_ubx_nav_sat) {
#if defined(CONFIG_GPS_LOG_USE_CJSON)
                item = cJSON_GetObjectItemCaseSensitive(gps, "logUBX_nav_sat");
#else
                item = json_find_member(gps, "logUBX_nav_sat");
#endif
            } 
            else if(i+CFG_GPS_ITEM_BASE == gps_cfg_ubx_file) {
#if defined(CONFIG_GPS_LOG_USE_CJSON)
                item = cJSON_GetObjectItemCaseSensitive(gps, "UBXfile");
#else
                item = json_find_member(gps, "UBXfile");
#endif
            }
            if (item) {
                gps_cnf_set_item(i+CFG_GPS_ITEM_BASE, item, 0);
            }
        }
    }
    if (c_gps_cfg.file_date_time == 0 && GETBIT(log_config.log_file_bits, SD_TXT) == 0)
        SETBIT(log_config.log_file_bits, SD_TXT);  // because txt file is needed for generating new file count !!
#if defined(CONFIG_GPS_LOG_USE_CJSON)
    cJSON_Delete(root);
#else
    json_delete(root);
#endif
    return ret;
}

static const char * const cfg_values [] = {
",\"values\":[",
"{\"value\":",
",\"title\":\"",
",\"info\":\"Speed display units\",\"type\":\"int\"",
",\"info\":\"gps_rate in Hz\",\"type\":\"int\"",
",\"info\":\"choice for dynamic model: 'Portable' (max 310m/s), 'Sea' (max 25m/s) and 'Automotive' (max 100m/s) deviation medium, 'Pedestrian' (max 30m/s) deviation small\",\"type\":\"int\"",
",\"info\":\"log to ",
"\",\"type\":\"bool\"",
",\"info\":\"log nav sat msg to .ubx\", \"depends\":\"",
",\"info\":\"timezone: The local time difference in hours with UTC\",\"type\":\"float\",\"ext\":\"h\"",
",\"info\":\"type of filenaming, with MAC adress or datetime\",\"type\":\"int\"",
",\"info\":\"your preferred filename base\",\"type\":\"str\"",
",\"info\":\"Wether device can switch dynamic model automatically. When disabled make sure dynamic model is in your expected speed limits.",
};

static uint8_t add_from_list(strbf_t *lsb, const char * const *list, size_t len) {
    strbf_puts(lsb, cfg_values[0]);
    for(uint8_t i = 0, j = len; i < j; i++) {
        strbf_puts(lsb, cfg_values[1]);
        strbf_putn(lsb, i);
        strbf_puts(lsb, cfg_values[2]);
        strbf_puts(lsb, list[i]);
        strbf_puts(lsb, "\"}");
        if(i < j-1) strbf_putc(lsb, ',');
    }
    strbf_puts(lsb, "]");
    return 0;
}

uint8_t gps_cnf_get_item(uint8_t pos, strbf_t * lsb, uint8_t mode) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG,"[%s] pos: %hhu",__func__, pos);
#endif
    if (!lsb) return 254;
    if(pos < CFG_GPS_ITEM_BASE || pos >= CFG_GPS_ITEM_BASE + gps_user_cfg_item_count) {
        return 255;
    }
    const char * start = lsb->cur;
    if (mode)
        strbf_puts(lsb, "{\"name\":");
    strbf_puts(lsb, "\"");
    strbf_puts(lsb, config_gps_items[CFG_TO_BASE(pos)]);
    strbf_puts(lsb, "\"");
    if (mode)
        strbf_puts(lsb, ",\"value\"");
    strbf_putc(lsb, ':');
    if(gps_cfg_lock(timeout_max) == pdTRUE) {
        switch (pos) {
            case gps_cfg_file_date_time:
                strbf_putn(lsb, c_gps_cfg.file_date_time);
                if (mode) {
                    strbf_puts(lsb, cfg_values[10]);
                    add_from_list(lsb, file_date_time_items, lengthof(file_date_time_items));
                }
                break;
            case gps_cfg_timezone:
                strbf_putd(lsb, c_gps_cfg.timezone, 1, 0);
                if (mode) {
                    strbf_puts(lsb, cfg_values[9]);
                    add_from_list(lsb, timezone_items, lengthof(timezone_items));
                }                                        // 2575
                break;
            case gps_cfg_stat_screens: // choice for stats field when no speed, here stat_screen 1, 2 and 3 will be active
                strbf_putn(lsb, c_gps_cfg.stat_screens);
                if (mode) {
                    strbf_puts(lsb, ",\"info\":\"Stat_screens choice : activate / deactivate screens to show.\",\"type\":\"int\"");
                    strbf_puts(lsb, ",\"toggles\":[");
                    uint16_t j = 1;
                    for(uint8_t i= 0, k = gps_stat_screen_item_count; i < k; i++, j <<= 1) {
                        strbf_puts(lsb, "{\"pos\":");
                        strbf_putn(lsb, i);
                        strbf_puts(lsb, cfg_values[2]);
                        strbf_puts(lsb, config_stat_screen_items[i]);
                        strbf_puts(lsb, "\",\"value\":");
                        strbf_putn(lsb, j);
                        strbf_puts(lsb, "}");
                        if(i < k-1) strbf_putc(lsb, ',');
                    }
                    strbf_puts(lsb, "]");
                }
                break;
            case gps_cfg_speed_unit:  // speed units, 0 = m/s 1 = km/h, 2 = knots
                strbf_putn(lsb, c_gps_cfg.speed_unit);
                if (mode) {
                    strbf_puts(lsb, cfg_values[3]);
                    add_from_list(lsb, speed_units, lengthof(speed_units));
                }
                break;
            case gps_cfg_sample_rate:  // gps_rate in Hz, 1, 5 or 10Hz !!!
                strbf_putn(lsb, rtc_config.output_rate);
                if (mode) {
                    strbf_puts(lsb, cfg_values[4]);
                    strbf_puts(lsb, cfg_values[0]);
                    for (uint8_t i = 0, j = lengthof(sample_rates); i < j; i++) {
                        strbf_puts(lsb, cfg_values[1]);
                        strbf_putn(lsb, i==0 ? UBX_OUTPUT_1HZ : i==1 ? UBX_OUTPUT_2HZ : i==2 ? UBX_OUTPUT_5HZ : i==3 ? UBX_OUTPUT_10HZ : i==4 ? UBX_OUTPUT_16HZ : UBX_OUTPUT_20HZ);
                        strbf_puts(lsb, cfg_values[2]);
                        strbf_puts(lsb, sample_rates[i]);
                        strbf_puts(lsb, "\"}");
                        if (i < j - 1)
                            strbf_putc(lsb, ',');
                    }
                    strbf_puts(lsb, "]");
                    strbf_puts(lsb, ",\"ext\":\"Hz\"");
                }
                break;
            case gps_cfg_ubx_file: // preferred filename base
                strbf_puts(lsb, "\"");
                strbf_puts(lsb, c_gps_cfg.ubx_file);
                strbf_puts(lsb, "\"");
                if (mode) {
                    strbf_puts(lsb, cfg_values[11]); // description
                }
                break;
            case gps_cfg_gnss:
                strbf_putn(lsb, rtc_config.gnss);
                if (mode) {
                    strbf_puts(lsb, ",\"info\":\"");
                    if (gps && gps->ubx_device->rtc_conf->hw_type >= UBX_TYPE_M9) {
                        strbf_puts(lsb, "For M9 default 4 gnss: ");
                        strbf_puts(lsb, gnss_desc[0]);
                    }
                    else {
                        strbf_puts(lsb, "For M10 default 3 gnss: ");
                        strbf_puts(lsb, gnss_desc[2]);
                    }
                    strbf_puts(lsb, ", (G=GPS, E=GALILEO, B=BEIDOU, R=GLONASS)");
                    const char * a[] = {GSS_DESC_ITEM_LIST(STRINGIFY)};
                    strbf_puts(lsb, "\",\"type\":\"int\"");
                    strbf_puts(lsb, cfg_values[0]);
                    for (uint8_t i = 0, j = lengthof(gnss_desc_val); i < j; i++) {
                        if (gps && gps->ubx_device->rtc_conf->hw_type < UBX_TYPE_M9 && i == 0) continue;
                        strbf_puts(lsb, cfg_values[1]);
                        strbf_putn(lsb, gnss_desc_val[i]);
                        strbf_puts(lsb, cfg_values[2]);
                        strbf_puts(lsb, gnss_desc[i]);
                        strbf_puts(lsb, "\"}");
                        if(i < j-1) strbf_putc(lsb, ',');
                    }
                    strbf_puts(lsb, "]");
                }
                break;
            case gps_cfg_dynamic_model:
                strbf_putn(lsb, rtc_config.nav_mode);
                if (mode) {
                    strbf_puts(lsb, cfg_values[5]);
                    strbf_puts(lsb, cfg_values[0]);
                    for (uint8_t i = 0, j = lengthof(dynamic_models); i < j; i++) {
                        strbf_puts(lsb, cfg_values[1]);
                        strbf_putn(lsb, i == 0 ? i : i+2);
                        strbf_puts(lsb, cfg_values[2]);
                        strbf_puts(lsb, dynamic_models[i]);
                        strbf_puts(lsb, "\"}");
                        if (i < j - 1) strbf_putc(lsb, ',');
                    }
                    strbf_puts(lsb, "]");
                }
                break;
            // case gps_cfg_dynamic_model_auto: // )])||!strcmp(name, "logTXT")) {  // switchinf off .txt files
            //     strbf_putc(lsb, rtc_config.nav_mode_auto == 1 ? '1' : '0');
            //     if (mode) {
            //         strbf_puts(lsb, cfg_values[12]); // ",\"info\":\"log to"
            //         strbf_puts(lsb, cfg_values[7]); // ",\"type\":\"bool\"");
            //     }
            //     break;
            case gps_cfg_log_txt: // )])||!strcmp(name, "logTXT")) {  // switchinf off .txt files
                strbf_putc(lsb, GETBIT(log_config.log_file_bits, SD_TXT) == 1 ? '1' : '0');
                if (mode) {
                    strbf_puts(lsb, cfg_values[6]); // ",\"info\":\"log to"
                    strbf_puts(lsb, str_txt);
                    strbf_puts(lsb, cfg_values[7]); // ",\"type\":\"bool\"");
                }
                break;
            case gps_cfg_log_ubx: // )])||!strcmp(name, "logUBX")) {  // log to .ubx
                strbf_putc(lsb, GETBIT(log_config.log_file_bits, SD_UBX) == 0 ? '0' : '1');
                if (mode) {
                    strbf_puts(lsb, cfg_values[6]); // ",\"info\":\"log to"
                    strbf_puts(lsb, str_ubx);
                    strbf_puts(lsb, cfg_values[7]); // ",\"type\":\"bool\"");
                }
                break;
            case gps_cfg_log_ubx_nav_sat: // )])||!strcmp(name, "logUBX_nav_sat")) {  // log nav sat msg to .ubx
                strbf_putn(lsb, rtc_config.msgout_sat);
                if (mode) {
                    strbf_puts(lsb, cfg_values[8]); // ",\"info\":\"log nav sat msg to .ubx"
                    strbf_puts(lsb, config_gps_items[CFG_TO_BASE(gps_cfg_log_ubx)]); // depends on logUBX
                    strbf_puts(lsb, cfg_values[7]); // ",\"type\":\"bool\"");
                }
                break;
            case gps_cfg_log_sbp: // )])||!strcmp(name, "logSBP")) {  // log to .sbp
                strbf_putc(lsb, GETBIT(log_config.log_file_bits, SD_SBP) == 0 ? '0' : '1');
                if (mode) {
                    strbf_puts(lsb, cfg_values[6]); // ",\"info\":\"log to"
                    strbf_puts(lsb, str_sbp);
                    strbf_puts(lsb, cfg_values[7]); // ",\"type\":\"bool\"");
                }
                break;
            case gps_cfg_log_gpx: // )])||!strcmp(name, "logGPX")) {   // log to .gpx
                strbf_putc(lsb, GETBIT(log_config.log_file_bits, SD_GPX) == 0 ? '0' : '1');
                if (mode) {
                    strbf_puts(lsb, cfg_values[6]); // ",\"info\":\"log to"
                    strbf_puts(lsb, str_gpx);
                    strbf_puts(lsb, cfg_values[7]); // ",\"type\":\"bool\"");
                }
                break;
#ifdef GPS_LOG_ENABLE_GPY
            case gps_cfg_log_gpy: // )])||!strcmp(name, "logGPY")) {   // log to .gpy
                strbf_putc(lsb, GETBIT(log_config.log_file_bits, SD_GPY) == 0 ? '0' : '1');
                if (mode) {
                    strbf_puts(lsb, cfg_values[6]); // ",\"info\":\"log to"
                    strbf_puts(lsb, str_gpy);
                    strbf_puts(lsb, cfg_values[7]); // ",\"type\":\"bool\"");
                }
                break;
#endif
            default:
                pos = 253;
                lsb->cur = (char*)start;
                goto err;
                break;
        }
        if (mode)
            strbf_puts(lsb, "}");
    err:
        gps_cfg_unlock();
    }
    DLOG(TAG, "[%s] conf: %s len: %d", __func__, strbf_finish(lsb), lsb->cur - lsb->start);
    return pos;
}

char *gps_config_get(const char *name, struct strbf_s *lsb, uint8_t mode) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s] name:%s", __func__, name ? name : "-");
#endif
    if(!lsb) return NULL;
    uint8_t pos = gps_cfg_get_pos(name);
    if (pos == 255) {
        goto end;
    }
    gps_cnf_get_item(pos, lsb, mode);
    end:
    return strbf_finish(lsb);
}

char * gps_config_encode(strbf_t *sb, uint8_t mode, uint8_t mode2) {
    if(!sb) return NULL;
    if(mode==2) {
        strbf_putc(sb, '{');
    }
    if(mode) {
        strbf_puts(sb, "\"gps\":{");
    }
    for (uint8_t i = 0, j = gps_user_cfg_item_count; i < j; ++i) {
        if(gps_cnf_get_item(i+CFG_GPS_ITEM_BASE, sb, mode2) >= 253) {
            continue;
        }
        if(i < j-1) {
            strbf_putc(sb, ',');
        }
        strbf_putc(sb, '\n');
    }
    if(mode) {
        strbf_putc(sb, '}');
    } 
    if(mode==2) {
       strbf_putc(sb, '}');
    }
    return strbf_finish(sb);
}
