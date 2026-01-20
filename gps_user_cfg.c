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
#include "config.h"

static const char * TAG = "gps_user_cfg"; // for logging

// Temporary defines for old enum values during refactor
#define gps_cfg_sample_rate 997 // No longer used - moved to UBX config
#define gps_cfg_gnss 998 // No longer used
#define gps_cfg_dynamic_model 999 // No longer used
#define gps_user_cfg_item_count 0 // Disabled during refactor

// Optimized string constants to reduce memory usage
// static const char str_on[] = "on";
// static const char str_off[] = "off";
// static const char str_not_set[] = "not set";
// static const char str_ubx[] = ".ubx";
// static const char str_sbp[] = ".sbp";
// static const char str_txt[] = ".txt";
// static const char str_gpx[] = ".gpx";
// static const char str_gpy[] = ".gpy";

// Pre-calculated timeout for common semaphore operations
// static const TickType_t timeout_max = portMAX_DELAY;

// Sample rate conversion: index 0-5 -> rate 1,2,5,10,16,20
// static const uint8_t sample_rate_values[] = {1, 2, 5, 10, 16, 20};

// Sample rate conversion: index 0-4 -> Hz values 1,5,10,15,20
// Sample rate conversion: index 0-5 -> rate 1,2,5,10,16,20
// static const uint8_t sample_rates_hz[] = {1, 2, 5, 10, 16, 20};

static SemaphoreHandle_t c_sem_lock = 0;
// GPS config is now part of unified RTC config
// Access via: g_rtc_config.gps
extern struct gps_context_s *gps;
extern gps_log_file_config_t log_config;

// OBSOLETE: This function is disabled during migration to config_manager system
// Config loading is now handled by config_manager_gps and config_manager_ubx
#if 0
static void gps_config_load_from_nvs(void) {
    FUNC_ENTRY(TAG);

    // Load GPS config items from sconfig into unified_config
    const struct sconfig_item *item;
    
    // GPS settings
    item = get_sconfig_item(SCONFIG_GROUP_GPS, gps_cfg_file_date_time);
    if (item) g_rtc_config.gps.file_date_time = sconfig_get_u8(item);

    item = get_sconfig_item(SCONFIG_GROUP_GPS, gps_cfg_timezone);
    if (item) {
        g_rtc_config.gps.timezone = (float)sconfig_get_i8(item);
    }

    item = get_sconfig_item(SCONFIG_GROUP_GPS, gps_cfg_stat_screens);
    if (item) g_rtc_config.gps.stat_screens = sconfig_get_u16(item);

    item = get_sconfig_item(SCONFIG_GROUP_GPS, gps_cfg_speed_unit);
    if (item) g_rtc_config.gps.speed_unit = sconfig_get_u8(item);

    item = get_sconfig_item(SCONFIG_GROUP_GPS, gps_cfg_ubx_file);
    if (item) {
        char *str = NULL;
        if (sconfig_get_str_blob_alloc(item, (void**)&str) == 0 && str) {
            strncpy(g_rtc_config.gps.ubx_file, str, sizeof(g_rtc_config.gps.ubx_file) - 1);
            free(str);
        }
    }

    // GPS logging enables
    item = get_sconfig_item(SCONFIG_GROUP_GPS, gps_cfg_log_txt);
    if (item) g_rtc_config.gps.log_txt = sconfig_get_bool(item);
    
    item = get_sconfig_item(SCONFIG_GROUP_GPS, gps_cfg_log_ubx);
    if (item) g_rtc_config.gps.log_ubx = sconfig_get_bool(item);
    
    item = get_sconfig_item(SCONFIG_GROUP_GPS, gps_cfg_log_sbp);
    if (item) g_rtc_config.gps.log_sbp = sconfig_get_bool(item);
    
    item = get_sconfig_item(SCONFIG_GROUP_GPS, gps_cfg_log_gpx);
    if (item) g_rtc_config.gps.log_gpx = sconfig_get_bool(item);

#if defined(CONFIG_GPS_LOG_GPY)
    item = get_sconfig_item(SCONFIG_GROUP_GPS, gps_cfg_log_gpy);
    if (item) g_rtc_config.gps.log_gpy = sconfig_get_bool(item);
#endif

    // Load UBX config items
    item = get_sconfig_item(SCONFIG_GROUP_GPS, gps_cfg_sample_rate);
    DLOG(TAG, "Looking for GPS sample_rate at group %d, index %d", SCONFIG_GROUP_GPS, gps_cfg_sample_rate);
    if (item) {
        uint8_t rate_index = sconfig_get_u8(item);
        DLOG(TAG, "Found sample_rate config item, raw value: %d", rate_index);
        if (rate_index < sizeof(sample_rates_hz) / sizeof(sample_rates_hz[0])) {
            g_rtc_config.ubx.output_rate = sample_rates_hz[rate_index];
            DLOG(TAG, "Loaded sample_rate index: %d -> Hz: %d", rate_index, g_rtc_config.ubx.output_rate);
        } else {
            ELOG(TAG, "Invalid sample_rate index: %d", rate_index);
            g_rtc_config.ubx.output_rate = 5;  // fallback to 5Hz
        }
    } else {
        ELOG(TAG, "Failed to get sample_rate config item");
        g_rtc_config.ubx.output_rate = 5;  // fallback
    }

    item = get_sconfig_item(SCONFIG_GROUP_GPS, gps_cfg_gnss);
    if (item) g_rtc_config.ubx.gnss = sconfig_get_u8(item);

    item = get_sconfig_item(SCONFIG_GROUP_GPS, gps_cfg_dynamic_model);
    if (item) g_rtc_config.ubx.nav_mode = sconfig_get_u8(item);

    item = get_sconfig_item(SCONFIG_GROUP_GPS, gps_cfg_log_ubx_nav_sat);
    if (item) {
        g_rtc_config.ubx.msgout_sat = sconfig_get_u8(item);
        g_rtc_config.gps.log_ubx_nav_sat = sconfig_get_bool(item);
    }
    
    // Load log_config items (bit fields) - also store in gps config
    item = get_sconfig_item(SCONFIG_GROUP_GPS, gps_cfg_log_txt);
    if (item) {
        bool enabled = sconfig_get_bool(item);
        g_rtc_config.gps.log_enables.bits.log_txt = enabled;
        log_config.log_file_enables.bits.log_txt = enabled;
    }
    
    item = get_sconfig_item(SCONFIG_GROUP_GPS, gps_cfg_log_ubx);
    if (item) {
        bool enabled = sconfig_get_bool(item);
        g_rtc_config.gps.log_enables.bits.log_ubx = enabled;
        log_config.log_file_enables.bits.log_ubx = enabled;
    }
    
    item = get_sconfig_item(SCONFIG_GROUP_GPS, gps_cfg_log_sbp);
    if (item) {
        bool enabled = sconfig_get_bool(item);
        g_rtc_config.gps.log_enables.bits.log_sbp = enabled;
        log_config.log_file_enables.bits.log_sbp = enabled;
    }
    
    item = get_sconfig_item(SCONFIG_GROUP_GPS, gps_cfg_log_gpx);
    if (item) {
        bool enabled = sconfig_get_bool(item);
        g_rtc_config.gps.log_enables.bits.log_gpx = enabled;
        log_config.log_file_enables.bits.log_gpx = enabled;
    }
    
#ifdef GPS_LOG_ENABLE_GPY
    item = get_sconfig_item(SCONFIG_GROUP_GPS, gps_cfg_log_gpy);
    if (item) {
        bool enabled = sconfig_get_bool(item);
        g_rtc_config.gps.log_enables.bits.log_gpy = enabled;
        log_config.log_file_enables.bits.log_gpy = enabled;
    }
#endif
    
    // Apply fixups
    DLOG(TAG, "Before fix_values: rtc_config.output_rate = %d", g_rtc_config.ubx.output_rate);
    gps_config_fix_values();
    DLOG(TAG, "After fix_values: rtc_config.output_rate = %d", g_rtc_config.ubx.output_rate);
}
#endif // Disabled gps_config_load_from_nvs

void gps_user_cfg_init(void) {
    if(!c_sem_lock) {
        c_sem_lock = xSemaphoreCreateMutex();
    }
    if(!c_sem_lock) {
        ELOG(TAG, "Failed to create semaphore");
    }
    // Load config from NVS into runtime structures
    // gps_config_load_from_nvs(); // Disabled - unified config system handles this now
}

void gps_user_cfg_deinit(void) {
    if(c_sem_lock) {
        vSemaphoreDelete(c_sem_lock);
        c_sem_lock = 0;
    }
}

static void set_gps_time_out_msg(void) {
    FUNC_ENTRY(TAG);
    gps->time_out_gps_msg = HZ_TO_MS(g_rtc_config.ubx.output_rate) + 75;  // max time out = 175 ms
}

// New simplified set_gps_cfg_item using config_manager cycle functions
// This cycles the config value and handles UBX hardware interaction if needed

#if defined(LOGGER_CONFIG_USE_OLD)

uint8_t gps_cfg_get_pos(const char *str) {
    FUNC_ENTRY_ARGS(TAG, "str: %s", str ? str : "-");
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
    FUNC_ENTRY_ARGS(TAG, "pos: %hhu", pos);
    if (!el) {
        return 254;
    }
    if(pos < CFG_GPS_ITEM_BASE || pos >= CFG_GPS_ITEM_BASE + gps_user_cfg_item_count) {
        return 255;
    }
    uint8_t changed = 255, ret = 0;
    size_t index = pos;
#if defined(CONFIG_GPS_LOG_USE_CJSON)
    cJSON *item = (cJSON *)el;
#else
    JsonNode *item = (JsonNode *)el;
#endif
    if(config_lock(timeout_max) == pdTRUE) {
        switch (pos) {
            case gps_cfg_file_date_time:
                ret = set_hhu(item, (uint8_t*)&g_rtc_config.gps.file_date_time, 0);
                if(!ret) {
                    char val_str[16];
                    snprintf(val_str, sizeof(val_str), "%u", g_rtc_config.gps.file_date_time);
                    config_gps_set_item(index, val_str);
                    changed = gps_cfg_file_date_time;
                }
                break;
            case gps_cfg_timezone:  // choice for timedifference in hours with UTC, for Belgium 1, 2, 3 for estonia (summer time)
                ret = set_f(item, &g_rtc_config.gps.timezone, 0);
                if(!ret) {
                    char val_str[16];
                    snprintf(val_str, sizeof(val_str), "%.0f", g_rtc_config.gps.timezone);
                    config_gps_set_item(index, val_str);
                    changed = gps_cfg_timezone;
                }
                break;
            case gps_cfg_stat_screens: // choice for stats field when no speed, here stat_screen 1, 2 and 3 will be active
                ret = set_u(item, &g_rtc_config.gps.stat_screens, 0);
                if(!ret) {
                    char val_str[16];
                    snprintf(val_str, sizeof(val_str), "%u", g_rtc_config.gps.stat_screens);
                    config_gps_set_item(index, val_str);
                    changed = gps_cfg_stat_screens;
                }
                break;
            case gps_cfg_speed_unit:  // conversion m/s to km/h, for knots use 1.944
                ret = set_hhu(item, (uint8_t*)&g_rtc_config.gps.speed_unit, 0);
                if(!ret) {
                    char val_str[16];
                    snprintf(val_str, sizeof(val_str), "%u", g_rtc_config.gps.speed_unit);
                    config_gps_set_item(index, val_str);
                    changed = gps_cfg_speed_unit;
                }
                break;
            case gps_cfg_sample_rate:  // gps_rate in Hz, 1, 2, 5 or 10Hz !!!
                ret = set_hhu(item, (uint8_t*)&rtc_config.output_rate, 0);
                if(!ret) {
                    char val_str[16];
                    snprintf(val_str, sizeof(val_str), "%u", rtc_config.output_rate);
                    config_gps_set_item(index, val_str);
                    changed = gps_cfg_sample_rate;
                    set_gps_time_out_msg();
                }
                break;
            case gps_cfg_gnss:  // default setting 2 GNSS, GPS & GLONAS
                ret = set_hhu(item, &rtc_config.gnss, 0);
                if(!ret) {
                    config_gps_set_item(index, &rtc_config.gnss);
                    changed = gps_cfg_gnss;
                }
                break;
            // case gps_cfg_dynamic_model_auto: /// wether device can switch mode automatically
            //     ret = set_hhu(item, (uint8_t*)&rtc_config.nav_mode_auto, 0);
            //     if(!ret) changed = gps_cfg_dynamic_model_auto;
            //     break;
            case gps_cfg_dynamic_model:  // sea, portable, automotive, pedestrian
                ret = set_hhu(item, (uint8_t*)&rtc_config.nav_mode, 0);
                if(!ret) {
                    config_gps_set_item(index, &rtc_config.nav_mode);
                    changed = gps_cfg_dynamic_model;
                }
                break;
            case gps_cfg_log_txt: // || !strcmp(var, "logTXT")) {  // switchinf off .txt files
                ret = set_bit(item, &log_config.log_file_enables.value, SD_TXT, 0);
                if(!ret) {
                    bool val = (log_config.log_file_enables.value & (1 << SD_TXT)) ? true : false;
                    config_gps_set_item(index, &val);
                    changed = gps_cfg_log_txt;
                }
                goto test_if_log_set; // if no log file selected, set SBP as default
            case gps_cfg_log_ubx: // || !strcmp(var, "logUBX")) {  // log to .ubx
                ret = set_bit(item, &log_config.log_file_enables.value, SD_UBX, 0);
                if(!ret) {
                    bool val = (log_config.log_file_enables.value & (1 << SD_UBX)) ? true : false;
                    config_gps_set_item(index, &val);
                    changed = gps_cfg_log_ubx;
                }
                goto test_if_log_set; // if no log file selected, set SBP as default
            case gps_cfg_log_ubx_nav_sat: // || !strcmp(var, "logUBX_nav_sat")) {  // log nav sat msg to .ubx
                ret = set_hhu(item, (uint8_t*)&rtc_config.msgout_sat, 0);
                if(!ret) {
                    config_gps_set_item(index, &rtc_config.msgout_sat);
                    changed = gps_cfg_log_ubx_nav_sat;
                }
                goto test_if_log_set; // if no log file selected, set SBP as default
            case gps_cfg_log_sbp: // || !strcmp(var, "logSBP")) {  // log to .sbp
                ret = set_bit(item, &log_config.log_file_enables.value, SD_SBP, 0);
                if(!ret) {
                    bool val = (log_config.log_file_enables.value & (1 << SD_SBP)) ? true : false;
                    config_gps_set_item(index, &val);
                    changed = gps_cfg_log_sbp;
                }
                test_if_log_set:
                if(!gps_log_file_bits_check(&log_config.log_file_enables))
                    log_config.log_file_enables.bits.log_sbp = 1; // set SBP if none selected
                break;
            case gps_cfg_log_gpx: //  || !strcmp(var, "logGPX")) {
                ret = set_bit(item, &log_config.log_file_enables.value, SD_GPX, 0);
                if(!ret) {
                    bool val = (log_config.log_file_enables.value & (1 << SD_GPX)) ? true : false;
                    config_gps_set_item(index, &val);
                    changed = gps_cfg_log_gpx;
                }
                goto test_if_log_set; // if no log file selected, set SBP as default
#ifdef GPS_LOG_ENABLE_GPY
            case gps_cfg_log_gpy: // || !strcmp(var, "logGPY")) {
                ret = set_bit(item, &log_config.log_file_enables.value, SD_GPY, 0);
                if(!ret) {
                    bool val = (log_config.log_file_enables.value & (1 << SD_GPY)) ? true : false;
                    config_gps_set_item(index, &val);
                    changed = gps_cfg_log_gpy;
                }
                goto test_if_log_set; // if no log file selected, set SBP as default
#endif
            case gps_cfg_ubx_file: // ]) || !strcmp(var, "UBXfile")) {
                ret = set_c(item, &g_rtc_config.gps.ubx_file[0], 0);
                if(!ret) {
                    config_gps_set_item(index, g_rtc_config.gps.ubx_file);
                    changed = gps_cfg_ubx_file;
                }
                break;
            default:
                changed = 253;
                break;
        }
        config_unlock();
    }
    if(changed < 253){
        esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_CFG_CHANGED, &changed, sizeof(changed), timeout_max);
    }
    return changed;
}

int gps_config_set(const char *str, void *root, uint8_t force) {
    FUNC_ENTRY_ARGS(TAG,"name: %s", str ? str : "-");
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
    FUNC_ENTRY(TAG);
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
    if (g_rtc_config.gps.file_date_time == 0 && GETBIT(log_config.log_file_bits, SD_TXT) == 0)
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
    FUNC_ENTRY_ARGS(TAG,"pos: %hhu", pos);
    if (!lsb) return 254;
    if(pos < CFG_GPS_ITEM_BASE || pos >= CFG_GPS_ITEM_BASE + gps_user_cfg_item_count) {
        return 255;
    }
    const char * start = lsb->cur;
    if (mode)
        strbf_puts(lsb, "{\"name\":");
    strbf_puts(lsb, "\"");
    strbf_puts(lsb, config_gps_items[pos]);
    strbf_puts(lsb, "\"");
    if (mode)
        strbf_puts(lsb, ",\"value\"");
    strbf_putc(lsb, ':');
    if(config_lock(timeout_max) == pdTRUE) {
        switch (pos) {
            case gps_cfg_file_date_time:
                strbf_putn(lsb, g_rtc_config.gps.file_date_time);
                if (mode) {
                    strbf_puts(lsb, cfg_values[10]);
                    add_from_list(lsb, file_date_time_items, file_date_time_items_count);
                }
                break;
            case gps_cfg_timezone:
                strbf_putd(lsb, g_rtc_config.gps.timezone, 1, 0);
                if (mode) {
                    strbf_puts(lsb, cfg_values[9]);
                    add_from_list(lsb, timezone_items, timezone_items_count);
                }                                        // 2575
                break;
            case gps_cfg_stat_screens: // choice for stats field when no speed, here stat_screen 1, 2 and 3 will be active
                strbf_putn(lsb, g_rtc_config.gps.stat_screens);
                if (mode) {
                    strbf_puts(lsb, ",\"info\":\"Stat_screens choice : activate / deactivate screens to show.\",\"type\":\"int\"");
                    strbf_puts(lsb, ",\"toggles\":[");
                    uint16_t j = 1;
                    for(uint8_t i= 0, k = gps_stat_screen_item_count; i < k; i++, j <<= 1) {
                        strbf_puts(lsb, "{\"pos\":");
                        strbf_putn(lsb, i);
                        strbf_puts(lsb, cfg_values[2]);
                        strbf_puts(lsb, gps_stat_screen_items[i]);
                        strbf_puts(lsb, "\",\"value\":");
                        strbf_putn(lsb, j);
                        strbf_puts(lsb, "}");
                        if(i < k-1) strbf_putc(lsb, ',');
                    }
                    strbf_puts(lsb, "]");
                }
                break;
            case gps_cfg_speed_unit:  // speed units, 0 = m/s 1 = km/h, 2 = knots
                strbf_putn(lsb, g_rtc_config.gps.speed_unit);
                if (mode) {
                    strbf_puts(lsb, cfg_values[3]);
                    add_from_list(lsb, speed_units, speed_units_items_count);
                }
                break;
            case gps_cfg_sample_rate:  // gps_rate in Hz, 1, 5 or 10Hz !!!
                strbf_putn(lsb, rtc_config.output_rate);
                if (mode) {
                    strbf_puts(lsb, cfg_values[4]);
                    strbf_puts(lsb, cfg_values[0]);
                    for (uint8_t i = 0, j = sample_rates_items_count; i < j; i++) {
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
                strbf_puts(lsb, g_rtc_config.gps.ubx_file);
                strbf_puts(lsb, "\"");
                if (mode) {
                    strbf_puts(lsb, cfg_values[11]); // description
                }
                break;
            case gps_cfg_gnss:
                strbf_putn(lsb, rtc_config.gnss);
                if (mode) {
                    strbf_puts(lsb, ",\"info\":\"");
                    if (gps && g_rtc_config.ubx.hw_type >= UBX_TYPE_M9) {
                        strbf_puts(lsb, "For M9 default 4 gnss: ");
                        strbf_puts(lsb, gnss_desc[0]);
                    }
                    else {
                        strbf_puts(lsb, "For M10 default 3 gnss: ");
                        strbf_puts(lsb, gnss_desc[2]);
                    }
                    strbf_puts(lsb, ", (G=GPS, E=GALILEO, B=BEIDOU, R=GLONASS)");
                    const char * a[] = {GNSS_DESC_ITEM_LIST(STRINGIFY)};
                    strbf_puts(lsb, "\",\"type\":\"int\"");
                    strbf_puts(lsb, cfg_values[0]);
                    for (uint8_t i = 0, j = gnss_desc_items_count; i < j; i++) {
                        if (gps && g_rtc_config.ubx.hw_type < UBX_TYPE_M9 && i == 0) continue;
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
                    for (uint8_t i = 0, j = dynamic_models_items_count; i < j; i++) {
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
                    strbf_puts(lsb, config_gps_items[gps_cfg_log_ubx]); // depends on logUBX
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
        config_unlock();
    }
    DLOG(TAG, "[%s] conf: %s len: %d", __func__, strbf_finish(lsb), lsb->cur - lsb->start);
    return pos;
}

char *gps_config_get(const char *name, struct strbf_s *lsb, uint8_t mode) {
    FUNC_ENTRY_ARGS(TAG, "name:%s", name ? name : "-");
    if(!lsb) return NULL;
    // GPS config now handled by config_manager
    if (config_manager_get_item_by_name(name, lsb) >= 253) {
        return NULL;
    }
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
    size_t gps_size = config_get_group_size(SCONFIG_GROUP_GPS);
    for (size_t i = 0; i < gps_size; ++i) {
        if(config_manager_get_item_json(SCONFIG_GROUP_GPS, i, sb) >= 253) {
            continue;
        }
        if(i < gps_size-1) {
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

#endif