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

#define CFG_TO_BASE(l) (l-CFG_GPS_ITEM_BASE)
#define SPEED_UNIT_ITEM_LIST(l) l(m/s) l(km/h) l(knots)
#define SAMPLE_RATE_ITEM_LIST(l) l(1 Hz) l(5 Hz) l(10 Hz) l(16 Hz) l(20 Hz)
#define DYNAMIC_MODEL_ITEM_LIST(l) l(Portable) l(Sea) l(Automotive)
#define GSS_DESC_ITEM_LIST(l) l(G + E + B + R) l(G + B + R) l(G + E + R) l(G + E + B) l(G + R) l(G + B) l(G + E)
#define GNSS_DESC_VAL_LIST(l) l(111) l(107) l(103) l(47) l(99) l(43) l(39)
#define TIMEZONE_ITEM_LIST(l) l(UTC) l(UTC+1) l(UTC+2) l(UTC+3)
#define FILE_DATE_TIME_ITEM_LIST(l) l(name_MAC_index) l(name_date_time) l(date_time_name)
#define L(x) x,
static const char * const config_gps_items[] = { CFG_GPS_USER_CFG_ITEM_LIST(STRINGIFY_V) };
const size_t gps_user_cfg_item_count = sizeof(config_gps_items) / sizeof(config_gps_items[0]);
const char * const speed_units[] = {SPEED_UNIT_ITEM_LIST(STRINGIFY)};
static const char * const sample_rates[] = {SAMPLE_RATE_ITEM_LIST(STRINGIFY)};
static const char * const dynamic_models[] = {DYNAMIC_MODEL_ITEM_LIST(STRINGIFY)};
static const char * const gnss_desc[] = {GSS_DESC_ITEM_LIST(STRINGIFY)};
static const char * const not_set = "not set";
static const uint8_t gnss_desc_val[] = {GNSS_DESC_VAL_LIST(L)};
static const char * const timezone[] = {TIMEZONE_ITEM_LIST(STRINGIFY)};
static const char * const file_date_time_items[] = {FILE_DATE_TIME_ITEM_LIST(STRINGIFY)};

static SemaphoreHandle_t c_sem_lock = 0;
// global unit for gps user config.
RTC_DATA_ATTR struct gps_user_cfg_s c_gps_cfg = GPS_USER_CFG_DEFAULTS();
extern struct gps_context_s *gps;
extern struct ubx_rtc_config_s rtc_config;
extern gps_log_file_config_t log_config;

void gps_user_cfg_init(void) {
    if(!c_sem_lock) {
        c_sem_lock = xSemaphoreCreateMutex();
    }
    if(!c_sem_lock) {
        ESP_LOGE(TAG, "Failed to create semaphore");
    }
    // set_speed_calibration_unit();
}

void gps_user_dfg_deinit(void) {
    if(c_sem_lock) {
        vSemaphoreDelete(c_sem_lock);
        c_sem_lock = 0;
    }
}

static bool gps_cfg_lock(int timeout) {
    if (!c_sem_lock) return false;
    const TickType_t timeout_ticks = (timeout == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout);
    return  xSemaphoreTake(c_sem_lock, timeout_ticks) == pdTRUE;
}

static void gps_cfg_unlock() {
    if (c_sem_lock) {
        xSemaphoreGive(c_sem_lock);
    }
}

struct m_config_item_s * get_gps_cfg_item(int num, struct m_config_item_s *item) {
    ILOG(TAG, "[%s] num:%d", __func__, num);
    if(!item) return 0;
    if(num < CFG_GPS_ITEM_BASE || num > CFG_GPS_ITEM_BASE + gps_user_cfg_item_count) num = CFG_GPS_ITEM_BASE;
    item->name = config_gps_items[CFG_TO_BASE(num)];
    item->pos = num;
    if(gps_cfg_lock(portMAX_DELAY) == pdTRUE) {
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
                    item->desc = not_set;
                }
                break;
            case gps_cfg_sample_rate:
                item->value = rtc_config.output_rate;
                if(rtc_config.output_rate == 1) {
                    item->desc = sample_rates[0];
                }
                else if(rtc_config.output_rate == 16) {
                    item->desc = sample_rates[3];
                }
                else if(rtc_config.output_rate%5 == 0 && rtc_config.output_rate <= 20) {
                    item->desc = sample_rates[rtc_config.output_rate/5];
                }
                else {
                    item->desc = not_set;
                }
                break;
            case gps_cfg_timezone:
                item->value = c_gps_cfg.timezone;
                for(int i = 0; i < sizeof(timezone) / sizeof(timezone[0]); i++) {
                    if(c_gps_cfg.timezone == i) {
                        item->desc = timezone[i];
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
                item->desc = item->value ? "on" : "off";
                break;
            case gps_cfg_log_ubx:
                item->value = GETBIT(log_config.log_file_bits, SD_UBX) ? 1 : 0;
                item->desc = item->value ? "on" : "off";
                break;
            case gps_cfg_log_sbp:
                item->value = GETBIT(log_config.log_file_bits, SD_SBP) ? 1 : 0;
                item->desc = item->value ? "on" : "off";
                break;
            case gps_cfg_log_gpx:
                item->value = GETBIT(log_config.log_file_bits, SD_GPX) ? 1 : 0;
                item->desc = item->value ? "on" : "off";
                break;
#ifdef GPS_LOG_ENABLE_GPY
            case gps_cfg_log_gpy:
                item->value = GETBIT(log_config.log_file_bits, SD_GPY) ? 1 : 0;
                item->desc = item->value ? "on" : "off";
                break;
#endif
            case gps_cfg_log_ubx_nav_sat:
                item->value = rtc_config.msgout_sat ? 1 : 0;
                item->desc = rtc_config.msgout_sat ? "on" : "off";
                break;
            case gps_cfg_dynamic_model:
                item->value = rtc_config.nav_mode;
                if(item->value < 3) {
                    item->desc = dynamic_models[item->value];
                }
                else {
                    item->desc = "portable";
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
                item->desc = not_set;
                break;
        }
        gps_cfg_unlock();
    }
    esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_CFG_GET, &item, sizeof(item), portMAX_DELAY);
    return item;
}

static void set_speed_calibration_unit(void) {
    c_gps_cfg.speed_calibration = c_gps_cfg.speed_unit == 1 ? 0.0036 : c_gps_cfg.speed_unit == 2 ? 0.00194384449 : 0.001;
}

int set_gps_cfg_item(int num) {
    ILOG(TAG, "[%s] num:%d", __func__, num);
    if(num < CFG_GPS_ITEM_BASE || num > CFG_GPS_ITEM_BASE + gps_user_cfg_item_count) return 255;
    const char *name = config_gps_items[CFG_TO_BASE(num)];
    if(gps_cfg_lock(portMAX_DELAY) == pdTRUE){
        switch(num) {
            case gps_cfg_gnss:
                if(rtc_config.gnss == 111) rtc_config.gnss = 107;
                else if(rtc_config.gnss == 107) rtc_config.gnss = 103;
                else if(rtc_config.gnss == 103) rtc_config.gnss = 47;
                else if(rtc_config.gnss == 47) rtc_config.gnss = 99;
                else if(rtc_config.gnss == 99) rtc_config.gnss = 43;
                else if(rtc_config.gnss == 43) rtc_config.gnss = 39;
                else if(rtc_config.gnss == 39) rtc_config.gnss = 111;
                else rtc_config.gnss = 111;
                break;
            case gps_cfg_sample_rate:
                if(rtc_config.output_rate == 5) rtc_config.output_rate = 1;
                else if(rtc_config.output_rate == 10) rtc_config.output_rate = 5;
                else if(rtc_config.output_rate == 16) rtc_config.output_rate = 10;
                else if(rtc_config.output_rate == 20) rtc_config.output_rate = 16;
                else rtc_config.output_rate = 20;
                break;
            case gps_cfg_timezone:
                if(c_gps_cfg.timezone == 1) c_gps_cfg.timezone = 2;
                else if(c_gps_cfg.timezone == 2) c_gps_cfg.timezone = 3;
                else if(c_gps_cfg.timezone == 3) c_gps_cfg.timezone = 1;
                else c_gps_cfg.timezone = 1;
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
            case gps_cfg_dynamic_model:
                if(rtc_config.nav_mode == 0) rtc_config.nav_mode = 2;
                else if(rtc_config.nav_mode == 2) rtc_config.nav_mode = 1;
                else rtc_config.nav_mode = 0;
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
        if(num == gps_cfg_speed_unit) {
            set_speed_calibration_unit();
        }
        else if(num == gps_cfg_gnss || num == gps_cfg_sample_rate || num == gps_cfg_log_ubx_nav_sat || num == gps_cfg_dynamic_model) {
            if(gps && gps->ubx_restart_requested == 0 && gps->ubx_device->config_ok)
                gps->ubx_restart_requested = 1;
        }
        gps_cfg_unlock();
    }
    esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_CFG_SET, &num, sizeof(num), portMAX_DELAY);
    return num;
}

uint8_t gps_cfg_get_pos(const char *str) {
    ILOG(TAG, "[%s] str: %s", __func__, str ? str : "-");
    if(!str) {
        return 254;
    }
    for (uint8_t i = 0, j = gps_user_cfg_item_count; i < j; i++) {
        if (!strcmp(str, config_gps_items[i])) {
            return i + CFG_GPS_ITEM_BASE;
        }
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
    ILOG(TAG, "[%s] pos: %hhu", __func__, pos);
    if (!el) {
        return 254;
    }
    if(pos < CFG_GPS_ITEM_BASE || pos >= CFG_GPS_ITEM_BASE + gps_user_cfg_item_count) {
        return 255;
    }
    uint8_t changed = 255, ret = 0;
#if defined(CONFIG_GPS_LOG_USE_CJSON)
    cJSON *value = (cJSON *)el;
#else
    JsonNode *value = (JsonNode *)el;
#endif
    if(gps_cfg_lock(portMAX_DELAY) == pdTRUE) {
        switch (pos) {
            case gps_cfg_file_date_time:
                ret = set_hhu(value, &c_gps_cfg.file_date_time, 0);
                if(!ret) changed = gps_cfg_file_date_time;
                break;
            case gps_cfg_timezone:  // choice for timedifference in hours with UTC, for Belgium 1 or 2 (summertime)
                ret = set_f(value, &c_gps_cfg.timezone, 0);
                if(!ret) changed = gps_cfg_timezone;
                break;
            case gps_cfg_speed_unit:  // conversion m/s to km/h, for knots use 1.944
                ret = set_hhu(value, &c_gps_cfg.speed_unit, 0);
                if(!ret) changed = gps_cfg_speed_unit;
                break;
            case gps_cfg_sample_rate:  // gps_rate in Hz, 1, 5 or 10Hz !!!
                ret = set_hhu(value, (uint8_t*)&rtc_config.output_rate, 0);
                if(!ret) changed = gps_cfg_sample_rate;
                break;
            case gps_cfg_gnss:  // default setting 2 GNSS, GPS & GLONAS
                ret = set_hhu(value, &rtc_config.gnss, 0);
                if(!ret) changed = gps_cfg_gnss;
                break;
            case gps_cfg_dynamic_model:  // choice for dynamic model "Sea",if 0 model "portable" is used !!
                ret = set_hhu(value, (uint8_t*)&rtc_config.nav_mode, 0);
                if(!ret) changed = gps_cfg_dynamic_model;
                break;
            case gps_cfg_log_txt: // || !strcmp(var, "logTXT")) {  // switchinf off .txt files
                ret = set_bit(value, &log_config.log_file_bits, SD_TXT, 0);
                if(!ret) changed = gps_cfg_log_txt;
                break;
            case gps_cfg_log_ubx: // || !strcmp(var, "logUBX")) {  // log to .ubx
                ret = set_bit(value, &log_config.log_file_bits, SD_UBX, 0);
                if(!ret) changed = gps_cfg_log_ubx;
                break;
            case gps_cfg_log_ubx_nav_sat: // || !strcmp(var, "logUBX_nav_sat")) {  // log nav sat msg to .ubx
                ret = set_hhu(value, (uint8_t*)&rtc_config.msgout_sat, 0);
                if(!ret) changed = gps_cfg_log_ubx_nav_sat;
                break;
            case gps_cfg_log_sbp: // || !strcmp(var, "logSBP")) {  // log to .sbp
                ret = set_bit(value, &log_config.log_file_bits, SD_SBP, 0);
                if(!ret) changed = gps_cfg_log_sbp;
                break;
            case gps_cfg_log_gpx: //  || !strcmp(var, "logGPX")) {
                ret = set_bit(value, &log_config.log_file_bits, SD_GPX, 0);
                if(!ret) changed = gps_cfg_log_gpx;
                break;
#ifdef GPS_LOG_ENABLE_GPY
            case gps_cfg_log_gpy: // || !strcmp(var, "logGPY")) {
                ret = set_bit(value, &log_config.log_file_bits, SD_GPY, 0);
                if(!ret) changed = gps_cfg_log_gpy;
                break;
#endif
            case gps_cfg_ubx_file: // ]) || !strcmp(var, "UBXfile")) {
                ret = set_c(value, &c_gps_cfg.ubx_file[0], 0);
                if(!ret) changed = gps_cfg_ubx_file;
                break;
            default:
                changed = 253;
                break;
        }
        gps_cfg_unlock();
    }
    if(changed < 253)
        esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_CFG_CHANGED, &changed, sizeof(changed), portMAX_DELAY);
    return changed;
}

int gps_config_set(const char *str, void *root, uint8_t force) {
    ILOG(TAG,"[%s] name: %s",__func__, str ? str : "-");
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
    ILOG(TAG,"[%s]",__func__);
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
            if(i+CFG_GPS_ITEM_BASE == gps_cfg_log_gpx) {
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
",\"info\":\"choice for dynamic model 'Sea', if 0 model 'Portable' is used !!\",\"type\":\"int\"",
",\"info\":\"log to ",
"\",\"type\":\"bool\"",
",\"info\":\"log nav sat msg to .ubx",
",\"info\":\"timezone: The local time difference in hours with UTC\",\"type\":\"float\",\"ext\":\"h\"",
",\"info\":\"type of filenaming, with MAC adress or datetime\",\"type\":\"int\"",
",\"info\":\"your preferred filename base\",\"type\":\"str\"",
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
    ILOG(TAG,"[%s] pos: %hhu",__func__, pos);
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
    if(gps_cfg_lock(portMAX_DELAY) == pdTRUE) {
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
                    add_from_list(lsb, timezone, lengthof(timezone));
                }                                        // 2575
                break;
            case gps_cfg_speed_unit:  // speed units, 0 = m/s 1 = km/h, 2 = knots
                strbf_putn(lsb, c_gps_cfg.speed_unit);
                if (mode) {
                    strbf_puts(lsb, cfg_values[3]); // ",\"info\":\"Speed display units\",\"type\":\"int\""
                    add_from_list(lsb, speed_units, lengthof(speed_units));
                }
                break;
            case gps_cfg_sample_rate:  // gps_rate in Hz, 1, 5 or 10Hz !!!
                strbf_putn(lsb, rtc_config.output_rate);
                if (mode) {
                    strbf_puts(lsb, cfg_values[4]); // ",\"info\":\"gps_rate in Hz\",\"type\":\"int\""
                    strbf_puts(lsb, cfg_values[0]);
                    for (uint8_t i = 0, j = lengthof(sample_rates); i < j; i++) {
                        strbf_puts(lsb, cfg_values[1]);
                        strbf_putn(lsb, i==0 ? 1 : i==1 ? 5 : i==2 ? 10 : i==3 ? 16 : 20);
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
            case gps_cfg_dynamic_model: // choice for dynamic model "Sea",if 0 model "portable" is used !!
                strbf_putn(lsb, rtc_config.nav_mode);
                if (mode) {
                    strbf_puts(lsb, cfg_values[5]); // ",\"info\":\"choice for dynamic model 'Sea', if 0 model 'Portable' is used !!\",\"type\":\"int\",");
                    add_from_list(lsb, dynamic_models, lengthof(dynamic_models));
                }
                break;
            case gps_cfg_log_txt: // )])||!strcmp(name, "logTXT")) {  // switchinf off .txt files
                strbf_putc(lsb, GETBIT(log_config.log_file_bits, SD_TXT) == 1 ? '1' : '0');
                if (mode) {
                    strbf_puts(lsb, cfg_values[6]); // ",\"info\":\"log to"
                    strbf_puts(lsb, ".txt");
                    strbf_puts(lsb, cfg_values[7]); // ",\"type\":\"bool\"");
                }
                break;
            case gps_cfg_log_ubx: // )])||!strcmp(name, "logUBX")) {  // log to .ubx
                strbf_putc(lsb, GETBIT(log_config.log_file_bits, SD_UBX) == 0 ? '0' : '1');
                if (mode) {
                    strbf_puts(lsb, cfg_values[6]); // ",\"info\":\"log to"
                    strbf_puts(lsb, ".ubx");
                    strbf_puts(lsb, cfg_values[7]); // ",\"type\":\"bool\"");
                }
                break;
            case gps_cfg_log_ubx_nav_sat: // )])||!strcmp(name, "logUBX_nav_sat")) {  // log nav sat msg to .ubx
                strbf_putn(lsb, rtc_config.msgout_sat);
                if (mode) {
                    strbf_puts(lsb, cfg_values[8]); // ",\"info\":\"log nav sat msg to .ubx"
                    strbf_puts(lsb, cfg_values[7]); // ",\"type\":\"bool\"");
                }
                break;
            case gps_cfg_log_sbp: // )])||!strcmp(name, "logSBP")) {  // log to .sbp
                strbf_putc(lsb, GETBIT(log_config.log_file_bits, SD_SBP) == 0 ? '0' : '1');
                if (mode) {
                    strbf_puts(lsb, cfg_values[6]); // ",\"info\":\"log to"
                    strbf_puts(lsb, ".sbp");
                    strbf_puts(lsb, cfg_values[7]); // ",\"type\":\"bool\"");
                }
                break;
            case gps_cfg_log_gpx: // )])||!strcmp(name, "logGPX")) {   // log to .gpx
                strbf_putc(lsb, GETBIT(log_config.log_file_bits, SD_GPX) == 0 ? '0' : '1');
                if (mode) {
                    strbf_puts(lsb, cfg_values[6]); // ",\"info\":\"log to"
                    strbf_puts(lsb, ".gpx");
                    strbf_puts(lsb, cfg_values[7]); // ",\"type\":\"bool\"");
                }
                break;
#ifdef GPS_LOG_ENABLE_GPY
            case gps_cfg_log_gpy: // )])||!strcmp(name, "logGPY")) {   // log to .gpy
                strbf_putc(lsb, GETBIT(log_config.log_file_bits, SD_GPY) == 0 ? '0' : '1');
                if (mode) {
                    strbf_puts(lsb, cfg_values[6]); // ",\"info\":\"log to"
                    strbf_puts(lsb, ".gpy");
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
    DLOG(TAG, "[%s] conf: %s len: %d\n", __func__, strbf_finish(lsb), lsb->cur - lsb->start);
    return pos;
}

char *gps_config_get(const char *name, struct strbf_s *lsb, uint8_t mode) {
    ILOG(TAG, "[%s] name:%s", __func__, name ? name : "-");
    assert(lsb);
    uint8_t pos = gps_cfg_get_pos(name);
    if (pos == 255) {
        goto end;
    }
    gps_cnf_get_item(pos, lsb, mode);
    end:
    return strbf_finish(lsb);
}

char * gps_config_encode(strbf_t *sb, uint8_t mode, uint8_t mode2) {
    assert(sb);
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
