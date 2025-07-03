#ifndef AB7E9635_4557_4135_A8A1_4B7B75B4D782
#define AB7E9635_4557_4135_A8A1_4B7B75B4D782

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "logger_common.h"

#define CFG_GPS_ITEM_BASE 80
extern const size_t gps_user_cfg_item_count;

#if defined (CONFIG_GPS_LOG_GPY)
#define CFG_GPS_USER_FILE_ITEMS(l, n) \
 l(log_txt,(CFG_GPS_ITEM_BASE + n)) \
 l(log_ubx,(CFG_GPS_ITEM_BASE + n + 1)) \
 l(log_sbp,(CFG_GPS_ITEM_BASE + n + 2)) \
 l(log_gpx,(CFG_GPS_ITEM_BASE + n + 3)) \
 l(log_gpy,(CFG_GPS_ITEM_BASE + n + 4))
#define CFG_GPS_USER_FILE_ITEMS_NUM 5
#else
#define CFG_GPS_USER_FILE_ITEMS(l, n) \
 l(log_txt,(CFG_GPS_ITEM_BASE + n)) \
 l(log_ubx,(CFG_GPS_ITEM_BASE + n + 1)) \
 l(log_sbp,(CFG_GPS_ITEM_BASE + n + 2)) \
 l(log_gpx,(CFG_GPS_ITEM_BASE + n + 3))
#define CFG_GPS_USER_FILE_ITEMS_NUM 4
#endif

#define CFG_GPS_USER_GNSS_ITEMS(l, n) \
 l(gnss,(CFG_GPS_ITEM_BASE+n)) \
 l(sample_rate,(CFG_GPS_ITEM_BASE + n + 1)) \
 l(timezone,(CFG_GPS_ITEM_BASE + n + 2)) \
 l(speed_unit,(CFG_GPS_ITEM_BASE + n + 3)) \
 l(stat_screens,(CFG_GPS_ITEM_BASE + n + 4)) \

#define CFG_GPS_USER_GNSS_ITEMS_NUM 5

#define CFG_GPS_USER_OTHER_ITEMS(l, n) \
 l(log_ubx_nav_sat,(CFG_GPS_ITEM_BASE + n)) \
 l(dynamic_model,(CFG_GPS_ITEM_BASE + n + 1)) \
 l(file_date_time,(CFG_GPS_ITEM_BASE + n + 2)) \
 l(ubx_file,(CFG_GPS_ITEM_BASE + n + 3))
#define CFG_GPS_USER_OTHER_ITEMS_NUM 4

#define CFG_GPS_USER_CFG_ITEM_LIST(l) \
 CFG_GPS_USER_GNSS_ITEMS(l, 0) \
 CFG_GPS_USER_FILE_ITEMS(l, CFG_GPS_USER_GNSS_ITEMS_NUM) \
 CFG_GPS_USER_OTHER_ITEMS(l, CFG_GPS_USER_GNSS_ITEMS_NUM + CFG_GPS_USER_FILE_ITEMS_NUM) \


#define GPS_CFG_ENUM_PRE(l) gps_cfg_##l
#define GPS_CFG_ENUM_PRE_N(l)  GPS_CFG_ENUM_PRE(l),
#define GPS_CFG_ENUM_PRE_L(l, m) GPS_CFG_ENUM_PRE(l) = (uint8_t)(m),
typedef enum {
    CFG_GPS_USER_CFG_ITEM_LIST(GPS_CFG_ENUM_PRE_L)
} gps_cfg_item_t;

#define GPS_UBX_FILE_MAX 22

typedef struct gps_user_cfg_s {
    uint8_t file_date_time;  // type of filenaming, with MAC adress or datetime
    uint8_t speed_unit;      // 0 = m/s, 1 = km/h, 2 = knots    
    float speed_calibration; // conversion m/s to km/h, for knots use 1.944
    float timezone;          // choice for timedifference in hours with UTC, for Belgium 1 or 2 (summertime)
    char ubx_file[GPS_UBX_FILE_MAX];    // your preferred filename
    uint16_t stat_screens;    // choice for stats field when no speed, here stat_screen 1, 2 and 3 will be active
    // takes 2 blocks as of 2*16bit
} gps_user_cfg_t;
// #define L_CONFIG_GPS_FIELDS sizeof(struct gps_user_cfg_s)

#define GPS_USER_CFG_DEFAULTS() { \
    .file_date_time = 1, \
    .speed_unit = 1, \
    .speed_calibration = 0.0036f, \
    .timezone = 2, \
    .ubx_file = "gps", \
    .stat_screens = 255U, \
}

struct gps_user_cfg_evt_data_s {
    int pos;
    int value;
};

#define GPS_STAT_S(a) uint8_t a;
#define GPS_STAT_E(l) .##l = 1, 
#define GPS_STAT_D(l)  GPS_STAT_D(l),

#define STAT_SCREEN_ITEM_LIST(l) \
    l(stat_10_sec) \
    l(stat_2_sec) \
    l(stat_250_m) \
    l(stat_500_m) \
    l(stat_1852_m) \
    l(stat_a500) \
    l(stat_avg_10sec) \
    l(stat_stat1) \
    l(stat_stat2) \
    l(stat_avg_a500)

enum stat_screen_items_e {
    STAT_SCREEN_ITEM_LIST(ENUM)
};

extern const char * const speed_units[];
extern const char * const dynamic_models[];
extern struct gps_user_cfg_s c_gps_cfg;
extern const size_t gps_user_cfg_item_count;
extern const size_t gps_stat_screen_item_count;
struct strbf_s;

void gps_user_cfg_init(void);
void gps_user_cfg_deinit(void);
struct m_config_item_s * get_gps_cfg_item(int num, struct m_config_item_s *item);
int set_gps_cfg_item(int num, bool skip_done_msg);
uint8_t gps_cfg_get_pos(const char *str);
uint8_t gps_cnf_set_item(uint8_t pos, void * el, uint8_t force);
int gps_config_set(const char *str, void * el, uint8_t force);
esp_err_t gps_config_decode(const char *json);
uint8_t gps_cnf_get_item(uint8_t pos, struct strbf_s * lsb, uint8_t mode);
char *gps_config_get(const char *name, struct strbf_s *sb, uint8_t mode);
char * gps_config_encode(struct strbf_s *sb, uint8_t mode, uint8_t mode2);
struct m_config_item_s * get_stat_screen_cfg_item(int num, struct m_config_item_s *item);
int set_stat_screen_cfg_item(int num);
void gps_config_fix_values(void);

#ifdef __cplusplus
}
#endif

#endif /* AB7E9635_4557_4135_A8A1_4B7B75B4D782 */
