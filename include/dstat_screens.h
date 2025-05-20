#ifndef A20E356D_FBD0_4BD9_A843_CEE33E78C520
#define A20E356D_FBD0_4BD9_A843_CEE33E78C520

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

typedef float (*screen_f_func_float_t)(void);

typedef size_t (*screen_f_func_time_t)(char *);

enum screen_f_types {
    SCR_TYPE_FLOAT = 0,
    SCR_TYPE_CHAR = 1,
    SCR_TYPE_TIME = 2,
    SCR_TYPE_TIME_HM = 3,
    SCR_TYPE_TIME_HMS = 4,
};

typedef struct screen_f_s {
    uint8_t id;
    enum screen_f_types type;
    union {
        screen_f_func_float_t num;
        screen_f_func_time_t timestr;
    } value;
    const char *name;
    const char *abbr;
    const char *grp;
} screen_f_t;

#define SCREEN_FIELD_MAX_LEN 16
typedef struct screen_field_s {
    const screen_f_t *field;
    char data[SCREEN_FIELD_MAX_LEN];
    uint32_t dlen;
} screen_field_t;

typedef struct stat_screen_s {
    uint8_t cols;
    uint8_t rows;
    const screen_field_t fields[12];
    uint8_t num_fields;
    bool use_abbr;
} stat_screen_t;

// #define STAT_SCREENS_NUM 7
// uint8_t get_stat_screens_count(void);
extern const screen_f_t avail_fields[];

size_t get_display_fld_str(const screen_f_t *fld, char *p1, size_t (*fn)(double, char *));

enum avail_fields_e {
  fld_s10_display_last = 0,
  fld_s10_display_max,
  fld_s10_display_avg,
  fld_s10_display_max_time,

  fld_s2_display_last,
  fld_s2_display_max,
  fld_s2_display_avg,
  fld_s2_display_max_time,
  
  fld_m500_display_last,
  fld_m500_display_max,
  fld_m500_display_avg,
  fld_m500_display_max_time,
  
  fld_m250_display_last,
  fld_m250_display_max,
  fld_m250_display_avg,
  fld_m250_display_max_time,
  
  fld_s10_r1_display,
  fld_s10_r2_display,
  fld_s10_r3_display,
  fld_s10_r4_display,
  fld_s10_r5_display,

  fld_m1852_display_last,
  fld_m1852_display_max,
  fld_m1852_display_avg,
  fld_m1852_display_max_time,

  fld_a500_display_last,
  fld_a500_display_max,
  fld_a500_display_avg,
  fld_a500_display_max_time,

  fld_m100_display_last,
  fld_m100_display_max,
  fld_m100_display_avg,
  fld_m100_display_max_time,
  
  fld_s1800_display_last,
  fld_s1800_display_max,
  fld_s1800_display_avg,
  fld_s1800_display_max_time,
  
  fld_s3600_display_last,
  fld_s3600_display_max,
  fld_s3600_display_avg,
  fld_s3600_display_max_time,
  
  fld_distance,
  
  fld_a500_r1_display,
  fld_a500_r2_display,
  fld_a500_r3_display,
  fld_a500_r4_display,
  fld_a500_r5_display,
  
  fld_s2_r1_display,
  fld_s2_r2_display,
  
  fld_s1800_r1_display,
  fld_s1800_r2_display,
  
  fld_m250_r1_display,
  fld_m250_r2_display,
  
  fld_m500_r1_display,
  fld_m500_r2_display,
  
  fld_m1852_r1_display,
  fld_m1852_r2_display,
  
  fld_s10_s_max,
  
  fld_total_time_hms,
  fld_total_time_sec,
  fld_run_time_sec,

  fld_a500_a_max,
  fld_m1852_m_max,
  fld_m500_m_max,
  fld_s1800_s_max,
  fld_s3600_s_max
};

#ifdef __cplusplus
}
#endif

#endif /* A20E356D_FBD0_4BD9_A843_CEE33E78C520 */
