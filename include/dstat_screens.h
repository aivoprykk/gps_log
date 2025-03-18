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
    type_float = 0,
    type_char = 1
};

typedef struct screen_f_s {
    uint8_t id;
    uint8_t type;
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
uint8_t get_stat_screens_count(void);
extern const screen_f_t avail_fields[];

size_t get_display_fld_str(const screen_f_t *fld, char *p1, size_t (*fn)(double, char *));

#ifdef __cplusplus
}
#endif

#endif /* A20E356D_FBD0_4BD9_A843_CEE33E78C520 */
