#include "dstat_screens.h"
#include "gps_data.h"
#include "gps_user_cfg.h"
#include "numstr.h"

#if defined(CONFIG_GPS_LOG_ENABLED)
// extern struct context_s m_context;
// extern struct context_rtc_s m_context_rtc;
extern struct gps_context_s * gps;
extern struct gps_user_cfg_s c_gps_cfg;

static inline float get_avg(const double *b) {
    return (float) ((b[9] + b[8] + b[7] + b[6] + b[5]) / 5) * c_gps_cfg.speed_calibration;
}
static inline float get_spd(float b) {
    return (float) b * c_gps_cfg.speed_calibration;
}

size_t get_display_fld_str(const screen_f_t *fld, char *p1, size_t (*fn)(double, char *)) {
    if (fld->type == type_float) {
        return fn(fld->value.num(), p1);
    } else {
        return fld->value.timestr(p1);
    }
}
static float s_display_max(struct gps_speed_by_time_s * s) {
    return get_spd((float)s->display_speed[9]);
}
static float s_display_last(struct gps_speed_by_time_s * s) {
    return get_spd((float)s->display_last_run);
}

static float S10_display_last(void) {
    return get_spd((float)gps->S10.display_last_run);
}
static float S10_display_max(void) {
    return get_spd((float)gps->S10.display_speed[9]);
}
static float S10_s_max(void) {
    return get_spd((float)gps->S10.s_max_speed);
}
static float S10_display_avg(void) {
    return get_spd((float)gps->S10.avg_5runs);
}
static size_t S10_display_max_time(char *p1) {
    return time_to_char_hm(gps->S10.time_hour[9], gps->S10.time_min[9], p1);
}
static float S10_r1_display(void) {
    return get_spd((float)gps->S10.display_speed[9]);
}
static float S10_r2_display(void) {
    return get_spd((float)gps->S10.display_speed[8]);
}
static float S10_r3_display(void) {
    return get_spd((float)gps->S10.display_speed[7]);
}
static float S10_r4_display(void) {
    return get_spd((float)gps->S10.display_speed[6]);
}
static float S10_r5_display(void) {
    return get_spd((float)gps->S10.display_speed[5]);
}
static float S2_display_last(void) {
    return get_spd((float)gps->S2.display_last_run);
}
static float S2_display_max(void) {
    return get_spd((float)gps->S2.display_speed[9]);
}
static float S2_r1_display(void) {
    return get_spd((float)gps->S2.display_speed[9]);
}
static float S2_r2_display(void) {
    return get_spd((float)gps->S2.display_speed[8]);
}
static size_t S2_display_max_time(char *p1) {
    return time_to_char_hm(gps->S2.time_hour[9], gps->S2.time_min[9], p1);
}
static float S1800_display_last(void) {
    return get_spd((float)gps->S1800.display_last_run);
}
static float S1800_display_max(void) {
    return get_spd((float)gps->S1800.display_max_speed);
}
static float S1800_s_max(void) {
    return get_spd((float)gps->S1800.avg_s);
}
static size_t S1800_display_max_time(char *p1) {
    return time_to_char_hm(gps->S1800.time_hour[9], gps->S1800.time_min[9], p1);
}
static float S1800_r1_display(void) {
    return get_spd((float)gps->S1800.display_speed[9]);
}
static float S1800_r2_display(void) {
    return get_spd((float)gps->S1800.display_speed[8]);
}
static float S3600_display_last(void) {
    return get_spd((float)gps->S3600.display_last_run);
}
static float S3600_display_max(void) {
    return get_spd((float)gps->S3600.display_max_speed);
}
static float S3600_s_max(void) {
    return get_spd(gps->S3600.avg_s);
}
static size_t S3600_display_max_time(char *p1) {
    return time_to_char_hm(gps->S3600.time_hour[9], gps->S3600.time_min[9], p1);
}
static float M250_display_last(void) {
    return get_spd((float)gps->M250.display_speed[0]);
}
static float M250_display_max(void) {
    return get_spd((float)gps->M250.display_max_speed);
}
static size_t M250_display_max_time(char *p1) {
    return time_to_char_hm(gps->M250.time_hour[9], gps->M250.time_min[9], p1);
}
static float M250_r1_display(void) {
    return get_spd((float)gps->M250.display_speed[9]);
}
static float M250_r2_display(void) {
    return get_spd((float)gps->M250.display_speed[8]);
}
static float M500_display_last(void) {
    return get_spd((float)gps->M500.display_speed[0]);
}
static float M500_display_max(void) {
    return get_spd((float)gps->M500.display_max_speed);
}
static float M500_m_max(void) {
    return get_spd((float)gps->M500.m_max_speed);
}
static size_t M500_display_max_time(char *p1) {
    return time_to_char_hm(gps->M500.time_hour[9], gps->M500.time_min[9], p1);
}
static float M500_r1_display(void) {
    return get_spd((float)gps->M500.display_speed[9]);
}
static float M500_r2_display(void) {
    return get_spd((float)gps->M500.display_speed[8]);
}
static float M1852_display_last(void) {
    return get_spd((float)gps->M1852.display_speed[0]);
}
static float M1852_display_max(void) {
    return get_spd((float)gps->M1852.display_speed[9]);
}
static float M1852_m_max(void) {
    return get_spd((float)gps->M1852.m_max_speed);
}
static size_t M1852_display_max_time(char *p1) {
    return time_to_char_hm(gps->M1852.time_hour[9], gps->M1852.time_min[9], p1);
}
static float M1852_r1_display(void) {
    return get_spd((float)gps->M1852.display_speed[9]);
}
static float M1852_r2_display(void) {
    return get_spd((float)gps->M1852.display_speed[8]);
}
static float M100_display_last(void) {
    return get_spd((float)gps->M100.display_speed[0]);
}
static float M100_display_max(void) {
    return get_spd((float)gps->M100.display_max_speed);
}
static size_t M100_display_max_time(char *p1) {
    return time_to_char_hm(gps->M100.time_hour[9], gps->M100.time_min[9], p1);
}
static float A500_display_last(void) {
    return get_spd((float)gps->A500.alfa_speed);
}
static float A500_display_max(void) {
    return get_spd((float)gps->A500.display_max_speed);
}
static float A500_a_max(void) {
    return get_spd((float)gps->A500.alfa_speed_max);
}
static float A500_display_avg(void) {
    return get_avg(gps->A500.avg_speed);
}
static size_t A500_display_max_time(char *p1) {
    return time_to_char_hm(gps->A500.time_hour[9], gps->A500.time_min[9], p1);
}
static float A500_r1_display(void) {
    return get_spd((float)gps->A500.avg_speed[9]);
}
static float A500_r2_display(void) {
    return get_spd((float)gps->A500.avg_speed[8]);
}
static float A500_r3_display(void) {
    return get_spd((float)gps->A500.avg_speed[7]);
}
static float A500_r4_display(void) {
    return get_spd((float)gps->A500.avg_speed[6]);
}
static float A500_r5_display(void) {
    return get_spd((float)gps->A500.avg_speed[5]);
}
static float distance(void) {
    return (float)gps->Ublox.total_distance / 1000000;
}
static float run_time_sec(void) {
    return (float)(get_millis() - gps->Ublox.run_start_time) / 1000;
}
static float total_time_sec(void) {
    return (float)(get_millis() - gps->start_logging_millis) / 1000;
}
static size_t total_time_hms(char *p1) {
    return sec_to_hms_str(total_time_sec(), p1);
}

const char s10[] = "10s";
const char s2[] = "2s";
const char m500[] = "500m";
const char m250[] = "250m";
const char m1852[] = "NM";
const char a500[] = "A500";
const char m100[] = "100m";
const char s1800[] = ".5h";
const char s3600[] = "1h";
const char ttime[] = "Time";

const screen_f_t avail_fields[] = {
    {0, 0, .value.num = S10_display_last, "10sLST", "L", s10},
    {1, 0, .value.num = S10_display_max, "10sMAX", "M", s10},
    {2, 0, .value.num = S10_display_avg, "10sAVG", "10sA", s10},
    {3, 1, .value.timestr = S10_display_max_time, "10sMTM", "T", s10},

    {4, 0, .value.num = S2_display_last, "2sLST", "L", s2},
    {5, 0, .value.num = S2_display_max, "2sMAX", "M", s2},
    {0},
    {7, 1, .value.timestr = S2_display_max_time, "2sMTm", "T", s2},

    {8, 0, .value.num = M500_display_last, "500LST", "L", m500}, // shows 0
    {9, 0, .value.num = M500_display_max, "500MAX", "M", m500}, // shows 0
    {0},
    {11, 1, .value.timestr = M500_display_max_time, "500MTM", "T", m500},

    {12, 0, .value.num = M250_display_last, "250LST", "L", m250}, // shows 0
    {13, 0, .value.num = M250_display_max, "250MAX", "M", m250}, // shows 0
    {0},
    {15, 1, .value.timestr = M250_display_max_time, "250mMTM", "T", m250},

    {16, 0, .value.num = S10_r1_display, "10sR1", "R1", s10},
    {17, 0, .value.num = S10_r2_display, "10sR2", "R2", s10},
    {18, 0, .value.num = S10_r3_display, "10sR3", "R3", s10},
    {19, 0, .value.num = S10_r4_display, "10sR4", "R4", s10},
    {20, 0, .value.num = S10_r5_display, "10sR5", "R5", s10},

    {21, 0, .value.num = M1852_display_last, "NmLST", "L", m1852}, // shows 0
    {22, 0, .value.num = M1852_display_max, "NmMAX", "M", m1852}, // shows 0
    {0},
    {24, 1, .value.timestr = M1852_display_max_time, "NmMTM", "T", m1852},

    {25, 0, .value.num = A500_display_last, "ALFLST", "L", a500},
    {26, 0, .value.num = A500_display_max, "ALFMAX", "M", a500},
    {27, 0, .value.num = A500_display_avg, "ALFAVG", "AlfA", a500},
    {28, 1, .value.timestr = A500_display_max_time, "ALFMTM", "T", a500},

    {29, 0, .value.num = M100_display_last, "100LST", "L", m100},
    {30, 0, .value.num = M100_display_max, "100MAX", "M", m100},
    {0},
    {32, 1, .value.timestr = M100_display_max_time, "100MTM", "T", m100},

    {33, 0, .value.num = S1800_display_last, ".5hLST", "L", s1800},
    {34, 0, .value.num = S1800_display_max, ".5hMAX", "M", s1800},
    {0},
    {36, 1, .value.timestr = S1800_display_max_time, ".5hMTM", "T", s1800},

    {37, 0, .value.num = S3600_display_last, "1hLST", "L", s3600},
    {38, 0, .value.num = S3600_display_max, "1hMAX", "M", s3600},
    {0},
    {40, 1, .value.timestr = S3600_display_max_time, "1hMTM", "T", s3600},

    {41, 0, .value.num = distance, "Dist", "Dst", "Distance"},

    {42, 0, .value.num = A500_r1_display, "ALFR1", "R1", a500},
    {43, 0, .value.num = A500_r2_display, "ALFR2", "R2", a500},
    {44, 0, .value.num = A500_r3_display, "ALFR3", "R3", a500},
    {45, 0, .value.num = A500_r4_display, "ALFR4", "R4", a500},
    {46, 0, .value.num = A500_r5_display, "ALFR5", "R5", a500},

    {47, 0, .value.num = S2_r1_display, "2sR1", "R1", s2},
    {48, 0, .value.num = S2_r2_display, "2sR2", "R2", s2},

    {49, 0, .value.num = S1800_r1_display, ".5hR1", "R1", s1800},
    {50, 0, .value.num = S1800_r2_display, ".5hR2", "R2", s1800},

    {51, 0, .value.num = M250_r1_display, "250R1", "R1", m250},
    {52, 0, .value.num = M250_r2_display, "250R2", "R2", m250},

    {53, 0, .value.num = M500_r1_display, "500R1", "R1", m500},
    {54, 0, .value.num = M500_r2_display, "500R2", "R2", m500},

    {55, 0, .value.num = M1852_r1_display, "NMR1", "R1", m1852},
    {56, 0, .value.num = M1852_r2_display, "NMR2", "R2", m1852},
    
    {57, 0, .value.num = S10_s_max, "10max", "M", s10}, // current max speed during run

    {58, 1, .value.timestr = total_time_hms, "TTime", "TT", ttime},
    {59, 0, .value.num = total_time_sec, "TTime", "TT", ttime},
    {60, 0, .value.num = run_time_sec, "RTime", "RT", ttime},

    {61, 0, .value.num = A500_a_max, "ALFmax", "M", a500}, // current max speed during run
    {62, 0, .value.num = M1852_m_max, "NMmax", "M", m1852}, // current max speed during run
    {63, 0, .value.num = M500_m_max, "500max", "M", m500}, // current max speed during run
    {64, 0, .value.num = S1800_s_max, ".5hmax", "M", s1800}, // current max speed during run
    {65, 0, .value.num = S3600_s_max, "1hmax", "M", s3600}, // current max speed during run
};

const stat_screen_t sc_screens[] = {
    { // 0 10s
        .cols = 1,
        .rows = 3,
        .num_fields = 3,
        .fields[0].field = &avail_fields[0],
        .fields[1].field = &avail_fields[1],
        .fields[2].field = &avail_fields[2],
        .use_abbr = false,
    },
    { // 1 2s
        .cols = 1,
        .rows = 2,
        .num_fields = 2,
        .fields[0].field = &avail_fields[4],
        .fields[1].field = &avail_fields[5],
        .use_abbr = false,
    },
    { // 2 250m
        .cols = 1,
        .rows = 2,
        .num_fields = 2,
        .fields[0].field = &avail_fields[12],
        .fields[1].field = &avail_fields[13],
        .use_abbr = false,
    },
    { // 3 500m
        .cols = 1,
        .rows = 2,
        .num_fields = 2,
        .fields[0].field = &avail_fields[8],
        .fields[1].field = &avail_fields[9],
        .use_abbr = false,
    },
    { // 4 1800m
        .cols = 1,
        .rows = 2,
        .num_fields = 2,
        .fields[0].field = &avail_fields[21],
        .fields[1].field = &avail_fields[22],
        .use_abbr = false,
    },
    { // 5 a500
        .cols = 1,
        .rows = 2,
        .num_fields = 2,
        .fields[0].field = &avail_fields[25],
        .fields[1].field = &avail_fields[26],
        .use_abbr = false,
    },
    { // 6 10sec avg 5runs
        .cols = 2,
        .rows = 3,
        .num_fields = 6,
        .fields[0].field = &avail_fields[2],  // 10s avg
        .fields[1].field = &avail_fields[16], // r1
        .fields[2].field = &avail_fields[17], // r2
        .fields[3].field = &avail_fields[18], // r3
        .fields[4].field = &avail_fields[19], // r4
        .fields[5].field = &avail_fields[20], // r5
        .use_abbr = true,
    },
    { //7 stats
        .cols = 2,
        .rows = 3,
        .num_fields = 6,
        .fields[0].field = &avail_fields[58], // toal time sec
        .fields[1].field = &avail_fields[41], // distance
        .fields[2].field = &avail_fields[13], // 250m max
        .fields[3].field = &avail_fields[22], // Nm max
        .fields[4].field = &avail_fields[5],  // 2s max
        .fields[5].field = &avail_fields[38], // 1h max
        .use_abbr = false,
    },
    { // 8 a500 avg
        .cols = 2,
        .rows = 3,
        .num_fields = 6,
        .fields[0].field = &avail_fields[27], // a500 avg
        .fields[1].field = &avail_fields[42], // r1
        .fields[2].field = &avail_fields[43], // r2
        .fields[3].field = &avail_fields[44], // r3
        .fields[4].field = &avail_fields[45], // r4
        .fields[5].field = &avail_fields[46], // r5
        .use_abbr = true,
    },
};

uint8_t get_stat_screens_count(void) {
    return sizeof(sc_screens) / sizeof(stat_screen_t);
}
#endif