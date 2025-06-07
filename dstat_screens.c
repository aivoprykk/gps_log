#include "dstat_screens.h"
#include "gps_data.h"
#include "gps_user_cfg.h"
#include "numstr.h"

#if defined(CONFIG_GPS_LOG_ENABLED)
// extern struct context_s m_context;
// extern struct context_rtc_s m_context_rtc;
extern struct gps_context_s * gps;

static inline float get_avg(const gps_run_t *b) {
    return (float) ((b[9].avg_speed + b[8].avg_speed + b[7].avg_speed + b[6].avg_speed + b[5].avg_speed) / 5) * c_gps_cfg.speed_calibration;
}
static inline float get_spd(float b) {
    return (float) b * c_gps_cfg.speed_calibration;
}

size_t get_display_fld_str(const screen_f_t *fld, char *p1, size_t (*fn)(double, char *)) {
    if (fld->type == SCR_TYPE_FLOAT) {
        return fn(fld->value.num(), p1);
    } else {
        return fld->value.timestr(p1);
    }
}
static float s_display_max(struct gps_speed_by_time_s * s) {
    return get_spd((float)s->speed.display.display_speed[9]);
}
static float s_display_last(struct gps_speed_by_time_s * s) {
    return get_spd((float)s->speed.display.display_last_run_max_speed);
}

static float S10_display_last(void) {
    return get_spd((float)gps->S10.speed.display.display_last_run_max_speed);
}
static float S10_display_max(void) {
    return get_spd((float)gps->S10.speed.display.display_speed[9]);
}
static float S10_s_max(void) {
    return get_spd((float)gps->S10.speed.max_speed);
}
static float S10_display_avg(void) {
    return get_spd((float)gps->S10.speed.avg_5runs);
}
static size_t S10_display_max_time(char *p1) {
    return time_to_char_hm(gps->S10.speed.runs[9].time.hour, gps->S10.speed.runs[9].time.minute, p1);
}
static float S10_r1_display(void) {
    return get_spd((float)gps->S10.speed.display.display_speed[9]);
}
static float S10_r2_display(void) {
    return get_spd((float)gps->S10.speed.display.display_speed[8]);
}
static float S10_r3_display(void) {
    return get_spd((float)gps->S10.speed.display.display_speed[7]);
}
static float S10_r4_display(void) {
    return get_spd((float)gps->S10.speed.display.display_speed[6]);
}
static float S10_r5_display(void) {
    return get_spd((float)gps->S10.speed.display.display_speed[5]);
}
static float S2_display_last(void) {
    return get_spd((float)gps->S2.speed.display.display_last_run_max_speed);
}
static float S2_display_max(void) {
    return get_spd((float)gps->S2.speed.display.display_speed[9]);
}
static float S2_r1_display(void) {
    return get_spd((float)gps->S2.speed.display.display_speed[9]);
}
static float S2_r2_display(void) {
    return get_spd((float)gps->S2.speed.display.display_speed[8]);
}
static size_t S2_display_max_time(char *p1) {
    return time_to_char_hm(gps->S2.speed.runs[9].time.hour, gps->S2.speed.runs[9].time.minute, p1);
}
static float S1800_display_last(void) {
    return get_spd((float)gps->S1800.speed.display.display_last_run_max_speed);
}
static float S1800_display_max(void) {
    return get_spd((float)gps->S1800.speed.display.display_max_speed);
}
static float S1800_s_max(void) {
    return get_spd((float)gps->S1800.speed.speed);
}
static size_t S1800_display_max_time(char *p1) {
    return time_to_char_hm(gps->S1800.speed.runs[9].time.hour, gps->S1800.speed.runs[9].time.minute, p1);
}
static float S1800_r1_display(void) {
    return get_spd((float)gps->S1800.speed.display.display_speed[9]);
}
static float S1800_r2_display(void) {
    return get_spd((float)gps->S1800.speed.display.display_speed[8]);
}
static float S3600_display_last(void) {
    return get_spd((float)gps->S3600.speed.display.display_last_run_max_speed);
}
static float S3600_display_max(void) {
    return get_spd((float)gps->S3600.speed.display.display_max_speed);
}
static float S3600_s_max(void) {
    return get_spd(gps->S3600.speed.speed);
}
static size_t S3600_display_max_time(char *p1) {
    return time_to_char_hm(gps->S3600.speed.runs[9].time.hour, gps->S3600.speed.runs[9].time.minute, p1);
}
static float M250_display_last(void) {
    return get_spd((float)gps->M250.speed.display.display_speed[0]);
}
static float M250_display_max(void) {
    return get_spd((float)gps->M250.speed.display.display_max_speed);
}
static size_t M250_display_max_time(char *p1) {
    return time_to_char_hm(gps->M250.speed.runs[9].time.hour, gps->M250.speed.runs[9].time.minute, p1);
}
static float M250_r1_display(void) {
    return get_spd((float)gps->M250.speed.display.display_speed[9]);
}
static float M250_r2_display(void) {
    return get_spd((float)gps->M250.speed.display.display_speed[8]);
}
static float M500_display_last(void) {
    return get_spd((float)gps->M500.speed.display.display_speed[0]);
}
static float M500_display_max(void) {
    return get_spd((float)gps->M500.speed.display.display_max_speed);
}
static float M500_m_max(void) {
    return get_spd((float)gps->M500.speed.max_speed);
}
static size_t M500_display_max_time(char *p1) {
    return time_to_char_hm(gps->M500.speed.runs[9].time.hour, gps->M500.speed.runs[9].time.minute, p1);
}
static float M500_r1_display(void) {
    return get_spd((float)gps->M500.speed.display.display_speed[9]);
}
static float M500_r2_display(void) {
    return get_spd((float)gps->M500.speed.display.display_speed[8]);
}
static float M1852_display_last(void) {
    return get_spd((float)gps->M1852.speed.display.display_speed[0]);
}
static float M1852_display_max(void) {
    return get_spd((float)gps->M1852.speed.display.display_speed[9]);
}
static float M1852_m_max(void) {
    return get_spd((float)gps->M1852.speed.max_speed);
}
static size_t M1852_display_max_time(char *p1) {
    return time_to_char_hm(gps->M1852.speed.runs[9].time.hour, gps->M1852.speed.runs[9].time.minute, p1);
}
static float M1852_r1_display(void) {
    return get_spd((float)gps->M1852.speed.display.display_speed[9]);
}
static float M1852_r2_display(void) {
    return get_spd((float)gps->M1852.speed.display.display_speed[8]);
}
static float M100_display_last(void) {
    return get_spd((float)gps->M100.speed.display.display_speed[0]);
}
static float M100_display_max(void) {
    return get_spd((float)gps->M100.speed.display.display_max_speed);
}
static size_t M100_display_max_time(char *p1) {
    return time_to_char_hm(gps->M100.speed.runs[9].time.hour, gps->M100.speed.runs[9].time.minute, p1);
}
static float A500_display_last(void) {
    return get_spd((float)gps->A500.speed.speed);
}
static float A500_display_max(void) {
    return get_spd((float)gps->A500.speed.display.display_max_speed);
}
static float A500_a_max(void) {
    return get_spd((float)gps->A500.speed.max_speed);
}
static float A500_display_avg(void) {
    return get_avg(gps->A500.speed.runs);
}
static size_t A500_display_max_time(char *p1) {
    return time_to_char_hm(gps->A500.speed.runs[9].time.hour, gps->A500.speed.runs[9].time.minute, p1);
}
static float A500_r1_display(void) {
    return get_spd(gps->A500.speed.runs[9].avg_speed);
}
static float A500_r2_display(void) {
    return get_spd((float)gps->A500.speed.runs[8].avg_speed);
}
static float A500_r3_display(void) {
    return get_spd((float)gps->A500.speed.runs[7].avg_speed);
}
static float A500_r4_display(void) {
    return get_spd((float)gps->A500.speed.runs[6].avg_speed);
}
static float A500_r5_display(void) {
    return get_spd((float)gps->A500.speed.runs[5].avg_speed);
}
static float distance(void) {
    return (float)MM_TO_KM(gps->Ublox.total_distance);
}
static float run_time_sec(void) {
    return (float) MS_TO_SEC(get_millis() - gps->Ublox.run_start_time);
}
static float total_time_sec(void) {
    return (float)MS_TO_SEC(get_millis() - gps->start_logging_millis);
}
static size_t total_time_hms(char *p1) {
    return sec_to_hms_str(total_time_sec(), p1);
}

const char s10[] = "10s";
const char s2[] = "2s";
const char m500[] = "500m";
const char m250[] = "250m";
const char m1852[] = "Nm";
const char a500[] = "A500";
const char m100[] = "100m";
const char s1800[] = ".5h";
const char s3600[] = "1h";
const char ttime[] = "Tm";

const screen_f_t avail_fields[] = {
    {fld_s10_display_last, SCR_TYPE_FLOAT, .value.num = S10_display_last, "10sLst", "L", s10},
    {fld_s10_display_max, SCR_TYPE_FLOAT, .value.num = S10_display_max, "10s", "M", s10},
    {fld_s10_display_avg, SCR_TYPE_FLOAT, .value.num = S10_display_avg, "Avg", "Avg", s10},
    {fld_s10_display_max_time, SCR_TYPE_TIME_HM, .value.timestr = S10_display_max_time, "10sTm", "Tm", s10},

    {fld_s2_display_last, SCR_TYPE_FLOAT, .value.num = S2_display_last, "2sLst", "L", s2},
    {fld_s2_display_max, SCR_TYPE_FLOAT, .value.num = S2_display_max, "2s", "M", s2},
    {0},
    {fld_s2_display_max_time, SCR_TYPE_TIME_HM, .value.timestr = S2_display_max_time, "2sTm", "Tm", s2},

    {fld_m500_display_last, SCR_TYPE_FLOAT, .value.num = M500_display_last, "500Lst", "L", m500}, // shows 0
    {fld_m500_display_max, SCR_TYPE_FLOAT, .value.num = M500_display_max, "500m", "M", m500}, // shows 0
    {0},
    {fld_m500_display_max_time, SCR_TYPE_TIME_HM, .value.timestr = M500_display_max_time, "500mTm", "Tm", m500},

    {fld_m250_display_last, SCR_TYPE_FLOAT, .value.num = M250_display_last, "250Lst", "L", m250}, // shows 0
    {fld_m250_display_max, SCR_TYPE_FLOAT, .value.num = M250_display_max, "250m", "M", m250}, // shows 0
    {0},
    {fld_m250_display_max_time, SCR_TYPE_TIME_HM, .value.timestr = M250_display_max_time, "250mTm", "Tm", m250},

    {fld_s10_r1_display, SCR_TYPE_FLOAT, .value.num = S10_r1_display, "10sR1", "R1", s10},
    {fld_s10_r2_display, SCR_TYPE_FLOAT, .value.num = S10_r2_display, "10sR2", "R2", s10},
    {fld_s10_r3_display, SCR_TYPE_FLOAT, .value.num = S10_r3_display, "10sR3", "R3", s10},
    {fld_s10_r4_display, SCR_TYPE_FLOAT, .value.num = S10_r4_display, "10sR4", "R4", s10},
    {fld_s10_r5_display, SCR_TYPE_FLOAT, .value.num = S10_r5_display, "10sR5", "R5", s10},

    {fld_m1852_display_last, SCR_TYPE_FLOAT, .value.num = M1852_display_last, "NmLst", "L", m1852}, // shows 0
    {fld_m1852_display_max, SCR_TYPE_FLOAT, .value.num = M1852_display_max, "Nm", "M", m1852}, // shows 0
    {0},
    {fld_m1852_display_max_time, SCR_TYPE_TIME_HM, .value.timestr = M1852_display_max_time, "NmTm", "Tm", m1852},

    {fld_a500_display_last, SCR_TYPE_FLOAT, .value.num = A500_display_last, "AlLst", "L", a500},
    {fld_a500_display_max, SCR_TYPE_FLOAT, .value.num = A500_display_max, "Al", "M", a500},
    {fld_a500_display_avg, SCR_TYPE_FLOAT, .value.num = A500_display_avg, "AlAvg", "Al", a500},
    {fld_a500_display_max_time, SCR_TYPE_TIME_HM, .value.timestr = A500_display_max_time, "AlTm", "Tm", a500},

    {fld_m100_display_last, SCR_TYPE_FLOAT, .value.num = M100_display_last, "100Lst", "L", m100},
    {fld_m100_display_max, SCR_TYPE_FLOAT, .value.num = M100_display_max, "100m", "M", m100},
    {0},
    {fld_m100_display_max_time, SCR_TYPE_TIME_HM, .value.timestr = M100_display_max_time, "100mtm", "Tm", m100},

    {fld_s1800_display_last, SCR_TYPE_FLOAT, .value.num = S1800_display_last, ".5hLst", "L", s1800},
    {fld_s1800_display_max, SCR_TYPE_FLOAT, .value.num = S1800_display_max, ".5h", "M", s1800},
    {0},
    {fld_s1800_display_max_time, SCR_TYPE_TIME_HM, .value.timestr = S1800_display_max_time, ".5hTm", "Tm", s1800},

    {fld_s3600_display_last, SCR_TYPE_FLOAT, .value.num = S3600_display_last, "1hLst", "L", s3600},
    {fld_s3600_display_max, SCR_TYPE_FLOAT, .value.num = S3600_display_max, "1h", "M", s3600},
    {0},
    {fld_s3600_display_max_time, SCR_TYPE_TIME_HM, .value.timestr = S3600_display_max_time, "1hTM", "Tm", s3600},

    {fld_distance, SCR_TYPE_FLOAT, .value.num = distance, "Dist", "Dst", "Distance"},

    {fld_a500_r1_display, SCR_TYPE_FLOAT, .value.num = A500_r1_display, "AlR1", "AR1", a500},
    {fld_a500_r2_display, SCR_TYPE_FLOAT, .value.num = A500_r2_display, "AlR2", "AR2", a500},
    {fld_a500_r3_display, SCR_TYPE_FLOAT, .value.num = A500_r3_display, "AlR3", "AR3", a500},
    {fld_a500_r4_display, SCR_TYPE_FLOAT, .value.num = A500_r4_display, "AlR4", "AR4", a500},
    {fld_a500_r5_display, SCR_TYPE_FLOAT, .value.num = A500_r5_display, "AlR5", "AR5", a500},

    {fld_s2_r1_display, SCR_TYPE_FLOAT, .value.num = S2_r1_display, "2sR1", "R1", s2},
    {fld_s2_r2_display, SCR_TYPE_FLOAT, .value.num = S2_r2_display, "2sR2", "R2", s2},

    {fld_s1800_r1_display, SCR_TYPE_FLOAT, .value.num = S1800_r1_display, ".5hR1", "R1", s1800},
    {fld_s1800_r2_display, SCR_TYPE_FLOAT, .value.num = S1800_r2_display, ".5hR2", "R2", s1800},

    {fld_m250_r1_display, SCR_TYPE_FLOAT, .value.num = M250_r1_display, "250R1", "R1", m250},
    {fld_m250_r2_display, SCR_TYPE_FLOAT, .value.num = M250_r2_display, "250R2", "R2", m250},

    {fld_m500_r1_display, SCR_TYPE_FLOAT, .value.num = M500_r1_display, "500R1", "R1", m500},
    {fld_m500_r2_display, SCR_TYPE_FLOAT, .value.num = M500_r2_display, "500R2", "R2", m500},

    {fld_m1852_r1_display, SCR_TYPE_FLOAT, .value.num = M1852_r1_display, "NMR1", "R1", m1852},
    {fld_m1852_r2_display, SCR_TYPE_FLOAT, .value.num = M1852_r2_display, "NMR2", "R2", m1852},
    
    {fld_s10_s_max, SCR_TYPE_FLOAT, .value.num = S10_s_max, "10s", "M", s10}, // current max speed during run

    {fld_total_time_hms, SCR_TYPE_TIME_HMS, .value.timestr = total_time_hms, "Tm", "Tm", ttime},
    {fld_total_time_sec, SCR_TYPE_FLOAT, .value.num = total_time_sec, "Tm", "Tm", ttime},
    {fld_run_time_sec, SCR_TYPE_FLOAT, .value.num = run_time_sec, "RTm", "RTm", ttime},

    {fld_a500_a_max, SCR_TYPE_FLOAT, .value.num = A500_a_max, "Al", "M", a500}, // current max speed during run
    {fld_m1852_m_max, SCR_TYPE_FLOAT, .value.num = M1852_m_max, "Nm", "M", m1852}, // current max speed during run
    {fld_m500_m_max, SCR_TYPE_FLOAT, .value.num = M500_m_max, "500m", "M", m500}, // current max speed during run
    {fld_s1800_s_max, SCR_TYPE_FLOAT, .value.num = S1800_s_max, ".5h", "M", s1800}, // current max speed during run
    {fld_s3600_s_max, SCR_TYPE_FLOAT, .value.num = S3600_s_max, "1h", "M", s3600}, // current max speed during run
};

const stat_screen_t sc_screens[] = {
    { // 0 10s
        .cols = 1,
        .rows = 3,
        .num_fields = 3,
        .fields[0].field = &avail_fields[fld_s10_display_last],
        .fields[1].field = &avail_fields[fld_s10_display_max],
        .fields[2].field = &avail_fields[fld_s10_display_avg],
        .use_abbr = false,
    },
    { // 1 2s
        .cols = 1,
        .rows = 2,
        .num_fields = 2,
        .fields[0].field = &avail_fields[fld_s2_display_last],
        .fields[1].field = &avail_fields[fld_s2_display_max],
        .use_abbr = false,
    },
    { // 2 250m
        .cols = 1,
        .rows = 2,
        .num_fields = 2,
        .fields[0].field = &avail_fields[fld_m250_display_last],
        .fields[1].field = &avail_fields[fld_m250_display_max],
        .use_abbr = false,
    },
    { // 3 500m
        .cols = 1,
        .rows = 2,
        .num_fields = 2,
        .fields[0].field = &avail_fields[fld_m500_display_last],
        .fields[1].field = &avail_fields[fld_m500_display_max],
        .use_abbr = false,
    },
    { // 4 1800m
        .cols = 1,
        .rows = 2,
        .num_fields = 2,
        .fields[0].field = &avail_fields[fld_m1852_display_last],
        .fields[1].field = &avail_fields[fld_m1852_display_max],
        .use_abbr = false,
    },
    { // 5 a500
        .cols = 1,
        .rows = 2,
        .num_fields = 2,
        .fields[0].field = &avail_fields[fld_a500_display_last],
        .fields[1].field = &avail_fields[fld_a500_display_max],
        .use_abbr = false,
    },
    { // 6 10sec avg 5runs
        .cols = 2,
        .rows = 3,
        .num_fields = 6,
        .fields[0].field = &avail_fields[fld_s10_display_avg],  // 10s avg
        .fields[1].field = &avail_fields[fld_s10_r1_display], // r1
        .fields[2].field = &avail_fields[fld_s10_r2_display], // r2
        .fields[3].field = &avail_fields[fld_s10_r3_display], // r3
        .fields[4].field = &avail_fields[fld_s10_r4_display], // r4
        .fields[5].field = &avail_fields[fld_s10_r5_display], // r5
        .use_abbr = true,
    },
    { //7 stats
        .cols = 2,
        .rows = 3,
        .num_fields = 6,
        .fields[0].field = &avail_fields[fld_total_time_hms], // toal time sec
        .fields[1].field = &avail_fields[fld_distance], // distance
        .fields[2].field = &avail_fields[fld_m250_display_max], // 250m max
        .fields[3].field = &avail_fields[fld_m500_display_max],  // 2s max
        .fields[4].field = &avail_fields[fld_m1852_display_max], // Nm max
        .fields[5].field = &avail_fields[fld_a500_display_max],
        .use_abbr = false,
    },
    { //8 stats
        .cols = 2,
        .rows = 3,
        .num_fields = 6,
        .fields[0].field = &avail_fields[fld_total_time_hms], // toal time sec
        .fields[1].field = &avail_fields[fld_distance], // distance
        .fields[2].field = &avail_fields[fld_s2_display_max],
        .fields[3].field = &avail_fields[fld_s10_display_max],
        .fields[4].field = &avail_fields[fld_s1800_display_max],
        .fields[5].field = &avail_fields[fld_s3600_display_max],
        .use_abbr = false,
    },
    { // 9 a500 avg
        .cols = 2,
        .rows = 3,
        .num_fields = 6,
        .fields[0].field = &avail_fields[fld_a500_display_avg], // a500 avg
        .fields[1].field = &avail_fields[fld_a500_r1_display], // r1
        .fields[2].field = &avail_fields[fld_a500_r2_display], // r2
        .fields[3].field = &avail_fields[fld_a500_r3_display], // r3
        .fields[4].field = &avail_fields[fld_a500_r4_display], // r4
        .fields[5].field = &avail_fields[fld_a500_r5_display], // r5
        .use_abbr = true,
    },
};

// uint8_t get_stat_screens_count(void) {
//     return sizeof(sc_screens) / sizeof(stat_screen_t);
// }
#endif
