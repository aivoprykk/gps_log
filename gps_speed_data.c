#include "log_private.h"
#include "gps_speed_data.h"
#include "ubx.h"
#include <math.h>

static const char *TAG = "gps_speed";

static const gps_speed_metrics_cfg_t initial_speed_metrics_sets[] = {
    { GPS_SPEED_TYPE_TIME, 2,    },
    { GPS_SPEED_TYPE_TIME, 10,   },
    { GPS_SPEED_TYPE_TIME, 1800, },
    { GPS_SPEED_TYPE_TIME, 3600, },
    { GPS_SPEED_TYPE_DIST, 100,  },
    { GPS_SPEED_TYPE_DIST|GPS_SPEED_TYPE_ALFA, 250,  },
    { GPS_SPEED_TYPE_DIST|GPS_SPEED_TYPE_ALFA, 500,  },
    { GPS_SPEED_TYPE_DIST, 1852, },
};

static inline uint32_t convert_distance_to_mm(int distance) {
     return M_TO_MM(distance) * gps->ubx_device->rtc_conf->output_rate;
}

static inline gps_speed_t * gps_select_speed_instance(int num, uint8_t flags) {
    if(!gps->speed_metrics) return 0;
    gps_speed_metrics_desc_t *spd = &gps->speed_metrics[num];
    if ((flags & GPS_SPEED_TYPE_ALFA) && spd->handle.dist && spd->handle.dist->alfa)
        return &spd->handle.dist->alfa->speed;
    else if ((flags & GPS_SPEED_TYPE_DIST) && spd->handle.dist)
        return &spd->handle.dist->speed;
    else if ((flags == GPS_SPEED_TYPE_TIME) && spd->handle.time)
        return &spd->handle.time->speed;
    return 0;
}

static gps_display_t * gps_get_time_display_struct(int set) {
    gps_speed_t *spd = gps_select_speed_instance(set, GPS_SPEED_TYPE_TIME);
    if(!spd) printf("alfa not found!\n");
    return spd ? &spd->display : 0;
}
static gps_display_t * gps_get_alfa_display_struct(int set) {
    gps_speed_t *spd = gps_select_speed_instance(set, GPS_SPEED_TYPE_ALFA);
    if(!spd) printf("alfa not found!\n");
    return spd ? &spd->display : 0;
}
static float gps_get_run_average_speed(int set, uint8_t type, int num) {
    gps_speed_t *spd = gps_select_speed_instance(set, type);
    return spd ? spd->runs[num].avg_speed : 0.0f;
}

static gps_tm_t * gps_get_run_time_struct(int set, uint8_t type, int num) {
    gps_speed_t *spd = gps_select_speed_instance(set, type);
    return spd ? &spd->runs[num].time : 0;
}

static float gps_get_cur_speed(int set, uint8_t type) {
    gps_speed_t *spd = gps_select_speed_instance(set, type);
    return spd ? spd->cur_speed : 0.0f;
}

static float gps_get_max_speed_value(int set, uint8_t type) {
    gps_speed_t *spd = gps_select_speed_instance(set, type);
    return spd ? spd->max_speed : 0.0f;
}

static float get_display_max_speed(int set, uint8_t type) {
    gps_speed_t *spd = gps_select_speed_instance(set, type);
    return spd ? spd->display.display_max_speed : 0.0f;
}

static float get_display_last_run_max_speed(int set, uint8_t type) {
    gps_speed_t *spd = gps_select_speed_instance(set, type);
    return spd ? spd->display.display_last_run_max_speed : 0.0f;
}

static float get_display_display_speed(int set, uint8_t type, int num) {
    gps_speed_t *spd = gps_select_speed_instance(set, type);
    return spd ? spd->display.display_speed[num] : 0.0f;
}

static bool get_display_record(int set, uint8_t type) {
    gps_speed_t *spd = gps_select_speed_instance(set, type);
    return spd ? spd->display.record : false;
}

gps_speed_op_t speed_ops = {
    &gps_get_run_average_speed,
    &gps_get_run_time_struct,
    &gps_get_cur_speed,
    &gps_get_max_speed_value,
    &get_display_max_speed,
    &get_display_last_run_max_speed,
    &get_display_display_speed,
    &get_display_record,
    &gps_get_time_display_struct,
    &gps_get_alfa_display_struct
};

esp_err_t gps_speed_metrics_add(const gps_speed_metrics_cfg_t *cfg, int pos) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s] idx: %d type: %d window: %d", __func__, pos, cfg->type, cfg->window);
#endif
    if (gps->speed_metrics) {
        gps_speed_metrics_desc_t *desc = &gps->speed_metrics[pos];
#if (C_LOG_LEVEL < 3)
        if(!desc) {
            ESP_LOGE(TAG, "[%s] Speed set %d not found", __func__, pos);
            return ESP_ERR_NOT_FOUND;
        }
        if(desc->handle.time || desc->handle.dist) {
            ESP_LOGW(TAG, "[%s] Speed set %d already exists, skipping.", __func__, pos);
            return ESP_OK; // Already exists
        }
#endif
        desc->type = cfg->type;
        desc->window = cfg->window;
        uint16_t size = 0;
        if ((cfg->type & SPEED_TYPE_MASK) == GPS_SPEED_TYPE_TIME) { // time bit set
            if(!desc->handle.time)
                check_and_alloc_buffer((void **)&desc->handle.time, size+1, sizeof(gps_speed_by_time_t), &size, MALLOC_CAP_DMA);
            if (desc->handle.time) {
                init_gps_speed_by_time(desc->handle.time, cfg->window);
                desc->handle.time->speed.flags = GPS_SPEED_TYPE_TIME;
            } else {
                goto err;
            }
        }
        else if ((cfg->type & (GPS_SPEED_TYPE_DIST | GPS_SPEED_TYPE_ALFA))) { // dist or alfa bit set
            if(!desc->handle.dist)
                check_and_alloc_buffer((void **)&desc->handle.dist, size+1, sizeof(gps_speed_by_dist_t), &size, MALLOC_CAP_DMA);
            if (desc->handle.dist) {
                init_gps_speed_by_distance(desc->handle.dist, cfg->window);
                desc->handle.dist->speed.flags = GPS_SPEED_TYPE_DIST;
                if ((cfg->type & GPS_SPEED_TYPE_ALFA)) { // check if alfa bit set
                    if(!desc->handle.dist->alfa)
                        // Allocate alfa speed instance if not already allocated
                        check_and_alloc_buffer((void **)&desc->handle.dist->alfa, size+1, sizeof(gps_speed_by_alfa_t), &size, MALLOC_CAP_DMA);
                    if (desc->handle.dist->alfa) {
                        init_gps_speed_by_alfa(desc->handle.dist);
                        desc->handle.dist->speed.flags |= GPS_SPEED_TYPE_ALFA;
                    } else {
                        goto err;
                    }
                }
            } else {
                goto err;
            }
        }
    } else {
        err:
        ESP_LOGE(TAG, "[%s] Failed to allocate memory.", __func__);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void gps_speed_metrics_check(const gps_speed_metrics_cfg_t *cfg, size_t num_sets) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    if(num_sets > gps->num_speed_metrics){
        check_and_alloc_buffer((void **)&gps->speed_metrics, num_sets, sizeof(gps_speed_metrics_desc_t), &gps->num_speed_metrics, MALLOC_CAP_DMA);
        memset(gps->speed_metrics, 0, num_sets * sizeof(gps_speed_metrics_desc_t));
        for (int i = 0; i < num_sets; ++i) {
            gps_speed_metrics_add(&cfg[i], i);
        }
    }
}

void gps_speed_metrics_init() {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    gps_speed_metrics_check(&initial_speed_metrics_sets[0], lengthof(initial_speed_metrics_sets));
}

void gps_speed_metrics_free(void) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    for (int i = 0; i < gps->num_speed_metrics; ++i) {
        if (gps->speed_metrics[i].type == GPS_SPEED_TYPE_TIME) {
            unalloc_buffer((void **)&gps->speed_metrics[i].handle.time);
        } else if (gps->speed_metrics[i].type & (GPS_SPEED_TYPE_DIST | GPS_SPEED_TYPE_ALFA)){
            if (gps->speed_metrics[i].type & (GPS_SPEED_TYPE_ALFA)) {
                unalloc_buffer((void **)&gps->speed_metrics[i].handle.dist->alfa);
            }
            unalloc_buffer((void **)&gps->speed_metrics[i].handle.dist);
        }
    }
    unalloc_buffer((void **)&gps->speed_metrics);
    gps->speed_metrics = NULL;
    gps->num_speed_metrics = 0;
}

void gps_speed_metrics_update(void) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s] sets: %hu", __func__, gps->num_speed_metrics);
#endif
    for(uint8_t i = 0, j = gps->num_speed_metrics; i < j; i++) {
        if (gps->speed_metrics[i].type == GPS_SPEED_TYPE_TIME) {
            update_speed_by_time(gps->speed_metrics[i].handle.time);
        } else if (gps->speed_metrics[i].type & (GPS_SPEED_TYPE_DIST | GPS_SPEED_TYPE_ALFA)) {
            update_speed_by_distance(gps->speed_metrics[i].handle.dist);
            update_speed_by_alfa(gps->speed_metrics[i].handle.dist);
        }
    }
    gps_update_max_speed();
}

void refresh_gps_speeds_by_distance(void) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    for(uint8_t i = 0, j = gps->num_speed_metrics; i < j; i++) {
        if (!(gps->speed_metrics[i].type  & (GPS_SPEED_TYPE_DIST))) continue;
        if (gps->speed_metrics[i].window == 0) continue; // skip zero window sets
        struct gps_speed_by_dist_s *spd = gps->speed_metrics[i].handle.dist;
        if(spd)
            spd->distance_window_raw = convert_distance_to_mm(spd->distance_window);
    }
}

static inline void store_time(gps_run_t *run) {
    struct tm tms;
    get_local_time(&tms);
    run->time.hour = (uint8_t)tms.tm_hour;
    run->time.minute = (uint8_t)tms.tm_min;
    run->time.second = (uint8_t)tms.tm_sec;
}

void gps_update_max_speed(void) {
    if(gps->gps_speed > gps->max_speed.avg_speed) {
        gps->max_speed.avg_speed = gps->gps_speed;
        store_time(&gps->max_speed);
        gps->max_speed.nr = gps->run_count;
    }
}

// Helper for byte-wise swap
static inline void swap_bytes(void *a, void *b, size_t n) {
    uint8_t *pa = (uint8_t *)a;
    uint8_t *pb = (uint8_t *)b;
    for (uint8_t k = 0; k < n; ++k) {
        uint8_t tmp = pa[k]; pa[k] = pb[k];pb[k] = tmp;
    }
}

// Unified sort routine for gps_run_t and up to n parallel arrays
static void sort_runs(
    gps_run_t runs[],
    void *arrs[], size_t strides[], int num_arrays,
    uint8_t size
) {
    for (uint8_t i = 0; i < (size - 1); i++) {
        for (uint8_t o = 0; o < (size - (i + 1)); o++) {
            if (runs[o].avg_speed > runs[o + 1].avg_speed) {
                gps_run_t t = runs[o]; runs[o] = runs[o + 1]; runs[o + 1] = t;
                for (int arr_idx = 0; arr_idx < num_arrays; ++arr_idx) {
                    if (arrs[arr_idx]) {
                        swap_bytes(
                            (uint8_t*)arrs[arr_idx] + o * strides[arr_idx],
                            (uint8_t*)arrs[arr_idx] + (o + 1) * strides[arr_idx],
                            strides[arr_idx]
                        );
                    }
                }
            }
        }
#if defined(GPS_STATS)
        gps_run_printf(&runs[i]);
#endif
    }
}

static void sort_display(float a[], uint8_t size) {
    for (uint8_t i = 0; i < (size - 1); i++) {
        for (uint8_t o = 0; o < (size - (i + 1)); o++) {
            if (a[o] > a[o + 1]) {
                float t = a[o]; a[o] = a[o + 1]; a[o + 1] = t;
            }
        }
    }
}

#if defined(GPS_STATS)
static esp_err_t gps_display_printf(const gps_display_t * me) {
    uint8_t i, j=NUM_OF_SPD_ARRAY_SIZE;
    printf(" display:{ \n");
    printf("  max_speed: %.02f, ", me->display_max_speed);
    printf("last_max_speed: %.02f, ", me->display_last_run_max_speed);
    printf("record: %d\n", me->record);
    printf("  speed: ");
    for (i = 0; i < j; i++) printf("%.02f ", me->display_speed[i]);
    printf("\n }\n");
    return ESP_OK;
}

static esp_err_t gps_speed_printf(const gps_speed_t * me) {
    uint8_t i, j=NUM_OF_SPD_ARRAY_SIZE;
    printf(" == speed:{\n");
    printf(" speed: %.02f, ", me->speed);
    // printf("speed_alfa: %.02f, ", me->speed_alfa);
    printf("max_speed: %.02f, ", me->max_speed);
    // printf("avg_5runs: %.02f\n", me->avg_5runs);
    for (i = 0; i < j; i++) {
        printf(" %hhu ", i); gps_run_printf(&me->runs[i]);
    }
    gps_display_printf(&me->display);
    printf(" } ==\n");
    return ESP_OK;
}
#endif


static void record_last_run(gps_speed_t * speed, uint16_t actual_run) {
    // printf("[%s] %.01f %hu %hu\n", __func__, speed->max_speed, actual_run, speed->display.nr_display_last_run);
    if ((actual_run != speed->display.nr_display_last_run) && (speed->max_speed > 3000.0f)) { // 3m/s
        speed->display.nr_display_last_run = actual_run;
        speed->display.display_last_run_max_speed = 0;
    } 
    else if (speed->display.display_last_run_max_speed < speed->max_speed) {
        speed->display.display_last_run_max_speed = speed->max_speed;
    }
}

#if defined(MUTABLE_RUNS)
static inline void reset_runs_avg(gps_run_t runs[]) {
    // printf("[%s]\n", __func__);
    memset(runs, 0, NUM_OF_SPD_ARRAY_SIZE * sizeof(gps_run_t));
}
#endif

static inline void reset_display_speed(float * arr) {
    // printf("[%s]\n", __func__);
    memset(arr, 0, NUM_OF_SPD_ARRAY_SIZE * sizeof(float));
}

// static void update_avg_5runs(gps_speed_t * speed, bool mode) {
//     // printf("[%s]\n", __func__);
//     speed->avg_5runs = speed->runs[(mode ? 0 : 5)].avg_speed;
//     for(uint8_t i = 6, j = mode ? 9 : NUM_OF_SPD_ARRAY_SIZE; i < j; i++) speed->avg_5runs += speed->runs[i].avg_speed;
//     speed->avg_5runs /= 5;
// }

static inline void update_display_speed_array(gps_display_t * display, gps_run_t runs[], uint8_t start, uint8_t end) {
    // printf("[%s]\n", __func__);
    for (uint8_t i = start; i < end; i++) display->display_speed[i] = runs[i].avg_speed;
}

static inline void refresh_display_speeds(gps_display_t * display, gps_run_t runs[], float max_speed) {
    // printf("[%s]\n", __func__);
    display->display_speed[5] = max_speed;
    update_display_speed_array(display, runs, 6, NUM_OF_SPD_ARRAY_SIZE);
    sort_display(display->display_speed, NUM_OF_SPD_ARRAY_SIZE);
}

static uint8_t update_display_speeds(gps_speed_t * speed, uint8_t * record) {
    uint8_t ret = 0;
    if (speed->max_speed > speed->RUNS_FOR_DISPLAY[IDX_OF_SPD_ARRAY_MIN_SPD].avg_speed) {
        refresh_display_speeds(&speed->display, speed->RUNS_FOR_DISPLAY, speed->max_speed);
        ret = 1;
    }
    if (speed->max_speed > speed->RUNS_FOR_DISPLAY[IDX_OF_SPD_ARRAY_MAX_SPD].avg_speed) {
        speed->display.display_max_speed = speed->max_speed;  // update on the fly, that's not correct here !!!
        speed->display.record = 1;
        *record = 1;
        ret = 2;
    }
    else
        speed->display.display_max_speed = speed->RUNS_FOR_DISPLAY[IDX_OF_SPD_ARRAY_MAX_SPD].avg_speed;
    return ret;
}

static void reset_last_run_speeds(gps_speed_t * speed) {
    // update_avg_5runs(speed, false);  // calculate the average of the last 5 best runs
    speed->max_speed = 0;
    memset(&speed->runs[0], 0, sizeof(gps_run_t));
#if defined(MUTABLE_RUNS)
    memset(&speed->runs_mutable[0], 0, sizeof(gps_run_t));
#endif
    speed->display.record = 0;
}

static bool store_run_max_speed(gps_speed_t * speed, uint16_t run_count) {
    if (speed->max_speed < speed->cur_speed) {
        speed->max_speed = speed->cur_speed;
        store_time(&speed->runs[0]);
        speed->runs[0].nr = run_count;
        speed->runs[0].avg_speed = speed->max_speed;
#if defined(MUTABLE_RUNS)
        memcpy(&speed->runs_mutable[0], &speed->runs[0], sizeof(gps_run_t));
#endif
        return 1;
    }
    return 0;
}

#if defined(GPS_STATS)
static void gps_run_printf(const struct gps_run_s * run) {
    printf("Run: {time: %02d:%02d.%02d, avg_speed: %.02f, nr: %hu}\n", run->time.hour, run->time.minute, run->time.second, run->avg_speed, run->nr);
}
#endif

#if defined(GPS_STATS) && defined(GPS_TRACE_MSG_SPEED_BY_DIST)
static esp_err_t gps_speed_by_dist_printf(const struct gps_speed_by_dist_s *me) {
    uint8_t i, j=NUM_OF_SPD_ARRAY_SIZE;
    printf("=== speed_by_dist: {\n");
    printf("m_set_dist: %hu, ", me->distance_window);
    printf("m_dist: %ld, ", me->distance);
    printf("m_sample: %lu, ", me->m_sample);
    printf("m_index: %ld\n", me->m_index);
    gps_speed_printf(&me->speed);
    printf("dist: ");
    for (i = 0; i < j; i++) printf("%lu ", me->dist[i]);
    printf("\n");
    printf("nr_samples: ");
    for (i = 0; i < j; i++) printf("%lu ", me->nr_samples[i]);
    printf("\n");
    printf("message_nr: ");
    for (i = 0; i < j; i++) printf("%lu ", me->message_nr[i]);
    printf("\n");
    printf("} === \n");
    return ESP_OK;
}
#endif

/// Instance to determine the average speed over a certain distance, 
/// when a new run starts, save the highest speed of the previous run.
struct gps_speed_by_dist_s *init_gps_speed_by_distance(struct gps_speed_by_dist_s *me, uint16_t dist) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    memset(me, 0, sizeof(struct gps_speed_by_dist_s));
    me->distance_window = dist;
    return me;
}

#if defined(MUTABLE_RUNS)
void reset_speed_stats(struct gps_speed_by_dist_s *me) {
    reset_runs_avg(me->speed.RUNS_FOR_DISPLAY);
    reset_display_speed(me->speed.display.display_speed);
}
#endif

static inline bool store_speed_by_dist(struct gps_speed_by_dist_s *me) {
    // printf("[%s]\n", __func__);
    if (me->distance >= me->distance_window_raw) {
        me->m_sample = log_p_lctx.index_gspeed - me->m_index + 1;  // Check for the number of samples to avoid division by zero
        if (me->m_sample > 0) {
            // Calculate the distance in mm, so multiply by 1000 and consider the sample_rate !!
            me->speed.cur_speed = (float)me->distance / me->m_sample;  // 10 samples op 1s aan 10mm/s = 100/10 = 10 mm /s
        }
        // if (me->m_sample > 1) {
        //     // Calculate the speed based on the distance and the number of samples
        //     me->speed_alfa = me->distance / me->m_sample - 1;
        // }
    } else if(me->speed.cur_speed > 0) {
        me->speed.cur_speed = 0;
        // me->speed_alfa = 0;
    }
    return me->speed.cur_speed > 0;  // return the speed in mm/s
}

static inline void store_dist_data(struct gps_speed_by_dist_s *me) {
    if (store_run_max_speed(&me->speed, gps->run_count)) {  // store max speed of this run
        me->speed.runs[0].data.dist.dist = me->distance;
        me->speed.runs[0].data.dist.nr_samples = me->m_sample;
        me->speed.runs[0].data.dist.message_nr = gps->ubx_device->ubx_msg.count_nav_pvt;
    }
    update_display_speeds(&me->speed, &gps->record);
}

static inline void store_and_reset_dist_data_after_run(struct gps_speed_by_dist_s *me) {
#if (C_LOG_LEVEL < 2)
    ILOG(TAG, "[%s]", __func__);
#endif
    // sort_run_alfa(me->speed.runs, me->dist, me->message_nr, (uint32_t*)me->nr_samples, 10);
    sort_runs(
        me->speed.runs,
        (void *[]){0},
        (size_t[]){0},
        0, NUM_OF_SPD_ARRAY_SIZE
    );
#if defined(MUTABLE_RUNS)
    sort_runs(
        me->speed.RUNS_FOR_DISPLAY,
        (void *[]){0},
        (size_t[]){0},
        0, NUM_OF_SPD_ARRAY_SIZE
    );
#endif
    update_display_speed_array(&me->speed.display, me->speed.RUNS_FOR_DISPLAY, 0, NUM_OF_SPD_ARRAY_SIZE);
    reset_last_run_speeds(&me->speed);
}

static inline void move_distance_window(struct gps_speed_by_dist_s *me) {
    // printf("[%s]\n", __func__);
    // Determine buffer m_index for the desired distance
    if(me->distance > me->distance_window_raw) {
        while (me->distance > me->distance_window_raw && (log_p_lctx.index_gspeed - me->m_index) < log_p_lctx.buf_gspeed_size) {
            me->distance -= log_p_lctx.buf_gspeed[buf_index(me->m_index++)];
        }
        me->distance = me->distance + log_p_lctx.buf_gspeed[buf_index(--me->m_index)];
    }
}

float update_speed_by_distance(struct gps_speed_by_dist_s *me) {
    // printf("[%s]\n", __func__);
    if(!me) return 0.0f;
    // uint32_t distance_window = convert_distance_to_mm(me->distance_window, gps->ubx_device->rtc_conf->output_rate);  // Note that m_distance_window should now be in mm, so multiply by 1000 and consider the sample_rate !!
    me->distance = me->distance + log_p_lctx.buf_gspeed[buf_index(log_p_lctx.index_gspeed)];  // the resolution of the distance is 0.1 mm
                                                                                  // the max int32  is 2,147,483,647 mm eq 214,748.3647 meters eq ~214 kilometers !!
    if ((log_p_lctx.index_gspeed - me->m_index) >= log_p_lctx.buf_gspeed_size) {  // controle buffer overflow
        printf("[%s] buffer overflow, resetting index_gspeed\n", __func__);
        me->distance = 0;
        me->m_index = log_p_lctx.index_gspeed;
    }
    move_distance_window(me);
    // printf("[%s] dist: %.1f, set: %hu spd: %.1f, max: %0.1f\n", __func__, get_distance_m(me->distance, gps->ubx_device->rtc_conf->output_rate), me->distance_window, me->speed.runs[0].avg_speed, me->speed.max_speed);
    if(store_speed_by_dist(me)) {  // store the speed if it is greater than 0
        store_dist_data(me);  // store the data in the speed struct
    }
    if ((gps->run_count != me->speed.nr_prev_run) && (me->speed.runs[0].nr == me->speed.nr_prev_run)) {  // opslaan hoogste snelheid van run + sorteren
        store_and_reset_dist_data_after_run(me);
    }
    me->speed.nr_prev_run = gps->run_count;
    record_last_run(&me->speed, gps->run_count);
    return me->speed.max_speed;
}


/// Instance to determine the average speed over a certain time window
struct gps_speed_by_time_s *init_gps_speed_by_time(struct gps_speed_by_time_s *me, uint16_t window) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    memset(me, 0, sizeof(struct gps_speed_by_time_s));
    me->time_window = window;
    return me;
}

#if defined(GPS_STATS) && defined(GPS_TRACE_MSG_SPEED_BY_TIME)
#if defined(SPEED_BAR_SETUP)
static void gps_speed_bar_data_printf(const struct gps_speed_bar_s *bar) {
    printf("bar_data: {bar_count: %hu, run_speeds: ", bar->bar_count);
    for (uint8_t i = 0; i < NR_OF_BAR; i++) {
        printf("%.02f ", bar->run_speed[i]);
    }
    printf("}\n");
}
#endif
static esp_err_t gps_speed_by_time_printf(const struct gps_speed_by_time_s *me) {
    printf("=== gps_speed_by_time: {\n");
    printf("time_window: %hu, ", me->time_window);
    gps_speed_printf(&me->speed);
    // printf("avg_s_sum: %ld\n", me->avg_s_sum);
    // printf("Mean_cno: ");
    // for (i = 0; i < j; i++)
    //     printf("%hu ", me->Mean_cno[i]);
    // printf("\n");
    // printf("Max_cno: ");
    // for (i = 0; i < j; i++)
    // printf("%hhu ", me->Max_cno[i]);
    // printf("\n");
    // printf("Min_cno: ");
    // for (i = 0; i < j; i++)
    //     printf("%hhu ", me->Min_cno[i]);
    // printf("\n");
    // printf("Mean_numSat: ");
    // for (i = 0; i < j; i++)
    //     printf("%hhu ", me->Mean_numSat[i]);
    // printf("\n");
    printf("} === \n");
    return ESP_OK;
}
#endif
#if defined(MUTABLE_RUNS)
void reset_time_stats(struct gps_speed_by_time_s *me) {
    reset_runs_avg(me->speed.RUNS_FOR_DISPLAY);
    reset_display_speed(me->speed.display.display_speed);
}
#endif

#if defined(SPEED_BAR_SETUP)
static inline void store_speed_bar_data(struct gps_speed_by_time_s *me, uint16_t run_count) {
    if (me->speed.max_speed > 5000.0f) me->bar.bar_count++;  // min speed bar graph = 5 m/s
    me->bar.run_speed[run_count % NR_OF_BAR] = me->speed.runs[0].avg_speed;
}
#endif

static inline void store_speed_by_time_data(struct gps_speed_by_time_s *me) {
    // printf("[%s]\n", __func__);
    if (store_run_max_speed(&me->speed, gps->run_count)) {
        me->speed.runs[0].data.time.Mean_cno = gps->Ublox_Sat.sat_info.Mean_mean_cno;
        me->speed.runs[0].data.time.Max_cno = gps->Ublox_Sat.sat_info.Mean_max_cno;
        me->speed.runs[0].data.time.Min_cno = gps->Ublox_Sat.sat_info.Mean_min_cno;
        me->speed.runs[0].data.time.Mean_numSat = gps->Ublox_Sat.sat_info.Mean_numSV;
#if defined(SPEED_BAR_SETUP)
        me->bar.run_speed[gps->run_count % NR_OF_BAR] = me->speed.cur_speed;
#endif
    }
    if(update_display_speeds(&me->speed, &gps->record)) {
        // update_avg_5runs(&me->speed, true); // average of the runs 0 and 6-9
    }
}

static inline void store_and_reset_time_data_after_run(struct gps_speed_by_time_s *me) {
#if (C_LOG_LEVEL < 2)
    ILOG(TAG, "[%s]", __func__);
#endif
    sort_runs(
        me->speed.runs,
        (void *[]){0},
        (size_t[]){0},
        0, NUM_OF_SPD_ARRAY_SIZE
    );
#if defined(MUTABLE_RUNS)
    sort_runs(
        me->speed.RUNS_FOR_DISPLAY,
        (void *[]){0},
        (size_t[]){0},
        0, NUM_OF_SPD_ARRAY_SIZE
    );
#endif
    update_display_speed_array(&me->speed.display, me->speed.RUNS_FOR_DISPLAY, 0, NUM_OF_SPD_ARRAY_SIZE);  // update the runs array
#if defined(SPEED_BAR_SETUP)
    store_speed_bar_data(me, context->run_count);  // store the speed bar data
#endif
    reset_last_run_speeds(&me->speed);  // reset the speed for the next run
}

static inline bool store_avg_speed_by_time(struct gps_speed_by_time_s *me, uint32_t time_window_delta, uint8_t sample_rate) {
    // printf("[%s] %lu\n", __func__, time_window_delta);
    bool window_reached = false;
    if (time_window_delta < log_p_lctx.buf_gspeed_size) {  // if time window is smaller than the sample_rate*BUFFER, use normal buffer
        me->avg_s_sum += log_p_lctx.buf_gspeed[buf_index(log_p_lctx.index_gspeed)];  // always add gSpeed at every update
        if (log_p_lctx.index_gspeed >= time_window_delta) { // once 10s is reached, subtract -10s from the sum again
            me->avg_s_sum = me->avg_s_sum - log_p_lctx.buf_gspeed[buf_index(log_p_lctx.index_gspeed - time_window_delta)];
            window_reached = true;  // only if the time window is reached, we can calculate the speed
            me->speed.cur_speed = (float)me->avg_s_sum / me->time_window / sample_rate;
        }
    } else if (log_p_lctx.index_gspeed % sample_rate == 0) {  // switch to seconds buffer, but only one update per second !!
         me->avg_s_sum += log_p_lctx.buf_sec_speed[sec_buf_index(log_p_lctx.index_sec)];  // log_p_lctx.buf_sec_speed[SEC_BUFFER_SIZE] and log_p_lctx.index_sec
        if (log_p_lctx.index_sec >= me->time_window) { // once 10s is reached, subtract -10s from the sum again
            me->avg_s_sum = me->avg_s_sum - log_p_lctx.buf_sec_speed[sec_buf_index(log_p_lctx.index_sec - me->time_window)];
            window_reached = true;  // only if the time window is reached, we can calculate the speed
            me->speed.cur_speed = (float)me->avg_s_sum / me->time_window;  // in the seconds array is the average of gspeed !!
        }
    }
    if(!window_reached && me->speed.cur_speed > 0) {
        me->speed.cur_speed = 0;  // if the time window is not reached, set the speed to 0
    }
    return window_reached;  // return true if the time window is reached, so we can calculate the speed
}

float update_speed_by_time(struct gps_speed_by_time_s *me) {
    if(!me) return 0.0f;
    uint8_t sample_rate = gps->ubx_device->rtc_conf->output_rate;
    // uint32_t actual_run = gps->run_count;
    uint32_t time_window_delta = me->time_window * sample_rate;
    if(store_avg_speed_by_time(me, time_window_delta, sample_rate))
        store_speed_by_time_data(me);  // store the run data if the speed is higher than the previous run
    if ((gps->run_count != me->speed.nr_prev_run) && (me->speed.runs[0].nr == me->speed.nr_prev_run)) {  // sorting only if new max during this run !!!
        store_and_reset_time_data_after_run(me);  // sort the runs and update the display speed}
    }
    me->speed.nr_prev_run = gps->run_count;
    record_last_run(&me->speed, gps->run_count); 
    return me->speed.max_speed;  // anders compiler waarschuwing control reaches end of non-void function [-Werror=return-type]
}

struct gps_speed_alfa_s *init_gps_speed_by_alfa(struct gps_speed_by_dist_s *m) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    if (!m || !m->alfa) return NULL;  // protection against NULL pointer
    memset(m->alfa, 0, sizeof(struct gps_speed_alfa_s));
    m->alfa->distance_window = m->distance_window;
    m->alfa->base = m;
    return m->alfa;
}

#if defined(MUTABLE_RUNS)
void reset_alfa_stats(struct gps_speed_alfa_s *me) {
    reset_runs_avg(me->speed.RUNS_FOR_DISPLAY);
}
#endif

#if defined(GPS_STATS) && defined(GPS_TRACE_MSG_SPEED_ALPHA)
static esp_err_t gps_speed_by_alpha_printf(const struct gps_speed_alfa_s *me) {
    uint8_t i, j=NUM_OF_SPD_ARRAY_SIZE;
    printf("=== speed_by_alfa: {\n");
    printf("set_alfa_dist: %hu, ", me->distance_window);
    printf("straight_dist_square: %.03f\n", me->straight_dist_square);
    gps_speed_printf(&me->speed);
    printf("real_distance: ");
    for (i = 0; i < j; i++) printf("%ld ", me->real_distance[i]);
    printf("\n");
    printf("alfa_distance: ");
    for (i = 0; i < j; i++) printf("%lu ", me->alfa_distance[i]);
    printf("\n");
    printf("message_nr: ");
    for (i = 0; i < j; i++) printf("%lu ", me->message_nr[i]);
    printf("\n");
    printf("} === \n");
    return ESP_OK;
}
#endif

#define USE_HAVERSINE

#if defined(USE_HAVERSINE_DIST_TO_LINE) || defined(USE_HAVERSINE)
// Helper: Convert lat/lon (degrees) to 3D Cartesian coordinates on unit sphere
static void latlon_to_xyz(const gps_point_t *pt, double *x, double *y, double *z) {
    double lat_rad = pt->latitude * DEG2RAD;
    double lon_rad = pt->longitude * DEG2RAD;
    *x = cos(lat_rad) * cos(lon_rad);
    *y = cos(lat_rad) * sin(lon_rad);
    *z = sin(lat_rad);
}
#endif

#if defined(USE_HAVERSINE)

/**
 * @brief Calculates the Haversine distance between two GPS points.
 *
 * This function computes the distance between two points on the Earth's surface using the Haversine formula.
 * The function uses the mean radius of the Earth (6,371 km) to calculate the distance.
 * The Haversine formula is used to calculate the distance between two points on the surface of a sphere.
 * The result is a double value representing the distance in meters.
 *
 * @note The function uses the constant EARTH_RADIUS_M, which is defined as 6,371,000 meters (mean radius of the Earth).
 * @note The function uses the constant DEG2RAD to convert degrees to radians.
 *
 * @param p1 Pointer to the gps_point_t structure representing the first GPS point (latitude and longitude).
 * @param p2 Pointer to the gps_point_t structure representing the second GPS point (latitude and longitude).
 * @return The Haversine distance between the two GPS points in meters.
 */
// static inline double haversine_previous(gps_point_t * p1, gps_point_t * p2) {
//     float dlat = DEG2RAD * (p2->latitude -  p1->latitude);
//     float dlon = DEG2RAD * (p2->longitude - p1->longitude);
//     double a = POW_2(sin(dlat/2)) + cos(DEG2RAD * p1->latitude) * cos(DEG2RAD *  p2->latitude) * POW_2(sin(dlon/2));
//     double c = 2 * atan2(sqrt(a), sqrt(1-a));
//     return EARTH_RADIUS_M * c;
// }

static inline double straight_dist_haversine(const gps_point_t *p1, const gps_point_t *p2) {
    double x1, y1, z1, x2, y2, z2;
    latlon_to_xyz(p1, &x1, &y1, &z1);
    latlon_to_xyz(p2, &x2, &y2, &z2);

    // Dot product of unit vectors
    double dot = x1 * x2 + y1 * y2 + z1 * z2;
    // Clamp dot to [-1, 1] to avoid NaN due to floating point errors
    if (dot > 1.0) dot = 1.0;
    if (dot < -1.0) dot = -1.0;

    double angle = acos(dot); // angle in radians
    return EARTH_RADIUS_M * angle; // distance in meters
}

#else

/// @brief Calculates the straight-line distance squared between two GPS points.
/// This function computes the squared distance between two points on the Earth's surface using a simplified formula.
/// The formula is based on the difference in latitude and longitude, adjusted for the curvature of the Earth.
/// The result is a double value representing the squared distance in meters.
/// This function is used to determine if the distance between two points is less than a certain threshold.
/// @note The function uses the constant DEG2RAD to convert degrees to radians.
/// @note The function uses the constant DEG_TO_METERS to convert degrees to meters.
/// @param p1 Pointer to the gps_point_t structure representing the first GPS point (latitude and longitude).
/// @param p2 Pointer to the gps_point_t structure representing the second GPS point (latitude and longitude).
/// @return The squared straight-line distance between the two GPS points in meters.
static inline double straight_dist_square(gps_point_t * p1, gps_point_t * p2) {
    // now calculate the absolute distance alfa_speed::alfa_update(GPS_speed M)
    // between the starting point and the end point of the 250m distance,
    // if < 50m this is an alfa !!! note, this is calculated in meters,
    // therefore alfa_circle also in m !! was (M.m_index-1), should be (M.m_index+1)
    float dlat = p1->latitude - p2->latitude;
    float px = cos(DEG2RAD * p1->latitude) * (p1->longitude - p2->longitude);
    return DEG_TO_METERS(POW_2(dlat) + POW_2(px));
}
#endif

#if defined(USE_HAVERSINE_DIST_TO_LINE)
/**
 * @brief Haversine-based perpendicular distance from a point to a great-circle defined by two points.
 * @param act Pointer to the point to measure from.
 * @param p1 Pointer to the first point defining the line.
 * @param p2 Pointer to the second point defining the line.
 * @return The shortest distance from act to the great-circle through p1 and p2, in meters.
 */
static double point_to_line_distance(const gps_point_t *act, const gps_point_t *p1, const gps_point_t *p2) {
    // Convert all points to 3D Cartesian coordinates
    double x0, y0, z0, x1, y1, z1, x2, y2, z2;
    latlon_to_xyz(act, &x0, &y0, &z0);
    latlon_to_xyz(p1, &x1, &y1, &z1);
    latlon_to_xyz(p2, &x2, &y2, &z2);

    // Compute normal vector of the plane (great circle) defined by p1 and p2
    double nx = y1 * z2 - z1 * y2;
    double ny = z1 * x2 - x1 * z2;
    double nz = x1 * y2 - y1 * x2;
    double norm = sqrt(nx * nx + ny * ny + nz * nz);
    nx /= norm; ny /= norm; nz /= norm;

    // The perpendicular (angular) distance is arcsin(dot product of act and normal))
    double dot = x0 * nx + y0 * ny + z0 * nz;
    double angle = fabs(asin(dot)); // radians

    return EARTH_RADIUS_M * angle; // meters
}
#else
/*
/// previous code for point_to_line_distance, not used anymore, but kept for reference
float point_to_line_distance(float long_act, float lat_act, float long_1, float lat_1, float long_2, float lat_2) {
    float corr_lat=111120;
    float corr_long=111120*cos(DEG2RAD*buf_alfa[log_p_lctx.index_gspeed%BUFFER_ALFA]);
    lambda_T=(log_p_lctx.P2_lat-log_p_lctx.P1_lat)*(P_lat-log_p_lctx.P1_lat)*corr_lat*corr_lat+(log_p_lctx.P2_long-log_p_lctx.P1_long)*(P_long-log_p_lctx.P1_long)*corr_long*corr_long;
    lambda_N=pow((log_p_lctx.P2_lat-log_p_lctx.P1_lat)*corr_lat,2)+pow((log_p_lctx.P2_long-log_p_lctx.P1_long)*corr_long,2);
    lambda=lambda_T/lambda_N;
    alfa_dist=sqrt(
        pow((P_lat-log_p_lctx.P1_lat-lambda*(log_p_lctx.P2_lat-log_p_lctx.P1_lat))*corr_lat,2)+
        pow((P_long-log_p_lctx.P1_long-lambda*(log_p_lctx.P2_long-log_p_lctx.P1_long))*corr_long,2));
}
*/
/// Calculates distance from point with act lat/long to line which passes points lat_1/long_1 and lat_2/long_2
/// The result is the perpendicular distance from the point to the line in meters.
/// This function uses the curvature of the Earth to calculate the distance.
/// @note The function uses the constant METERS_PER_LATITUDE_DEGREE to convert latitude degrees to meters.
/// @note The function uses the constant DEG2RAD to convert degrees to radians.
/// @param act Pointer to the gps_point_t structure representing the current position (latitude and longitude).
/// @param p1 Pointer to the gps_point_t structure representing the first point on the line (latitude and longitude).
/// @param p2 Pointer to the gps_point_t structure representing the second point on the line (latitude and longitude).
/// @return The perpendicular distance from the point to the line in meters.
static float point_to_line_distance(gps_point_t * act,  gps_point_t * p1, gps_point_t * p2) {
    // printf("[%s]\n", __func__);
    float corr_lat = METERS_PER_LATITUDE_DEGREE;
    float corr_long = corr_lat * cos(DEG2RAD * act->latitude);  // meters per longitude degree, this is a function of latitude !
    float dlat = p2->latitude - p1->latitude;
    float dlong = p2->longitude - p1->longitude;
    float dlat_act = act->latitude - p1->latitude;
    float dlong_act = act->longitude - p1->longitude;
    float lambda_T = ((dlat * dlat_act) * POW_2(corr_lat)) + ((dlong * dlong_act) * POW_2(corr_long));
    float lambda_N = POW_2(dlat * corr_lat) + POW_2(dlong * corr_long);
    // float lambda = 0.0f;
    // if (lambda_N != 0.0f) {
    float lambda = lambda_T / lambda_N;
    // } else {
    //     // Avoid division by zero; return a large value or 0 as appropriate
    //     return 0.0f;
    // }
    float alfa_distance = sqrt(POW_2((dlat_act - lambda * dlat) * corr_lat) + POW_2((dlong_act - lambda * dlong) * corr_long));
// #if (C_LOG_LEVEL < 2)
//     printf("lambda_T: %.2f, lambda_N: %.2f, lambda: %.2f, adist: %.02f\n", lambda_T, lambda_N, lambda, alfa_distance);
// #endif
    return alfa_distance;
}

#endif

static inline void store_alfa_data(struct gps_speed_alfa_s *me, uint32_t dist) {
    // printf("[%s]\n", __func__);
    if (store_run_max_speed(&me->speed, gps->run_count)) {
        me->speed.runs[0].data.alfa.message_nr = gps->ubx_device->ubx_msg.count_nav_pvt;
        me->speed.runs[0].data.alfa.real_distance = (int32_t)me->straight_dist_square;
        me->speed.runs[0].data.alfa.dist = dist;
    }
    update_display_speeds(&me->speed, &gps->record);
}

static inline void store_and_reset_alfa_data_after_run(struct gps_speed_alfa_s *me) { 
#if (C_LOG_LEVEL < 2)
    ILOG(TAG, "[%s]", __func__);
#endif
    // sort_run_alfa(me->speed.runs, me->real_distance, me->message_nr, me->dist, 10);
    sort_runs(
        me->speed.runs,
        (void *[]){0},
        (size_t[]){0},
        0, NUM_OF_SPD_ARRAY_SIZE
    );
#if defined(MUTABLE_RUNS)
    sort_runs(
        me->speed.RUNS_FOR_DISPLAY,
        (void *[]){0},
        (size_t[]){0},
        0, NUM_OF_SPD_ARRAY_SIZE
    );
#endif
    reset_last_run_speeds(&me->speed);  // reset the speed for the next run
}

// Attention, here the distance traveled must be less than 500 m! Therefore, 
// an extra variable, m_speed_alfa, is provided in GPS_speed!!!
float update_speed_by_alfa(struct gps_speed_by_dist_s *m) {
    if(!m || !m->alfa) return 0.0f;
    struct gps_speed_alfa_s *me = m->alfa;
    // if (gps->Ublox.run_distance_after_turn < 375000.0f) {
#if defined(USE_HAVERSINE)
        me->straight_dist_square = straight_dist_haversine(
#else
        me->straight_dist_square = straight_dist_square(
#endif
            &log_p_lctx.alfa_buf[al_buf_index(log_p_lctx.index_gspeed)], 
            &log_p_lctx.alfa_buf[al_buf_index(m->m_index + 1)]
        );
#if defined(USE_HAVERSINE)
        if (me->straight_dist_square < 50.0f) {
#else
        if (me->straight_dist_square < POW_2(me->set_alfa_dist)) {
#endif
            me->speed.cur_speed = m->speed.cur_speed; // current speed in mm/s
            if (m->m_sample >= log_p_lctx.alfa_buf_size) {
                printf("Warning: m_sample %ld >= al_buf_size %hd, setting speed to 0\n", m->m_sample, log_p_lctx.alfa_buf_size);
                me->speed.cur_speed = 0;  // avoid overflow at low speeds
            }
            store_alfa_data(me, m->distance);
        }
        // printf("[%s] dist: %.1f, set: %hu spd: %.1f, max: %0.1f\n", __func__, get_distance_m(m->distance, gps->ubx_device->rtc_conf->output_rate), me->base->distance_window, me->speed.runs[0].avg_speed, me->speed.max_speed);
    // }
    // if((alfa_speed_max>0.0f)&(straight_dist_square>(alfa_circle_square*1.4))){ //alfa max only resets to 0 if 500 m after the jibe, straight distance after the jibe
    if (gps->run_count != me->speed.nr_prev_run) {
        store_and_reset_alfa_data_after_run(me);
    }
    me->speed.nr_prev_run = gps->run_count;
    record_last_run(&me->speed, gps->run_count); 
    return me->speed.max_speed;
}

static inline float unwrap_heading(float actual_heading, float *old_heading, float *delta_heading) {
    // printf("[%s]\n", __func__);
    if ((actual_heading - *old_heading) >  300.0f) *delta_heading -= 360.0f;
    if ((actual_heading - *old_heading) < -300.0f) *delta_heading += 360.0f;
    *old_heading = actual_heading;
    return actual_heading + *delta_heading;
}

static inline void detect_run_start_end(float S2_speed) {
    /// detection stand still, more then 2s with velocity < 1m/s 
    if (S2_speed > SPEED_DETECTION_MIN) log_p_lctx.velocity_5 = 1;  // 2s with speed over 4m/s
    if ((S2_speed < STANDSTILL_DETECTION_MAX) && (log_p_lctx.velocity_5)) log_p_lctx.velocity_0 = 1; // stopping detected, 2s with speed < 1 m/s
    else log_p_lctx.velocity_0 = 0;
    /// New run detected due to standstill
    if (log_p_lctx.velocity_0) {
        log_p_lctx.velocity_5 = 0;
        log_p_lctx.delay_count_before_run = 0;
    }
}

/* Calculation of the average heading over the last 10 seconds 
************************************************************************/
static inline bool is_straight_course(float heading_diff) {
    return (heading_diff < STRAIGHT_COURSE_MAX_DEV);
}

static inline bool detect_jibe(float heading_diff) {
    return heading_diff > JIBE_COURSE_DEVIATION_MIN && log_p_lctx.straight_course;
}

uint32_t new_run_detection(gps_context_t *context, float actual_heading, float S2_speed) {
    // printf("[%s]\n", __func__);
    if(!context) return 0;  // return 0 if context is NULL
    ubx_config_t *ubx = context->ubx_device;
    uint8_t sample_rate = ubx->rtc_conf->output_rate;

    log_p_lctx.heading = unwrap_heading(actual_heading, &log_p_lctx.old_heading, &log_p_lctx.delta_heading);    
    /// detect heading change over 15s is more than 40°, a new run is started
    uint16_t mean_heading_delta_time = MEAN_HEADING_TIME * sample_rate;
    log_p_lctx.heading_mean = log_p_lctx.heading_mean * (mean_heading_delta_time - 1) / (mean_heading_delta_time) + log_p_lctx.heading / (mean_heading_delta_time);
    
    /// detection stand still, more then 2s with velocity < 1m/s 
    detect_run_start_end(S2_speed);

    /// New run detected due to heading change
    float heading_diff = fabs(log_p_lctx.heading_mean - log_p_lctx.heading);
    if(is_straight_course(heading_diff) && log_p_lctx.velocity_5) {
        // printf("Straight course detected, heading_diff: %.02f\n", heading_diff);
        if(!log_p_lctx.straight_course) log_p_lctx.straight_course = 1;  // straight course detected, set straight_course to true
    }
    if(detect_jibe(heading_diff)) {
// #if (C_LOG_LEVEL < 2)
        printf("Jibe detected, heading_diff: %.02f\n", heading_diff);
// #endif
        if(gps->skip_alfa_after_stop) gps->skip_alfa_after_stop = 0;  // reset the skip alfa after stop counter
        if(log_p_lctx.straight_course) log_p_lctx.straight_course = 0;  // jibe detected, straight course is false
        log_p_lctx.delay_count_before_run = 0;
        context->alfa_count++;  // jibe detection for alfa_indicator ....
    }
    log_p_lctx.delay_count_before_run++;
    if (log_p_lctx.delay_count_before_run == (TIME_DELAY_NEW_RUN * sample_rate)) {
        ++context->run_count;
// #if (C_LOG_LEVEL < 2)
//         WLOG(TAG, "=== Run finished, count changed to %hu ===", context->run_count);
// #endif
    }
// #if (C_LOG_LEVEL < 2)
//     printf("delay_count_before_run: %lu, start_cond: %u ...\n", log_p_lctx.delay_count_before_run, (TIME_DELAY_NEW_RUN * sample_rate));
//     printf("run_count: %hu, context->alfa_count: %hu, log_p_lctx.straight_course: %d\n", context->run_count, context->alfa_count, log_p_lctx.straight_course);
// #endif
    return context->run_count;
}

// #define USE_LONG_POINTS  // define this to use the long points for jibe detection, otherwise the short points are used
static inline void update_jibe_reference_points(int32_t m_1, int32_t m_2, gps_point_t * buf, gps_point_t *p1, gps_point_t *p2) {
    // printf("[%s]\n", __func__);
    // this is the point at -250 m from the current position (speed extrapolation from -500m)
    p1->latitude  = buf[m_1].latitude;
    p1->longitude = buf[m_1].longitude;
    // this is the point at -100 m from the current position (speed extrapolation from -250m)
    p2->latitude  = buf[m_2].latitude;
    p2->longitude = buf[m_2].longitude;
}

/* Here the current "alfa distance" is calculated based on 2 points for the jibe: 
 P1 = 250m and P2 = 100m for the jibe. These points determine an imaginary line, 
 the perpendicular distance to the current position must be less than 50m/s
 when point P1 is passed.
 */
float alfa_indicator(float actual_heading) {
    // Update jibe reference points if needed
    if (gps->alfa_count != log_p_lctx.old_alfa_count) {
        // the distance traveled since the jibe detection 10*100.000/10.000=100 samples ?
        gps->Ublox.run_distance_after_turn = 0;
        update_jibe_reference_points(
            al_buf_index(gps->speed_metrics[dist_250m].handle.dist->m_index),
            al_buf_index(gps->speed_metrics[dist_100m].handle.dist->m_index),
            log_p_lctx.alfa_buf,
            &log_p_lctx.alfa_p1,
            &log_p_lctx.alfa_p2
        );
        log_p_lctx.old_alfa_count = gps->alfa_count;
    }
    // Current position
    int32_t idx_cur = al_buf_index(log_p_lctx.index_gspeed);
    gps_point_t cur = { log_p_lctx.alfa_buf[idx_cur].latitude, log_p_lctx.alfa_buf[idx_cur].longitude};

    // Position 2 seconds ago
    int32_t idx_prev = al_buf_index(log_p_lctx.index_gspeed - (2 * gps->ubx_device->rtc_conf->output_rate));
    gps_point_t prev = {log_p_lctx.alfa_buf[idx_prev].latitude, log_p_lctx.alfa_buf[idx_prev].longitude};

    gps->alfa_exit = point_to_line_distance(&log_p_lctx.alfa_p1, &cur, &prev); // turn-250m point distance to line {cur,cur-2sec}
    gps->alfa_window = point_to_line_distance(&cur, &log_p_lctx.alfa_p1, &log_p_lctx.alfa_p2); // cur point distance to line {turn-m250,turn-m100}
#if (C_LOG_LEVEL < 3)
    printf("[%s] run: %hu, exit: %.1f, window: %.1f, atdist: %.1f\n", __func__, 
            gps->run_count, gps->alfa_exit, 
            gps->alfa_window, 
            (float)(MM_TO_M(gps->Ublox.run_distance_after_turn)));
#endif
    return gps->alfa_window;  // current perpendicular distance relative to the line P2-P1, may be max 50m for a valid alfa !!
}

/*Heading tov reference***********************************************************************************
// Calculate reference heading (in degrees) from P2 to P1.
// This is tested and works correctly, but is +180° compared to the actual direction.
ref_heading = atan2(((P1_long - P2_long) * corr_long), ((P1_lat - P2_lat) * corr_lat)) * 180 / PI;
 if(ref_heading<0)ref_heading=ref_heading+360; // atan2 returns a value between -PI and +PI radians.
 delta_heading=(int)(actual_heading-ref_heading*180/PI)%360; // due to P1-P2, this is the opposite direction from travelling ! if(delta_heading>180)
 delta_heading=delta_heading-360; if(delta_heading<-180)
 delta_heading=delta_heading+360;
 */
