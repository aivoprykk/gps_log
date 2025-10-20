#include "log_private.h"
#include "ubx_msg.h"

static const char * TAG = "gps_satellite";

#if defined(GPS_STATS)
#if defined(GPS_TRACE_MSG_SAT)
static esp_err_t gps_sat_info_printf(const struct gps_sat_info_s *me) {
    printf("GPS_SAT_info:{ ");
    printf("mean_cno: %hu, ", me->mean_cno);
    printf("min_cno: %hhu, ", me->min_cno);
    printf("max_cno: %hhu, ", me->max_cno);
    printf("nr_sats: %hhu, ", me->nr_sats);
    printf("index_sat_info: %lu, ", me->index_sat_info);
    printf("Mean_mean_cno: %hu, ", me->sat_info.Mean_mean_cno);
    printf("Mean_max_cno: %"PRIu8", ", me->sat_info.Mean_max_cno);
    printf("Mean_min_cno: %hhu, ", me->sat_info.Mean_min_cno);
    printf("Mean_numSV: %hhu, ", me->sat_info.Mean_numSV);
    printf("}\n");
    return ESP_OK;
}
#endif
#endif

// constructor for SAT_info
struct gps_sat_info_s *init_gps_sat_info(struct gps_sat_info_s *me) {
    FUNC_ENTRY(TAG);
    memset(me, 0, sizeof(struct gps_sat_info_s));
    me->index_sat_info = 0;
    return me;
}

// function to extract info out of NAV_SAT, and push it to array
// For every NAV_SAT frame, the Mean CNO, the Max cno, the Min cno and the nr of
// sats in the nav solution are stored Then, the means are calculated out of the
// last NAV_SAT_BUFFER frames (now 16 frames, @5Hz, this 0.5Hz NAV_SAT ca 32 s)
void push_gps_sat_info(struct gps_sat_info_s *me, struct nav_sat_s *nav_sat) {
    // #define NAV_SAT_BUFFER 10
    me->mean_cno = 0;
    me->min_cno = 0xFF;
    me->max_cno = 0;
    me->nr_sats = 0;
    for (uint8_t i = 0; i < nav_sat->numSvs; i++) {  // only evaluate nr of Sats that is in NAV_SAT
        if (nav_sat->sat[i].flags & 0x8) {  // only evaluate nr of Sats who are in nav solution, bit3 from X4
            me->mean_cno = me->mean_cno + nav_sat->sat[i].cno;
            if (nav_sat->sat[i].cno < me->min_cno)
                me->min_cno = nav_sat->sat[i].cno;
            if (nav_sat->sat[i].cno > me->max_cno)
                me->max_cno = nav_sat->sat[i].cno;
            me->nr_sats++;
        }
    }
    if (me->nr_sats) {  // protection divide int/0 !!
        me->mean_cno = me->mean_cno / me->nr_sats;
        uint32_t idx = me->index_sat_info % NAV_SAT_BUFFER;
        me->sat_info.Mean_cno[idx] = me->mean_cno;
        me->sat_info.Max_cno[idx] = me->max_cno;
        me->sat_info.Min_cno[idx] = me->min_cno;
        me->sat_info.numSV[idx] = me->nr_sats;
        me->mean_cno = 0;
        me->min_cno = 0;
        me->max_cno = 0;
        me->nr_sats = 0;
        if (me->index_sat_info > NAV_SAT_BUFFER) {
            for (uint8_t i = 0; i < NAV_SAT_BUFFER; i++) {
                idx = (me->index_sat_info - NAV_SAT_BUFFER + i) % NAV_SAT_BUFFER;
                me->mean_cno = me->mean_cno + me->sat_info.Mean_cno[idx];
                me->max_cno = me->max_cno + me->sat_info.Max_cno[idx];
                me->min_cno = me->min_cno + me->sat_info.Min_cno[idx];
                me->nr_sats = me->nr_sats + me->sat_info.numSV[idx];
            }
            me->mean_cno = me->mean_cno / NAV_SAT_BUFFER;
            me->max_cno = me->max_cno / NAV_SAT_BUFFER;
            me->min_cno = me->min_cno / NAV_SAT_BUFFER;
            me->nr_sats = me->nr_sats / NAV_SAT_BUFFER;
            me->sat_info.Mean_mean_cno = me->mean_cno;
            me->sat_info.Mean_max_cno = me->max_cno;
            me->sat_info.Mean_min_cno = me->min_cno;
            me->sat_info.Mean_numSV = me->nr_sats;
        }
        me->index_sat_info++;
    }
};
