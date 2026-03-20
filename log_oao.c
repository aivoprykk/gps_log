#include "log_private.h"
#if (defined(CONFIG_UBLOX_ENABLED) && defined(CONFIG_GPS_LOG_ENABLED) && defined(GPS_LOG_HAS_OAO))

#include "oao.h"
#include "gps_log_file.h"
#include "ubx_msg.h"
#include "ubx.h"

enum {
    OAO_MODE_GNSS_ALIGNED = 0x0AD4,
    OAO_MODE_GNSS_UNALIGNED = 0x0AD5,
    OAO_GNSS_FRAME_LENGTH = 52,
};

static void oao_finalize_checksum(uint16_t length, uint8_t *buffer);
static int64_t oao_days_from_civil(int32_t year, uint32_t month, uint32_t day);
static uint64_t oao_utc_ms_from_nav_pvt(const nav_pvt_t *nav_pvt,
                                        int32_t *normalized_millis);

void log_header_OAO(struct gps_context_s *context) {
    (void)context;
}

void log_OAO(struct gps_context_s * context) {
    if (NOOAO || !context || !context->ubx_device) {
        return;
    }

    const ubx_msg_t *ubx_message = &context->ubx_device->ubx_msg;
    const nav_pvt_t *nav_pvt = &ubx_message->navPvt;
    union OAO_Frame frame = {0};
    int32_t normalized_millis = 0;

    frame.mode = (uint16_t)(oao_utc_ms_from_nav_pvt(nav_pvt,
                                                    &normalized_millis)
                            && normalized_millis == 0
                                ? OAO_MODE_GNSS_ALIGNED
                                : OAO_MODE_GNSS_UNALIGNED);
    frame.latitude = nav_pvt->lat;
    frame.longitude = nav_pvt->lon;
    frame.altitude = nav_pvt->hMSL;
    frame.speed = (uint32_t)(nav_pvt->gSpeed < 0 ? -nav_pvt->gSpeed
                                                 : nav_pvt->gSpeed);
    frame.heading = (uint32_t)(nav_pvt->heading < 0 ? 0 : nav_pvt->heading);
    frame.utc_gnss = oao_utc_ms_from_nav_pvt(nav_pvt, &normalized_millis);
    frame.fix = nav_pvt->fixType;
    frame.satellites = nav_pvt->numSV;
    frame.accuracy_speed = nav_pvt->sAcc;
    frame.accuracy_horizontal = nav_pvt->hAcc;
    frame.accuracy_vertical = nav_pvt->vAcc;
    frame.accuracy_heading = nav_pvt->headingAcc;
    frame.accuracy_hDOP = ubx_message->navDOP.hDOP;

    oao_finalize_checksum(OAO_GNSS_FRAME_LENGTH, frame.bytes_gnss);
    WRITEOAO(frame.bytes_gnss, sizeof(frame.bytes_gnss));

}

static void oao_finalize_checksum(uint16_t length, uint8_t *buffer) {
    uint8_t checksum_a = 0;
    uint8_t checksum_b = 0;

    for (uint16_t i = 0; i < 2; ++i) {
        checksum_b += (checksum_a += buffer[i]);
    }

    for (uint16_t i = 4; i < length; ++i) {
        checksum_b += (checksum_a += buffer[i]);
    }

    buffer[2] = checksum_a;
    buffer[3] = checksum_b;
}

static int64_t oao_days_from_civil(int32_t year, uint32_t month, uint32_t day) {
    year -= month <= 2U;
    const int32_t era = (year >= 0 ? year : year - 399) / 400;
    const uint32_t year_of_era = (uint32_t)(year - era * 400);
    const uint32_t month_prime = month + (month > 2U ? (uint32_t)-3 : 9U);
    const uint32_t day_of_year =
        (153U * month_prime + 2U) / 5U + day - 1U;
    const uint32_t day_of_era =
        year_of_era * 365U + year_of_era / 4U - year_of_era / 100U + day_of_year;

    return (int64_t)era * 146097LL + (int64_t)day_of_era - 719468LL;
}

static uint64_t oao_utc_ms_from_nav_pvt(const nav_pvt_t *nav_pvt,
                                        int32_t *normalized_millis) {
    uint32_t year = nav_pvt->year;
    uint8_t month = nav_pvt->month;
    uint8_t day = nav_pvt->day;
    uint8_t hour = nav_pvt->hour;
    uint8_t minute = nav_pvt->minute;
    uint8_t second = nav_pvt->second;
    int32_t millis = c_nano_to_millis_round(nav_pvt->nano);

    c_normalize_utc_fields(&year, &month, &day, &hour, &minute, &second,
                           &millis, 1000U);
    if (normalized_millis) {
        *normalized_millis = millis;
    }

    if (year < 1970U || month == 0U || day == 0U) {
        return 0;
    }

    const int64_t days = oao_days_from_civil((int32_t)year, month, day);
    const int64_t seconds = days * 86400LL + (int64_t)hour * 3600LL
                            + (int64_t)minute * 60LL + (int64_t)second;

    if (seconds < 0) {
        return 0;
    }

    return (uint64_t)seconds * 1000ULL + (uint64_t)millis;
}

#endif