#include "log_private.h"
#if (defined(CONFIG_UBLOX_ENABLED) && defined(CONFIG_GPS_LOG_ENABLED))

#include "gpx.h"
#include "gps_log_file.h"
#include "numstr.h"
#include "strbf.h"
#include "ubx_msg.h"
#include "ubx.h"

//static const char* TAG = "gpx";

//extern struct UBXMessage ubxMessage;

static void gpx_put_zero_padded_u(strbf_t *sb, uint32_t value, uint8_t width) {
    uint8_t digits = 1;
    uint32_t tmp = value;

    while (tmp >= 10U) {
        tmp /= 10U;
        ++digits;
    }

    while (digits < width) {
        strbf_putc(sb, '0');
        ++digits;
    }
    strbf_putul(sb, value);
}

static void gpx_put_coordinate(strbf_t *sb, int32_t value) {
    int64_t abs_value = value;

    if (abs_value < 0) {
        strbf_putc(sb, '-');
        abs_value = -abs_value;
    }
    strbf_putul(sb, (uint64_t)(abs_value / 10000000LL));
    strbf_putc(sb, '.');
    gpx_put_zero_padded_u(sb, (uint32_t)(abs_value % 10000000LL), 7);
}

static void gpx_put_timestamp(strbf_t *sb, int year, uint8_t month,
                              uint8_t day, uint8_t hour, uint8_t minute,
                              uint8_t second) {
    if (!sb || !sb->cur) {
        return;
    }
    if (sb->max && (size_t)(sb->max - sb->cur) < 20U) {
        return;
    }

    sb->cur += date_to_char(day, month, year, 1, sb->cur);
    *sb->cur++ = 'T';
    sb->cur += time_to_char_hms(hour, minute, second, sb->cur);
    *sb->cur++ = 'Z';
}

static void gpx_put_fixed_u(strbf_t *sb, uint32_t whole, uint32_t fraction,
                            uint8_t fraction_width) {
    strbf_putul(sb, whole);
    strbf_putc(sb, '.');
    gpx_put_zero_padded_u(sb, fraction, fraction_width);
}

void log_header_GPX(struct gps_context_s *context) {
    char bufferTx[384];
    const char *version = "unknown";
    size_t version_len = 0;
    strbf_t sb;

    if (NOGPX)
        return;
    if (context && context->SW_version && context->SW_version[0] != '\0') {
        version = context->SW_version;
    }
    version_len = strnlen(version, 31U);

    strbf_inits(&sb, bufferTx, sizeof(bufferTx));
    strbf_puts(&sb,
               "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>\n");
    strbf_puts(&sb, "<gpx version=\"1.0\" creator=\"ESP-GPS SW ");
    strbf_put(&sb, version, version_len);
    strbf_puts(&sb, "\"\nxmlns=\"http://www.topografix.com/GPX/1/0\"\n");
    strbf_puts(&sb,
               "xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\n");
    strbf_puts(&sb,
               "xsi:schemaLocation=\"http://www.topografix.com/GPX/1/0 ");
    strbf_puts(&sb, "http://www.topografix.com/GPX/1/0/gpx.xsd\">\n");
    strbf_puts(&sb, "<trk>\n<trkseg>\n");
    WRITEGPX(strbf_finish(&sb), (size_t)(sb.cur - sb.start));
}

void log_GPX(struct gps_context_s * context) {
    if(NOGPX)
        return;
    const struct ubx_msg_s *ubxMessage = &context->ubx_device->ubx_msg;
    char bufferTx[384];
    strbf_t sb;
    uint32_t gps_year = ubxMessage->navPvt.year;
    uint8_t gps_month = ubxMessage->navPvt.month;
    uint8_t gps_day = ubxMessage->navPvt.day;
    uint8_t gps_hour = ubxMessage->navPvt.hour;
    uint8_t gps_minute = ubxMessage->navPvt.minute;
    uint8_t gps_second = ubxMessage->navPvt.second;
    int32_t gps_millis = c_nano_to_millis_round(ubxMessage->navPvt.nano);

    c_normalize_utc_fields(&gps_year, &gps_month, &gps_day, &gps_hour,
                           &gps_minute, &gps_second, &gps_millis, 1000U);

    if (gps_millis == 0) {  // only log every normalized full second
        // Use integer math to avoid expensive float sprintf
        int32_t lat_i = ubxMessage->navPvt.lat;
        int32_t lon_i = ubxMessage->navPvt.lon;
        int32_t hMSL_mm = ubxMessage->navPvt.hMSL;
        int32_t heading_e5 = ubxMessage->navPvt.heading;
        int32_t gSpeed_mms = ubxMessage->navPvt.gSpeed;
        uint16_t hDOP_e2 = ubxMessage->navDOP.hDOP;
        int sat = ubxMessage->navPvt.numSV;
        // elevation: mm → meters (integer)
        int32_t ele_m = hMSL_mm / 1000;

        // course: 1e-5 deg → integer degrees
        int32_t course_deg = heading_e5 / 100000;

        // hdop: 1e-2 → value with 2 decimals
        uint32_t speed_abs = (uint32_t)(gSpeed_mms < 0 ? -gSpeed_mms : gSpeed_mms);

        strbf_inits(&sb, bufferTx, sizeof(bufferTx));
        strbf_puts(&sb, "<trkpt lat=\"");
        gpx_put_coordinate(&sb, lat_i);
        strbf_puts(&sb, "\" lon=\"");
        gpx_put_coordinate(&sb, lon_i);
        strbf_puts(&sb, "\"><ele>");
        strbf_putl(&sb, ele_m);
        strbf_puts(&sb, "</ele><time>");
        gpx_put_timestamp(&sb, (int)gps_year, gps_month, gps_day, gps_hour,
                          gps_minute, gps_second);
        strbf_puts(&sb, "</time><course>");
        strbf_putl(&sb, course_deg);
        strbf_puts(&sb, "</course><speed>");
        gpx_put_fixed_u(&sb, speed_abs / 1000U, (speed_abs % 1000U) / 10U,
                        2);
        strbf_puts(&sb, "</speed><sat>");
        strbf_putl(&sb, sat);
        strbf_puts(&sb, "</sat><hdop>");
        gpx_put_fixed_u(&sb, hDOP_e2 / 100U, hDOP_e2 % 100U, 2);
        strbf_puts(&sb, "</hdop></trkpt>\n");
        WRITEGPX(strbf_finish(&sb), (size_t)(sb.cur - sb.start));
    }
}

void log_footer_GPX(struct gps_context_s *context) {
    char bufferTx[32];
    strbf_t sb;

    (void)context;
    if (NOGPX)
        return;

    strbf_inits(&sb, bufferTx, sizeof(bufferTx));
    strbf_puts(&sb, "</trkseg>\n</trk>\n</gpx>\n");
    WRITEGPX(strbf_finish(&sb), (size_t)(sb.cur - sb.start));
}

#endif