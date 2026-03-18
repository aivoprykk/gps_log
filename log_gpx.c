#include "log_private.h"
#if (defined(CONFIG_UBLOX_ENABLED) && defined(CONFIG_GPS_LOG_ENABLED))

#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/unistd.h>

#include "gpx.h"
#include "ubx_msg.h"
#include "gps_log_file.h"
#include "gps_data.h"
#include "ubx.h"

//static const char* TAG = "gpx";

//extern struct UBXMessage ubxMessage;

void log_GPX(struct gps_context_s * context, int part) {
    if(NOGPX)
        return;
    const struct ubx_msg_s *ubxMessage = &context->ubx_device->ubx_msg;
    char bufferTx[384];
    int y;
    const int bufsz = sizeof(bufferTx);
    if (part == GPX_HEADER) {
        y = 0;
        y += snprintf(&bufferTx[y], bufsz - y,
            "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>\n"
            "<gpx version=\"1.0\" creator=\"ESP-GPS SW 5.75\"\n"
            "xmlns=\"http://www.topografix.com/GPX/1/0\"\n"
            "xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\n"
            "xsi:schemaLocation=\"http://www.topografix.com/GPX/1/0 "
            "http://www.topografix.com/GPX/1/0/gpx.xsd\">\n"
            "<trk>\n<trkseg>\n");
        WRITEGPX(bufferTx, y * sizeof(uint8_t));
    }
    if (part == GPX_FRAME) {
        if (ubxMessage->navPvt.nano / 1000000 == 0) {  // only log every full second !!!!
            // Use integer math to avoid expensive float sprintf
            int32_t lat_i = ubxMessage->navPvt.lat;
            int32_t lon_i = ubxMessage->navPvt.lon;
            int32_t hMSL_mm = ubxMessage->navPvt.hMSL;
            int32_t heading_e5 = ubxMessage->navPvt.heading;
            int32_t gSpeed_mms = ubxMessage->navPvt.gSpeed;
            uint16_t hDOP_e2 = ubxMessage->navDOP.hDOP;
            int sat = ubxMessage->navPvt.numSV;
            int year = (int)ubxMessage->navPvt.year;
            int month = (int)ubxMessage->navPvt.month;
            int day = (int)ubxMessage->navPvt.day;
            int hour = (int)ubxMessage->navPvt.hour;
            int minute = (int)ubxMessage->navPvt.minute;
            int sec = (int)ubxMessage->navPvt.second;

            // lat/lon: divide by 10^7, print 7 decimals via integer split
            int32_t lat_deg = lat_i / 10000000;
            int32_t lat_frac = lat_i % 10000000;
            if (lat_frac < 0) lat_frac = -lat_frac;
            int32_t lon_deg = lon_i / 10000000;
            int32_t lon_frac = lon_i % 10000000;
            if (lon_frac < 0) lon_frac = -lon_frac;

            // elevation: mm → meters (integer)
            int32_t ele_m = hMSL_mm / 1000;

            // course: 1e-5 deg → integer degrees
            int32_t course_deg = heading_e5 / 100000;

            // speed: mm/s → m/s with 2 decimals (integer split)
            int32_t speed_int = gSpeed_mms / 1000;
            int32_t speed_frac = (gSpeed_mms % 1000) / 10;
            if (speed_frac < 0) speed_frac = -speed_frac;

            // hdop: 1e-2 → value with 2 decimals
            int32_t hdop_int = hDOP_e2 / 100;
            int32_t hdop_frac = hDOP_e2 % 100;

            y = 0;
            y += snprintf(&bufferTx[y], bufsz - y,
                "<trkpt lat=\"%s%"PRId32".%07"PRId32"\" lon=\"%s%"PRId32".%07"PRId32"\">",
                lat_i < 0 ? "-" : "", lat_deg < 0 ? -lat_deg : lat_deg, lat_frac,
                lon_i < 0 ? "-" : "", lon_deg < 0 ? -lon_deg : lon_deg, lon_frac);
            y += snprintf(&bufferTx[y], bufsz - y,
                "<ele>%"PRId32"</ele>", ele_m);
            y += snprintf(&bufferTx[y], bufsz - y,
                "<time>%d-%02d-%02dT%02d:%02d:%02dZ</time>",
                year, month, day, hour, minute, sec);
            y += snprintf(&bufferTx[y], bufsz - y,
                "<course>%"PRId32"</course>", course_deg);
            y += snprintf(&bufferTx[y], bufsz - y,
                "<speed>%"PRId32".%02"PRId32"</speed>",
                speed_int < 0 ? -speed_int : speed_int, speed_frac);
            y += snprintf(&bufferTx[y], bufsz - y,
                "<sat>%d</sat>", sat);
            y += snprintf(&bufferTx[y], bufsz - y,
                "<hdop>%"PRId32".%02"PRId32"</hdop>",
                hdop_int, hdop_frac);
            y += snprintf(&bufferTx[y], bufsz - y,
                "</trkpt>\n");
            WRITEGPX(bufferTx, y * sizeof(uint8_t));
        }
    }
    if (part == GPX_END) {
        y = 0;
        y += snprintf(&bufferTx[y], bufsz - y,
            "</trkseg>\n</trk>\n</gpx>\n");
        WRITEGPX(bufferTx, y * sizeof(uint8_t));
    }
}

#endif