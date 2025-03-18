#include "log_private.h"
#if (defined(CONFIG_UBLOX_ENABLED) && defined(CONFIG_GPS_LOG_ENABLED))

#include <errno.h>
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
    char bufferTx[512];
    int i, y;
    int year, month, day, hour, minute, sec, sat;
    if (part == GPX_HEADER) {
        y = 0;
        i = sprintf( &bufferTx[y], "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>\n");
        y = y + i;
        i = sprintf(&bufferTx[y], "<gpx version=\"1.0\" creator=\"ESP-GPS SW 5.75\"\n");
        y = y + i;
        i = sprintf(&bufferTx[y], "xmlns=\"http://www.topografix.com/GPX/1/0\"\n");
        y = y + i;
        i = sprintf(&bufferTx[y],
            "xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\n");
        y = y + i;
        i = sprintf(&bufferTx[y],
                    "xsi:schemaLocation=\"http://www.topografix.com/GPX/1/0 "
                    "http://www.topografix.com/GPX/1/0/gpx.xsd\">\n");
        y = y + i;
        i = sprintf(&bufferTx[y], "<trk>\n");
        y = y + i;
        i = sprintf(&bufferTx[y], "<trkseg>\n");
        y = y + i;
        WRITEGPX(bufferTx, y * sizeof(uint8_t));
    }
    if (part == GPX_FRAME) {
        if (ubxMessage->navPvt.nano / 1000000 == 0) {  // only log every full second !!!!
            float lat, lon, hdop, speed, msl, course;
            lat = ubxMessage->navPvt.lat / 10000000.0f;
            lon = ubxMessage->navPvt.lon / 10000000.0f;
            hdop = ubxMessage->navDOP.hDOP /
                   100.0f;  // resolution in ubx nav dop is 0.01
            course = ubxMessage->navPvt.heading / 100000.0f;
            speed = ubxMessage->navPvt.gSpeed / 1000.0f;
            msl = ubxMessage->navPvt.hMSL / 1000.0f;
            sat = ubxMessage->navPvt.numSV;
            speed = ubxMessage->navPvt.gSpeed / 1000.0f;
            year = ubxMessage->navPvt.year;
            month = ubxMessage->navPvt.month;
            day = ubxMessage->navPvt.day;
            hour = ubxMessage->navPvt.hour;
            minute = ubxMessage->navPvt.minute;
            sec = ubxMessage->navPvt.second;
            y = 0;
            i = sprintf(&bufferTx[y], "<trkpt lat=\"%.7f\" lon=\"%.7f\">", lat, lon);
            y = y + i;
            i = sprintf(&bufferTx[y], "<ele>%.0f</ele>", msl);
            y = y + i;  // was float !!!
            i = sprintf(&bufferTx[y],
                        "<time>%d-%'02d-%'02dT%'02d:%'02d:%'02dZ</time>", year, month, day, hour, minute, sec);
            y = y + i;
            i = sprintf(&bufferTx[y], "<course>%.0f</course>", course);
            y = y + i;
            i = sprintf(&bufferTx[y], "<speed>%.2f</speed>", speed);
            y = y + i;
            i = sprintf(&bufferTx[y], "<sat>%d</sat>", sat);
            y = y + i;
            i = sprintf(&bufferTx[y], "<hdop>%.2f</hdop>", hdop);
            y = y + i;
            i = sprintf(&bufferTx[y], "</trkpt>\n");
            y = y + i;
            WRITEGPX(bufferTx, y * sizeof(uint8_t));
        }
    }
    if (part == GPX_END) {
        y = 0;
        i = sprintf(&bufferTx[y], "</trkseg>\n");
        y = y + i;
        i = sprintf(&bufferTx[y], "</trk>\n");
        y = y + i;
        i = sprintf(&bufferTx[y], "</gpx>\n");
        y = y + i;
        WRITEGPX(bufferTx, y * sizeof(uint8_t));
    }
}

#endif