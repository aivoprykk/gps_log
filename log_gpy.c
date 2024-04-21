
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/time.h>
#include <errno.h>

#include "esp_log.h"
#include <esp_mac.h>

#include "gpy.h"
#include "ubx.h"
#include "gps_log_file.h"
#include "gps_data.h"
#include "log_private.h"

//static const char * TAG = "gpy";
// extern int next_gpy_full_frame;
// extern unsigned char mac[6];
// extern char SW_version[16];

//extern struct UBXMessage ubxMessage;
//extern struct context_s m_context;

struct GPY_Header gpy_header = {
    .Type_identifier = 0xF0,  // Frame identifier, header = 0xF0
    .Flags = 0,
    .Length = 72,     // length = 6 + 4 * STRING_IO_LENGTH + 2 = 72
    .DeviceType = 2,  // ublox = 2
    .deviceDescription = "ESP-GPS",
    .deviceName = "Boom",
    .serialNumber = "macAddr",
    .firmwareVersion = "SW_version",
    .Checksum = 0};

struct GPY_Frame gpy_frame = {
    .Type_identifier = 0xE0,  // Frame identifier for full frame = 0xE0
    .Flags = 0x0,
    .HDOP = 0,         // HDOP
    .Unix_time = 0,    // ms
    .Speed = 0,        // mm/s
    .Speed_error = 0,  // sAccCourse_Over_Ground,
    .Latitude = 0,
    .Longitude = 0,
    .COG = 0,  // Course over ground
    .Sat = 0,  // number of sats
    .fix = 0,
    .Checksum = 0};

struct GPY_Frame_compressed gpy_frame_compressed = {
    .Type_identifier = 0xD0,  // Frame identifier for compressed frame = 0xD0
    .Flags = 0x0,
    .HDOP = 0x0,               // HDOP
    .delta_time = 0x0,         // ms
    .delta_Speed = 0x0,        // mm/
    .delta_Speed_error = 0x0,  // sAccCourse_Over_Ground,
    .delta_Latitude = 0x0,
    .delta_Longitude = 0x0,
    .delta_COG = 0x0,  // delta (course / 1000) !
    .Sat = 0x0,        // number of sats
    .fix = 0x0,
    .Checksum = 0x0};

uint16_t Fletcher16(uint8_t *data, int count) {
    uint16_t sum1 = 0;
    uint16_t sum2 = 0;
    /* Sum all the bytes, but not the last 2 */
    for (int i = 0; i < (count - 2); ++i) {
        sum1 = (sum1 + data[i]) & 0xFF;  // divide by 256 in stead of 255 !!
        sum2 = (sum2 + sum1) & 0xFF;
    }
    data[count - 2] = sum1;
    data[count - 1] = sum2;
    return (sum2 << 8) | sum1;
}
// https://community.particle.io/t/make-epoch-timestamp-maketime-solved/10013
/*
time_t tmConvert_t(int YYYY, byte MM, byte DD, byte hh, byte mm, byte ss)
{
  tmElements_t tmSet;
  tmSet.Year = YYYY;
  tmSet.Month = MM;
  tmSet.Day = DD;
  tmSet.Hour = hh;
  tmSet.Minute = mm;
  tmSet.Second = ss;
  return makeTime(tmSet);
}
*/
void log_GPY_Header(const struct gps_context_s *context) {
    if(NOGPY)
        return;
    for (int i = 0; i < 16; i++) {
        gpy_header.firmwareVersion[i] = context->SW_version[i];
    }
    sprintf(gpy_header.serialNumber, "%02x%02x%02x%02x%02x%02x", MAC2STR(context->mac_address));
    Fletcher16((uint8_t *)&gpy_header, 72);
    WRITEGPY(&gpy_header, 72 * sizeof(uint8_t));
}

void log_GPY(struct gps_context_s *context) {
    if(NOGPY)
        return;
    // convert a date and time into unix time, offset 1970, Arduino 8bytes = LL
    // !!!!!
    const ubx_msg_t * ubxMessage = &context->ublox_config->ubx_msg;
    time_t utc_Sec;
    struct tm frame_time;  // time elements structure
    frame_time.tm_sec = ubxMessage->navPvt.second;
    frame_time.tm_hour = ubxMessage->navPvt.hour;
    frame_time.tm_min = ubxMessage->navPvt.minute;
    frame_time.tm_mday = ubxMessage->navPvt.day;
    frame_time.tm_mon = ubxMessage->navPvt.month - 1;  // month 0 - 11 with
                                                      // mktime
    frame_time.tm_year = ubxMessage->navPvt.year - 1900;  // years since 1900, so deduct 1900
    frame_time.tm_isdst = 0;            // No daylight saving
    utc_Sec = mktime(&frame_time);  // mktime returns local time, so TZ is important !!!
    int64_t utc_ms = utc_Sec * 1000LL + (ubxMessage->navPvt.nano + 500000) / 1000000LL;

    // calcultation of delta values
    int delta_time = utc_ms - gpy_frame.Unix_time;                 // ms
    int delta_Speed = ubxMessage->navPvt.gSpeed - gpy_frame.Speed;  // mm/
    int delta_Speed_error = ubxMessage->navPvt.sAcc - gpy_frame.Speed_error;  // sAccCourse_Over_Ground;
    int delta_Latitude = ubxMessage->navPvt.lat - gpy_frame.Latitude;
    int delta_Longitude = ubxMessage->navPvt.lon - gpy_frame.Longitude;
    int delta_COG = ubxMessage->navPvt.heading / 1000 - gpy_frame.COG / 1000;  // delta (course / 1000) !
#define SIGNED_INT 30000  // if delta is more, a full frame is written
    int full_frame = 0;
    static int first_frame = 0;
    if ((delta_time > SIGNED_INT) | (delta_time < -SIGNED_INT))
        full_frame = 1;
    if ((delta_Speed > SIGNED_INT) | (delta_Speed < -SIGNED_INT))
        full_frame = 1;
    if ((delta_Speed_error > SIGNED_INT) | (delta_Speed_error < -SIGNED_INT))
        full_frame = 1;
    if ((delta_Latitude > SIGNED_INT) | (delta_Latitude < -SIGNED_INT))
        full_frame = 1;
    if ((delta_Longitude > SIGNED_INT) | (delta_Longitude < -SIGNED_INT))
        full_frame = 1;
    if ((delta_COG > SIGNED_INT) | (delta_COG < -SIGNED_INT))
        full_frame = 1;
    if (first_frame == 0)
        full_frame = 1;  // first frame is always a full frame
    if (context->next_gpy_full_frame) {
        full_frame = 1;
        context->next_gpy_full_frame = 0;
    }  // if a navPvt frame is lost, next frame = full frame !!!
    if (full_frame == 1) {
        gpy_frame.HDOP = ubxMessage->navDOP.hDOP;  // ubxMessage->navPvt.pDOP;
        gpy_frame.Unix_time = utc_ms;
        gpy_frame.Speed = ubxMessage->navPvt.gSpeed;
        gpy_frame.Speed_error = ubxMessage->navPvt.sAcc;
        gpy_frame.Latitude = ubxMessage->navPvt.lat;
        gpy_frame.Longitude = ubxMessage->navPvt.lon;
        gpy_frame.COG = ubxMessage->navPvt.heading;
        gpy_frame.Sat = ubxMessage->navPvt.numSV;
        gpy_frame.fix = ubxMessage->navPvt.fixType;
        Fletcher16((uint8_t *)&gpy_frame, 36);
        WRITEGPY(&gpy_frame, 36 * sizeof(uint8_t));
        first_frame = 1;
        // Serial.println(" full ");
    } else {
        gpy_frame_compressed.HDOP =
            ubxMessage->navDOP.hDOP;                    // ubxMessage->navPvt.pDOP
        gpy_frame_compressed.delta_time = delta_time;  // ms
        gpy_frame_compressed.delta_Speed = delta_Speed;  // mm/
        gpy_frame_compressed.delta_Speed_error =
            delta_Speed_error;  // sAccCourse_Over_Ground;
        gpy_frame_compressed.delta_Latitude = delta_Latitude;
        gpy_frame_compressed.delta_Longitude = delta_Longitude;
        gpy_frame_compressed.delta_COG = delta_COG;  // delta (course / 1000) !
        gpy_frame_compressed.Sat = ubxMessage->navPvt.numSV;  // number of sats
        gpy_frame_compressed.fix = ubxMessage->navPvt.fixType;
        Fletcher16((uint8_t *)&gpy_frame_compressed, 20);
        WRITEGPY(&gpy_frame_compressed, 20 * sizeof(uint8_t));
    }
}
