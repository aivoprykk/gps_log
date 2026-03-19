
#include "log_private.h"
#if (defined(CONFIG_UBLOX_ENABLED) && defined(CONFIG_GPS_LOG_ENABLED))
#if defined(CONFIG_GPS_LOG_GPY)
#include <stdlib.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/time.h>
#include <errno.h>

#include <esp_mac.h>

#include "gpy.h"
#include "strbf.h"
#include "ubx.h"
#include "gps_log_file.h"
#include "gps_data.h"

//static const char * TAG = "gpy";
// extern int next_gpy_full_frame;
// extern unsigned char mac[6];
// extern char SW_version[16];

//extern struct UBXMessage ubxMessage;

#define GPY_HEADER_DEVICE_DESCRIPTION "ESP-GPS"
#define GPY_HEADER_DEVICE_NAME        "ESP-GPS"

struct GPY_Header gpy_header = {
    .Type_identifier = 0xF0,  // Frame identifier, header = 0xF0
    .Flags = 0,
    .Length = 72,     // length = 6 + 4 * STRING_IO_LENGTH + 2 = 72
    .DeviceType = 2,  // ublox = 2
    .deviceDescription = "",
    .deviceName = "",
    .serialNumber = "",
    .firmwareVersion = "",
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
    const char *firmware_version = "unknown";
    strbf_t sb;

    if(NOGPY)
        return;
    memset(gpy_header.deviceDescription, 0, sizeof(gpy_header.deviceDescription));
    memset(gpy_header.deviceName, 0, sizeof(gpy_header.deviceName));
    memset(gpy_header.serialNumber, 0, sizeof(gpy_header.serialNumber));
    memset(gpy_header.firmwareVersion, 0, sizeof(gpy_header.firmwareVersion));

    strbf_inits(&sb, gpy_header.deviceDescription,
                sizeof(gpy_header.deviceDescription));
    strbf_puts(&sb, GPY_HEADER_DEVICE_DESCRIPTION);
    strbf_inits(&sb, gpy_header.deviceName, sizeof(gpy_header.deviceName));
    strbf_puts(&sb, GPY_HEADER_DEVICE_NAME);
    if (context && context->SW_version && context->SW_version[0] != '\0') {
        firmware_version = context->SW_version;
    }
    strbf_inits(&sb, gpy_header.firmwareVersion,
                sizeof(gpy_header.firmwareVersion));
    strbf_put(&sb, firmware_version,
              strnlen(firmware_version,
                      sizeof(gpy_header.firmwareVersion) - 1U));
    if (context && context->mac_address) {
        strbf_inits(&sb, gpy_header.serialNumber, sizeof(gpy_header.serialNumber));
        for (uint8_t i = 0; i < 6; ++i) {
            strbf_put_hex_u8(&sb, context->mac_address[i]);
        }
    } else {
        strbf_inits(&sb, gpy_header.serialNumber, sizeof(gpy_header.serialNumber));
        strbf_puts(&sb, "000000000000");
    }
    Fletcher16((uint8_t *)&gpy_header, 72);
    WRITEGPY(&gpy_header, 72 * sizeof(uint8_t));
}

void log_GPY(struct gps_context_s *context) {
    if(NOGPY)
        return;
    // convert a date and time into unix time, offset 1970, Arduino 8bytes = LL
    // !!!!!
    const ubx_msg_t * ubxMessage = &context->ubx_device->ubx_msg;
    time_t utc_Sec;
    uint32_t gps_year = ubxMessage->navPvt.year;
    uint8_t gps_month = ubxMessage->navPvt.month;
    uint8_t gps_day = ubxMessage->navPvt.day;
    uint8_t gps_hour = ubxMessage->navPvt.hour;
    uint8_t gps_minute = ubxMessage->navPvt.minute;
    uint8_t gps_second = ubxMessage->navPvt.second;
    int32_t gps_millis = c_nano_to_millis_round(ubxMessage->navPvt.nano);
    struct tm frame_time = {0};  // time elements structure

    c_normalize_utc_fields(&gps_year, &gps_month, &gps_day, &gps_hour,
                           &gps_minute, &gps_second, &gps_millis, 1000U);
    frame_time.tm_sec = gps_second;
    frame_time.tm_hour = gps_hour;
    frame_time.tm_min = gps_minute;
    frame_time.tm_mday = gps_day;
    frame_time.tm_mon = gps_month - 1;  // month 0 - 11 with mktime
    frame_time.tm_year = gps_year - 1900;  // years since 1900, so deduct 1900
    frame_time.tm_isdst = 0;            // No daylight saving
    utc_Sec = mktime(&frame_time);  // mktime returns local time, so TZ is important !!!
    int64_t utc_ms = utc_Sec * 1000LL + gps_millis;

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

#endif
#endif