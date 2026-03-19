
#include "log_private.h"
#if (defined(CONFIG_UBLOX_ENABLED) && defined(CONFIG_GPS_LOG_ENABLED))

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <sys/unistd.h>

#include "sbp.h"
#include "ubx_msg.h"
#include "gps_log_file.h"
#include "gps_data.h"
#include "strbf.h"
#include "ubx.h"

//static const char* TAG = "sbp";
//extern struct UBXMessage ubxMessage;

/* Locosys SBP structures */

#define SBP_HEADER_USER_NAME       "ESP-GPS"
#define SBP_HEADER_MAX_FW_LEN      13U

struct SBP_Header sbp_header = {
    .Text_length = 0,   // set from actual identity length before writing
    .Id1 = 0xa0,        // seems to be necessary so that GP3S accept .sbp
    .Id2 = 0xa2,        // seems to be necessary so that GP3S accept .sbp
    .Again_length = 0,
    .Start = 0xfd,
    .Identity = ""};

struct SBP_frame sbp_frame = {
    .HDOP = 0,                 /* HDOP [0..51] with resolution 0.2 */
    .SVIDCnt = 0,              /* Number of SVs in solution [0 to 12] */
    .UtcSec = 0,               /* UTC Second [0 to 59] in seconds with resolution 0.001 */
    .date_time_UTC_packed = 0, /* refer to protocol doc*/
    .SVIDList = 0,             /* SVs in solution:  Bit 0=1: SV1, Bit 1=1: SV2, ... , Bit 31=1: SV32 */
    .Lat = 0,                  /* Latitude [-90 to 90] in degrees with resolution 0.0000001 */
    .Lon = 0,                  /* Longitude [-180 to 180] in degrees with resolution 0.0000001 */
    .AltCM = 0,                /* Altitude from Mean Sea Level in centi meters */
    .Sog = 0,                  /* Speed Over Ground in m/sec with resolution 0.01 */
    .Cog = 0,                  /* Course Over Ground [0 to 360] in degrees with resolution 0.01 */
    .ClmbRte = 0,              /* Climb rate in m/sec with resolution 0.01 */
    .sdop = 0,                 /* GT31 */
    .vsdop = 0};

void log_header_SBP(struct gps_context_s * context) {
    const char *firmware_version = "unknown";
    uint8_t log_rate = 0;
    strbf_t sb;

    if (NOSBP)
        return;
    if (context && context->SW_version && context->SW_version[0] != '\0') {
        firmware_version = context->SW_version;
    }
    log_rate = g_rtc_config.ubx.output_rate;

    memset(sbp_header.Identity, 0, sizeof(sbp_header.Identity));
    strbf_inits(&sb, sbp_header.Identity, sizeof(sbp_header.Identity));
    strbf_puts(&sb, SBP_HEADER_USER_NAME);
    strbf_putc(&sb, ',');
    if (context && context->mac_address) {
        strbf_put_hex_u8(&sb, context->mac_address[2]);
        strbf_put_hex_u8(&sb, context->mac_address[3]);
        strbf_put_hex_u8(&sb, context->mac_address[4]);
        strbf_put_hex_u8(&sb, context->mac_address[5]);
    } else {
        strbf_puts(&sb, "00000000");
    }
    strbf_putc(&sb, ',');
    strbf_putul(&sb, log_rate);
    strbf_putc(&sb, ',');
    strbf_put(&sb, firmware_version,
              strnlen(firmware_version, SBP_HEADER_MAX_FW_LEN));
    sbp_header.Text_length = (uint16_t)(sb.cur - sb.start);
    sbp_header.Again_length = sbp_header.Text_length;
    for (uint8_t i = (uint8_t)(7U + sbp_header.Text_length); i < 64U; i++) {
        ((uint8_t*)(&sbp_header))[i] = 0xFF;
    }  // fill with 0xFF
    WRITESBP(&sbp_header, 64 * sizeof(uint8_t));
    // fprintf(file, (const uint8_t *)&sbp_header,64);
}

void log_SBP(struct gps_context_s * context) {
    if (NOSBP)
        return;
    const struct ubx_msg_s *ubxMessage = &context->ubx_device->ubx_msg;
    uint32_t year = ubxMessage->navPvt.year;
    uint8_t month = ubxMessage->navPvt.month;
    uint8_t day = ubxMessage->navPvt.day;
    uint8_t hour = ubxMessage->navPvt.hour;
    uint8_t minute = ubxMessage->navPvt.minute;
    uint8_t second = ubxMessage->navPvt.second;
    int32_t utc_ms = c_nano_to_millis_round(ubxMessage->navPvt.nano);
    uint32_t numSV = 0xFFFFFFFF;
    uint32_t HDOP = (ubxMessage->navDOP.hDOP + 1) / 20;  // from 0.01 resolution to 0.2 ,reformat pDOP to HDOP 8-bit !!
    if (HDOP > 255)
        HDOP = 255;                               // has to fit in 8 bit
    uint32_t sdop = ubxMessage->navPvt.sAcc / 10;  // was sAcc
    if (sdop > 255)
        sdop = 255;
    uint32_t vsdop = ubxMessage->navPvt.vAcc / 10;  // was headingAcc ???
    if (vsdop > 255)
        vsdop = 255;
    c_normalize_utc_fields(&year, &month, &day, &hour, &minute, &second,
                           &utc_ms, 1000U);
    sbp_frame.UtcSec = (uint16_t)((second * 1000U) + (uint32_t)utc_ms);
    sbp_frame.date_time_UTC_packed = (((year - 2000) * 12 + month) << 22) +
                                     (day << 17) + (hour << 12) +
                                     (minute << 6) + second;
    sbp_frame.Lat = ubxMessage->navPvt.lat;
    sbp_frame.Lon = ubxMessage->navPvt.lon;
    sbp_frame.AltCM = ubxMessage->navPvt.hMSL / 10;     // omrekenen naar cm/s
    sbp_frame.Sog = ubxMessage->navPvt.gSpeed / 10;     // omrekenen naar cm/s
    sbp_frame.Cog = ubxMessage->navPvt.heading / 1000;  // omrekenen naar 0.01 degrees
    sbp_frame.SVIDCnt = ubxMessage->navPvt.numSV;
    sbp_frame.SVIDList = numSV >> (32 - ubxMessage->navPvt.numSV);
    sbp_frame.HDOP = HDOP;
    sbp_frame.ClmbRte = -ubxMessage->navPvt.velD / 10;  // omrekenen naar cm/s
    sbp_frame.sdop = sdop;
    sbp_frame.vsdop = vsdop;
    WRITESBP(&sbp_frame, 32 * sizeof(uint8_t));
    // fprintf(file, (const uint8_t *)&sbp_frame,32);
}

#endif
