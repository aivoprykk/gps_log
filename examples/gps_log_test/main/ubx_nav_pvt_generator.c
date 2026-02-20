/**
 * @file ubx_nav_pvt_generator.c
 * @brief Generate UBX NAV-PVT messages with custom GPS data
 */

#include "ubx_nav_pvt_generator.h"
#include <string.h>
#include <time.h>
#include <math.h>

// UBX checksum calculation
static void ubx_checksum(const uint8_t *data, size_t len, uint8_t *ck_a, uint8_t *ck_b) {
    *ck_a = 0;
    *ck_b = 0;
    for (size_t i = 0; i < len; i++) {
        *ck_a += data[i];
        *ck_b += *ck_a;
    }
}

// Write little-endian int32
static void write_i32_le(uint8_t *buf, int32_t val) {
    buf[0] = (uint8_t)(val & 0xFF);
    buf[1] = (uint8_t)((val >> 8) & 0xFF);
    buf[2] = (uint8_t)((val >> 16) & 0xFF);
    buf[3] = (uint8_t)((val >> 24) & 0xFF);
}

// Write little-endian uint32
static void write_u32_le(uint8_t *buf, uint32_t val) {
    buf[0] = (uint8_t)(val & 0xFF);
    buf[1] = (uint8_t)((val >> 8) & 0xFF);
    buf[2] = (uint8_t)((val >> 16) & 0xFF);
    buf[3] = (uint8_t)((val >> 24) & 0xFF);
}

// Write little-endian uint16
static void write_u16_le(uint8_t *buf, uint16_t val) {
    buf[0] = (uint8_t)(val & 0xFF);
    buf[1] = (uint8_t)((val >> 8) & 0xFF);
}

// Write little-endian int16
static void write_i16_le(uint8_t *buf, int16_t val) {
    buf[0] = (uint8_t)(val & 0xFF);
    buf[1] = (uint8_t)((val >> 8) & 0xFF);
}

size_t generate_ubx_nav_pvt(
    uint8_t *buffer,
    double lat,
    double lon,
    float speed_ms,
    float heading,
    uint32_t itow,
    uint8_t num_sv
) {
    if (!buffer) return 0;

    // Get current time for date/time fields
    time_t now = time(NULL);
    struct tm timeinfo;
    gmtime_r(&now, &timeinfo);

    // UBX NAV-PVT structure (98 bytes: no sync bytes, just class+id+len+payload+checksum)
    // The nav_pvt_t struct starts with cls/id, NOT with sync bytes
    uint8_t *p = buffer;

    // Header (NO sync bytes - those are protocol framing)
    *p++ = 0x01;  // Class = NAV
    *p++ = 0x07;  // ID = PVT
    *p++ = 0x5C;  // Length LSB = 92
    *p++ = 0x00;  // Length MSB

    // Payload (92 bytes)
    uint8_t *payload_start = p;

    // iTOW (4 bytes)
    write_u32_le(p, itow); p += 4;

    // Date/Time (11 bytes)
    write_u16_le(p, timeinfo.tm_year + 1900); p += 2;  // year
    *p++ = timeinfo.tm_mon + 1;  // month (1-12)
    *p++ = timeinfo.tm_mday;     // day (1-31)
    *p++ = timeinfo.tm_hour;     // hour (0-23)
    *p++ = timeinfo.tm_min;      // minute (0-59)
    *p++ = timeinfo.tm_sec;      // second (0-59)
    *p++ = 0x07;                 // valid flags (validDate | validTime | fullyResolved)
    write_u32_le(p, 1000);       // tAcc = 1000 ns
    p += 4;
    write_i32_le(p, 0);          // nano = 0 ns
    p += 4;

    // Fix status (4 bytes)
    *p++ = 0x03;      // fixType = 3D-fix
    *p++ = 0x01;      // flags = gnssFixOK
    *p++ = 0x00;      // flags2
    *p++ = num_sv;    // numSV (number of satellites)

    // Position (16 bytes)
    int32_t lon_i = (int32_t)(lon * 1e7);  // degrees * 1e7
    int32_t lat_i = (int32_t)(lat * 1e7);  // degrees * 1e7
    write_i32_le(p, lon_i); p += 4;        // lon
    write_i32_le(p, lat_i); p += 4;        // lat
    write_i32_le(p, 500); p += 4;          // height = 500mm
    write_i32_le(p, 300); p += 4;          // hMSL = 300mm

    // Accuracy (8 bytes)
    write_u32_le(p, 1000); p += 4;         // hAcc = 1000mm
    write_u32_le(p, 500); p += 4;          // vAcc = 500mm

    // Velocity (24 bytes)
    int32_t speed_mm_s = (int32_t)(speed_ms * 1000.0f);  // Convert m/s to mm/s
    float heading_rad = heading * 0.017453292f;  // degrees to radians
    int32_t vel_n = (int32_t)(speed_mm_s * cosf(heading_rad));
    int32_t vel_e = (int32_t)(speed_mm_s * sinf(heading_rad));

    write_i32_le(p, vel_n); p += 4;        // velN (north velocity mm/s)
    write_i32_le(p, vel_e); p += 4;        // velE (east velocity mm/s)
    write_i32_le(p, 0); p += 4;            // velD (down velocity = 0)
    write_i32_le(p, speed_mm_s); p += 4;   // gSpeed (ground speed mm/s)
    int32_t head_mot = (int32_t)(heading * 1e5);  // heading * 1e-5 degrees
    write_i32_le(p, head_mot); p += 4;     // headMot
    write_u32_le(p, 1000); p += 4;         // sAcc = 1000 mm/s speed accuracy

    // Heading accuracy (4 bytes)
    write_u32_le(p, 1000000); p += 4;      // headAcc = 10° * 1e-5

    // DOP (2 bytes)
    write_u16_le(p, 100); p += 2;          // pDOP = 1.00 * 100

    // Reserved + flags3 (6 bytes)
    memset(p, 0, 6); p += 6;

    // Vehicle heading and magnetic (8 bytes)
    write_i32_le(p, head_mot); p += 4;     // headVeh = heading
    write_i16_le(p, 0); p += 2;            // magDec = 0
    write_u16_le(p, 0); p += 2;            // magAcc = 0

    // Calculate checksum
    size_t payload_len = p - payload_start;
    if (payload_len != 92) {
        return 0;  // Error - payload should be exactly 92 bytes
    }

    uint8_t ck_a, ck_b;
    ubx_checksum(buffer, 4 + payload_len, &ck_a, &ck_b);  // Class + ID + Length + Payload (no sync bytes)
    *p++ = ck_a;
    *p++ = ck_b;

    return (p - buffer);  // Should be 98 bytes (was 100 with sync bytes)
}

size_t generate_ubx_nav_dop(uint8_t *buffer, uint32_t itow) {
    if (!buffer) return 0;

    uint8_t *p = buffer;

    // Header (NO sync bytes)
    *p++ = 0x01;  // Class = NAV
    *p++ = 0x04;  // ID = DOP
    *p++ = 0x12;  // Length LSB = 18
    *p++ = 0x00;  // Length MSB

    // Payload
    write_u32_le(p, itow); p += 4;
    write_u16_le(p, 100); p += 2;  // gDOP = 1.00
    write_u16_le(p, 100); p += 2;  // pDOP = 1.00
    write_u16_le(p, 100); p += 2;  // tDOP = 1.00
    write_u16_le(p, 100); p += 2;  // vDOP = 1.00
    write_u16_le(p, 100); p += 2;  // hDOP = 1.00
    write_u16_le(p, 100); p += 2;  // nDOP = 1.00
    write_u16_le(p, 100); p += 2;  // eDOP = 1.00

    // Checksum
    uint8_t ck_a, ck_b;
    ubx_checksum(buffer, 4 + 18, &ck_a, &ck_b);
    *p++ = ck_a;
    *p++ = ck_b;

    return (p - buffer);  // Should be 24 bytes (was 26 with sync bytes)
}

size_t generate_ubx_nav_sat(uint8_t *buffer, uint32_t itow, uint8_t num_sv) {
    if (!buffer || num_sv == 0 || num_sv > 12) return 0;

    uint8_t *p = buffer;

    // Header (NO sync bytes)
    *p++ = 0x01;  // Class = NAV
    *p++ = 0x35;  // ID = SAT

    uint16_t payload_len = 8 + (num_sv * 12);  // Header(8) + satellites(12 each)
    write_u16_le(p, payload_len); p += 2;

    // Payload
    write_u32_le(p, itow); p += 4;
    *p++ = 0x01;      // version
    *p++ = num_sv;    // numSvs
    *p++ = 0x00;      // reserved
    *p++ = 0x00;      // reserved

    // Generate satellite data
    for (uint8_t i = 0; i < num_sv; i++) {
        uint8_t gnss_id = (i < 8) ? 0x00 : 0x06;  // GPS or GLONASS
        uint8_t sv_id = (i < 8) ? (i + 1) : (i - 7);
        uint8_t cno = 35 + (i % 10);  // Signal strength 35-45 dBHz
        int8_t elev = 30 + (i * 5);   // Elevation angle
        int16_t azim = (i * 45);      // Azimuth angle

        *p++ = gnss_id;
        *p++ = sv_id;
        *p++ = cno;
        *p++ = (uint8_t)elev;
        write_i16_le(p, azim); p += 2;
        write_i16_le(p, 0); p += 2;   // prRes
        write_u32_le(p, 0x07030000 | (cno << 8)); p += 4;  // flags: used + healthy
    }

    // Checksum
    uint8_t ck_a, ck_b;
    ubx_checksum(buffer, 4 + payload_len, &ck_a, &ck_b);
    *p++ = ck_a;
    *p++ = ck_b;

    return (p - buffer);  // 2 bytes less than before (no sync bytes)
}
