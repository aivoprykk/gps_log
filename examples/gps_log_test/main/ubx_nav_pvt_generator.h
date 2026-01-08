/**
 * @file ubx_nav_pvt_generator.h
 * @brief Generate UBX NAV-PVT messages with custom GPS data
 * 
 * This module creates valid UBX NAV-PVT messages with realistic GPS data
 * from the GPS track generator. Used for integration testing of gps_log module.
 */

#ifndef UBX_NAV_PVT_GENERATOR_H
#define UBX_NAV_PVT_GENERATOR_H

#include <stdint.h>
#include <stddef.h>

/**
 * Generate UBX NAV-PVT message with custom GPS data
 * 
 * @param buffer Output buffer (must be at least 100 bytes)
 * @param lat Latitude in degrees
 * @param lon Longitude in degrees
 * @param speed_ms Speed in meters/second
 * @param heading Heading in degrees
 * @param itow GPS time of week in milliseconds
 * @param num_sv Number of satellites
 * @return Size of generated message (100 bytes) or 0 on error
 */
size_t generate_ubx_nav_pvt(
    uint8_t *buffer,
    double lat,
    double lon,
    float speed_ms,
    float heading,
    uint32_t itow,
    uint8_t num_sv
);

/**
 * Generate UBX NAV-DOP message
 * @param buffer Output buffer (must be at least 26 bytes)
 * @param itow GPS time of week in milliseconds
 * @return Size of generated message (26 bytes) or 0 on error
 */
size_t generate_ubx_nav_dop(uint8_t *buffer, uint32_t itow);

/**
 * Generate UBX NAV-SAT message with increasing satellites
 * @param buffer Output buffer (must be at least 120 bytes)
 * @param itow GPS time of week in milliseconds
 * @param num_sv Number of satellites (1-12)
 * @return Size of generated message or 0 on error
 */
size_t generate_ubx_nav_sat(uint8_t *buffer, uint32_t itow, uint8_t num_sv);

#endif // UBX_NAV_PVT_GENERATOR_H
