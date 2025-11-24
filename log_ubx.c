#include "log_private.h"
#if (defined(CONFIG_UBLOX_ENABLED) && defined(CONFIG_GPS_LOG_ENABLED))

#include "gps_log_file.h"
#include "ubx.h"
#include "gps_data.h"

void log_ubx(gps_context_t *context, ubx_msg_t * ubxMessage, bool log_ubx_nav_sat) {
    const uint8_t i[2] = {0xB5, 0x62};
    // write nav_pvt
    WRITEUBX(&(i[0]), 2);
    WRITEUBX(&ubxMessage->navPvt, sizeof(ubxMessage->navPvt));
    // write nav_sat
    if (ubxMessage->count_nav_sat != ubxMessage->count_nav_sat_prev) { // only add nav_sat msg to ubx file if new nav_sat message
        ubxMessage->count_nav_sat_prev = ubxMessage->count_nav_sat;
        WRITEUBX(&(i[0]), 2);
        WRITEUBX(&ubxMessage->nav_sat, (ubxMessage->nav_sat.len + 4)); // payload + 2 bit for header + 2 bit for size
        WRITEUBX(&ubxMessage->nav_sat.chkA, 2); // checkA and checkB are 2 bytes
    }
    // write nav_dop
    if (log_ubx_nav_sat) {  // only add navDOP msg to ubx file if nav_sat active
        WRITEUBX(&(i[0]), 2);
        WRITEUBX(&ubxMessage->navDOP, sizeof(ubxMessage->navDOP));
    }
}

#endif
