#include "gps_log_file.h"
#include "ubx.h"
#include "gps_data.h"
#include "logger_config.h"
#include "log_private.h"

void log_ubx(gps_context_t *context, ubx_msg_t * ubxMessage) {
    logger_config_t *config = context->log_config->config;
    const uint8_t i[2] = {0xB5, 0x62};
    // write nav_pvt
    WRITEUBX(&(i[0]), sizeof(uint8_t));
    WRITEUBX(&(i[1]), sizeof(uint8_t));
    WRITEUBX(&ubxMessage->navPvt, sizeof(ubxMessage->navPvt));
    // write nav_sat
    if (ubxMessage->count_nav_sat != ubxMessage->count_nav_sat_prev) { // only add nav_sat msg to ubx file if new nav_sat message
        ubxMessage->count_nav_sat_prev = ubxMessage->count_nav_sat;
        WRITEUBX(&(i[0]), sizeof(uint8_t));
        WRITEUBX(&(i[1]), sizeof(uint8_t));
        //WRITEUBX(&ubxMessage->navPvt, sizeof(ubxMessage->navPvt));
        WRITEUBX(&ubxMessage->nav_sat, (ubxMessage->nav_sat.len + 6));// nav_sat has a variable length, add chkA and chkB !!!
    }
    // write nav_dop
    if (config->log_ubx_nav_sat) {  // only add navDOP msg to ubx file if nav_sat active
        WRITEUBX(&(i[0]), sizeof(uint8_t));
        WRITEUBX(&(i[1]), sizeof(uint8_t));
        WRITEUBX(&ubxMessage->navDOP, sizeof(ubxMessage->navDOP));
    }
}
