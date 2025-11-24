#ifndef C07B91B3_B291_4BBE_9DB5_830BE329AC8A
#define C07B91B3_B291_4BBE_9DB5_830BE329AC8A

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_event.h"
#include "logger_common.h"

// Declare an event base
ESP_EVENT_DECLARE_BASE(GPS_LOG_EVENT);        // declaration of the LOG_EVENT family

#define GPS_LOG_EVENT_LIST(l) \
    l(GPS_LOG_EVENT_LOG_FILES_OPEN_FAILED) \
    l(GPS_LOG_EVENT_LOG_FILES_OPENED) \
    l(GPS_LOG_EVENT_LOG_FILES_SAVED) \
    l(GPS_LOG_EVENT_LOG_FILES_CLOSED) \
    l(GPS_LOG_EVENT_GPS_FRAME_LOST) \
    l(GPS_LOG_EVENT_GPS_FIRST_FIX) \
    l(GPS_LOG_EVENT_GPS_IS_MOVING) \
    l(GPS_LOG_EVENT_GPS_IS_STOPPING) \
    l(GPS_LOG_EVENT_GPS_REQUEST_RESTART) \
    l(GPS_LOG_EVENT_GPS_SHUT_DOWN_DONE) \
    l(GPS_LOG_EVENT_GPS_SAVE_FILES) \
    l(GPS_LOG_EVENT_GPS_TIME_SET) \
    l(GPS_LOG_EVENT_GPS_NEW_RUN) \
    l(GPS_LOG_EVENT_GPS_NAV_MODE_CHANGED) \
    l(GPS_LOG_EVENT_CFG_SET) \
    l(GPS_LOG_EVENT_CFG_GET) \
    l(GPS_LOG_EVENT_CFG_CHANGED) \
// declaration of the specific events under the LOG_EVENT family
enum {                                       
    GPS_LOG_EVENT_LIST(ENUM)
};

const char * gps_log_event_strings(int id);

#ifdef __cplusplus
}
#endif

#endif /* C07B91B3_B291_4BBE_9DB5_830BE329AC8A */
