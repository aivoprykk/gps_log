#ifndef C07B91B3_B291_4BBE_9DB5_830BE329AC8A
#define C07B91B3_B291_4BBE_9DB5_830BE329AC8A

#include "esp_event.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "logger_common.h"

// Declare an event base
ESP_EVENT_DECLARE_BASE(GPS_LOG_EVENT);        // declaration of the LOG_EVENT family

#define GPS_LOG_EVENT_LIST(l) \
    l(GPS_LOG_EVENT_LOG_FILES_OPEN_FAILED) \
    l(GPS_LOG_EVENT_LOG_FILES_OPENED) \
    l(GPS_LOG_EVENT_LOG_FILES_SAVED) \
    l(GPS_LOG_EVENT_LOG_FILES_CLOSED) \
    l(GPS_LOG_EVENT_GPS_FRAME_LOST)
// declaration of the specific events under the LOG_EVENT family
enum {                                       
    GPS_LOG_EVENT_LIST(ENUM)
};

extern const char * const gps_log_event_strings[];

#ifdef __cplusplus
}
#endif

#endif /* C07B91B3_B291_4BBE_9DB5_830BE329AC8A */
