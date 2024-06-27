#ifndef C07B91B3_B291_4BBE_9DB5_830BE329AC8A
#define C07B91B3_B291_4BBE_9DB5_830BE329AC8A

#include "esp_event.h"

#ifdef __cplusplus
extern "C" {
#endif

// Declare an event base
ESP_EVENT_DECLARE_BASE(GPS_LOG_EVENT);        // declaration of the LOG_EVENT family

// declaration of the specific events under the LOG_EVENT family
enum {                                       
    GPS_LOG_EVENT_LOG_FILES_OPEN_FAILED,
    GPS_LOG_EVENT_LOG_FILES_OPENED, 
    GPS_LOG_EVENT_LOG_FILES_SAVED, 
    GPS_LOG_EVENT_LOG_FILES_CLOSED,
    GPS_LOG_EVENT_GPS_FRAME_LOST,
};

extern const char * gps_log_event_strings[];

#ifdef __cplusplus
}
#endif

#endif /* C07B91B3_B291_4BBE_9DB5_830BE329AC8A */
