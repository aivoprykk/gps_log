#ifndef C07B91B3_B291_4BBE_9DB5_830BE329AC8A
#define C07B91B3_B291_4BBE_9DB5_830BE329AC8A

#ifdef __cplusplus
extern "C" {
#endif

#define GPS_EVENT_BASE 0x20  // Component ID 2

#include "esp_event.h"
#include "logger_common.h"

#define GPS_LOG_EVENT_LIST(l) \
    l(REQUEST_FILE_OPEN) \
    l(REQUEST_FILE_FLUSH) \
    l(LOG_FILES_OPEN_FAILED) \
    l(LOG_FILES_OPENED) \
    l(LOG_FILES_SAVED) \
    l(LOG_FILES_CLOSED) \
    l(GPS_FRAME_LOST) \
    l(GPS_FIRST_FIX) \
    l(GPS_IS_MOVING) \
    l(GPS_IS_STOPPING) \
    l(GPS_REQUEST_RESTART) \
    l(GPS_SHUT_DOWN_DONE) \
    l(GPS_SAVE_FILES) \
    l(GPS_TIME_SET) \
    l(GPS_NEW_RUN) \
    l(GPS_NAV_MODE_CHANGED) \
    l(LOW_STORAGE) \
    l(CFG_SET) \
    l(CFG_GET) \
    l(CFG_CHANGED) \
    l(CONFIG_REFRESHED)

ESP_EVENT_DECLARE_BASE(GPS_LOG_EVENT);        // declaration of the LOG_EVENT family
// declaration of the specific events under the LOG_EVENT family
#define LL(x) GPS_LOG_EVENT_ ## x,
enum {GPS_LOG_EVENT_LIST(LL)};
#undef LL

const char * gps_log_event_strings(int id);

#ifdef __cplusplus
}
#endif

#endif /* C07B91B3_B291_4BBE_9DB5_830BE329AC8A */
