#ifndef DB99F2E7_B596_4059_B6AF_FAD2A14CD6A0
#define DB99F2E7_B596_4059_B6AF_FAD2A14CD6A0

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

struct gps_context_s;

int log_get_fd(const struct gps_context_s * context, uint8_t file);
size_t log_write(const struct gps_context_s * context, uint8_t file, const void * msg, size_t len);
int log_close(const struct gps_context_s * context, uint8_t file);
int log_fsync(const struct gps_context_s * context, uint8_t file);
void printFile(const char *filename);

#define NOGPY (log_get_fd((context), SD_GPY)==-1)
#define NOGPX (log_get_fd((context), SD_GPX)==-1)
#define NOSBP (log_get_fd((context), SD_SBP)==-1)
#define NOTXT (log_get_fd((context), SD_TXT)==-1)

#define WRITEGPX(msg, len) log_write((context), SD_GPX, (msg), (len))
#define WRITEGPY(msg, len) log_write((context), SD_GPY, (msg), (len))
#define WRITEUBX(msg, len) log_write((context), SD_UBX, (msg), (len))
#define WRITESBP(msg, len) log_write((context), SD_SBP, (msg), (len))
#define WRITETXT(msg, len) log_write((context), SD_TXT, (msg), (len))

#define GET_FD(f) (context->log_config->filefds[f])

#if (CONFIG_GPS_LOG_LEVEL <= 2)

#include "esp_timer.h"
#include "esp_log.h"

#endif

#if (CONFIG_GPS_LOG_LEVEL == CONFIG_GPS_LOG_LEVEL_TRACE) // "A lot of logs to give detailed information"

#define DLOG(a, b, ...) ESP_LOGI(a, b, __VA_ARGS__)
#define DMEAS_START() uint64_t _start = (esp_timer_get_time()), _end = 0
#define DMEAS_END(a, b, ...) \
    _end = (esp_timer_get_time());  \
    ESP_LOGI(a, b, __VA_ARGS__, _end - _start)
#define ILOG DLOG
#define IMEAS_START DMEAS_START
#define IMEAS_END DMEAS_END
#define WLOG DLOG
#define WMEAS_START DMEAS_START
#define WMEAS_END DMEAS_END

#elif (CONFIG_GPS_LOG_LEVEL == CONFIG_GPS_LOG_LEVEL_INFO) // "Log important events"

#define DLOG(a, b, ...) ((void)0)
#define DMEAS_START() ((void)0)
#define DMEAS_END(a, b, ...) ((void)0)
#define ILOG(a, b, ...) ESP_LOGI(a, b, __VA_ARGS__)
#define IMEAS_START() uint64_t _start = (esp_timer_get_time()), _end = 0
#define IMEAS_END(a, b, ...) \
    _end = (esp_timer_get_time());  \
    ESP_LOGI(a, b, __VA_ARGS__, _end - _start)
#define WLOG ILOG
#define WMEAS_START IMEAS_START
#define WMEAS_END IMEAS_END

#elif (CONFIG_GPS_LOG_LEVEL == CONFIG_GPS_LOG_LEVEL_WARN) // "Log if something unwanted happened but didn't cause a problem"

#define DLOG(a, b, ...) ((void)0)
#define DMEAS_START() ((void)0)
#define DMEAS_END(a, b, ...) ((void)0)
#define ILOG(a, b, ...) ((void)0)
#define IMEAS_START() ((void)0)
#define IMEAS_END(a, b, ...) ((void)0)
#define WLOG(a, b, ...) ESP_LOGI(a, b, __VA_ARGS__)
#define WMEAS_START() uint64_t _start = (esp_timer_get_time()), _end = 0
#define WMEAS_END(a, b, ...) \
    _end = (esp_timer_get_time());  \
    ESP_LOGI(a, b, __VA_ARGS__, _end - _start)

#else // "Do not log anything"

#define DLOG(a, b, ...) ((void)0)
#define DMEAS_START() ((void)0)
#define DMEAS_END(a, b, ...) ((void)0)
#define ILOG(a, b, ...) ((void)0)
#define IMEAS_START() ((void)0)
#define IMEAS_END(a, b, ...) ((void)0)
#define WLOG(a, b, ...) ((void)0)
#define WMEAS_START() ((void)0)
#define WMEAS_END(a, b, ...) ((void)0)
#endif

#ifdef __cplusplus
}
#endif
#endif /* DB99F2E7_B596_4059_B6AF_FAD2A14CD6A0 */
