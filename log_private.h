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

#ifdef __cplusplus
}
#endif
#endif /* DB99F2E7_B596_4059_B6AF_FAD2A14CD6A0 */
