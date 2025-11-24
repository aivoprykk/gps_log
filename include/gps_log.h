#ifndef BD17E2A6_4D79_405F_8807_A3829F226F80
#define BD17E2A6_4D79_405F_8807_A3829F226F80

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
struct gps_context_s;

void gps_init(struct gps_context_s * _gps);
void gps_deinit(void);
int gps_start(void);
int gps_shut_down(void);
uint8_t gps_read_msg_timeout(uint8_t magnitude);
uint8_t gps_has_version_set(void);

#ifdef __cplusplus
}
#endif

#endif /* BD17E2A6_4D79_405F_8807_A3829F226F80 */
