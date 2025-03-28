#ifndef BD17E2A6_4D79_405F_8807_A3829F226F80
#define BD17E2A6_4D79_405F_8807_A3829F226F80

#ifdef __cplusplus
extern "C" {
#endif

struct gps_context_s;

void gps_task_start(void);
void gps_task_stop(void);
void gps_init(struct gps_context_s * _gps);
void gps_deinit(void);
int gps_shut_down(void);

#ifdef __cplusplus
}
#endif

#endif /* BD17E2A6_4D79_405F_8807_A3829F226F80 */
