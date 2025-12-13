#ifndef AB7E9635_4557_4135_A8A1_4B7B75B4D782
#define AB7E9635_4557_4135_A8A1_4B7B75B4D782

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "logger_common.h"
#include "config_manager.h"
#include "config_gps.h"

struct strbf_s;

void gps_user_cfg_init(void);
void gps_user_cfg_deinit(void);
#if defined(LOGGER_CONFIG_USE_OLD)
uint8_t gps_cfg_get_pos(const char *str);
uint8_t gps_cnf_set_item(uint8_t pos, void * el, uint8_t force);
int gps_config_set(const char *str, void * el, uint8_t force);
esp_err_t gps_config_decode(const char *json);
uint8_t gps_cnf_get_item(uint8_t pos, struct strbf_s * lsb, uint8_t mode);
char *gps_config_get(const char *name, struct strbf_s *sb, uint8_t mode);
char * gps_config_encode(struct strbf_s *sb, uint8_t mode, uint8_t mode2);
#endif
#ifdef __cplusplus
}
#endif

#endif /* AB7E9635_4557_4135_A8A1_4B7B75B4D782 */
