#include "log_private.h"
#include "gps_log_file.h"

#if (defined(CONFIG_UBLOX_ENABLED) && defined(CONFIG_GPS_LOG_ENABLED))

#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/unistd.h>
#include <math.h>
#include <stdio.h>

#include <esp_mac.h>

#include "gps_log.h"
#include "esp_log.h"

#include "strbf.h"
#include "numstr.h"
#include "ubx.h"
#include "gpx.h"
#include "sbp.h"
#ifdef GPS_LOG_ENABLE_GPY
#include "gpy.h"
#endif
#include "gps_log_events.h"

static const char *TAG = "gps_log";

RTC_DATA_ATTR gps_log_file_config_t log_config = GPS_LOG_DEFAULT_CONFIG();

// static const char * const speed_units[] = {
//     "m/s", "km/h","knots"
// };

//static gps_log_file_config_t log_config = GPS_LOG_DEFAULT_CONFIG();


static float get_avg(const gps_run_t *b) {
    return (float) get_avg5((const float[]){b[5].avg_speed,b[6].avg_speed,b[7].avg_speed,b[8].avg_speed,b[9].avg_speed}, get_spd, 0);
}

size_t log_write(const struct gps_context_s * context, uint8_t file, const void *msg, size_t len) {
    int fd = GET_FD(file);
    if (fd < 0)
        return fd;
    ssize_t result = write(fd, msg, len);
    if (result < 0) {
        ESP_LOGE(TAG, "Failed to write (%s) %" PRIu8, strerror(errno), file);
    }
    return result;
}

int log_close(const struct gps_context_s * context, uint8_t file) {
    int fd = GET_FD(file);
    if (fd < 0)
        return fd;
    log_fsync(context, file);
    int result = close(fd);
    if (result < 0) {
        ESP_LOGE(TAG, "Failed to close (%s) fd: %" PRIu8, strerror(errno), file);
    }
    return result;
}

int log_fsync(const struct gps_context_s * context, uint8_t file) {
    int fd = GET_FD(file);
    if (fd < 0)
        return fd;
    int result = fsync(fd);
    if (result < 0) {
        ESP_LOGE(TAG, "Failed to sync (%s) %" PRIu8, strerror(errno), file);
    }
    return result;
}

void log_err(const gps_context_t *context, const char *message) {
    if(!context) return;
    if (GETBIT(log_config.log_file_bits, SD_TXT) == 1) {
        WRITETXT(message, strlen(message));
    }
}

// esp_err_t log_config_add_config(gps_log_file_config_t * log, logger_config_t *config) {
//     if (!config || !log)
//         return ESP_ERR_INVALID_ARG;
//     log->config = config;
//     return ESP_OK;
// }

gps_log_file_config_t *log_config_init() {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s] partno: %hhu", __func__, vfs_ctx.gps_log_part);
#endif
    strbf_t buf;
    strbf_inits(&buf, log_config.base_path, ESP_VFS_PATH_MAX+1);
    if(vfs_ctx.parts[vfs_ctx.gps_log_part].mount_point) {
    struct stat sb = {0};
    int statok=-1, i = 0;
    strbf_put_path(&buf, vfs_ctx.parts[vfs_ctx.gps_log_part].mount_point);
    }
    strbf_finish(&buf);
#if (C_LOG_LEVEL < 2)
    DLOG(TAG, "[%s] log path: %s\n", __func__, &log_config.base_path[0]);
#endif
    return &log_config;
}

#ifndef FILE_APPEND
#define FILE_APPEND "a"
#endif

// esp_err_t save_log_file_bits(gps_context_t *context, uint8_t *log_file_bits) {
//     assert(context && context->log_config);
//     logger_config_t *config = context->log_config->config;
//     if (!config)
//         return ESP_ERR_INVALID_ARG;
//     if(c_gps_cfg.log_ubx)
//         SETBIT(*log_file_bits, SD_UBX);
//     if(c_gps_cfg.log_gpy)
//         SETBIT(*log_file_bits, SD_GPY);
//     if(c_gps_cfg.log_sbp)
//         SETBIT(*log_file_bits, SD_SBP);
//     if(c_gps_cfg.log_gpx)
//         SETBIT(*log_file_bits, SD_GPX);
//     if(c_gps_cfg.log_txt) 
//         SETBIT(*log_file_bits, SD_TXT);
//     return ESP_OK;
// }

bool log_files_opened(gps_context_t *context) {
    if(!context) return false;
    return context->files_opened;
}

void open_files(gps_context_t *context) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    if(!context) return;
    if (context->files_opened)
        return;
    gps_log_file_config_t *config = context->log_config;
    // logger_config_t *cfg = config->config;
    // save_log_file_bits(context, log_config.log_file_bits);
    char *filename = c_gps_cfg.ubx_file;
    while (*filename == '/' || *filename == ' ')
        ++filename;
    strbf_t sb;
    uint8_t * open_failed = 0;
    strbf_inits(&sb, config->filename_NO_EXT, PATH_MAX_CHAR_SIZE);
    //*filename_NO_EXT = 0;
    if (c_gps_cfg.file_date_time) {
        // struct tm tmstruct;
        // char extension[16] = ".txt";  //
        char timestamp[16];
        strbf_t tsb;
        struct tm tms;
        strbf_inits(&tsb, timestamp, 16);
        get_local_time(&tms);
        strbf_putl(&tsb, tms.tm_year + 1900);
        if (tms.tm_mon < 9)
            strbf_putc(&tsb, '0');
        strbf_putl(&tsb, tms.tm_mon + 1);
        if (tms.tm_mday < 10)
            strbf_putc(&tsb, '0');
        strbf_putl(&tsb, tms.tm_mday);
        if (tms.tm_hour < 10)
            strbf_putc(&tsb, '0');
        strbf_putl(&tsb, tms.tm_hour);
        if (tms.tm_min < 10)
            strbf_putc(&tsb, '0');
        strbf_putl(&tsb, tms.tm_min);
        // sprintf(timestamp, "%u%02u%02u%02u%02u", (tms.tm_year) + 1900, (tms.tm_mon) + 1, tms.tm_mday, tms.tm_hour, tms.tm_min);
        if (c_gps_cfg.file_date_time == 1) {
            strbf_puts(&sb, filename);  // copy filename from config
            strbf_putc(&sb, '_');
            strbf_puts(&sb, timestamp);  // add timestamp
        }
        if (c_gps_cfg.file_date_time == 2) {
            strbf_puts(&sb, timestamp);  // add timestamp
            strbf_putc(&sb, '_');
            strbf_puts(&sb, filename);  // copy filename from config
        }
        // strbf_puts(&sb, extension);  // add extension.txt
    } else {
        // char txt[16] = "000.txt";
        char macAddr[24];
        sprintf(macAddr, MACSTR, MAC2STR(context->mac_address));
        strbf_puts(&sb, filename);  // copy filename from config
        strbf_putc(&sb, '_');
        strbf_puts(&sb, macAddr);
        int filenameSize = sb.cur - sb.start;  // dit is dan 7 + NULL = 8
        strbf_puts(&sb, "000");                 // dit wordt dan /BN280A000.txt
        for (uint16_t i = 0; i < 1000; i++) {
            config->filename_NO_EXT[filenameSize + 2] = '0' + i % 10;
            config->filename_NO_EXT[filenameSize + 1] = '0' + ((i / 10) % 10);
            config->filename_NO_EXT[filenameSize] = '0' + ((i / 100) % 10);
            // create if does not exist, do not open existing, write, sync after
            // write
#if defined(CONFIG_LOGGER_VFS_ENABLED)
            if (!s_xfile_exists(config->filename_NO_EXT)) {
                break;
            }
#endif
        }
    }
    const char * fn = 0;
    if(!gps_log_file_bits_check(config->log_file_bits)) // at least one file must be opened
        SETBIT(config->log_file_bits, SD_SBP);
    for(uint8_t i = 0; i < SD_FD_END; i++) {
#if (C_LOG_LEVEL < 2)
        DLOG(TAG, "[%s] opening %s\n", __func__, config->filenames[i]);
#endif
        if (GETBIT(config->log_file_bits, i)) {
            fn = 
#if defined(GPS_LOG_ENABLE_GPY)
            i == SD_GPY ? ".gpy" : 
#endif
            i == SD_UBX ? ".ubx" : i == SD_SBP ? ".sbp" : i == SD_GPX ? ".gpx" : ".txt";
            strcpy(config->filenames[i], config->filename_NO_EXT);
            strcat(config->filenames[i], fn);
#if (C_LOG_LEVEL < 2)
            DLOG(TAG, "[%s] opening %s\n", __func__, config->filenames[i]);
#endif
#if defined(CONFIG_LOGGER_VFS_ENABLED)
            GET_FD(i) = s_open(config->filenames[i], config->base_path, FILE_APPEND);
            if(GET_FD(i)<=0) {
                open_failed++;
            }
            else {
                SETBIT(config->log_file_open_bits, i);
            }
#endif
        }
    }
    esp_event_post(GPS_LOG_EVENT, open_failed ? GPS_LOG_EVENT_LOG_FILES_OPEN_FAILED : GPS_LOG_EVENT_LOG_FILES_OPENED, NULL, 0, portMAX_DELAY);
    context->files_opened = 1;
}

void close_files(gps_context_t *context) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    if(!context) return;
    if (!context->files_opened) {
        return;
    }
    gps_log_file_config_t *config = context->log_config;
    for (int i = 0, j = SD_FD_END; i < j; ++i){
        if (GETBIT(config->log_file_bits, i)) {
            if(i == SD_GPX)
                log_GPX(context, GPX_END);
            log_close(context, i);
        }
    }
    context->files_opened = 0;
    esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_LOG_FILES_CLOSED, NULL,0, portMAX_DELAY);
}

static int load_balance = 0;

void flush_files(const gps_context_t *context) {
    if(!context) return;
    if (!context->files_opened) {
        return;
    }
    if (rtc_config.output_rate <= 10) {
        if (load_balance == 0) {
            log_fsync(context, SD_UBX);
        }
        if (load_balance == 1) {
            log_fsync(context, SD_TXT);
        }
#ifdef GPS_LOG_ENABLE_GPY
        if (load_balance == 2) {
            log_fsync(context, SD_GPY);
        }
#endif
        if (load_balance == 3) {
            log_fsync(context, SD_SBP);
        }
        if (load_balance == 4) {
            log_fsync(context, SD_GPX);
            load_balance = -1;
        }
        load_balance++;
    }
}

void log_to_file(gps_context_t *context) {
    if(!context || !context->files_opened) return;
    // logger_config_t *config = context->log_config->config;
    ubx_config_t *ubx = context->ubx_device;
    struct nav_pvt_s * nav_pvt = &ubx->ubx_msg.navPvt;
    if (context->time_set == 1) {
        strbf_t sb;
        char datastr[255] = {0}, buffer[56] = {0};
        strbf_inits(&sb, datastr, 255);
        if ((context->files_opened) && (ubx->ubx_msg.count_nav_pvt > 10) && gps_read_msg_timeout(1)) {  // check for timeout navPvt message !!
            // printf("sd log txt err\n");
            context->next_gpy_full_frame = 1;
            time_to_char_hms(nav_pvt->hour, nav_pvt->minute, nav_pvt->second, buffer);
            strbf_puts(&sb, buffer);
            strbf_putc(&sb, ':');
            strbf_putl(&sb, ubx->ubx_msg.count_nav_pvt);
            strbf_putc(&sb, ':');
            strbf_puts(&sb, "Lost ubx frame!\n");
            if (GET_FD(SD_TXT) > 0) {
                WRITETXT(strbf_finish(&sb), sb.cur - sb.start);
            }
            ++context->lost_frames;
            WLOG(TAG, "Lost ubx frame frame: %lu, time: %s", ubx->ubx_msg.count_nav_pvt, buffer);
            esp_event_post(GPS_LOG_EVENT, GPS_LOG_EVENT_GPS_FRAME_LOST, NULL, 0, portMAX_DELAY);
        }
        if (GETBIT(context->log_config->log_file_bits, SD_UBX)) {
            log_ubx(context, &ubx->ubx_msg, rtc_config.msgout_sat);
        }
#ifdef GPS_LOG_ENABLE_GPY
        if (GETBIT(context->log_config->log_file_bits, SD_GPY)) {
            log_GPY(context);
        }
#endif
        if (GETBIT(context->log_config->log_file_bits, SD_SBP)) {
            log_SBP(context);
        }
        if (GETBIT(context->log_config->log_file_bits, SD_GPX)) {
            log_GPX(context, GPX_FRAME);
        }
    }
}

// Prints the content of a file to the Serial
// static void printFile(gps_log_file_config_t *log, const char *filename) {
//     // Open file for reading
//     char *f = s_read_from_file(filename, log->base_path);
//     if (f) {
//         printf("%s", f);
//         free(f);
//     }
// }

#define MSGBUF_SIZE 512
static char message[MSGBUF_SIZE] = {0};
#define TXTBUF_SIZE 48
char tekst[TXTBUF_SIZE] = {0};

static void session_info(const gps_context_t *context, struct gps_data_s *G) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    if(!context) return;
    if (!context->files_opened) {
        ESP_LOGE(TAG, "[%s] files not open", __FUNCTION__);
        return;
    }
    // logger_config_t *config = context->log_config->config;
    const ubx_config_t *ubx = gps->ubx_device;
    const ubx_msg_t * ubxMessage = &ubx->ubx_msg;
    int32_t millis = get_millis();
    strbf_t sb;
    strbf_inits(&sb, &(message[0]), MSGBUF_SIZE);
    strbf_puts(&sb, "T5 MAC adress: ");
    sprintf(tekst, MACSTR, MAC2STR(gps->mac_address));
    strbf_puts(&sb, tekst);
    strbf_puts(&sb, "\n");
    strbf_puts(&sb, "GPS Logger: ");
    strbf_puts(&sb, gps->SW_version);
    strbf_puts(&sb, "\n");
    strbf_puts(&sb, "First fix : ");
    strbf_putn(&sb, MS_TO_SEC(gps->first_fix));
    strbf_puts(&sb, " s\n");
    strbf_puts(&sb, "Total time : ");
    strbf_putn(&sb, MS_TO_SEC(millis - gps->start_logging_millis));
    strbf_puts(&sb, " s\n");

    strbf_puts(&sb, "Total distance : ");
    strbf_putn(&sb, MM_TO_M(G->total_distance));
    strbf_puts(&sb, " m\n");
    strbf_puts(&sb, "Sample rate : ");
    strbf_putn(&sb, ubx->rtc_conf->output_rate);
    strbf_puts(&sb, " Hz\n");
    strbf_puts(&sb, "Speed units: ");
    strbf_puts(&sb, speed_units[c_gps_cfg.speed_unit > 2 ? 2 : c_gps_cfg.speed_unit]);
    strbf_puts(&sb, " \n");
    strbf_puts(&sb, "Timezone : ");
    strbf_putn(&sb, c_gps_cfg.timezone);
    strbf_puts(&sb, " h\n");
    strbf_puts(&sb, "Dynamic model: ");
    strbf_puts(&sb, dynamic_models[rtc_config.nav_mode==0 ? 0 : rtc_config.nav_mode-2]);
    strbf_puts(&sb, " \n");
    strbf_puts(&sb, "Ublox GNSS-enabled : ");
    strbf_putl(&sb, ubxMessage->monGNSS.enabled_Gnss);
    strbf_puts(&sb, "\n");
    strbf_puts(&sb, "GNSS = GPS + ");
    if (ubxMessage->monGNSS.enabled_Gnss == 3)
        strbf_puts(&sb, "GLONAS");
    if (ubxMessage->monGNSS.enabled_Gnss == 9)
        strbf_puts(&sb, "GALILEO");
    if (ubxMessage->monGNSS.enabled_Gnss == 11)
        strbf_puts(&sb, "GLONAS + GALILEO");
    if (ubxMessage->monGNSS.enabled_Gnss == 13)
        strbf_puts(&sb, "GLONAS + BEIDOU");
    if (ubxMessage->monGNSS.enabled_Gnss == 15)
        strbf_puts(&sb, "GLONAS + GALILEO + BEIDOU");
    strbf_puts(&sb, " \n");
    strbf_puts(&sb, "Ublox SW-version : ");
    strbf_puts(&sb, ubxMessage->mon_ver.swVersion);
    strbf_puts(&sb, " \n");
    strbf_puts(&sb, "Ublox HW-version : ");
    strbf_puts(&sb, ubxMessage->mon_ver.hwVersion);
    strbf_puts(&sb, " \n");
    ubx_hw_t ublox_hw = ubx->rtc_conf->hw_type;
    strbf_puts(&sb, "Ublox ");
    strbf_puts(&sb, ubx_chip_str(ubx));
    strbf_puts(&sb, " ID = ");
    strbf_sprintf(&sb,"%02x%02x%02x%02x%02x", &ubxMessage->ubxId.ubx_id_1, &ubxMessage->ubxId.ubx_id_2, &ubxMessage->ubxId.ubx_id_3, &ubxMessage->ubxId.ubx_id_4, &ubxMessage->ubxId.ubx_id_5);
    if (ublox_hw > UBX_TYPE_M8)
        strbf_sprintf(&sb, "%02x\n", ubxMessage->ubxId.ubx_id_6);
    WRITETXT(strbf_finish(&sb), sb.cur - sb.start);
#if (C_LOG_LEVEL < 2)
    DLOG(TAG, "[%s] %s\n", __FUNCTION__, sb.start);
#endif
}

const char * strings[] = {
    "==="
};

static void result_speed_avg(gps_speed_t *speed, strbf_t *sb, const char * units, const char * unit, uint16_t window) {
    f3_to_char(get_avg(&speed->runs[0]), tekst);
    strbf_puts(sb, tekst);
    strbf_puts(sb, units);
    strbf_putc(sb, ' ');
    strbf_puts(sb, strings[0]);
    strbf_putc(sb, ' ');
    strbf_puts(sb, unit);
    strbf_putl(sb, window);
    strbf_puts(sb, " avg 5_best_runs ");
    strbf_puts(sb, strings[0]);
    strbf_putc(sb, '\n');
}

static void gps_metrics_result_dist(struct gps_speed_by_dist_s *me) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    if(!me) return;
    if (!gps->files_opened) {
        ESP_LOGE(TAG, "[%s] files not open", __FUNCTION__);
        return;
    }
    // logger_config_t *config = gps->log_config->config;
    strbf_t sb;
    *tekst = 0;
    strbf_inits(&sb, &(message[0]), MSGBUF_SIZE);
    const char * units = speed_units[c_gps_cfg.speed_unit > 2 ? 2 : c_gps_cfg.speed_unit];
    const char * unit = " M";
    result_speed_avg(&me->speed, &sb, units, unit, me->distance_window);
    for (int i = 9; i > 4; i--) {
        gps_run_t *run = &me->speed.runs[i];
        f3_to_char(get_spd(run->avg_speed), tekst);
        strbf_puts(&sb, tekst);
        strbf_puts(&sb, units);
        strbf_putc(&sb, ' ');
        time_to_char_hms((int)run->time.hour, (int)run->time.minute, (int)run->time.second, tekst);
        strbf_puts(&sb, tekst);
        strbf_puts(&sb, " Distance: ");
        f2_to_char(get_distance_m(run->data.dist.dist, gps->ubx_device->rtc_conf->output_rate), tekst);
        strbf_puts(&sb, tekst);
        strbf_puts(&sb, " Msg_nr: ");
        strbf_putl(&sb, run->data.dist.message_nr);
        strbf_puts(&sb, " Samples: ");
        strbf_putl(&sb, run->data.dist.nr_samples);
        strbf_puts(&sb, " Run: ");
        strbf_putl(&sb, run->nr);
        strbf_puts(&sb, unit);
        strbf_putl(&sb, me->distance_window);
        strbf_puts(&sb, "\n");
        WRITETXT(strbf_finish(&sb), sb.cur - sb.start);
#if (C_LOG_LEVEL < 2)
        DLOG(TAG, "[%s] %s\n", __FUNCTION__, sb.start);
#endif
        strbf_reset(&sb);
    }
}

static void gps_metrics_result_time(struct gps_speed_by_time_s *me) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    if(!me) return;
    if (!gps->files_opened) {
        ESP_LOGE(TAG, "[%s] files not open", __FUNCTION__);
        return;
    }
    // logger_config_t *config = gps->log_config->config;
    strbf_t sb;
    *tekst = 0;
    strbf_inits(&sb, &(message[0]), MSGBUF_SIZE);
    const char * units = speed_units[c_gps_cfg.speed_unit > 2 ? 2 : c_gps_cfg.speed_unit];
    const char * unit = " S";
    result_speed_avg(&me->speed, &sb, units, unit, me->time_window);
    // errorfile.open();
    // write(sdcard_config.errorfile, message, sb.cur - sb.start);
    // ESP_LOGI(TAG, "[%s] %s", __FUNCTION__, sb.start);
    // errorfile.close();
    // appendFile(SD,filenameERR,message);
    for (int i = 9; i > 4; i--) {
        gps_run_t *run = &me->speed.runs[i];
        f3_to_char(get_spd(run->avg_speed), tekst);
        strbf_puts(&sb, tekst);
        strbf_puts(&sb, units);
        strbf_putc(&sb, ' ');
        time_to_char_hms((int)run->time.hour, (int)run->time.minute, (int)run->time.second, tekst);
        strbf_puts(&sb, tekst);
        strbf_puts(&sb, " Run:");
        strbf_putl(&sb, run->nr);
        strbf_puts(&sb, unit);
        strbf_putl(&sb, me->time_window);
        if (rtc_config.msgout_sat) {
            strbf_sprintf(&sb, " CNO Max: %u Avg: %u Min: %u nr Sat: %u\n", run->data.time.Max_cno, run->data.time.Mean_cno, run->data.time.Min_cno, run->data.time.Mean_numSat);
        } else
            strbf_puts(&sb, "\n");
        WRITETXT(message, sb.cur - sb.start);
#if (C_LOG_LEVEL < 2)
        DLOG(TAG, "[%s] %s\n", __FUNCTION__, sb.start);
#endif
        strbf_reset(&sb);
    }
}

static void gps_metrics_result_alfa(struct gps_speed_by_dist_s *me) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    if(!me || !me->alfa) return;
    if (!gps->files_opened) {
        ESP_LOGE(TAG, "[%s] files not open", __FUNCTION__);
        return;
    }
    gps_speed_by_alfa_t *A = me->alfa;
    strbf_t sb;
    strbf_inits(&sb, &(message[0]), MSGBUF_SIZE);
    const char * units = speed_units[c_gps_cfg.speed_unit > 2 ? 2 : c_gps_cfg.speed_unit];
    const char * unit = " A";
    result_speed_avg(&A->speed, &sb, units, unit, me->distance_window);
    for (int i = 9; i > 4; i--) {
        gps_run_t *run = &A->speed.runs[i];
        f3_to_char(get_spd(run->avg_speed), tekst);
        strbf_puts(&sb, tekst);
        strbf_puts(&sb, units);
        strbf_putc(&sb, ' ');
        f2_to_char(sqrt((float)run->data.alfa.real_distance), tekst);
        strbf_puts(&sb, tekst);
        strbf_puts(&sb, " m ");
        strbf_putl(&sb, get_distance_m(run->data.alfa.dist, gps->ubx_device->rtc_conf->output_rate));
        strbf_puts(&sb, " m ");
        time_to_char_hms((int)run->time.hour, (int)run->time.minute, (int)run->time.second, tekst);
        strbf_puts(&sb, tekst);
        strbf_puts(&sb, " Run: ");
        strbf_putl(&sb, run->nr);
        strbf_puts(&sb, " Msg_nr: ");
        strbf_putl(&sb, run->data.alfa.message_nr);
        strbf_puts(&sb, unit);
        strbf_putl(&sb, A->distance_window);
        strbf_puts(&sb, "\n");
        WRITETXT(strbf_finish(&sb), sb.cur - sb.start);
#if (C_LOG_LEVEL < 2)
        DLOG(TAG, "[%s] %s\n", __FUNCTION__, sb.start);
#endif
        strbf_reset(&sb);
    }
}

static void gps_metrics_result_max(void) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    if (!gps->files_opened) {
        ESP_LOGE(TAG, "[%s] files not open", __FUNCTION__);
        return;
    }
    strbf_t sb;
    strbf_inits(&sb, &(message[0]), MSGBUF_SIZE);
    gps_run_t *run = &gps->max_speed;
    f3_to_char(get_spd(run->avg_speed), tekst);
    strbf_puts(&sb, tekst);
    strbf_puts(&sb, speed_units[c_gps_cfg.speed_unit > 2 ? 2 : c_gps_cfg.speed_unit]);
    strbf_putc(&sb, ' ');
    time_to_char_hms((int)run->time.hour, (int)run->time.minute, (int)run->time.second, tekst);
    strbf_puts(&sb, tekst);
    strbf_puts(&sb, " Run: ");
    strbf_putl(&sb, run->nr);
    strbf_putc(&sb, ' ');
    strbf_puts(&sb, strings[0]);
    strbf_puts(&sb, " Max speed ");
    strbf_puts(&sb, strings[0]);
    strbf_puts(&sb, "\n");
    WRITETXT(strbf_finish(&sb), sb.cur - sb.start);
#if (C_LOG_LEVEL < 2)
    DLOG(TAG, "[%s] %s\n", __FUNCTION__, sb.start);
#endif
    strbf_reset(&sb);
}

void gps_speed_metrics_save_session(void) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    if (GETBIT(gps->log_config->log_file_bits, SD_TXT) && gps->log_config->filefds[SD_TXT] > 0) {
        session_info(gps, &gps->Ublox);
        gps_metrics_result_max();
        for(uint8_t i = 0, j = gps->num_speed_metrics; i < j; i++) {
            if (gps->speed_metrics[i].type == GPS_SPEED_TYPE_TIME) {
                gps_metrics_result_time(gps->speed_metrics[i].handle.time);
            } else if (gps->speed_metrics[i].type & (GPS_SPEED_TYPE_DIST | GPS_SPEED_TYPE_ALFA)) {
                gps_metrics_result_dist(gps->speed_metrics[i].handle.dist);
                if (gps->speed_metrics[i].type & (GPS_SPEED_TYPE_ALFA)) {
                    gps_metrics_result_alfa(gps->speed_metrics[i].handle.dist);
                }
            }
        }
    }
}

#endif
