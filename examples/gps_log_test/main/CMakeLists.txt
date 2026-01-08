idf_component_register(
    SRCS "main.c" "gps_track_generator.c" "ubx_nav_pvt_generator.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES 
        esp_system 
        esp_timer
        esp_event
        driver
        logger_ubx
        gps_log
        logger_common
        logger_config
        logger_context
        logger_vfs
        sconfig
        strutil
        fatfs
        wear_levelling
        ccan_json
)
