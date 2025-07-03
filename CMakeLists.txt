SET(SRCS 
gps_data.c
gps_log_file.c
log_gpx.c
log_gpy.c
log_sbp.c
log_ubx.c
dstat_screens.c
gps_log.c
gps_user_cfg.c
gps_speed_data.c
gps_satellite_data.c)
SET(INCLUDE include)
idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS ${INCLUDE}
    PRIV_REQUIRES logger_common strutil
)
if(CONFIG_UBLOX_ENABLED)
    target_link_libraries(${COMPONENT_LIB} PRIVATE idf::logger_ubx)
endif()
if(CONFIG_LOGGER_VFS_ENABLED)
    target_link_libraries(${COMPONENT_LIB} PUBLIC idf::logger_vfs)
endif()
if(NOT ${IDF_TARGET} STREQUAL "linux")
        target_link_libraries(${COMPONENT_LIB} PUBLIC idf::vfs)
endif()