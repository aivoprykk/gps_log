
SET(SRCS gps_log_core.c gps_log_file.c log_gpx.c log_gpy.c log_sbp.c log_ubx.c)
SET(INCLUDE include)
idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS ${INCLUDE}
    PRIV_REQUIRES logger_ubx logger_common logger_vfs logger_config logger_str
)
