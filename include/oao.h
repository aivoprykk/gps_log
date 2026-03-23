#ifndef D73B5004_92D6_4897_AA4A_F9C0730B5E4B
#define D73B5004_92D6_4897_AA4A_F9C0730B5E4B
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>

struct gps_context_s;

/// !!! oao format write is not actually prohibited by owner
/// https://www.motion-gps.com/motion/documentation/oao-file-format.html
/// we just include this code in the gps_log component but this is not default enabled

union __attribute__((__packed__)) OAO_Frame {

  struct {

    uint16_t  mode;

    uint8_t   checksum_a;
    uint8_t   checksum_b;

    union {

      struct {
        int32_t   latitude;
        int32_t   longitude;              /* Mode Track 0x0AD1 stops here */

        int32_t   altitude;
        uint32_t  speed;
        uint32_t  heading;

        uint64_t  utc_gnss;

        union {
          struct {
            uint16_t  identifier;         /* Mode Emergency 0x0AD2 stops here */
          } __attribute__((__packed__));

          struct {
            uint8_t   fix;
            uint8_t   satellites;         /* Mode POI 0x0AD3 stops here */

            uint32_t  accuracy_speed;
            uint32_t  accuracy_horizontal;
            uint32_t  accuracy_vertical;
            uint32_t  accuracy_heading;
            uint16_t  accuracy_hDOP;      /* Mode GNSS 0x0AD4 or 0x0AD5 stops here */
          } __attribute__((__packed__));
        };
      } __attribute__((__packed__));

      struct {
        uint64_t  utc_imu;

        int16_t   attitude_w;
        int16_t   attitude_x;
        int16_t   attitude_y;
        int16_t   attitude_z;

        int16_t   angular_velocity_x;
        int16_t   angular_velocity_y;
        int16_t   angular_velocity_z;

        int16_t   linear_acceleration_x;
        int16_t   linear_acceleration_y;
        int16_t   linear_acceleration_z;  /* Mode IMU 0x0AD6 stops here, unused past Motion 1600 */
      } __attribute__((__packed__));

    };

  } __attribute__((__packed__));

  uint8_t bytes_track     [12];
  uint8_t bytes_imu       [32];
  uint8_t bytes_emergency [34];
  uint8_t bytes_poi       [34];
  uint8_t bytes_gnss      [52];
  uint8_t bytes           [52];

};

typedef struct best_t {
  uint32_t utc_seconds;
  uint32_t value;
} __attribute__((__packed__)) best_t;

union __attribute__((__packed__)) OAO_Header {

  struct {

    uint16_t  mode;               /* Mode Header 0x0AD0 */

    uint8_t   checksum_a;
    uint8_t   checksum_b;

    uint16_t  identifier;
    char      nickname[10];

    uint64_t  start_date;
    int32_t   start_latitude;
    int32_t   start_longitude;
    int32_t   start_altitude;

    uint64_t  end_date;
    int32_t   end_latitude;
    int32_t   end_longitude;
    int32_t   end_altitude;

    uint32_t  distance;

    int32_t   minimum_latitude;
    int32_t   minimum_longitude;
    int32_t   minimum_altitude;
    uint32_t  minimum_speed;

    int32_t   maximum_latitude;
    int32_t   maximum_longitude;
    int32_t   maximum_altitude;
    uint32_t  maximum_speed;

    uint32_t  above_12kn;
    uint32_t  above_12kn_seconds;

    best_t    bests_over_1s   [5];
    best_t    bests_over_10s  [5];
    best_t    bests_over_1h   [5];
    best_t    bests_over_500m [5];
    best_t    bests_over_1000m[5];
    best_t    bests_over_1852m[5];
    best_t    bests_gybe_min  [5];

    uint32_t  elevation_gain;

    uint8_t   unused   [64];
    uint8_t   signature[64];

  } __attribute__((__packed__));

  uint8_t bytes[512];

};

void log_header_OAO(struct gps_context_s *context);
void log_OAO(struct gps_context_s *context);

#ifdef __cplusplus
}
#endif

#endif /* D73B5004_92D6_4897_AA4A_F9C0730B5E4B */
