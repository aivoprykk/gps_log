# GPS Logger Component

A comprehensive GPS logging and data processing module for ESP32-based GPS tracking devices, providing multi-format logging, real-time data analysis, and file management capabilities.

## Features

### GPS Data Processing
- **Real-time GPS Data**: Latitude, longitude, speed, and satellite information
- **Distance Tracking**: Total distance, run distance, and distance calculations
- **Speed Metrics**: Advanced speed analysis with configurable metrics
- **Satellite Monitoring**: GPS satellite constellation tracking and signal quality
- **Time Synchronization**: GPS-based time setting and synchronization

### Multi-Format Logging
- **TXT Format**: Human-readable text logging with timestamps
- **UBX Format**: Raw u-blox binary messages for detailed analysis
- **SBP Format**: Swift Navigation Binary Protocol format
- **GPX Format**: GPS Exchange Format for mapping applications
- **GPY Format**: Custom binary format for high-performance logging

### File Management
- **SD Card Support**: Automatic file creation and management
- **Multi-File Logging**: Simultaneous logging to multiple formats
- **File Rotation**: Configurable file naming and rotation
- **Error Handling**: Robust file operation error recovery
- **MAC Address Integration**: Device-specific file naming

### Data Buffering
- **Configurable Buffers**: Adjustable buffer sizes for different use cases
- **Speed Buffer**: Ground speed data buffering (5128 samples default)
- **Alpha Buffer**: Speed calculation buffer (2000 samples default)
- **Satellite Buffer**: NAV-SAT message storage (10 messages default)

### Performance Monitoring
- **Frame Statistics**: GPS message reception statistics
- **Error Tracking**: Message parsing error monitoring
- **Timing Analysis**: GPS update rate and timing verification
- **Memory Usage**: Buffer utilization monitoring

### Display Integration
- **Statistics Screens**: GPS data visualization screens
- **Real-time Updates**: Live GPS status display
- **Signal Quality**: Satellite signal strength indicators
- **Speed Graphs**: Visual speed and distance representations

## Installation

### ESP-IDF Integration
Add to your `main/CMakeLists.txt`:
```cmake
idf_component_register(SRCS "main.c"
                      INCLUDE_DIRS "."
                      REQUIRES gps_log ublox)
```

### PlatformIO
Add to your `platformio.ini`:
```ini
[env]
lib_deps =
    https://github.com/aivoprykk/esp-gps-logger.git#components/gps_log
```

## Configuration

### Kconfig Options
Configure via `idf.py menuconfig`:

- **GPS_LOG_ENABLED**: Enable/disable GPS logging module
- **GPS_BUFFER_SIZE**: Ground speed buffer size (default 5128)
- **GPS_ALFA_BUFFER_SIZE**: Alpha calculation buffer (default 2000)
- **GPS_NAV_SAT_BUFFER_SIZE**: Satellite info buffer (default 10)
- **GPS_LOG_STACK_SIZE**: Task stack size (default 3072)
- **GPS_LOG_ENABLE_GPY**: Enable GPY format logging
- **GPS_SPEED_ERROR_LOGGING**: Enable detailed speed error logging

### JSON Library Selection
Choose between:
- **CCAN JSON**: Lightweight JSON library (default)
- **CJSON**: Full-featured JSON library

## Usage

### Basic Initialization
```c
#include "gps_log.h"
#include "gps_data.h"

// Initialize GPS context
gps_context_t gps_context = CONTEXT_GPS_DEFAULT_CONFIG();
gps_context.mac_address = get_mac_address();
gps_context.SW_version = "1.0.0";

// Initialize GPS logging
gps_init(&gps_context);

// Start GPS operations
if (gps_start() == ESP_OK) {
    ESP_LOGI(TAG, "GPS logging started");
}
```

### File Logging Setup
```c
#include "gps_log_file.h"

// Initialize log configuration
gps_log_file_config_t *log_config = log_config_init();
strcpy(log_config->base_path, "/sdcard");
strcpy(log_config->filename_NO_EXT, "gps_track");

// Open log files
open_files(&gps_context);

// Enable specific log formats
log_config->log_file_bits |= (1 << SD_TXT);  // Enable TXT logging
log_config->log_file_bits |= (1 << SD_GPX);  // Enable GPX logging
log_config->log_file_bits |= (1 << SD_UBX);  // Enable UBX logging
```

### GPS Data Access
```c
// Access GPS data
float latitude = gps_context.Ublox.latitude;
float longitude = gps_context.Ublox.longitude;
float speed = gps_context.gps_speed / 1000.0f;  // Convert to m/s
float total_distance = gps_context.Ublox.total_distance;

// Check GPS status
bool has_fix = gps_context.Gps_fields_OK;
bool is_moving = gps_context.gps_is_moving;
uint32_t first_fix_time = gps_context.first_fix;
```

### Speed Metrics
```c
// Access speed metrics
gps_speed_metrics_desc_t *metrics = gps_context.speed_metrics;
for (int i = 0; i < gps_context.num_speed_metrics; i++) {
    ESP_LOGI(TAG, "Speed metric %d: %.2f km/h", i, metrics[i].speed_kmh);
}

// Get maximum speed
gps_run_t max_speed = gps_context.max_speed;
ESP_LOGI(TAG, "Max speed: %.2f km/h at %lu", max_speed.speed, max_speed.time);
```

### Satellite Information
```c
// Access satellite data
gps_sat_info_t *sat_info = &gps_context.Ublox_Sat;
ESP_LOGI(TAG, "Satellites visible: %d", sat_info->num_sats_visible);
ESP_LOGI(TAG, "Satellites used: %d", sat_info->num_sats_used);

for (int i = 0; i < sat_info->num_sats_visible; i++) {
    gps_satellite_t *sat = &sat_info->satellites[i];
    ESP_LOGI(TAG, "Sat %d: PRN=%d, CNO=%d, elevation=%d",
             i, sat->prn, sat->cno, sat->elevation);
}
```

### Logging Control
```c
// Log current GPS data to all enabled formats
log_to_file(&gps_context);

// Flush all open files
flush_files(&gps_context);

// Close all log files
close_files(&gps_context);

// Check if files are opened
bool files_open = log_files_opened(&gps_context);
```

### Event Handling
```c
#include "gps_log_events.h"

// Register for GPS events
esp_event_handler_register(GPS_LOG_EVENT, ESP_EVENT_ANY_ID, gps_event_handler, NULL);

static void gps_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data) {
    switch (event_id) {
        case GPS_LOG_EVENT_FIX_ACQUIRED:
            ESP_LOGI(TAG, "GPS fix acquired");
            break;
        case GPS_LOG_EVENT_FIX_LOST:
            ESP_LOGI(TAG, "GPS fix lost");
            break;
        case GPS_LOG_EVENT_LOGGING_STARTED:
            ESP_LOGI(TAG, "GPS logging started");
            break;
    }
}
```

## API Reference

### Core Functions
- `gps_init()` / `gps_deinit()`: Initialize/deinitialize GPS module
- `gps_start()` / `gps_shut_down()`: Start/stop GPS operations
- `gps_read_msg_timeout()`: Read GPS messages with timeout
- `gps_has_version_set()`: Check if GPS version is configured

### File Operations
- `open_files()` / `close_files()`: Open/close log files
- `flush_files()`: Flush all open files
- `log_to_file()`: Write current GPS data to files
- `log_files_opened()`: Check file status

### Data Access
- GPS context structure provides access to all GPS data
- Speed metrics and satellite information
- Distance and timing calculations
- Signal quality and fix status

## Logging Formats

### TXT Format
Human-readable text format with timestamps:
```
2024-01-01 12:00:00, 60.123456, 24.654321, 45.6, 8
```

### GPX Format
Standard GPS Exchange Format for mapping applications:
```xml
<gpx version="1.0">
  <trk>
    <trkseg>
      <trkpt lat="60.123456" lon="24.654321">
        <time>2024-01-01T12:00:00Z</time>
        <speed>12.5</speed>
      </trkpt>
    </trkseg>
  </trk>
</gpx>
```

### UBX Format
Raw u-blox binary messages for detailed analysis and replay.

### SBP Format
Swift Navigation Binary Protocol for high-precision applications.

### GPY Format
Custom binary format optimized for ESP32 storage and processing.

## Performance Considerations

### Buffer Sizing
- **Speed Buffer**: Larger buffers needed for high-frequency logging (10Hz/20Hz)
- **Alpha Buffer**: Sized for speed calculation windows (typically 500m distance)
- **Satellite Buffer**: Stores recent NAV-SAT messages for averaging

### Memory Usage
- **Context Structure**: ~200 bytes for GPS context
- **Buffers**: Configurable based on use case (default ~20KB)
- **File Handles**: 5 file descriptors for multi-format logging

### CPU Usage
- **Logging Frequency**: 1Hz default, configurable up to 20Hz
- **File Operations**: SD card I/O can impact performance
- **Data Processing**: Real-time calculations for speed and distance

## Troubleshooting

### Common Issues
1. **No GPS Fix**: Check antenna connection and satellite visibility
2. **File Write Errors**: Verify SD card mounting and permissions
3. **Buffer Overflows**: Increase buffer sizes for high-frequency logging
4. **Memory Issues**: Monitor heap usage with large buffers

### Debug Information
```c
// Check GPS status
ESP_LOGI(TAG, "GPS fix: %s", gps_context.Gps_fields_OK ? "OK" : "NO");
ESP_LOGI(TAG, "Satellites: %d used, %d visible",
         gps_context.Ublox_Sat.num_sats_used,
         gps_context.Ublox_Sat.num_sats_visible);

// Check file status
ESP_LOGI(TAG, "Files opened: %s", log_files_opened(&gps_context) ? "YES" : "NO");
for (int i = 0; i < SD_FD_END; i++) {
    if (log_config.filefds[i] >= 0) {
        ESP_LOGI(TAG, "File %d: %s", i, log_config.filenames[i]);
    }
}
```

### Log Levels
Configure via Kconfig:
- TRACE: Detailed GPS message parsing and timing
- DEBUG: Buffer operations and file I/O
- INFO: GPS fix status and major events
- ERROR: Critical errors and failures

## Dependencies

- ESP-IDF v4.4+
- ublox component (GPS receiver interface)
- logger_common component
- VFS or logger_vfs component
- JSON library (CCAN or CJSON)

## Hardware Compatibility

### GPS Modules
- u-blox NEO-M8N, NEO-M8P, etc.
- Any UART-based GPS receiver with NMEA/UBX support

### Storage
- SD card via SPI or SDMMC interface
- FAT32 file system support
- Sufficient storage for logging duration

## License

See LICENSE file in component directory.

