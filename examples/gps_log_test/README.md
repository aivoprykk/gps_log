# GPS Log Integration Test

## Purpose

End-to-end stress test for the GPS log module's production pipeline:
- **UBX Message Parsing** - Validate NAV-PVT/DOP/SAT message handling
- **Speed Calculations** - Test M500, S10, alfa tracking with realistic speedsurfing data  
- **File I/O** - Measure write throughput and flush latency under load
- **Pipeline Performance** - Identify bottlenecks and maximum sustainable rates

## Architecture

### What's Real (Production Code)
✅ `gpsTask` - Main GPS processing task with two-phase event loop  
✅ `ubx_parse()` - UBX protocol parser and message handler  
✅ `push_gps_data()` - Data ingestion with alfa/sec ring buffers  
✅ Speed calculations - M500, S10, alfa window computations  
✅ `log_write()` / `flush_files()` - Real file I/O to VFS  
✅ `vfs_init()` - Production filesystem mounting (SD/FATFS/SPIFFS)

### What's Mocked
🎭 **UART input stream** - Generates UBX bytes from GPS track simulation  
🎭 **GPS track generator** - 600m speedsurfing loop with realistic physics

**Key Point**: Only the UART hardware is mocked. All GPS processing logic is real production code.

## Test Scenario

### Test Phases

The test simulates a complete GPS session lifecycle:

**1. GPS Initialization Phase** (16-20 seconds)
- Start with 0 satellites
- Gradually acquire satellites (1 every 2 seconds → 8 total)
- Simulates real GPS cold start behavior
- Tests UBX NAV-SAT message handling

**2. First Fix Achievement** (~16 seconds elapsed)
- When 5+ satellites locked: GPS time becomes valid
- GPS module sets `time_set=1` and `signal_ok=1` flags
- Tests time synchronization and signal validation logic

**3. Stationary Phase** (20 seconds)
- GPS stays stationary with valid fix
- `GPS_delay` counter increments every 100ms
- After threshold (~50): GPS module opens log files
- Tests file initialization and delay logic

**4. Speedsurfing Phase** (120 seconds)
- 600m speedsurfing loop with realistic speed profile
- Tests M500/S10/alfa calculations under dynamic movement
- Validates file I/O at configured save frequency
- Measures pipeline performance and limits

**Total Test Duration**: ~156 seconds (2 minutes 36 seconds)

Set `INIT_PHASE_ENABLED 0` in [config.h](main/config.h) to skip phases 1-3 and test speedsurfing only.

### GPS Track Profile
**600m Speedsurfing Loop** simulating real-world high-performance tracking:

1. **Start**: 58.3780°N, 26.7290°E, 0 m/s
2. **Acceleration Phase 1**: 600m straight, 0→70 km/h (19.4 m/s)
3. **Turn Phase**: 25m radius, 70→20 km/h (deceleration)
4. **Acceleration Phase 2**: 50m straight, 20→70 km/h
5. **Return Phase**: Decelerate to start, 70→0 km/h

### Test Rates
- **10Hz** (Normal) - Typical GPS update rate
- **20Hz** (Edge) - High-rate real-world tracking
- **30Hz** (Stress) - Maximum sustainable rate

Change `TEST_GPS_RATE` in [config.h](main/config.h) to test different rates.

### Save Frequency Testing

Test validates correct speed calculations and logging at different save intervals:

- **1Hz** (`TEST_SAVE_FREQ_1HZ`) - Save every 1 second (file I/O stress test)
- **5Hz** (`TEST_SAVE_FREQ_5HZ`) - Save every 5 seconds (typical usage)
- **10Hz** (`TEST_SAVE_FREQ_10HZ`) - Save every 10 seconds (low frequency)

Change `TEST_SAVE_FREQUENCY` in [config.h](main/config.h) to test different save rates.

This tests:
- Ring buffer integrity at different flush intervals
- M500/S10 calculation accuracy with varying data retention
- File I/O performance vs GPS throughput
- Log file size and storage efficiency

## Building and Running

```bash
cd components/gps_log/examples/gps_log_test

# Set up ESP-IDF environment
source ~/esp/esp-idf/export.sh

# Build
idf.py build

# Flash and monitor
idf.py -p /dev/cu.usbserial-XXXXX flash monitor
```

Press `Ctrl+]` to exit monitor.

## Expected Output

```
=== GPS Log Integration Test ===

TEST PHASES:
  1. Initialization: 8 satellites acquisition (0→8, 2000ms interval)
  2. First Fix: 5+ satellites, GPS time set
  3. Stationary: 20s for GPS_delay increment & file opening
  4. Speedsurfing: 120s at 10 Hz, 600m loop

Total Duration: ~156 seconds
Save Frequency: Every 5 seconds
Track: 600m loop with speed profile 0→70→20→70→0 km/h

Initializing VFS (production filesystem handling)
Mounted: /sd (type=1, total=15523840, free=15511552)

Initializing GPS module
Starting GPS processing task
Mock UART task started

========== TEST STATISTICS ==========
Test Phase: GPS INITIALIZATION
Test Rate: 10 Hz | Elapsed: 4 s / 156 s
Satellites: 2 | Track Phase: STATIONARY
Track: lat=58.378000 lon=26.729000 speed=0.0 m/s heading=0.0°
Samples Generated: 40 (10.0 samples/s)

REAL GPS PIPELINE STATS:
  UBX Messages: total=40 pvt=40 sat=4 errors=0
  GPS Context: time_set=0 signal_ok=0 files_open=0 GPS_delay=0
  GPS Data: total_dist=0.0 M500=0.0 S10=0.0 alfa=0.0
  Log Files: flush_count=0
=====================================

Satellites acquired: 3/8
Satellites acquired: 4/8
Satellites acquired: 5/8
=== FIRST FIX ACHIEVED (5 satellites) ===
=== ENTERING STATIONARY PHASE ===

========== TEST STATISTICS ==========
Test Phase: STATIONARY (GPS_delay increment)
Test Rate: 10 Hz | Elapsed: 25 s / 156 s
Satellites: 5 | Track Phase: STATIONARY
Track: lat=58.378000 lon=26.729000 speed=0.0 m/s heading=0.0°
Samples Generated: 250 (10.0 samples/s)

REAL GPS PIPELINE STATS:
  UBX Messages: total=250 pvt=250 sat=25 errors=0
  GPS Context: time_set=1 signal_ok=1 files_open=1 GPS_delay=67
  GPS Data: total_dist=0.0 M500=0.0 S10=0.0 alfa=0.0
  Log Files: flush_count=0
=====================================

=== STARTING SPEEDSURFING TEST ===
Track: 600m loop, 10 Hz, save every 5 seconds

========== TEST STATISTICS ==========
Test Phase: SPEEDSURFING TEST
Test Rate: 10 Hz | Elapsed: 45 s / 156 s
Satellites: 8 | Track Phase: ACCELERATION_1
Track: lat=58.381234 lon=26.729000 speed=15.2 m/s heading=0.0°
Samples Generated: 450 (10.0 samples/s)

REAL GPS PIPELINE STATS:
  UBX Messages: total=450 pvt=450 sat=45 errors=0
  GPS Context: time_set=1 signal_ok=1 files_open=1 GPS_delay=317
  GPS Data: total_dist=304.5 M500=48.2 S10=52.1 alfa=54.3
  Log Files: flush_count=4
=====================================
  GPS Context: time_set=1 signal_ok=1 files_open=1
  GPS Data: total_dist=243.5
=====================================

...

========== TEST RESULTS ==========
Test Duration: 120.0 seconds
Total Samples: 1200
Throughput: 10.0 samples/sec
Message Success Rate: 100.0% (1200/1200)
Parse Errors: 0

Test complete - SUCCESS
==================================
```

## What to Look For

### Performance Metrics
- **Throughput**: Samples/sec should match `TEST_GPS_RATE` (±5%)
- **Message Success Rate**: Should be >99% (pvt_count / total_msg)
- **Parse Errors**: Should be 0 or very low (<0.1%)
- **File I/O**: Check flush_count increases periodically
- **Distance**: Total should be ~600-650m for one loop

### Bottleneck Indicators
🔴 **Queue Saturation**: Generated samples > UBX messages processed  
🔴 **Phase 2 Lag**: File I/O + speed calc falling behind message decode  
🔴 **Parse Errors**: Increasing error count indicates buffer corruption  
🔴 **Throughput Drop**: Actual rate < target rate consistently  
🔴 **Stalled Processing**: GPS stats stop updating while samples generate

### Success Criteria
✅ Test runs to completion (120 seconds)  
✅ Message success rate >99%  
✅ No parse errors or <10 total  
✅ Throughput stable at target rate  
✅ GPS distance calculations correct (~600-650m total)  
✅ File writes complete without errors

## Tuning Test Parameters

### Increase Stress Level
Edit [config.h](main/config.h):
1. Set `TEST_GPS_RATE` to `GPS_RATE_EDGE` (20Hz) or `GPS_RATE_STRESS` (30Hz)
2. Reduce `TEST_DURATION_MS` to 60000 (1 minute) for faster iteration
3. Reduce `STATS_INTERVAL_MS` to 2000 for more frequent updates

### Modify GPS Track
Edit [gps_track_generator.c](main/gps_track_generator.c):
- `TRACK_ACCEL1_DIST_M` - Change acceleration distance (default 600m)
- `TRACK_TURN_RADIUS_M` - Tighter/wider turns (default 25m)
- `TRACK_ACCEL2_DIST_M` - Second acceleration length (default 50m)
- Speed profile - Modify calculations for different max speeds

### Test Different Scenarios

**Sprint Test**:
```c
#define TRACK_ACCEL1_DIST_M 100.0f  // Short 100m run
#define TEST_DURATION_MS 30000       // 30 seconds
```

**Endurance Test**:
```c
#define TRACK_ACCEL1_DIST_M 5000.0f  // 5km loop
#define TEST_DURATION_MS 600000      // 10 minutes
```

**Stationary Test** (idle handling):
```c
// In gps_track_generator.c, always return:
*speed_ms = 0.0f;
```

## Troubleshooting

### Build Fails

**Symptoms**: Compilation errors or linker errors

**Solutions**:
- Ensure ESP-IDF v5.5+ is sourced: `source ~/esp/esp-idf/export.sh`
- Check component paths in CMakeLists.txt match your structure
- Clean build: `rm -rf build && idf.py build`
- Verify `unified_config.h` is included in main.c

### No VFS Mount

**Symptoms**: "VFS initialization failed" or "Mount failed"

**Solutions**:
- Check sdkconfig has partition table with data partition
- Verify filesystem format configured: FAT/SPIFFS/LittleFS
- Check partition table: `idf.py partition-table`
- Format filesystem if corrupt: Add `vfs_format()` call

### Low Throughput

**Symptoms**: Actual samples/sec < target rate consistently

**Solutions**:
- Reduce `TEST_GPS_RATE` to 10Hz baseline
- Increase stack size for gpsTask: Check `GPS_TASK_STACK_SIZE` in gps_log.c
- Increase mock_uart_task stack: Change `xTaskCreate` stack param
- Check for mutex contention: Enable DEBUG logging
- Profile CPU usage: Add FreeRTOS runtime stats

### High Parse Errors

**Symptoms**: Parse error count increasing rapidly

**Solutions**:
- Verify UBX checksum calculation in generators:
  ```c
  for (i = 0; i < payload_len; i++) {
      ck_a += payload[i];
      ck_b += ck_a;
  }
  ```
- Check byte order (little-endian) for all multi-byte fields
- Increase UART event queue size in gps_log module
- Verify buffer sizes match UBX message max sizes

### GPS Stats Not Updating

**Symptoms**: "REAL GPS PIPELINE STATS" frozen while samples generate

**Solutions**:
- Check gpsTask is running: Monitor task list
- Verify UART event queue not full: Check queue watermark
- Check for deadlock: Enable Phase 2 watchdog logging
- Verify inject_ubx_message() is called: Add debug prints

## Advanced Testing

### Measure File I/O Latency

Add timing around log_write() calls in gps_log.c:
```c
uint32_t start = esp_timer_get_time();
log_write(gps, itow);
uint32_t latency = esp_timer_get_time() - start;
if (latency > 10000) {  // >10ms
    WLOG(TAG, "Slow write: %u us", latency);
}
```

### Profile CPU Usage

Enable FreeRTOS runtime stats in menuconfig:
```
Component config → FreeRTOS → Enable FreeRTOS runtime counter
```

Add to print_statistics():
```c
TaskStatus_t *task_array = malloc(uxTaskGetNumberOfTasks() * sizeof(TaskStatus_t));
uint32_t total_runtime;
int num_tasks = uxTaskGetSystemState(task_array, uxTaskGetNumberOfTasks(), &total_runtime);

for (int i = 0; i < num_tasks; i++) {
    uint32_t cpu_percent = (task_array[i].ulRunTimeCounter * 100) / total_runtime;
    printf("Task %s: %lu%% CPU\n", task_array[i].pcTaskName, cpu_percent);
}
free(task_array);
```

### Test Memory Leaks

Run for extended duration (10+ minutes) and monitor heap:
```c
printf("Free heap: %u bytes (min: %u)\n", 
       esp_get_free_heap_size(), 
       esp_get_minimum_free_heap_size());
```

Check for heap fragmentation:
```c
multi_heap_info_t info;
heap_caps_get_info(&info, MALLOC_CAP_DEFAULT);
printf("Largest free block: %u bytes\n", info.largest_free_block);
```

### Simulate GPS Signal Loss

Modify mock_uart_task() to inject periods of no GPS data:
```c
if (elapsed_ms % 30000 < 5000) {
    // Simulate 5 second signal loss every 30 seconds
    vTaskDelay(pdMS_TO_TICKS(100));
    continue;
}
```

## Known Limitations

⚠️ **UART Injection**: Uses `uart_write_bytes()` + event posting instead of real DMA.  
   Timing characteristics differ from physical UART reception.

⚠️ **No RF Environment**: Doesn't simulate signal loss, multipath, interference, or Doppler effects.

⚠️ **Simplified Satellite Data**: NAV-SAT messages use basic satellite tracking,  
   not realistic constellation geometry or signal strength variations.

⚠️ **No Time Sync**: GPS time not synchronized with system time. iTOW increments artificially.

⚠️ **Single Thread Mock**: Real GPS would have interrupt-driven UART with concurrent data arrival.

## Implementation Details

### UBX Message Injection

Messages are injected into the GPS module's UART event queue:
```c
static void inject_ubx_message(const uint8_t *data, size_t len) {
    uart_event_t event = {
        .type = UART_DATA,
        .size = len
    };
    uart_write_bytes(gps->ubx_device->uart_num, (const char*)data, len);
    xQueueSend(gps->ubx_device->uart_event_queue, &event, portMAX_DELAY);
}
```

This triggers the same event handling path as real UART hardware.

### GPS Track Generation

Track is computed using Haversine formula for accurate lat/lon:
```c
double lat1_rad = lat * M_PI / 180.0;
double lon1_rad = lon * M_PI / 180.0;
double bearing_rad = bearing * M_PI / 180.0;

double lat2_rad = asin(sin(lat1_rad) * cos(distance / R) +
                       cos(lat1_rad) * sin(distance / R) * cos(bearing_rad));
double lon2_rad = lon1_rad + atan2(sin(bearing_rad) * sin(distance / R) * cos(lat1_rad),
                                    cos(distance / R) - sin(lat1_rad) * sin(lat2_rad));
```

Earth radius R = 6371000 meters.

### UBX Checksum Calculation

Standard UBX checksum algorithm:
```c
uint8_t ck_a = 0, ck_b = 0;
for (size_t i = 2; i < len - 2; i++) {  // Skip header and checksum
    ck_a += msg[i];
    ck_b += ck_a;
}
msg[len - 2] = ck_a;
msg[len - 1] = ck_b;
```

Applied to class+id+length+payload (excludes 0xB5 0x62 header).

## Next Steps

1. **Flash to Hardware**: Test on real ESP32 with SD card or filesystem
2. **Compare with Live GPS**: Run test, then switch to real U-blox module at same rate
3. **Optimize Bottlenecks**: Use results to tune buffer sizes, task priorities, I/O strategies
4. **Add Load Variants**: Test with WiFi/HTTP active, display updates, button handling
5. **Extend Coverage**: Add tests for configuration changes, error recovery, edge cases

## License

Part of ESP-IDF GPS Logger project. See LICENSE file in repository root.
