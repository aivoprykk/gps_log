#ifndef CONFIG_H
#define CONFIG_H

// Test mode selection
#define USE_REAL_GPS_PIPELINE 1    // Use real gpsTask, push_gps_data, speed calculations
#define USE_UBX_MOCK 1             // Mock UART input only

// Test rates
#define GPS_RATE_NORMAL 10         // 10Hz - normal operation
#define GPS_RATE_EDGE 20           // 20Hz - edge case
#define GPS_RATE_STRESS 30         // 30Hz - stress test (can push to 40Hz)

#define TEST_GPS_RATE GPS_RATE_NORMAL  // Change this to test different rates
#define MOCK_DATA_INTERVAL_MS (1000 / TEST_GPS_RATE)  // Auto-calculate interval

// Map GPS rate to UBX output rate enum values
#if TEST_GPS_RATE == 10
    #define TEST_UBX_OUTPUT_RATE UBX_OUTPUT_10HZ
#elif TEST_GPS_RATE == 20
    #define TEST_UBX_OUTPUT_RATE UBX_OUTPUT_20HZ
#elif TEST_GPS_RATE == 30
    #define TEST_UBX_OUTPUT_RATE 0x1E  // 30Hz (not in standard enum)
#else
    #define TEST_UBX_OUTPUT_RATE UBX_OUTPUT_10HZ  // Default
#endif

// === GPS INITIALIZATION PHASE ===
// Simulate GPS startup: 0 satellites → lock → time set → first fix
#define INIT_PHASE_ENABLED       1      // Set to 0 to skip initialization
#define INIT_START_SATELLITES    0      // Start with no satellites
#define INIT_TARGET_SATELLITES   8      // Acquire up to 8 satellites
#define INIT_SAT_INCREMENT       1      // Satellites acquired per interval
#define INIT_SAT_INTERVAL_MS     2000   // Add satellite every 2 seconds
#define INIT_MIN_SATS_FOR_FIX    5      // Minimum satellites for valid fix
#define INIT_STATIONARY_TIME_MS  20000  // Stay stationary for 20 seconds after fix

// Test duration
#define SPEEDSURFING_DURATION_MS 120000  // 2 minutes of speedsurfing
#define STATS_INTERVAL_MS        5000    // Print stats every 5 seconds

// Total test duration includes initialization + speedsurfing
#define TOTAL_TEST_DURATION_MS  (INIT_PHASE_ENABLED ? \
                                 (INIT_TARGET_SATELLITES * INIT_SAT_INTERVAL_MS + \
                                  INIT_STATIONARY_TIME_MS + SPEEDSURFING_DURATION_MS) : \
                                 SPEEDSURFING_DURATION_MS)

// Realistic GPS track simulation (600m speedsurfing loop)
// Track phases:
// 1. Start: lat=58.3780, lon=26.7290, speed=0
// 2. Accelerate: 600m straight north, 0→70 km/h (0→19.4 m/s)
// 3. Turn: ~25m radius, decelerate 70→20 km/h (19.4→5.6 m/s)
// 4. Accelerate: 50m straight, 20→70 km/h (5.6→19.4 m/s)
// 5. Return: Decelerate back to start, 70→0 km/h
#define TRACK_START_LAT 58.3780
#define TRACK_START_LON 26.7290
#define TRACK_ACCEL1_DIST_M 600.0f   // First acceleration distance
#define TRACK_TURN_RADIUS_M 25.0f    // Turn radius
#define TRACK_ACCEL2_DIST_M 50.0f    // Second acceleration distance
#define TRACK_MAX_SPEED_MS 19.4f     // 70 km/h in m/s
#define TRACK_TURN_SPEED_MS 5.6f     // 20 km/h in m/s

#endif // CONFIG_H
