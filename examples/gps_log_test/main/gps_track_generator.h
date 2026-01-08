#ifndef GPS_TRACK_GENERATOR_H
#define GPS_TRACK_GENERATOR_H

#include <stdint.h>
#include <stdbool.h>

// GPS track state for realistic speedsurfing simulation
typedef struct {
    // Current position
    double lat;
    double lon;
    float speed_ms;      // Speed in m/s
    float heading_deg;   // Heading in degrees
    
    // Track phase
    enum {
        TRACK_PHASE_START,
        TRACK_PHASE_ACCEL1,   // 600m straight, 0→70 km/h
        TRACK_PHASE_TURN,     // Turn with radius ~25m, 70→20 km/h
        TRACK_PHASE_ACCEL2,   // 50m straight, 20→70 km/h
        TRACK_PHASE_RETURN,   // Return to start, 70→0 km/h
        TRACK_PHASE_COMPLETE
    } phase;
    
    // Phase progress
    float phase_distance_m;   // Distance traveled in current phase
    uint32_t sample_count;    // Total samples generated
    uint32_t phase_samples;   // Samples in current phase
    
    // Track configuration
    float rate_hz;           // GPS sample rate (10/20/30 Hz)
    float start_lat;
    float start_lon;
    
} gps_track_state_t;

/**
 * Initialize GPS track generator
 * @param state Track state structure
 * @param rate_hz GPS sample rate (10/20/30 Hz)
 * @param start_lat Starting latitude
 * @param start_lon Starting longitude
 */
void gps_track_init(gps_track_state_t *state, float rate_hz, double start_lat, double start_lon);

/**
 * Reset GPS track to start position (restart the loop)
 * @param state Track state
 */
void gps_track_reset(gps_track_state_t *state);

/**
 * Generate next GPS sample on the track
 * @param state Track state
 * @param lat Output latitude
 * @param lon Output longitude
 * @param speed_ms Output speed in m/s
 * @param heading Output heading in degrees
 * @return true if track still running, false if complete
 */
bool gps_track_next_sample(gps_track_state_t *state, double *lat, double *lon, float *speed_ms, float *heading);

/**
 * Get current phase name for debugging
 */
const char* gps_track_phase_name(gps_track_state_t *state);

#endif // GPS_TRACK_GENERATOR_H
