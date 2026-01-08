#include "gps_track_generator.h"
#include "config.h"
#include <math.h>
#include <stdio.h>

#define DEG_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEG (180.0 / M_PI)
#define EARTH_RADIUS_M 6371000.0

// Convert meters to degrees latitude (approximate)
static double meters_to_lat(double meters) {
    return meters / 111320.0;  // 1 degree ≈ 111.32 km at equator
}

// Convert meters to degrees longitude at given latitude
static double meters_to_lon(double meters, double lat) {
    return meters / (111320.0 * cos(lat * DEG_TO_RAD));
}

const char* gps_track_phase_name(gps_track_state_t *state) {
    switch (state->phase) {
        case TRACK_PHASE_START: return "START";
        case TRACK_PHASE_ACCEL1: return "ACCEL1_600m";
        case TRACK_PHASE_TURN: return "TURN_25m";
        case TRACK_PHASE_ACCEL2: return "ACCEL2_50m";
        case TRACK_PHASE_RETURN: return "RETURN";
        case TRACK_PHASE_COMPLETE: return "COMPLETE";
        default: return "UNKNOWN";
    }
}

void gps_track_init(gps_track_state_t *state, float rate_hz, double start_lat, double start_lon) {
    state->lat = start_lat;
    state->lon = start_lon;
    state->speed_ms = 0.0f;
    state->heading_deg = 0.0f;  // Start heading north
    state->phase = TRACK_PHASE_START;
    state->phase_distance_m = 0.0f;
    state->sample_count = 0;
    state->phase_samples = 0;
    state->rate_hz = rate_hz;
    state->start_lat = start_lat;
    state->start_lon = start_lon;
}

void gps_track_reset(gps_track_state_t *state) {
    // Reset to start position and initial state
    state->lat = state->start_lat;
    state->lon = state->start_lon;
    state->speed_ms = 0.0f;
    state->heading_deg = 0.0f;
    state->phase = TRACK_PHASE_START;
    state->phase_distance_m = 0.0f;
    state->phase_samples = 0;
    // Keep sample_count running for total test tracking
}

bool gps_track_next_sample(gps_track_state_t *state, double *lat, double *lon, float *speed_ms, float *heading) {
    if (state->phase == TRACK_PHASE_COMPLETE) {
        return false;
    }
    
    float dt = 1.0f / state->rate_hz;  // Time step in seconds
    float distance_step_m = 0.0f;      // Distance traveled this step
    
    // Update speed and position based on current phase
    switch (state->phase) {
        case TRACK_PHASE_START:
            // Dwell at start for 1 second
            if (state->phase_samples >= (uint32_t)state->rate_hz) {
                state->phase = TRACK_PHASE_ACCEL1;
                state->phase_distance_m = 0.0f;
                state->phase_samples = 0;
            }
            break;
            
        case TRACK_PHASE_ACCEL1: {
            // Accelerate from 0 to 70 km/h over 600m
            // Use constant acceleration: v² = v0² + 2*a*d
            // a = (v_max² - v0²) / (2 * distance)
            float v_max = TRACK_MAX_SPEED_MS;
            float a = (v_max * v_max) / (2.0f * TRACK_ACCEL1_DIST_M);
            
            // Update speed: v = v0 + a*dt
            state->speed_ms += a * dt;
            if (state->speed_ms > v_max) state->speed_ms = v_max;
            
            // Move north
            distance_step_m = state->speed_ms * dt;
            state->lat += meters_to_lat(distance_step_m);
            state->heading_deg = 0.0f;  // North
            
            state->phase_distance_m += distance_step_m;
            if (state->phase_distance_m >= TRACK_ACCEL1_DIST_M) {
                state->phase = TRACK_PHASE_TURN;
                state->phase_distance_m = 0.0f;
                state->phase_samples = 0;
            }
            break;
        }
        
        case TRACK_PHASE_TURN: {
            // Turn with radius ~25m, decelerate from 70 to 20 km/h
            // Circular arc: distance = radius * angle
            float radius = TRACK_TURN_RADIUS_M;
            float turn_distance = M_PI * radius;  // 180 degree turn
            
            // Deceleration
            float v_start = TRACK_MAX_SPEED_MS;
            float v_end = TRACK_TURN_SPEED_MS;
            float a = -(v_start * v_start - v_end * v_end) / (2.0f * turn_distance);
            
            state->speed_ms += a * dt;
            if (state->speed_ms < v_end) state->speed_ms = v_end;
            
            // Move along circular arc
            distance_step_m = state->speed_ms * dt;
            float angle_rad = distance_step_m / radius;
            
            // Update heading (turning right, so heading increases)
            state->heading_deg += angle_rad * RAD_TO_DEG;
            if (state->heading_deg > 180.0f) state->heading_deg = 180.0f;  // South
            
            // Update position (circular motion)
            double heading_rad = state->heading_deg * DEG_TO_RAD;
            state->lat += meters_to_lat(distance_step_m * cos(heading_rad));
            state->lon += meters_to_lon(distance_step_m * sin(heading_rad), state->lat);
            
            state->phase_distance_m += distance_step_m;
            if (state->phase_distance_m >= turn_distance) {
                state->phase = TRACK_PHASE_ACCEL2;
                state->phase_distance_m = 0.0f;
                state->phase_samples = 0;
                state->heading_deg = 180.0f;  // South
            }
            break;
        }
        
        case TRACK_PHASE_ACCEL2: {
            // Accelerate from 20 to 70 km/h over 50m
            float v_start = TRACK_TURN_SPEED_MS;
            float v_max = TRACK_MAX_SPEED_MS;
            float a = (v_max * v_max - v_start * v_start) / (2.0f * TRACK_ACCEL2_DIST_M);
            
            state->speed_ms += a * dt;
            if (state->speed_ms > v_max) state->speed_ms = v_max;
            
            // Move south
            distance_step_m = state->speed_ms * dt;
            state->lat -= meters_to_lat(distance_step_m);
            state->heading_deg = 180.0f;  // South
            
            state->phase_distance_m += distance_step_m;
            if (state->phase_distance_m >= TRACK_ACCEL2_DIST_M) {
                state->phase = TRACK_PHASE_RETURN;
                state->phase_distance_m = 0.0f;
                state->phase_samples = 0;
            }
            break;
        }
        
        case TRACK_PHASE_RETURN: {
            // Decelerate from 70 km/h to 0 over remaining distance back to start
            float distance_to_start = fabs(state->lat - state->start_lat) * 111320.0;
            
            if (distance_to_start < 1.0f) {
                // Close enough - stop
                state->speed_ms = 0.0f;
                state->lat = state->start_lat;
                state->lon = state->start_lon;
                state->phase = TRACK_PHASE_COMPLETE;
            } else {
                // Decelerate: v² = v0² - 2*a*d
                float v_current = state->speed_ms;
                float a = (v_current * v_current) / (2.0f * distance_to_start);
                
                state->speed_ms -= a * dt;
                if (state->speed_ms < 0.0f) state->speed_ms = 0.0f;
                
                // Move south toward start
                distance_step_m = state->speed_ms * dt;
                state->lat -= meters_to_lat(distance_step_m);
                state->heading_deg = 180.0f;  // South
                
                state->phase_distance_m += distance_step_m;
            }
            break;
        }
        
        case TRACK_PHASE_COMPLETE:
            return false;
    }
    
    // Output current state
    *lat = state->lat;
    *lon = state->lon;
    *speed_ms = state->speed_ms;
    *heading = state->heading_deg;
    
    state->sample_count++;
    state->phase_samples++;
    
    return true;
}
