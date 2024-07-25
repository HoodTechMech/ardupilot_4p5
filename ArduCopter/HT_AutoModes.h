#pragma once
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>

// takeoff params
#define TAKEOFF_PITCHROLL_LIM   500.0f    // limit in degrees for roll/pitch at takeoff.
#define TAKEOFF_MIN_ALT         50.0f   // Min height the copter will do in takeoff hop
#define TAKEOFF_MAX_ALT         400.0f  // max height copter will do in take off hop.

#define LOG_PD                  50      // msec
#define COMMUNIC_SAFE                   1000
#define COMMUNIC_UNSAFE                 2000