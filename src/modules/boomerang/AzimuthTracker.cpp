/****************************************************************************
 * AzimuthTracker.cpp
 ****************************************************************************/

#include "AzimuthTracker.hpp"
#include <mathlib/mathlib.h>

// Out-of-line definition required for the static constexpr array in C++14.
// In C++17 this line is technically redundant but harmless; keep it for
// compatibility with PX4's mixed C++14/17 build environment.
constexpr float AzimuthTracker::BLADE_OFFSET_RAD[NUM_BLADES];

void AzimuthTracker::update(float yaw_rad_ekf, float yaw_rate_ned, hrt_abstime now_us)
{
    // -----------------------------------------------------------------------
    // Blade azimuths
    // The EKF2 yaw is the azimuth of blade 0 in the NED frame.
    // All other blades are offset by fixed multiples of 90°.
    // Wrap each angle to [-pi, pi] to keep arithmetic well-behaved.
    // -----------------------------------------------------------------------
    for (int i = 0; i < NUM_BLADES; ++i) {
        _state.theta[i] = matrix::wrap_pi(yaw_rad_ekf + BLADE_OFFSET_RAD[i]);
    }

    // -----------------------------------------------------------------------
    // Rotor angular velocity
    // yaw_rate_ned from vehicle_angular_velocity.xyz[2] (body-z) is the spin
    // rate of the entire vehicle in the NED frame.  Because the sensors ARE
    // in the rotating frame, this is also the rotor omega.
    //
    // Sign: positive omega = CCW rotation viewed from above (NED down = +Z).
    // -----------------------------------------------------------------------
    _state.omega_rad_s = yaw_rate_ned;
    _state.rpm         = (yaw_rate_ned / (2.0f * M_PI_F)) * 60.0f;
    _state.valid       = true;
    _last_update_us    = now_us;
}
