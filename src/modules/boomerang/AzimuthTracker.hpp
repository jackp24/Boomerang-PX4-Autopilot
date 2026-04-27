/****************************************************************************
 * AzimuthTracker.hpp
 *
 * Maintains the current azimuth angle (radians, NED) for each of the 4
 * blades.  Blade 0 is defined as the blade whose azimuth is reported
 * directly by EKF2 yaw.  Blades 1-3 trail by 90°, 180°, 270° respectively.
 *
 * "Azimuth" here means the angle of the blade's span vector projected onto
 * the NED horizontal plane, measured clockwise from North — i.e. it is
 * exactly the EKF2 yaw for blade 0.
 *
 * The virtual heading is a fixed NED angle (captured at startup) that is
 * fed back to PX4's heading consumers (mc_att_control, Navigator) so the
 * rest of the stack believes the vehicle has a stable heading.
 ****************************************************************************/

#pragma once

#include <stdint.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/defines.h>
#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>

// FIX: was `static constexpr int NUM_BLADES = 4` — the `static` keyword at
// file scope gives every translation unit its own copy, which is an ODR
// violation when the symbol is used as a template argument or array bound
// across TUs.  A plain constexpr (no static) in a header has external
// linkage in C++17 (implicitly inline) and avoids the hazard.
constexpr int NUM_BLADES = 4;

struct AzimuthState {
    float   theta[NUM_BLADES];  // azimuth of each blade (rad, NED, 0=North, CW+)
    float   omega_rad_s;        // rotor angular velocity (rad/s, + = CCW from above)
    float   rpm;                // derived from omega_rad_s for convenience
    bool    valid;              // false until first update
};

class AzimuthTracker
{
public:
    AzimuthTracker() = default;

    /**
     * update()
     *
     * Call once per control loop tick.
     *
     * @param yaw_rad_ekf   EKF2 yaw (radians, NED) — this IS blade-0 azimuth.
     * @param yaw_rate_ned  EKF2 yaw rate (rad/s, from vehicle_angular_velocity.xyz[2]).
     * @param now_us        Current time in microseconds.
     */
    void update(float yaw_rad_ekf, float yaw_rate_ned, hrt_abstime now_us);

    const AzimuthState& state() const { return _state; }

private:
    AzimuthState _state{};
    hrt_abstime  _last_update_us{0};

    // Blade angular offsets from blade 0.
    // 4-blade symmetric rotor: blades are 90° apart.
    static constexpr float BLADE_OFFSET_RAD[NUM_BLADES] = {
        0.0f,
        math::radians(90.0f),
        math::radians(180.0f),
        math::radians(270.0f)
    };
};
