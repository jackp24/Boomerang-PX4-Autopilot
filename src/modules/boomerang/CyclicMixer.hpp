/****************************************************************************
 * CyclicMixer.hpp
 *
 * Core rotating-frame transform.  Receives a cyclic demand expressed in the
 * NED inertial frame and produces a flap deflection command for each of the
 * 4 blades based on their current azimuth.
 *
 * Math:
 *   For desired pitch demand P and roll demand R (both normalised [-1,+1]):
 *
 *     flap_i = (P * cos(θ_i + φ) + R * sin(θ_i + φ)) * max_deflection
 *
 *   where:
 *     θ_i  = current azimuth of blade i (radians, NED) from AzimuthTracker
 *     φ    = phase advance (radians, BC_CYC_PHASE parameter)
 *
 * Phase advance φ compensates for:
 *   a) Servo lag:  0.182 s/60° spec → ~10° lag at 300 RPM
 *   b) Gyroscopic precession of the rotor disk (~90° for an ideal rigid rotor)
 *   c) Any remaining aero lag
 *   Start at φ = 0 and tune by observing whether stick-forward produces
 *   forward motion or a 90°-offset motion.
 *
 * Inputs come from vehicle_attitude_setpoint (desired roll/pitch from
 * mc_att_control) rather than from a custom PID, since mc_att_control owns
 * the outer attitude loop.
 *
 * Output: per-blade flap commands, normalised [-1, +1].
 *   +1 = maximum positive deflection (trailing-edge down for standard flap).
 *   OutputStage maps this to a PWM µs value.
 ****************************************************************************/

#pragma once

#include "AzimuthTracker.hpp"   // NUM_BLADES

struct CyclicMixerOutput {
    float flap_cmd[NUM_BLADES];   // normalised [-1, +1]
};

class CyclicMixer
{
public:
    /**
     * @param phase_advance_rad  Phase lead angle (rad). Start at 0, tune up.
     * @param max_deflection     Output ceiling [0,1]. Fraction of full servo
     *                           range that cyclic is allowed to use.
     */
    CyclicMixer(float phase_advance_rad, float max_deflection);

    /**
     * mix()
     *
     * @param cyclic_pitch_ned  Normalised pitch demand [-1,+1] in NED frame.
     *                          Derived from vehicle_attitude_setpoint desired
     *                          quaternion (pitch component).
     * @param cyclic_roll_ned   Normalised roll  demand [-1,+1] in NED frame.
     * @param theta             Per-blade azimuth array from AzimuthTracker.
     */
    CyclicMixerOutput mix(float cyclic_pitch_ned,
                          float cyclic_roll_ned,
                          const float theta[NUM_BLADES]) const;

    void set_phase_advance(float rad)     { _phase_advance_rad = rad; }
    void set_max_deflection(float scale)  { _max_deflection = scale; }

private:
    float _phase_advance_rad;
    float _max_deflection;
};
