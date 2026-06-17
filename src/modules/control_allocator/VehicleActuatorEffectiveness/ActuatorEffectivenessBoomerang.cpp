/****************************************************************************
 * ActuatorEffectivenessBoomerang.cpp
 ****************************************************************************/

#include "ActuatorEffectivenessBoomerang.hpp"
#include <mathlib/mathlib.h>
#include <math.h>

ActuatorEffectivenessBoomerang::ActuatorEffectivenessBoomerang(ModuleParams *parent)
    : ActuatorEffectivenessHelicopter(parent, ActuatorType::MOTORS)
      // Pass MOTORS as tail type — no tail rotor, but parent ctor requires an arg.
      // The tail actuator output is intentionally left unconnected in the airframe config.
{}

void ActuatorEffectivenessBoomerang::updateSetpoint(
    const matrix::Vector<float, NUM_AXES> &control_sp,
    int matrix_index,
    ActuatorVector &actuator_sp,
    const ActuatorVector &actuator_min,
    const ActuatorVector &actuator_max)
{
    // Pull latest azimuth state from BoomerangEstimator
    _sub_azimuth.update(&_azimuth);

    // -----------------------------------------------------------------------
    // Let the helicopter parent handle collective throttle and pitch curves.
    // This fills actuator_sp(0) = throttle and actuator_sp(1) = tail (unused).
    // The swashplate servo slots (indices 2+) are filled by the parent using
    // the fixed CA_SP0_ANG geometry — we overwrite those below with the live
    // despin result.
    // -----------------------------------------------------------------------
    ActuatorEffectivenessHelicopter::updateSetpoint(
        control_sp, matrix_index, actuator_sp, actuator_min, actuator_max);

    // -----------------------------------------------------------------------
    // If azimuth state is not yet valid, leave the parent's output unchanged.
    // The parent uses fixed blade angles which will produce a wrong but
    // bounded output; this is acceptable during the brief startup window.
    // -----------------------------------------------------------------------
    if (!_azimuth.valid) {
        return;
    }

    // -----------------------------------------------------------------------
    // Extract normalised cyclic demands from the control setpoint.
    // ControlAllocator provides ROLL and PITCH in the virtual (despun) frame
    // because BoomerangEstimator has already overridden vehicle_angular_velocity
    // with despun rates, so mc_rate_control operates in the virtual frame.
    // -----------------------------------------------------------------------
    const float cyclic_pitch = control_sp(ControlAxis::PITCH);
    const float cyclic_roll  = control_sp(ControlAxis::ROLL);

    const float phase_advance   = _param_phase_advance.get();
    const float max_deflection  = _param_max_deflection.get();

    // -----------------------------------------------------------------------
    // Despin transform — overwrite parent's swashplate servo outputs
    // (indices _first_swash_plate_servo_index + 0..3)
    // -----------------------------------------------------------------------
    for (int i = 0; i < NUM_BLADES; ++i) {
        const float theta_eff = _azimuth.theta[i] + phase_advance;
        const float raw = cyclic_pitch * cosf(theta_eff)
                        + cyclic_roll  * sinf(theta_eff);

        const int idx = _first_swash_plate_servo_index + i;
        actuator_sp(idx) = math::constrain(raw * max_deflection, -1.0f, 1.0f);
    }

    // Yaw (tail rotor) — not used on Boomerang, zero it out
    actuator_sp(1) = 0.0f;
}
