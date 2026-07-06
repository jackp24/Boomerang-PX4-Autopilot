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
    PX4_INFO("Boomerang: updateSetpoint: called");
    // Pull latest azimuth state from BoomerangEstimator
    _sub_azimuth.update(&_azimuth);

    // -----------------------------------------------------------------------
    // Let the helicopter parent handle collective throttle and pitch curves.
    // This fills actuator_sp(0) = throttle and actuator_sp(1) = tail (unused).
    // The swashplate servo slots (indices 2+) are filled by the parent using
    // the fixed CA_SP0_ANG geometry — we overwrite those below with the live
    // despin result.
    // -----------------------------------------------------------------------
    // ActuatorEffectivenessHelicopter::updateSetpoint(
        // control_sp, matrix_index, actuator_sp, actuator_min, actuator_max);

    // -----------------------------------------------------------------------
    // If azimuth state is not yet valid, leave the parent's output unchanged.
    // The parent uses fixed blade angles which will produce a wrong but
    // bounded output; this is acceptable during the brief startup window.
    // -----------------------------------------------------------------------
    if (!_azimuth.valid) {
        return;
    }

    _saturation_flags = {};

    PX4_INFO("Boomerang: updateSetpoint: azimuth valid");

	const float spoolup_progress = throttleSpoolupProgress();
	float rpm_control_output = 0;

    // _rpm_control.setSpoolupProgress(spoolup_progress);
	// rpm_control_output = _rpm_control.getActuatorCorrection();

	// throttle/collective pitch curve
	const float throttle = (math::interpolateN(-control_sp(ControlAxis::THRUST_Z), _geometry.throttle_curve)
				+ rpm_control_output) * spoolup_progress;
	const float collective_pitch = math::interpolateN(-control_sp(ControlAxis::THRUST_Z), _geometry.pitch_curve);

	// actuator mapping
	actuator_sp(0) = mainMotorEnaged() ? throttle : NAN;

	actuator_sp(1) = control_sp(ControlAxis::YAW) * _geometry.yaw_sign
			 + fabsf(collective_pitch - _geometry.yaw_collective_pitch_offset) * _geometry.yaw_collective_pitch_scale
			 + throttle * _geometry.yaw_throttle_scale;

	// Saturation check for yaw
	if (actuator_sp(1) < actuator_min(1)) {
		setSaturationFlag(_geometry.yaw_sign, _saturation_flags.yaw_neg, _saturation_flags.yaw_pos);

	} else if (actuator_sp(1) > actuator_max(1)) {
		setSaturationFlag(_geometry.yaw_sign, _saturation_flags.yaw_pos, _saturation_flags.yaw_neg);
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
