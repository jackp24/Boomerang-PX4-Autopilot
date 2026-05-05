/****************************************************************************
 * OutputStage.cpp
 ****************************************************************************/

#include "OutputStage.hpp"
#include <mathlib/mathlib.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/log.h>
#include <math.h>  // NAN

OutputStage::OutputStage(float rpm_min, float rpm_max)
    : _rpm_min(rpm_min)
    , _rpm_max(rpm_max)
{}

void OutputStage::write(const OutputCommand &cmd)
{
    const hrt_abstime now = hrt_absolute_time();

    // -----------------------------------------------------------------------
    // Motor commands
    // -----------------------------------------------------------------------
    actuator_motors_s motors{};
    motors.timestamp          = now;
    motors.timestamp_sample   = now;
    // reversible_flags: none of our motors run in reverse
    motors.reversible_flags   = 0;

    if (!cmd.armed) {
        // PX4 convention: NAN = disarmed/off.  The output driver idles ESCs.
        for (int i = 0; i < NUM_BLADES; ++i) {
            motors.control[i] = NAN;
        }
        // Zero remaining channels
        for (int i = NUM_BLADES; i < actuator_motors_s::NUM_CONTROLS; ++i) {
            motors.control[i] = NAN;
        }
    } else {
        const float throttle = _rpm_to_throttle(cmd.collective_rpm);
        for (int i = 0; i < NUM_BLADES; ++i) {
            motors.control[i] = throttle;
        }
        for (int i = NUM_BLADES; i < actuator_motors_s::NUM_CONTROLS; ++i) {
            motors.control[i] = NAN;
        }
    }

    _pub_motors.publish(motors);

    // -----------------------------------------------------------------------
    // Servo (flap) commands
    // actuator_servos uses normalised [-1, +1].  The output driver maps this
    // to the PWM range configured in QGC (typically 1000–2000 µs with 1500 neutral).
    // -----------------------------------------------------------------------
    actuator_servos_s servos{};
    servos.timestamp        = now;
    servos.timestamp_sample = now;

    for (int i = 0; i < NUM_BLADES; ++i) {
        // When disarmed return flaps to neutral (0.0 = 1500 µs).
        servos.control[i] = cmd.armed
                            ? math::constrain(cmd.flap_cmd[i], -1.0f, 1.0f)
                            : 0.0f;
	// servos.control[i] = -0.5f;
	// PX4_INFO("command : %.1f", (double)cmd.flap_cmd[i]);
    }
    for (int i = NUM_BLADES; i < actuator_servos_s::NUM_CONTROLS; ++i) {
        servos.control[i] = 0.0f;
    }

    last_cmd = cmd;

    _pub_servos.publish(servos);
}

void OutputStage::set_rpm_limits(float rpm_min, float rpm_max)
{
    _rpm_min = rpm_min;
    _rpm_max = rpm_max;
}

float OutputStage::_rpm_to_throttle(float rpm) const
{
    const float span = _rpm_max - _rpm_min;
    if (span < 1.0f) { return 0.0f; }
    return math::constrain((rpm - _rpm_min) / span, 0.0f, 1.0f);
}
