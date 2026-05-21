/****************************************************************************
 * OutputStage.cpp
 *
 * Changes: RPM abstraction removed. collective_throttle is now a direct
 * normalized 0.0–1.0 value passed straight to actuator_motors.control[].
 * Removed: _rpm_min, _rpm_max, _rpm_to_throttle(), set_rpm_limits().
 ****************************************************************************/

#include "OutputStage.hpp"
#include <mathlib/mathlib.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/log.h>
#include <math.h>

void OutputStage::write(const OutputCommand &cmd)
{
    const hrt_abstime now = hrt_absolute_time();

    // -----------------------------------------------------------------------
    // Motor commands
    // actuator_motors.control[] expects normalized 0.0–1.0 throttle.
    // NAN = disarmed/off — the output driver idles ESCs.
    // -----------------------------------------------------------------------
    actuator_motors_s motors{};
    motors.timestamp        = now;
    motors.timestamp_sample = now;
    motors.reversible_flags = 0;

    if (!cmd.armed) {
        for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; ++i) {
            motors.control[i] = NAN;
        }
    } else {
        const float throttle = math::constrain(cmd.collective_throttle, 0.0f, 1.0f);
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
    // actuator_servos uses normalised [-1, +1].
    // -----------------------------------------------------------------------
    actuator_servos_s servos{};
    servos.timestamp        = now;
    servos.timestamp_sample = now;

    for (int i = 0; i < NUM_BLADES; ++i) {
        // servos.control[i] = cmd.armed
                            // ? math::constrain(cmd.flap_cmd[i], -1.0f, 1.0f)
                            // : 0.0f;
        servos.control[i] = -0.3f;
    }
    for (int i = NUM_BLADES; i < actuator_servos_s::NUM_CONTROLS; ++i) {
        servos.control[i] = 0.0f;
    }

    last_cmd = cmd;

    _pub_servos.publish(servos);
}
