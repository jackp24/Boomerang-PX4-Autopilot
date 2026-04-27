/****************************************************************************
 * OutputStage.hpp
 *
 * Translates controller outputs into PX4 actuator commands on the correct
 * modern uORB topics:
 *   actuator_motors  — normalised throttle [0,1] for each of the 4 ESCs
 *   actuator_servos  — normalised position [-1,+1] for each of the 4 flaps
 *
 * These topics feed directly into the PX4 output driver (PWM/DShot) without
 * passing through the ControlAllocator, since our mixing is done upstream.
 * The OutputStage registers as ACTUATOR_FUNCTION_CUSTOM to bypass the
 * standard allocator pipeline.
 *
 * Hardware channel mapping (must match physical wiring and output config):
 *   Motor channels  (actuator_motors):
 *     [0] Blade 0 ESC
 *     [1] Blade 1 ESC
 *     [2] Blade 2 ESC
 *     [3] Blade 3 ESC
 *   Servo channels  (actuator_servos):
 *     [0] Blade 0 flap
 *     [1] Blade 1 flap
 *     [2] Blade 2 flap
 *     [3] Blade 3 flap
 *
 * RPM → throttle:
 *   Linear map: rpm_min → 0.0, rpm_max → 1.0.
 *   Assumes ESCs are calibrated for this range.  If ESCs are in open-loop
 *   throttle mode (not RPM governor), tune rpm_min/max so that 0→1 spans
 *   the desired throttle range instead.
 *
 * Safety:
 *   When disarmed, motors are commanded to NAN (PX4 disarm value) and
 *   servos return to 0 (neutral).
 ****************************************************************************/

#pragma once

#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>
#include "AzimuthTracker.hpp"   // NUM_BLADES

struct OutputCommand {
    float collective_rpm;           // desired RPM for all 4 motors (equal)
    float flap_cmd[NUM_BLADES];     // normalised [-1,+1]
    bool  armed;                    // false → safe/disarmed outputs
};

class OutputStage
{
public:
    /**
     * @param rpm_min  RPM that maps to motor throttle 0.0
     * @param rpm_max  RPM that maps to motor throttle 1.0
     */
    OutputStage(float rpm_min, float rpm_max);

    /**
     * write()
     * Converts OutputCommand to actuator_motors + actuator_servos and
     * publishes both.  Call once per control loop tick.
     */
    void write(const OutputCommand &cmd);

    void set_rpm_limits(float rpm_min, float rpm_max);

    // Sim-only accessors for reading back published values in test_harness.cpp.
    // Not present in the real PX4 build (uORB::Publication has no last() in flight).
    const uORB::Publication<actuator_motors_s>& pub_motors() const { return _pub_motors; }
    const uORB::Publication<actuator_servos_s>& pub_servos() const { return _pub_servos; }

private:
    uORB::Publication<actuator_motors_s> _pub_motors{ORB_ID(actuator_motors)};
    uORB::Publication<actuator_servos_s> _pub_servos{ORB_ID(actuator_servos)};

    float _rpm_min;
    float _rpm_max;

    // Linear map RPM → normalised throttle [0,1]
    float _rpm_to_throttle(float rpm) const;
};
