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
    float collective_throttle;      // normalised throttle [0,1] for all 4 motors
    float flap_cmd[NUM_BLADES];     // normalised [-1,+1]
    bool  armed;                    // false → safe/disarmed outputs
};

class OutputStage
{
public:
    OutputStage() = default;

    /**
     * write()
     * Converts OutputCommand to actuator_motors + actuator_servos and
     * publishes both.  Call once per control loop tick.
     */
    void write(const OutputCommand &cmd);

    // Sim-only accessors for reading back published values in test_harness.cpp.
    const uORB::Publication<actuator_motors_s>& pub_motors() const { return _pub_motors; }
    const uORB::Publication<actuator_servos_s>& pub_servos() const { return _pub_servos; }

    OutputCommand last_cmd{};

private:
    uORB::Publication<actuator_motors_s> _pub_motors{ORB_ID(actuator_motors)};
    uORB::Publication<actuator_servos_s> _pub_servos{ORB_ID(actuator_servos)};
};
