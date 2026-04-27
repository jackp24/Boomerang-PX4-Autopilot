/****************************************************************************
 * CyclicMixer.cpp
 ****************************************************************************/

#include "CyclicMixer.hpp"
#include <mathlib/mathlib.h>
#include <math.h>

CyclicMixer::CyclicMixer(float phase_advance_rad, float max_deflection)
    : _phase_advance_rad(phase_advance_rad)
    , _max_deflection(max_deflection)
{}

CyclicMixerOutput CyclicMixer::mix(float cyclic_pitch_ned,
                                    float cyclic_roll_ned,
                                    const float theta[NUM_BLADES]) const
{
    CyclicMixerOutput out{};

    for (int i = 0; i < NUM_BLADES; ++i) {
        // Apply phase advance: command the flap earlier in the rotation so
        // the blade reaches its target deflection at the aerodynamically
        // effective azimuth.
        const float eff = theta[i] + _phase_advance_rad;

        // Despin transform:
        //   When blade i faces North (theta=0): receives full pitch demand.
        //   When blade i faces East  (theta=π/2): receives full roll demand.
        const float raw = cyclic_pitch_ned * cosf(eff)
                        + cyclic_roll_ned  * sinf(eff);

        out.flap_cmd[i] = math::constrain(raw * _max_deflection, -1.0f, 1.0f);
    }

    return out;
}
