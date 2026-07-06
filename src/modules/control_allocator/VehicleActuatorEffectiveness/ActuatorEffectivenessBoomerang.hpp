/****************************************************************************
 * ActuatorEffectivenessBoomerang.hpp
 *
 * Derived from ActuatorEffectivenessHelicopter.
 *
 * DIFFERENCES FROM HELICOPTER
 * ───────────────────────────
 * 1. No tail rotor. Yaw torque is ignored.
 * 2. Blades rotate continuously — fixed swashplate geometry is replaced by
 *    a live despin transform using per-blade azimuth θ_i(t) from the
 *    boomerang_azimuth uORB topic published by BoomerangEstimator.
 * 3. 4-blade layout: blades are 90° apart. CA_SP0_ANG0..3 map to the
 *    static blade offsets; the live θ_i component comes from the azimuth topic.
 * 4. Collective pitch and throttle curves are inherited unchanged.
 *
 * DESPIN MIX (per blade i)
 * ────────────────────────
 *   flap_i = (pitch * cos(θ_i + φ) + roll * sin(θ_i + φ)) * max_deflection
 *
 * where:
 *   θ_i  = boomerang_azimuth.theta[i]  (live, from BoomerangEstimator)
 *   φ    = BC_CYC_PHASE                (phase advance parameter)
 *   pitch, roll = normalised cyclic demands from ControlAllocator
 *
 * WIRING INTO CONTROLALLOCATOR
 * ─────────────────────────────
 * Add CA_AIRFRAME value 15 (or next available) in:
 *   src/modules/control_allocator/ControlAllocator.hpp  (EffectivenessSource enum)
 *   src/modules/control_allocator/ControlAllocator.cpp  (switch case)
 *   src/modules/control_allocator/module.yaml           (CA_AIRFRAME enum values)
 *
 * Set CA_AIRFRAME=15 in QGC or your airframe init script.
 ****************************************************************************/

#pragma once

#include <ActuatorEffectivenessHelicopter.hpp>
#include <RpmControlBoomerang.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/topics/boomerang_azimuth.h>
#include <px4_platform_common/module_params.h>
#include <mathlib/mathlib.h>

class ActuatorEffectivenessBoomerang final : public ActuatorEffectivenessHelicopter
{
public:
    explicit ActuatorEffectivenessBoomerang(ModuleParams *parent);
    ~ActuatorEffectivenessBoomerang() override = default;

    const char *name() const override { return "Boomerang"; }

    /**
     * Override updateSetpoint to inject the live despin transform.
     * Collective and throttle curves are inherited from the helicopter class.
     */
    void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
                        int matrix_index,
                        ActuatorVector &actuator_sp,
                        const ActuatorVector &actuator_min,
                        const ActuatorVector &actuator_max) override;

private:
    static constexpr int NUM_BLADES = 4;

    uORB::Subscription _sub_azimuth{ORB_ID(boomerang_azimuth)};
    boomerang_azimuth_s _azimuth{};

    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::BC_CYC_PHASE>)   _param_phase_advance,
        (ParamFloat<px4::params::BC_CYC_MAX_DEF>) _param_max_deflection
    )

    RpmControlBoomerang _rpm_control {this};
};
