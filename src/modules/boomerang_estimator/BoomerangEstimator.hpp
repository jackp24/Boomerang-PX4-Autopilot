/****************************************************************************
 * BoomerangEstimator.hpp
 *
 * Companion estimator module for the Boomerang spinning-blade vehicle.
 *
 * ROLE
 * ────
 * This module sits alongside EKF2 as a pure observer — it reads raw EKF2
 * attitude and angular velocity and publishes three derived topics:
 *
 *   vehicle_attitude_virtual   — EKF2 attitude with yaw replaced by a fixed
 *                                virtual heading, so mc_att_control sees a
 *                                stable non-spinning frame.
 *
 *   vehicle_angular_velocity   — Override of the standard topic with body
 *                                rates rotated into the virtual (despun)
 *                                frame, so mc_rate_control works in the
 *                                platform frame.
 *
 *   boomerang_azimuth          — Live blade azimuth angles θ_i(t) and rotor
 *                                omega, consumed by ActuatorEffectivenessBoomerang
 *                                inside ControlAllocator for the despin mix.
 *
 * STARTUP SEQUENCE
 * ────────────────
 *   WAITING_FOR_EKF   → spins until EKF2 attitude timestamp > 0
 *   CAPTURING_HEADING → circular mean of EKF2 yaw over ~0.25 s (50 samples)
 *   RUNNING           → publishes all three topics every tick
 *   FAULT             → EKF2 lost; stops publishing overrides
 *
 * The captured virtual heading is written to parameter BC_VIRT_HDG so it
 * is visible in QGC and can be read by other modules (e.g. heading_led).
 ****************************************************************************/

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/log.h>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
// #include <uORB/topics/vehicle_attitude_virtual.h>
#include <uORB/topics/boomerang_azimuth.h>

#include <matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <drivers/drv_hrt.h>

class BoomerangEstimator final
    : public ModuleBase
    , public ModuleParams
    , public px4::ScheduledWorkItem
{
public:
    BoomerangEstimator();
    ~BoomerangEstimator();

    static ModuleBase::Descriptor _descriptor;

    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    bool init();
    void Run() override;
    int  print_status() override;

private:
    // -----------------------------------------------------------------------
    // State machine
    // -----------------------------------------------------------------------
    enum class State : uint8_t {
        WAITING_FOR_EKF,
        CAPTURING_HEADING,
        RUNNING,
        FAULT,
    };
    State _state{State::WAITING_FOR_EKF};

    static constexpr float LOOP_RATE_HZ  = 200.0f;
    static constexpr int   HEADING_SAMPLES = 50;   // ~0.25 s at 200 Hz
    static constexpr int   NUM_BLADES    = 4;

    static constexpr float BLADE_OFFSET_RAD[NUM_BLADES] = {
        0.0f,
        math::radians(90.0f),
        math::radians(180.0f),
        math::radians(270.0f)
    };

    // -----------------------------------------------------------------------
    // Heading capture state
    // -----------------------------------------------------------------------
    int   _heading_sample_count{0};
    float _heading_sin_sum{0.0f};
    float _heading_cos_sum{0.0f};
    float _virtual_heading_rad{0.0f};

    // -----------------------------------------------------------------------
    // uORB subscriptions
    // -----------------------------------------------------------------------
    uORB::Subscription _sub_attitude{ORB_ID(vehicle_attitude)};
    uORB::Subscription _sub_ang_vel{ORB_ID(vehicle_angular_velocity)};
    uORB::Subscription _sub_param_update{ORB_ID(parameter_update)};

    // -----------------------------------------------------------------------
    // uORB publications
    // -----------------------------------------------------------------------
    uORB::Publication<vehicle_attitude_s> _pub_attitude_virtual{ORB_ID(vehicle_attitude_virtual)};
    uORB::Publication<vehicle_angular_velocity_s> _pub_ang_vel_despun{ORB_ID(vehicle_angular_velocity)};
    uORB::Publication<boomerang_azimuth_s>         _pub_azimuth{ORB_ID(boomerang_azimuth)};

    // -----------------------------------------------------------------------
    // Cached topic data
    // -----------------------------------------------------------------------
    vehicle_attitude_s          _attitude{};
    vehicle_angular_velocity_s  _ang_vel{};

    // -----------------------------------------------------------------------
    // Params
    // -----------------------------------------------------------------------
    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::BC_VIRT_HDG>) _param_virt_hdg
    )

    // -----------------------------------------------------------------------
    // Internal helpers
    // -----------------------------------------------------------------------
    void _capture_reference_heading(float heading_rad);
    void _publish_virtual_attitude();
    void _publish_despun_rates(float ekf_yaw_rad);
    void _publish_azimuth(float ekf_yaw_rad, float yaw_rate);
    perf_counter_t _loop_perf{nullptr};
};
