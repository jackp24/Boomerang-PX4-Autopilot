/****************************************************************************
 * BoomerangController.hpp
 *
 * Top-level PX4 module for the propeller-driven helicopter blade ("Boomerang") vehicle.
 *
 * ARCHITECTURE OVERVIEW
 * ─────────────────────
 * This module sits at the BOTTOM of the PX4 control stack, replacing
 * ControlAllocator only.  mc_rate_control is now ENABLED and sits between
 * mc_att_control and this module:
 *
 *   Commander / Navigator          (state machine, missions, failsafes)
 *          ↓
 *   mc_pos_control                 (position → velocity → attitude setpoint)
 *          ↓
 *   mc_att_control                 (attitude setpoint → rate setpoint)
 *          ↓  vehicle_rates_setpoint
 *   mc_rate_control                (rate setpoint + despun body rates → torque)
 *          ↓  vehicle_torque_setpoint
 *   BoomerangController            (torque → cyclic mixing → actuators)
 *          ↓
 *   actuator_motors + actuator_servos
 *
 * KEY OVERRIDES THIS MODULE PUBLISHES EACH TICK
 * ──────────────────────────────────────────────
 *  vehicle_attitude          — replaces spinning EKF2 yaw with fixed virtual
 *                              heading so mc_att_control sees a stable frame.
 *  vehicle_angular_velocity  — replaces spinning body rates with rates rotated
 *                              into the virtual (non-spinning) body frame so
 *                              mc_rate_control works in the platform frame.
 *
 * DESPIN MATH (vehicle_angular_velocity override)
 * ────────────────────────────────────────────────
 * Raw body rates [p, q] are in the spinning sensor frame.  To express them
 * in the virtual body frame (aligned with the fixed virtual heading), rotate
 * by the negative of the current EKF2 yaw θ:
 *
 *   p_virt =  p_body * cos(θ) + q_body * sin(θ)
 *   q_virt = -p_body * sin(θ) + q_body * cos(θ)
 *   r_virt =  0   (spin is the rotor, not a platform yaw rate)
 *
 * mc_rate_control reads vehicle_angular_velocity and vehicle_rates_setpoint,
 * then writes vehicle_torque_setpoint [roll, pitch, yaw] normalised ≈ [-1,1].
 * BoomerangController reads torque_setpoint and scales roll/pitch into the
 * CyclicMixer.  Yaw torque is ignored (MC_YAW_WEIGHT=0 upstream).
 *
 * STARTUP SEQUENCE
 * ────────────────
 *  WAITING_FOR_EKF   → spin until EKF2 attitude_valid
 *  CAPTURING_HEADING → average EKF2 yaw over ~0.25 s (circular mean)
 *  RUNNING           → normal control loop
 *  FAULT             → EKF lost; all outputs zeroed
 *
 * MODULE COMMAND INTERFACE
 * ────────────────────────
 *   boomerang start
 *   boomerang stop
 *   boomerang status
 ****************************************************************************/

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>

#include <matrix/math.hpp>
#include <mathlib/mathlib.h>

#include "AzimuthTracker.hpp"
#include "CyclicMixer.hpp"
#include "OutputStage.hpp"

class BoomerangController final
    : public ModuleBase           // v1.17: plain non-template base class
    , public ModuleParams
    , public px4::ScheduledWorkItem
{
public:
    BoomerangController();
    ~BoomerangController() override = default;

    static ModuleBase::Descriptor _descriptor;

    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    bool init();

    void Run() override;

    int print_status() override;

private:
    // Debug state exposed to print_status()
    float _dbg_torque_roll{0.0f};
    float _dbg_torque_pitch{0.0f};
    float _dbg_p_virt{0.0f};
    float _dbg_q_virt{0.0f};

    float _raw_ekf_yaw_rad{0.0f};

    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::BC_CYC_PHASE>)      _param_cyc_phase,
        (ParamFloat<px4::params::BC_CYC_MAX_DEF>)    _param_cyc_max_def,
        (ParamFloat<px4::params::BC_RPM_MIN>)         _param_rpm_min,
        (ParamFloat<px4::params::BC_RPM_MAX>)         _param_rpm_max,
        (ParamFloat<px4::params::BC_RPM_HOVER>)       _param_rpm_hover,
        (ParamFloat<px4::params::BC_VIRT_HDG>)        _param_virt_hdg,
        (ParamFloat<px4::params::BC_PILOT_EXPO>)      _param_pilot_expo,
        (ParamFloat<px4::params::BC_TILT_MAX>)        _param_tilt_max,
        (ParamFloat<px4::params::BC_ALT_RATE_MAX>)    _param_alt_rate_max,
        (ParamFloat<px4::params::BC_T_ROLL_GAIN>) _param_torque_roll_gain,
        (ParamFloat<px4::params::BC_T_PITCH_GAIN>) _param_torque_pitch_gain
    )

    // ------------------------------------------------------------------
    // Loop timing
    // ------------------------------------------------------------------
    static constexpr float LOOP_RATE_HZ = 200.0f;
    static constexpr float LOOP_DT_S    = 1.0f / LOOP_RATE_HZ;
    hrt_abstime _last_run_us{0};

    // ------------------------------------------------------------------
    // State machine
    // ------------------------------------------------------------------
    enum class State : uint8_t {
        WAITING_FOR_EKF,
        CAPTURING_HEADING,
        RUNNING,
        FAULT,
    };
    State _state{State::WAITING_FOR_EKF};

    static constexpr int HEADING_SAMPLES = 50;  // ~0.25 s at 200 Hz
    int   _heading_sample_count{0};
    float _heading_sin_sum{0.0f};
    float _heading_cos_sum{0.0f};
    float _virtual_heading_rad{0.0f};

    // ------------------------------------------------------------------
    // Core subsystems
    // ------------------------------------------------------------------
    AzimuthTracker _azimuth_tracker{};
    CyclicMixer   *_cyclic_mixer{nullptr};
    OutputStage   *_output_stage{nullptr};

    // ------------------------------------------------------------------
    // uORB subscriptions
    // ------------------------------------------------------------------
    uORB::Subscription _sub_param_update{ORB_ID(parameter_update)};
    uORB::Subscription _sub_vehicle_status{ORB_ID(vehicle_status)};
    uORB::Subscription _sub_control_mode{ORB_ID(vehicle_control_mode)};
    uORB::Subscription _sub_attitude{ORB_ID(vehicle_attitude)};
    uORB::Subscription _sub_ang_vel{ORB_ID(vehicle_angular_velocity)};
    uORB::Subscription _sub_local_pos{ORB_ID(vehicle_local_position)};
    uORB::Subscription _sub_att_sp{ORB_ID(vehicle_attitude_setpoint)};
    uORB::Subscription _sub_manual{ORB_ID(manual_control_setpoint)};

    // mc_rate_control output — this replaces our hand-rolled P controller.
    // mc_rate_control reads our overridden vehicle_angular_velocity and writes
    // vehicle_torque_setpoint.  We subscribe here and map into CyclicMixer.
    uORB::Subscription _sub_torque_sp{ORB_ID(vehicle_torque_setpoint)};

    // Override publications — these stomp the EKF2 originals so that upstream
    // modules (mc_att_control, mc_rate_control) operate in the virtual frame.
    uORB::Publication<vehicle_attitude_s>         _pub_attitude_override{ORB_ID(vehicle_attitude)};
    uORB::Publication<vehicle_angular_velocity_s> _pub_ang_vel_override{ORB_ID(vehicle_angular_velocity)};

    // ------------------------------------------------------------------
    // Cached topic data (updated each tick)
    // ------------------------------------------------------------------
    vehicle_status_s            _vehicle_status{};
    vehicle_control_mode_s      _control_mode{};
    vehicle_attitude_s          _attitude{};
    vehicle_angular_velocity_s  _ang_vel{};
    vehicle_local_position_s    _local_pos{};
    vehicle_attitude_setpoint_s _att_sp{};
    manual_control_setpoint_s   _manual{};
    vehicle_torque_setpoint_s   _torque_sp{};

    // ------------------------------------------------------------------
    // Internal helpers
    // ------------------------------------------------------------------
    void _update_subscriptions();
    void _publish_virtual_heading();
    void _publish_despun_rates(float ekf_yaw_rad);
    void _run_control_loop(float dt_s);
    void _capture_reference_heading(float heading_rad);
    void _update_params();
    bool _is_armed() const;

    static float _expo(float x, float e);
    static float _deadband(float x, float db);
};
