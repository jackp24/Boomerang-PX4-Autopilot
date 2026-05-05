/****************************************************************************
 * BoomerangController.hpp
 *
 * Top-level PX4 module for the propeller-driven helicopter blade ("Boomerang") vehicle.
 *
 * ARCHITECTURE OVERVIEW
 * ─────────────────────
 * This module sits at the BOTTOM of the PX4 control stack, replacing
 * mc_rate_control and ControlAllocator.  Everything above it runs unchanged:
 *
 *   Commander / Navigator          (state machine, missions, failsafes)
 *          ↓
 *   mc_pos_control                 (position → velocity → attitude setpoint)
 *          ↓
 *   mc_att_control                 (attitude setpoint → ... but NOT rate cmd)
 *          ↓  vehicle_attitude_setpoint  ←── this module subscribes here
 *   BoomerangController             (attitude SP → cyclic mixing → actuators)
 *          ↓
 *   actuator_motors + actuator_servos  (to PWM/DShot output driver)
 *
 * mc_att_control is configured with MC_YAW_WEIGHT=0 so it ignores yaw error.
 * mc_rate_control is DISABLED in the board config (not compiled in).
 *
 * PX4 v1.17 MODULE API
 * ────────────────────
 * ModuleBase<T> (CRTP template) was removed in v1.17 and replaced with a
 * plain non-template ModuleBase class using a Descriptor pattern:
 *
 *   - Inherit from ModuleBase with NO template argument.
 *   - Declare a static ModuleBase::Descriptor _descriptor in the class;
 *     define it in the .cpp, passing &task_spawn, &custom_command,
 *     &print_usage as constructor arguments.
 *   - boomerang_main() calls ModuleBase::main(_descriptor, argc, argv).
 *   - task_spawn() stores the instance pointer and task id:
 *       _descriptor.object.store(instance)
 *       _descriptor.task_id = task_id_is_work_queue
 *   - Run() calls ModuleBase::exit_and_cleanup(_descriptor) on exit.
 *   - Use ModuleBase::get_instance<BoomerangController>(_descriptor) to
 *     get the typed instance pointer from static methods if needed.
 *
 * WHAT THIS MODULE DOES EACH TICK (200 Hz)
 * ─────────────────────────────────────────
 *  1. Read vehicle_status → gate on armed + correct control mode.
 *  2. Read vehicle_attitude_setpoint from mc_att_control (or manual input).
 *  3. Read vehicle_local_position for thrust normalisation.
 *  4. Update AzimuthTracker (blade angles from EKF2 yaw + yaw rate).
 *  5. Publish virtual heading override on vehicle_attitude so the rest of the
 *     stack sees a stable heading instead of the spinning EKF2 yaw.
 *  6. Extract desired roll/pitch from attitude setpoint quaternion.
 *  7. Map normalised thrust setpoint → collective RPM.
 *  8. Run CyclicMixer → per-blade flap commands.
 *  9. Publish actuator_motors + actuator_servos via OutputStage.
 *
 * STARTUP SEQUENCE
 * ────────────────
 *  WAITING_FOR_EKF   → spin until EKF2 attitude_valid
 *  CAPTURING_HEADING → average EKF2 yaw over ~0.25 s with circular mean
 *                      to get a stable startup heading despite rotor spin
 *  RUNNING           → normal control loop
 *  FAULT             → EKF lost; all outputs zeroed; wait for operator
 *
 * YAW HANDLING
 * ────────────
 * The EKF2 yaw spins at ~300 RPM.  This module captures a "virtual heading"
 * at startup and re-publishes it on vehicle_attitude each tick, overwriting
 * the spinning yaw so mc_att_control and Navigator see a stable heading.
 * MC_YAW_WEIGHT=0 ensures mc_att_control ignores any residual yaw error.
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

    // ----------------------------------------------------------------
    // v1.17 Descriptor — one static instance per module type.
    // Holds function pointers, the running object pointer, and task_id.
    // Constructed in BoomerangController.cpp with the three function
    // pointers below. boomerang_main() passes it to ModuleBase::main().
    // ----------------------------------------------------------------
    static ModuleBase::Descriptor _descriptor;

    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    bool init();

    // ---- ScheduledWorkItem interface ----
    void Run() override;

    int print_status() override;

private:
    float d_pitch = 0.0f;
    float d_roll = 0.0f;

    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::BC_CYC_PHASE>)    _param_cyc_phase,
        (ParamFloat<px4::params::BC_CYC_MAX_DEF>)  _param_cyc_max_def,
        (ParamFloat<px4::params::BC_RPM_MIN>)       _param_rpm_min,
        (ParamFloat<px4::params::BC_RPM_MAX>)       _param_rpm_max,
        (ParamFloat<px4::params::BC_RPM_HOVER>)     _param_rpm_hover,
        (ParamFloat<px4::params::BC_VIRT_HDG>)      _param_virt_hdg,
        (ParamFloat<px4::params::BC_PILOT_EXPO>)    _param_pilot_expo,
        (ParamFloat<px4::params::BC_TILT_MAX>)      _param_tilt_max,
        (ParamFloat<px4::params::BC_ALT_RATE_MAX>)  _param_alt_rate_max
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
        WAITING_FOR_EKF,      // waiting for EKF2 attitude to become valid
        CAPTURING_HEADING,    // averaging startup yaw for virtual heading
        RUNNING,              // normal control loop active
        FAULT,                // EKF lost or other error; outputs zeroed
    };
    State _state{State::WAITING_FOR_EKF};

    // Heading capture (circular mean over HEADING_SAMPLES ticks)
    static constexpr int HEADING_SAMPLES = 50;  // ~0.25 s at 200 Hz
    int   _heading_sample_count{0};
    float _heading_sin_sum{0.0f};
    float _heading_cos_sum{0.0f};
    float _virtual_heading_rad{0.0f};

    // ------------------------------------------------------------------
    // Core subsystems
    // ------------------------------------------------------------------
    AzimuthTracker _azimuth_tracker{};
    CyclicMixer   *_cyclic_mixer{nullptr};   // constructed after params loaded
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

    // We re-publish vehicle_attitude with a synthetic (constant) yaw so that
    // mc_att_control and Navigator see a stable heading instead of the
    // spinning EKF2 yaw.
    uORB::Publication<vehicle_attitude_s> _pub_attitude_override{ORB_ID(vehicle_attitude)};

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

    // ------------------------------------------------------------------
    // Internal helpers
    // ------------------------------------------------------------------
    void _update_subscriptions();
    void _publish_virtual_heading();
    void _run_control_loop(float dt_s);
    void _capture_reference_heading(float heading_rad);
    void _update_params();
    bool _is_armed() const;

    static void _quat_to_roll_pitch(const matrix::Quatf &q,
                                    float &roll_rad, float &pitch_rad);
    static float _expo(float x, float e);
    static float _deadband(float x, float db);
};
