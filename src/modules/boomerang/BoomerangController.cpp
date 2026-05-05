/****************************************************************************
 * BoomerangController.cpp
 *
 * See BoomerangController.hpp for full architecture documentation.
 *
 * Changes vs. original:
 *   1. OutputStage and CyclicMixer are constructed in init() (after an early
 *      updateParams()), not lazily in _capture_reference_heading().  This
 *      ensures actuator_servos is published from the very first tick so the
 *      PWM output driver always has a valid signal and servos don't buzz.
 *
 *   2. WAITING_FOR_EKF and CAPTURING_HEADING both publish a disarmed/neutral
 *      OutputCommand every tick, keeping servos quiet during startup.
 *
 *   3. The EKF-ready gate no longer requires v_xy_valid (which needs GPS).
 *      attitude.timestamp > 0 is sufficient for bench testing and for
 *      vehicles that use optical flow or operate without position estimate.
 *      Re-add && _local_pos.v_xy_valid if you want to enforce GPS before
 *      arming is possible.
 ****************************************************************************/

#include "BoomerangController.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>
#include <matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <math.h>

// ---------------------------------------------------------------------------
// v1.17 Descriptor definition
// ---------------------------------------------------------------------------
ModuleBase::Descriptor BoomerangController::_descriptor{
    &BoomerangController::task_spawn,
    &BoomerangController::custom_command,
    &BoomerangController::print_usage
};

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
BoomerangController::BoomerangController()
    : ModuleParams(nullptr)
    , ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{}

// ---------------------------------------------------------------------------
// init
//
// Key change: params are loaded here so OutputStage and CyclicMixer can be
// constructed immediately.  A neutral OutputCommand is then published so the
// PWM driver has a valid signal before the first Run() tick.
// ---------------------------------------------------------------------------
bool BoomerangController::init()
{
    // Load parameters before constructing subsystems that depend on them.
    updateParams();

    _output_stage = new OutputStage(_param_rpm_min.get(), _param_rpm_max.get());
    _cyclic_mixer = new CyclicMixer(_param_cyc_phase.get(), _param_cyc_max_def.get());

    if (!_output_stage || !_cyclic_mixer) {
        PX4_ERR("boomerang: subsystem alloc failed");
        return false;
    }

    // Publish a neutral/disarmed command immediately so the PWM output driver
    // has a valid signal.  Without this the servo output pins are undefined
    // and servos buzz from the moment they are powered.
    OutputCommand neutral{};
    neutral.armed = false;
    _output_stage->write(neutral);

    ScheduleOnInterval(static_cast<uint32_t>(1e6f / LOOP_RATE_HZ));
    PX4_INFO("boomerang: started at %.0f Hz", (double)LOOP_RATE_HZ);
    return true;
}

// ---------------------------------------------------------------------------
// Main loop — called by the work queue every 5 ms (200 Hz)
// ---------------------------------------------------------------------------
void BoomerangController::Run()
{
    if (should_exit()) {
        ScheduleClear();
        ModuleBase::exit_and_cleanup(_descriptor);
        return;
    }

    const hrt_abstime now_us = hrt_absolute_time();
    const float dt_s = (_last_run_us > 0)
        ? math::constrain((float)(now_us - _last_run_us) * 1e-6f, 0.001f, 0.05f)
        : LOOP_DT_S;
    _last_run_us = now_us;

    _update_subscriptions();

    parameter_update_s param_upd{};
    if (_sub_param_update.update(&param_upd)) {
        _update_params();
    }

    // Neutral command reused in pre-RUNNING states to keep servos quiet.
    // _output_stage is guaranteed non-null after init() succeeds.
    OutputCommand neutral{};
    neutral.armed = false;

    switch (_state) {

    case State::WAITING_FOR_EKF:
        // Keep servos at neutral while waiting — prevents buzzing.
        _output_stage->write(neutral);

        // v_xy_valid (GPS) is NOT required here; attitude.timestamp > 0 means
        // EKF2 is publishing a valid orientation estimate which is all we need
        // to start the heading capture.  Re-add && _local_pos.v_xy_valid if
        // you want to enforce a position fix before proceeding.
        if (_attitude.timestamp > 0) {
            PX4_INFO("boomerang: EKF valid — capturing heading");
            _state = State::CAPTURING_HEADING;
        }
        break;

    case State::CAPTURING_HEADING: {
        // Keep servos at neutral during heading capture.
        _output_stage->write(neutral);

        const float yaw = matrix::Eulerf(matrix::Quatf(_attitude.q)).psi();
        _heading_sin_sum += sinf(yaw);
        _heading_cos_sum += cosf(yaw);
        ++_heading_sample_count;

        if (_heading_sample_count >= HEADING_SAMPLES) {
            const float mean_yaw = atan2f(_heading_sin_sum, _heading_cos_sum);
            _capture_reference_heading(mean_yaw);
            _state = State::RUNNING;
            PX4_INFO("boomerang: virtual heading = %.1f deg — RUNNING",
                     (double)math::degrees(mean_yaw));
        }
        break;
    }

    case State::RUNNING:
        if (_attitude.timestamp == 0) {
            PX4_WARN("boomerang: EKF attitude lost — FAULT");
            _state = State::FAULT;
            break;
        }
        _publish_virtual_heading();
        _run_control_loop(dt_s);
        break;

    case State::FAULT:
        _output_stage->write(neutral);
        if (_attitude.timestamp > 0) {
            PX4_INFO("boomerang: EKF recovered — re-waiting");
            // Reset heading capture accumulators for a clean re-capture.
            _heading_sample_count = 0;
            _heading_sin_sum      = 0.0f;
            _heading_cos_sum      = 0.0f;
            _state = State::WAITING_FOR_EKF;
        }
        break;
    }
}

// ---------------------------------------------------------------------------
// _update_subscriptions
// ---------------------------------------------------------------------------
void BoomerangController::_update_subscriptions()
{
    _sub_vehicle_status.update(&_vehicle_status);
    _sub_control_mode.update(&_control_mode);
    _sub_attitude.update(&_attitude);
    _sub_ang_vel.update(&_ang_vel);
    _sub_local_pos.update(&_local_pos);
    _sub_att_sp.update(&_att_sp);
    _sub_manual.update(&_manual);
}

// ---------------------------------------------------------------------------
// _publish_virtual_heading
// ---------------------------------------------------------------------------
void BoomerangController::_publish_virtual_heading()
{
    const matrix::Quatf  q_ekf(_attitude.q);
    const matrix::Eulerf euler_ekf(q_ekf);

    const matrix::Eulerf euler_virt(euler_ekf.phi(),
                                    euler_ekf.theta(),
                                    _virtual_heading_rad);
    const matrix::Quatf q_virt(euler_virt);

    vehicle_attitude_s att_out = _attitude;
    q_virt.copyTo(att_out.q);
    _pub_attitude_override.publish(att_out);
}

// ---------------------------------------------------------------------------
// _run_control_loop
// ---------------------------------------------------------------------------
void BoomerangController::_run_control_loop(float dt_s)
{
    (void)dt_s;

    const bool armed = _is_armed();

    if (!armed) {
        OutputCommand zero{};
        zero.armed = false;
        _output_stage->write(zero);
        return;
    }

    // Azimuth tracker — use raw EKF yaw (pre-patch value in _attitude cache)
    const matrix::Eulerf euler_raw(matrix::Quatf(_attitude.q));
    const float raw_yaw  = euler_raw.psi();
    const float yaw_rate = _ang_vel.xyz[2];

    _azimuth_tracker.update(raw_yaw, yaw_rate, hrt_absolute_time());
    const AzimuthState &az = _azimuth_tracker.state();

    // Desired roll/pitch from attitude setpoint
    float desired_roll_rad  = 0.0f;
    float desired_pitch_rad = 0.0f;

    const bool att_sp_valid = (_att_sp.timestamp > 0) &&
                               _control_mode.flag_control_attitude_enabled;

    if (att_sp_valid) {
        _quat_to_roll_pitch(matrix::Quatf(_att_sp.q_d),
                            desired_roll_rad, desired_pitch_rad);
    } else {
        if (_manual.timestamp > 0) {
            const float expo     = _param_pilot_expo.get();
            const float tilt_max = _param_tilt_max.get();

            const float raw_pitch = _expo(_deadband(_manual.pitch, 0.05f), expo);
            const float raw_roll  = _expo(_deadband(_manual.roll,  0.05f), expo);

            const float c = cosf(_virtual_heading_rad);
            const float s = sinf(_virtual_heading_rad);
            desired_pitch_rad = (raw_pitch * c - raw_roll * s) * tilt_max;
            desired_roll_rad  = (raw_pitch * s + raw_roll * c) * tilt_max;
        }
    }

    // Collective RPM from thrust setpoint
    float collective_rpm = _param_rpm_hover.get();

    if (att_sp_valid) {
        const float thrust_norm = math::constrain(-_att_sp.thrust_body[2], 0.0f, 1.0f);
        collective_rpm = _param_rpm_min.get()
                       + thrust_norm * (_param_rpm_max.get() - _param_rpm_min.get());

    } else if (_manual.timestamp > 0 && _control_mode.flag_control_manual_enabled) {
        const float throttle = math::constrain(_manual.throttle, 0.0f, 1.0f);
        collective_rpm = _param_rpm_min.get()
                       + throttle * (_param_rpm_max.get() - _param_rpm_min.get());
    }

    collective_rpm = math::constrain(collective_rpm,
                                     _param_rpm_min.get(),
                                     _param_rpm_max.get());

    // Normalise tilt demand
    const float tilt_max     = _param_tilt_max.get();
    const float cyclic_pitch = math::constrain(desired_pitch_rad / tilt_max, -1.0f, 1.0f);
    const float cyclic_roll  = math::constrain(desired_roll_rad  / tilt_max, -1.0f, 1.0f);

    // Cyclic mixer
    const CyclicMixerOutput mix = _cyclic_mixer->mix(cyclic_pitch, cyclic_roll, az.theta);

    // Publish actuator commands
    OutputCommand cmd{};
    cmd.armed          = true;
    cmd.collective_rpm = collective_rpm;
    for (int i = 0; i < NUM_BLADES; ++i) {
        cmd.flap_cmd[i] = mix.flap_cmd[i];
    }
    _output_stage->write(cmd);
}

// ---------------------------------------------------------------------------
// _capture_reference_heading
//
// Now only responsible for storing the heading and saving it to the param.
// OutputStage and CyclicMixer are already constructed by init().
// ---------------------------------------------------------------------------
void BoomerangController::_capture_reference_heading(float heading_rad)
{
    _virtual_heading_rad = heading_rad;

    param_t h = param_find("BC_VIRT_HDG");
    if (h != PARAM_INVALID) {
        param_set(h, &heading_rad);
    }

    // Update subsystems with the now-confirmed param values (user may have
    // changed them in QGC during the startup window).
    _cyclic_mixer->set_phase_advance(_param_cyc_phase.get());
    _cyclic_mixer->set_max_deflection(_param_cyc_max_def.get());
    _output_stage->set_rpm_limits(_param_rpm_min.get(), _param_rpm_max.get());
}

// ---------------------------------------------------------------------------
// _update_params
// ---------------------------------------------------------------------------
void BoomerangController::_update_params()
{
    updateParams();

    if (_cyclic_mixer) {
        _cyclic_mixer->set_phase_advance(_param_cyc_phase.get());
        _cyclic_mixer->set_max_deflection(_param_cyc_max_def.get());
    }
    if (_output_stage) {
        _output_stage->set_rpm_limits(_param_rpm_min.get(), _param_rpm_max.get());
    }
}

// ---------------------------------------------------------------------------
// _is_armed
// ---------------------------------------------------------------------------
bool BoomerangController::_is_armed() const
{
    return (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
}

// ---------------------------------------------------------------------------
// _quat_to_roll_pitch
// ---------------------------------------------------------------------------
void BoomerangController::_quat_to_roll_pitch(const matrix::Quatf &q,
                                              float &roll_rad, float &pitch_rad)
{
    const matrix::Eulerf euler(q);
    roll_rad  = euler.phi();
    pitch_rad = euler.theta();
}

// ---------------------------------------------------------------------------
// _expo / _deadband
// ---------------------------------------------------------------------------
float BoomerangController::_expo(float x, float e)
{
    return e * x * x * x + (1.0f - e) * x;
}

float BoomerangController::_deadband(float x, float db)
{
    if (fabsf(x) < db) { return 0.0f; }
    const float sign = (x > 0.0f) ? 1.0f : -1.0f;
    return sign * (fabsf(x) - db) / (1.0f - db);
}

// ---------------------------------------------------------------------------
// task_spawn
// ---------------------------------------------------------------------------
int BoomerangController::task_spawn(int argc, char *argv[])
{
    BoomerangController *instance = new BoomerangController();

    if (!instance) {
        PX4_ERR("alloc failed");
        return PX4_ERROR;
    }

    _descriptor.object.store(instance);
    _descriptor.task_id = task_id_is_work_queue;

    if (!instance->init()) {
        _descriptor.object.store(nullptr);
        _descriptor.task_id = -1;
        delete instance;
        return PX4_ERROR;
    }

    return PX4_OK;
}

// ---------------------------------------------------------------------------
// custom_command / print_usage
// ---------------------------------------------------------------------------
int BoomerangController::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int BoomerangController::print_usage(const char *reason)
{
    if (reason) { PX4_WARN("%s\n", reason); }

    PRINT_MODULE_DESCRIPTION("Propeller-driven helicopter blade (boomerang) flight controller.\n"
        "Replaces mc_rate_control + ControlAllocator.\n"
        "Requires mc_att_control and mc_pos_control to be running above it.");
    PRINT_MODULE_USAGE_NAME("boomerang", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
    return 0;
}

int BoomerangController::print_status()
{
    const char *state_str = "UNKNOWN";
    switch (_state) {
        case State::WAITING_FOR_EKF:    state_str = "WAITING_FOR_EKF";    break;
        case State::CAPTURING_HEADING:  state_str = "CAPTURING_HEADING";  break;
        case State::RUNNING:            state_str = "RUNNING";             break;
        case State::FAULT:              state_str = "FAULT";               break;
    }

    PX4_INFO("state:           %s", state_str);
    PX4_INFO("virtual heading: %.1f deg", (double)math::degrees(_virtual_heading_rad));
    PX4_INFO("heading samples: %d / %d", _heading_sample_count, HEADING_SAMPLES);

    if (_azimuth_tracker.state().valid) {
        PX4_INFO("rotor RPM:       %.1f", (double)_azimuth_tracker.state().rpm);
        PX4_INFO("blade0 azimuth:  %.1f deg", (double)math::degrees(_azimuth_tracker.state().theta[0]));
    } else {
        PX4_INFO("azimuth tracker: not valid");
    }

    if (_output_stage) {
        PX4_INFO("output stage:    initialized");
    } else {
        PX4_INFO("output stage:    NULL (not yet constructed)");
    }

    return 0;
}

// ---------------------------------------------------------------------------
// Module entry point
// ---------------------------------------------------------------------------
extern "C" __EXPORT int boomerang_main(int argc, char *argv[])
{
    return ModuleBase::main(BoomerangController::_descriptor, argc, argv);
}
