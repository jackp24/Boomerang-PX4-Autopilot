/****************************************************************************
 * BoomerangController.cpp
 *
 * Changes vs. previous revision:
 *   - RPM abstraction removed. collective_throttle is a direct 0.0–1.0 value.
 *   - Removed: _param_rpm_min, _param_rpm_max, _param_rpm_hover.
 *   - OutputStage constructor takes no arguments.
 *   - set_rpm_limits() calls removed.
 *   - OutputCommand::collective_rpm renamed to collective_throttle.
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
// ---------------------------------------------------------------------------
bool BoomerangController::init()
{
    updateParams();

    _output_stage = new OutputStage();   // no RPM args
    _cyclic_mixer = new CyclicMixer(_param_cyc_phase.get(), _param_cyc_max_def.get());

    if (!_output_stage || !_cyclic_mixer) {
        PX4_ERR("boomerang: subsystem alloc failed");
        return false;
    }

    OutputCommand neutral{};
    neutral.armed = false;
    _output_stage->write(neutral);

    ScheduleOnInterval(static_cast<uint32_t>(1e6f / LOOP_RATE_HZ));
    PX4_INFO("boomerang: started at %.0f Hz", (double)LOOP_RATE_HZ);
    return true;
}

// ---------------------------------------------------------------------------
// Run — called by the work queue every 5 ms (200 Hz)
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

    OutputCommand neutral{};
    neutral.armed = false;

    switch (_state) {

    case State::WAITING_FOR_EKF:
        _output_stage->write(neutral);
        if (_attitude.timestamp > 0) {
            PX4_INFO("boomerang: EKF valid — capturing heading");
            _state = State::CAPTURING_HEADING;
        }
        break;

    case State::CAPTURING_HEADING: {
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

        _raw_ekf_yaw_rad = matrix::Eulerf(matrix::Quatf(_attitude.q)).psi();

        _publish_virtual_heading();
        _publish_despun_rates(_raw_ekf_yaw_rad);
        _run_control_loop(dt_s);
        break;

    case State::FAULT:
        _output_stage->write(neutral);
        if (_attitude.timestamp > 0) {
            PX4_INFO("boomerang: EKF recovered — re-waiting");
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
    _sub_torque_sp.update(&_torque_sp);
    _sub_attitude_virtual.update(&_attitude_virtual);
}

// ---------------------------------------------------------------------------
// _publish_virtual_heading
// ---------------------------------------------------------------------------
void BoomerangController::_publish_virtual_heading()
{
    const matrix::Quatf  q_ekf(_attitude.q);
    const matrix::Eulerf euler_ekf(q_ekf);

    // const matrix::Eulerf euler_virt(euler_ekf.phi(),
    //                                 euler_ekf.theta(),
    //                                 _virtual_heading_rad);
    const matrix::Eulerf euler_virt(0,0, _virtual_heading_rad);
    const matrix::Quatf q_virt(euler_virt);

    vehicle_attitude_s att_out = _attitude;
    q_virt.copyTo(att_out.q);
    _pub_attitude_virtual.publish(att_out);
}

// ---------------------------------------------------------------------------
// _publish_despun_rates
// ---------------------------------------------------------------------------
void BoomerangController::_publish_despun_rates(float ekf_yaw_rad)
{
    const float p = _ang_vel.xyz[0];
    const float q = _ang_vel.xyz[1];
    const float c = cosf(ekf_yaw_rad);
    const float s = sinf(ekf_yaw_rad);

    const float p_virt =  p * c + q * s;
    const float q_virt = -p * s + q * c;

    _dbg_p_virt = p_virt;
    _dbg_q_virt = q_virt;

    vehicle_angular_velocity_s ang_vel_out = _ang_vel;
    ang_vel_out.xyz[0] = p_virt;
    ang_vel_out.xyz[1] = q_virt;
    ang_vel_out.xyz[2] = 0.0f;
    _pub_ang_vel_override.publish(ang_vel_out);
}

// ---------------------------------------------------------------------------
// _run_control_loop
// ---------------------------------------------------------------------------
void BoomerangController::_run_control_loop(float /*dt_s*/)
{
    const bool armed = _is_armed();

    if (!armed) {
        OutputCommand zero{};
        zero.armed = false;
        _output_stage->write(zero);
        return;
    }

    const float raw_yaw  = _raw_ekf_yaw_rad;
    const float yaw_rate = _ang_vel.xyz[2];

    _azimuth_tracker.update(raw_yaw, yaw_rate, hrt_absolute_time());
    const AzimuthState &az = _azimuth_tracker.state();

    // -----------------------------------------------------------------------
    // Collective throttle — direct 0.0–1.0, no RPM conversion
    // -----------------------------------------------------------------------
    float collective_throttle = 0.0f;

    const bool att_sp_valid = (_att_sp.timestamp > 0)
        && _control_mode.flag_control_attitude_enabled
        && (_control_mode.flag_control_position_enabled
            || _control_mode.flag_control_velocity_enabled
            || _control_mode.flag_control_offboard_enabled);

    if (att_sp_valid) {
        // thrust_body[2] is negative for upward thrust in NED, so negate it.
        collective_throttle = math::constrain(-_att_sp.thrust_body[2], 0.0f, 1.0f);

    } else if (_manual.timestamp > 0 && _control_mode.flag_control_manual_enabled) {
        collective_throttle = math::constrain(_manual.throttle, 0.0f, 1.0f);
    }

    // -----------------------------------------------------------------------
    // Cyclic demand
    // -----------------------------------------------------------------------
    float cyclic_roll_virt  = 0.0f;
    float cyclic_pitch_virt = 0.0f;

    const bool torque_valid = (_torque_sp.timestamp > 0);

    if (torque_valid && _control_mode.flag_control_attitude_enabled) {
        cyclic_roll_virt  = math::constrain(_torque_sp.xyz[0] * _param_torque_roll_gain.get(),  -1.0f, 1.0f);
        cyclic_pitch_virt = math::constrain(_torque_sp.xyz[1] * _param_torque_pitch_gain.get(), -1.0f, 1.0f);

    } else if (_manual.timestamp > 0 && _control_mode.flag_control_manual_enabled) {
        const float expo = _param_pilot_expo.get();
        cyclic_pitch_virt = _expo(_deadband(_manual.pitch, 0.05f), expo);
        cyclic_roll_virt  = _expo(_deadband(_manual.roll,  0.05f), expo);
    }

    // Rotate virtual-body frame demands to NED frame
    const float c_hdg = cosf(_virtual_heading_rad);
    const float s_hdg = sinf(_virtual_heading_rad);
    const float cyclic_pitch_ned =  cyclic_pitch_virt * c_hdg + cyclic_roll_virt * s_hdg;
    const float cyclic_roll_ned  = -cyclic_pitch_virt * s_hdg + cyclic_roll_virt * c_hdg;

    _dbg_torque_roll  = cyclic_roll_ned;
    _dbg_torque_pitch = cyclic_pitch_ned;

    // -----------------------------------------------------------------------
    // Cyclic mixer → per-blade flap commands
    // -----------------------------------------------------------------------
    const CyclicMixerOutput mix = _cyclic_mixer->mix(cyclic_pitch_ned, cyclic_roll_ned, az.theta);

    OutputCommand cmd{};
    cmd.armed               = true;
    cmd.collective_throttle = collective_throttle;   // renamed from collective_rpm
    for (int i = 0; i < NUM_BLADES; ++i) {
        cmd.flap_cmd[i] = mix.flap_cmd[i];
    }
    _output_stage->write(cmd);
}

// ---------------------------------------------------------------------------
// _capture_reference_heading
// ---------------------------------------------------------------------------
void BoomerangController::_capture_reference_heading(float heading_rad)
{
    _virtual_heading_rad = heading_rad;

    param_t h = param_find("BC_VIRT_HDG");
    if (h != PARAM_INVALID) {
        param_set(h, &heading_rad);
    }

    _cyclic_mixer->set_phase_advance(_param_cyc_phase.get());
    _cyclic_mixer->set_max_deflection(_param_cyc_max_def.get());
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
}

// ---------------------------------------------------------------------------
// _is_armed
// ---------------------------------------------------------------------------
bool BoomerangController::_is_armed() const
{
    return (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
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

    PRINT_MODULE_DESCRIPTION(
        "Propeller-driven helicopter blade (boomerang) flight controller.\n"
        "Replaces ControlAllocator only.  mc_rate_control, mc_att_control,\n"
        "and mc_pos_control run above it in the standard stack.\n"
        "Publishes virtual heading and despun body rates to keep all upstream\n"
        "modules operating in the non-spinning platform frame.");
    PRINT_MODULE_USAGE_NAME("boomerang", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
    return 0;
}

// ---------------------------------------------------------------------------
// print_status
// ---------------------------------------------------------------------------
int BoomerangController::print_status()
{
    const char *state_str = "UNKNOWN";
    switch (_state) {
        case State::WAITING_FOR_EKF:    state_str = "WAITING_FOR_EKF";    break;
        case State::CAPTURING_HEADING:  state_str = "CAPTURING_HEADING";  break;
        case State::RUNNING:            state_str = "RUNNING";             break;
        case State::FAULT:              state_str = "FAULT";               break;
    }

    PX4_INFO("state:              %s", state_str);
    PX4_INFO("virtual heading:    %.1f deg", (double)math::degrees(_virtual_heading_rad));
    PX4_INFO("heading samples:    %d / %d", _heading_sample_count, HEADING_SAMPLES);

    if (_azimuth_tracker.state().valid) {
        PX4_INFO("rotor RPM:          %.1f", (double)_azimuth_tracker.state().rpm);
        PX4_INFO("blade0 azimuth:     %.1f deg",
                 (double)math::degrees(_azimuth_tracker.state().theta[0]));
    } else {
        PX4_INFO("azimuth tracker:    not valid");
    }

    PX4_INFO("despun p_virt:      %.4f rad/s", (double)_dbg_p_virt);
    PX4_INFO("despun q_virt:      %.4f rad/s", (double)_dbg_q_virt);
    PX4_INFO("cyclic roll:        %.3f", (double)_dbg_torque_roll);
    PX4_INFO("cyclic pitch:       %.3f", (double)_dbg_torque_pitch);
    PX4_INFO("torque_sp valid:    %s", (_torque_sp.timestamp > 0) ? "yes" : "no");

    if (_output_stage) {
        PX4_INFO("throttle:           %.3f", (double)_output_stage->last_cmd.collective_throttle);
        PX4_INFO("flap[0..3]:         %.3f  %.3f  %.3f  %.3f",
                 (double)_output_stage->last_cmd.flap_cmd[0],
                 (double)_output_stage->last_cmd.flap_cmd[1],
                 (double)_output_stage->last_cmd.flap_cmd[2],
                 (double)_output_stage->last_cmd.flap_cmd[3]);
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
