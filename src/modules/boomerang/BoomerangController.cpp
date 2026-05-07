/****************************************************************************
 * BoomerangController.cpp
 *
 * Changes vs. previous revision:
 *   1. Removed hand-rolled proportional attitude controller.
 *   2. Added _publish_despun_rates(): overrides vehicle_angular_velocity with
 *      rates rotated into the virtual (non-spinning) body frame.  mc_rate_control
 *      reads this and operates entirely in the platform frame.
 *   3. Added _sub_torque_sp subscription to vehicle_torque_setpoint.
 *      mc_rate_control writes here; we read roll/pitch and pass into CyclicMixer.
 *   4. BC_ATT_P / BC_ATT_D parameters removed; replaced by
 *      BC_TORQUE_ROLL_GAIN / BC_TORQUE_PITCH_GAIN.
 *   5. vehicle_attitude override priority note: both EKF2 and this module
 *      publish on ORB_ID(vehicle_attitude).  In PX4 v1.17 the last publisher
 *      wins on the single-instance topic.  We publish AFTER reading _attitude
 *      each tick, so our virtual-heading version is always the most recent
 *      message mc_att_control will see.  Run 'uorb top vehicle_attitude' to
 *      confirm only one instance exists.
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

    _output_stage = new OutputStage(_param_rpm_min.get(), _param_rpm_max.get());
    _cyclic_mixer = new CyclicMixer(_param_cyc_phase.get(), _param_cyc_max_def.get());

    if (!_output_stage || !_cyclic_mixer) {
        PX4_ERR("boomerang: subsystem alloc failed");
        return false;
    }

    // Publish neutral command immediately — keeps servos quiet at power-on.
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
        // Publish both overrides every tick so upstream modules always see
        // the virtual frame.  Order matters: attitude first, then rates.
        _publish_virtual_heading();
        _publish_despun_rates(matrix::Eulerf(matrix::Quatf(_attitude.q)).psi());
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
}

// ---------------------------------------------------------------------------
// _publish_virtual_heading
//
// Overwrites the spinning EKF2 yaw with the fixed virtual heading so that
// mc_att_control and Navigator see a stable, non-rotating heading.
// Roll and pitch are passed through unchanged from EKF2.
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
// _publish_despun_rates
//
// Rotates the raw spinning-frame body rates into the virtual (non-spinning)
// body frame and publishes the result on vehicle_angular_velocity so that
// mc_rate_control operates entirely in the platform frame.
//
// Transform (rotation by -θ around Z):
//   p_virt =  p_body * cos(θ) + q_body * sin(θ)
//   q_virt = -p_body * sin(θ) + q_body * cos(θ)
//   r_virt =  0   (the spin IS the rotor; there is no platform yaw rate)
//
// @param ekf_yaw_rad  Current EKF2 yaw == blade-0 azimuth in NED (radians).
// ---------------------------------------------------------------------------
void BoomerangController::_publish_despun_rates(float ekf_yaw_rad)
{
    const float p = _ang_vel.xyz[0];
    const float q = _ang_vel.xyz[1];
    const float c = cosf(ekf_yaw_rad);
    const float s = sinf(ekf_yaw_rad);

    const float p_virt =  p * c + q * s;
    const float q_virt = -p * s + q * c;
    // r_virt is zeroed — spin is the rotor, not a platform yaw motion.
    // If a small residual yaw wobble needs to be fed to mc_rate_control,
    // replace 0.0f with a low-pass filtered (_ang_vel.xyz[2] - omega_rotor).

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

    // -----------------------------------------------------------------------
    // Azimuth tracker — always uses raw EKF yaw, not the virtual-heading value.
    // -----------------------------------------------------------------------
    const matrix::Eulerf euler_raw(matrix::Quatf(_attitude.q));
    const float raw_yaw  = euler_raw.psi();
    const float yaw_rate = _ang_vel.xyz[2];

    _azimuth_tracker.update(raw_yaw, yaw_rate, hrt_absolute_time());
    const AzimuthState &az = _azimuth_tracker.state();

    // -----------------------------------------------------------------------
    // Collective RPM from thrust setpoint
    // -----------------------------------------------------------------------
    float collective_rpm = _param_rpm_hover.get();

    const bool att_sp_valid = (_att_sp.timestamp > 0)
        && _control_mode.flag_control_attitude_enabled
        && (_control_mode.flag_control_position_enabled
            || _control_mode.flag_control_velocity_enabled
            || _control_mode.flag_control_offboard_enabled);

    if (att_sp_valid) {
        const float thrust_norm = math::constrain(-_att_sp.thrust_body[2], 0.0f, 1.0f);
        collective_rpm = _param_rpm_min.get()
                       + thrust_norm * (_param_rpm_max.get() - _param_rpm_min.get());

    } else if (_manual.timestamp > 0 && _control_mode.flag_control_manual_enabled) {
        const float throttle = math::constrain(_manual.throttle, 0.0f, 1.0f);
        collective_rpm = _param_rpm_min.get()
                       + throttle * (_param_rpm_max.get() - _param_rpm_min.get());
    }

    collective_rpm = math::constrain(collective_rpm, _param_rpm_min.get(), _param_rpm_max.get());


    // Cyclic demand — virtual-body frame → NED frame rotation
    float cyclic_roll_virt  = 0.0f;
    float cyclic_pitch_virt = 0.0f;

    const bool torque_valid = (_torque_sp.timestamp > 0);

    if (torque_valid && _control_mode.flag_control_attitude_enabled) {
        // mc_rate_control outputs torque in the virtual body frame because it
        // receives our despun rates.  Apply gain then rotate to NED below.
        cyclic_roll_virt  = math::constrain(_torque_sp.xyz[0] * _param_torque_roll_gain.get(), -1.0f, 1.0f);
        cyclic_pitch_virt = math::constrain(_torque_sp.xyz[1] * _param_torque_pitch_gain.get(), -1.0f, 1.0f);

    } else if (_manual.timestamp > 0 && _control_mode.flag_control_manual_enabled) {
        // Stick demands are in the pilot/virtual-body frame.
        const float expo = _param_pilot_expo.get();
        cyclic_pitch_virt = _expo(_deadband(_manual.pitch, 0.05f), expo);
        cyclic_roll_virt  = _expo(_deadband(_manual.roll,  0.05f), expo);
    }

    // Rotate virtual-body frame to NED frame
    const float c_hdg = cosf(_virtual_heading_rad);
    const float s_hdg = sinf(_virtual_heading_rad);
    const float cyclic_pitch_ned =  cyclic_pitch_virt * c_hdg + cyclic_roll_virt * s_hdg;
    const float cyclic_roll_ned  = -cyclic_pitch_virt * s_hdg + cyclic_roll_virt * c_hdg;

    _dbg_torque_roll  = cyclic_roll_ned;
    _dbg_torque_pitch = cyclic_pitch_ned;

    // -----------------------------------------------------------------------
    // Cyclic mixer to per-blade flap commands
    // -----------------------------------------------------------------------
    const CyclicMixerOutput mix = _cyclic_mixer->mix(cyclic_pitch_ned, cyclic_roll_ned, az.theta);

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
