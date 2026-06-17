/****************************************************************************
 * BoomerangEstimator.cpp
 ****************************************************************************/

#include "BoomerangEstimator.hpp"

// Out-of-line definition for static constexpr array (C++14 compatibility)
constexpr float BoomerangEstimator::BLADE_OFFSET_RAD[BoomerangEstimator::NUM_BLADES];

// ---------------------------------------------------------------------------
// Descriptor
// ---------------------------------------------------------------------------
ModuleBase::Descriptor BoomerangEstimator::_descriptor{
    &BoomerangEstimator::task_spawn,
    &BoomerangEstimator::custom_command,
    &BoomerangEstimator::print_usage
};

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
BoomerangEstimator::BoomerangEstimator()
    : ModuleParams(nullptr)
    , ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{}

BoomerangEstimator::~BoomerangEstimator()
{
    perf_free(_loop_perf);
}

// ---------------------------------------------------------------------------
// init
// ---------------------------------------------------------------------------
bool BoomerangEstimator::init()
{
    updateParams();
    ScheduleOnInterval(static_cast<uint32_t>(1e6f / LOOP_RATE_HZ));
    PX4_INFO("boomerang_estimator: started at %.0f Hz", (double)LOOP_RATE_HZ);
    _loop_perf = perf_alloc(PC_ELAPSED, "boomerang_estimator: loop time");
    return true;
}

// ---------------------------------------------------------------------------
// Run — 200 Hz
// ---------------------------------------------------------------------------
void BoomerangEstimator::Run()

{
    if (should_exit()) {
        ScheduleClear();
        ModuleBase::exit_and_cleanup(_descriptor);
        return;
    }
    perf_begin(_loop_perf);

    // Refresh params on change
    parameter_update_s param_upd{};
    if (_sub_param_update.update(&param_upd)) {
        updateParams();
    }

    // Always pull fresh EKF2 data — subscribing to raw vehicle_attitude
    // is safe here because we publish on vehicle_attitude_virtual (different topic).
    _sub_attitude.update(&_attitude);
    _sub_ang_vel.update(&_ang_vel);

    switch (_state) {

    case State::WAITING_FOR_EKF:
        if (_attitude.timestamp > 0) {
            PX4_INFO("boomerang_estimator: EKF valid — capturing heading");
            _state = State::CAPTURING_HEADING;
        }
        break;

    case State::CAPTURING_HEADING: {
        const float yaw = matrix::Eulerf(matrix::Quatf(_attitude.q)).psi();
        _heading_sin_sum += sinf(yaw);
        _heading_cos_sum += cosf(yaw);
        ++_heading_sample_count;

        if (_heading_sample_count >= HEADING_SAMPLES) {
            const float mean_yaw = atan2f(_heading_sin_sum, _heading_cos_sum);
            _capture_reference_heading(mean_yaw);
            _state = State::RUNNING;
            PX4_INFO("boomerang_estimator: virtual heading = %.1f deg — RUNNING",
                     (double)math::degrees(mean_yaw));
        }
        break;
    }

    case State::RUNNING: {
        if (_attitude.timestamp == 0) {
            PX4_WARN("boomerang_estimator: EKF attitude lost — FAULT");
            _state = State::FAULT;
            break;
        }

        const float ekf_yaw  = matrix::Eulerf(matrix::Quatf(_attitude.q)).psi();
        const float yaw_rate = _ang_vel.xyz[2];

        _publish_virtual_attitude();
        _publish_despun_rates(ekf_yaw);
        _publish_azimuth(ekf_yaw, yaw_rate);
        break;
    }

    case State::FAULT:
        if (_attitude.timestamp > 0) {
            PX4_INFO("boomerang_estimator: EKF recovered — re-waiting");
            _heading_sample_count = 0;
            _heading_sin_sum      = 0.0f;
            _heading_cos_sum      = 0.0f;
            _state = State::WAITING_FOR_EKF;
        }
        break;
    }
    perf_end(_loop_perf);
}

// ---------------------------------------------------------------------------
// _capture_reference_heading
// ---------------------------------------------------------------------------
void BoomerangEstimator::_capture_reference_heading(float heading_rad)
{
    _virtual_heading_rad = heading_rad;

    // Persist to parameter so QGC can display it and heading_led can read it
    param_t h = param_find("BC_VIRT_HDG");
    if (h != PARAM_INVALID) {
        param_set(h, &heading_rad);
    }
}

// ---------------------------------------------------------------------------
// _publish_virtual_attitude
// Replaces the spinning EKF2 yaw with the fixed virtual heading.
// Published on vehicle_attitude_virtual — never stomps vehicle_attitude.
// ---------------------------------------------------------------------------
void BoomerangEstimator::_publish_virtual_attitude()
{
    const matrix::Quatf  q_ekf(_attitude.q);
    const matrix::Eulerf euler_ekf(q_ekf);

    // // Replace yaw with the fixed virtual heading; keep raw roll/pitch
    // const matrix::Eulerf euler_virt(euler_ekf.phi(),
    //                                 euler_ekf.theta(),
    //                                 _virtual_heading_rad);
    const matrix::Eulerf euler_virt(0.0f,
                                    0.0f,
                                    _virtual_heading_rad);

    const matrix::Quatf q_virt(euler_virt);

    vehicle_attitude_s out{};
    out.timestamp        = hrt_absolute_time();
    out.timestamp_sample = _attitude.timestamp;
    q_virt.copyTo(out.q);

    _pub_attitude_virtual.publish(out);
}

// ---------------------------------------------------------------------------
// _publish_despun_rates
// Rotates spinning body rates into the virtual (non-spinning) body frame.
//   p_virt =  p * cos(θ) + q * sin(θ)
//   q_virt = -p * sin(θ) + q * cos(θ)
//   r_virt =  0  (spin is the rotor, not a platform yaw rate)
// ---------------------------------------------------------------------------
void BoomerangEstimator::_publish_despun_rates(float ekf_yaw_rad)
{
    const float p = _ang_vel.xyz[0];
    const float q = _ang_vel.xyz[1];
    const float c = cosf(ekf_yaw_rad);
    const float s = sinf(ekf_yaw_rad);

    vehicle_angular_velocity_s out = _ang_vel;
    out.xyz[0] =  p * c + q * s;
    out.xyz[1] = -p * s + q * c;
    out.xyz[2] =  0.0f;

    _pub_ang_vel_despun.publish(out);
}

// ---------------------------------------------------------------------------
// _publish_azimuth
// Computes per-blade azimuth from EKF2 yaw and publishes for ControlAllocator.
// ---------------------------------------------------------------------------
void BoomerangEstimator::_publish_azimuth(float ekf_yaw_rad, float yaw_rate)
{
    boomerang_azimuth_s out{};
    out.timestamp   = hrt_absolute_time();
    out.omega_rad_s = yaw_rate;
    out.rpm         = (yaw_rate / (2.0f * M_PI_F)) * 60.0f;
    out.valid       = true;

    for (int i = 0; i < NUM_BLADES; ++i) {
        out.theta[i] = matrix::wrap_pi(ekf_yaw_rad + BLADE_OFFSET_RAD[i]);
    }

    _pub_azimuth.publish(out);
}

// ---------------------------------------------------------------------------
// print_status
// ---------------------------------------------------------------------------
int BoomerangEstimator::print_status()
{
    const char *state_str = "UNKNOWN";
    switch (_state) {
        case State::WAITING_FOR_EKF:   state_str = "WAITING_FOR_EKF";   break;
        case State::CAPTURING_HEADING: state_str = "CAPTURING_HEADING"; break;
        case State::RUNNING:           state_str = "RUNNING";           break;
        case State::FAULT:             state_str = "FAULT";             break;
    }

    PX4_INFO("state:            %s", state_str);
    PX4_INFO("virtual heading:  %.1f deg", (double)math::degrees(_virtual_heading_rad));
    PX4_INFO("heading samples:  %d / %d", _heading_sample_count, HEADING_SAMPLES);

    if (_state == State::RUNNING) {
        const float ekf_yaw = matrix::Eulerf(matrix::Quatf(_attitude.q)).psi();
        PX4_INFO("EKF yaw (blade0): %.1f deg", (double)math::degrees(ekf_yaw));
        PX4_INFO("spin rate:        %.1f rpm  (%.3f rad/s)",
                 (double)((_ang_vel.xyz[2] / (2.0f * M_PI_F)) * 60.0f),
                 (double)_ang_vel.xyz[2]);
    }
	// Print perf
	perf_print_counter(_loop_perf);

    return 0;
}

// ---------------------------------------------------------------------------
// task_spawn / custom_command / print_usage
// ---------------------------------------------------------------------------
int BoomerangEstimator::task_spawn(int argc, char *argv[])
{
    BoomerangEstimator *instance = new BoomerangEstimator();
    if (!instance) { PX4_ERR("alloc failed"); return PX4_ERROR; }

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

int BoomerangEstimator::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int BoomerangEstimator::print_usage(const char *reason)
{
    if (reason) { PX4_WARN("%s\n", reason); }
    PRINT_MODULE_DESCRIPTION(
        "Boomerang companion estimator.\n"
        "Publishes vehicle_attitude_virtual (despun heading), overrides\n"
        "vehicle_angular_velocity with despun body rates, and publishes\n"
        "boomerang_azimuth for ControlAllocator.");
    PRINT_MODULE_USAGE_NAME("boomerang_estimator", "estimator");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
    return 0;
}

extern "C" __EXPORT int boomerang_estimator_main(int argc, char *argv[])
{
    return ModuleBase::main(BoomerangEstimator::_descriptor, argc, argv);
}
