#include "HeadingLed.hpp"

#include <px4_platform_common/log.h>
#include <matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <math.h>
#include <px4_arch/io_timer.h>

// -----------------------------------------------------------------------
// *** FILL THIS IN after: grep -n "FMU_CH5\|AUX5" boards/cubepilot/cubeorangeplus/src/board_config.h
// Example form (fmu-v5): GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN13
// -----------------------------------------------------------------------
#define GPIO_HEADING_LED  /* PD14 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN14)

HeadingLed::HeadingLed()
    : ModuleParams(nullptr)
    , ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{}

bool HeadingLed::init()
{
    updateParams();
    px4_arch_configgpio(GPIO_HEADING_LED);
    px4_arch_gpiowrite(GPIO_HEADING_LED, false);
    ScheduleOnInterval(static_cast<uint32_t>(1e6f / LOOP_RATE_HZ));
    _loop_perf = perf_alloc(PC_INTERVAL, "my_module_loop_interval");
    PX4_INFO("heading_led: started");
    return true;
}

void HeadingLed::Run()
{
    // Trigger the counter at the top of your loop
    perf_count(_loop_perf);

    if (should_exit()) {
        px4_arch_gpiowrite(GPIO_HEADING_LED, false);
        ScheduleClear();
        perf_free(_loop_perf);
        ModuleBase::exit_and_cleanup(_descriptor);
        return;
    }

    // Refresh params on change
    parameter_update_s param_upd{};
    if (_sub_param_update.update(&param_upd)) {
        updateParams();
    }

    if (!_sub_attitude.update(&_attitude) || _attitude.timestamp == 0) {
        px4_arch_gpiowrite(GPIO_HEADING_LED, false);
        return;
    }

    const float yaw     = matrix::Eulerf(matrix::Quatf(_attitude.q)).psi();
    // const float ref     = _param_ref_hdg.get();                        // rad
    const float ref     = matrix::Eulerf(matrix::Quatf(_attitude_virtual.q)).psi();  // rad, from virtual attitude
    const float win_rad = math::radians(_param_window.get());          // deg → rad

    // Shortest angular distance with wraparound
    float diff = yaw - ref;
    while (diff >  M_PI_F) { diff -= 2.0f * M_PI_F; }
    while (diff < -M_PI_F) { diff += 2.0f * M_PI_F; }

    const bool in_window = (fabsf(diff) <= win_rad);
    px4_arch_gpiowrite(GPIO_HEADING_LED, in_window);
    // px4_arch_gpiowrite(GPIO_HEADING_LED, true);
}

ModuleBase::Descriptor HeadingLed::_descriptor{
    &HeadingLed::task_spawn,
    &HeadingLed::custom_command,
    &HeadingLed::print_usage
};

int HeadingLed::task_spawn(int argc, char *argv[])
{
    HeadingLed *instance = new HeadingLed();
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

int HeadingLed::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int HeadingLed::print_usage(const char *reason)
{
    if (reason) { PX4_WARN("%s\n", reason); }
    PRINT_MODULE_USAGE_NAME("heading_led", "driver");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
    return 0;
}

extern "C" __EXPORT int heading_led_main(int argc, char *argv[])
{
	return ModuleBase::main(HeadingLed::_descriptor, argc, argv);
}

int HeadingLed::print_status()
{
    const float yaw_deg = math::degrees(
        matrix::Eulerf(matrix::Quatf(_attitude.q)).psi());
    // const float ref_deg = math::degrees(_param_ref_hdg.get());
    const float ref_deg = math::degrees(matrix::Eulerf(matrix::Quatf(_attitude_virtual.q)).psi());
    const float win_deg = _param_window.get();

    float diff = yaw_deg - ref_deg;
    while (diff >  180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;

    PX4_INFO("ref heading:  %.1f deg", (double)ref_deg);
    PX4_INFO("current yaw:  %.1f deg", (double)yaw_deg);
    PX4_INFO("diff:         %.1f deg", (double)diff);
    PX4_INFO("window:       +/-%.1f deg", (double)win_deg);
    PX4_INFO("LED:          %s", (fabsf(diff) <= win_deg) ? "ON" : "OFF");

    // print current loop rate
    perf_print_counter(_loop_perf);

    return 0;
}
