#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <px4_platform_common/px4_config.h>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/heading_led.h>

#include <drivers/drv_hrt.h>
#include <px4_arch/io_timer.h>   // px4_arch_configgpio / gpiowrite
#include <px4_platform_common/time.h>
using namespace time_literals;

class HeadingLed final
    : public ModuleBase
    , public ModuleParams
    , public px4::ScheduledWorkItem

{
public:
    HeadingLed();
    ~HeadingLed() override = default;

    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    bool init();
    static ModuleBase::Descriptor _descriptor;

private:
    void Run() override;

    uORB::Subscription _sub_attitude{ORB_ID(vehicle_attitude)};
    uORB::Subscription _sub_attitude_virtual{ORB_ID(vehicle_attitude_virtual)};
    uORB::Subscription _sub_angular_velocity{ORB_ID(vehicle_angular_velocity)};
    uORB::Subscription _sub_param_update{ORB_ID(parameter_update)};

    uORB::Publication<heading_led_s> _heading_led_pub{ORB_ID(heading_led_status)};

    vehicle_attitude_s _attitude{};
    vehicle_attitude_s _attitude_virtual{};

    // ---- params ----
    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::BC_VIRT_HDG>)  _param_ref_hdg,   // rad, set by Boomerang
        (ParamFloat<px4::params::BC_LED_WIN>)  _param_window      // half-window, deg
    )

    static constexpr float LOOP_RATE_HZ = 200.0f;
    int print_status();
    perf_counter_t _loop_perf{nullptr};
    hrt_abstime _start_time_us{0};
    bool        _gpio_configured{false};

    hrt_abstime led_curr_time{0};
};
