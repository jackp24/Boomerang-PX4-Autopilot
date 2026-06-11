px4_add_module(
    MODULE modules__heading_led
    MAIN heading_led
    SRCS HeadingLed.cpp
    DEPENDS
        px4_work_queue
)
