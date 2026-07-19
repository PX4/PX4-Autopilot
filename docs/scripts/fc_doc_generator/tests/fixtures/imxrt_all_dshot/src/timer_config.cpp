#include <px4_arch/io_timer_hw_description.h>

constexpr io_timers_t io_timers[MAX_IO_TIMERS] = {
};

constexpr timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
    initIOTimerChannelDshot(io_timers, {PWM::PWM2_PWM_A, PWM::Submodule0}, {}),
    initIOTimerChannelDshot(io_timers, {PWM::PWM2_PWM_B, PWM::Submodule0}, {}),
    initIOTimerChannelDshot(io_timers, {PWM::PWM2_PWM_A, PWM::Submodule1}, {}),
    initIOTimerChannelDshot(io_timers, {PWM::PWM2_PWM_B, PWM::Submodule1}, {}),
    initIOTimerChannelDshot(io_timers, {PWM::PWM4_PWM_A, PWM::Submodule1}, {}),
    initIOTimerChannelDshot(io_timers, {PWM::PWM4_PWM_B, PWM::Submodule1}, {}),
    initIOTimerChannelDshot(io_timers, {PWM::PWM4_PWM_A, PWM::Submodule2}, {}),
    initIOTimerChannelDshot(io_timers, {PWM::PWM4_PWM_B, PWM::Submodule2}, {}),
};
