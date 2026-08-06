#include <px4_arch/io_timer_hw_description.h>

constexpr io_timers_t io_timers[MAX_IO_TIMERS] = {
    initIOTimer(Timer::Timer1),
    initIOTimer(Timer::Timer4),
    initIOTimer(Timer::Timer5),
};

constexpr timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
    initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel1}, {}),
    initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel2}, {}),
    initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel3}, {}),
    initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel4}, {}),
    initIOTimerChannel(io_timers, {Timer::Timer5, Timer::Channel2}, {}),
    initIOTimerChannel(io_timers, {Timer::Timer5, Timer::Channel3}, {}),
};
