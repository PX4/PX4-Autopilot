#include <px4_arch/io_timer_hw_description.h>

constexpr io_timers_t io_timers[MAX_IO_TIMERS] = {
    initIOTimer(Timer::Timer3, DMA{DMA::Index1}),
    initIOTimer(Timer::Timer2, DMA{DMA::Index1}),
    initIOTimer(Timer::Timer5, DMA{DMA::Index1}),
    initIOTimer(Timer::Timer8, DMA{DMA::Index1}),
};

constexpr timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
    initIOTimerChannel(io_timers, {Timer::Timer3, Timer::Channel3}, {}),
    initIOTimerChannel(io_timers, {Timer::Timer3, Timer::Channel4}, {}),
    initIOTimerChannel(io_timers, {Timer::Timer2, Timer::Channel2}, {}),
    initIOTimerChannel(io_timers, {Timer::Timer2, Timer::Channel3}, {}),
    initIOTimerChannel(io_timers, {Timer::Timer5, Timer::Channel1}, {}),
    initIOTimerChannel(io_timers, {Timer::Timer5, Timer::Channel3}, {}),
    initIOTimerChannel(io_timers, {Timer::Timer8, Timer::Channel3}, {}),
    initIOTimerChannel(io_timers, {Timer::Timer8, Timer::Channel4}, {}),
};
