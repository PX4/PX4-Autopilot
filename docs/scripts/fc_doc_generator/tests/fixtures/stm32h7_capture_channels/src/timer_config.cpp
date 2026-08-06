#include <px4_arch/io_timer_hw_description.h>

/* Fixture modelling boards like cuav/x25-evo that use initIOTimerChannelCapture
 * for their higher-numbered outputs.  Timer5 and Timer4 have DMA (DShot); the
 * remaining three timers do not.
 */

constexpr io_timers_t io_timers[MAX_IO_TIMERS] = {
    initIOTimer(Timer::Timer5, DMA{DMA::Index1}),
    initIOTimer(Timer::Timer4, DMA{DMA::Index1}),
    initIOTimer(Timer::Timer1),
    initIOTimer(Timer::Timer8),
    initIOTimer(Timer::Timer12),
};

constexpr timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
    initIOTimerChannel(io_timers, {Timer::Timer5, Timer::Channel4}, {}),
    initIOTimerChannel(io_timers, {Timer::Timer5, Timer::Channel3}, {}),
    initIOTimerChannel(io_timers, {Timer::Timer5, Timer::Channel2}, {}),
    initIOTimerChannel(io_timers, {Timer::Timer5, Timer::Channel1}, {}),
    initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel2}, {}),
    initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel3}, {}),
    initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel1}, {}),
    initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel4}, {}),
    initIOTimerChannelCapture(io_timers, {Timer::Timer1, Timer::Channel2}, {}),
    initIOTimerChannelCapture(io_timers, {Timer::Timer1, Timer::Channel1}, {}),
    initIOTimerChannelCapture(io_timers, {Timer::Timer1, Timer::Channel3}, {}),
    initIOTimerChannelCapture(io_timers, {Timer::Timer8, Timer::Channel2}, {}),
    initIOTimerChannelCapture(io_timers, {Timer::Timer8, Timer::Channel3}, {}),
    initIOTimerChannelCapture(io_timers, {Timer::Timer8, Timer::Channel1}, {}),
    initIOTimerChannelCapture(io_timers, {Timer::Timer12, Timer::Channel1}, {}),
    initIOTimerChannelCapture(io_timers, {Timer::Timer12, Timer::Channel2}, {}),
};

constexpr io_timers_channel_mapping_t io_timers_channel_mapping =
    initIOTimerChannelMapping(io_timers, timer_io_channels);
