#include <px4_arch/io_timer_hw_description.h>

/* Fixture modelling an STM32F7 board (e.g. Pixhawk 5X / fmu-v5x).
 * Timer1 and Timer4 have DMA configured in io_timers[], which would ordinarily
 * indicate DShot capability.  This fixture verifies that DShot is suppressed at
 * the family level (DSHOT_UNSUPPORTED_FAMILIES) regardless of DMA presence.
 * Timer12 has no DMA.  Timer5 is capture-only (one initIOTimerChannelCapture entry).
 */

constexpr io_timers_t io_timers[MAX_IO_TIMERS] = {
    initIOTimer(Timer::Timer1, DMA{DMA::Index2, DMA::Stream5, DMA::Channel6}),
    initIOTimer(Timer::Timer4, DMA{DMA::Index1, DMA::Stream6, DMA::Channel2}),
    initIOTimer(Timer::Timer12),
    initIOTimer(Timer::Timer5),
};

constexpr timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
    initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel4}, {}),
    initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel3}, {}),
    initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel2}, {}),
    initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel1}, {}),
    initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel2}, {}),
    initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel3}, {}),
    initIOTimerChannel(io_timers, {Timer::Timer12, Timer::Channel1}, {}),
    initIOTimerChannel(io_timers, {Timer::Timer12, Timer::Channel2}, {}),
    initIOTimerChannelCapture(io_timers, {Timer::Timer5, Timer::Channel4}, {}),
};

constexpr io_timers_channel_mapping_t io_timers_channel_mapping =
    initIOTimerChannelMapping(io_timers, timer_io_channels);
