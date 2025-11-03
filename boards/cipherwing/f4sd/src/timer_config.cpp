/****************************************************************************
 *  Cipherwing F4SD - IO timer configuration
 *
 *  PWM pins:
 *   TIM3: PB0 (CH3), PB1 (CH4)
 *   TIM4: PB7 (CH2)
 *   TIM8: PC6 (CH1), PC8 (CH3), PC9 (CH4)
 *
 *  HRT on TIM5 (set in board_config.h). Do NOT list TIM5 here.
 ****************************************************************************/

#include <px4_arch/io_timer_hw_description.h>

// Valid DMA mappings on STM32F405:
//   TIM3_UP : DMA1 Stream2 Channel5
//   TIM4_UP : DMA1 Stream6 Channel2
//   TIM8_UP : DMA2 Stream1 Channel7
constexpr io_timers_t io_timers[] = {
	initIOTimer(Timer::Timer3, DMA{DMA::Index1, DMA::Stream2, DMA::Channel5}), // TIM3
	initIOTimer(Timer::Timer4, DMA{DMA::Index1, DMA::Stream6, DMA::Channel2}), // TIM4
	initIOTimer(Timer::Timer8, DMA{DMA::Index2, DMA::Stream1, DMA::Channel7}), // TIM8
};

constexpr timer_io_channels_t timer_io_channels[] = {
	// M1: PB0 - TIM3 CH3
	initIOTimerChannel(io_timers, {Timer::Timer3, Timer::Channel3}, {GPIO::PortB, GPIO::Pin0}),
	// M2: PB1 - TIM3 CH4
	initIOTimerChannel(io_timers, {Timer::Timer3, Timer::Channel4}, {GPIO::PortB, GPIO::Pin1}),
	// M3: PC8 - TIM8 CH3
	initIOTimerChannel(io_timers, {Timer::Timer8, Timer::Channel3}, {GPIO::PortC, GPIO::Pin8}),
	// M4: PC9 - TIM8 CH4
	initIOTimerChannel(io_timers, {Timer::Timer8, Timer::Channel4}, {GPIO::PortC, GPIO::Pin9}),
	// M5: PC6 - TIM8 CH1
	initIOTimerChannel(io_timers, {Timer::Timer8, Timer::Channel1}, {GPIO::PortC, GPIO::Pin6}),
	// M6: PB7 - TIM4 CH2
	initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel2}, {GPIO::PortB, GPIO::Pin7}),
};

// Export mapping required by PX4 core
constexpr io_timers_channel_mapping_t io_timers_channel_mapping =
	initIOTimerChannelMapping(io_timers, timer_io_channels);
