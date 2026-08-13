/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * PWM / Motor Timer configuration for CORVON 743V2
 *
 * Motor layout (10 DShot + 2 Servo):
 *   M1  = PE9   TIM1_CH1   (DMA)
 *   M2  = PE11  TIM1_CH2   (DMA)
 *   M3  = PE13  TIM1_CH3   (DMA)
 *   M4  = PE14  TIM1_CH4   (DMA)
 *   M5  = PB0   TIM3_CH3   (DMA)
 *   M6  = PB1   TIM3_CH4   (DMA)
 *   M7  = PD12  TIM4_CH1   (DMA)
 *   M8  = PD13  TIM4_CH2   (DMA)
 *   M9  = PD14  TIM4_CH3   (DMA)
 *   M10 = PD15  TIM4_CH4   (DMA, no BDShot - TIM4_CH4 has no capture DMA)
 *   M11 = PB14  TIM12_CH1  (no DMA, PWM only - servo / gimbal)
 *   M12 = PB15  TIM12_CH2  (no DMA, PWM only - servo / gimbal)
 *
 * NeoPixel LED strip (driver currently disabled):
 *   PA5 TIM2_CH1 (DMA) - driven by drivers/lights/neopixel (CONFIG_DRIVERS_LIGHTS_NEOPIXEL=n)
 *
 * Buzzer:
 *   PA7 TIM14_CH1 - driven by drivers/tone_alarm
 *
 * HRT:
 *   TIM2 (see HRT_TIMER in board_config.h). TIM2 is free because the NeoPixel
 *   driver is disabled. If NeoPixel is re-enabled and needs TIM2, move it to
 *   another timer rather than putting HRT back on TIM5 (TIM5 + I2C2 caused
 *   work-queue starvation on this board; STM32H743 has no TIM23 / TIM24).
 */

#include <px4_arch/io_timer_hw_description.h>

constexpr io_timers_t io_timers[MAX_IO_TIMERS] = {
	initIOTimer(Timer::Timer1,  DMA{DMA::Index1, DMA::Stream0}),  // M1-M4
	initIOTimer(Timer::Timer3,  DMA{DMA::Index1, DMA::Stream4}),  // M5/M6
	initIOTimer(Timer::Timer4,  DMA{DMA::Index1, DMA::Stream5}),  // M7-M10
	initIOTimer(Timer::Timer12),                                  // M11/M12 (no DMA)
};

constexpr timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
	initIOTimerChannel(io_timers, {Timer::Timer1,  Timer::Channel1}, {GPIO::PortE, GPIO::Pin9}),   // M1
	initIOTimerChannel(io_timers, {Timer::Timer1,  Timer::Channel2}, {GPIO::PortE, GPIO::Pin11}),  // M2
	initIOTimerChannel(io_timers, {Timer::Timer1,  Timer::Channel3}, {GPIO::PortE, GPIO::Pin13}),  // M3
	initIOTimerChannel(io_timers, {Timer::Timer1,  Timer::Channel4}, {GPIO::PortE, GPIO::Pin14}),  // M4
	initIOTimerChannel(io_timers, {Timer::Timer3,  Timer::Channel3}, {GPIO::PortB, GPIO::Pin0}),   // M5
	initIOTimerChannel(io_timers, {Timer::Timer3,  Timer::Channel4}, {GPIO::PortB, GPIO::Pin1}),   // M6
	initIOTimerChannel(io_timers, {Timer::Timer4,  Timer::Channel1}, {GPIO::PortD, GPIO::Pin12}),  // M7
	initIOTimerChannel(io_timers, {Timer::Timer4,  Timer::Channel2}, {GPIO::PortD, GPIO::Pin13}),  // M8
	initIOTimerChannel(io_timers, {Timer::Timer4,  Timer::Channel3}, {GPIO::PortD, GPIO::Pin14}),  // M9
	initIOTimerChannel(io_timers, {Timer::Timer4,  Timer::Channel4}, {GPIO::PortD, GPIO::Pin15}),  // M10
	initIOTimerChannel(io_timers, {Timer::Timer12, Timer::Channel1}, {GPIO::PortB, GPIO::Pin14}),  // M11 (servo)
	initIOTimerChannel(io_timers, {Timer::Timer12, Timer::Channel2}, {GPIO::PortB, GPIO::Pin15}),  // M12 (servo)
};

constexpr io_timers_channel_mapping_t io_timers_channel_mapping =
	initIOTimerChannelMapping(io_timers, timer_io_channels);
