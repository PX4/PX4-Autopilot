/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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

#include <px4_arch/io_timer_hw_description.h>

/* Timer allocation
 *
 * TIM8_CH4  T AUX_CH1
 * TIM8_CH3  T AUX_CH2
 * TIM8_CH2  T AUX_CH3
 * TIM8_CH1  T AUX_CH4
 * TIM2_CH3  T AUX_CH5
 * TIM2_CH1  T AUX_CH6
 * TIM9_CH1  T AUX_CH7
 * TIM9_CH2  T AUX_CH8
 *
 * TIM4_CH1 T FMU_CH1
 * TIM4_CH2 T FMU_CH2
 * TIM4_CH3 T FMU_CH3
 * TIM4_CH4 T FMU_CH4
 * TIM1_CH2 T FMU_CH5                 > PWM OUT or GPIO
 * TIM1_CH3 T FMU_CH6                 > PWM OUT or GPIO
 *
 * TIM1_CH1  T HEATER               > PWM output or GPIO
 * TIM3_CH2  T SPIX_SYNC              > Pulse or GPIO strobe
 * TIM3_CH4  T FMU_CAP1              < input capture
 * TIM13_CH1 T BUZZER_1              - Driven by other driver
 *
 * TIM5_CH4 T FMU_PPM_INPUT       - ? Sampled byt HRT by other driver
 */

constexpr io_timers_t io_timers[MAX_IO_TIMERS] = {
	initIOTimer(Timer::Timer8),
	initIOTimer(Timer::Timer3),
	initIOTimer(Timer::Timer9),
	initIOTimer(Timer::Timer4),
	initIOTimer(Timer::Timer1),
	initIOTimer(Timer::Timer2),
	//initIOTimer(Timer::Timer13),
	//initIOTimer(Timer::Timer5),
	//initIOTimer(Timer::Timer14),
	//initIOTimer(Timer::Timer16),
};

constexpr timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
	//AUX CH1~8
	initIOTimerChannel(io_timers, {Timer::Timer8, Timer::Channel1}, {GPIO::PortC, GPIO::Pin6}),
	initIOTimerChannel(io_timers, {Timer::Timer8, Timer::Channel2}, {GPIO::PortC, GPIO::Pin7}),
	initIOTimerChannel(io_timers, {Timer::Timer8, Timer::Channel3}, {GPIO::PortC, GPIO::Pin8}),
	initIOTimerChannel(io_timers, {Timer::Timer8, Timer::Channel4}, {GPIO::PortC, GPIO::Pin9}),
	initIOTimerChannel(io_timers, {Timer::Timer2, Timer::Channel3}, {GPIO::PortB, GPIO::Pin10}),
	initIOTimerChannel(io_timers, {Timer::Timer2, Timer::Channel1}, {GPIO::PortA, GPIO::Pin15}),
	initIOTimerChannel(io_timers, {Timer::Timer9, Timer::Channel1}, {GPIO::PortE, GPIO::Pin5}),
	initIOTimerChannel(io_timers, {Timer::Timer9, Timer::Channel2}, {GPIO::PortE, GPIO::Pin6}),
	//FMU CH1~6
	initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel1}, {GPIO::PortD, GPIO::Pin12}),
	initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel2}, {GPIO::PortD, GPIO::Pin13}),
	initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel3}, {GPIO::PortD, GPIO::Pin14}),
	initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel4}, {GPIO::PortD, GPIO::Pin15}),
	initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel2}, {GPIO::PortE, GPIO::Pin11}),
	initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel3}, {GPIO::PortA, GPIO::Pin10}),
	//others
	initIOTimerChannelCapture(io_timers, {Timer::Timer3, Timer::Channel4}, {GPIO::PortB, GPIO::Pin1}), /* FMU_CAP1*/
};

constexpr io_timers_channel_mapping_t io_timers_channel_mapping =
	initIOTimerChannelMapping(io_timers, timer_io_channels);
