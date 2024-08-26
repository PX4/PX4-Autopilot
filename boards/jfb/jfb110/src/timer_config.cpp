/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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

constexpr io_timers_t io_timers[MAX_IO_TIMERS] = {
	initIOTimer(Timer::Timer1),
	initIOTimer(Timer::Timer3),
	initIOTimer(Timer::Timer4, DMA{DMA::Index1}),
	initIOTimer(Timer::Timer5, DMA{DMA::Index1}),
	initIOTimer(Timer::Timer12),
	initIOTimer(Timer::Timer15),
};

constexpr timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
	initIOTimerChannel(io_timers, {Timer::Timer15, Timer::Channel1}, {GPIO::PortA, GPIO::Pin2}),   // pwm(1)
	initIOTimerChannel(io_timers, {Timer::Timer15, Timer::Channel2}, {GPIO::PortE, GPIO::Pin6}),   // pwm(2)
	initIOTimerChannel(io_timers, {Timer::Timer3,  Timer::Channel2}, {GPIO::PortA, GPIO::Pin7}),   // pwm(3)
	initIOTimerChannel(io_timers, {Timer::Timer3,  Timer::Channel1}, {GPIO::PortA, GPIO::Pin6}),   // pwm(4)
	initIOTimerChannel(io_timers, {Timer::Timer4,  Timer::Channel4}, {GPIO::PortD, GPIO::Pin15}),  // pwm(5)
	initIOTimerChannel(io_timers, {Timer::Timer1,  Timer::Channel1}, {GPIO::PortE, GPIO::Pin9}),   // pwm(6)
	initIOTimerChannel(io_timers, {Timer::Timer5,  Timer::Channel2}, {GPIO::PortH, GPIO::Pin11}),  // pwm(7)
	initIOTimerChannel(io_timers, {Timer::Timer5,  Timer::Channel1}, {GPIO::PortH, GPIO::Pin10}),  // pwm(8)
	initIOTimerChannel(io_timers, {Timer::Timer1,  Timer::Channel3}, {GPIO::PortA, GPIO::Pin10}),  // pwm(9)
	initIOTimerChannel(io_timers, {Timer::Timer1,  Timer::Channel2}, {GPIO::PortA, GPIO::Pin9}),   // pwm(10)
	initIOTimerChannel(io_timers, {Timer::Timer4,  Timer::Channel3}, {GPIO::PortD, GPIO::Pin14}),  // pwm(11)
	initIOTimerChannel(io_timers, {Timer::Timer4,  Timer::Channel2}, {GPIO::PortD, GPIO::Pin13}),  // pwm(12)
	initIOTimerChannel(io_timers, {Timer::Timer4,  Timer::Channel1}, {GPIO::PortD, GPIO::Pin12}),  // pwm(13)
	initIOTimerChannel(io_timers, {Timer::Timer12, Timer::Channel2}, {GPIO::PortH, GPIO::Pin9}),   // pwm(14)
	initIOTimerChannel(io_timers, {Timer::Timer5,  Timer::Channel3}, {GPIO::PortH, GPIO::Pin12}),  // pwm(15)
	initIOTimerChannel(io_timers, {Timer::Timer12, Timer::Channel1}, {GPIO::PortH, GPIO::Pin6}),   // pwm(16)
};

constexpr io_timers_channel_mapping_t io_timers_channel_mapping =
	initIOTimerChannelMappingNonContinuous(io_timers, timer_io_channels);
