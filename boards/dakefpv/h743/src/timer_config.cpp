/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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
        initIOTimer(Timer::Timer2,  DMA{DMA::Index1}),  // M1-M4
        initIOTimer(Timer::Timer4,  DMA{DMA::Index1}),  // M5-M8
        initIOTimer(Timer::Timer15),                     // S1-S2
        initIOTimer(Timer::Timer8,  DMA{DMA::Index1}),  // S3-S4 (HRT is TIM5, so TIM8 is free)
};

constexpr timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
        initIOTimerChannel(io_timers, {Timer::Timer2,  Timer::Channel1}, {GPIO::PortA, GPIO::Pin0}),  // M1
        initIOTimerChannel(io_timers, {Timer::Timer2,  Timer::Channel2}, {GPIO::PortA, GPIO::Pin1}),  // M2
        initIOTimerChannel(io_timers, {Timer::Timer2,  Timer::Channel3}, {GPIO::PortA, GPIO::Pin2}),  // M3
        initIOTimerChannel(io_timers, {Timer::Timer2,  Timer::Channel4}, {GPIO::PortA, GPIO::Pin3}),  // M4

        initIOTimerChannel(io_timers, {Timer::Timer4,  Timer::Channel1}, {GPIO::PortD, GPIO::Pin12}), // M5
        initIOTimerChannel(io_timers, {Timer::Timer4,  Timer::Channel2}, {GPIO::PortD, GPIO::Pin13}), // M6
        initIOTimerChannel(io_timers, {Timer::Timer4,  Timer::Channel3}, {GPIO::PortD, GPIO::Pin14}), // M7
        initIOTimerChannel(io_timers, {Timer::Timer4,  Timer::Channel4}, {GPIO::PortD, GPIO::Pin15}), // M8

        initIOTimerChannel(io_timers, {Timer::Timer15, Timer::Channel1}, {GPIO::PortE, GPIO::Pin5}),  // S1
        initIOTimerChannel(io_timers, {Timer::Timer15, Timer::Channel2}, {GPIO::PortE, GPIO::Pin6}),  // S2

        initIOTimerChannel(io_timers, {Timer::Timer8,  Timer::Channel3}, {GPIO::PortC, GPIO::Pin8}),  // S3
        initIOTimerChannel(io_timers, {Timer::Timer8,  Timer::Channel4}, {GPIO::PortC, GPIO::Pin9}),  // S4
};

constexpr io_timers_channel_mapping_t io_timers_channel_mapping = initIOTimerChannelMapping(io_timers, timer_io_channels);

constexpr io_timers_t led_pwm_timers[MAX_LED_TIMERS] = {
        initIOTimer(Timer::Timer1),  // PE9 LED strip pad (TIM1_CH1)
};
