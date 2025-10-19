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
    // index‑0 ─ TIM3  : DMA1 / Stream‑2 / Channel‑5  (unchanged)
    initIOTimer(Timer::Timer3,  DMA{DMA::Index1, DMA::Stream2, DMA::Channel5}),

    // index‑1 ─ TIM5  : DMA1 / Stream‑0 / Channel‑6  (unchanged)
    initIOTimer(Timer::Timer5,  DMA{DMA::Index1, DMA::Stream0, DMA::Channel6}),

    // index‑2 ─ TIM4  : DMA1 / Stream‑6 / Channel‑2  (unchanged)
    initIOTimer(Timer::Timer4,  DMA{DMA::Index1, DMA::Stream6, DMA::Channel2}),

    // index‑3 ─ TIM1  : ADVANCED timer for motors 9‑10  (replaces TIM8)
    //                 Uses DMA1 with DMAMUX‑assigned request; no fixed stream needed.
    initIOTimer(Timer::Timer1,  DMA{DMA::Index1}),

    // index‑4 ─ TIM15 : Aux 11‑12  (unchanged; software PWM OK if DMA not required)
    // initIOTimer(Timer::Timer2),

    // index‑4 ─ TIM15 : Aux 11‑12  (unchanged; software PWM OK if DMA not required)
    initIOTimer(Timer::Timer15),

    // index‑5 ─ TIM16 : future / capture  (unchanged)
    // initIOTimer(Timer::Timer16),
};


constexpr timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {

    // ─── TIM5 (32‑bit, DMA‑burst) : Motors 1‑4 ────────────────────────────────
    initIOTimerChannel(io_timers, {Timer::Timer5, Timer::Channel1}, {GPIO::PortA, GPIO::Pin0}),
    initIOTimerChannel(io_timers, {Timer::Timer5, Timer::Channel2}, {GPIO::PortA, GPIO::Pin1}),
    initIOTimerChannel(io_timers, {Timer::Timer5, Timer::Channel3}, {GPIO::PortA, GPIO::Pin2}),
    initIOTimerChannel(io_timers, {Timer::Timer5, Timer::Channel4}, {GPIO::PortA, GPIO::Pin3}),

    // ─── TIM4 (DMA‑capable GP) : Motors 5‑8 ───────────────────────────────────
    initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel1}, {GPIO::PortD, GPIO::Pin12}),
    initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel2}, {GPIO::PortD, GPIO::Pin13}),
    initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel3}, {GPIO::PortD, GPIO::Pin14}),
    initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel4}, {GPIO::PortD, GPIO::Pin15}),

    // ─── TIM1 (advanced) : Motors 9‑10 ────────────────────────────────────────
    initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel2}, {GPIO::PortE, GPIO::Pin11}),
    initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel3}, {GPIO::PortA, GPIO::Pin10}),

    // ─── TIM15 : Aux 11‑12 ────────────────────────────────────────────────────
    initIOTimerChannel(io_timers, {Timer::Timer15, Timer::Channel1}, {GPIO::PortE, GPIO::Pin5}),
    initIOTimerChannel(io_timers, {Timer::Timer15, Timer::Channel2}, {GPIO::PortE, GPIO::Pin6}),

    // ─── TIM3 : Aux / Capture 13‑16 ───────────────────────────────────────────
    initIOTimerChannel(io_timers, {Timer::Timer3, Timer::Channel1}, {GPIO::PortC, GPIO::Pin6}),
    initIOTimerChannel(io_timers, {Timer::Timer3, Timer::Channel2}, {GPIO::PortC, GPIO::Pin7}),
    initIOTimerChannel(io_timers, {Timer::Timer3, Timer::Channel3}, {GPIO::PortC, GPIO::Pin8}),
    initIOTimerChannel(io_timers, {Timer::Timer3, Timer::Channel4}, {GPIO::PortC, GPIO::Pin9}),
};

constexpr io_timers_channel_mapping_t io_timers_channel_mapping =
	initIOTimerChannelMapping(io_timers, timer_io_channels);

// constexpr io_timers_t led_pwm_timers[MAX_LED_TIMERS] = {
// 	initIOTimer(Timer::Timer1),
// };

// #define CCER_C1_NUM_BITS   4
// #define POLARITY(c)    (GTIM_CCER_CC1P << (((c)-1) * CCER_C1_NUM_BITS))
// #define DRIVE_TYPE(p)  ((p)|GPIO_OPENDRAIN|GPIO_PULLUP)

// static inline constexpr timer_io_channels_t initIOTimerChannelLED(const io_timers_t io_timers_conf[MAX_LED_TIMERS],
// 		Timer::TimerChannel timer, GPIO::GPIOPin pin, int ui_polarity)
// {
// 	timer_io_channels_t ret = initIOTimerChannel(io_timers_conf, timer, pin);
// 	ret.gpio_out = DRIVE_TYPE(ret.gpio_out);
// 	ret.masks = POLARITY(ui_polarity);
// 	return ret;
// }

// constexpr timer_io_channels_t led_pwm_channels[MAX_TIMER_LED_CHANNELS] = {
// 	initIOTimerChannelLED(led_pwm_timers, {Timer::Timer1, Timer::Channel1}, {GPIO::PortA, GPIO::Pin8}, 1),
// };
