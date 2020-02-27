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



constexpr io_timers_t io_timers[MAX_IO_TIMERS] = {
	initIOTimer(Timer::Timer5, DMA{DMA::Index1}),
	initIOTimer(Timer::Timer4, DMA{DMA::Index1}),
	//initIOTimer(Timer::Timer8),
	initIOTimer(Timer::Timer1),
	initIOTimer(Timer::Timer12),
};

constexpr timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
	initIOTimerChannel(io_timers, {Timer::Timer5, Timer::Channel4}, {GPIO::PortI, GPIO::Pin0}),
	initIOTimerChannel(io_timers, {Timer::Timer5, Timer::Channel3}, {GPIO::PortH, GPIO::Pin12}),
	initIOTimerChannel(io_timers, {Timer::Timer5, Timer::Channel2}, {GPIO::PortH, GPIO::Pin11}),
	initIOTimerChannel(io_timers, {Timer::Timer5, Timer::Channel1}, {GPIO::PortH, GPIO::Pin10}),

	initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel4}, {GPIO::PortD, GPIO::Pin15}),
	initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel3}, {GPIO::PortD, GPIO::Pin14}),
	initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel2}, {GPIO::PortD, GPIO::Pin13}),
	initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel1}, {GPIO::PortD, GPIO::Pin12}),

	// initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel4}, {GPIO::PortE, GPIO::Pin14}),
	// initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel3}, {GPIO::PortA, GPIO::Pin10}),
	// initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel2}, {GPIO::PortE, GPIO::Pin11}),
	// initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel1}, {GPIO::PortE, GPIO::Pin9}),

	// initIOTimerChannel(io_timers, {Timer::Timer12, Timer::Channel2}, {GPIO::PortH, GPIO::Pin9}),
	// initIOTimerChannel(io_timers, {Timer::Timer12, Timer::Channel1}, {GPIO::PortH, GPIO::Pin6}),

};

constexpr io_timers_channel_mapping_t io_timers_channel_mapping =
	initIOTimerChannelMapping(io_timers, timer_io_channels);

#if defined(BOARD_HAS_LED_PWM)
constexpr io_timers_t led_pwm_timers[MAX_LED_TIMERS] = {
#  if defined(BOARD_HAS_LED_PWM)
	initIOTimer(Timer::Timer8),
#  endif
};

/* Support driving active low (preferred) or active high LED
 * on both the onboard status LEDs or the [n]UI_LED_<color>[_EXTERNAL]
 *
 * Use open drain to drive the LED. This will ensure that
 * if the LED has a 5 Volt supply that the LED will be
 * off when high.
 */
#define CCER_C1_NUM_BITS   4
#define ACTIVE_LOW(c)      (GTIM_CCER_CC1P << (((c)-1) * CCER_C1_NUM_BITS))
#define ACTIVE_HIGH(c)     0

#if defined(BOARD_LED_PWM_DRIVE_ACTIVE_LOW)
#  define POLARITY(c)      ACTIVE_LOW(c)
#  define DRIVE_TYPE(p)    ((p)|GPIO_OPENDRAIN|GPIO_PULLUP)
#else
#  define POLARITY(c)      ACTIVE_HIGH((c))
#  define DRIVE_TYPE(p)    ((p)|GPIO_OPENDRAIN|GPIO_PULLUP)
#endif

#if defined(BOARD_UI_LED_PWM_DRIVE_ACTIVE_LOW)
#  define UI_POLARITY(c)    ACTIVE_LOW(c)
#  define UI_DRIVE_TYPE(p)  ((p)|GPIO_OPENDRAIN|GPIO_PULLUP)
#else
#  define UI_POLARITY(c)    ACTIVE_HIGH((c))
#  define UI_DRIVE_TYPE(p)  ((p)|GPIO_OPENDRAIN|GPIO_PULLUP)
#endif

static inline constexpr timer_io_channels_t initIOTimerChannelUILED(const io_timers_t io_timers_conf[MAX_LED_TIMERS],
		Timer::TimerChannel timer, GPIO::GPIOPin pin, int ui_polarity)
{
	timer_io_channels_t ret = initIOTimerChannel(io_timers_conf, timer, pin);
	ret.gpio_out = UI_DRIVE_TYPE(ret.gpio_out);
	ret.masks = UI_POLARITY(ui_polarity);
	return ret;
}

static inline constexpr timer_io_channels_t initIOTimerChannelControlLED(const io_timers_t
		io_timers_conf[MAX_LED_TIMERS],
		Timer::TimerChannel timer, GPIO::GPIOPin pin, int polarity)
{
	timer_io_channels_t ret = initIOTimerChannel(io_timers_conf, timer, pin);
	ret.gpio_out = DRIVE_TYPE(ret.gpio_out);
	ret.masks = POLARITY(polarity);
	return ret;
}

constexpr timer_io_channels_t led_pwm_channels[MAX_TIMER_LED_CHANNELS] = {
#  if defined(BOARD_HAS_LED_PWM)

	initIOTimerChannelUILED(led_pwm_timers, {Timer::Timer8, Timer::Channel3}, {GPIO::PortI, GPIO::Pin7}, 3),
	initIOTimerChannelUILED(led_pwm_timers, {Timer::Timer8, Timer::Channel2}, {GPIO::PortI, GPIO::Pin6}, 2),
	initIOTimerChannelUILED(led_pwm_timers, {Timer::Timer8, Timer::Channel1}, {GPIO::PortI, GPIO::Pin5}, 1),

#  endif
};
#endif // BOARD_HAS_LED_PWM || BOARD_HAS_UI_LED_PWM
