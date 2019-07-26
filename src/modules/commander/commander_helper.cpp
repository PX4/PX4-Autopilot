/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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

/**
 * @file commander_helper.cpp
 * Commander helper functions implementations
 *
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 *
 */

#include <px4_defines.h>
#include <px4_posix.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <fcntl.h>
#include <math.h>
#include <string.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/led_control.h>
#include <uORB/topics/tune_control.h>
#include <systemlib/err.h>
#include <parameters/param.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_tone_alarm.h>

#include "commander_helper.h"
#include "DevMgr.hpp"

using namespace DriverFramework;

#define VEHICLE_TYPE_FIXED_WING 1
#define VEHICLE_TYPE_QUADROTOR 2
#define VEHICLE_TYPE_COAXIAL 3
#define VEHICLE_TYPE_HELICOPTER 4
#define VEHICLE_TYPE_GROUND_ROVER 10
#define VEHICLE_TYPE_HEXAROTOR 13
#define VEHICLE_TYPE_OCTOROTOR 14
#define VEHICLE_TYPE_TRICOPTER 15
#define VEHICLE_TYPE_VTOL_DUOROTOR 19
#define VEHICLE_TYPE_VTOL_QUADROTOR 20
#define VEHICLE_TYPE_VTOL_TILTROTOR 21
#define VEHICLE_TYPE_VTOL_RESERVED2 22
#define VEHICLE_TYPE_VTOL_RESERVED3 23
#define VEHICLE_TYPE_VTOL_RESERVED4 24
#define VEHICLE_TYPE_VTOL_RESERVED5 25

#define BLINK_MSG_TIME	700000	// 3 fast blinks (in us)

bool is_multirotor(const struct vehicle_status_s *current_status)
{
	return ((current_status->system_type == VEHICLE_TYPE_QUADROTOR) ||
		(current_status->system_type == VEHICLE_TYPE_HEXAROTOR) ||
		(current_status->system_type == VEHICLE_TYPE_OCTOROTOR) ||
		(current_status->system_type == VEHICLE_TYPE_TRICOPTER));
}

bool is_rotary_wing(const struct vehicle_status_s *current_status)
{
	return is_multirotor(current_status) || (current_status->system_type == VEHICLE_TYPE_HELICOPTER)
	       || (current_status->system_type == VEHICLE_TYPE_COAXIAL);
}

bool is_vtol(const struct vehicle_status_s *current_status)
{
	return (current_status->system_type == VEHICLE_TYPE_VTOL_DUOROTOR ||
		current_status->system_type == VEHICLE_TYPE_VTOL_QUADROTOR ||
		current_status->system_type == VEHICLE_TYPE_VTOL_TILTROTOR ||
		current_status->system_type == VEHICLE_TYPE_VTOL_RESERVED2 ||
		current_status->system_type == VEHICLE_TYPE_VTOL_RESERVED3 ||
		current_status->system_type == VEHICLE_TYPE_VTOL_RESERVED4 ||
		current_status->system_type == VEHICLE_TYPE_VTOL_RESERVED5);
}

bool is_fixed_wing(const struct vehicle_status_s *current_status)
{
	return current_status->system_type == VEHICLE_TYPE_FIXED_WING;
}

bool is_ground_rover(const struct vehicle_status_s *current_status)
{
	return current_status->system_type == VEHICLE_TYPE_GROUND_ROVER;
}

static hrt_abstime blink_msg_end = 0;	// end time for currently blinking LED message, 0 if no blink message
static hrt_abstime tune_end = 0;		// end time of currently played tune, 0 for repeating tunes or silence
static int tune_current = TONE_STOP_TUNE;		// currently playing tune, can be interrupted after tune_end
static unsigned int tune_durations[TONE_NUMBER_OF_TUNES];

static DevHandle h_leds;
static DevHandle h_buzzer;
static led_control_s led_control = {};
static orb_advert_t led_control_pub = nullptr;
static tune_control_s tune_control = {};
static orb_advert_t tune_control_pub = nullptr;

int buzzer_init()
{
	tune_end = 0;
	tune_current = 0;
	memset(tune_durations, 0, sizeof(tune_durations));
	tune_durations[TONE_NOTIFY_POSITIVE_TUNE] = 800000;
	tune_durations[TONE_NOTIFY_NEGATIVE_TUNE] = 900000;
	tune_durations[TONE_NOTIFY_NEUTRAL_TUNE] = 500000;
	tune_durations[TONE_ARMING_WARNING_TUNE] = 3000000;
	tune_durations[TONE_HOME_SET] = 800000;
	tune_durations[TONE_BATTERY_WARNING_FAST_TUNE] = 800000;
	tune_durations[TONE_BATTERY_WARNING_SLOW_TUNE] = 800000;
	tune_durations[TONE_SINGLE_BEEP_TUNE] = 300000;
	tune_control_pub = orb_advertise_queue(ORB_ID(tune_control), &tune_control, tune_control_s::ORB_QUEUE_LENGTH);
	return PX4_OK;
}

void buzzer_deinit()
{
	orb_unadvertise(tune_control_pub);
}

void set_tune_override(int tune)
{
	tune_control.tune_id = tune;
	tune_control.volume = tune_control_s::VOLUME_LEVEL_DEFAULT;
	tune_control.tune_override = 1;
	tune_control.timestamp = hrt_absolute_time();
	orb_publish(ORB_ID(tune_control), tune_control_pub, &tune_control);
}

void set_tune(int tune)
{
	unsigned int new_tune_duration = tune_durations[tune];

	/* don't interrupt currently playing non-repeating tune by repeating */
	if (tune_end == 0 || new_tune_duration != 0 || hrt_absolute_time() > tune_end) {
		/* allow interrupting current non-repeating tune by the same tune */
		if (tune != tune_current || new_tune_duration != 0) {
			tune_control.tune_id = tune;
			tune_control.volume = tune_control_s::VOLUME_LEVEL_DEFAULT;
			tune_control.tune_override = 0;
			tune_control.timestamp = hrt_absolute_time();
			orb_publish(ORB_ID(tune_control), tune_control_pub, &tune_control);
		}

		tune_current = tune;

		if (new_tune_duration != 0) {
			tune_end = hrt_absolute_time() + new_tune_duration;

		} else {
			tune_end = 0;
		}
	}
}

void tune_home_set(bool use_buzzer)
{
	blink_msg_end = hrt_absolute_time() + BLINK_MSG_TIME;
	rgbled_set_color_and_mode(led_control_s::COLOR_GREEN, led_control_s::MODE_BLINK_FAST);

	if (use_buzzer) {
		set_tune(TONE_HOME_SET);
	}
}

void tune_mission_ok(bool use_buzzer)
{
	blink_msg_end = hrt_absolute_time() + BLINK_MSG_TIME;
	rgbled_set_color_and_mode(led_control_s::COLOR_GREEN, led_control_s::MODE_BLINK_FAST);

	if (use_buzzer) {
		set_tune(TONE_NOTIFY_NEUTRAL_TUNE);
	}
}

void tune_mission_fail(bool use_buzzer)
{
	blink_msg_end = hrt_absolute_time() + BLINK_MSG_TIME;
	rgbled_set_color_and_mode(led_control_s::COLOR_GREEN, led_control_s::MODE_BLINK_FAST);

	if (use_buzzer) {
		set_tune(TONE_NOTIFY_NEGATIVE_TUNE);
	}
}

/**
 * Blink green LED and play positive tune (if use_buzzer == true).
 */
void tune_positive(bool use_buzzer)
{
	blink_msg_end = hrt_absolute_time() + BLINK_MSG_TIME;
	rgbled_set_color_and_mode(led_control_s::COLOR_GREEN, led_control_s::MODE_BLINK_FAST);

	if (use_buzzer) {
		set_tune(TONE_NOTIFY_POSITIVE_TUNE);
	}
}

/**
 * Blink white LED and play neutral tune (if use_buzzer == true).
 */
void tune_neutral(bool use_buzzer)
{
	blink_msg_end = hrt_absolute_time() + BLINK_MSG_TIME;
	rgbled_set_color_and_mode(led_control_s::COLOR_WHITE, led_control_s::MODE_BLINK_FAST);

	if (use_buzzer) {
		set_tune(TONE_NOTIFY_NEUTRAL_TUNE);
	}
}

/**
 * Blink red LED and play negative tune (if use_buzzer == true).
 */
void tune_negative(bool use_buzzer)
{
	blink_msg_end = hrt_absolute_time() + BLINK_MSG_TIME;
	rgbled_set_color_and_mode(led_control_s::COLOR_RED, led_control_s::MODE_BLINK_FAST);

	if (use_buzzer) {
		set_tune(TONE_NOTIFY_NEGATIVE_TUNE);
	}
}

void tune_failsafe(bool use_buzzer)
{
	blink_msg_end = hrt_absolute_time() + BLINK_MSG_TIME;
	rgbled_set_color_and_mode(led_control_s::COLOR_PURPLE, led_control_s::MODE_BLINK_FAST);

	if (use_buzzer) {
		set_tune(TONE_BATTERY_WARNING_FAST_TUNE);
	}
}

int blink_msg_state()
{
	if (blink_msg_end == 0) {
		return 0;

	} else if (hrt_absolute_time() > blink_msg_end) {
		blink_msg_end = 0;
		return 2;

	} else {
		return 1;
	}
}

int led_init()
{
	blink_msg_end = 0;

	led_control.led_mask = 0xff;
	led_control.mode = led_control_s::MODE_OFF;
	led_control.priority = 0;
	led_control.timestamp = hrt_absolute_time();
	led_control_pub = orb_advertise_queue(ORB_ID(led_control), &led_control, LED_UORB_QUEUE_LENGTH);

	/* first open normal LEDs */
	DevMgr::getHandle(LED0_DEVICE_PATH, h_leds);

	if (!h_leds.isValid()) {
		PX4_WARN("LED: getHandle fail\n");
		return PX4_ERROR;
	}

	/* the green LED is only available on FMUv5 */
	(void)h_leds.ioctl(LED_ON, LED_GREEN);

	/* the blue LED is only available on AeroCore but not FMUv2 */
	(void)h_leds.ioctl(LED_ON, LED_BLUE);

	/* switch blue off */
	led_off(LED_BLUE);

	/* we consider the amber led mandatory */
	if (h_leds.ioctl(LED_ON, LED_AMBER)) {
		PX4_WARN("Amber LED: ioctl fail\n");
		return PX4_ERROR;
	}

	/* switch amber off */
	led_off(LED_AMBER);

	return 0;
}

void led_deinit()
{
	orb_unadvertise(led_control_pub);
	DevMgr::releaseHandle(h_leds);
}

int led_toggle(int led)
{
	return h_leds.ioctl(LED_TOGGLE, led);
}

int led_on(int led)
{
	return h_leds.ioctl(LED_ON, led);
}

int led_off(int led)
{
	return h_leds.ioctl(LED_OFF, led);
}

void rgbled_set_color_and_mode(uint8_t color, uint8_t mode, uint8_t blinks, uint8_t prio)
{
	led_control.mode = mode;
	led_control.color = color;
	led_control.num_blinks = blinks;
	led_control.priority = prio;
	led_control.timestamp = hrt_absolute_time();
	orb_publish(ORB_ID(led_control), led_control_pub, &led_control);
}

void rgbled_set_color_and_mode(uint8_t color, uint8_t mode)
{
	rgbled_set_color_and_mode(color, mode, 0, 0);
}
