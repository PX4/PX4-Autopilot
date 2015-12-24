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
#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_tone_alarm.h>
#include <drivers/drv_led.h>
#include <drivers/drv_rgbled.h>

#include "commander_helper.h"
#include "DevMgr.hpp"

using namespace DriverFramework;

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#define BLINK_MSG_TIME	700000	// 3 fast blinks

bool is_multirotor(const struct vehicle_status_s *current_status)
{
	return ((current_status->system_type == vehicle_status_s::VEHICLE_TYPE_QUADROTOR) ||
		(current_status->system_type == vehicle_status_s::VEHICLE_TYPE_HEXAROTOR) ||
		(current_status->system_type == vehicle_status_s::VEHICLE_TYPE_OCTOROTOR) ||
		(current_status->system_type == vehicle_status_s::VEHICLE_TYPE_TRICOPTER));
}

bool is_rotary_wing(const struct vehicle_status_s *current_status)
{
	return is_multirotor(current_status) || (current_status->system_type == vehicle_status_s::VEHICLE_TYPE_HELICOPTER)
	       || (current_status->system_type == vehicle_status_s::VEHICLE_TYPE_COAXIAL);
}

bool is_vtol(const struct vehicle_status_s * current_status) {
	return (current_status->system_type == vehicle_status_s::VEHICLE_TYPE_VTOL_DUOROTOR ||
		current_status->system_type == vehicle_status_s::VEHICLE_TYPE_VTOL_QUADROTOR ||
		current_status->system_type == vehicle_status_s::VEHICLE_TYPE_VTOL_HEXAROTOR ||
		current_status->system_type == vehicle_status_s::VEHICLE_TYPE_VTOL_OCTOROTOR);
}

static hrt_abstime blink_msg_end = 0;	// end time for currently blinking LED message, 0 if no blink message
static hrt_abstime tune_end = 0;		// end time of currently played tune, 0 for repeating tunes or silence
static int tune_current = TONE_STOP_TUNE;		// currently playing tune, can be interrupted after tune_end
static unsigned int tune_durations[TONE_NUMBER_OF_TUNES];

static DevHandle h_leds;
static DevHandle h_rgbleds;
static DevHandle h_buzzer;

static param_t bat_v_empty_h;
static param_t bat_v_full_h;
static param_t bat_n_cells_h;
static param_t bat_capacity_h;
static param_t bat_v_load_drop_h;
static float bat_v_empty = 3.4f;
static float bat_v_full = 4.2f;
static float bat_v_load_drop = 0.06f;
static int bat_n_cells = 3;
static float bat_capacity = -1.0f;
static unsigned int counter = 0;
static float throttle_lowpassed = 0.0f;

int battery_init()
{
	bat_v_empty_h = param_find("BAT_V_EMPTY");
	bat_v_full_h = param_find("BAT_V_CHARGED");
	bat_n_cells_h = param_find("BAT_N_CELLS");
	bat_capacity_h = param_find("BAT_CAPACITY");
	bat_v_load_drop_h = param_find("BAT_V_LOAD_DROP");

	return PX4_OK;
}

int buzzer_init()
{
	tune_end = 0;
	tune_current = 0;
	memset(tune_durations, 0, sizeof(tune_durations));
	tune_durations[TONE_NOTIFY_POSITIVE_TUNE] = 800000;
	tune_durations[TONE_NOTIFY_NEGATIVE_TUNE] = 900000;
	tune_durations[TONE_NOTIFY_NEUTRAL_TUNE] = 500000;
	tune_durations[TONE_ARMING_WARNING_TUNE] = 3000000;

	DevMgr::getHandle(TONEALARM0_DEVICE_PATH, h_buzzer);

	if (!h_buzzer.isValid()) {
		PX4_WARN("Buzzer: px4_open fail\n");
		return ERROR;
	}

	return PX4_OK;
}

void buzzer_deinit()
{
	DevMgr::releaseHandle(h_buzzer);
}

void set_tune_override(int tune)
{
	h_buzzer.ioctl(TONE_SET_ALARM, tune);
}

void set_tune(int tune)
{
	unsigned int new_tune_duration = tune_durations[tune];

	/* don't interrupt currently playing non-repeating tune by repeating */
	if (tune_end == 0 || new_tune_duration != 0 || hrt_absolute_time() > tune_end) {
		/* allow interrupting current non-repeating tune by the same tune */
		if (tune != tune_current || new_tune_duration != 0) {
			h_buzzer.ioctl(TONE_SET_ALARM, tune);
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
	rgbled_set_color(RGBLED_COLOR_GREEN);
	rgbled_set_mode(RGBLED_MODE_BLINK_FAST);

	if (use_buzzer) {
		set_tune(TONE_HOME_SET);
	}
}

void tune_mission_ok(bool use_buzzer)
{
	blink_msg_end = hrt_absolute_time() + BLINK_MSG_TIME;
	rgbled_set_color(RGBLED_COLOR_GREEN);
	rgbled_set_mode(RGBLED_MODE_BLINK_FAST);

	if (use_buzzer) {
		set_tune(TONE_NOTIFY_NEUTRAL_TUNE);
	}
}

void tune_mission_fail(bool use_buzzer)
{
	blink_msg_end = hrt_absolute_time() + BLINK_MSG_TIME;
	rgbled_set_color(RGBLED_COLOR_GREEN);
	rgbled_set_mode(RGBLED_MODE_BLINK_FAST);

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
	rgbled_set_color(RGBLED_COLOR_GREEN);
	rgbled_set_mode(RGBLED_MODE_BLINK_FAST);

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
	rgbled_set_color(RGBLED_COLOR_WHITE);
	rgbled_set_mode(RGBLED_MODE_BLINK_FAST);

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
	rgbled_set_color(RGBLED_COLOR_RED);
	rgbled_set_mode(RGBLED_MODE_BLINK_FAST);

	if (use_buzzer) {
		set_tune(TONE_NOTIFY_NEGATIVE_TUNE);
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

	/* first open normal LEDs */
	DevMgr::getHandle(LED0_DEVICE_PATH, h_leds);

	if (!h_leds.isValid()) {
		PX4_WARN("LED: getHandle fail\n");
		return ERROR;
	}

	/* the blue LED is only available on FMUv1 & AeroCore but not FMUv2 */
	(void)h_leds.ioctl(LED_ON, LED_BLUE);

	/* switch blue off */
	led_off(LED_BLUE);

	/* we consider the amber led mandatory */
	if (h_leds.ioctl(LED_ON, LED_AMBER)) {
		PX4_WARN("Amber LED: ioctl fail\n");
		return ERROR;
	}

	/* switch amber off */
	led_off(LED_AMBER);

	/* then try RGB LEDs, this can fail on FMUv1*/
	DevHandle h;
	DevMgr::getHandle(RGBLED0_DEVICE_PATH, h_rgbleds);

	if (!h_rgbleds.isValid()) {
		PX4_WARN("No RGB LED found at " RGBLED0_DEVICE_PATH);
	}

	return 0;
}

void led_deinit()
{
	DevMgr::releaseHandle(h_leds);
	DevMgr::releaseHandle(h_rgbleds);
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

void rgbled_set_color(rgbled_color_t color)
{

	h_rgbleds.ioctl(RGBLED_SET_COLOR, (unsigned long)color);
}

void rgbled_set_mode(rgbled_mode_t mode)
{

	h_rgbleds.ioctl(RGBLED_SET_MODE, (unsigned long)mode);
}

void rgbled_set_pattern(rgbled_pattern_t *pattern)
{

	h_rgbleds.ioctl(RGBLED_SET_PATTERN, (unsigned long)pattern);
}

unsigned battery_get_n_cells() {
	return bat_n_cells;
}

float battery_remaining_estimate_voltage(float voltage, float discharged, float throttle_normalized)
{
	float ret = 0;

	if (counter % 100 == 0) {
		param_get(bat_v_empty_h, &bat_v_empty);
		param_get(bat_v_full_h, &bat_v_full);
		param_get(bat_v_load_drop_h, &bat_v_load_drop);
		param_get(bat_n_cells_h, &bat_n_cells);
		param_get(bat_capacity_h, &bat_capacity);
	}

	counter++;

	// XXX this time constant needs to become tunable
	// but really, the right fix are smart batteries.
	float val = throttle_lowpassed * 0.97f + throttle_normalized * 0.03f;
	if (PX4_ISFINITE(val)) {
		throttle_lowpassed = val;
	}

	/* remaining charge estimate based on voltage and internal resistance (drop under load) */
	float bat_v_empty_dynamic = bat_v_empty - (bat_v_load_drop * throttle_lowpassed);
	/* the range from full to empty is the same for batteries under load and without load,
	 * since the voltage drop applies to both the full and empty state
	 */
	float voltage_range = (bat_v_full - bat_v_empty);
	float remaining_voltage = (voltage - (bat_n_cells * bat_v_empty_dynamic)) / (bat_n_cells * voltage_range);

	if (bat_capacity > 0.0f) {
		/* if battery capacity is known, use discharged current for estimate, but don't show more than voltage estimate */
		ret = fminf(remaining_voltage, 1.0f - discharged / bat_capacity);

	} else {
		/* else use voltage */
		ret = remaining_voltage;
	}

	/* limit to sane values */
	ret = (ret < 0.0f) ? 0.0f : ret;
	ret = (ret > 1.0f) ? 1.0f : ret;
	return ret;
}
