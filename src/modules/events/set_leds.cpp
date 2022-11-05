/****************************************************************************
 *
 *   Copyright (c) 2017-2021 PX4 Development Team. All rights reserved.
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
 * @file set_leds.cpp
 * Separate the set_leds() function
 *
 * @author Christoph Tobler <christoph@px4.io>
 */

#include "status_display.h"

#include <board_config.h>
#include <px4_log.h>
#include <matrix/math.hpp>
#include <drivers/drv_led.h>

using namespace time_literals;

namespace events
{
namespace status
{

void StatusDisplay::set_leds()
{
	bool gps_lock_valid = !_failsafe_flags_sub.get().global_position_invalid;
	bool home_position_valid = !_failsafe_flags_sub.get().home_position_invalid;
	int nav_state = _vehicle_status_sub.get().nav_state;

#if defined(BOARD_FRONT_LED_MASK)

	// try to publish the static LED for the first 10s
	// this avoid the problem if a LED driver did not subscribe to the topic yet
	if (hrt_absolute_time() < 10_s) {

		// set the base color for front LED
		_led_control.led_mask = BOARD_FRONT_LED_MASK;
		_led_control.color = led_control_s::COLOR_WHITE;
		_led_control.mode = led_control_s::MODE_ON;

		publish();
	}

#endif // BOARD_FRONT_LED_MASK

#if defined(BOARD_REAR_LED_MASK)
	// set the led mask for the status led which are the back LED
	_led_control.led_mask = BOARD_REAR_LED_MASK;

	if (nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL
	    || nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND) {
		_led_control.color = led_control_s::COLOR_PURPLE;

	} else if (nav_state == vehicle_status_s::NAVIGATION_STATE_ALTCTL) {
		_led_control.color = led_control_s::COLOR_BLUE;

	} else if (nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION) {
		_led_control.color = led_control_s::COLOR_GREEN;

	} else {
		_led_control.color = led_control_s::COLOR_YELLOW;   // TODO fix yellow and purple error
	}

	// blink if no GPS and home are set
	if (gps_lock_valid && home_position_valid) {
		_led_control.mode = led_control_s::MODE_ON;

	} else {
		_led_control.mode = led_control_s::MODE_BLINK_NORMAL;
	}

	// handle battery warnings, once a state is reached it can not be reset
	if (_battery_status_sub.get().warning == battery_status_s::BATTERY_WARNING_CRITICAL || _critical_battery) {
		_led_control.color = led_control_s::COLOR_RED;
		_led_control.mode = led_control_s::MODE_BLINK_FAST;
		_critical_battery = true;

	} else if (_battery_status_sub.get().warning == battery_status_s::BATTERY_WARNING_LOW || _low_battery) {
		_led_control.color = led_control_s::COLOR_RED;
		_led_control.mode = led_control_s::MODE_FLASH;
		_low_battery = true;
	}

	if (nav_state != _old_nav_state
	    || gps_lock_valid != _old_gps_lock_valid
	    || home_position_valid != _old_home_position_valid
	    || _battery_status_sub.get().warning != _old_battery_status_warning) {

		publish();
	}

#endif // BOARD_REAR_LED_MASK

	// copy actual state
	_old_nav_state = nav_state;
	_old_gps_lock_valid = gps_lock_valid;
	_old_home_position_valid = home_position_valid;
	_old_battery_status_warning = _battery_status_sub.get().warning;
}

} /* namespace status */
} /* namespace events */
