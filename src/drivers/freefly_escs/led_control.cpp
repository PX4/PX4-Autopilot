/****************************************************************************
 *
 *   Copyright (c) 2020-2023 PX4 Development Team. All rights reserved.
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

#include "led_control.h"

#ifdef CONFIG_ARCH_CHIP_STM32
#include "stm32.h"
#endif
#include "stm32_can.h"

#include <px4_platform_common/events.h>

LedControl::LedControl(ModuleParams *parent) :
	ModuleParams(parent)
{}

void
LedControl::updateLeds(bool motors_armed, uint8_t online_esc_mask)
{
	updateSubscriptions();

	if (!motors_armed) {
		// Disarmed
		setLedsToParameterColors(0.34f * _param_boom_brt.get());
		// Reset sticky flags
		_battery_low = _battery_fault = _mag_unhealthy = _failure_detector_failure = _arming_state_high = false;

	} else if (_kill_switch_engaged
		   || _battery_fault
		   || _mag_unhealthy
		   || _failure_detector_failure
		   || _arming_state_high) {
		// Kill switch (non-sticky) or fault (sticky until disarm)
		blinkLeds(LED_BLINK_SPEED_FAST);

	} else if (_battery_low) {
		// Battery Low
		blinkLeds(LED_BLINK_SPEED_SLOW);

	} else {
		// Armed
		setLedsToParameterColors(_param_boom_brt.get());
	}

	// Send RGB values to LEDs if they are not yet up to date and ESC is online
	for (int index = 0; index < MOTOR_COUNT; index++) {
		uint8_t index_mask = 1 << index;

		if ((index_mask & ~_led_color_sent_mask & online_esc_mask) != 0) {
			// LED not up to date, send out new RGB values
			sendLedRgbCanMessage(index, _led_rgb_state[index][0], _led_rgb_state[index][1], _led_rgb_state[index][2]);
			_led_color_sent_mask |= index_mask;
			break;
		}
	}

	_rgb_counter++;
}

void LedControl::updateSubscriptions()
{
	if (_battery_status_subs.updated()) {
		for (auto &battery_sub : _battery_status_subs) {
			battery_status_s battery;

			if (!battery_sub.copy(&battery)) {
				continue;
			}

			_battery_fault |= (battery.faults > 0) || !battery.connected;
			_battery_low |= battery.warning > battery_status_s::WARNING_NONE;
		}
	}

	actuator_armed_s actuator_armed{};

	if (_actuator_armed_sub.update(&actuator_armed)) {
		_kill_switch_engaged = actuator_armed.kill;
	}

	health_report_s health_report{};

	if (_health_report_sub.update(&health_report)) {
		_mag_unhealthy |=
			(((health_report.arming_check_error_flags
			   | health_report.arming_check_warning_flags
			   | health_report.health_error_flags
			   | health_report.health_warning_flags)
			  & (uint64_t)events::px4::enums::health_component_t::magnetometer)
			 > 0);
	}

	vehicle_status_s vehicle_status{};

	if (_vehicle_status_sub.update(&vehicle_status)) {
		_arming_state_high |= vehicle_status.arming_state > vehicle_status_s::ARMING_STATE_ARMED;
	}

	failure_detector_status_s failure_detector_status{};

	if (_failure_detector_status_sub.update(&failure_detector_status)) {
		_failure_detector_failure |= failure_detector_status.fd_roll;
		_failure_detector_failure |= failure_detector_status.fd_pitch;
		_failure_detector_failure |= failure_detector_status.fd_alt;
		_failure_detector_failure |= failure_detector_status.fd_ext;
		_failure_detector_failure |= failure_detector_status.fd_arm_escs;
		_failure_detector_failure |= failure_detector_status.fd_battery;
		_failure_detector_failure |= failure_detector_status.fd_imbalanced_prop;
		_failure_detector_failure |= failure_detector_status.fd_motor;
	}
}

void LedControl::updateParams()
{
	// update parameters from storage
	ModuleParams::updateParams();
	_led_colors_from_parameters[0] = static_cast<BoomColor>(_param_boom_color_1.get());
	_led_colors_from_parameters[1] = static_cast<BoomColor>(_param_boom_color_2.get());
	_led_colors_from_parameters[2] = static_cast<BoomColor>(_param_boom_color_3.get());
	_led_colors_from_parameters[3] = static_cast<BoomColor>(_param_boom_color_4.get());
}

void
LedControl::blinkLeds(hrt_abstime blink_timespan)
{
	if ((hrt_absolute_time() - _last_blink_time) > blink_timespan) {
		if (_blink_state_on) {
			_blink_state_on = false;
			setLedsOff();

		} else {
			_blink_state_on = true;
			setLedsToParameterColors(_param_boom_brt.get());
		}

		_last_blink_time = hrt_absolute_time();
	}
}

void LedControl::updateRgbWheel(uint8_t esc_id)
{
	if (_rgb_counter % MOTOR_COUNT == esc_id) {
		if (_rgb_counter <= 255) {
			setLedRgb(esc_id, _rgb_counter, 0, 255 - _rgb_counter);

		} else if (_rgb_counter <= 510) {
			setLedRgb(esc_id, 255 - (_rgb_counter - 255), (_rgb_counter - 255), 0);

		} else if (_rgb_counter <= 765) {
			setLedRgb(esc_id, 0, 255 - (_rgb_counter - 510), (_rgb_counter - 510));

		} else {
			_rgb_counter = 0;
		}
	}
}

void LedControl::setLedsToParameterColors(float brightness)
{
	for (int i = 0; i < MOTOR_COUNT; i++) {
		setLedColor(i, _led_colors_from_parameters[i], brightness);
	}
}

void LedControl::setLedsToColor(BoomColor color, float brightness)
{
	for (int i = 0; i < MOTOR_COUNT; i++) {
		setLedColor(i, color, brightness);
	}
}

void LedControl::setLedsOff()
{
	for (int i = 0; i < MOTOR_COUNT; i++) {
		setLedRgb(i, 0, 0, 0);
	}
}

void LedControl::setLedColor(uint8_t esc_id, BoomColor color, float brightness)
{
	brightness = math::constrain(brightness, 0.f, 1.f);
	brightness *= .75f; // Dimming down is required because of thermal limitations of the Astro RGB LEDs
	uint8_t R = 0, B = 0, G = 0;

	switch (color) {
	case BoomColor::Off:
		break;

	case BoomColor::Red:
		R = 255;
		break;

	case BoomColor::Orange:
		R = 255;
		G = 128;
		break;

	case BoomColor::Yellow:
		R = 255;
		G = 255;
		break;

	case BoomColor::Green:
		G = 255;
		break;

	case BoomColor::Cyan:
		G = 255;
		B = 255;
		break;

	case BoomColor::Blue:
		B = 255;
		break;

	case BoomColor::Purple:
		R = 255;
		B = 255;
		break;

	case BoomColor::White:
		R = 255;
		G = 255;
		B = 255;
		break;

	case BoomColor::RgbWheel:
		updateRgbWheel(esc_id);
		return;

	default:
		R = 100; G = 100; B = 100; break;
	}

	setLedRgb(esc_id, R * brightness, G * brightness, B * brightness);
}

void LedControl::setLedRgb(uint8_t esc_id, uint8_t red, uint8_t green, uint8_t blue)
{
	// Only plan an update message to the LED if the RGB values changed
	if (esc_id < MOTOR_COUNT
	    && (red != _led_rgb_state[esc_id][0]
		|| green != _led_rgb_state[esc_id][1]
		|| blue != _led_rgb_state[esc_id][2])) {
		_led_rgb_state[esc_id][0] = red;
		_led_rgb_state[esc_id][1] = green;
		_led_rgb_state[esc_id][2] = blue;
		_led_color_sent_mask &= ~(1 << esc_id);
	}
}

void LedControl::sendLedRgbCanMessage(uint8_t esc_id, uint8_t red, uint8_t green, uint8_t blue)
{
	static constexpr uint8_t lower_limit = 0U;
	static constexpr uint8_t upper_limit = 255U;
	red = math::constrain(red, lower_limit, upper_limit);
	green = math::constrain(green, lower_limit, upper_limit);
	blue = math::constrain(blue, lower_limit, upper_limit);

	// Setup message
	struct can_msg_s msg;
	msg.cm_hdr.ch_id = 0x38; // Message 38 for color
	msg.cm_hdr.ch_rtr = 0;
	msg.cm_hdr.ch_dlc = 4;

	// set msg payload to command
	msg.cm_data[0] = esc_id + 1; // in the message the index is 1 based
	msg.cm_data[1] = red;
	msg.cm_data[2] = green;
	msg.cm_data[3] = blue;

	// send the message and wait until Tx complete
	send_can_msg(msg);
}

int LedControl::send_can_msg(const struct can_msg_s &msg_p)
{
	// Variables for controlling CAN transmission
	size_t msgsize;
	ssize_t bytes_written;

	// CAN_MSGLEN is a convenience macro.
	// It gets the data length of the message and computes the actual size of the CAN message.
	msgsize = CAN_MSGLEN(msg_p.cm_hdr.ch_dlc);

	// Do the write operation
	bytes_written = write(_file_descriptor, (uint8_t *)&msg_p, msgsize);

	// Check if message send is success.
	if ((size_t)bytes_written != msgsize || bytes_written < 0) {
		PX4_DEBUG("CAN Write Error: write(%d) returned %d", msgsize, bytes_written);
		return ERROR;

	} else {
		// Write success.
		return OK;
	}

	// Shouldn't reach here.
	return -2;
}
