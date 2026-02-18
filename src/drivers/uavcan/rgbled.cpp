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

#include "rgbled.hpp"
#include <lib/mathlib/mathlib.h>

UavcanRGBController::UavcanRGBController(uavcan::INode &node) :
	ModuleParams(nullptr),
	_node(node),
	_uavcan_pub_lights_cmd(node),
	_timer(node)
{
	_uavcan_pub_lights_cmd.setPriority(uavcan::TransferPriority::Lowest);
}

int UavcanRGBController::init()
{
	// Cache number of lights (0 disables the feature)
	_num_lights = math::min(static_cast<uint8_t>(_param_uavcan_lgt_num.get()), MAX_NUM_UAVCAN_LIGHTS);

	if (_num_lights == 0) {
		return 0; // Disabled, don't start timer
	}

	// Cache parameter handles and values for each light
	for (uint8_t i = 0; i < _num_lights; i++) {
		char param_name[17];

		// Light ID parameter
		snprintf(param_name, sizeof(param_name), "UAVCAN_LGT_ID%u", i);
		_light_id_params[i] = param_find(param_name);

		if (_light_id_params[i] != PARAM_INVALID) {
			int32_t light_id = 0;
			param_get(_light_id_params[i], &light_id);
			_light_ids[i] = static_cast<uint8_t>(light_id);
		}

		// Light function parameter
		snprintf(param_name, sizeof(param_name), "UAVCAN_LGT_FN%u", i);
		_light_fn_params[i] = param_find(param_name);

		if (_light_fn_params[i] != PARAM_INVALID) {
			int32_t light_fn = 0;
			param_get(_light_fn_params[i], &light_fn);
			_light_functions[i] = static_cast<LightFunction>(light_fn);
		}
	}

	// Setup timer and call back function for periodic updates
	_timer.setCallback(TimerCbBinder(this, &UavcanRGBController::periodic_update));
	_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(1000 / MAX_RATE_HZ));
	return 0;
}

void UavcanRGBController::periodic_update(const uavcan::TimerEvent &)
{
	// Early return if disabled or no lights configured
	if (_num_lights == 0) {
		return;
	}

	// Check for status color updates from led_controller
	LedControlData led_control_data;

	if (_led_controller.update(led_control_data) != 1) {
		return; // No update, nothing to do
	}

	// Compute status color from led_control_data
	uavcan::equipment::indication::RGB565 status_color = color_to_rgb565(led_control_data.leds[0].color, led_control_data.leds[0].brightness);

	// Build and send light commands for all configured lights
	uavcan::equipment::indication::LightsCommand light_command;
	const bool light_on = is_light_on();

	for (uint8_t i = 0; i < _num_lights; i++) {
		uavcan::equipment::indication::RGB565 color_on, color_off;
		color_on = color_off = color_to_rgb565(led_control_s::COLOR_OFF);

		switch (_light_functions[i]) {
		// Always show status
		case LightFunction::Status:
			color_on = color_off = status_color;
			break;

		// Static color when UAVCAN_LGT_MODE active
		case LightFunction::White:
			color_on = color_to_rgb565(led_control_s::COLOR_WHITE);
			break;

		case LightFunction::Red:
			color_on = color_to_rgb565(led_control_s::COLOR_RED);
			break;

		case LightFunction::Green:
			color_on = color_to_rgb565(led_control_s::COLOR_GREEN);
			break;

		// Hybrid functions: show status when UAVCAN_LGT_MODE inactive, static color when active
		case LightFunction::StatusOrWhite:
			color_on = color_to_rgb565(led_control_s::COLOR_WHITE);
			color_off = status_color;
			break;

		case LightFunction::StatusOrRed:
			color_on = color_to_rgb565(led_control_s::COLOR_RED);
			color_off = status_color;
			break;

		case LightFunction::StatusOrGreen:
			color_on = color_to_rgb565(led_control_s::COLOR_GREEN);
			color_off = status_color;
			break;

		case LightFunction::StatusOrOff:
			color_off = status_color;
			break;
		}

		uavcan::equipment::indication::SingleLightCommand cmd;
		cmd.light_id = _light_ids[i];
		cmd.color = light_on ? color_on : color_off;
		light_command.commands.push_back(cmd);
	}

	_uavcan_pub_lights_cmd.broadcast(light_command);
}

bool UavcanRGBController::is_light_on()
{
	actuator_armed_s actuator_armed{};
	_actuator_armed_sub.copy(&actuator_armed);

	switch (_param_uavcan_lgt_mode.get()) {
	case 3: // Always on
		return true;

	case 2: // When autopilot is prearmed
		return actuator_armed.armed || actuator_armed.prearmed;

	case 1: // When autopilot is armed
		return actuator_armed.armed;

	case 0: // Always off
	default:
		return false;
	}
}

uavcan::equipment::indication::RGB565 UavcanRGBController::color_to_rgb565(uint8_t color, uint8_t brightness)
{
	uint8_t R = 0, G = 0, B = 0;

	switch (color) {
	case led_control_s::COLOR_RED:
		R = brightness;
		break;

	case led_control_s::COLOR_GREEN:
		G = brightness;
		break;

	case led_control_s::COLOR_BLUE:
		B = brightness;
		break;

	case led_control_s::COLOR_AMBER: // make it the same as yellow
	case led_control_s::COLOR_YELLOW:
		R = brightness;
		G = brightness;
		break;

	case led_control_s::COLOR_PURPLE:
		R = brightness;
		B = brightness;
		break;

	case led_control_s::COLOR_CYAN:
		G = brightness;
		B = brightness;
		break;

	case led_control_s::COLOR_WHITE:
		R = brightness;
		G = brightness;
		B = brightness;
		break;

	default:
	case led_control_s::COLOR_OFF:
		break;
	}

	return rgb888_to_rgb565(R, G, B);
}

uavcan::equipment::indication::RGB565 UavcanRGBController::rgb888_to_rgb565(uint8_t red, uint8_t green, uint8_t blue)
{
	// RGB565: Full brightness is (31, 63, 31), off is (0, 0, 0)
	uavcan::equipment::indication::RGB565 rgb565{};
	rgb565.red = (red * 31 + 127) / 255;
	rgb565.green = (green * 63 + 127) / 255;
	rgb565.blue = (blue * 31 + 127) / 255;
	return rgb565;
}
