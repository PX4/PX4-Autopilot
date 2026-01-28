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

#pragma once

#include <uORB/topics/actuator_armed.h>

#include <uavcan/uavcan.hpp>
#include <uavcan/equipment/indication/LightsCommand.hpp>

#include <lib/led/led.h>
#include <px4_platform_common/module_params.h>

class UavcanRGBController : public ModuleParams
{
public:
	UavcanRGBController(uavcan::INode &node);
	~UavcanRGBController() = default;

	// setup periodic updater
	int init();

private:
	// Max update rate to avoid excessive bus traffic
	static constexpr unsigned MAX_RATE_HZ = 20;

	// NOTE: This value must match __max_num_uavcan_lights in module.yaml
	static constexpr uint8_t MAX_NUM_UAVCAN_LIGHTS = 2;

	// Light function types
	enum class LightFunction : uint8_t {
		Status = 0,                    // System status colors from led_control
		AntiCollision = 1,             // White beacon based on arm state
		RedNavigation = 2,             // Red navigation light
		GreenNavigation = 3,           // Green navigation light
		WhiteNavigation = 4,           // White navigation light
		StatusOrAntiCollision = 5,     // Status when LGT_MODE inactive, white beacon when active
		StatusOrRedNavigation = 6,     // Status when LGT_MODE inactive, red nav when active
		StatusOrGreenNavigation = 7,   // Status when LGT_MODE inactive, green nav when active
		StatusOrWhiteNavigation = 8    // Status when LGT_MODE inactive, white nav when active
	};

	enum class LightMode : uint8_t {
		Off = 0,
		WhenArmed = 1,
		WhenPrearmed = 2,
		AlwaysOn = 3
	};

	// White light intensity levels
	enum class Brightness { None, Full };

	void periodic_update(const uavcan::TimerEvent &);

	bool check_light_state(LightMode mode);

	uavcan::equipment::indication::RGB565 rgb888_to_rgb565(uint8_t red, uint8_t green, uint8_t blue);

	typedef uavcan::MethodBinder<UavcanRGBController *, void (UavcanRGBController::*)(const uavcan::TimerEvent &)>
	TimerCbBinder;

	uavcan::INode &_node;
	uavcan::Publisher<uavcan::equipment::indication::LightsCommand> _uavcan_pub_lights_cmd;
	uavcan::TimerEventForwarder<TimerCbBinder> _timer;

	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};

	LedController _led_controller;

	// Cached configuration (set during init, requires reboot to change)
	uint8_t _num_lights{0};
	uint8_t _light_ids[MAX_NUM_UAVCAN_LIGHTS] {};
	LightFunction _light_functions[MAX_NUM_UAVCAN_LIGHTS] {};

	// Cached parameter handles
	param_t _light_id_params[MAX_NUM_UAVCAN_LIGHTS] {};
	param_t _light_fn_params[MAX_NUM_UAVCAN_LIGHTS] {};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::UAVCAN_LGT_NUM>) _param_lgt_num,
		(ParamInt<px4::params::UAVCAN_LGT_MODE>) _param_lgt_mode
	)
};
