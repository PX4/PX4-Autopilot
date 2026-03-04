/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#include <float.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/Serial.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vtx.h>
#include <uORB/topics/input_rc.h>

#include "protocol.h"
#include "../vtxtable/VtxTable.hpp"

/**
 * @author Niklas Hauser <niklas@auterion.com>
 */
class VTX : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:

	static Descriptor desc;

	VTX(const char *device);
	virtual ~VTX();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	int init();

private:
	void Run() override;

	void update_params(int *new_band, int *new_channel, int *new_frequency, int *new_power, int *new_pit_mode);
	void handle_uorb();

	char _serial_path[20] {};
	vtx::Protocol *_protocol{nullptr};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::VTX_BAND>) _param_vtx_band,
		(ParamInt<px4::params::VTX_CHANNEL>) _param_vtx_channel,
		(ParamInt<px4::params::VTX_FREQUENCY>) _param_vtx_frequency,
		(ParamInt<px4::params::VTX_POWER>) _param_vtx_power,
		(ParamBool<px4::params::VTX_PIT_MODE>) _param_vtx_pit_mode,
		(ParamInt<px4::params::VTX_MAP_CONFIG>) _param_vtx_map_config,
		(ParamInt<px4::params::VTX_DEVICE>) _param_vtx_device
	);

	perf_counter_t _perf_cycle;
	perf_counter_t _perf_error;
	perf_counter_t _perf_get_settings;
	perf_counter_t _perf_set_power;
	perf_counter_t _perf_set_frequency;
	perf_counter_t _perf_set_pit_mode;

	uORB::PublicationMulti<vtx_s> _vtx_pub{ORB_ID(vtx)};
	uORB::Subscription _input_rc_sub{ORB_ID(input_rc)};

	vtx_s _vtx_msg{};

	int8_t _band{0};
	int8_t _channel{0};
	int16_t _frequency{0};
	int16_t _power{0};
	uint8_t _device{0};
	bool _pit_mode{};

	enum : uint8_t {
		STATE_INIT = 0,
		STATE_GET_SETTINGS = 0b0001,
		STATE_SET_PIT_MODE = 0b0010,
		STATE_SET_FREQUENCY = 0b0100,
		STATE_SET_POWER = 0b1000,
		STATE_UPDATE = 0b1'0000,
		STATE_SET_ALL = STATE_SET_FREQUENCY | STATE_SET_POWER | STATE_SET_PIT_MODE,
		STATE_MASK = 0b1111,
	} _state{STATE_INIT};
	uint8_t _pending{};
	uint8_t _schedule{STATE_GET_SETTINGS};
	int8_t _update_counter{};

	bool _comms_ok{};
	vtx::Config::ChangeType _last_config_change{};
};
