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

#include "VTX.hpp"

#include <fcntl.h>
#include <termios.h>
#include <drivers/drv_hrt.h>
#include "smart_audio.h"
#include "tramp.h"

using namespace time_literals;

ModuleBase::Descriptor VTX::desc{task_spawn, custom_command, print_usage};

VTX::VTX(const char *device) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(device)),
	_perf_cycle(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")),
	_perf_error(perf_alloc(PC_COUNT, MODULE_NAME": errors")),
	_perf_get_settings(perf_alloc(PC_COUNT, MODULE_NAME": errors get settings")),
	_perf_set_power(perf_alloc(PC_COUNT, MODULE_NAME": errors set power")),
	_perf_set_frequency(perf_alloc(PC_COUNT, MODULE_NAME": errors set frequency")),
	_perf_set_pit_mode(perf_alloc(PC_COUNT, MODULE_NAME": errors set pit mode"))
{
	if (device) {
		strlcpy(_serial_path, device, sizeof(_serial_path));
	}
}

VTX::~VTX()
{
	delete _protocol;
	perf_free(_perf_cycle);
	perf_free(_perf_error);
	perf_free(_perf_get_settings);
	perf_free(_perf_set_power);
	perf_free(_perf_set_frequency);
	perf_free(_perf_set_pit_mode);
}

int VTX::init()
{
	_param_vtx_device.update();
	_device = (_param_vtx_device.get() >> 8);
	const uint8_t protocol = (_param_vtx_device.get() & 0xff);

	if (_protocol == nullptr) {
		if (protocol == vtx_s::PROTOCOL_TRAMP) {
			_protocol = new vtx::Tramp();

		} else {
			_protocol = new vtx::SmartAudio();
		}
	}

	if (_protocol == nullptr) {
		PX4_ERR("Protocol alloc failed");
		return PX4_ERROR;
	}

	if (_protocol->init(_serial_path) != PX4_OK) {
		return PX4_ERROR;
	}

	return PX4_OK;
}

void VTX::Run()
{
	static constexpr auto _INTERVAL{50_ms};

	if (should_exit()) {
		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_perf_cycle);

	switch (_state) {
	case STATE_INIT:
		if (init() == PX4_OK) {
			_state = STATE_UPDATE;
			_pending = STATE_SET_ALL;

		} else {
			perf_count(_perf_error);
		}

		break;

	case STATE_GET_SETTINGS:
		if (_protocol->get_settings() == PX4_OK) {
			if (!_comms_ok) {
				// previous get settings failed, send all settings again
				_pending |= STATE_SET_ALL;
			}

			_pending &= ~STATE_GET_SETTINGS;
			_comms_ok = true;

		} else {
			perf_count(_perf_get_settings);
			_comms_ok = false;
		}

		_update_counter = 1_s / _INTERVAL;
		_state = STATE_UPDATE;
		break;

	case STATE_SET_FREQUENCY:
		if ((_frequency ? _protocol->set_frequency(_frequency) : _protocol->set_channel(_band, _channel)) == PX4_OK) {
			_pending &= ~STATE_SET_FREQUENCY;
			_pending |= STATE_GET_SETTINGS;

		} else {
			perf_count(_perf_set_frequency);
		}

		_state = STATE_UPDATE;
		break;

	case STATE_SET_POWER:
		if (_protocol->set_power(_power) == PX4_OK) {
			_pending &= ~STATE_SET_POWER;
			_pending |= STATE_GET_SETTINGS;

		} else {
			perf_count(_perf_set_power);
		}

		_state = STATE_UPDATE;
		break;

	case STATE_SET_PIT_MODE:
		if (_protocol->set_pit_mode(_pit_mode) == PX4_OK) {
			_pending &= ~STATE_SET_PIT_MODE;
			_pending |= STATE_GET_SETTINGS;

		} else {
			perf_count(_perf_set_pit_mode);
		}

		_state = STATE_UPDATE;
		break;

	default:
		// Poll parameters and update settings if needed
		int new_band{_band}, new_channel{_channel}, new_frequency{_frequency}, new_power{_power}, new_pit_mode{_pit_mode};
		update_params(&new_band, &new_channel, &new_frequency, &new_power, &new_pit_mode);

		const vtx::Config::ChangeType current_change = vtxtable().get_change();

		if (_last_config_change != current_change) {
			_last_config_change = current_change;
			_pending |= STATE_SET_ALL;
		}

		if (_band != new_band || _channel != new_channel) {
			_band = new_band;
			_channel = new_channel;
			_pending |= STATE_SET_FREQUENCY;
		}

		if (_frequency != new_frequency) {
			_frequency = new_frequency;
			_pending |= STATE_SET_FREQUENCY;
		}

		if (_power != new_power) {
			_power = new_power;
			_pending |= STATE_SET_POWER;
		}

		if (_pit_mode != new_pit_mode) {
			_pit_mode = new_pit_mode;
			// Some VTX set power to 0 in pit mode, so also set power again
			_pending |= STATE_SET_PIT_MODE | STATE_SET_POWER;
		}

		if (_update_counter <= 0) {
			_pending |= STATE_GET_SETTINGS;

		} else {
			_update_counter--;
		}

		// Round robin schedule the next pending flag test
		uint8_t current{0};

		while (_pending && !current) {
			current = uint8_t(_pending & _schedule);
			_schedule <<= 1;

			if (!(_schedule & STATE_MASK)) {
				_schedule = STATE_GET_SETTINGS;
				break;
			}
		}

		// Convert pending flags to state
		if (current & STATE_GET_SETTINGS) {
			_state = STATE_GET_SETTINGS;

		} else if (current & STATE_SET_FREQUENCY) {
			_state = STATE_SET_FREQUENCY;

		} else if (current & STATE_SET_POWER) {
			_state = STATE_SET_POWER;

		} else if (current & STATE_SET_PIT_MODE) {
			_state = STATE_SET_PIT_MODE;
		}

		handle_uorb();
		break;
	}

	ScheduleDelayed(_INTERVAL);
	perf_end(_perf_cycle);
}

void VTX::update_params(int *new_band, int *new_channel, int *new_frequency, int *new_power, int *new_pit_mode)
{
	_param_vtx_band.update();
	_param_vtx_channel.update();
	_param_vtx_power.update();
	_param_vtx_frequency.update();
	_param_vtx_pit_mode.update();
	_param_vtx_map_config.update();
	const int map_config = _param_vtx_map_config.get();

	// Set transmit channel based on either parameter or manual auxiliary channel input
	if (map_config > 1) {
		input_rc_s input_rc{};

		if (_input_rc_sub.update(&input_rc) && !input_rc.rc_lost) {
			int8_t band{0}, channel{0}, power{0};
			vtxtable().map_lookup(input_rc.values, input_rc.channel_count, &band, &channel, &power);

			if (map_config > 2) {
				if (band > 0) { _param_vtx_band.commit_no_notification(band - 1); }

				if (channel > 0) { _param_vtx_channel.commit_no_notification(channel - 1); }
			}

			if (map_config == 2 || map_config == 4) {
				if (power == -1) {
					_param_vtx_power.commit_no_notification(0);
					_param_vtx_pit_mode.commit_no_notification(true);

				} else if (power > 0) {
					_param_vtx_power.commit_no_notification(power - 1);
					_param_vtx_pit_mode.commit_no_notification(false);
				}
			}
		}
	}

	*new_band = _param_vtx_band.get() % 24;
	*new_channel = _param_vtx_channel.get() & 0xF;
	*new_power = _param_vtx_power.get() & 0xF;
	*new_frequency = _param_vtx_frequency.get();
	*new_pit_mode = _param_vtx_pit_mode.get();
}

void VTX::handle_uorb()
{
	vtx_s msg{};
	msg.band = _band;
	msg.channel = _channel;
	msg.power_level = _power;
	msg.device = _device;

	if (!_comms_ok || !_protocol->copy_to(&msg)) {
		msg.protocol = vtx_s::PROTOCOL_NONE;
		msg.frequency = (_frequency ? _frequency : vtxtable().frequency(_band, _channel));
		msg.mode = _pit_mode ? vtx_s::MODE_PIT : vtx_s::MODE_NORMAL;
	}

	// If frequency is set, overwrite band and channel to -1
	if (_frequency) {
		msg.band = -1;
		msg.channel = -1;
		msg.band_letter = 'f';
		strncpy((char *)msg.band_name, "FREQUENCY", sizeof(msg.band_name));

	} else {
		msg.band_letter = vtxtable().band_letter(msg.band);
		strncpy((char *)msg.band_name, vtxtable().band_name(msg.band), sizeof(msg.band_name));
	}

	strncpy((char *)msg.power_label, vtxtable().power_label(msg.power_level), sizeof(msg.power_label));

	// Workarounds for specific devices
	if (_device == vtx_s::DEVICE_PEAK_THOR_T67) {
		// This device always reports pit mode, but still works fine
		msg.frequency = vtxtable().frequency(_band, _channel);
		msg.mode = _pit_mode ? vtx_s::MODE_PIT : vtx_s::MODE_NORMAL;
	}

	// Only publish if something changed
	if (memcmp(&msg, &_vtx_msg, sizeof(msg)) != 0) {
		_vtx_msg = msg;
		msg.timestamp = hrt_absolute_time();
		_vtx_pub.publish(msg);
	}
}

int VTX::custom_command(int argc, char *argv[])
{
	if (!strcmp(argv[0], "start")) {
		if (is_running(desc)) {
			return print_usage("already running");
		}

		int ret = VTX::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

	if (!is_running(desc)) {
		return print_usage("not running");
	}

	// Check if the first argument is a number and delegate to the VtxTable
	char *endptr{};
	strtol(argv[0], &endptr, 10);

	if (endptr != argv[0]) {
		extern int vtxtable_custom_command(int argc, char *argv[]);
		return vtxtable_custom_command(argc, argv);
	}

	return print_usage("unknown command");
}

int VTX::task_spawn(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	const char *device_name = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device_name = myoptarg;
			break;

		default:
			PX4_WARN("unrecognized flag");
			return -1;
			break;
		}
	}

	if (device_name && (access(device_name, R_OK | W_OK) == 0)) {
		auto *const instance = new VTX(device_name);

		if (instance == nullptr) {
			PX4_ERR("alloc failed");
			return PX4_ERROR;
		}

		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		instance->ScheduleNow();

		return PX4_OK;

	} else {
		if (device_name) {
			PX4_ERR("invalid device (-d) %s", device_name);

		} else {
			PX4_INFO("valid device required");
		}
	}

	return PX4_ERROR;
}

int VTX::print_status()
{
	if (_serial_path[0]) {
		PX4_INFO("UART device: %s", _serial_path);
	}

#ifdef CONFIG_VTX_CRSF_MSP_SUPPORT
	const char *const config_map[] {"Disabled", "MSP", "MSP=Band/Channel, AUX=Power", "MSP=Power, AUX=Band/Channel", "AUX"};
#else
	const char *const config_map[] {"Disabled", "Disabled", "Power", "Band/Channel", "Power/Band/Channel"};
#endif
	const char *const config_str = config_map[_param_vtx_map_config.get() <= 4 ? _param_vtx_map_config.get() : 0];
	PX4_INFO("RC mapping: %s", config_str);

	PX4_INFO("Parameters:");
	PX4_INFO("  band: %d", (_frequency ? -1 : (_band + 1)));
	PX4_INFO("  channel: %d", (_frequency ? -1 : (_channel + 1)));
	PX4_INFO("  frequency: %u MHz", (_frequency ? _frequency : vtxtable().frequency(_band, _channel)));
	PX4_INFO("  power level: %u", _power + 1);
	PX4_INFO("  power: %hi = %s", vtxtable().power_value(_power), vtxtable().power_label(_power));
	PX4_INFO("  pit mode: %s", _pit_mode ? "on" : "off");

	if (!(_comms_ok && _protocol && _protocol->print_settings())) {
		PX4_ERR("%s device not found", _param_vtx_device.get() == 1 ? "Tramp" : "SmartAudio");
	}

	perf_print_counter(_perf_cycle);
	perf_print_counter(_perf_error);
	perf_print_counter(_perf_get_settings);
	perf_print_counter(_perf_set_power);
	perf_print_counter(_perf_set_frequency);
	perf_print_counter(_perf_set_pit_mode);

	return 0;
}

int VTX::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module communicates with a VTX camera via serial port. It can be used to
configure the camera settings and to control the camera's video transmission.
Supported protocols are:
- SmartAudio v1, v2.0, v2.1
- Tramp

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("vtx", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<file:dev>", "VTX device", false);

	PRINT_MODULE_USAGE_COMMAND_DESCR("<int>", "Sets an entry in the mapping table: <index> <aux channel> <band> <channel> <power level> <start range> <end range>");

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int vtx_main(int argc, char *argv[])
{
	return ModuleBase::main(VTX::desc, argc, argv);
}
