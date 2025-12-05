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
		strlcpy(_device, device, sizeof(_device));
	}
#ifdef CONFIG_VTX_USE_STORAGE
	load();
#endif
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
	if (_protocol == nullptr) {
		if (_param_vtx_protocol.get() == 1) {
			_protocol = new vtx::Tramp(&_config);

		} else {
			_protocol = new vtx::SmartAudio(&_config);
		}
	}

	if (_protocol == nullptr) {
		PX4_ERR("Protocol alloc failed");
		return PX4_ERROR;
	}

	if (_protocol->init(_device) != PX4_OK) {
		return PX4_ERROR;
	}
	return PX4_OK;
}

void VTX::Run()
{
	static constexpr auto _INTERVAL{50_ms};

	if (should_exit()) {
		exit_and_cleanup();
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
	{
		if ((_frequency ? _protocol->set_frequency(_frequency) : _protocol->set_channel(_channel >> 3, _channel & 0b111)) == PX4_OK) {
			_pending &= ~STATE_SET_FREQUENCY;
			_pending |= STATE_GET_SETTINGS;

		} else {
			perf_count(_perf_set_frequency);
		}

		_state = STATE_UPDATE;
		break;
	}

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
		int new_channel{_channel}, new_frequency{_frequency}, new_power{_power}, new_pit_mode{_pit_mode};
		update_params(&new_channel, &new_frequency, &new_power, &new_pit_mode);

		if (_channel != new_channel) {
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

void VTX::update_params(int *new_channel, int *new_frequency, int *new_power, int *new_pit_mode)
{
	_param_vtx_band.update();
	_param_vtx_channel.update();
	_param_vtx_power.update();
	_param_vtx_frequency.update();
	_param_vtx_pit_mode.update();
	_param_vtx_map_config.update();
	_param_vtx_protocol.update();
	const int map_config = _param_vtx_map_config.get();


	// Set transmit channel based on either parameter or manual auxiliary channel input
	if (map_config > 1) {
		input_rc_s input_rc{};

		if (_input_rc_sub.update(&input_rc) && !input_rc.rc_lost) {
			int8_t band{0}, channel{0}, power{0};
			_config.map_lookup(input_rc.values, input_rc.channel_count, &band, &channel, &power);

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

	*new_channel = ((_param_vtx_band.get() & 0b111) << 3) | (_param_vtx_channel.get() & 0b111);
	*new_power = _param_vtx_power.get();
	*new_frequency = _param_vtx_frequency.get();
	*new_pit_mode = _param_vtx_pit_mode.get();
}

void VTX::handle_uorb()
{
	vtx_s msg{};
	msg.band = (_channel >> 3) & 0b111;
	msg.channel = _channel & 0b111;

	if (!_comms_ok || !_protocol->copy_to(&msg)) {
		msg.protocol = vtx_s::PROTOCOL_NONE;
		msg.frequency = (_frequency ? _frequency : _config.frequency(_param_vtx_band.get(), _param_vtx_channel.get()));
		msg.power_level = _param_vtx_power.get();
		msg.power_mw = _config.power_mW(_param_vtx_power.get());
		msg.mode = _param_vtx_pit_mode.get() ? vtx_s::MODE_PIT : vtx_s::MODE_NORMAL;
	}

	// If frequency is set, overwrite band and channel to A1
	if (_frequency) {
		msg.band = 0;
		msg.channel = 0;
	}

	// Only publish if something changed
	if (memcmp(&msg, &_vtx_msg, sizeof(msg)) != 0) {
		_vtx_msg = msg;
		msg.timestamp = hrt_absolute_time();
		_vtx_pub.publish(msg);
	}

#ifdef CONFIG_VTX_UORB_CONFIG

	vtx_aux_map_s vtx_aux_map{};
	vtx_table_s vtx_table{};

	if (_vtx_table_sub.update(&vtx_table)) {
		_config.copy_from(&vtx_table);

		if (_config.store(_config_file) != PX4_OK) {
			perf_count(_perf_error);
		}
	}

	if (_vtx_aux_map_sub.update(&vtx_aux_map)) {
		_config.copy_from(&vtx_aux_map);

		if (_config.store(_config_file) != PX4_OK) {
			perf_count(_perf_error);
		}
	}

#endif
}

#ifdef CONFIG_VTX_USE_STORAGE
int VTX::store(const char *filename)
{
	if (filename == nullptr) { filename = _config_file; }

	const int rv = _config.store(filename);

	if (rv < 0) {
		PX4_ERR("%s is not accessible!", filename);
		return PX4_ERROR;
	}

	PX4_INFO("saved to %s", filename);
	return PX4_OK;
}

int VTX::load(const char *filename)
{
	if (filename == nullptr) { filename = _config_file; }

	const int rv = _config.load(filename);

	if (rv < 0) {
		switch (rv) {
		case -ENOENT:
			PX4_ERR("%s not found!", filename);
			break;

		case -EPROTO:
			PX4_ERR("VTX config serialization format is unsupported in %s!", filename);
			break;

		case -EMSGSIZE:
			PX4_ERR("VTX config in %s is incomplete!", filename);
			break;

		default:
			PX4_ERR("Loading VTX config from %s failed!", filename);
			break;
		}

		return PX4_ERROR;
	}

	PX4_INFO("loaded from %s", filename);
	return PX4_OK;
}
#endif

int VTX::custom_command(int argc, char *argv[])
{
	/* start the FMU if not running */
	if (!strcmp(argv[0], "start")) {
		if (is_running()) {
			return print_usage("already running");
		}

		int ret = VTX::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

	if (!is_running()) {
		return print_usage("not running");
	}

	auto *const instance = _object.load();

	if (!strcmp(argv[0], "name")) {
		if (argc < 1) {
			return print_usage("not enough arguments");
		}

		return instance->_config.set_name(argv[1]) ? PX4_OK : PX4_ERROR;
	}

	if (!strcmp(argv[0], "band")) {
		if (argc < 5) {
			return print_usage("not enough arguments");
		}

		const size_t band = atoi(argv[1]) - 1;
		instance->_config.set_band_name(band, argv[2]);
		instance->_config.set_band_letter(band, argv[3][0]);

		for (int i = 4; i < argc; i++) {
			instance->_config.set_frequency(band, i - 4, atoi(argv[i]));
		}

		instance->_pending = STATE_SET_ALL;
		return PX4_OK;
	}

	if (!strcmp(argv[0], "bands")) {
		if (argc < 2) {
			return print_usage("not enough arguments");
		}

		instance->_pending = STATE_SET_ALL;
		return instance->_config.set_bands(atoi(argv[1])) ? PX4_OK : PX4_ERROR;
	}

	if (!strcmp(argv[0], "powervalues")) {
		uint8_t levels{1};

		for (; levels < argc; levels++) {
			instance->_config.set_power_value(levels - 1, atoi(argv[levels]));
		}

		instance->_pending = STATE_SET_ALL;
		return instance->_config.set_power_levels(levels - 1) ? PX4_OK : PX4_ERROR;
	}

	if (!strcmp(argv[0], "powerlabels")) {
		uint8_t levels{1};

		for (; levels < argc; levels++) {
			instance->_config.set_power_mW(levels - 1, atoi(argv[levels]));
		}

		instance->_pending = STATE_SET_ALL;
		return instance->_config.set_power_levels(levels - 1) ? PX4_OK : PX4_ERROR;
	}

	if (!strcmp(argv[0], "powerlevels")) {
		if (argc < 2) {
			return print_usage("not enough arguments");
		}

		instance->_pending = STATE_SET_ALL;
		return instance->_config.set_power_levels(atoi(argv[1])) ? PX4_OK : PX4_ERROR;
	}

	if (!strcmp(argv[0], "map")) {
		if (argc < 7) {
			return print_usage("not enough arguments");
		}

		instance->_pending = STATE_SET_ALL;
		return instance->_config.set_map_entry(
			       atoi(argv[1]), atoi(argv[2]) + 4 - 1, atoi(argv[3]),
			       atoi(argv[4]), atoi(argv[5]), atoi(argv[6]), atoi(argv[7])) ? PX4_OK : PX4_ERROR;
	}

#ifdef CONFIG_VTX_USE_STORAGE
	const char *const filename = (argc == 2) ? argv[1] : nullptr;

	if (!strcmp(argv[0], "save")) {
		return instance->store(filename);
	}

	if (!strcmp(argv[0], "load")) {
		instance->_pending = STATE_SET_ALL;
		return instance->load(filename);
	}

#endif

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

		_object.store(instance);
		_task_id = task_id_is_work_queue;

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
	if (_device[0]) {
		PX4_INFO("UART device: %s", _device);
	}

	PX4_INFO("VTX table \"%s\":", _config.name());

	for (uint8_t b = 0; b < _config.bands(); b++) {
		PX4_INFO("  %c: %-*s= %4hu %4hu %4hu %4hu %4hu %4hu %4hu %4hu",
			 _config.band_letter(b), vtx::Config::NAME_LENGTH, _config.band_name(b),
			 _config.frequency(b, 0), _config.frequency(b, 1), _config.frequency(b, 2), _config.frequency(b, 3),
			 _config.frequency(b, 4), _config.frequency(b, 5), _config.frequency(b, 6), _config.frequency(b, 7));
	}

	PX4_INFO("Power levels:");

	for (uint8_t p = 0; p < _config.power_levels(); p++) {
		PX4_INFO("  %u: %2hi = %4hi mW", p + 1, _config.power_value(p), _config.power_mW(p));
	}

#ifdef CONFIG_VTX_CRSF_MSP_SUPPORT
	const char *const config_map[] {"Disabled", "MSP", "MSP=Band/Channel, AUX=Power", "MSP=Power, AUX=Band/Channel", "AUX"};
#else
	const char *const config_map[] {"Disabled", "Disabled", "Power", "Band/Channel", "Power/Band/Channel"};
#endif
	const char *const config_str = config_map[_param_vtx_map_config.get() <= 4 ? _param_vtx_map_config.get() : 0];
	PX4_INFO("RC mapping: %s", config_str);

	for (uint8_t i = 0; i < vtx::Config::MAP_LENGTH; i++) {
		uint8_t rc_channel{}, band{}, channel{};
		int8_t power_level{};
		uint16_t start_range{}, end_range{};
		_config.map_entry(i, &rc_channel, &band, &channel, &power_level, &start_range, &end_range);

		if (!start_range && !end_range) { break; }

		if (!band && !channel) {
			if (power_level == -1) {
				PX4_INFO("  %2u: Ch%-2u, %4hu - %4hu = pit mode",
					 i, rc_channel, start_range, end_range);

			} else {
				PX4_INFO("  %2u: Ch%-2u, %4hu - %4hu = %4hi mW",
					 i, rc_channel, start_range, end_range,
					 _config.power_mW(power_level - 1));
			}

		} else {
			PX4_INFO("  %2u: Ch%-2u, %4hu - %4hu =  %u + %u",
				 i, rc_channel, start_range, end_range,
				 band, channel);
		}
	}

	PX4_INFO("Parameters:");
	PX4_INFO("  band: %u", (_frequency ? 0 : ((_channel >> 3) & 0b111) + 1));
	PX4_INFO("  channel: %u", (_frequency ? 0 : (_channel & 0b111) + 1));
	PX4_INFO("  frequency: %u MHz", (_frequency ? _frequency : _config.frequency((_channel >> 3) & 0b111,
					 _channel & 0b111)));
	PX4_INFO("  power level: %u", _power + 1);
	PX4_INFO("  power: %hi = %hi mW", _config.power_value(_power), _config.power_mW(_power));
	PX4_INFO("  pit mode: %s", _pit_mode ? "on" : "off");

	if (!(_comms_ok && _protocol && _protocol->print_settings())) {
		PX4_ERR("%s device not found", _param_vtx_protocol.get() == 1 ? "Tramp" : "SmartAudio");
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

	PRINT_MODULE_USAGE_COMMAND_DESCR("name", "Sets the VTX table name: <name>");

	PRINT_MODULE_USAGE_COMMAND_DESCR("bands", "Sets the number of bands: int");
	PRINT_MODULE_USAGE_COMMAND_DESCR("band", "Sets the band frequencies: <1-index> <name> <letter> <8x frequency>");

	PRINT_MODULE_USAGE_COMMAND_DESCR("powerlevels", "Sets number of power levels: <number of power levels>");
	PRINT_MODULE_USAGE_COMMAND_DESCR("powervalues", "Sets the ≤8 power levels values: <level-0> <level-1>...");
	PRINT_MODULE_USAGE_COMMAND_DESCR("powerlabels", "Sets the ≤8 power levels in mW: <mW level-0> <mW level-1>...");

	PRINT_MODULE_USAGE_COMMAND_DESCR("map", "Sets an entry in the mapping table: <index> <aux channel> <band> <channel> <power level> <start range> <end range>");

#ifdef CONFIG_VTX_USE_STORAGE
	PRINT_MODULE_USAGE_COMMAND_DESCR("save", "Saves the current VTX config to a file: <file>");
	PRINT_MODULE_USAGE_COMMAND_DESCR("load", "Loads the VTX config from a file: <file>");
#endif

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int vtx_main(int argc, char *argv[])
{
	return VTX::main(argc, argv);
}
