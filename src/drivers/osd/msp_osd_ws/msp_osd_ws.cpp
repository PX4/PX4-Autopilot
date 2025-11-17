/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include "msp_osd_ws.hpp"
#include "msp_defines.h"
#include "MspV1_ws.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

MspOsd::MspOsd(const char *device) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	// back up device name for connection later
	strcpy(_device, device);
}

MspOsd::~MspOsd()
{
}

bool MspOsd::init()
{
	ScheduleOnInterval(100_ms);
	return true;
}

void MspOsd::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	// perform first time initialization, if needed
	if (!_is_initialized) {
		struct termios t;
		_msp_fd = open(_device, O_RDWR | O_NONBLOCK);

		if (_msp_fd < 0) {
			PX4_ERR("[msp_osd_ws] Failed to open device %s (errno: %d)", _device, errno);
			_performance_data.initialization_problems = true;
			return;
		}

		tcgetattr(_msp_fd, &t);
		cfsetospeed(&t, B115200);
		
		// Set raw mode for both input and output
		t.c_cflag = (t.c_cflag & ~CSIZE) | CS8;  // 8 data bits
		t.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);  // 1 stop bit, no parity, no hardware flow control
		t.c_cflag |= (CLOCAL | CREAD);  // Enable receiver, ignore modem control lines
		
		t.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
		t.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON | IXOFF);
		t.c_oflag &= ~OPOST;  // Raw output, no processing
		
		t.c_cc[VMIN] = 0;
		t.c_cc[VTIME] = 0;
		
		tcsetattr(_msp_fd, TCSANOW, &t);
		tcflush(_msp_fd, TCIOFLUSH);  // Flush any pending I/O

		_msp = MspV1(_msp_fd);

		_is_initialized = true;
	}

	// Handle VTX channel changes
	if (change_channel) {
		msp_get_vtx_config_t vtx_set_config{0};
		vtx_set_config.low_power_disarm = vtx_config.low_power_disarm;
		vtx_set_config.pit_mode = vtx_config.pit_mode;
		vtx_set_config.vtx_type = VTXDEV_MSP;
		vtx_set_config.band = vtx_config.user_band;
		vtx_set_config.channel = vtx_config.user_channel;
		if (_msp.Send(MSP_GET_VTX_CONFIG, &vtx_set_config, sizeof(msp_get_vtx_config_t))) {
			_performance_data.successful_sends++;
		} else {
			_performance_data.unsuccessful_sends++;
		}
		change_channel = false;
	}

	// Handle incoming MSP requests
	this->Receive();

	// Request VTX config if we don't have it yet
	if (!has_vtx_config) {
		if (_msp.Send(MSP_GET_VTX_CONFIG, nullptr, 0)) {
			_performance_data.successful_sends++;
		} else {
			_performance_data.unsuccessful_sends++;
		}
	}

	// Send basic OSD frame sequence matching Arduino format: HEARTBEAT, CLEAR_SCREEN, WRITE_STRING, DRAW_SCREEN
	{
		uint8_t payload[1] = { MSP_DP_HEARTBEAT };
		if (_msp.Send(MSP_CMD_DISPLAYPORT, payload, sizeof(payload))) {
			_performance_data.successful_sends++;
		} else {
			_performance_data.unsuccessful_sends++;
		}
	}

	{
		uint8_t payload[1] = { MSP_DP_CLEAR_SCREEN };
		if (_msp.Send(MSP_CMD_DISPLAYPORT, payload, sizeof(payload))) {
			_performance_data.successful_sends++;
		} else {
			_performance_data.unsuccessful_sends++;
		}
	}

	// Send craft name using Arduino-compatible format
	{
		const char name[] = "PX4_AVATAR_OSD";
		
		// Payload layout for MSP_DP_WRITE_STRING (matching Arduino):
		// [0] subcmd (MSP_DP_WRITE_STRING)
		// [1] x (column)
		// [2] y (row)
		// [3..] ASCII characters, zero-terminated
		uint8_t buffer[64];
		buffer[0] = MSP_DP_WRITE_STRING;
		buffer[1] = 1;  // x (column)
		buffer[2] = 2;  // y (row)
		
		// Copy string, ensure NUL-terminated and within buffer.
		size_t i = 0;
		while (name[i] != '\0' && (3 + i) < (sizeof(buffer) - 1)) {
			buffer[3 + i] = static_cast<uint8_t>(name[i]);
			++i;
		}
		buffer[3 + i] = 0;  // NUL terminator
		
		uint8_t payloadLen = static_cast<uint8_t>(4 + i);  // subcmd + x + y + string+NUL
		if (_msp.Send(MSP_CMD_DISPLAYPORT, buffer, payloadLen)) {
			_performance_data.successful_sends++;
		} else {
			_performance_data.unsuccessful_sends++;
		}
	}

	{
		uint8_t payload[1] = { MSP_DP_DRAW_SCREEN };
		if (_msp.Send(MSP_CMD_DISPLAYPORT, payload, sizeof(payload))) {
			_performance_data.successful_sends++;
		} else {
			_performance_data.unsuccessful_sends++;
		}
	}

}

void MspOsd::Receive()
{
	uint8_t packet[255];
	uint8_t message_id;
	int ret;

	while ((ret = _msp.Receive(packet, &message_id)) != -EWOULDBLOCK) {

		if (ret >= 0) {
			switch (message_id) {

			case MSP_SET_VTX_CONFIG: {
				if (ret == 0xF) {
					memcpy((void *)&vtx_config, packet, sizeof(vtx_config));
					has_vtx_config = true;
				}
				break;
			}

			case MSP_SET_VTXTABLE_BAND: {
				msp_set_vtxtable_band_t *band_info = (msp_set_vtxtable_band_t *)&packet[0];

				// Only supported fixed name length and < 8 channels for now
				if (band_info->band <= BAND_COUNT && band_info->band_name_length == 8 && band_info->channel_count <= 8) {
					memcpy((void *)&vtx_bands[band_info->band - 1], packet, sizeof(msp_set_vtxtable_band_t));

					if (has_vtx_config && band_info->band == vtx_config.band_count) {
						has_vtx_bands = true;
					}
				}
				break;
			}

			case MSP_SET_VTXTABLE_POWERLEVEL: {
				if ((packet[0] - 1) < POWER_LEVEL_COUNT) {
					memcpy((void *)&power_levels[packet[0] - 1], packet, sizeof(msp_set_vtxtable_powerlevel_t));
					has_power_config = true;
				}
				break;
			}

			default:
				break;
			}
		}
	}
}

int MspOsd::task_spawn(int argc, char *argv[])
{
	// initialize device
	const char *device = nullptr;
	bool error_flag = false;

	// loop through input arguments
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device = myoptarg;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return PX4_ERROR;
	}

	if (!device) {
		PX4_ERR("Missing device");
		return PX4_ERROR;
	}

	MspOsd *instance = new MspOsd(device);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MspOsd::print_status()
{
	PX4_INFO("Running on %s", _device);
	PX4_INFO("\tinitialized: %d", _is_initialized);
	PX4_INFO("\tinitialization issues: %d", _performance_data.initialization_problems);
	PX4_INFO("\tsuccessful sends: %lu", _performance_data.successful_sends);
	PX4_INFO("\tunsuccessful sends: %lu", _performance_data.unsuccessful_sends);

	if (has_vtx_config) {
		PX4_INFO("=== VTX Configuration ===");

		if (has_vtx_bands) {
			PX4_INFO("Channel: %c%u", vtx_bands[vtx_config.user_band - 1].band_letter, vtx_config.user_channel);
		} else {
			PX4_INFO("Band: %u", vtx_config.user_band);
			PX4_INFO("Channel: %u", vtx_config.user_channel);
		}

		PX4_INFO("Frequency: %u MHz", vtx_config.user_freq);

		if (has_power_config && (vtx_config.power_level - 1) < POWER_LEVEL_COUNT) {
			PX4_INFO("Transmit power: %.*s mW", power_levels[vtx_config.power_level - 1].power_label_length,
				 power_levels[vtx_config.power_level - 1].power_label_name);
		} else {
			PX4_INFO("Power Level: %u/%u", vtx_config.power_level,  vtx_config.power_count);
		}

		PX4_INFO("PIT Mode: %s", vtx_config.pit_mode ? "On" : "Off");

		const char *disarm_modes[] = {
			"Off",
			"Always",
			"Until First Arm"
		};

		if (vtx_config.low_power_disarm < 3) {
			PX4_INFO("Low Power Disarm: %s", disarm_modes[vtx_config.low_power_disarm]);
		} else {
			PX4_INFO("Low Power Disarm: Unknown (%u)", vtx_config.low_power_disarm);
		}

		PX4_INFO("PIT Frequency: %u MHz", vtx_config.pit_freq);

	} else {
		PX4_INFO("No VTX Configuration available, can't do channel switching");
	}

	return 0;
}

int MspOsd::set_channel(char *new_channel)
{
	char band_letter = toupper(new_channel[0]);

	if (!has_vtx_bands) {
		return -2;
	}

	for (int i = 0; i < BAND_COUNT; i++) {
		if (vtx_bands[i].band != 0) {
			if (band_letter == toupper(vtx_bands[i].band_letter)) {
				int channel = atoi(&new_channel[1]);

				if (channel > 0 && channel <= vtx_config.channel_count && vtx_bands[i].frequency[channel - 1] != 0) {
					vtx_config.user_band = vtx_bands[i].band;
					vtx_config.user_channel = channel;
					vtx_config.user_freq = vtx_bands[i].frequency[channel - 1];
					change_channel = true;
					return 0;
				}
			}
		}
	}

	return -1;
}

int MspOsd::custom_command(int argc, char *argv[])
{
	if (argc > 0 && strcmp("channel", argv[0]) == 0) {
		if (argc == 1) {
			PX4_INFO("Please provide a channel");

		} else if (is_running() && _object.load()) {
			MspOsd *object = _object.load();
			int ret = object->set_channel(argv[1]);

			if (ret == -1) {
				PX4_INFO("Channel not found");

			} else if (ret == -2) {
				PX4_INFO("No VTX Channel table available");
			}

		} else {
			PX4_INFO("not running");
		}
	}

	return 0;
}

int MspOsd::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
MSP telemetry streamer for Walksnail/Avatar VTX

### Implementation
Converts uORB messages to MSP telemetry packets

### Examples
CLI usage example:
$ msp_osd_ws -d /dev/ttyS2

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("msp_osd_ws", "driver");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	PRINT_MODULE_USAGE_COMMAND_DESCR("channel", "Change VTX channel");

	return 0;
}

extern "C" __EXPORT int msp_osd_ws_main(int argc, char *argv[])
{
	return MspOsd::main(argc, argv);
}
