/****************************************************************************
 *
 *   Copyright (c) 2012-2020 PX4 Development Team. All rights reserved.
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


#include <string>
#include <px4_log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/getopt.h>
#include <uORB/PublicationMulti.hpp>
#include <drivers/device/qurt/uart.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/input_rc.h>
#include <lib/parameters/param.h>

#include "protocol.h"

#define ASYNC_UART_READ_WAIT_US 2000


extern "C" { __EXPORT int dsp_sbus_main(int argc, char *argv[]); }

namespace dsp_sbus
{

std::string _port = "7";
int _uart_fd = -1;
IOPacket _packet;
bool _initialized = false;
bool _is_running = false;
uint64_t _rc_last_valid;		///< last valid timestamp
uint16_t _rc_valid_update_count = 0;

static px4_task_t _task_handle = -1;

uORB::PublicationMulti<input_rc_s> _rc_pub{ORB_ID(input_rc)};

int bus_exchange(IOPacket *packet)
{
	int ret = 0;
	int read_retries = 3;
	int read_succeeded = 0;
	int packet_size = sizeof(IOPacket);

	(void) qurt_uart_write(_uart_fd, (const char *) packet, packet_size);

	usleep(100);

	// The UART read on SLPI is via an asynchronous service so specify a timeout
	// for the return. The driver will poll periodically until the read comes in
	// so this may block for a while. However, it will timeout if no read comes in.
	while (read_retries) {
		ret = qurt_uart_read(_uart_fd, (char *) packet, packet_size, ASYNC_UART_READ_WAIT_US);

		if (ret) {
			// PX4_INFO("Read %d bytes", ret);

			/* Check CRC */
			uint8_t crc = packet->crc;
			packet->crc = 0;

			if (crc != crc_packet(packet)) {
				PX4_ERR("PX4IO packet CRC error");
				return -EIO;

			} else if (PKT_CODE(*packet) == PKT_CODE_CORRUPT) {
				PX4_ERR("PX4IO packet corruption");
				return -EIO;

			} else {
				read_succeeded = 1;
				break;
			}
		}

		PX4_ERR("Read attempt %d failed", read_retries);
		read_retries--;
	}


	if (! read_succeeded) {
		return -EIO;
	}

	return 0;
}

int io_reg_get(uint8_t page, uint8_t offset, uint16_t *values, unsigned num_values)
{
	/* range check the transfer */
	// if (num_values > ((_max_transfer) / sizeof(*values))) {
	// 	PX4_ERR("io_reg_get: too many registers (%u, max %u)", num_values, _max_transfer / 2);
	// 	return -1;
	// }

	// int ret = _interface->read((page << 8) | offset, reinterpret_cast<void *>(values), num_values);
	int ret = 0;

	_packet.count_code = num_values | PKT_CODE_READ;
	_packet.page = page;
	_packet.offset = offset;

	_packet.crc = 0;
	_packet.crc = crc_packet(&_packet);

	ret = bus_exchange(&_packet);

	if (ret != 0) {
		// PX4_ERR("px4io io_reg_get(%hhu,%hhu,%u): data error %d", page, offset, num_values, ret);
		return -1;
	}

	memcpy(values, &_packet.regs[0], num_values * 2);

	return OK;
}

uint32_t io_reg_get(uint8_t page, uint8_t offset)
{
	uint16_t value;

	if (io_reg_get(page, offset, &value, 1) != OK) {
		// Registers are only 16 bit so any value over 0xFFFF can signal a fault
		return 0xFFFFFFFF;
	}

	return value;
}

int initialize()
{
	if (_initialized) {
		// Already successfully initialized
		return 0;
	}

	if (_uart_fd < 0) {
		_uart_fd = qurt_uart_open(_port.c_str(), 921600);
	}

	if (_uart_fd < 0) {
		PX4_ERR("Open failed in %s", __FUNCTION__);
		return -1;
	}

	// Verify connectivity and version number
	unsigned protocol = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_PROTOCOL_VERSION);

	if (protocol != PX4IO_PROTOCOL_VERSION) {
		PX4_ERR("dsp_sbus version error: %u", protocol);
		_uart_fd = -1;
		return -1;
	}

	_initialized = true;

	return 0;
}

void dsp_sbus_task()
{

	uint16_t status_regs[2] {};
	input_rc_s	rc_val;
	const unsigned prolog = (PX4IO_P_RAW_RC_BASE - PX4IO_P_RAW_RC_COUNT);
	uint16_t rc_regs[input_rc_s::RC_INPUT_MAX_CHANNELS + prolog];
	uint32_t channel_count = 0;

	_is_running = true;

	while (true) {

		usleep(20000); // Update every 20ms

		memset(&rc_val, 0, sizeof(input_rc_s));

		if (io_reg_get(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_FLAGS, &status_regs[0],
			       sizeof(status_regs) / sizeof(status_regs[0])) == OK) {
			// PX4_INFO("dsp_sbus status 0x%.4x", status_regs[0]);
			// PX4_INFO("dsp_sbus alarms 0x%.4x", status_regs[1]);
		} else {
			// PX4_ERR("Failed to read status / alarm registers");
			continue;
		}

		/* fetch values from IO */

		// When starting the RC flag will not be okay if the receiver isn't
		// getting a signal from the transmitter. Once it does, then this flag
		// will say okay even if later the signal is lost.
		if (!(status_regs[0] & PX4IO_P_STATUS_FLAGS_RC_OK)) {
			// PX4_INFO("RC lost status flag set");
			rc_val.rc_lost = true;

		} else {
			// PX4_INFO("RC lost status flag is not set");
			rc_val.rc_lost = false;
		}

		if (status_regs[0] & PX4IO_P_STATUS_FLAGS_RC_SBUS) {
			rc_val.input_source = input_rc_s::RC_INPUT_SOURCE_PX4IO_SBUS;
			// PX4_INFO("Got valid SBUS");

		} else {
			rc_val.input_source = input_rc_s::RC_INPUT_SOURCE_UNKNOWN;
			// PX4_INFO("SBUS not valid");
		}

		rc_val.timestamp = hrt_absolute_time();

		// No point in reading the registers if we haven't acquired a transmitter signal yet
		if (! rc_val.rc_lost) {
			if (io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_COUNT, &rc_regs[0],
				       sizeof(rc_regs) / sizeof(rc_regs[0])) != OK) {
				// PX4_ERR("Failed to read RC registers");
				continue;
				// } else {
				// 	PX4_INFO("Successfully read RC registers");
				// 	PX4_INFO("Prolog: %u 0x%.4x 0x%.4x 0x%.4x 0x%.4x 0x%.4x",
				// 			 rc_regs[0], rc_regs[1], rc_regs[2], rc_regs[3], rc_regs[4], rc_regs[5]);
			}

			channel_count = rc_regs[PX4IO_P_RAW_RC_COUNT];

			// const uint16_t rc_valid_update_count = rc_regs[PX4IO_P_RAW_FRAME_COUNT];
			// const bool rc_updated = (rc_valid_update_count != _rc_valid_update_count);
			//
			// if (!rc_updated) {
			// 	PX4_INFO("Didn't get an RC update indication. %u %u", rc_valid_update_count, _rc_valid_update_count);
			// 	continue;
			// }
			//
			// _rc_valid_update_count = rc_valid_update_count;
			//
			// PX4_INFO("Got an RC update indication");

			/* limit the channel count */
			if (channel_count > input_rc_s::RC_INPUT_MAX_CHANNELS) {
				// PX4_INFO("Got %u for channel count. Limiting to 18", channel_count);
				channel_count = input_rc_s::RC_INPUT_MAX_CHANNELS;
			}

			rc_val.channel_count = channel_count;
			// PX4_INFO("RC channel count: %u", rc_val.channel_count);

			// rc_val.rc_ppm_frame_length = rc_regs[PX4IO_P_RAW_RC_DATA];
			rc_val.rc_ppm_frame_length = 0;

			rc_val.rc_failsafe = (rc_regs[PX4IO_P_RAW_RC_FLAGS] & PX4IO_P_RAW_RC_FLAGS_FAILSAFE);
			// rc_val.rc_lost = !(rc_regs[PX4IO_P_RAW_RC_FLAGS] & PX4IO_P_RAW_RC_FLAGS_RC_OK);
			rc_val.rc_lost = rc_val.rc_failsafe;
			rc_val.rc_lost_frame_count = rc_regs[PX4IO_P_RAW_LOST_FRAME_COUNT];
			rc_val.rc_total_frame_count = rc_regs[PX4IO_P_RAW_FRAME_COUNT];

			if (!rc_val.rc_lost && !rc_val.rc_failsafe) {
				_rc_last_valid = rc_val.timestamp;
				rc_val.rssi = rc_regs[PX4IO_P_RAW_RC_NRSSI];
				rc_val.link_quality = rc_regs[PX4IO_P_RAW_RC_NRSSI];

				/* last thing set are the actual channel values as 16 bit values */
				for (unsigned i = 0; i < channel_count; i++) {
					rc_val.values[i] = rc_regs[prolog + i];
					// PX4_INFO("RC channel %u: %.4u", i, rc_val.values[i]);
				}

				/* zero the remaining fields */
				for (unsigned i = channel_count; i < (sizeof(rc_val.values) / sizeof(rc_val.values[0])); i++) {
					rc_val.values[i] = 0;
				}
			}

			rc_val.timestamp_last_signal = _rc_last_valid;
		}

		_rc_pub.publish(rc_val);

	}
}

int start(int argc, char *argv[])
{

	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "p:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			_port = myoptarg;
			PX4_INFO("Setting port to %s", _port.c_str());
			break;

		default:
			break;
		}
	}

	if (! _initialized) {
		if (initialize()) {
			return -1;
		}
	}

	if (_is_running) {
		PX4_WARN("Already started");
		return 0;
	}

	_task_handle = px4_task_spawn_cmd("dsp_sbus_main",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_DEFAULT,
					  2000,
					  (px4_main_t) &dsp_sbus_task,
					  (char *const *)argv);

	if (_task_handle < 0) {
		PX4_ERR("task start failed");
		return -1;
	}

	return 0;
}

void
usage()
{
	PX4_INFO("Usage: dsp_sbus start [options]");
	PX4_INFO("Options: -p <number>    uart port number");
}

} // End namespance dsp_sbus

int dsp_sbus_main(int argc, char *argv[])
{
	int myoptind = 1;

	if (argc <= 1) {
		dsp_sbus::usage();
		return -1;
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		return dsp_sbus::start(argc - 1, argv + 1);

	} else {
		dsp_sbus::usage();
		return -1;
	}

	return 0;
}
