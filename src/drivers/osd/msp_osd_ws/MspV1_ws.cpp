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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <syslog.h>

#include <sys/types.h>
#include <stdbool.h>
#include <float.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <drivers/drv_pwm_output.h>
#include <drivers/drv_hrt.h>

#include "msp_defines.h"
#include "MspV1_ws.hpp"


static constexpr uint8_t MSP_HEADER_0 = '$';
static constexpr uint8_t MSP_HEADER_1 = 'M';
static constexpr uint8_t MSP_DIR_TO_FC   = '<';
static constexpr uint8_t MSP_DIR_FROM_FC = '>';

MspV1::MspV1(int fd) :
	_fd(fd)
{
}

bool MspV1::Send(const uint8_t message_id, const void *payload, uint32_t payload_size)
{
	uint8_t checksum = 0;
	uint8_t byte_value;

	// Write MSP header: '$' 'M' '>'
	byte_value = '$';
	if (write(_fd, &byte_value, 1) != 1) return false;

	byte_value = 'M';
	if (write(_fd, &byte_value, 1) != 1) return false;

	byte_value = '>';
	if (write(_fd, &byte_value, 1) != 1) return false;

	// Write payload length and start checksum calculation
	byte_value = (uint8_t)payload_size;
	if (write(_fd, &byte_value, 1) != 1) return false;
	checksum ^= byte_value;

	// Write message ID
	byte_value = message_id;
	if (write(_fd, &byte_value, 1) != 1) return false;
	checksum ^= byte_value;

	// Write payload bytes
	if (payload && payload_size > 0) {
		const uint8_t *payload_data = static_cast<const uint8_t *>(payload);
		for (uint32_t i = 0; i < payload_size; ++i) {
			byte_value = payload_data[i];
			if (write(_fd, &byte_value, 1) != 1) return false;
			checksum ^= byte_value;
		}
	}

	// Write checksum
	if (write(_fd, &checksum, 1) != 1) return false;

	// Flush output to ensure all bytes are transmitted
	tcdrain(_fd);

	return true;
}


int MspV1::Receive(uint8_t *payload, uint8_t *message_id)
{
	uint8_t payload_size;
	uint8_t crc;
	uint8_t calc_crc;
	int ret;
	while (!has_header) {
		int bytes_available = 0;

		if (ioctl(_fd, FIONREAD, &bytes_available) < 0) {
			return -EIO;
		}

		if (bytes_available < 5) {
			return -EWOULDBLOCK;
		}

		while (bytes_available > 4) {
			if ((ret = read(_fd, header, 1)) != 1) {
				return ret;
			}

			bytes_available--;

			if (header[0] == '$') {
				break;
			}

		}

		if (header[0] != '$') {
			return -EWOULDBLOCK;
		}

		if ((ret = read(_fd, &header[1], 4)) != 4) {
			return ret;
		}

		if (header[0] == '$' && header[1] == 'M' && header[2] == '<') {
			has_header = true;
		}
	}

	payload_size = header[3];
	*message_id = header[4];

	if ((ret = read(_fd, payload, payload_size + MSP_CRC_SIZE)) != payload_size + MSP_CRC_SIZE) {
		if (ret != -EWOULDBLOCK) {
			has_header = false;
		}

		return ret;
	}

	has_header = false;

	crc = payload[payload_size];

	calc_crc = payload_size ^ header[4];

	for (int i = 0; i < payload_size; i++) {
		calc_crc ^= payload[i];
	}

	if (calc_crc != crc) {
		return -EINVAL;
	}

	// If this is a request from host/Avatar (direction '<'), handle it automatically
	if (header[2] == '<') {
		handleMspRequest(*message_id, payload, payload_size);
	}

	return payload_size;
}

void MspV1::handleMspRequest(uint8_t cmd, const uint8_t *payload, uint8_t len) {
	(void)payload;
	(void)len;
	// Respond to a few identity queries the Avatar might send, similar to the
	// Arduino `handleMspRequest` used in the ESP32 emulator example.

	PX4_INFO("handleMspRequest: cmd=%d, len=%d", cmd, len);
	switch (cmd) {
	case MSP_API_VERSION: {
		uint8_t resp[3] = MSP_API_VERSION_BYTES;
		Send(MSP_API_VERSION, resp, sizeof(resp));
		break;
	}

	case MSP_FC_VARIANT: {
		uint8_t resp[4] = MSP_FC_VARIANT_BYTES;
		Send(MSP_FC_VARIANT, resp, sizeof(resp));
		break;
	}

	case MSP_FC_VERSION: {
		uint8_t resp[3] = MSP_FC_VERSION_BYTES;
		Send(MSP_FC_VERSION, resp, sizeof(resp));
		break;
	}

	case MSP_NAME: {
		const char name[] = "Betaflight";
		uint8_t resp[sizeof(name)];
		memcpy(resp, name, sizeof(resp));
		Send(MSP_NAME, resp, sizeof(resp));
		break;
	}


	case MSP_STATUS: {
		msp_status_BF_t status_BF{};
		status_BF.task_delta_time = 1000;
		status_BF.sensor_status = MSP_STATUS_SENSOR_ACC | MSP_STATUS_SENSOR_BARO | MSP_STATUS_SENSOR_MAG;
		status_BF.flight_mode_flags = ARM_ACRO_BF;
		status_BF.pid_profile = 0;
		status_BF.system_load = 10;
		status_BF.gyro_cycle_time = 125;
		status_BF.arming_disable_flags_count = 1;
		status_BF.arming_disable_flags = 0;
		status_BF.extra_flags = 0;
		Send(MSP_STATUS, &status_BF, sizeof(msp_status_BF_t));
		break;
	}

	case MSP_STATUS_EX: {
		msp_status_ex_t status_ex{};
		status_ex.cycleTime = 1000;
		status_ex.sensor = MSP_STATUS_SENSOR_ACC | MSP_STATUS_SENSOR_BARO | MSP_STATUS_SENSOR_MAG;
		status_ex.flightModeFlags = ARM_ACRO_BF;
		status_ex.system_load = 10;
		status_ex.gyro_time = 125;
		Send(MSP_STATUS_EX, &status_ex, sizeof(status_ex));
		break;
	}

	default:
		break;
	}
}


