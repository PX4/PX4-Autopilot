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
#include <syslog.h>

#include <sys/types.h>
#include <stdbool.h>
#include <float.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

#include <drivers/drv_pwm_output.h>
#include <drivers/drv_hrt.h>

#include "msp_defines.h"
#include "MspV1.hpp"

MspV1::MspV1(int fd) :
	_fd(fd)
{
}

int MspV1::GetMessageSize(int message_type)
{
	return 0;
}

struct msp_message_descriptor_t {
	uint8_t message_id;
	bool fixed_size;
	uint8_t message_size;
};

#define MSP_DESCRIPTOR_COUNT 12
const msp_message_descriptor_t msp_message_descriptors[MSP_DESCRIPTOR_COUNT] = {
	{MSP_OSD_CONFIG, true, sizeof(msp_osd_config_t)},
	{MSP_NAME, true, sizeof(msp_name_t)},
	{MSP_ANALOG, true, sizeof(msp_analog_t)},
	{MSP_STATUS, true, sizeof(msp_status_BF_t)},
	{MSP_BATTERY_STATE, true, sizeof(msp_battery_state_t)},
	{MSP_RAW_GPS, true, sizeof(msp_raw_gps_t)},
	{MSP_ATTITUDE, true, sizeof(msp_attitude_t)},
	{MSP_ALTITUDE, true, sizeof(msp_altitude_t)},
	{MSP_COMP_GPS, true, sizeof(msp_comp_gps_t)},
	{MSP_ESC_SENSOR_DATA, true, sizeof(msp_esc_sensor_data_dji_t)},
	{MSP_MOTOR_TELEMETRY, true, sizeof(msp_motor_telemetry_t)},
	{MSP_RC, true, sizeof(msp_rc_t)},
};

bool MspV1::Send(const uint8_t message_id, const void *payload)
{
	uint32_t payload_size = 0;

	msp_message_descriptor_t *desc = nullptr;

	for (int i = 0; i < MSP_DESCRIPTOR_COUNT; i++) {
		if (message_id == msp_message_descriptors[i].message_id) {
			desc = (msp_message_descriptor_t *)&msp_message_descriptors[i];
			break;
		}
	}

	if (!desc) {
		return false;
	}

	if (!desc->fixed_size) {
		return false;
	}

	payload_size = desc->message_size;

	uint8_t packet[MSP_FRAME_START_SIZE + payload_size + MSP_CRC_SIZE];
	uint8_t crc;

	packet[0] = '$';
	packet[1] = 'M';
	packet[2] = '>';
	packet[3] = payload_size;
	packet[4] = message_id;

	crc = payload_size ^ message_id;

	memcpy(packet + MSP_FRAME_START_SIZE, payload, payload_size);

	for (uint32_t i = 0; i < payload_size; i ++) {
		crc ^= packet[MSP_FRAME_START_SIZE + i];
	}

	packet[MSP_FRAME_START_SIZE + payload_size] = crc;

	int packet_size =  MSP_FRAME_START_SIZE + payload_size + MSP_CRC_SIZE;
	return  write(_fd, packet, packet_size) == packet_size;
}

bool MspV1::Send(const uint8_t message_id, const void *payload, uint32_t payload_size)
{
	uint8_t packet[MSP_FRAME_START_SIZE + payload_size + MSP_CRC_SIZE];
	uint8_t crc;

	packet[0] = '$';
	packet[1] = 'M';
	packet[2] = '>';
	packet[3] = payload_size;
	packet[4] = message_id;

	crc = payload_size ^ message_id;

	memcpy(packet + MSP_FRAME_START_SIZE, payload, payload_size);

	for (uint32_t i = 0; i < payload_size; i ++) {
		crc ^= packet[MSP_FRAME_START_SIZE + i];
	}

	packet[MSP_FRAME_START_SIZE + payload_size] = crc;

	int packet_size =  MSP_FRAME_START_SIZE + payload_size + MSP_CRC_SIZE;
	return  write(_fd, packet, packet_size) == packet_size;
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

	return payload_size;
}
