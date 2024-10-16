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

#include <drivers/drv_pwm_output.h>
#include <drivers/drv_hrt.h>

#include <drivers/osd/msp_osd/msp_defines.h>
#include "msp_dp_defines.h"
#include "MspDPV1.hpp"

#include <px4_platform_common/log.h>

MspDPV1::MspDPV1(int fd) :
	_fd(fd)
{
}

int MspDPV1::GetMessageSize(int message_type)
{
	return 0;
}

struct msp_message_descriptor_t {
	uint8_t message_id;
	bool fixed_size;
	uint8_t message_size;
};

// #define MSP_DESCRIPTOR_COUNT 11
#define MSP_DESCRIPTOR_COUNT 16
const msp_message_descriptor_t msp_message_descriptors[MSP_DESCRIPTOR_COUNT] = {
	{MSP_OSD_CONFIG, true, sizeof(msp_osd_config_t)},
	{MSP_NAME, true, sizeof(msp_name_t)},
	{MSP_ANALOG, true, sizeof(msp_analog_t)},
	{MSP_STATUS, true, sizeof(msp_dp_status_t)},
	{MSP_BATTERY_STATE, true, sizeof(msp_battery_state_t)},
	{MSP_RAW_GPS, true, sizeof(msp_raw_gps_t)},
	{MSP_ATTITUDE, true, sizeof(msp_attitude_t)},
	{MSP_ALTITUDE, true, sizeof(msp_altitude_t)},
	{MSP_COMP_GPS, true, sizeof(msp_comp_gps_t)},
	{MSP_ESC_SENSOR_DATA, true, sizeof(msp_esc_sensor_data_dji_t)},
	{MSP_MOTOR_TELEMETRY, true, sizeof(msp_motor_telemetry_t)},
	{MSP_RC, true, sizeof(msp_rc_t)},
	{MSP_SET_OSD_CANVAS, true, sizeof(msp_dp_canvas_t)},
	{MSP_FC_VARIANT, true, sizeof(msp_fc_variant_t)},
	{MSP_VTX_CONFIG, true, sizeof(msp_dp_vtx_config_t)},
	{MSP_CMD_DISPLAYPORT, false, sizeof(msp_dp_cmd_t)},
};

#define MSP_FRAME_START_SIZE 5
#define MSP_CRC_SIZE 1
bool MspDPV1::Send(const uint8_t message_id, const void *payload, mspDirection_e direction)
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

	// need to handle different size Displayport commands
	if (!desc->fixed_size) {
		if (desc->message_id ==  MSP_CMD_DISPLAYPORT){
			uint8_t subcmd[1]{0};
			memcpy(subcmd, payload, 1);
			if (subcmd[0] == MSP_DP_DRAW_SCREEN){
				payload_size = 1;
			} else if(subcmd[0] == MSP_DP_WRITE_STRING){	// Case when we write string.. payload size may vary 
				payload_size+=sizeof(msp_dp_cmd_t);
				char dp_payload[sizeof(msp_dp_cmd_t)+MSP_OSD_MAX_STRING_LENGTH];
				memcpy(dp_payload, payload, sizeof(dp_payload));
				// Find length of string in input (may not be whole array)
				for (int i=0;i<MSP_OSD_MAX_STRING_LENGTH;++i){
					if(dp_payload[MSP_OSD_DP_WRITE_PAYLOAD + i] == '\0') break;
					payload_size++;
				}
			} else {
				payload_size = desc->message_size;
			}
		} 
	} else {
		payload_size = desc->message_size;
	}

	uint8_t packet[MSP_FRAME_START_SIZE + payload_size + MSP_CRC_SIZE];
	uint8_t crc;

	packet[0] = MSP_HEADER;
	packet[1] = MSP_START;
	packet[2] = direction ? MSP_CMD : MSP_REPLY;	// HDZero VTX firmware only supports 'replies'...
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
