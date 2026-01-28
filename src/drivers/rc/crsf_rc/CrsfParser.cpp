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

/**
* @file CrsfParser.cpp
*
* Parser for incoming CRSF packets
*
* @author Chris Seto <chris1seto@gmail.com>
*/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "QueueBuffer.hpp"
#include "CrsfParser.hpp"
#include "Crc8.hpp"
#ifdef CONFIG_VTX_CRSF_MSP_SUPPORT
#include <px4_platform_common/param.h>
#endif

#define CRSF_CHANNEL_VALUE_MIN  172
#define CRSF_CHANNEL_VALUE_MAX  1811
#define CRSF_CHANNEL_VALUE_SPAN (CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MIN)
#define CRSF_MAX_PACKET_LEN 64
#define CRSF_HEADER 0xc8

enum CRSF_PAYLOAD_SIZE {
	CRSF_PAYLOAD_SIZE_GPS = 15,
	CRSF_PAYLOAD_SIZE_BATTERY = 8,
	CRSF_PAYLOAD_SIZE_LINK_STATISTICS = 10,
	CRSF_PAYLOAD_SIZE_LINK_STATISTICS_TX = -1,
	CRSF_PAYLOAD_SIZE_RC_CHANNELS = 22,
	CRSF_PAYLOAD_SIZE_ATTITUDE = 6,
	CRSF_PAYLOAD_SIZE_MSP_WRITE = -1, // -1 means variable length
	CRSF_PAYLOAD_SIZE_ELRS_STATUS = -1, // unclear how large this message is
};

enum CRSF_PACKET_TYPE {
	CRSF_PACKET_TYPE_GPS = 0x02,
	CRSF_PACKET_TYPE_BATTERY_SENSOR = 0x08,
	CRSF_PACKET_TYPE_LINK_STATISTICS = 0x14,
	CRSF_PACKET_TYPE_OPENTX_SYNC = 0x10,
	CRSF_PACKET_TYPE_RADIO_ID = 0x3A,
	CRSF_PACKET_TYPE_RC_CHANNELS_PACKED = 0x16,
	CRSF_PACKET_TYPE_LINK_STATISTICS_RX = 0x1C,
	CRSF_PACKET_TYPE_LINK_STATISTICS_TX = 0x1D,
	CRSF_PACKET_TYPE_ATTITUDE = 0x1E,
	CRSF_PACKET_TYPE_FLIGHT_MODE = 0x21,
	// Extended Header Frames, range: 0x28 to 0x96
	CRSF_PACKET_TYPE_DEVICE_PING = 0x28,
	CRSF_PACKET_TYPE_DEVICE_INFO = 0x29,
	CRSF_PACKET_TYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
	CRSF_PACKET_TYPE_PARAMETER_READ = 0x2C,
	CRSF_PACKET_TYPE_PARAMETER_WRITE = 0x2D,
	CRSF_PACKET_TYPE_ELRS_STATUS = 0x2E,
	CRSF_PACKET_TYPE_COMMAND = 0x32,
	// MSP commands
	CRSF_PACKET_TYPE_MSP_REQ = 0x7A,   // response request using msp sequence as command
	CRSF_PACKET_TYPE_MSP_RESP = 0x7B,  // reply with 58 byte chunked binary
	CRSF_PACKET_TYPE_MSP_WRITE = 0x7C, // write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit)
};

enum CRSF_ADDRESS {
	CRSF_ADDRESS_BROADCAST = 0x00,
	CRSF_ADDRESS_USB = 0x10,
	CRSF_ADDRESS_TBS_CORE_PNP_PRO = 0x80,
	CRSF_ADDRESS_RESERVED1 = 0x8A,
	CRSF_ADDRESS_CURRENT_SENSOR = 0xC0,
	CRSF_ADDRESS_GPS = 0xC2,
	CRSF_ADDRESS_TBS_BLACKBOX = 0xC4,
	CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8,
	CRSF_ADDRESS_RESERVED2 = 0xCA,
	CRSF_ADDRESS_RACE_TAG = 0xCC,
	CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA,
	CRSF_ADDRESS_CRSF_RECEIVER = 0xEC,
	CRSF_ADDRESS_CRSF_TRANSMITTER = 0xEE,
};

#define HEADER_SIZE           1
#define PACKET_SIZE_SIZE      1
#define PACKET_TYPE_SIZE      1
#define PACKET_SIZE_TYPE_SIZE 2
#define CRC_SIZE              1

enum PARSER_STATE {
	PARSER_STATE_HEADER,
	PARSER_STATE_SIZE_TYPE,
	PARSER_STATE_PAYLOAD,
	PARSER_STATE_CRC,
};

typedef struct {
	uint8_t packet_type;
	int32_t packet_size;
	bool (*processor)(const uint8_t *data, const uint32_t size, CrsfPacket_t *const new_packet);
} CrsfPacketDescriptor_t;

static bool ProcessChannelData(const uint8_t *data, const uint32_t size, CrsfPacket_t *const new_packet);
static bool ProcessLinkStatistics(const uint8_t *data, const uint32_t size, CrsfPacket_t *const new_packet);
static bool ProcessLinkStatisticsTx(const uint8_t *data, const uint32_t size, CrsfPacket_t *const new_packet);
static bool ProcessElrsStatus(const uint8_t *data, const uint32_t size, CrsfPacket_t *const new_packet);
#ifdef CONFIG_VTX_CRSF_MSP_SUPPORT
static bool ProcessMspWrite(const uint8_t *data, const uint32_t size, CrsfPacket_t *const new_packet);
#endif

static const CrsfPacketDescriptor_t crsf_packet_descriptors[] = {
	{CRSF_PACKET_TYPE_RC_CHANNELS_PACKED, CRSF_PAYLOAD_SIZE_RC_CHANNELS, ProcessChannelData},
	{CRSF_PACKET_TYPE_LINK_STATISTICS, CRSF_PAYLOAD_SIZE_LINK_STATISTICS, ProcessLinkStatistics},
	{CRSF_PACKET_TYPE_LINK_STATISTICS_TX, CRSF_PAYLOAD_SIZE_LINK_STATISTICS_TX, ProcessLinkStatisticsTx},
	{CRSF_PACKET_TYPE_ELRS_STATUS, CRSF_PAYLOAD_SIZE_ELRS_STATUS, ProcessElrsStatus},
#ifdef CONFIG_VTX_CRSF_MSP_SUPPORT
	{CRSF_PACKET_TYPE_MSP_WRITE, CRSF_PAYLOAD_SIZE_MSP_WRITE, ProcessMspWrite},
#endif
};
#define CRSF_PACKET_DESCRIPTOR_COUNT  (sizeof(crsf_packet_descriptors) / sizeof(CrsfPacketDescriptor_t))

static enum PARSER_STATE parser_state = PARSER_STATE_HEADER;
static uint32_t working_index = 0;
static uint32_t working_segment_size = HEADER_SIZE;

#define RX_QUEUE_BUFFER_SIZE 200
static QueueBuffer_t rx_queue;
static uint8_t rx_queue_buffer[RX_QUEUE_BUFFER_SIZE];
#ifdef CONFIG_RC_CRSF_INJECT
static QueueBuffer_t inject_queue;
static uint8_t inject_queue_buffer[RX_QUEUE_BUFFER_SIZE];
static uint8_t temp_queue_buffer[RX_QUEUE_BUFFER_SIZE];
#endif
static uint8_t process_buffer[CRSF_MAX_PACKET_LEN];
static CrsfPacketDescriptor_t *working_descriptor = NULL;

static CrsfPacketDescriptor_t *FindCrsfDescriptor(const enum CRSF_PACKET_TYPE packet_type);

void CrsfParser_Init(void)
{
	QueueBuffer_Init(&rx_queue, rx_queue_buffer, RX_QUEUE_BUFFER_SIZE);
#ifdef CONFIG_RC_CRSF_INJECT
	QueueBuffer_Init(&inject_queue, inject_queue_buffer, RX_QUEUE_BUFFER_SIZE);
#endif
}

static float ConstrainF(const float x, const float min, const float max)
{
	if (x < min) {
		return min;

	} else if (x > max) {
		return max;
	}

	return x;
}

static float MapF(const float x, const float in_min, const float in_max, const float out_min, const float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#define CONSTRAIN_CHAN(x) ConstrainF(x, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX)

static bool ProcessChannelData(const uint8_t *data, const uint32_t size, CrsfPacket_t *const new_packet)
{
	uint32_t raw_channels[CRSF_CHANNEL_COUNT];
	uint32_t i;

	new_packet->message_type = CRSF_MESSAGE_TYPE_RC_CHANNELS;

	// Decode channel data
	raw_channels[0] = CONSTRAIN_CHAN((data[0] | data[1] << 8) & 0x07FF);
	raw_channels[1] = CONSTRAIN_CHAN((data[1]  >> 3 | data[2] << 5) & 0x07FF);
	raw_channels[2] = CONSTRAIN_CHAN((data[2] >> 6 | data[3] << 2 | data[4] << 10) & 0x07FF);
	raw_channels[3] = CONSTRAIN_CHAN((data[4] >> 1 | data[5] << 7) & 0x07FF);
	raw_channels[4] = CONSTRAIN_CHAN((data[5] >> 4 | data[6] << 4) & 0x07FF);
	raw_channels[5] = CONSTRAIN_CHAN((data[6] >> 7 | data[7] << 1 | data[8] << 9) & 0x07FF);
	raw_channels[6] = CONSTRAIN_CHAN((data[8] >> 2 | data[9] << 6) & 0x07FF);
	raw_channels[7] = CONSTRAIN_CHAN((data[9] >> 5 | data[10] << 3) & 0x07FF);
	raw_channels[8] = CONSTRAIN_CHAN((data[11] | data[12] << 8) & 0x07FF);
	raw_channels[9] = CONSTRAIN_CHAN((data[12] >> 3 | data[13] << 5) & 0x07FF);
	raw_channels[10] = CONSTRAIN_CHAN((data[13] >> 6 | data[14] << 2 | data[15] << 10) & 0x07FF);
	raw_channels[11] = CONSTRAIN_CHAN((data[15] >> 1 | data[16] << 7) & 0x07FF);
	raw_channels[12] = CONSTRAIN_CHAN((data[16] >> 4 | data[17] << 4) & 0x07FF);
	raw_channels[13] = CONSTRAIN_CHAN((data[17] >> 7 | data[18] << 1 | data[19] << 9) & 0x07FF);
	raw_channels[14] = CONSTRAIN_CHAN((data[19] >> 2 | data[20] << 6) & 0x07FF);
	raw_channels[15] = CONSTRAIN_CHAN((data[20] >> 5 | data[21] << 3) & 0x07FF);

	for (i = 0; i < CRSF_CHANNEL_COUNT; i++) {
		new_packet->channel_data.channels[i] = MapF((float)raw_channels[i], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX,
						       1000.0f, 2000.0f);
	}

	return true;
}

static bool ProcessLinkStatistics(const uint8_t *data, const uint32_t size, CrsfPacket_t *const new_packet)
{
	new_packet->message_type = CRSF_MESSAGE_TYPE_LINK_STATISTICS;

	new_packet->link_statistics.uplink_rssi_1 = data[0];
	new_packet->link_statistics.uplink_rssi_2 = data[1];
	new_packet->link_statistics.uplink_link_quality = data[2];
	new_packet->link_statistics.uplink_snr = data[3];
	new_packet->link_statistics.active_antenna = data[4];
	new_packet->link_statistics.rf_mode = data[5];
	new_packet->link_statistics.uplink_tx_power = data[6];
	new_packet->link_statistics.downlink_rssi = data[7];
	new_packet->link_statistics.downlink_link_quality = data[8];
	new_packet->link_statistics.downlink_snr = data[9];

	return true;
}

static bool ProcessLinkStatisticsTx(const uint8_t *data, const uint32_t size, CrsfPacket_t *const new_packet)
{
	new_packet->message_type = CRSF_MESSAGE_TYPE_LINK_STATISTICS_TX;

	new_packet->link_statistics_tx.uplink_rssi = data[0];
	new_packet->link_statistics_tx.uplink_rssi_pct = data[1];
	new_packet->link_statistics_tx.uplink_link_quality = data[2];
	new_packet->link_statistics_tx.uplink_snr = data[3];
	new_packet->link_statistics_tx.downlink_power = data[4];
	new_packet->link_statistics_tx.uplink_fps = data[5];

	return true;
}

static bool ProcessElrsStatus(const uint8_t *data, const uint32_t size, CrsfPacket_t *const new_packet)
{
	new_packet->message_type = CRSF_MESSAGE_TYPE_ELRS_STATUS;

	// Try: crsf_rc inject 0x2E 0x13 0x50 0xFB 0x53 0x31 0x63 0x63 0x63 0x63

	new_packet->elrs_status.packets_bad = data[2];
	new_packet->elrs_status.packets_good = (data[3] << 8) | data[4];
	new_packet->elrs_status.flags = data[5];
	strlcpy(new_packet->elrs_status.message, (const char *)&data[6], sizeof(new_packet->elrs_status.message));

	return true;
}

#ifdef CONFIG_VTX_CRSF_MSP_SUPPORT
static bool ProcessMspWrite(const uint8_t *data, const uint32_t size, CrsfPacket_t *const)
{
	// Write the band/channel into the parameters, so it is thread-safe
	// data contains the following:
	// 0: CRSF v3: destination
	// 1: CRSF v3: origin
	// 2: CRSF v3: status
	// 3: MSP: size
	// 4: MSP: command
	// 5: MSP: data[≤57]
	// Try: crsf_rc inject 0x7C 0xC8 0xEA 0x30 0x4 0x59 0x22 0x0 0x1 0x0

	int32_t map_config{};
	param_get(int(px4::params::VTX_MAP_CONFIG), &map_config);

	if (map_config == 0) {
		// no mapping, just return
		return false;
	}

	if (data[2] == 0x30 && data[4] == 0x59) {
		const uint8_t length = data[3];

		if (map_config == 1 || map_config == 2) {
			// Status = bit4=new frame, bit5,6=MSPv1
			// MSP command 0x59 is MSP_SET_VTX_CONFIG
			uint32_t frequency = (data[6] << 8) | data[5];

			if (frequency <= 0x3f) {
				// first byte contains band and channel: 0b00bb'bccc
				const int32_t band = (data[5] >> 3) & 0x07;
				const int32_t channel = data[5] & 0x07;

				param_set_no_notification(int(px4::params::VTX_BAND), &band);
				param_set_no_notification(int(px4::params::VTX_CHANNEL), &channel);
				frequency = 0; // Disable the frequency override
			}

			param_set_no_notification(int(px4::params::VTX_FREQUENCY), &frequency);
		}

		if (length > 2 && (map_config == 1 || map_config == 3)) {
			const int32_t pit_mode = (data[8] || (data[7] == 0)) ? 1 : 0;
			param_set_no_notification(int(px4::params::VTX_PIT_MODE), &pit_mode);
			const int32_t power{pit_mode ? 0 : data[7] - 1};
			param_set_no_notification(int(px4::params::VTX_POWER), &power);
		}
	}

	// nothing else is implemented yet
	return false;
}
#endif

static CrsfPacketDescriptor_t *FindCrsfDescriptor(const enum CRSF_PACKET_TYPE packet_type)
{
	uint32_t i;

	for (i = 0; i < CRSF_PACKET_DESCRIPTOR_COUNT; i++) {
		if (crsf_packet_descriptors[i].packet_type == packet_type) {
			return (CrsfPacketDescriptor_t *)&crsf_packet_descriptors[i];
		}
	}

	return NULL;
}

bool CrsfParser_LoadBuffer(const uint8_t *buffer, const uint32_t size)
{
	return QueueBuffer_AppendBuffer(&rx_queue, buffer, size);
}

#ifdef CONFIG_RC_CRSF_INJECT
bool CrsfParser_InjectBuffer(const uint8_t *buffer, const uint32_t size)
{
	return QueueBuffer_AppendBuffer(&inject_queue, buffer, size);
}
#endif

uint32_t CrsfParser_FreeQueueSize(void)
{
	return RX_QUEUE_BUFFER_SIZE - QueueBuffer_Count(&rx_queue);
}

// 0xC8 [packet len] [packet type] [data] [crc]
bool CrsfParser_TryParseCrsfPacket(CrsfPacket_t *const new_packet, CrsfParserStatistics_t *const parser_statistics)
{
	uint32_t buffer_count;
	uint8_t working_byte;
	uint8_t packet_size;
	uint8_t packet_type;
	bool valid_packet = false;

	buffer_count = QueueBuffer_Count(&rx_queue);

	// Iterate through the buffer to parse the message out
	while ((working_index < buffer_count) && (buffer_count - working_index) >= working_segment_size) {
		switch (parser_state) {
		// Header
		case PARSER_STATE_HEADER:
			if (QueueBuffer_Get(&rx_queue, &working_byte)) {
				if (working_byte == CRSF_HEADER) {
					parser_state = PARSER_STATE_SIZE_TYPE;
					working_segment_size = PACKET_SIZE_TYPE_SIZE;
					working_index = 0;
					buffer_count = QueueBuffer_Count(&rx_queue);
					continue;

				} else {
					parser_statistics->disposed_bytes++;
				}
			}

			working_index = 0;
			working_segment_size = HEADER_SIZE;
			break;

		// Packet size type
		case PARSER_STATE_SIZE_TYPE:
			QueueBuffer_Peek(&rx_queue, working_index++, &packet_size);
			QueueBuffer_Peek(&rx_queue, working_index++, &packet_type);

			working_descriptor = FindCrsfDescriptor((enum CRSF_PACKET_TYPE)packet_type);

			// If we know what this packet is...
			if (working_descriptor != NULL) {
				// Validate length
				if (working_descriptor->packet_size == -1) {
					working_segment_size = packet_size - PACKET_SIZE_TYPE_SIZE;

				} else {
					if (packet_size != working_descriptor->packet_size + PACKET_SIZE_TYPE_SIZE) {
						parser_statistics->invalid_known_packet_sizes++;
						parser_state = PARSER_STATE_HEADER;
						working_segment_size = HEADER_SIZE;
						working_index = 0;
						buffer_count = QueueBuffer_Count(&rx_queue);
						continue;
					}

					working_segment_size = working_descriptor->packet_size;
				}

			} else {
				// We don't know what this packet is, so we'll let the parser continue
				// just so that we can dequeue it in one shot
				working_segment_size = packet_size - PACKET_SIZE_TYPE_SIZE;

				if (working_index + working_segment_size + CRC_SIZE > CRSF_MAX_PACKET_LEN) {
					parser_statistics->invalid_unknown_packet_sizes++;
					parser_state = PARSER_STATE_HEADER;
					working_segment_size = HEADER_SIZE;
					working_index = 0;
					buffer_count = QueueBuffer_Count(&rx_queue);
					continue;
				}
			}

			parser_state = PARSER_STATE_PAYLOAD;
			break;

		// Full packet content
		case PARSER_STATE_PAYLOAD:
			working_index += working_segment_size;
			working_segment_size = CRC_SIZE;
			parser_state = PARSER_STATE_CRC;
			break;

		// CRC
		case PARSER_STATE_CRC:
			// Fetch the suspected packet as a contingous block of memory
			QueueBuffer_PeekBuffer(&rx_queue, 0, process_buffer, working_index + CRC_SIZE);

			// Verify checksum
			if (Crc8Calc(process_buffer + PACKET_SIZE_SIZE, working_index - PACKET_SIZE_SIZE) == process_buffer[working_index]) {
				if (working_descriptor != NULL) {
					if (working_descriptor->processor != NULL) {
						if (working_descriptor->processor(process_buffer + PACKET_SIZE_TYPE_SIZE, working_index - PACKET_SIZE_TYPE_SIZE,
										  new_packet)) {
							parser_statistics->crcs_valid_known_packets++;
							valid_packet = true;
						}
					}

				} else {
					// No working_descriptor at this point means unknown packet
					parser_statistics->crcs_valid_unknown_packets++;
				}

				// Remove the sucessfully processed data from the queue
				QueueBuffer_Dequeue(&rx_queue, working_index + CRC_SIZE);

			} else {
				parser_statistics->crcs_invalid++;
			}

			working_index = 0;
			working_segment_size = HEADER_SIZE;
			parser_state = PARSER_STATE_HEADER;

			if (valid_packet) {
#ifdef CONFIG_RC_CRSF_INJECT

				if (!QueueBuffer_IsEmpty(&inject_queue)) {
					// copy the remaining bytes from the rx queue to the temp buffer
					const uint32_t temp_size = QueueBuffer_Count(&rx_queue);

					if (temp_size) {
						QueueBuffer_PeekBuffer(&rx_queue, 0, temp_queue_buffer, temp_size);
						// clear the rx queue
						QueueBuffer_Dequeue(&rx_queue, QueueBuffer_Count(&rx_queue));
					}

					// append the inject queue to the rx queue
					uint8_t inject_byte;

					while (QueueBuffer_Get(&inject_queue, &inject_byte)) {
						QueueBuffer_Append(&rx_queue, inject_byte);
					}

					if (temp_size) {
						// append the temp buffer back to the rx queue
						QueueBuffer_AppendBuffer(&rx_queue, temp_queue_buffer, temp_size);
					}

				} else {
					return true;
				}

#else
				return true;
#endif
			}

			break;
		}

		buffer_count = QueueBuffer_Count(&rx_queue);
	}

	return false;
}
