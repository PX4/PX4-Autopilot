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
* @file CrsfParser.hpp
*
* Parser for incoming CRSF packets
*
* @author Chris Seto <chris1seto@gmail.com>
*/

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define CRSF_CHANNEL_COUNT 16

struct CrsfChannelData_t {
	float channels[CRSF_CHANNEL_COUNT];
};

struct CrsfLinkStatistics_t {
	uint8_t uplink_rssi_1;
	uint8_t uplink_rssi_2;
	uint8_t uplink_link_quality;
	int8_t uplink_snr;
	uint8_t active_antenna;
	uint8_t rf_mode;
	uint8_t uplink_tx_power;
	uint8_t downlink_rssi;
	uint8_t downlink_link_quality;
	int8_t downlink_snr;
};

struct CrsfParserStatistics_t {
	uint32_t disposed_bytes;
	uint32_t crcs_valid_known_packets;
	uint32_t crcs_valid_unknown_packets;
	uint32_t crcs_invalid;
	uint32_t invalid_known_packet_sizes;
	uint32_t invalid_unknown_packet_sizes;
};

enum CRSF_MESSAGE_TYPE {
	CRSF_MESSAGE_TYPE_RC_CHANNELS,
	CRSF_MESSAGE_TYPE_LINK_STATISTICS,
};

typedef struct {
	CRSF_MESSAGE_TYPE message_type;

	union {
		CrsfChannelData_t channel_data;
		CrsfLinkStatistics_t link_statistics;
	};
} CrsfPacket_t;

void CrsfParser_Init(void);
bool CrsfParser_LoadBuffer(const uint8_t *buffer, const uint32_t size);
uint32_t CrsfParser_FreeQueueSize(void);
bool CrsfParser_TryParseCrsfPacket(CrsfPacket_t *const new_packet, CrsfParserStatistics_t *const parser_statistics);
