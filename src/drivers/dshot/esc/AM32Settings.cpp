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

#include "AM32Settings.h"
#include "../DShotCommon.h"
#include <px4_platform_common/log.h>

static constexpr int EEPROM_SIZE = 48;  // AM32 sends raw eeprom data
static constexpr int RESPONSE_SIZE = 49; // 48B data + 1B CRC

uORB::Publication<am32_eeprom_read_s> AM32Settings::_am32_eeprom_read_pub{ORB_ID(am32_eeprom_read)};

AM32Settings::AM32Settings(int index)
	: _esc_index(index)
{}

int AM32Settings::getExpectedResponseSize()
{
	return RESPONSE_SIZE;
}

void AM32Settings::publish_latest()
{
	// PX4_INFO("publish_latest()");
	am32_eeprom_read_s data = {};
	data.timestamp = hrt_absolute_time();
	data.index = _esc_index;
	memcpy(data.data, &_eeprom_data, sizeof(data.data));
	_am32_eeprom_read_pub.publish(data);
}

bool AM32Settings::decodeInfoResponse(const uint8_t *buf, int size)
{
	if (size != RESPONSE_SIZE) {
		return false;
	}

	uint8_t checksum = crc8(buf, EEPROM_SIZE);
	uint8_t checksum_data = buf[EEPROM_SIZE];

	if (checksum != checksum_data) {
		PX4_WARN("Command Response checksum failed!");
		return false;
	}

	PX4_INFO("Successfully received AM32 settings from ESC%d", _esc_index + 1);

	// Store data for retrieval later if requested
	memcpy(&_eeprom_data, buf, EEPROM_SIZE);

	// Publish data immedietly
	publish_latest();

	return true;
}
