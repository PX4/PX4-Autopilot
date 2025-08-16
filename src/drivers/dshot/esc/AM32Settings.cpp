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

param_t AM32Settings::_param_pole_count = {PARAM_INVALID};
param_t AM32Settings::_param_beep_volum = {PARAM_INVALID};

void AM32Settings::initParams(uint8_t motor_index)
{
	// Mark all params as used
	_param_pole_count = param_find("AM32_POLE_COUNT");
	_param_beep_volum = param_find("AM32_BEEP_VOLUM");
}

int AM32Settings::getExpectedResponseSize()
{
	return RESPONSE_SIZE;
}

bool AM32Settings::decodeInfoResponse(const uint8_t *buf, int size)
{
	if (size != RESPONSE_SIZE) {
		return false;
	}

	uint8_t checksum = crc8(buf, EEPROM_SIZE);
	uint8_t checksum_data = buf[EEPROM_SIZE];

	if (checksum == checksum_data) {

		DSHOT_CMD_DEBUG("Successfully received AM32 settings!");
		// auto now  = hrt_absolute_time();

		// We need to use
		// - _command_response_motor_index;

		// TODO
		// Iterate over each setting and write to our parameters

		// for (int j = 0; j < EEPROM_SIZE; j++) {
		// 	DSHOT_CMD_DEBUG("%d", buf[j]);
		// }

	} else {
		PX4_WARN("Command Response checksum failed!");
	}

	return true;
}
