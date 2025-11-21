/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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

#include "ADS1115.h"
#include <cassert>

int ADS1115::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		return ret;
	}

	readChannel(Channel::A0);  // prepare for the first measure.

	ScheduleOnInterval(SAMPLE_INTERVAL / 4, SAMPLE_INTERVAL / 4);

	return PX4_OK;
}

int ADS1115::probe()
{
	// The ADS1115 has no ID register, so we read out the threshold registers
	// and check their default values. We cannot use the config register, as
	// this is changed by this driver. Note the default value is in BE.
	static constexpr uint32_t DEFAULT{0xFF7F0080};
	union {
		struct {
			uint8_t low[2];
			uint8_t high[2];
		} parts;
		uint32_t threshold{};
	};
	int ret = readReg(ADDRESSPOINTER_REG_LO_THRESH, parts.low, 2);

	if (ret != PX4_OK) {
		DEVICE_DEBUG("lo_thresh read failed (%i)", ret);
		return ret;
	}

	ret = readReg(ADDRESSPOINTER_REG_HI_THRESH, parts.high, 2);

	if (ret != PX4_OK) {
		DEVICE_DEBUG("hi_thresh read failed (%i)", ret);
		return ret;
	}

	if (threshold == DEFAULT) {
		return PX4_OK;
	}

	DEVICE_DEBUG("ADS1115 not found");
	return PX4_ERROR;
}

int ADS1115::readChannel(ADS1115::Channel ch)
{
	uint8_t buf[2];
	buf[0] = CONFIG_HIGH_OS_START_SINGLE | uint8_t(ch) | CONFIG_HIGH_PGA_6144 | CONFIG_HIGH_MODE_SS;
	buf[1] = CONFIG_LOW_DR_250SPS | CONFIG_LOW_COMP_MODE_TRADITIONAL | CONFIG_LOW_COMP_POL_RESET |
		 CONFIG_LOW_COMP_LAT_NONE | CONFIG_LOW_COMP_QU_DISABLE;
	return writeReg(ADDRESSPOINTER_REG_CONFIG, buf, 2);    // must write whole register to take effect
}

int ADS1115::isSampleReady()
{
	uint8_t buf[1] = {0x00};

	if (readReg(ADDRESSPOINTER_REG_CONFIG, buf, 1) != PX4_OK) { return -1; } // Pull config register

	return (buf[0] & (uint8_t) 0x80) ? 1 : 0;
}

ADS1115::Channel ADS1115::getMeasurement(int16_t *value)
{
	uint8_t buf[2] = {0x00};

	if (readReg(ADDRESSPOINTER_REG_CONFIG, buf, 1) != PX4_OK) { return Channel::Invalid; }

	const auto channel{Channel(buf[0] & CONFIG_HIGH_MUX_P3NG)};

	if (readReg(ADDRESSPOINTER_REG_CONVERSATION, buf, 2) != PX4_OK) { return Channel::Invalid; }

	*value = int16_t((buf[0] << 8) | buf[1]);

	return channel;
}

int ADS1115::readReg(uint8_t addr, uint8_t *buf, size_t len)
{
	return transfer(&addr, 1, buf, len);
}

int ADS1115::writeReg(uint8_t addr, uint8_t *buf, size_t len)
{
	uint8_t buffer[len + 1];
	buffer[0] = addr;
	memcpy(buffer + 1, buf, sizeof(uint8_t)*len);
	return transfer(buffer, len + 1, nullptr, 0);
}
