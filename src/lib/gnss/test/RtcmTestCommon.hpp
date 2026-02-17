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

#pragma once

#include <gtest/gtest.h>
#include "rtcm.h"
#include <vector>
#include <cstring>
#include <random>

using namespace gnss;

class RtcmTest : public ::testing::Test
{
protected:
	Rtcm3Parser parser;

	// Helper to build a valid RTCM3 frame with proper CRC
	std::vector<uint8_t> buildValidFrame(uint16_t msg_type, const std::vector<uint8_t> &payload_data)
	{
		// Payload = 12-bit message type + rest of payload_data
		// Message type is stored in first 12 bits of payload
		std::vector<uint8_t> payload;
		payload.push_back((msg_type >> 4) & 0xFF);
		payload.push_back(((msg_type & 0x0F) << 4) | (payload_data.empty() ? 0 : (payload_data[0] >> 4)));

		for (size_t i = 0; i < payload_data.size(); i++) {
			uint8_t current = (payload_data[i] << 4) & 0xF0;

			if (i + 1 < payload_data.size()) {
				current |= (payload_data[i + 1] >> 4) & 0x0F;
			}

			payload.push_back(current);
		}

		return buildRawFrame(payload);
	}

	// Helper to build a frame with raw payload bytes (no message type encoding)
	std::vector<uint8_t> buildRawFrame(const std::vector<uint8_t> &payload)
	{
		std::vector<uint8_t> frame;
		size_t payload_len = payload.size();

		// Header: preamble + length
		frame.push_back(RTCM3_PREAMBLE);
		frame.push_back((payload_len >> 8) & 0x03);  // Upper 2 bits of length (reserved bits = 0)
		frame.push_back(payload_len & 0xFF);         // Lower 8 bits of length

		// Payload
		frame.insert(frame.end(), payload.begin(), payload.end());

		// CRC
		uint32_t crc = rtcm3_crc24q(frame.data(), frame.size());
		frame.push_back((crc >> 16) & 0xFF);
		frame.push_back((crc >> 8) & 0xFF);
		frame.push_back(crc & 0xFF);

		return frame;
	}

	// Helper to build a frame with bad CRC
	std::vector<uint8_t> buildBadCrcFrame(const std::vector<uint8_t> &payload)
	{
		auto frame = buildRawFrame(payload);
		// Corrupt the CRC
		frame[frame.size() - 1] ^= 0xFF;
		return frame;
	}
};
