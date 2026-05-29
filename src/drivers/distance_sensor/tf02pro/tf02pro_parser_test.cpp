/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file tf02pro_parser_test.cpp
 *
 * Standalone unit test for the TF02 Pro parser state machine.
 */

#include <iostream>
#include <cassert>
#include <cmath>
#include "tf02pro_parser.h"

// Helper to reset parsing state
void reset_parser(TF02PRO_PARSE_STATE &state, unsigned &buf_idx)
{
	state = TF02PRO_PARSE_STATE::STATE0_UNSYNC;
	buf_idx = 0;
}

int main()
{
	std::cout << "[INFO] Starting TF02 Pro Parser Unit Tests..." << std::endl;

	TF02PRO_PARSE_STATE state = TF02PRO_PARSE_STATE::STATE0_UNSYNC;
	uint8_t buf[9]{};
	unsigned buf_idx = 0;
	float distance = -1.0f;

	// ==========================================
	// Test Case 1: Valid frame parsing
	// Frame: 0x59 0x59 Dist_L Dist_H Str_L Str_H Temp_L Temp_H Checksum
	// Dist = 480 cm (4.8 m) (0x01E0) -> Dist_L = 0xE0, Dist_H = 0x01
	// Strength = 80 (0x0050) -> Str_L = 0x50, Str_H = 0x00
	// Temp = 2835 (0x0B1A)   -> Temp_L = 0x1A, Temp_H = 0x0B
	// Sum = 0x59 + 0x59 + 0xE0 + 0x01 + 0x50 + 0x00 + 0x1A + 0x0B = 520 (0x208)
	// Checksum (lower 8 bits) = 0x08
	// ==========================================
	{
		reset_parser(state, buf_idx);
		uint8_t valid_frame[9] = {0x59, 0x59, 0xE0, 0x01, 0x50, 0x00, 0x1A, 0x0B, 0x08};
		int result = -1;

		for (int i = 0; i < 9; i++) {
			result = tf02pro_parse(valid_frame[i], buf, &buf_idx, &state, &distance);
		}

		assert(result == 0);
		assert(std::fabs(distance - 4.8f) < 1e-4);
		assert(state == TF02PRO_PARSE_STATE::STATE9_GOT_CHECKSUM);
		std::cout << "[PASS] Test 1: Valid frame parsed correctly (Distance: " << distance << " m)" << std::endl;
	}

	// ==========================================
	// Test Case 2: Checksum failure rejection
	// Correct checksum is 0x08, we feed 0x09
	// ==========================================
	{
		reset_parser(state, buf_idx);
		uint8_t bad_cksm_frame[9] = {0x59, 0x59, 0xE0, 0x01, 0x50, 0x00, 0x1A, 0x0B, 0x09};
		int result = -1;

		for (int i = 0; i < 9; i++) {
			result = tf02pro_parse(bad_cksm_frame[i], buf, &buf_idx, &state, &distance);
		}

		assert(result == -1);
		assert(state == TF02PRO_PARSE_STATE::STATE0_UNSYNC); // should fall back to unsync state
		std::cout << "[PASS] Test 2: Invalid checksum rejected successfully" << std::endl;
	}

	// ==========================================
	// Test Case 3: Low signal strength filter (< 60)
	// Strength = 50 (0x0032) -> Str_L = 0x32, Str_H = 0x00
	// Sum = 0x59 + 0x59 + 0xE0 + 0x01 + 0x32 + 0x00 + 0x1A + 0x0B = 490 (0x1EA)
	// Checksum = 0xEA
	// ==========================================
	{
		reset_parser(state, buf_idx);
		uint8_t weak_frame[9] = {0x59, 0x59, 0xE0, 0x01, 0x32, 0x00, 0x1A, 0x0B, 0xEA};
		int result = -1;

		std::cout << "[DEBUG] Test 3 byte-by-byte states:" << std::endl;
		for (int i = 0; i < 9; i++) {
			result = tf02pro_parse(weak_frame[i], buf, &buf_idx, &state, &distance);
			std::cout << "  Byte " << i << " (" << std::hex << (int)weak_frame[i] 
			          << "): state=" << std::dec << (int)state 
			          << ", buf_idx=" << buf_idx << std::endl;
		}

		std::cout << "  Result: " << result << std::endl;
		std::cout << "  Buffer content: ";
		for (int i = 0; i < 9; i++) std::cout << std::hex << (int)buf[i] << " ";
		std::cout << std::dec << std::endl;

		assert(result == -1); // Checksum matches, but strength < 60 -> filtered
		assert(state == TF02PRO_PARSE_STATE::STATE9_GOT_CHECKSUM); // State goes to STATE9 (valid frame format)
		std::cout << "[PASS] Test 3: Low signal strength frame filtered out" << std::endl;
	}

	// ==========================================
	// Test Case 4: Extreme distance filter (>= 4500 cm)
	// Dist = 46000 (0xB3B0)
	// Sum = 0x59 + 0x59 + 0xB0 + 0xB3 + 0x50 + 0x00 + 0x1A + 0x0B = 734 (0x2DE)
	// Checksum = 0xDE
	// ==========================================
	{
		reset_parser(state, buf_idx);
		uint8_t huge_dist_frame[9] = {0x59, 0x59, 0xB0, 0xB3, 0x50, 0x00, 0x1A, 0x0B, 0x8A};
		int result = -1;

		for (int i = 0; i < 9; i++) {
			result = tf02pro_parse(huge_dist_frame[i], buf, &buf_idx, &state, &distance);
		}

		assert(result == -1); // Valid checksum, strength, but distance too large -> filtered
		assert(state == TF02PRO_PARSE_STATE::STATE9_GOT_CHECKSUM);
		std::cout << "[PASS] Test 4: Out-of-bounds distance frame filtered out" << std::endl;
	}

	// ==========================================
	// Test Case 5: Stream shift & re-sync recovery
	// Feeding garbage, then partial headers, then valid frame
	// ==========================================
	{
		reset_parser(state, buf_idx);
		uint8_t stream[] = {
			0x12, 0x34, 0x59, 0x78, // garbage containing partial header sync byte (0x59)
			0x59, 0x59, 0xE0, 0x01, 0x50, 0x00, 0x1A, 0x0B, 0x08 // valid frame
		};
		int result = -1;

		for (unsigned i = 0; i < sizeof(stream); i++) {
			result = tf02pro_parse(stream[i], buf, &buf_idx, &state, &distance);
		}

		assert(result == 0);
		assert(std::fabs(distance - 4.8f) < 1e-4);
		assert(state == TF02PRO_PARSE_STATE::STATE9_GOT_CHECKSUM);
		std::cout << "[PASS] Test 5: Re-synchronized and recovered from stream offsets" << std::endl;
	}

	std::cout << "[SUCCESS] All TF02 Pro Parser tests passed successfully!" << std::endl;
	return 0;
}
