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
 * @file spartn.cpp
 *
 * SPARTN transport-layer implementation.
 */

#include "spartn.h"

namespace gnss
{

uint32_t spartn_message_crc(const uint8_t *data, size_t len, uint8_t crc_type)
{
	// Use uint64_t so n==32 (CRC-32) does not shift a 32-bit value by 32.
	const unsigned n = 8u * (static_cast<unsigned>(crc_type) + 1u);
	const uint64_t poly = (crc_type == 0) ? 0x07ull :
			      (crc_type == 1) ? 0x1021ull :
			      (crc_type == 2) ? 0x864CFBull : 0x04C11DB7ull;
	const uint64_t top = 1ull << n;
	const uint64_t g = top | poly;
	uint64_t crc = (crc_type == 3) ? 0xFFFFFFFFull : 0ull;

	for (size_t i = 0; i < len; i++) {
		crc ^= static_cast<uint64_t>(data[i]) << (n - 8);

		for (int b = 0; b < 8; b++) {
			crc <<= 1;

			if (crc & top) {
				crc ^= g;
			}
		}
	}

	if (crc_type == 3) {
		crc ^= 0xFFFFFFFFull;
	}

	return static_cast<uint32_t>(crc & (top - 1ull));
}

} // namespace gnss
