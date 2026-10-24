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

#include "dshot_telemetry.h"

bool dshot_decode_gcr_payload(uint32_t value, uint16_t *payload)
{
	static const uint8_t gcr_decode[32] = {
		0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10,
		0x10, 0x9, 0xA, 0xB, 0x10, 0xD, 0xE, 0xF,
		0x10, 0x10, 0x2, 0x3, 0x10, 0x5, 0x6, 0x7,
		0x10, 0x0, 0x8, 0x1, 0x10, 0x4, 0xC, 0x10
	};

	const uint8_t nibble0 = gcr_decode[value & 0x1fU];
	const uint8_t nibble1 = gcr_decode[(value >> 5U) & 0x1fU];
	const uint8_t nibble2 = gcr_decode[(value >> 10U) & 0x1fU];
	const uint8_t nibble3 = gcr_decode[(value >> 15U) & 0x1fU];

	if ((nibble0 | nibble1 | nibble2 | nibble3) & 0x10U) {
		return false;
	}

	uint32_t data = nibble0;
	data |= (uint32_t)nibble1 << 4U;
	data |= (uint32_t)nibble2 << 8U;
	data |= (uint32_t)nibble3 << 12U;

	uint32_t checksum = data ^ (data >> 8U);
	checksum ^= checksum >> 4U;

	if ((checksum & 0xfU) != 0xfU) {
		return false;
	}

	*payload = (data >> 4U) & 0xfffU;
	return true;
}
