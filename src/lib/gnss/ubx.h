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
 * @file ubx.h
 *
 * UBX transport-layer definitions used to locate frame boundaries.
 * Payload content is not decoded. Framing lives in CorrectionFramer.
 *
 * Used for AssistNow MGA (UBX-MGA-*) delivered ahead of RTCM3/SPARTN
 * corrections on PointPerfect -MGA NTRIP mountpoints.
 *
 * Frame layout (u-blox UBX protocol):
 *   Byte 0:     Sync1 (0xB5)
 *   Byte 1:     Sync2 (0x62)
 *   Byte 2:     Class
 *   Byte 3:     ID
 *   Byte 4-5:   Payload length (uint16 little-endian)
 *   Byte 6..N:  Payload
 *   Last 2:     CK_A, CK_B (8-bit Fletcher over class..end of payload)
 */

#pragma once

#include <cstdint>
#include <cstddef>

namespace gnss
{

static constexpr uint8_t  UBX_SYNC1            = 0xB5;
static constexpr uint8_t  UBX_SYNC2            = 0x62;
static constexpr size_t   UBX_HEADER_LEN       = 6;   // sync1, sync2, class, id, len_lo, len_hi
static constexpr size_t   UBX_CK_LEN           = 2;
// Protocol allows up to 65535 payload bytes; cap for correction injection so the
// shared framer buffer stays comparable to RTCM3/SPARTN. AssistNow MGA ephemeris
// messages are well under this.
static constexpr size_t   UBX_MAX_PAYLOAD_LEN  = 1024;
static constexpr size_t   UBX_MAX_FRAME_LEN    = UBX_HEADER_LEN + UBX_MAX_PAYLOAD_LEN + UBX_CK_LEN;

/**
 * 8-bit Fletcher checksum used by UBX (CK_A, CK_B).
 *
 * Covers every byte from Class through the end of the payload (not the sync
 * bytes and not the two checksum bytes themselves).
 *
 * @param data Pointer to Class (first covered byte)
 * @param len  Number of covered bytes (class + id + length field + payload)
 * @param ck_a Out: CK_A
 * @param ck_b Out: CK_B
 */
inline void ubx_checksum(const uint8_t *data, size_t len, uint8_t *ck_a, uint8_t *ck_b)
{
	uint8_t a = 0;
	uint8_t b = 0;

	for (size_t i = 0; i < len; i++) {
		a = static_cast<uint8_t>(a + data[i]);
		b = static_cast<uint8_t>(b + a);
	}

	*ck_a = a;
	*ck_b = b;
}

} // namespace gnss
