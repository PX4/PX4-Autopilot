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
 * @file spartn.h
 *
 * SPARTN transport-layer definitions used to locate frame boundaries.
 * Payload content is not decoded. Framing lives in CorrectionFramer.
 *
 * Frame layout (SPARTN ICD v2.x): preamble 0x73, then bit-packed TF002-TF018:
 *   TF002 message type      7 bits
 *   TF003 payload length   10 bits
 *   TF004 EAF               1 bit   (encryption and authentication flag)
 *   TF005 message CRC type  2 bits
 *   TF006 frame CRC         4 bits
 *   TF007..TF015                    (description block, 4-8 bytes)
 *   TF016 payload
 *   TF017 embedded authentication   (if EAF)
 *   TF018 message CRC       1-4 bytes
 *
 * Reference: https://www.spartnformat.org/
 */

#pragma once

#include <cstdint>
#include <cstddef>

namespace gnss
{

static constexpr uint8_t  SPARTN_PREAMBLE        = 0x73;
static constexpr size_t   SPARTN_FRAME_START_LEN = 4;
static constexpr size_t   SPARTN_MAX_PAYLOAD_LEN = 1023;
static constexpr size_t   SPARTN_MAX_AUTH_LEN    = 64;
static constexpr size_t   SPARTN_MAX_DESC_LEN    = 8;
static constexpr size_t   SPARTN_MAX_CRC_LEN     = 4;
static constexpr size_t   SPARTN_MAX_FRAME_LEN   =
	SPARTN_FRAME_START_LEN + SPARTN_MAX_DESC_LEN + SPARTN_MAX_PAYLOAD_LEN +
	SPARTN_MAX_AUTH_LEN + SPARTN_MAX_CRC_LEN;

/**
 * Whether TF002 names a message type the format defines.
 *
 * 0-4 are defined by SPARTN 2.x (OCB, HPAC, GAD, BPAC, EAS), 120-127 are
 * proprietary, and 5-119 are reserved. Reserved types are rejected: TF003 spans
 * its full range and TF006 is only 4 bits over a non-byte-aligned field, so this
 * is the sole header check standing between a stray preamble and the message CRC.
 */
inline bool spartn_message_type_defined(uint8_t message_type)
{
	return (message_type <= 4) || (message_type >= 120);
}

/**
 * Length of TF017 (embedded authentication).
 *
 * @param auth_ind     TF014, authentication indicator; TF017 is present only above 1
 * @param emb_auth_len TF015, 0=64b, 1=96b, 2=128b, 3=256b, 4=512b
 */
inline size_t spartn_auth_length_bytes(bool eaf, uint8_t auth_ind, uint8_t emb_auth_len)
{
	if (!eaf || auth_ind <= 1) {
		return 0;
	}

	static constexpr uint8_t kBytes[] = {8, 12, 16, 32, 64};
	return (emb_auth_len < 5) ? kBytes[emb_auth_len] : 64;
}

/**
 * Length of the description block (TF007..TF015), which follows the 4-byte
 * frame start and precedes the payload.
 */
inline size_t spartn_desc_length_bytes(bool eaf, bool time_tag_32bit)
{
	return (time_tag_32bit ? 6u : 4u) + (eaf ? 2u : 0u);
}

/**
 * Calculate the SPARTN message CRC (TF018) over TF002..TF017.
 *
 * @param data     Pointer to TF002 (i.e. the byte after the preamble)
 * @param len      Number of bytes covered
 * @param crc_type TF005: 0=CRC-8 poly 0x07, 1=CRC-16 poly 0x1021,
 *                 2=CRC-24 poly 0x864CFB, 3=CRC-32 poly 0x04C11DB7 init/xor ~0
 * @return         CRC value, in the low crc_type+1 bytes
 */
uint32_t spartn_message_crc(const uint8_t *data, size_t len, uint8_t crc_type);

} // namespace gnss
