/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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
 * @file CanFrameCodec.hpp
 *
 * Validation of CAN frames received in MAVLink CAN_FRAME / CANFD_FRAME messages.
 *
 * The MAVLink id field uses the same layout as uavcan::CanFrame::id: bit 31 marks an extended
 * frame, the remaining bits hold the 29 bit (or 11 bit) identifier. len is a byte count.
 * Only classic CAN is supported (len <= 8), the DroneCAN stack in PX4 has no CAN FD support.
 */

#pragma once

#include <stdint.h>
#include <string.h>

namespace can_forward
{

static constexpr uint32_t CanFlagEff = 1U << 31;
static constexpr uint32_t CanFlagRtr = 1U << 30;
static constexpr uint32_t CanFlagErr = 1U << 29;
static constexpr uint32_t CanMaskStdId = 0x7FFU;
static constexpr uint32_t CanMaskExtId = 0x1FFFFFFFU;
static constexpr uint8_t CanMaxDataLen = 8;

struct Frame {
	uint32_t id;                 ///< CAN ID, EFF flag in bit 31
	uint8_t len;
	uint8_t data[CanMaxDataLen]; ///< bytes past len are zero
};

enum class DecodeResult : uint8_t {
	Ok,
	BadLength, ///< more than 8 bytes
	BadId,     ///< standard ID above 11 bits
	BadFlags,  ///< RTR or error frame
};

/**
 * Validate a frame from the GCS and copy it.
 *
 * Remote and error frames are refused: DroneCAN never uses them and a GCS must not be able to
 * put error frames on the bus.
 */
static inline DecodeResult decodeFrame(uint32_t id, uint8_t len, const uint8_t *data, uint8_t data_size, Frame &out)
{
	if (len > CanMaxDataLen || len > data_size) {
		return DecodeResult::BadLength;
	}

	if (id & (CanFlagRtr | CanFlagErr)) {
		return DecodeResult::BadFlags;
	}

	if (!(id & CanFlagEff) && (id & ~CanMaskStdId)) {
		return DecodeResult::BadId;
	}

	out.id = id;
	out.len = len;
	memset(out.data, 0, sizeof(out.data));
	memcpy(out.data, data, len);
	return DecodeResult::Ok;
}

} // namespace can_forward
