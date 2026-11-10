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
 * @file CanForward.hpp
 *
 * Helpers for DroneCAN over MAVLink. CAN IDs carry the extended frame flag in bit 31.
 */

#pragma once

#include <stdint.h>

namespace can_forward
{

static constexpr uint32_t CanFlagEff = 1U << 31;
static constexpr uint32_t CanFlagRtr = 1U << 30;
static constexpr uint32_t CanFlagErr = 1U << 29;
static constexpr uint32_t CanMaskStdId = 0x7FFU;
static constexpr uint32_t CanMaskExtId = 0x1FFFFFFFU;
static constexpr uint8_t CanMaxDataLen = 8;

/// classic CAN data frame with a valid ID
static inline bool isValidFrame(uint32_t id, uint8_t len)
{
	return len <= CanMaxDataLen
	       && (id & (CanFlagRtr | CanFlagErr)) == 0
	       && ((id & CanFlagEff) || (id & ~CanMaskStdId) == 0);
}

/// DroneCAN data type ID as used by CAN_FILTER_MODIFY (same as ArduPilot), 0 for anonymous frames
static inline uint16_t dronecanTypeId(uint32_t can_id)
{
	const uint32_t id = can_id & CanMaskExtId;

	if ((id & 0xFFU) == 0) {
		return 0;
	}

	if (id & 0x80U) {
		return static_cast<uint8_t>(id >> 16);
	}

	return static_cast<uint16_t>(id >> 8);
}

/// CAN_FILTER_MODIFY data type filter, empty forwards everything
class Filter
{
public:
	static constexpr uint8_t Capacity = 32;

	/// CAN_FILTER_OP
	enum Op : uint8_t {
		Replace = 0,
		Add = 1,
		Remove = 2,
	};

	bool apply(uint8_t op, const uint16_t *ids, uint8_t num_ids)
	{
		switch (op) {
		case Replace:
			_size = 0;
			return add(ids, num_ids);

		case Add:
			return add(ids, num_ids);

		case Remove:
			for (uint8_t i = 0; i < num_ids; i++) {
				for (uint8_t j = 0; j < _size; j++) {
					if (_ids[j] == ids[i]) {
						_ids[j] = _ids[--_size];
						break;
					}
				}
			}

			return true;

		default:
			return false;
		}
	}

	bool matches(uint32_t can_id) const
	{
		if (_size == 0) {
			return true;
		}

		return (can_id & CanFlagEff) && contains(dronecanTypeId(can_id));
	}

	void clear() { _size = 0; }
	uint8_t size() const { return _size; }

private:
	bool contains(uint16_t id) const
	{
		for (uint8_t i = 0; i < _size; i++) {
			if (_ids[i] == id) {
				return true;
			}
		}

		return false;
	}

	bool add(const uint16_t *ids, uint8_t num_ids)
	{
		for (uint8_t i = 0; i < num_ids; i++) {
			if (!contains(ids[i])) {
				if (_size >= Capacity) {
					return false;
				}

				_ids[_size++] = ids[i];
			}
		}

		return true;
	}

	uint16_t _ids[Capacity] {};
	uint8_t _size{0};
};

} // namespace can_forward
