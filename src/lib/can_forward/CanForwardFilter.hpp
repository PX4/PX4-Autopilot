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
 * @file CanForwardFilter.hpp
 *
 * DroneCAN data type filter for MAVLink CAN forwarding (CAN_FILTER_MODIFY).
 *
 * The filter holds DroneCAN data type IDs. A frame passes if its data type ID is in the set,
 * or if the set is empty. The type ID is taken from the CAN ID the same way ArduPilot does it,
 * so the filter lists that existing clients (dronecan_gui_tool, Mission Planner) send work unchanged.
 */

#pragma once

#include <stdint.h>

namespace can_forward
{

class CanForwardFilter
{
public:
	/// Matches the size of CAN_FILTER_MODIFY.ids
	static constexpr uint8_t MaxIdsPerMessage = 16;

	/// Total IDs held after any number of ADD operations
	static constexpr uint16_t Capacity = 64;

	/// CAN_FILTER_OP
	enum class Op : uint8_t {
		Replace = 0,
		Add = 1,
		Remove = 2,
	};

	enum class Result : uint8_t {
		Ok,
		TooManyIds,   ///< more than MaxIdsPerMessage in one message
		UnknownOp,
		Overflow,     ///< the result would exceed Capacity
	};

	/**
	 * Apply one CAN_FILTER_MODIFY operation.
	 *
	 * Either fully applied or, on any error, the set is left unchanged.
	 * REPLACE with zero IDs clears the filter (forward everything).
	 */
	Result apply(uint8_t op, const uint16_t *ids, uint8_t num_ids);

	/// true if a frame with this CAN ID (uavcan::CanFrame layout, EFF in bit 31) is forwarded
	bool matches(uint32_t can_id) const;

	void clear() { _size = 0; }
	bool empty() const { return _size == 0; }
	uint16_t size() const { return _size; }
	uint16_t at(uint16_t index) const { return _ids[index]; }

	/**
	 * DroneCAN data type ID of a frame as used by the filter.
	 *
	 * - anonymous message (source node 0): 0
	 * - service frame (bit 7 set): 8 bit service type ID (bits 16..23)
	 * - message frame: 16 bit message type ID (bits 8..23)
	 */
	static uint16_t dronecanTypeId(uint32_t can_id);

private:
	bool contains(uint16_t id) const;

	uint16_t _ids[Capacity] {};
	uint16_t _size{0};
};

} // namespace can_forward
