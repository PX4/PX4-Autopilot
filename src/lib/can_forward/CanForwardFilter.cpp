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

#include "CanForwardFilter.hpp"
#include "CanFrameCodec.hpp"

namespace can_forward
{

// sorts a small array in place and removes duplicates, returns the new size
static uint8_t sortUnique(uint16_t *ids, uint8_t n)
{
	for (uint8_t i = 1; i < n; i++) {
		const uint16_t v = ids[i];
		int j = i - 1;

		while (j >= 0 && ids[j] > v) {
			ids[j + 1] = ids[j];
			j--;
		}

		ids[j + 1] = v;
	}

	uint8_t out = 0;

	for (uint8_t i = 0; i < n; i++) {
		if (out == 0 || ids[out - 1] != ids[i]) {
			ids[out++] = ids[i];
		}
	}

	return out;
}

uint16_t CanForwardFilter::dronecanTypeId(uint32_t can_id)
{
	const uint32_t id = can_id & CanMaskExtId;

	if ((id & 0xFFU) == 0) {
		// anonymous frame: source node ID 0 and not a service
		return 0;
	}

	if (id & 0x80U) {
		return static_cast<uint8_t>(id >> 16);
	}

	return static_cast<uint16_t>(id >> 8);
}

bool CanForwardFilter::contains(uint16_t id) const
{
	int lo = 0;
	int hi = static_cast<int>(_size) - 1;

	while (lo <= hi) {
		const int mid = (lo + hi) / 2;

		if (_ids[mid] == id) {
			return true;

		} else if (_ids[mid] < id) {
			lo = mid + 1;

		} else {
			hi = mid - 1;
		}
	}

	return false;
}

bool CanForwardFilter::matches(uint32_t can_id) const
{
	if (_size == 0) {
		return true;
	}

	// DroneCAN uses extended frames only, a standard frame has no data type ID to match
	if ((can_id & CanFlagEff) == 0) {
		return false;
	}

	return contains(dronecanTypeId(can_id));
}

CanForwardFilter::Result CanForwardFilter::apply(uint8_t op, const uint16_t *ids, uint8_t num_ids)
{
	if (num_ids > MaxIdsPerMessage) {
		return Result::TooManyIds;
	}

	if (op > static_cast<uint8_t>(Op::Remove)) {
		return Result::UnknownOp;
	}

	uint16_t in[MaxIdsPerMessage];

	for (uint8_t i = 0; i < num_ids; i++) {
		in[i] = ids[i];
	}

	const uint8_t n = sortUnique(in, num_ids);

	switch (static_cast<Op>(op)) {
	case Op::Replace:
		for (uint8_t i = 0; i < n; i++) {
			_ids[i] = in[i];
		}

		_size = n;
		return Result::Ok;

	case Op::Add: {
			uint16_t added = 0;

			for (uint8_t i = 0; i < n; i++) {
				if (!contains(in[i])) {
					added++;
				}
			}

			if (_size + added > Capacity) {
				return Result::Overflow;
			}

			// merge from the back, both inputs are sorted and unique
			int a = static_cast<int>(_size) - 1;
			int b = static_cast<int>(n) - 1;
			int w = static_cast<int>(_size + added) - 1;

			while (b >= 0) {
				if (a >= 0 && _ids[a] > in[b]) {
					_ids[w--] = _ids[a--];

				} else if (a >= 0 && _ids[a] == in[b]) {
					_ids[w--] = _ids[a--];
					b--;

				} else {
					_ids[w--] = in[b--];
				}
			}

			_size += added;
			return Result::Ok;
		}

	case Op::Remove: {
			uint16_t out = 0;

			for (uint16_t i = 0; i < _size; i++) {
				bool remove = false;

				for (uint8_t j = 0; j < n; j++) {
					if (in[j] == _ids[i]) {
						remove = true;
						break;
					}
				}

				if (!remove) {
					_ids[out++] = _ids[i];
				}
			}

			_size = out;
			return Result::Ok;
		}
	}

	return Result::UnknownOp;
}

} // namespace can_forward
