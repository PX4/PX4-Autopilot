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
 * @file ByteCursor.hpp
 * @brief Bounds-checked byte access and raw integer helpers for IMU wire decoders.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace imu
{

constexpr int kHz = 1000;
constexpr int MHz = 1000000;

/** Byte order within one wire value or a single 16-bit word. */
enum class ByteOrder : uint8_t {
	kBigEndian,
	kLittleEndian,
};

/** Order of the two 16-bit words forming a 32-bit channel. */
enum class WordOrder : uint8_t {
	kLowFirst,
	kHighFirst,
};

/** Bounds-checked view of bytes actually received, not the transfer capacity. */
class ByteCursor
{
public:
	ByteCursor(const uint8_t *data, size_t size) : _data(data), _remaining(data ? size : 0) {}

	size_t remaining() const { return _remaining; }

	const uint8_t *take(size_t count)
	{
		if (count > _remaining) {
			return nullptr;
		}

		const uint8_t *result = _data;

		if (count != 0) {
			_data += count;
			_remaining -= count;
		}

		return result;
	}

private:
	const uint8_t *_data;
	size_t        _remaining;
};

constexpr int16_t readBe16(const uint8_t *data)
{
	return static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
}

constexpr int16_t readLe16(const uint8_t *data)
{
	return static_cast<int16_t>((static_cast<uint16_t>(data[1]) << 8) | data[0]);
}

template<ByteOrder order, WordOrder words>
inline int32_t readWordPair32(const uint8_t *data)
{
	static_assert(order == ByteOrder::kBigEndian || order == ByteOrder::kLittleEndian, "Invalid byte order");
	static_assert(words == WordOrder::kLowFirst || words == WordOrder::kHighFirst, "Invalid word order");

	constexpr size_t low  = words == WordOrder::kLowFirst ? 0 : 2;
	constexpr size_t high = words == WordOrder::kLowFirst ? 2 : 0;

	const auto word = [](const uint8_t *bytes) {
		return static_cast<uint16_t>(order == ByteOrder::kBigEndian ? readBe16(bytes) : readLe16(bytes));
	};

	const uint32_t bits = (static_cast<uint32_t>(word(data + high)) << 16) | word(data + low);

	int32_t value = 0;
	memcpy(&value, &bits, sizeof(value));

	return value;
}

constexpr int16_t negateSaturated(int16_t value)
{
	return value == INT16_MIN ? INT16_MAX : static_cast<int16_t>(-value);
}

inline uint16_t byteSum16(const uint8_t *data, size_t size)
{
	uint16_t sum = 0;

	if (data) {
		for (size_t i = 0; i < size; ++i) {
			sum += data[i];
		}
	}

	return sum;
}

template<unsigned bytes, ByteOrder order>
constexpr uint32_t readUnsigned(const uint8_t *data)
{
	static_assert(bytes > 0 && bytes <= 4, "Invalid wire field size");
	static_assert(order == ByteOrder::kBigEndian || order == ByteOrder::kLittleEndian, "Invalid byte order");

	uint32_t value = 0;

	for (unsigned i = 0; i < bytes; ++i) {
		const unsigned offset = order == ByteOrder::kBigEndian ? bytes - i - 1 : i;
		value |= static_cast<uint32_t>(data[i]) << (8 * offset);
	}

	return value;
}

} // namespace imu
