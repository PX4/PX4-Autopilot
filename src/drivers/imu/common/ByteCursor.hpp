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
 * @file
 * @brief Bounds-checked byte access and raw integer helpers for family-owned wire decoders.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace imu
{

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
	/**
	 * @brief Borrow the bytes received from a device without allocating or copying.
	 * @param[in] data Storage that outlives the cursor; null creates an empty view.
	 * @param[in] size Number of readable bytes, not transfer capacity.
	 */
	ByteCursor(const uint8_t *data, size_t size) : _data(data), _remaining(data ? size : 0) {}

	/** @return Number of unread bytes. */
	size_t remaining() const { return _remaining; }

	/**
	 * @brief Consume a bounded contiguous byte range.
	 * @param[in] count Requested number of bytes; zero leaves the cursor unchanged.
	 * @return Range start, or nullptr if it does not fit (cursor unchanged) or the underlying view is null.
	 * @note The returned range borrows the original storage; only count bytes may be read.
	 */
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

/**
 * @brief Decode one big-endian signed word, including potentially unaligned wire data.
 * @param[in] data At least two readable bytes; the caller owns the bounds check.
 * @return Signed raw count.
 */
constexpr int16_t readBe16(const uint8_t *data)
{
	return static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
}

/**
 * @brief Decode one little-endian signed word, including potentially unaligned wire data.
 * @param[in] data At least two readable bytes; the caller owns the bounds check.
 * @return Signed raw count.
 */
constexpr int16_t readLe16(const uint8_t *data)
{
	return static_cast<int16_t>((static_cast<uint16_t>(data[1]) << 8) | data[0]);
}

/**
 * @brief Decode a signed 32-bit channel from two explicitly ordered 16-bit words.
 * @tparam order Byte order within each word.
 * @tparam words Low-word-first or high-word-first transmission.
 * @param[in] data Four readable bytes; bounds are the caller's responsibility.
 * @return Signed raw count, retaining the low word without alignment assumptions.
 */
template<ByteOrder order, WordOrder words>
inline int32_t readWordPair32(const uint8_t *data)
{
	static_assert(
		order == ByteOrder::kBigEndian
		|| order == ByteOrder::kLittleEndian,
		"Invalid byte order");
	static_assert(
		words == WordOrder::kLowFirst
		|| words == WordOrder::kHighFirst,
		"Invalid word order");

	// Word order selects the offsets; byte order controls decoding within each word.
	constexpr size_t low  = words == WordOrder::kLowFirst ? 0 : 2;
	constexpr size_t high = words == WordOrder::kLowFirst ? 2 : 0;

	const auto word = [](const uint8_t *bytes) {
		return static_cast<uint16_t>(order == ByteOrder::kBigEndian ? readBe16(bytes) : readLe16(bytes));
	};

	const uint32_t bits = static_cast<uint32_t>(word(data + high)) << 16 | word(data + low);

	// Preserve the assembled bit pattern when interpreting the result as a signed count.
	int32_t value;

	memcpy(&value, &bits, sizeof(value));

	return value;
}

namespace detail
{
/// @cond INTERNAL
// Only storage support and accumulator selection are family-independent policy.
// Use the standard INT*_MIN/MAX macros directly, without duplicating numeric limits.
template<typename Raw>
struct RawTraits {
	static constexpr bool kSupported { false };
};

template<>
struct RawTraits<int16_t> {
	using Accumulator = int32_t;

	static constexpr bool kSupported { true };
};

template<>
struct RawTraits<int32_t> {
	using Accumulator = int64_t;

	static constexpr bool kSupported { true };
};
/// @endcond

} // namespace detail

/**
 * @brief Negate a raw signed count without signed overflow.
 * @tparam Raw int16_t or int32_t storage, independent of the effective sensor resolution.
 * @param[in] value Raw count.
 * @return Negated count; the storage minimum saturates to its maximum.
 * @note Saturation is an explicit frame-conversion policy, not lossless negation.
 */
template<typename Raw>
constexpr Raw negateSaturated(Raw value)
{
	static_assert(detail::RawTraits<Raw>::kSupported, "Raw counts require signed 16/32-bit storage");

	if constexpr(sizeof(Raw) == sizeof(int16_t)) {
		return value == INT16_MIN ? INT16_MAX : -value;

	} else {
		return value == INT32_MIN ? INT32_MAX : -value;
	}
}

/**
 * @brief Sum unsigned bytes with modulo-65536 arithmetic.
 * @param[in] data Readable byte range; null returns zero.
 * @param[in] size Range length in bytes.
 * @return Additive byte checksum, without complement or protocol framing.
 */
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

/**
 * @brief Sign-extend an assembled two's-complement field into int32_t.
 * @tparam valid_bits Number of valid low bits (1..32); upper bits are ignored.
 * @param[in] value Unsigned assembled field, after family-specific unpacking.
 * @return Signed raw count without narrowing or floating-point conversion.
 * @note A 32-bit field is handled without a shift by 32 or signed overflow.
 * This helper does not decide whether a device-specific sentinel is valid data.
 */
template<unsigned valid_bits>
constexpr int32_t signExtend(uint32_t value)
{
	static_assert(
		valid_bits > 0
		&& valid_bits <= 32,
		"Invalid signed field width");

	constexpr uint32_t mask = UINT32_MAX >> (32 - valid_bits);
	constexpr uint32_t sign = uint32_t{1} << (valid_bits - 1);

	const uint32_t bits = value & mask;

	return (bits & sign) ? -1 - static_cast<int32_t>((~bits) & mask) : static_cast<int32_t>(bits);
}

/**
 * @brief Read one unsigned, possibly unaligned contiguous wire field.
 * @tparam bytes Number of bytes to read (1..4), not the host storage width.
 * @tparam order Wire byte order.
 * @param[in] data At least bytes readable bytes; bounds are checked by the caller.
 * @return Zero-extended field. Packed, noncontiguous sensor fields remain decoder policy.
 */
template<unsigned bytes, ByteOrder order>
constexpr uint32_t readUnsigned(const uint8_t *data)
{
	static_assert(
		bytes > 0
		&& bytes <= 4,
		"Invalid wire field size");
	static_assert(
		order == ByteOrder::kBigEndian
		|| order == ByteOrder::kLittleEndian,
		"Invalid byte order");

	uint32_t value = 0;

	for (unsigned i = 0; i < bytes; ++i) {
		const unsigned offset = order == ByteOrder::kBigEndian ? bytes - i - 1 : i;

		value |= static_cast<uint32_t>(data[i]) << (8 * offset);
	}

	return value;
}

/**
 * @brief Explicitly retain the high bits for a legacy int16 publication adapter.
 * @tparam shift Low bits to discard (0..31), chosen with the matching scale by the family.
 * @param[in] value Signed wide raw count.
 * @param[out] narrowed Shifted count; unchanged when it does not fit int16_t.
 * @return True on a representable result, false rather than silently overflowing.
 * @note Negative values round toward negative infinity, matching arithmetic right shift.
 * This conversion is deliberately lossy. Batch append never invokes it implicitly.
 */
template<unsigned shift>
constexpr bool retainHigh16(int32_t value, int16_t &narrowed)
{
	static_assert(shift < 32, "Invalid compatibility shift");

	const int32_t shifted = value >= 0 ? static_cast<int32_t>(static_cast<uint32_t>(value) >> shift)
				: -1 - static_cast<int32_t>(static_cast<uint32_t>(-1 - value) >> shift);

	if (shifted < INT16_MIN || shifted > INT16_MAX) {
		return false;
	}

	narrowed = static_cast<int16_t>(shifted);

	return true;
}

} // namespace imu
