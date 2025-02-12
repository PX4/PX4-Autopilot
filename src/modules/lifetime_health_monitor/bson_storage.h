/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#pragma once

#include <cstring>

#include <lib/tinybson/tinybson.h>
#include <px4_platform_common/log.h>

#define KEY_LENGTH 3

template<typename T>
class BsonKeyValue
{
public:
	// Constructor that takes a string literal
	constexpr BsonKeyValue(const char (&key)[KEY_LENGTH + 1]) : _key(key)
	{
		_value = 0;
	}

	// Constructor that takes a string literal and initial value
	constexpr BsonKeyValue(const char (&key)[KEY_LENGTH + 1], T init_value) : _key(key)
	{
		_value = init_value;
	}

	// Encode value to BSON
	int encode(bson_encoder_s *encoder) const
	{
		if (encoder == nullptr) {
			PX4_ERR("BSON encoder is null");
			return -1;
		}

		// uint8_t and uint16_t both encode as int32
		if (sizeof(T) <= sizeof(uint16_t)) {
			return bson_encoder_append_int32(encoder, _key, static_cast<int32_t>(_value));

		} else if (sizeof(T) == sizeof(uint32_t)) {
			return bson_encoder_append_int32(encoder, _key, _value);

		} else if (sizeof(T) == sizeof(uint64_t)) {
			return bson_encoder_append_int64(encoder, _key, _value);
		}

		PX4_ERR("Unsupported type size for BSON encoding: %zu bytes", sizeof(T));
		return -1;
	}

	// Decode value from BSON node
	bool decode(const bson_node_s *node)
	{
		if (node == nullptr || strcmp(node->name, _key) != 0) {
			return false;
		}

		// Handle uint8_t and uint16_t
		if (sizeof(T) <= sizeof(uint16_t)) {
			if (node->type != BSON_INT32 || node->i32 < 0) {
				return false;
			}

			const uint32_t max_value = (sizeof(T) == sizeof(uint8_t)) ? UINT8_MAX : UINT16_MAX;

			if (static_cast<uint32_t>(node->i32) > max_value) {
				return false;
			}

			_value = static_cast<T>(node->i32);
			return true;
		}

		// Handle uint32_t or int
		if (sizeof(T) == sizeof(uint32_t)) {
			if (node->type != BSON_INT32 || node->i32 < 0) {
				return false;
			}

			_value = static_cast<T>(node->i32);
			return true;
		}

		// Handle uint64_t
		if (sizeof(T) == sizeof(uint64_t)) {
			if (node->type != BSON_INT64 || node->i64 < 0) {
				return false;
			}

			_value = static_cast<T>(node->i64);
			return true;
		}

		return false;
	}

	// Access the stored value
	T &value() { return _value; }
	const T &value() const { return _value; }

private:
	const char (&_key)[KEY_LENGTH + 1];
	T _value;
};


template<typename T, size_t N>
class BsonKeyValueArray
{
public:
	// Ensure T is one of the supported unsigned integer types
	static_assert(sizeof(T) == 1 || sizeof(T) == 2 || sizeof(T) == 4 || sizeof(T) == 8,
		      "Type must be uint8_t, uint16_t, uint32_t or uint64_t");

	BsonKeyValueArray(const char (&key)[KEY_LENGTH + 1]) : _key(key)
	{
		memset(_values, 0, sizeof(_values));
	}

	BsonKeyValueArray(const char (&key)[KEY_LENGTH + 1], T init_value) : _key(key)
	{
		for (size_t i = 0; i < N; i++) {
			_values[i] = init_value;
		}
	}

	// Access array elements
	T &operator[](size_t index) { return _values[index]; }
	const T &operator[](size_t index) const { return _values[index]; }

	// Get array size
	static constexpr size_t size() { return N; }

	// Get key
	const char *key() const { return _key; }

	/**
	 * Encode array to BSON format using binary type.
	 * Format:
	 * - 1 byte: Element type (BSON_INT32 or BSON_INT64)
	 * - 4 bytes: Number of elements (N)
	 * - N * sizeof(T) bytes: Array data
	 *
	 * @param encoder BSON encoder to write the binary data to
	 * @return 0 on success, -1 on failure
	 */
	int encode(bson_encoder_t encoder) const
	{
		// Create buffer with metadata + array data
		uint8_t buffer[1 + sizeof(uint32_t) + sizeof(T) * N];
		uint8_t *ptr = buffer;

		// Write element type flag
		*ptr++ = (sizeof(T) <= sizeof(uint32_t)) ? BSON_INT32 : BSON_INT64;

		// Write number of elements
		uint32_t count = N;
		memcpy(ptr, &count, sizeof(uint32_t));
		ptr += sizeof(uint32_t);

		// Copy array data
		memcpy(ptr, _values, sizeof(T) * N);

		// Store as binary data
		return bson_encoder_append_binary(encoder, _key, BSON_BIN_USER,
						  sizeof(buffer), buffer);
	}


	/**
	 * Decode array from BSON binary data
	 * @param node BSON node containing the binary data
	 * @return 0 on success, -1 on failure
	 */
	int decode(bson_decoder_t decoder, const bson_node_t node)
	{
		if (node->type != BSON_BINDATA) {
			PX4_ERR("FAIL: decoder: data1 type %d, expected %d", node->type, BSON_BINDATA);
			return 1;
		}

		size_t size = bson_decoder_data_pending(decoder);

		if (node->subtype != BSON_BIN_USER) {
			PX4_ERR("FAIL: decoder: data1 subtype %d, expected %d", node->subtype, BSON_BIN_BINARY);
			return 1;
		}

		uint8_t data[size];

		if (bson_decoder_copy_data(decoder, data)) {
			PX4_ERR("FAIL: decoder: data1 copy failed");
			return 1;
		}

		if (bson_decoder_data_pending(decoder) != 0) {
			PX4_ERR("FAIL: decoder: data1 copy did not exhaust all data");
			return 1;
		}

		// Verify minimum size (type byte + element count)
		if (size < (1 + sizeof(uint32_t))) {
			return -1;
		}

		// Verify element type matches T
		uint8_t type = data[0];

		if (((sizeof(T) <= sizeof(uint32_t)) && type != BSON_INT32) ||
		    ((sizeof(T) > sizeof(uint32_t)) && type != BSON_INT64)) {
			return -1;
		}

		// Get number of elements
		uint32_t stored_count;
		memcpy(&stored_count, data + 1, sizeof(uint32_t));

		// Verify array size matches
		if (stored_count != N || size != (1 + sizeof(uint32_t) + sizeof(T) * N)) {
			return -1;
		}

		// Copy array data
		memcpy(_values, data + 1 + sizeof(uint32_t), sizeof(T) * N);
		return 0;
	}

private:
	const char (&_key)[KEY_LENGTH + 1];
	T _values[N];
};
