/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *   SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

#include "IIOScan.hpp"

#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace iio_imu
{

bool normalize_attribute(char *text, size_t bytes, size_t capacity)
{
	if (text == nullptr || bytes == 0 || bytes >= capacity) {
		return false;
	}

	size_t length = 0;

	while (length < bytes && text[length] != '\0') {
		++length;
	}

	// Permit NUL padding, but do not silently ignore non-text data after an embedded NUL.
	for (size_t i = length; i < bytes; ++i) {
		if (text[i] != '\0') {
			return false;
		}
	}

	while (length > 0 && isspace(static_cast<unsigned char>(text[length - 1]))) {
		--length;
	}

	text[length] = '\0';
	return length != 0;
}

bool parse_unsigned(const char *text, uint32_t &value, int base)
{
	if (text == nullptr || text[0] < '0' || text[0] > '9') {
		return false;
	}

	char *end = nullptr;
	errno = 0;
	const unsigned long parsed = strtoul(text, &end, base);

	if (errno != 0 || *end != '\0' || parsed > UINT32_MAX) {
		return false;
	}

	value = static_cast<uint32_t>(parsed);
	return true;
}

bool parse_float(const char *text, float &value)
{
	if (text == nullptr || text[0] == '\0') {
		return false;
	}

	char *end = nullptr;
	errno = 0;
	const float parsed = strtof(text, &end);

	if (errno != 0 || end == text || *end != '\0' || !isfinite(parsed)) {
		return false;
	}

	value = parsed;
	return true;
}

bool SensorScale::configure(const char *scale_text, const char *range_text, uint8_t bits)
{
	if (bits < 2 || bits > 16 || !parse_float(scale_text, scale) || scale <= 0.f) {
		return false;
	}

	const uint32_t full_scale = uint32_t{1} << (bits - 1);
	range_from_bits = range_text == nullptr;
	range_raw = full_scale;

	if (!range_from_bits && (!parse_unsigned(range_text, range_raw) || range_raw == 0 || range_raw > full_scale)) {
		return false;
	}

	return isfinite(range()) && range() > 0.f;
}

bool parse_type(const char *text, Channel &channel)
{
	if (text == nullptr || (strncmp(text, "le:", 3) != 0 && strncmp(text, "be:", 3) != 0)
	    || (text[3] != 's' && text[3] != 'u')) {
		return false;
	}

	// Bound each integer before narrowing it; do not accept partial sscanf matches.
	const char *position = text + 4;
	auto number = [&position](unsigned long & value) {
		if (*position < '0' || *position > '9') {
			return false;
		}

		char *end = nullptr;
		errno = 0;
		value = strtoul(position, &end, 10);
		position = end;
		return errno == 0 && value <= 64;
	};

	unsigned long bits = 0;
	unsigned long storage = 0;
	unsigned long shift = 0;

	if (!number(bits) || *position != '/') {
		return false;
	}

	++position;

	if (!number(storage)) {
		return false;
	}

	if (*position == 'X') {
		++position;
		unsigned long repeat = 0;

		if (!number(repeat) || repeat != 1) {
			return false;
		}
	}

	if (strncmp(position, ">>", 2) == 0) {
		position += 2;

		if (!number(shift)) {
			return false;
		}
	}

	if (*position != '\0' || bits == 0 || bits + shift > storage
	    || (storage != 8 && storage != 16 && storage != 32 && storage != 64)) {
		return false;
	}

	channel.bits = static_cast<uint8_t>(bits);
	channel.storage_bits = static_cast<uint8_t>(storage);
	channel.shift = static_cast<uint8_t>(shift);
	channel.little_endian = text[0] == 'l';
	channel.is_signed = text[3] == 's';
	return true;
}

bool make_layout(Channel(&channels)[MAX_CHANNEL_COUNT], size_t &scan_size, bool temperature_enabled)
{
	const size_t count = temperature_enabled ? MAX_CHANNEL_COUNT : CHANNEL_COUNT;
	size_t order[MAX_CHANNEL_COUNT] {};

	for (size_t i = 0; i < count; ++i) {
		const Channel &channel = channels[i];

		if (channel.bits == 0 || channel.bits + channel.shift > channel.storage_bits) {
			return false;
		}

		if (i == TIMESTAMP_CHANNEL) {
			if (!channel.is_signed || channel.bits != 64 || channel.storage_bits != 64 || channel.shift != 0) {
				return false;
			}

		} else {
			if (channel.index >= channels[TIMESTAMP_CHANNEL].index
			    || (channel.storage_bits != 8 && channel.storage_bits != 16 && channel.storage_bits != 32)) {
				return false;
			}

			if (i == TEMPERATURE_CHANNEL) {
				if (channel.bits > 32) {
					return false;
				}

			} else if (!channel.is_signed || channel.bits < 2 || channel.bits > 16) {
				return false;
			}
		}

		order[i] = i;

		for (size_t j = i; j > 0 && channels[order[j]].index < channels[order[j - 1]].index; --j) {
			const size_t previous = order[j - 1];
			order[j - 1] = order[j];
			order[j] = previous;
		}
	}

	for (size_t axis = 1; axis < 3; ++axis) {
		if (channels[axis].bits != channels[0].bits || channels[axis + 3].bits != channels[3].bits) {
			return false;
		}
	}

	size_t bytes = 0;
	size_t alignment = 1;

	for (size_t i = 0; i < count; ++i) {
		Channel &channel = channels[order[i]];

		if (i > 0 && channel.index == channels[order[i - 1]].index) {
			return false;
		}

		const size_t storage_bytes = channel.storage_bits / 8;
		bytes = (bytes + storage_bytes - 1) / storage_bytes * storage_bytes;
		channel.offset = bytes;
		bytes += storage_bytes;

		if (storage_bytes > alignment) {
			alignment = storage_bytes;
		}
	}

	scan_size = (bytes + alignment - 1) / alignment * alignment;
	return scan_size <= MAX_SCAN_SIZE;
}

int64_t decode(const uint8_t *scan, const Channel &channel)
{
	uint64_t value = 0;
	const size_t bytes = channel.storage_bits / 8;

	for (size_t i = 0; i < bytes; ++i) {
		const size_t byte = channel.little_endian ? i : bytes - i - 1;
		value |= static_cast<uint64_t>(scan[channel.offset + byte]) << (8 * i);
	}

	value >>= channel.shift;

	if (channel.bits == 64) {
		// This also handles negative timestamps without implementation-defined unsigned casts.
		return value <= INT64_MAX ? static_cast<int64_t>(value) : -1 - static_cast<int64_t>(~value);
	}

	value &= (uint64_t{1} << channel.bits) - 1;
	const int64_t raw = static_cast<int64_t>(value);
	return channel.is_signed && (value & (uint64_t{1} << (channel.bits - 1))) ? raw - (int64_t{1} << channel.bits) : raw;
}

bool TemperatureScale::configure(const char *scale_text, const char *offset_text)
{
	offset = 0.f;
	return parse_float(scale_text, scale) && scale > 0.f
	       && (offset_text == nullptr || parse_float(offset_text, offset));
}

float TemperatureScale::celsius(float raw) const
{
	return (raw + offset) * scale * 0.001f;
}

} // namespace iio_imu
