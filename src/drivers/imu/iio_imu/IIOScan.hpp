/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *   SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace iio_imu
{

static constexpr size_t CHANNEL_COUNT = 7;
static constexpr size_t TIMESTAMP_CHANNEL = 6;
static constexpr size_t TEMPERATURE_CHANNEL = 7;
static constexpr size_t MAX_CHANNEL_COUNT = 8;
static constexpr size_t MAX_SCAN_SIZE = 64;

// Array order is accel XYZ, angular velocity XYZ, timestamp, optional temperature.
// Scan index determines wire order; the timestamp must have the greatest index.
struct Channel {
	uint32_t index{0};
	size_t offset{0};
	uint8_t bits{0};
	uint8_t storage_bits{0};
	uint8_t shift{0};
	bool little_endian{true};
	bool is_signed{true};
};

struct SensorScale {
	float scale{0.f};
	uint32_t range_raw{0};
	bool range_from_bits{false};

	// nullptr means the range attribute is absent, not invalid or unreadable.
	bool configure(const char *scale_text, const char *range_text, uint8_t bits);
	float range() const { return static_cast<float>(range_raw) * scale; }
};

struct TemperatureScale {
	float scale{0.f};
	float offset{0.f};

	bool configure(const char *scale_text, const char *offset_text);
	float celsius(float raw) const;
};

bool parse_unsigned(const char *text, uint32_t &value, int base = 10);
bool parse_float(const char *text, float &value);
// Some older sysfs show() callbacks include a terminating NUL in their byte count.
bool normalize_attribute(char *text, size_t bytes, size_t capacity);
bool parse_type(const char *text, Channel &channel);
bool make_layout(Channel(&channels)[MAX_CHANNEL_COUNT], size_t &scan_size, bool temperature_enabled = false);
int64_t decode(const uint8_t *scan, const Channel &channel);

} // namespace iio_imu
