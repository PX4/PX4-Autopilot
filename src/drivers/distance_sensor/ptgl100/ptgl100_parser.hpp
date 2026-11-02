/*
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace ptgl100
{

static constexpr size_t FRAME_SIZE = 8;
static constexpr uint8_t HEADER_HIGH = 0x55;
static constexpr uint8_t HEADER_LOW = 0xaa;
static constexpr uint8_t SELF_TEST = 0x80;
static constexpr uint8_t SINGLE_MEASUREMENT = 0x68;
static constexpr uint8_t CONTINUOUS_MEASUREMENT = 0x69;
static constexpr uint8_t STOP_MEASUREMENT = 0x8e;
static constexpr uint8_t STATUS_SUCCESS = 1;
static constexpr float MIN_DISTANCE_M = 0.2f;
static constexpr float MAX_DISTANCE_M = 100.f;

// Revised protocol: unsigned 24-bit big-endian distance in millimetres.
constexpr float distance_m(uint32_t raw)
{
	// Preserve exact whole metres even when -freciprocal-math replaces division
	// with multiplication (100000 * 0.001f can round above the 100 m range limit).
	return static_cast<float>(raw / 1000u) + static_cast<float>(raw % 1000u) / 1000.f;
}

void make_command(uint8_t command, uint8_t (&frame)[FRAME_SIZE]);

struct Reply {
	uint8_t command{0};
	uint8_t status{0};
	uint32_t distance_raw{0};
	uint8_t error_code{0};

	bool is_measurement() const { return command == SINGLE_MEASUREMENT || command == CONTINUOUS_MEASUREMENT; }
};

class Parser
{
public:
	enum class Result {
		Incomplete,
		Reply,
		BadChecksum,
		UnknownCommand,
	};

	Result parse(uint8_t byte, Reply &reply);
	void reset() { _length = 0; }

private:
	void resynchronise();

	uint8_t _buffer[FRAME_SIZE] {};
	size_t _length{0};
};

} // namespace ptgl100
