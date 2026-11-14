// SPDX-License-Identifier: BSD-3-Clause
// Configurable Quad-X corner -> PX4 logical motor -> local DShot output mapping.
// Output slots are zero based internally, motor IDs are 1 based.
#pragma once
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace dragonfly_mapping
{
enum Corner : uint8_t { FL = 0, FR = 1, RL = 2, RR = 3 };
enum class Error : uint8_t {
	NONE, UNCONFIGURED, MOTOR_ID, DUPLICATE_MOTOR, OUTPUT_COUNT,
	MISSING_OUTPUT, DUPLICATE_OUTPUT, EXTRA_FUNCTION, INACTIVE_OUTPUT
};
struct Config {
	int32_t motor[4] {0, 0, 0, 0};
};
struct Resolved {
	uint32_t corner[4] {0, 0, 0, 0};
	uint32_t all{0};
	Error error{Error::UNCONFIGURED};
	bool valid() const
	{
		return error == Error::NONE;
	}
};
inline Error validate(const Config &cfg)
{
	for (unsigned i = 0; i < 4; ++i) {
		if (cfg.motor[i] == 0) {
			return Error::UNCONFIGURED;
		}

		// Turtle recovery is restricted to one four-motor vehicle.
		if (cfg.motor[i] < 1 || cfg.motor[i] > 4) {
			return Error::MOTOR_ID;
		}

		for (unsigned j = 0; j < i; ++j) {
			if (cfg.motor[i] == cfg.motor[j]) {
				return Error::DUPLICATE_MOTOR;
			}
		}
	}

	return Error::NONE;
}
inline Resolved resolve(const Config &cfg, const int32_t *functions,
			size_t count, uint32_t active_outputs)
{
	Resolved r;
	r.error = validate(cfg);

	if (!r.valid()) {
		return r;
	}

	if (!functions || count == 0 || count > 32) {
		r.error = Error::OUTPUT_COUNT;
		return r;
	}

	// Only disabled outputs or precisely Motor 1..4 may share this DShot instance.
	for (size_t slot = 0; slot < count; ++slot) {
		const int32_t fn = functions[slot];

		if (fn == 0) {
			continue;
		}

		unsigned corner = 4;

		for (unsigned c = 0; c < 4; ++c) {
			if (fn == 100 + cfg.motor[c]) {
				corner = c;
				break;
			}
		}

		if (corner == 4) {
			r.error = Error::EXTRA_FUNCTION;
			return r;
		}

		const uint32_t bit = uint32_t{1} << slot;

		if (!(active_outputs & bit)) {
			r.error = Error::INACTIVE_OUTPUT;
			return r;
		}

		if (r.corner[corner]) {
			r.error = Error::DUPLICATE_OUTPUT;
			return r;
		}

		r.corner[corner] = bit;
		r.all |= bit;
	}

	for (unsigned c = 0; c < 4; ++c) {
		if (!r.corner[c]) {
			r.error = Error::MISSING_OUTPUT;
			return r;
		}
	}

	r.error = Error::NONE;
	return r;
}
inline bool same(const Resolved &a, const Resolved &b)
{
	if (!a.valid() || !b.valid() || a.all != b.all) {
		return false;
	}

	for (unsigned i = 0; i < 4; ++i) {
		if (a.corner[i] != b.corner[i]) {
			return false;
		}
	}

	return true;
}
inline uint32_t select(const Resolved &map, float roll, float pitch, float deadband = 0.35f)
{
	if (!map.valid() || !std::isfinite(roll) || !std::isfinite(pitch)
	    || std::fabs(roll) > 1.05f || std::fabs(pitch) > 1.05f) {
		return 0;
	}

	if (!std::isfinite(deadband) || deadband < 0.05f || deadband > 0.6f) {
		return 0;
	}

	uint32_t mask = map.all;
	bool selected = false;

	if (roll > deadband) {
		mask &= map.corner[FR] | map.corner[RR];
		selected = true;

	} else if (roll < -deadband) {
		mask &= map.corner[FL] | map.corner[RL];
		selected = true;
	}

	// Positive pitch selects the front motors.
	if (pitch > deadband) {
		mask &= map.corner[FL] | map.corner[FR];
		selected = true;

	} else if (pitch < -deadband) {
		mask &= map.corner[RL] | map.corner[RR];
		selected = true;
	}

	return selected ? mask : 0;
}
inline unsigned outputNumber(uint32_t single_bit)
{
	if (!single_bit || (single_bit & (single_bit - 1))) {
		return 0;
	}

	unsigned n = 1;

	while ((single_bit >>= 1) != 0) {
		++n;
	}

	return n;
}
} // namespace dragonfly_mapping
