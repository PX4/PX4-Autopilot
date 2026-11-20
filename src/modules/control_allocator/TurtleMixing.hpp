// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 PX4 Development Team.

#pragma once

#include "VehicleActuatorEffectiveness/ActuatorEffectivenessRotors.hpp"
#include <mathlib/math/Limits.hpp>
#include <px4_platform_common/defines.h>
#include <uORB/topics/actuator_motors.h>

#include <cmath>
#include <cstring>

class TurtleMixing
{
public:
	using Geometry = ActuatorEffectivenessRotors::Geometry;
	static constexpr unsigned MOTOR_COUNT = 4;
	static constexpr uint16_t MOTOR_MASK = (1u << MOTOR_COUNT) - 1u;
	static constexpr float STICK_DEADBAND = 0.2f;

	void configure(const Geometry *geometry)
	{
		_geometry_valid = false;

		if (!geometry || geometry->num_rotors != static_cast<int>(MOTOR_COUNT)) {
			return;
		}

		std::memset(_quadrant_masks, 0, sizeof(_quadrant_masks));
		constexpr float POSITION_EPSILON = 1e-4f;
		constexpr float AXIS_TOLERANCE = 0.01f;

		for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
			const auto &rotor = geometry->rotors[i];

			if (!rotor.position.isAllFinite() || !rotor.axis.isAllFinite()
			    || fabsf(rotor.position(0)) <= POSITION_EPSILON || fabsf(rotor.position(1)) <= POSITION_EPSILON
			    || rotor.axis(2) >= -POSITION_EPSILON
			    || fabsf(rotor.axis(0)) > -rotor.axis(2) * AXIS_TOLERANCE
			    || fabsf(rotor.axis(1)) > -rotor.axis(2) * AXIS_TOLERANCE
			    || !PX4_ISFINITE(rotor.thrust_coef) || rotor.thrust_coef <= 0.f
			    || !PX4_ISFINITE(rotor.moment_ratio) || rotor.tilt_index >= 0) {
				return;
			}

			uint8_t &quadrant = _quadrant_masks[rotor.position(1) > 0.f][rotor.position(0) > 0.f];

			if (quadrant != 0) {
				return;
			}

			quadrant = 1u << i;
		}

		_geometry_valid = true;
	}

	bool geometryValid() const { return _geometry_valid; }

	bool sameMapping(const TurtleMixing &other) const
	{
		return _geometry_valid == other._geometry_valid
		       && (!_geometry_valid || std::memcmp(_quadrant_masks, other._quadrant_masks, sizeof(_quadrant_masks)) == 0);
	}

	static void stop(float (&controls)[actuator_motors_s::NUM_CONTROLS])
	{
		for (float &control : controls) {
			control = NAN;
		}
	}

	bool mix(float roll, float pitch, float max_thrust, float (&controls)[actuator_motors_s::NUM_CONTROLS]) const
	{
		stop(controls);

		if (!_geometry_valid || !PX4_ISFINITE(roll) || !PX4_ISFINITE(pitch)
		    || fabsf(roll) > 1.f || fabsf(pitch) > 1.f
		    || !PX4_ISFINITE(max_thrust) || max_thrust <= 0.f || max_thrust > 1.f) {
			return false;
		}

		const float abs_roll = fabsf(roll);
		const float abs_pitch = fabsf(pitch);
		const float deflection = math::max(abs_roll, abs_pitch);

		if (deflection <= STICK_DEADBAND) {
			return true;
		}

		uint8_t motors = MOTOR_MASK;

		if (abs_roll > STICK_DEADBAND) {
			const auto &side = _quadrant_masks[roll > 0.f];
			motors = side[0] | side[1];
		}

		if (abs_pitch > STICK_DEADBAND) {
			// Positive manual pitch requests nose-down rotation: reversed front motors supply it.
			const bool front = pitch > 0.f;
			motors &= _quadrant_masks[0][front] | _quadrant_masks[1][front];
		}

		const float thrust = max_thrust * ((deflection - STICK_DEADBAND) / (1.f - STICK_DEADBAND));

		// Zero is armed idle in the normal motor pipeline. Keep all motors stopped if thrust rounds to zero.
		if (thrust > 0.f) {
			for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
				if (motors & (1u << i)) {
					controls[i] = thrust;
				}
			}
		}

		return true;
	}

private:
	uint8_t _quadrant_masks[2][2] {}; // Indexed by positive body Y (right) and positive body X (front).
	bool _geometry_valid{false};
};
