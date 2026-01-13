#pragma once

#include <matrix/math.hpp>
#include <px4_platform_common/module_params.h>
#include <float.h>
#include <math.h>


namespace MulticopterTurtleUtil
{

	enum class TurtleModeState {
		FLYING_DISABLED = 0,    // Initial/disabled state - turtle mode not available
		OPTIONAL_TURTLE = 1,     // Turtle mode available but not active
		ENTERING_TURTLE = 2,    // Transitioning into turtle mode
		ACTIVE_TURTLE = 3,      // Turtle mode is active
		EXITING_TURTLE = 4,     // Transitioning out of turtle mode
	};


	enum class TurtleModeESCInternalState {
		TURTLE_MODE_ESC_INTERNAL_STATE_3D_ON = 0,
		TURTLE_MODE_ESC_INTERNAL_STATE_3D_OFF = 1,
	};

	enum class Quadrants {
		FIRST = 0, // (+,+)
		SECOND = 1, // (-,+)
		THIRD = 2, // (-,-)
		FOURTH = 3, // (+,-)
		Longitudinal = 4, // nose axis
		Lateral = 5, // wing axis 
		Invalid = 6, // center or problem value 
	};

	struct MotorCommands {
		float roll;
		float pitch;
	};

	struct Motor_data {
		uint16_t motor_number;
		bool motor_direction;
		float x_position;
		float y_position;
		Quadrants motor_quadrant;
	};




inline Quadrants getQuadrant(float x, float y) {
	// Handle edge cases first using epsilon comparison for floating-point safety
	const float epsilon = 1e-6f;
	const bool x_is_zero = fabsf(x) < epsilon;
	const bool y_is_zero = fabsf(y) < epsilon;
	
	if (x_is_zero && y_is_zero) {
		return Quadrants::Invalid;
	}
	
	if (y_is_zero) {
		return Quadrants::Longitudinal;
	}
	
	if (x_is_zero) {
		return Quadrants::Lateral;
	}
	
	// Determine quadrant based on signs
	// Quadrant mapping: FIRST(+,+), SECOND(-,+), THIRD(-,-), FOURTH(+,-)
	const bool x_positive = x > 0.0f;
	const bool y_positive = y > 0.0f;
	
	if (x_positive) {
		return y_positive ? Quadrants::FIRST : Quadrants::FOURTH;
	}
	
	return y_positive ? Quadrants::SECOND : Quadrants::THIRD;
}


inline float Threshold(float value, float threshold){

	if (abs(value) < threshold) { return NAN; }

	return value;

}



}
