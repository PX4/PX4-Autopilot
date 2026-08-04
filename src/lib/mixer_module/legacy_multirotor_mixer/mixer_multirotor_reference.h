// Reference reconstruction of MultirotorMixer::mix() (historically
// src/modules/systemlib/mixer/mixer_multirotor.cpp), which no longer exists in
// this tree -- the standalone mixer library was replaced by the
// ControlAllocation / ActuatorEffectiveness system (src/modules/control_allocator,
// src/lib/mixer_module). Kept here as a reference reconstruction of a real,
// measured optimization, not a copy of the current mixing architecture.
//
// mix()'s body below is copied verbatim from the pre-PR-#997 source (only the
// class/member scaffolding around it is reconstructed, to make it a
// self-contained, compilable, testable reference). PR #997
// (https://github.com/PX4/PX4-Autopilot/pull/997) added a mandatory safety
// clamp to the final scale loop: unclamped, a saturated command could leave
// the valid output range. See mixer_multirotor_ours.h for the optimized
// version that keeps that clamp.
//
// Not referenced by any build file. Not built.

#pragma once
#include <cstdint>
#include <cmath>

static inline float constrain(float val, float min, float max)
{
	return (val < min) ? min : ((val > max) ? max : val);
}

struct Rotor {
	float roll_scale;
	float pitch_scale;
	float yaw_scale;
};

struct MultirotorMixerRef {
	const Rotor *_rotors;
	unsigned _rotor_count;
	float _roll_scale = 1.0f;
	float _pitch_scale = 1.0f;
	float _yaw_scale = 1.0f;
	float _idle_speed = 0.0f;

	// stand-ins for get_control(0, N): roll/pitch/yaw/thrust setpoints, as
	// plain fields instead of the real control-group indirection.
	float control_roll = 0.0f;
	float control_pitch = 0.0f;
	float control_yaw = 0.0f;
	float control_thrust = 0.0f;

	MultirotorMixerRef(const Rotor *rotors, unsigned rotor_count)
		: _rotors(rotors), _rotor_count(rotor_count) {}

	// base as PR #997 modified it: mandatory safety clamp on the final scale
	// loop (the only change PR #997 makes; everything above it is unchanged
	// from pre-#997 base, copied verbatim).
	unsigned mix_base_with_clamp(float *outputs)
	{
		float roll    = constrain(control_roll * _roll_scale, -1.0f, 1.0f);
		float pitch   = constrain(control_pitch * _pitch_scale, -1.0f, 1.0f);
		float yaw     = constrain(control_yaw * _yaw_scale, -1.0f, 1.0f);
		float thrust  = constrain(control_thrust, 0.0f, 1.0f);
		float min_out = 0.0f;
		float max_out = 0.0f;

		/* perform initial mix pass yielding unbounded outputs, ignore yaw */
		for (unsigned i = 0; i < _rotor_count; i++) {
			float out = roll * _rotors[i].roll_scale +
				    pitch * _rotors[i].pitch_scale +
				    thrust;

			/* limit yaw if it causes outputs clipping */
			if (out >= 0.0f && out < -yaw * _rotors[i].yaw_scale) {
				yaw = -out / _rotors[i].yaw_scale;
			}

			if (out < min_out) { min_out = out; }
			if (out > max_out) { max_out = out; }

			outputs[i] = out;
		}

		/* scale down roll/pitch controls if some outputs are negative, don't add yaw, keep total thrust */
		if (min_out < 0.0f) {
			float scale_in = thrust / (thrust - min_out);

			for (unsigned i = 0; i < _rotor_count; i++) {
				outputs[i] = scale_in * (roll * _rotors[i].roll_scale + pitch * _rotors[i].pitch_scale) + thrust;
			}

		} else {
			for (unsigned i = 0; i < _rotor_count; i++) {
				outputs[i] += yaw * _rotors[i].yaw_scale;
			}
		}

		/* scale down all outputs if some outputs are too large, reduce total thrust */
		float scale_out = (max_out > 1.0f) ? (1.0f / max_out) : 1.0f;

		/* scale outputs to range _idle_speed..1, and do final limiting (PR #997) */
		for (unsigned i = 0; i < _rotor_count; i++) {
			outputs[i] = constrain(_idle_speed + (outputs[i] * (1.0f - _idle_speed) * scale_out), _idle_speed, 1.0f);
		}

		return _rotor_count;
	}
};
