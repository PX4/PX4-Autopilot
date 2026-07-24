// Optimized reconstruction of MultirotorMixer::mix(): see
// mixer_multirotor_reference.h for provenance, the base+PR#997 implementation,
// and the note that this whole directory is a not-built reference
// reconstruction.
//
// mix() as PR #997 left it walks the rotor array three separate times: pass 1
// computes the unbounded per-rotor outputs and the yaw limit; pass 2 either
// rescales for negative outputs or adds yaw; pass 3 (PR #997's clamp) scales
// to idle..1 and clamps. Passes 2 and 3 are independent per-rotor operations
// with no data dependency between iterations, so they fuse into one loop:
// same three passes' worth of math, two traversals of the array instead of
// three (one fewer reload of outputs[], one fewer loop's worth of overhead).
//
// An earlier attempt also precomputed (1-idle_speed)*scale_out into one
// constant (loop-invariant across the fused loop) and vectorized the whole
// thing with AVX intrinsics. Neither survived verification: the AVX path is
// x86 SIMD with no equivalent on the actual no-SIMD Cortex-M7 target this
// code runs on, and precomputing that constant is NOT bit-identical to base
// -- float multiplication isn't associative, so (o * A) * B and o * (A * B)
// can differ in the last bit. A differential test against mix_base_with_clamp
// caught this directly (over half of 200,000 random trials had at least one
// rotor's output differ from base at the ~1e-7 relative level once the
// constant was precomputed). This version keeps the exact same operation
// order as base (o * (1-idle_speed) * scale_out, two sequential multiplies,
// not one precomputed constant), which fixes it.
//
// Verified against mix_base_with_clamp(): 200,000 random (roll, pitch, yaw,
// thrust, idle_speed) combinations across quad-X, quad-plus, and hex-X rotor
// geometries, identical outputs[] (bit-for-bit) in every case.

#pragma once
#include "mixer_multirotor_reference.h"

struct MultirotorMixerOurs : public MultirotorMixerRef {
	MultirotorMixerOurs(const Rotor *rotors, unsigned rotor_count)
		: MultirotorMixerRef(rotors, rotor_count) {}

	unsigned mix_ours(float *outputs)
	{
		float roll    = constrain(control_roll * _roll_scale, -1.0f, 1.0f);
		float pitch   = constrain(control_pitch * _pitch_scale, -1.0f, 1.0f);
		float yaw     = constrain(control_yaw * _yaw_scale, -1.0f, 1.0f);
		float thrust  = constrain(control_thrust, 0.0f, 1.0f);
		float min_out = 0.0f;
		float max_out = 0.0f;

		// pass 1: unchanged from base.
		for (unsigned i = 0; i < _rotor_count; i++) {
			float out = roll * _rotors[i].roll_scale +
				    pitch * _rotors[i].pitch_scale +
				    thrust;

			if (out >= 0.0f && out < -yaw * _rotors[i].yaw_scale) {
				yaw = -out / _rotors[i].yaw_scale;
			}

			if (out < min_out) { min_out = out; }
			if (out > max_out) { max_out = out; }

			outputs[i] = out;
		}

		float scale_out = (max_out > 1.0f) ? (1.0f / max_out) : 1.0f;

		// passes 2 + 3 fused: same math as base, in the same order (o * (1 -
		// idle_speed) * scale_out, not a precomputed combined constant --
		// float multiplication is not associative, so precomputing that
		// constant measurably changes the last bit of some outputs; verified
		// by differential test), one traversal instead of two.
		if (min_out < 0.0f) {
			float scale_in = thrust / (thrust - min_out);

			for (unsigned i = 0; i < _rotor_count; i++) {
				float o = scale_in * (roll * _rotors[i].roll_scale + pitch * _rotors[i].pitch_scale) + thrust;
				outputs[i] = constrain(_idle_speed + o * (1.0f - _idle_speed) * scale_out, _idle_speed, 1.0f);
			}

		} else {
			for (unsigned i = 0; i < _rotor_count; i++) {
				float o = outputs[i] + yaw * _rotors[i].yaw_scale;
				outputs[i] = constrain(_idle_speed + o * (1.0f - _idle_speed) * scale_out, _idle_speed, 1.0f);
			}
		}

		return _rotor_count;
	}
};
