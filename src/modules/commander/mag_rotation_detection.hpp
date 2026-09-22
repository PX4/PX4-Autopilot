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
 * @file mag_rotation_detection.hpp
 *
 * Magnetometer rotation detection using gravity as reference (no reference mag needed).
 *
 * The vertical component of the earth magnetic field (B * sin(inclination)) is constant
 * regardless of the vehicle attitude. For every candidate rotation the calibrated mag
 * samples are rotated to body frame and projected onto the gravity direction measured by
 * the accelerometer at the same time. The correct rotation is the one where this projection
 * varies the least across all samples. Same idea as ArduPilot's COMPASS_AUTO_ROT, but using
 * gravity only, so it does not depend on a (possibly mag-corrupted) heading estimate.
 */

#pragma once

#include <float.h>
#include <math.h>

#include <lib/conversion/rotation.h>
#include <matrix/math.hpp>

namespace mag_rotation_detection
{

static constexpr float GRAVITY_MSS = 9.80665f;
static constexpr float GRAVITY_TOLERANCE = 0.2f;        ///< reject samples with |accel| outside g * (1 +- tolerance)
static constexpr unsigned MIN_SAMPLES = 50;              ///< minimum number of samples with valid gravity
static constexpr float CONFIDENCE_ANY = 4.f;             ///< variance ratio to accept any rotation (incl. 45 deg)
static constexpr float CONFIDENCE_RIGHT_ANGLE = 2.f;     ///< variance ratio to accept a 90/180 deg rotation
static constexpr float MAX_STD_GAUSS = 0.15f;            ///< max std dev of the vertical field for the best rotation

struct Result {
	bool valid{false};                           ///< true if a rotation was confidently determined
	Rotation best_rotation{ROTATION_NONE};
	Rotation second_rotation{ROTATION_NONE};
	float best_std{NAN};                         ///< [Gauss] std dev of the vertical field component with best rotation
	float confidence{0.f};                       ///< variance(second best) / variance(best)
	unsigned samples_used{0};
};

/// Rotations that are not tested (duplicates or special cases)
inline bool rotation_skipped(int r)
{
	switch (r) {
	case ROTATION_PITCH_180_YAW_90:           // same as ROTATION_ROLL_180_YAW_270
	case ROTATION_PITCH_180_YAW_270:          // same as ROTATION_ROLL_180_YAW_90
	case ROTATION_ROLL_90_PITCH_68_YAW_293:   // special case, never auto detected
		return true;

	default:
		return false;
	}
}

/// True for rotations composed of 90 degree steps only
inline bool rotation_right_angle(int r)
{
	switch (r) {
	case ROTATION_YAW_45:
	case ROTATION_YAW_135:
	case ROTATION_YAW_225:
	case ROTATION_YAW_315:
	case ROTATION_ROLL_180_YAW_45:
	case ROTATION_ROLL_180_YAW_135:
	case ROTATION_ROLL_180_YAW_225:
	case ROTATION_ROLL_180_YAW_315:
	case ROTATION_ROLL_90_YAW_45:
	case ROTATION_ROLL_90_YAW_135:
	case ROTATION_ROLL_270_YAW_45:
	case ROTATION_ROLL_270_YAW_135:
	case ROTATION_ROLL_90_PITCH_68_YAW_293:
	case ROTATION_PITCH_315:
	case ROTATION_ROLL_90_PITCH_315:
		return false;

	default:
		return (r >= 0) && (r < ROTATION_MAX);
	}
}

/**
 * Determine the mag rotation (sensor to body) from gravity.
 *
 * @param x, y, z   calibrated (offsets/scale removed) mag samples in sensor frame [Gauss]
 * @param accel     accelerometer specific force in body frame [m/s^2] for each sample (NAN if not available)
 * @param n         number of samples
 * @param variance  optional output, variance of the vertical field component for each rotation (FLT_MAX if skipped)
 */
inline Result detect(const float *x, const float *y, const float *z, const matrix::Vector3f *accel, unsigned n,
		     float *variance = nullptr)
{
	Result result{};

	float var[ROTATION_MAX];

	for (int r = 0; r < ROTATION_MAX; r++) {
		var[r] = FLT_MAX;

		if (rotation_skipped(r)) {
			continue;
		}

		const matrix::Dcmf R = get_rot_matrix(static_cast<Rotation>(r));

		double sum = 0.;
		double sum_sq = 0.;
		unsigned count = 0;

		for (unsigned i = 0; i < n; i++) {
			const matrix::Vector3f &a = accel[i];

			if (!a.isAllFinite()) {
				continue;
			}

			const float a_norm = a.norm();

			if (fabsf(a_norm - GRAVITY_MSS) > GRAVITY_MSS * GRAVITY_TOLERANCE) {
				// vehicle accelerating, gravity direction not reliable
				continue;
			}

			const matrix::Vector3f m_body = R * matrix::Vector3f{x[i], y[i], z[i]};
			const double vertical = m_body.dot(a) / a_norm;

			sum += vertical;
			sum_sq += vertical * vertical;
			count++;
		}

		result.samples_used = count;

		if (count >= MIN_SAMPLES) {
			const double mean = sum / count;
			var[r] = fmaxf(static_cast<float>(sum_sq / count - mean * mean), 0.f);
		}
	}

	if (variance != nullptr) {
		for (int r = 0; r < ROTATION_MAX; r++) {
			variance[r] = var[r];
		}
	}

	if (result.samples_used < MIN_SAMPLES) {
		return result;
	}

	// find best and second best, over all rotations and over right angle rotations only
	int best = -1, second = -1;
	int best_90 = -1, second_90 = -1;

	for (int r = 0; r < ROTATION_MAX; r++) {
		if (var[r] >= FLT_MAX) {
			continue;
		}

		if ((best < 0) || (var[r] < var[best])) {
			second = best;
			best = r;

		} else if ((second < 0) || (var[r] < var[second])) {
			second = r;
		}

		if (rotation_right_angle(r)) {
			if ((best_90 < 0) || (var[r] < var[best_90])) {
				second_90 = best_90;
				best_90 = r;

			} else if ((second_90 < 0) || (var[r] < var[second_90])) {
				second_90 = r;
			}
		}
	}

	if ((best < 0) || (second < 0) || (best_90 < 0) || (second_90 < 0)) {
		return result;
	}

	static constexpr float VAR_MIN = 1e-9f; // avoid division by zero with perfect data
	const float confidence = var[second] / fmaxf(var[best], VAR_MIN);

	if (confidence > CONFIDENCE_ANY) {
		result.best_rotation = static_cast<Rotation>(best);
		result.second_rotation = static_cast<Rotation>(second);
		result.confidence = confidence;

	} else {
		// 45 degree rotations are hard to distinguish, only consider right angle rotations
		result.best_rotation = static_cast<Rotation>(best_90);
		result.second_rotation = static_cast<Rotation>(second_90);
		result.confidence = var[second_90] / fmaxf(var[best_90], VAR_MIN);
	}

	result.best_std = sqrtf(var[result.best_rotation]);
	result.valid = (result.confidence > CONFIDENCE_RIGHT_ANGLE) && (result.best_std < MAX_STD_GAUSS);

	return result;
}

} // namespace mag_rotation_detection
