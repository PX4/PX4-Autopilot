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

#include "LosEstimator.hpp"

#include <mathlib/mathlib.h>
#include <float.h>

using namespace matrix;

namespace through
{

void AttitudeBuffer::push(uint64_t t, const Quatf &q)
{
	_samples[_head] = Sample{t, q};
	_head = (_head + 1) % SIZE;

	if (_count < SIZE) {
		++_count;
	}
}

uint64_t AttitudeBuffer::latestTime() const
{
	return (_count > 0) ? _samples[(_head + SIZE - 1) % SIZE].t : 0;
}

bool AttitudeBuffer::lookup(uint64_t t, uint64_t max_dt, Quatf &q) const
{
	// closest samples at or before t and at or after t
	int before = -1;
	int after = -1;

	for (int i = 0; i < _count; ++i) {
		const uint64_t ts = _samples[i].t;

		if (ts <= t && (before < 0 || ts > _samples[before].t)) {
			before = i;
		}

		if (ts >= t && (after < 0 || ts < _samples[after].t)) {
			after = i;
		}
	}

	const bool before_ok = (before >= 0) && (t - _samples[before].t <= max_dt);
	const bool after_ok = (after >= 0) && (_samples[after].t - t <= max_dt);

	if (before_ok && after_ok) {
		const Sample &a = _samples[before];
		const Sample &b = _samples[after];

		if (b.t == a.t) {
			q = a.q;
			return true;
		}

		// normalized linear interpolation (samples are a few ms apart), on the same hemisphere
		const float s = static_cast<float>(t - a.t) / static_cast<float>(b.t - a.t);
		const Quatf qb = (a.q.dot(b.q) < 0.f) ? Quatf(-b.q) : b.q;
		q = Quatf(a.q * (1.f - s) + qb * s);
		q.normalize();
		return true;
	}

	if (before_ok) {
		q = _samples[before].q;
		return true;
	}

	if (after_ok) {
		q = _samples[after].q;
		return true;
	}

	return false;
}

void LosEstimator::reset()
{
	_u = Vector3f{1.f, 0.f, 0.f};
	_u_dot.zero();
	_valid = false;
	_initialized = false;
	_have_prev = false;
	_last_sample_time = 0;
	_attitude_source = AttitudeSource::None;
}

Vector3f LosEstimator::cameraRay(float pan, float tilt)
{
	return Vector3f{1.f, tanf(pan), tanf(tilt)}.normalized();
}

bool LosEstimator::update(const Measurement &measurement)
{
	if (!measurement.valid || !PX4_ISFINITE(measurement.pan) || !PX4_ISFINITE(measurement.tilt)) {
		// keep u, pause the filter: the next valid sample re-initializes the previous value
		_valid = false;
		_have_prev = false;
		return false;
	}

	if (_initialized && measurement.timestamp_sample <= _last_sample_time) {
		return false; // same or older image
	}

	// Step 1: camera attitude at the capture time
	Quatf q_cam;

	if (_gimbal_buffer.lookup(measurement.timestamp_sample, _params.max_attitude_dt, q_cam)) {
		_attitude_source = AttitudeSource::GimbalFeedback;

	} else if (_fallback_buffer.lookup(measurement.timestamp_sample, _params.max_attitude_dt, q_cam)) {
		_attitude_source = AttitudeSource::Fallback;

	} else {
		_attitude_source = AttitudeSource::None;
		_valid = false;
		_have_prev = false;
		return false;
	}

	const Vector3f u = q_cam.rotateVector(cameraRay(measurement.pan, measurement.tilt)).normalized();

	// Step 2: LOS rate from consecutive images
	const float dt = (measurement.timestamp_sample - _last_sample_time) * 1e-6f;

	if (_have_prev && dt > FLT_EPSILON && dt < _params.max_vision_dt) {
		const Vector3f u_dot_raw = (u - _u) / dt;
		const float tau = 1.f / (2.f * M_PI_F * math::max(_params.lpf_cutoff_hz, 0.01f));
		const float alpha = dt / (dt + tau);
		Vector3f u_dot = _u_dot + alpha * (u_dot_raw - _u_dot);
		u_dot -= u_dot.dot(u) * u; // only the component perpendicular to the LOS
		_u_dot = u_dot;
	}

	_u = u;
	_q_cam = q_cam;
	_last_sample_time = measurement.timestamp_sample;
	_valid = true;
	_initialized = true;
	_have_prev = true;
	return true;
}

float LosEstimator::elevation() const
{
	return -asinf(math::constrain(_u(2), -1.f, 1.f));
}

float LosEstimator::azimuth() const
{
	return atan2f(_u(1), _u(0));
}

} // namespace through
