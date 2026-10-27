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
 * @file LosEstimator.hpp
 *
 * Line-of-sight reconstruction of the THROUGH mode (spec section 7, steps 1-3).
 * No uORB I/O, so it can be unit tested in isolation.
 *
 * Step 1: u = normalize(R_cam_ned(t_sample) * r_c), r_c = normalize([1, tan(pan), tan(tilt)])
 *         The camera attitude at the image capture time comes from a buffer of gimbal
 *         attitude feedback (interpolated); if there is none, from a buffer of the
 *         fallback attitude R_nb * R_bc(gimbal pitch command).
 * Step 2: u_dot from consecutive image samples, low-pass filtered, component along u
 *         removed; omega_LOS = u x u_dot.
 * Step 3: closing velocity vc = v . u (target at rest).
 *
 * Frames: NED world frame, camera frame x forward (optical axis), y right, z down.
 */

#pragma once

#include <stdint.h>
#include <matrix/matrix/math.hpp>

namespace through
{

/** Timestamped attitude ring buffer, interpolated lookup. */
class AttitudeBuffer
{
public:
	static constexpr int SIZE = 64;  ///< 0.5 s at 125 Hz

	void reset() { _count = 0; _head = 0; }

	void push(uint64_t t, const matrix::Quatf &q);

	/**
	 * Attitude at t: interpolated between the samples around t, or the nearest one if t
	 * is outside the buffer. Fails if no sample is within max_dt of t.
	 */
	bool lookup(uint64_t t, uint64_t max_dt, matrix::Quatf &q) const;

	uint64_t latestTime() const;

private:
	struct Sample {
		uint64_t t;
		matrix::Quatf q;
	};

	Sample _samples[SIZE] {};
	int _head{0};   ///< next write index
	int _count{0};
};

class LosEstimator
{
public:
	enum class AttitudeSource : uint8_t {
		None = 0,
		GimbalFeedback,
		Fallback,
	};

	struct Params {
		float lpf_cutoff_hz{8.f};         ///< u_dot low-pass cutoff (THRU_LOS_LPF)
		uint64_t max_attitude_dt{20000};  ///< max time between image sample and attitude sample [us]
		float max_vision_dt{0.5f};        ///< max time between two images to differentiate u [s]
	};

	struct Measurement {
		uint64_t timestamp_sample{0};     ///< image capture time [us]
		float pan{0.f};                   ///< [rad] + = target right of image center
		float tilt{0.f};                  ///< [rad] + = target below image center
		bool valid{false};
	};

	void setParams(const Params &params) { _params = params; }

	/** Reset the LOS state and the u_dot filter (attitude buffers are kept). */
	void reset();

	/** Camera attitude (camera FRD -> NED) from the gimbal feedback. */
	void addGimbalAttitude(uint64_t t, const matrix::Quatf &q_cam_ned) { _gimbal_buffer.push(t, q_cam_ned); }

	/** Fallback camera attitude R_nb * R_bc(commanded gimbal angles). */
	void addFallbackAttitude(uint64_t t, const matrix::Quatf &q_cam_ned) { _fallback_buffer.push(t, q_cam_ned); }

	/**
	 * Process a vision measurement. Invalid measurements leave u unchanged and pause the
	 * u_dot filter; the next valid one restarts the differentiation without a jump.
	 * @return true if u was updated
	 */
	bool update(const Measurement &measurement);

	/** Unit target ray in the camera frame for the given image angles. */
	static matrix::Vector3f cameraRay(float pan, float tilt);

	bool valid() const { return _valid; }        ///< last measurement produced u
	bool initialized() const { return _initialized; } ///< u was computed at least once since reset
	uint64_t lastSampleTime() const { return _last_sample_time; }
	AttitudeSource attitudeSource() const { return _attitude_source; }

	const matrix::Vector3f &u() const { return _u; }
	const matrix::Vector3f &uDot() const { return _u_dot; }
	matrix::Vector3f omega() const { return _u.cross(_u_dot); }  ///< omega_LOS [rad/s]

	/** Closing velocity v . u (before any clamping). */
	float closingVelocity(const matrix::Vector3f &v_ned) const { return v_ned.dot(_u); }

	/** LOS elevation [rad], + = above the horizon. */
	float elevation() const;

	/** LOS azimuth [rad] (NED, from north). */
	float azimuth() const;

	/** Camera attitude used for the last valid measurement. */
	const matrix::Quatf &cameraAttitude() const { return _q_cam; }

private:
	Params _params{};

	AttitudeBuffer _gimbal_buffer{};
	AttitudeBuffer _fallback_buffer{};

	matrix::Vector3f _u{1.f, 0.f, 0.f};
	matrix::Vector3f _u_dot{};
	matrix::Quatf _q_cam{};

	bool _valid{false};
	bool _initialized{false};
	bool _have_prev{false};          ///< previous sample usable for differentiation
	uint64_t _last_sample_time{0};
	AttitudeSource _attitude_source{AttitudeSource::None};
};

} // namespace through
