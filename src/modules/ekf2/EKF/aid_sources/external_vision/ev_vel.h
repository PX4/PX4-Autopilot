/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file ev_vel.h
 * Handling different types of External Vision velocity measurements
 */

#ifndef ExternalVisionVel_H
#define ExternalVisionVel_H

#include "ekf.h"
#include <uORB/topics/estimator_aid_source3d.h>

class ExternalVisionVel
{
public:
	ExternalVisionVel(Ekf &ekf_instance, const extVisionSample &vision_sample, const float ev_vel_noise,
			  const imuSample &imu_sample) : _ekf(ekf_instance), _sample(vision_sample)
	{
		_min_variance = sq(ev_vel_noise);
		const Vector3f angular_velocity = imu_sample.delta_ang / imu_sample.delta_ang_dt - _ekf._state.gyro_bias;
		Vector3f position_offset_body = _ekf._params.ev_pos_body - _ekf._params.imu_pos_body;
		_velocity_offset_body = angular_velocity % position_offset_body;
	}

	virtual ~ExternalVisionVel() = default;
	virtual bool fuseVelocity(estimator_aid_source3d_s &aid_src, float gate)
	{
		_ekf.fuseLocalFrameVelocity(aid_src, aid_src.timestamp, _measurement,
					    _measurement_var, gate);
		return aid_src.fused;

	}

	virtual void resetVelocity()
	{
		_ekf.resetVelocityTo(_measurement, _measurement_var);
	}

	void resetVerticalVelocity()
	{
		_ekf.resetVerticalVelocityTo(_measurement(2), _measurement_var(2));
	}

	void enforceMinimumVariance()
	{
		_min_variance = math::max(_min_variance, sq(0.01f));

		for (int i = 0; i < 3; ++i) {
			_measurement_var(i) = math::max(_measurement_var(i), _min_variance);
		}
	}

	Ekf &_ekf;
	const extVisionSample &_sample;
	float _min_variance;
	Vector3f _measurement{NAN, NAN, NAN};
	Vector3f _measurement_var;
	Vector3f _velocity_offset_body;
};

class EvVelBodyFrameFrd : public ExternalVisionVel
{
public:
	EvVelBodyFrameFrd(Ekf &ekf_instance, extVisionSample &vision_sample, const float ev_vel_noise,
			  const imuSample &imu_sample) :
		ExternalVisionVel(ekf_instance, vision_sample, ev_vel_noise, imu_sample)
	{
		_measurement = _sample.vel - _velocity_offset_body;
		_measurement_var = _sample.velocity_var;
		enforceMinimumVariance();
	}

	void resetVelocity() override
	{
		const matrix::SquareMatrix3f rotated_variance = _ekf._R_to_earth * matrix::diag(
					_measurement_var) * _ekf._R_to_earth.transpose();
		// bump the variance to decrease cross-correlation and increase uncertainty of velocity
		const Vector3f measurement_variance_ekf_frame = rotated_variance.diag() * 5.f;
		_ekf.resetVelocityTo(_ekf._R_to_earth * _measurement, measurement_variance_ekf_frame);
	}

	void resetVerticalVelocity()
	{
		const matrix::SquareMatrix3f rotated_variance = _ekf._R_to_earth * matrix::diag(
					_measurement_var) * _ekf._R_to_earth.transpose();
		// bump the variance to decrease cross-correlation and increase uncertainty of velocity
		const Vector3f measurement_variance_ekf_frame = rotated_variance.diag() * 5.f;
		_ekf.resetVerticalVelocityTo((_ekf._R_to_earth * _measurement)(2, 0),
					     measurement_variance_ekf_frame(2));
	}

	bool fuseVelocity(estimator_aid_source3d_s &aid_src, float gate) override
	{
		_ekf.fuseBodyFrameVelocity(aid_src, _sample.time_us, _measurement,
					   _measurement_var, gate);
		return aid_src.fused;
	}
};

class EvVelLocalFrameNed : public ExternalVisionVel
{
public:
	EvVelLocalFrameNed(Ekf &ekf_instance, extVisionSample &vision_sample, const float ev_vel_noise,
			   const imuSample &imu_sample) :
		ExternalVisionVel(ekf_instance, vision_sample, ev_vel_noise, imu_sample)
	{
		const Vector3f velocity_offset_earth = _ekf._R_to_earth * _velocity_offset_body;

		if (_ekf._control_status.flags.yaw_align) {
			_measurement = _sample.vel - velocity_offset_earth;
			_measurement_var = _sample.velocity_var;
			enforceMinimumVariance();
		}
	}
};

class EvVelLocalFrameFrd : public ExternalVisionVel
{
public:
	EvVelLocalFrameFrd(Ekf &ekf_instance, extVisionSample &vision_sample, const float ev_vel_noise,
			   const imuSample &imu_sample) :
		ExternalVisionVel(ekf_instance,	vision_sample, ev_vel_noise, imu_sample)
	{
		const Vector3f velocity_offset_earth = _ekf._R_to_earth * _velocity_offset_body;

		if (_ekf._control_status.flags.ev_yaw) {
			// Using EV frame
			_measurement = _sample.vel - velocity_offset_earth;
			_measurement_var = _sample.velocity_var;

		} else {
			// Rotate EV to the EKF reference frame
			const Dcmf rotation_ev_to_ekf = Dcmf(_ekf._ev_q_error_filt.getState());
			_measurement = rotation_ev_to_ekf * _sample.vel - velocity_offset_earth;
			_measurement_var = matrix::SquareMatrix3f(rotation_ev_to_ekf * matrix::diag(
						   _sample.velocity_var) * rotation_ev_to_ekf.transpose()).diag();
			_min_variance = math::max(_min_variance, _sample.orientation_var.max());
		}

		enforceMinimumVariance();
	}
};

#endif // ExternalVisionVel_H
