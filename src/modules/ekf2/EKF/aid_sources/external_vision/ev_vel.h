/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
	ExternalVisionVel(Ekf &ekf_instance, const extVisionSample &vision_sample, const imuSample &imu_sample)
		: ekf(ekf_instance), sample(vision_sample)
	{

		const Vector3f angular_velocity = imu_sample.delta_ang / imu_sample.delta_ang_dt - ekf._state.gyro_bias;
		position_offset_body = ekf._params.ev_pos_body - ekf._params.imu_pos_body;
		velocity_offset_body = angular_velocity % position_offset_body;
	}

	virtual ~ExternalVisionVel() = default;
	virtual bool setMeasurement() = 0;
	virtual bool fuseVelocity(estimator_aid_source3d_s &aid_src, float gate)
	{
		ekf.fuseLocalFrameVelocity(aid_src, aid_src.timestamp, measurement,
					   measurement_var, gate);
		return aid_src.fused;

	}

	virtual void resetVelocity()
	{
		ekf.resetVelocityTo(measurement, measurement_var);
	}

	void resetVerticalVelocity()
	{
		ekf.resetVerticalVelocityTo(measurement(2), measurement_var(2));
	}

	void enforceMinimumVariance()
	{
		min_variance = math::max(min_variance, sq(0.01f));

		for (int i = 0; i < 3; ++i) {
			measurement_var(i) = math::max(measurement_var(i), min_variance);
		}
	}

	Ekf &ekf;
	const extVisionSample &sample;
	float min_variance;
	Vector3f measurement;
	Vector3f measurement_var;
	Vector3f position_offset_body;
	Vector3f velocity_offset_body;
	Vector3f velocity_offset_earth;
};

class BodyFrameEV : public ExternalVisionVel
{
public:
	BodyFrameEV(Ekf &ekf_instance, extVisionSample &vision_sample, const imuSample &imu_sample) :
		ExternalVisionVel(ekf_instance, vision_sample, imu_sample) {}

	void resetVelocity() override
	{
		const matrix::SquareMatrix<float, 3> rotated_variance = ekf._R_to_earth * matrix::diag(
					measurement_var) * ekf._R_to_earth.transpose();
		const Vector3f measurement_variance_ekf_frame = rotated_variance.diag() * 5.f; // variance bump
		ekf.resetVelocityTo(ekf._R_to_earth * measurement, measurement_variance_ekf_frame);
	}

	void resetVerticalVelocity()
	{
		const matrix::SquareMatrix<float, 3> rotated_variance = ekf._R_to_earth * matrix::diag(
					measurement_var) * ekf._R_to_earth.transpose();
		const Vector3f measurement_variance_ekf_frame = rotated_variance.diag() * 5.f; // variance bump
		ekf.resetVerticalVelocityTo((ekf._R_to_earth * measurement)(2, 0),
					    measurement_variance_ekf_frame(2));
	}

	bool setMeasurement() override
	{
		measurement = sample.vel - velocity_offset_body;
		measurement_var = sample.velocity_var;
		enforceMinimumVariance();
		return true;
	}

	bool fuseVelocity(estimator_aid_source3d_s &aid_src, float gate) override
	{
		ekf.fuseBodyFrameVelocity(aid_src, sample.time_us, measurement,
					  measurement_var, gate);
		return aid_src.fused;
	}
};

class NEDLocalFrameEV : public ExternalVisionVel
{
public:
	NEDLocalFrameEV(Ekf &ekf_instance, extVisionSample &vision_sample, const imuSample &imu_sample) :
		ExternalVisionVel(ekf_instance, vision_sample, imu_sample) {}

	bool setMeasurement() override
	{
		if (ekf._control_status.flags.yaw_align) {
			measurement = sample.vel - velocity_offset_body;
			measurement_var = sample.velocity_var;
			enforceMinimumVariance();
			return true;
		}

		return false;
	}

};

class FRDLocalFrameEV : public ExternalVisionVel
{
public:
	FRDLocalFrameEV(Ekf &ekf_instance, extVisionSample &vision_sample, const imuSample &imu_sample) :
		ExternalVisionVel(ekf_instance,	vision_sample, imu_sample) {}

	bool setMeasurement() override
	{
		velocity_offset_earth = ekf._R_to_earth * velocity_offset_body;

		if (ekf._control_status.flags.ev_yaw) {
			// Using EV frame
			measurement = sample.vel - velocity_offset_earth;
			measurement_var = sample.velocity_var;

		} else {
			// Rotate EV to the EKF reference frame
			const Dcmf rotation_ev_to_ekf = Dcmf(ekf._ev_q_error_filt.getState());
			measurement = rotation_ev_to_ekf * sample.vel - velocity_offset_earth;
			measurement_var = matrix::SquareMatrix3f(rotation_ev_to_ekf * matrix::diag(
						  sample.velocity_var) * rotation_ev_to_ekf.transpose()).diag();
			min_variance = math::max(min_variance, sample.orientation_var.max());
		}

		printf("var pre %f, %f", (double)measurement_var(0), (double)measurement_var(1));

		enforceMinimumVariance();
		printf("var post %f, %f", (double)measurement_var(0), (double)measurement_var(1));
		return true;
	}

};

#endif // ExternalVisionVel_H
