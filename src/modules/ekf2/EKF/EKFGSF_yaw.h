/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
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

#ifndef EKF_EKFGSF_YAW_H
#define EKF_EKFGSF_YAW_H

#include <lib/geo/geo.h>
#include <matrix/math.hpp>
#include <mathlib/mathlib.h>

#include "common.h"

using matrix::AxisAnglef;
using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Matrix3f;
using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector3f;
using matrix::wrap_pi;

static constexpr uint8_t N_MODELS_EKFGSF = 5;

using namespace estimator;

class EKFGSF_yaw
{
public:
	EKFGSF_yaw();

	// Update Filter States - this should be called whenever new IMU data is available
	void predict(const imuSample &imu_sample, bool in_air = false);

	void fuseVelocity(const Vector2f &vel_NE, // NE velocity measurement (m/s)
			  float vel_accuracy,	  // 1-sigma accuracy of velocity measurement (m/s)
			  bool in_air);

	void setTrueAirspeed(float true_airspeed) { _true_airspeed = true_airspeed; }

	void setGyroBias(const Vector3f &imu_gyro_bias)
	{
		// Initialise to gyro bias estimate from main filter because there could be a large
		// uncorrected rate gyro bias error about the gravity vector
		if (!_ahrs_ekf_gsf_tilt_aligned || !_ekf_gsf_vel_fuse_started) {
			// init gyro bias for each model
			for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index++) {
				_ahrs_ekf_gsf[model_index].gyro_bias = imu_gyro_bias;
			}
		}
	}

	// get solution data for logging
	bool getLogData(float *yaw_composite,
			float *yaw_composite_variance,
			float yaw[N_MODELS_EKFGSF],
			float innov_VN[N_MODELS_EKFGSF],
			float innov_VE[N_MODELS_EKFGSF],
			float weight[N_MODELS_EKFGSF]) const;

	bool isActive() const { return _ekf_gsf_vel_fuse_started; }

	float getYaw() const { return _gsf_yaw; }
	float getYawVar() const { return _gsf_yaw_variance; }

	void reset();

private:

	// Parameters - these could be made tuneable
	const float _gyro_noise{1.0e-1f}; 	// yaw rate noise used for covariance prediction (rad/sec)
	const float _accel_noise{2.0f};		// horizontal accel noise used for covariance prediction (m/sec**2)
	const float _tilt_gain{0.2f};		// gain from tilt error to gyro correction for complementary filter (1/sec)
	const float _gyro_bias_gain{0.04f};	// gain applied to integral of gyro correction for complementary filter (1/sec)

	// Declarations used by the bank of N_MODELS_EKFGSF AHRS complementary filters
	float _true_airspeed{NAN};	// true airspeed used for centripetal accel compensation (m/s)

	struct {
		Dcmf R{matrix::eye<float, 3>()}; // matrix that rotates a vector from body to earth frame
		Vector3f gyro_bias{};            // gyro bias learned and used by the quaternion calculation
	} _ahrs_ekf_gsf[N_MODELS_EKFGSF] {};

	bool _ahrs_ekf_gsf_tilt_aligned{false};  // true the initial tilt alignment has been calculated
	Vector3f _ahrs_accel{0.f, 0.f, 0.f};     // low pass filtered body frame specific force vector used by AHRS calculation (m/s/s)

	// calculate the gain from gravity vector misalingment to tilt correction to be used by all AHRS filters
	float ahrsCalcAccelGain() const;

	// update specified AHRS rotation matrix using IMU and optionally true airspeed data
	void ahrsPredict(const uint8_t model_index, const Vector3f &delta_ang, const float delta_ang_dt);

	// align all AHRS roll and pitch orientations using IMU delta velocity vector
	void ahrsAlignTilt(const Vector3f &delta_vel);

	// align all AHRS yaw orientations to initial values
	void ahrsAlignYaw();

	// Efficient propagation of a delta angle in body frame applied to the body to earth frame rotation matrix
	Matrix3f ahrsPredictRotMat(const Matrix3f &R, const Vector3f &g);

	// Declarations used by a bank of N_MODELS_EKFGSF EKFs

	struct _ekf_gsf_struct {
		matrix::Vector3f X{};                       // Vel North (m/s),  Vel East (m/s), yaw (rad)s
		matrix::SquareMatrix<float, 3> P{};         // covariance matrix
		matrix::SquareMatrix<float, 2> S_inverse{}; // inverse of the innovation covariance matrix
		float S_det_inverse{};                      // inverse of the innovation covariance matrix determinant
		matrix::Vector2f innov{};                   // Velocity N,E innovation (m/s)
	} _ekf_gsf[N_MODELS_EKFGSF] {};

	bool _ekf_gsf_vel_fuse_started{}; // true when the EKF's have started fusing velocity data and the prediction and update processing is active

	// initialise states and covariance data for the GSF and EKF filters
	void initialiseEKFGSF(const Vector2f &vel_NE, const float vel_accuracy);

	// predict state and covariance for the specified EKF using inertial data
	void predictEKF(const uint8_t model_index, const Vector3f &delta_ang, const float delta_ang_dt,
			const Vector3f &delta_vel, const float delta_vel_dt, bool in_air = false);

	// update state and covariance for the specified EKF using a NE velocity measurement
	// return false if update failed
	bool updateEKF(const uint8_t model_index, const Vector2f &vel_NE, const float vel_accuracy);

	inline float sq(float x) const { return x * x; };

	// Declarations used by the Gaussian Sum Filter (GSF) that combines the individual EKF yaw estimates

	matrix::Vector<float, N_MODELS_EKFGSF> _model_weights{};
	float _gsf_yaw{}; 		// yaw estimate (rad)
	float _gsf_yaw_variance{}; 	// variance of yaw estimate (rad^2)

	// return the probability of the state estimate for the specified EKF assuming a gaussian error distribution
	float gaussianDensity(const uint8_t model_index) const;
};
#endif // !EKF_EKFGSF_YAW_H
