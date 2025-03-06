/****************************************************************************
 *
 *   Copyright (c) 2022-2024 PX4. All rights reserved.
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

#ifndef EKF_OUTPUT_PREDICTOR_H
#define EKF_OUTPUT_PREDICTOR_H

#include <matrix/math.hpp>

#include "../RingBuffer.h"

#include <lib/geo/geo.h>
#include <lib/lat_lon_alt/lat_lon_alt.hpp>

class OutputPredictor
{
public:
	OutputPredictor()
	{
		reset();
	};

	~OutputPredictor() = default;

	// modify output filter to match the the EKF state at the fusion time horizon
	void alignOutputFilter(const matrix::Quatf &quat_state, const matrix::Vector3f &vel_state,
			       const LatLonAlt &gpos_state);
	/*
	* Implement a strapdown INS algorithm using the latest IMU data at the current time horizon.
	* Buffer the INS states and calculate the difference with the EKF states at the delayed fusion time horizon.
	* Calculate delta angle, delta velocity and velocity corrections from the differences and apply them at the
	* current time horizon so that the INS states track the EKF states at the delayed fusion time horizon.
	* The inspiration for using a complementary filter to correct for time delays in the EKF
	* is based on the work by A Khosravian:
	* “Recursive Attitude Estimation in the Presence of Multi-rate and Multi-delay Vector Measurements”
	* A Khosravian, J Trumpf, R Mahony, T Hamel, Australian National University
	*/
	void calculateOutputStates(const uint64_t time_us, const matrix::Vector3f &delta_angle, const float delta_angle_dt,
				   const matrix::Vector3f &delta_velocity, const float delta_velocity_dt);

	void correctOutputStates(const uint64_t time_delayed_us,
				 const matrix::Quatf &quat_state, const matrix::Vector3f &vel_state, const LatLonAlt &gpos_state,
				 const matrix::Vector3f &gyro_bias, const matrix::Vector3f &accel_bias);

	void resetQuaternion(const matrix::Quatf &quat_change);

	void resetHorizontalVelocityTo(const matrix::Vector2f &delta_horz_vel);
	void resetVerticalVelocityTo(float delta_vert_vel);

	void resetLatLonTo(const double &new_latitude, const double &new_longitude);
	void resetAltitudeTo(float new_altitude, float vert_pos_change);

	void print_status();

	bool allocate(uint8_t size)
	{
		if (_output_buffer.allocate(size) && _output_vert_buffer.allocate(size)) {
			reset();
			return true;
		}

		return false;
	}

	void reset();

	const matrix::Quatf &getQuaternion() const { return _output_new.quat_nominal; }

	// get a yaw value solely based on bias-removed gyro integration
	float getUnaidedYaw() const { return _unaided_yaw; }

	// get the velocity of the body frame origin in local NED earth frame
	matrix::Vector3f getVelocity() const { return _output_new.vel - _vel_imu_rel_body_ned; }

	// get the mean velocity derivative in earth frame since reset (see `resetVelocityDerivativeAccumulation()`)
	matrix::Vector3f getVelocityDerivative() const;

	void resetVelocityDerivativeAccumulation();

	// get the derivative of the vertical position of the body frame origin in local NED earth frame
	float getVerticalPositionDerivative() const { return _output_vert_new.vert_vel - _vel_imu_rel_body_ned(2); }

	LatLonAlt getLatLonAlt() const
	{
		// rotate the position of the IMU relative to the boy origin into earth frame
		const matrix::Vector3f pos_offset_earth{_R_to_earth_now * _imu_pos_body};
		// subtract from the EKF position (which is at the IMU) to get position at the body origin
		return _global_ref + (_output_new.pos - pos_offset_earth);
	}

	// return an array containing the output predictor angular, velocity and position tracking
	// error magnitudes (rad), (m/sec), (m)
	const matrix::Vector3f &getOutputTrackingError() const { return _output_tracking_error; }

	void set_imu_offset(const matrix::Vector3f &offset) { _imu_pos_body = offset; }
	void set_pos_correction_tc(const float tau) { _pos_tau = tau; }
	void set_vel_correction_tc(const float tau) { _vel_tau = tau; }

private:

	/*
	* Calculate a correction to be applied to vert_vel that casues vert_vel_integ to track the EKF
	* down position state at the fusion time horizon using an alternative algorithm to what
	* is used for the vel and pos state tracking. The algorithm applies a correction to the vert_vel
	* state history and propagates vert_vel_integ forward in time using the corrected vert_vel history.
	* This provides an alternative vertical velocity output that is closer to the first derivative
	* of the position but does degrade tracking relative to the EKF state.
	*/
	void applyCorrectionToVerticalOutputBuffer(float vert_vel_correction, const float pos_ref_change);

	/*
	* Calculate corrections to be applied to vel and pos output state history.
	* The vel and pos state history are corrected individually so they track the EKF states at
	* the fusion time horizon. This option provides the most accurate tracking of EKF states.
	*/
	void applyCorrectionToOutputBuffer(const matrix::Vector3f &vel_correction, const matrix::Vector3f &pos_correction);

	// return the square of two floating point numbers - used in auto coded sections
	static constexpr float sq(float var) { return var * var; }

	struct outputSample {
		uint64_t         time_us{0};                       ///< timestamp of the measurement (uSec)
		matrix::Quatf    quat_nominal{1.f, 0.f, 0.f, 0.f}; ///< nominal quaternion describing vehicle attitude
		matrix::Vector3f vel{0.f, 0.f, 0.f};               ///< NED velocity estimate in earth frame (m/sec)
		matrix::Vector3f pos{0.f, 0.f, 0.f};               ///< NED position estimate in earth frame (m/sec)
	};

	struct outputVert {
		uint64_t time_us{0};          ///< timestamp of the measurement (uSec)
		float    vert_vel{0.f};       ///< Vertical velocity calculated using alternative algorithm (m/sec)
		float    vert_vel_integ{0.f}; ///< Integral of vertical velocity (m)
		float    dt{0.f};             ///< delta time (sec)
	};

	LatLonAlt _global_ref{0.0, 0.0, 0.f};

	RingBuffer<outputSample> _output_buffer{12};
	RingBuffer<outputVert> _output_vert_buffer{12};

	matrix::Vector3f _accel_bias{};
	matrix::Vector3f _gyro_bias{};

	float _dt_update_states_avg{0.005f};  // average imu update period in s
	float _dt_correct_states_avg{0.010f}; // average update rate of the ekf in s

	uint64_t _time_last_update_states_us{0}; ///< last time the output states were updated (uSec)
	uint64_t _time_last_correct_states_us{0}; ///< last time the output states were updated (uSec)

	// Output Predictor
	outputSample _output_new{};		// filter output on the non-delayed time horizon
	outputVert _output_vert_new{};		// vertical filter output on the non-delayed time horizon
	matrix::Matrix3f _R_to_earth_now{};		// rotation matrix from body to earth frame at current time
	matrix::Vector3f _vel_imu_rel_body_ned{};		// velocity of IMU relative to body origin in NED earth frame
	matrix::Vector3f _delta_vel_sum{};	// accumulation of delta velocity at the IMU in NED earth frame (m/s/s)
	float _delta_vel_dt{};			// duration of delta velocity integration (s)

	// output predictor states
	matrix::Vector3f _delta_angle_corr{};	///< delta angle correction vector (rad)
	matrix::Vector3f _vel_err_integ{};	///< integral of velocity tracking error (m)
	matrix::Vector3f _pos_err_integ{};	///< integral of position tracking error (m.s)

	matrix::Vector3f _output_tracking_error{}; ///< contains the magnitude of the angle, velocity and position track errors (rad, m/s, m)

	matrix::Vector3f _imu_pos_body{};                ///< xyz position of IMU in body frame (m)

	float _unaided_yaw{};

	// output complementary filter tuning
	float _vel_tau{0.25f};                   ///< velocity state correction time constant (1/sec)
	float _pos_tau{0.25f};                   ///< position state correction time constant (1/sec)
};

#endif // !EKF_OUTPUT_PREDICTOR_H
