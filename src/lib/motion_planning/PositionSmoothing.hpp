/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#pragma once

#include <cmath>
#include <motion_planning/VelocitySmoothing.hpp>
#include <motion_planning/VelocitySmoothing2D.hpp>
#include <mathlib/mathlib.h>

#include <matrix/matrix/math.hpp>
#include <px4_defines.h>

using matrix::Vector2f;
using matrix::Vector3f;

/**
 * @brief Class that generates setpoints for a smooth trajectory
 * to position waypoints.
 *
 * This is achieved by first generating an unsmoothed velocity setpoint
 * which then gets smoothed using the VelocitySmoothing library.
 */
class PositionSmoothing
{

public:
	/**
	 * @brief Result type of the position smoothing algorithm.
	 * The included values shall then be used as setpoints for
	 * the controllers.
	 */
	struct PositionSmoothingSetpoints {
		Vector3f jerk;
		Vector3f acceleration;
		Vector3f velocity;
		Vector3f position;
		Vector3f unsmoothed_velocity;
	};

	/**
	 * @brief Generates new setpoints for jerk, acceleration, velocity and position
	 * to reach the given waypoint triplet smoothly from current position
	 *
	 * @param position Current position of the vehicle
	 * @param waypoints 0: Past waypoint, 1: target, 2: Target after next target
	 * @param feedforward_velocity FF velocity
	 * @param delta_time Time since last invocation of the function
	 * @param force_zero_velocity_setpoint Force vehicle to stop. Generate trajectory that ends with still vehicle.
	 * @param out_setpoints Output of the generated setpoints
	 */
	inline void generateSetpoints(
		const Vector3f &position,
		const Vector3f(&waypoints)[3],
		const Vector3f &feedforward_velocity,
		float delta_time,
		bool force_zero_velocity_setpoint,
		PositionSmoothingSetpoints &out_setpoints
	)
	{
		_generateSetpoints(position, waypoints, false, feedforward_velocity, delta_time, force_zero_velocity_setpoint,
				   out_setpoints);
	}

	/**
	 * @brief Generates new setpoints for jerk, acceleration, velocity and position
	 * to reach the given waypoint smoothly from current position.
	 *
	 * @param position Current position of the vehicle
	 * @param waypoint desired waypoint
	 * @param feedforward_velocity FF velocity
	 * @param delta_time Time since last invocation of the function
	 * @param force_zero_velocity_setpoint Force vehicle to stop. Generate trajectory that ends with still vehicle.
	 * @param out_setpoints Output of the generated setpoints
	 */
	inline void generateSetpoints(
		const Vector3f &position,
		const Vector3f &waypoint,
		const Vector3f &feedforward_velocity,
		float delta_time,
		bool force_zero_velocity_setpoint,
		PositionSmoothingSetpoints &out_setpoints
	)
	{
		Vector3f waypoints[3] = {waypoint, waypoint, waypoint};
		_generateSetpoints(position, waypoints, true, feedforward_velocity, delta_time, force_zero_velocity_setpoint,
				   out_setpoints);
	}


	/**
	 * @brief Reset internal state to the given values
	 *
	 * @param acceleration current acceleration
	 * @param velocity current velocity
	 * @param position current position
	 */
	void reset(const Vector3f &acceleration, const Vector3f &velocity, const Vector3f &position)
	{
		_trajectory_xy.reset(acceleration.xy(), velocity.xy(), position.xy());
		_trajectory_z.reset(acceleration(2), velocity(2), position(2));
	}

	/**
	 * @return Vector3f Current trajectory jerk
	 */
	inline Vector3f getCurrentJerk() const
	{
		return {_trajectory_xy.getCurrentJerk()(0), _trajectory_xy.getCurrentJerk()(1), _trajectory_z.getCurrentJerk()};
	}

	/**
	 * @return float Current trajectory acceleration in X
	 */
	inline float getCurrentAccelerationX() const
	{
		return _trajectory_xy.getCurrentAcceleration()(0);
	}

	/**
	 * @return float Current trajectory acceleration in Y
	 */
	inline float getCurrentAccelerationY() const
	{
		return _trajectory_xy.getCurrentAcceleration()(1);
	}

	/**
	 * @return float Current trajectory acceleration in Z
	 */
	inline float getCurrentAccelerationZ() const
	{
		return _trajectory_z.getCurrentAcceleration();
	}

	/**
	 * @return float Current trajectory acceleration
	 */
	inline Vector3f getCurrentAcceleration() const
	{
		return {getCurrentAccelerationX(), getCurrentAccelerationY(), getCurrentAccelerationZ()};
	}

	/**
	 * @return float Current trajectory acceleration in X and Y
	 */
	inline Vector2f getCurrentAccelerationXY() const
	{
		return {getCurrentAccelerationX(), getCurrentAccelerationY()};
	}

	/**
	 * @return float Current trajectory velocity in X
	 */
	inline float getCurrentVelocityX() const
	{
		return _trajectory_xy.getCurrentVelocity()(0);
	}

	/**
	 * @return float Current trajectory velocity in Y
	 */
	inline float getCurrentVelocityY() const
	{
		return _trajectory_xy.getCurrentVelocity()(1);
	}

	/**
	 * @return float Current trajectory velocity in Z
	 */
	inline float getCurrentVelocityZ() const
	{
		return _trajectory_z.getCurrentVelocity();
	}

	/**
	 * @return float Current trajectory velocity
	 */
	inline Vector3f getCurrentVelocity() const
	{
		return {getCurrentVelocityX(), getCurrentVelocityY(), getCurrentVelocityZ()};
	}

	/**
	 * @return float Current trajectory velocity in X and Y
	 */
	inline Vector2f getCurrentVelocityXY() const
	{
		return {getCurrentVelocityX(), getCurrentVelocityY()};
	}

	/**
	 * @return float Current trajectory position in X
	 */
	inline float getCurrentPositionX() const
	{
		return _trajectory_xy.getCurrentPosition()(0);
	}

	/**
	 * @return float Current trajectory position in Y
	 */
	inline float getCurrentPositionY() const
	{
		return _trajectory_xy.getCurrentPosition()(1);
	}

	/**
	 * @return float Current trajectory position in Z
	 */
	inline float getCurrentPositionZ() const
	{
		return _trajectory_z.getCurrentPosition();
	}

	/**
	 * @return float Current trajectory position
	 */
	inline Vector3f getCurrentPosition() const
	{
		return {getCurrentPositionX(), getCurrentPositionY(), getCurrentPositionZ()};
	}

	/**
	 * @param jerk maximum jerk for generated trajectory
	 */
	inline void setMaxJerkXY(float jerk)
	{
		_trajectory_xy.setMaxJerk(jerk);
	}

	/**
	 * @param jerk maximum jerk for generated trajectory
	 */
	inline void setMaxJerkZ(float jerk)
	{
		_trajectory_z.setMaxJerk(jerk);
	}

	/**
	 * @param jerk maximum jerk for generated trajectory
	 */
	inline void setMaxJerk(const float jerk)
	{
		_trajectory_xy.setMaxJerk(jerk);
		_trajectory_z.setMaxJerk(jerk);
	}

	/**
	 * @param accel maximum acceleration for generated trajectory
	 */
	inline void setMaxAccelerationXY(float accel)
	{
		_trajectory_xy.setMaxAccel(accel);
	}

	/**
	 * @param accel maximum acceleration for generated trajectory
	 */
	inline void setMaxAccelerationZ(float accel)
	{
		_trajectory_z.setMaxAccel(accel);
	}

	/**
	 * @param accel maximum acceleration for generated trajectory
	 */
	inline void setMaxAcceleration(const Vector3f &accel)
	{
		_trajectory_xy.setMaxAccel(math::min(accel(0), accel(1)));
		_trajectory_z.setMaxAccel(accel(2));
	}

	/**
	 * @param vel maximum velocity for generated trajectory
	 */
	inline void setMaxVelocityXY(float vel)
	{
		_trajectory_xy.setMaxVel(vel);
	}

	/**
	 * @param vel maximum velocity for generated trajectory
	 */
	inline void setMaxVelocityZ(float vel)
	{
		_trajectory_z.setMaxVel(vel);
	}

	/**
	 * @param vel maximum velocity for generated trajectory
	 */
	inline void setMaxVelocity(const Vector3f &vel)
	{
		_trajectory_xy.setMaxVel(math::min(vel(0), vel(1)));
		_trajectory_z.setMaxVel(vel(2));
	}

	/**
	 * @param error Maximum horizontal error allowed by the trajectory generator. Often set to param MPC_XY_ERR_MAX
	 */
	inline void setMaxAllowedHorizontalError(float error)
	{
		_max_allowed_horizontal_error = error;
	}

	/**
	 * @param error Maximum vertical error allowed by the trajectory generator. Set to param MPC_Z_ERR_MAX.
	 * Zero disables vertical time-stretch — trajectory integration is only slowed by horizontal lag.
	 * A finite value (param default is 1 m) makes the smoother slow down when the virtual trajectory leads
	 * the drone in Z, preventing a noisy altitude reference from walking the commanded z away from the drone.
	 */
	inline void setMaxAllowedVerticalError(float error)
	{
		_max_allowed_vertical_error = error;
	}

	/**
	 * @param radius  Altitude Acceptance Radius. Often set to NAV_MC_ALT_RAD
	 */
	inline void setVerticalAcceptanceRadius(float radius)
	{
		_vertical_acceptance_radius = radius;
	}

	/**
	 * @param speed vehicle cruise speed
	 */
	inline void setCruiseSpeed(float speed)
	{
		_cruise_speed = speed;
	}

	/**
	 * @param gain Proportional gain for horizontal trajectory position error. Set to MPC_XY_TRAJ_P
	 */
	inline void setHorizontalTrajectoryGain(float gain)
	{
		_horizontal_trajectory_gain = gain;
	}

	/**
	 * @param radius target acceptance radius
	 */
	inline void setTargetAcceptanceRadius(float radius)
	{
		_target_acceptance_radius = radius;
	}

	/**
	 * @brief Set the current position in the trajectory to the given value.
	 * Any coordinate with NAN will not be set
	 *
	 * @param position
	 */
	void forceSetPosition(const Vector3f &position)
	{
		if (Vector2f(position).isAllFinite()) {
			_trajectory_xy.setCurrentPosition(position.xy());
		}

		if (PX4_ISFINITE(position(2))) {
			_trajectory_z.setCurrentPosition(position(2));
		}
	}

	/**
	 * @brief Set the current velocity in the trajectory to the given value.
	 * Any coordinate with NAN will not be set
	 *
	 * @param velocity
	 */
	void forceSetVelocity(const Vector3f &velocity)
	{
		if (Vector2f(velocity).isAllFinite()) {
			_trajectory_xy.setCurrentVelocity(velocity.xy());
		}

		if (PX4_ISFINITE(velocity(2))) {
			_trajectory_z.setCurrentVelocity(velocity(2));
		}
	}

	/**
	 * @brief Set the current acceleration in the trajectory to the given value.
	 * Any coordinate with NAN will not be set
	 *
	 * @param acceleration
	 */
	void forceSetAcceleration(const Vector3f &acceleration)
	{
		if (Vector2f(acceleration).isAllFinite()) {
			_trajectory_xy.setCurrentAcceleration(acceleration.xy());
		}

		if (PX4_ISFINITE(acceleration(2))) {
			_trajectory_z.setCurrentAcceleration(acceleration(2));
		}
	}


private:
	/* params, only modified from external */
	float _max_allowed_horizontal_error{0.f};
	float _max_allowed_vertical_error{0.f};
	float _vertical_acceptance_radius{0.f};
	float _cruise_speed{0.f};
	float _horizontal_trajectory_gain{0.f};
	float _target_acceptance_radius{0.f};


	/* Internal state */
	VelocitySmoothing2D _trajectory_xy; ///< Horizontal trajectory
	VelocitySmoothing _trajectory_z; ///< Vertical trajectory
	float _max_speed_previous{0.f};

	/* Internal functions */
	bool _isTurning(const Vector3f &target) const;

	void _generateSetpoints(
		const Vector3f &position,
		const Vector3f(&waypoints)[3],
		bool is_single_waypoint,
		const Vector3f &feedforward_velocity,
		float delta_time,
		bool force_zero_velocity_setpoint,
		PositionSmoothingSetpoints &out_setpoints
	);

	const Vector3f _generateVelocitySetpoint(const Vector3f &position, const Vector3f(&waypoints)[3],
			bool is_single_waypoint,
			const Vector3f &feedforward_velocity_setpoint);
	const Vector3f _getL1Point(const Vector3f &position, const Vector3f(&waypoints)[3]) const;
	const Vector3f _getCrossingPoint(const Vector3f &position, const Vector3f(&waypoints)[3]) const;
	float _getMaxXYSpeed(const Vector3f(&waypoints)[3]) const;
	float _getMaxZSpeed(const Vector3f(&waypoints)[3]) const;

	void _generateTrajectory(
		const Vector3f &position,
		const Vector3f &velocity_setpoint,
		float delta_time,
		PositionSmoothingSetpoints &out_setpoints);
};
