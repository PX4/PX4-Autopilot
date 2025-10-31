/****************************************************************************
 *
 *   Copyright (c) 2018 - 2019 PX4 Development Team. All rights reserved.
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
 * @file LqrControl.hpp
 *
 * A cascaded position controller for position/velocity control only.
 */

#pragma once

#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <uORB/topics/actuator_controls.h>
// #include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

#include <lib/lqr/lqr.hpp>

using namespace matrix;


struct PositionControlStates {
	matrix::Vector3f position;
	matrix::Vector3f velocity;
	matrix::Vector3f attitude;
	matrix::Vector3f angular_velocity;
};


class LqrControl
{
public:

	LqrControl() = default;
	~LqrControl() = default;

	// /**
	//  * Set the maximum velocity to execute with feed forward and position control
	//  * @param vel_horizontal horizontal velocity limit
	//  * @param vel_up upwards velocity limit
	//  * @param vel_down downwards velocity limit
	//  */
	// void setVelocityLimits(const float vel_horizontal, const float vel_up, float vel_down);

	/**
	 * Set the minimum and maximum collective normalized thrust [0,1] that can be output by the controller
	 * @param min minimum thrust e.g. 0.1 or 0
	 * @param max maximum thrust e.g. 0.9 or 1
	 */
	void setThrustLimits(const float min, const float max);

	/**
	 * Set margin that is kept for horizontal control when prioritizing vertical thrust
	 * @param margin of normalized thrust that is kept for horizontal control e.g. 0.3
	 */
	void setHorizontalThrustMargin(const float margin);

	// /**
	//  * Set the maximum tilt angle in radians the output attitude is allowed to have
	//  * @param tilt angle in radians from level orientation
	//  */
	// void setTiltLimit(const float tilt) { _lim_tilt = tilt; }

	/**
	 * Set the normalized hover thrust
	 * @param thrust [0.1, 0.9] with which the vehicle hovers not acelerating down or up with level orientation
	 */
	void setHoverThrust(const float hover_thrust) { _hover_thrust = math::constrain(hover_thrust, 0.1f, 0.9f); }

	/**
	 * Update the hover thrust without immediately affecting the output
	 * by adjusting the integrator. This prevents propagating the dynamics
	 * of the hover thrust signal directly to the output of the controller.
	 */
	void updateHoverThrust(const float hover_thrust_new);

	/**
	 * Pass the current vehicle state to the controller
	 * @param PositionControlStates structure
	 */
	void setState(const PositionControlStates &states);

	/**
	 * Pass the desired setpoints
	 * Note: NAN value means no feed forward/leave state uncontrolled if there's no higher order setpoint.
	 * @param setpoint a vehicle_local_position_setpoint_s structure
	 */
	void setInputSetpoint(const vehicle_local_position_setpoint_s &setpoint);

	/**
	 * Apply P-position and PID-velocity controller that updates the member
	 * thrust, yaw- and yawspeed-setpoints.
	 * @see _thr_sp
	 * @see _yaw_sp
	 * @see _yawspeed_sp
	 * @param dt time in seconds since last iteration
	 * @return true if update succeeded and output setpoint is executable, false if not
	 */
	bool update(const float dt);

	/**
	 * Get the controllers output local position setpoint
	 * These setpoints are the ones which were executed on including PID output and feed-forward.
	 * The acceleration or thrust setpoints can be used for attitude control.
	 * @param local_position_setpoint reference to struct to fill up
	 */
	void getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const;

	void getActuatorControls(actuator_controls_s &actuator_controls) const;


private:
	bool _updateSuccessful();

	Lqr<12, 4> lqr;

	// Limits
	// float _lim_vel_horizontal{}; ///< Horizontal velocity limit with feed forward and position control
	// float _lim_vel_up{}; ///< Upwards velocity limit with feed forward and position control
	// float _lim_vel_down{}; ///< Downwards velocity limit with feed forward and position control
	float _lim_thr_min{}; ///< Minimum collective thrust allowed as output [-1,0] e.g. -0.9
	float _lim_thr_max{}; ///< Maximum collective thrust allowed as output [-1,0] e.g. -0.1
	float _lim_thr_xy_margin{}; ///< Margin to keep for horizontal control when saturating prioritized vertical thrust
	// float _lim_tilt{}; ///< Maximum tilt from level the output attitude is allowed to have

	float _hover_thrust{}; ///< Thrust [0.1, 0.9] with which the vehicle hovers not accelerating down or up with level orientation

	Vector<float, 12> _x;
	Vector<float, 12> _x_sp;
	Vector<float, 4> _u;

	// // States
	// matrix::Vector3f _pos; /**< current position */
	// matrix::Vector3f _vel; /**< current velocity */
	// matrix::Vector3f _vel_dot; /**< velocity derivative (replacement for acceleration estimate) */
	// matrix::Vector3f _vel_int; /**< integral term of the velocity controller */
	// float _yaw{}; /**< current heading */

	// // Setpoints
	// matrix::Vector3f _pos_sp; /**< desired position */
	// matrix::Vector3f _vel_sp; /**< desired velocity */
	// matrix::Vector3f _acc_sp; /**< desired acceleration */
	// matrix::Vector3f _thr_sp; /**< desired thrust */
	// float _yaw_sp{}; /**< desired heading */
	// float _yawspeed_sp{}; /** desired yaw-speed */
};
