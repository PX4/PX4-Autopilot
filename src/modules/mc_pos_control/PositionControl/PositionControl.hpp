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
 * @file PositionControl.hpp
 *
 * A cascaded position controller for position/velocity control only.
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_constraints.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include "vector"

struct PositionControlStates {
	matrix::Vector3f position;
	matrix::Vector3f velocity;
	matrix::Vector3f acceleration;
	float yaw;
};

/**
 * 	Core Position-Control for MC.
 * 	This class contains P-controller for position and
 * 	PID-controller for velocity.
 * 	Inputs:
 * 		vehicle position/velocity/yaw
 * 		desired set-point position/velocity/thrust/yaw/yaw-speed
 * 		constraints that are stricter than global limits
 * 	Output
 * 		thrust vector and a yaw-setpoint
 *
 * 	If there is a position and a velocity set-point present, then
 * 	the velocity set-point is used as feed-forward. If feed-forward is
 * 	active, then the velocity component of the P-controller output has
 * 	priority over the feed-forward component.
 *
 * 	A setpoint that is NAN is considered as not set.
 * 	If there is a position/velocity- and thrust-setpoint present, then
 *  the thrust-setpoint is ommitted and recomputed from position-velocity-PID-loop.
 */
class PositionControl
{
public:

	PositionControl() = default;
	~PositionControl() = default;

	/**
	 * Set the position control gains
	 * @param P 3D vector of proportional gains for x,y,z axis
	 */
	void setPositionGains(const matrix::Vector3f &P) { _gain_pos_p = P; }

	/**
	 * Set the velocity control gains
	 * @param P 3D vector of proportional gains for x,y,z axis
	 * @param I 3D vector of integral gains
	 * @param D 3D vector of derivative gains
	 */
	void setVelocityGains(const matrix::Vector3f &P, const matrix::Vector3f &I, const matrix::Vector3f &D);

	/**
	 * Set the maximum velocity to execute with feed forward and position control
	 * @param vel_horizontal horizontal velocity limit
	 * @param vel_up upwards velocity limit
	 * @param vel_down downwards velocity limit
	 */
	void setVelocityLimits(const float vel_horizontal, const float vel_up, float vel_down);

	/**
	 * Set the minimum and maximum collective normalized thrust [0,1] that can be output by the controller
	 * @param min minimum thrust e.g. 0.1 or 0
	 * @param max maximum thrust e.g. 0.9 or 1
	 */
	void setThrustLimits(const float min, const float max);

	/**
	 * Set the maximum tilt angle in radians the output attitude is allowed to have
	 * @param tilt angle in radians from level orientation
	 */
	void setTiltLimit(const float tilt) { _lim_tilt = tilt; }

	/**
	 * Set the normalized hover thrust
	 * @param thrust [0,1] with which the vehicle hovers not acelerating down or up with level orientation
	 */
	void setHoverThrust(const float hover_thrust) { _hover_thrust = hover_thrust; }

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
	 * Pass constraints that are stricter than the global limits
	 * Note: NAN value means no constraint, take maximum limit of controller.
	 * @param constraints a PositionControl structure with supported constraints
	 */
	void setConstraints(const vehicle_constraints_s &constraints);

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
	 * Set the integral term in xy to 0.
	 * @see _vel_int
	 */
	void resetIntegral() { _vel_int.setZero(); }

	/**
	 * Get the controllers output local position setpoint
	 * These setpoints are the ones which were executed on including PID output and feed-forward.
	 * The acceleration or thrust setpoints can be used for attitude control.
	 * @param local_position_setpoint reference to struct to fill up
	 */
	void getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const;

	/**
	 * Get the controllers output attitude setpoint
	 * This attitude setpoint was generated from the resulting acceleration setpoint after position and velocity control.
	 * It needs to be executed by the attitude controller to achieve velocity and position tracking.
	 * @param attitude_setpoint reference to struct to fill up
	 */
	void getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const;

		/**
	 * 	Get the
	 * 	@see z_k_Pr_R
	 * 	@return The z variable used by RCAC in the P controller
	 */
	const matrix::Vector3f get_RCAC_pos_z();

	/**
	 * 	Get the
	 * 	@see u_k_Pr_R
	 * 	@return The u variable computed by RCAC in the P controller
	 */
	const matrix::Vector3f get_RCAC_pos_u();
	/**
	 * 	Get the
	 * 	@see theta_k_Pr_R
	 * 	@return The theta variable computed by RCAC in the P controller
	 */
	const matrix::Vector3f get_RCAC_pos_theta();

	/**
	 * 	Get the
	 * 	@see Pos P gains
	 * 	@return The P gains
	 */
	const matrix::Vector3f get_PX4_pos_theta();

	/**
	 * 	Get the
	 * 	@see PID gains
	 * 	@return PX4's PID gains in the outer loop
	 */
	const matrix::Matrix<float, 9,1> get_PX4_ol_theta();

	/**
	 * 	Get the
	 * 	@see z_k_Pv_R
	 * 	@return The z variable used by RCAC in the PID velocity controller
	 */
	const matrix::Vector3f get_RCAC_vel_z();

	/**
	 * 	Get the
	 * 	@see u_k_Pr_R
	 * 	@return The u variable computed by RCAC in the P controller
	 */
	const matrix::Vector3f get_RCAC_vel_u();

	/**
	 * 	Get the
	 * 	@see theta_k_Pr_R
	 * 	@return The theta variable computed by RCAC in the P controller
	 */
	const matrix::Matrix<float, 9,1> get_RCAC_vel_theta();

	/**
	 * 	Get the
	 * 	@see ii
	 * 	@return Iteration step of the RCAC position controller
	 */
	const int &get_RCAC_pos_ii() { return ii_Pr_R; }

	/**
	 * 	Get the
	 * 	@see ii
	 * 	@return Iteration step of the RCAC velocity controller
	 */
	const int &get_RCAC_vel_ii() { return ii_Pv_R; }

	/**
	 * 	Set the RCAC position switch.
	 * 	@see _thr_int
	 */
	void set_RCAC_pos_switch(float switch_RCAC);

	/**
	 * 	Set the RCAC velocity switch.
	 * 	@see _thr_int
	 */
	void set_RCAC_vel_switch(float switch_RCAC);

	/**
	 * 	Set the PID scaling factor.
	 * 	@see _thr_int
	 */
	void set_PID_pv_factor(float PID_factor, float pos_alpha, float vel_alpha);
	/**
	 * 	Get the
	 * 	@see RCAC_Pr_ON
	 * 	@return Get RCAC pos controller switch
	 */
	const bool &get_RCAC_pos_switch() {return RCAC_Pr_ON;}

	/**
	 * 	Get the
	 * 	@see RCAC_Pr_ON
	 * 	@return Get RCAC vel controller switch
	 */
	const bool &get_RCAC_vel_switch() {return RCAC_Pv_ON;}

	/**
	 * 	Get the
	 * 	@see alpha_PID_pos
	 * 	@return Get gain that multiplies the position PID gains
	 */
	const float &get_pid_pos_alpha() {return alpha_PID_pos;}

	/**
	 * 	Get the
	 * 	@see alpha_PID_vel
	 * 	@return Get gain that multiplies the velocity PID gains
	 */
	const float &get_pid_vel_alpha() {return alpha_PID_vel;}

	/**
	 * 	Get the
	 * 	@see P_Pr_R
	 * 	@return RCAC P(1,1) of the Position controller
	 */
	const float &get_RCAC_P11_Pos() { return P_11_r; }

	/**
	 * 	Get the
	 * 	@see P_vel_x
	 * 	@return RCAC P(1,1) of the Velcity x controller
	 */
	const float &get_RCAC_P11_Velx() { return P_11_vx; }

	/**
	 * 	Reset RCAC variables
	 * 	@see _thr_int
	 */
	void resetRCAC(float rcac_pos_p0, float rcac_vel_p0);
	void init_RCAC(float rcac_pos_p0, float rcac_vel_p0);

private:
	bool _updateSuccessful();

	void _positionControl(); ///< Position proportional control
	void _velocityControl(const float dt); ///< Velocity PID control
	void _accelerationControl(); ///< Acceleration setpoint processing

	// Gains
	matrix::Vector3f _gain_pos_p; ///< Position control proportional gain
	matrix::Vector3f _gain_vel_p; ///< Velocity control proportional gain
	matrix::Vector3f _gain_vel_i; ///< Velocity control integral gain
	matrix::Vector3f _gain_vel_d; ///< Velocity control derivative gain

	// Limits
	float _lim_vel_horizontal{}; ///< Horizontal velocity limit with feed forward and position control
	float _lim_vel_up{}; ///< Upwards velocity limit with feed forward and position control
	float _lim_vel_down{}; ///< Downwards velocity limit with feed forward and position control
	float _lim_thr_min{}; ///< Minimum collective thrust allowed as output [-1,0] e.g. -0.9
	float _lim_thr_max{}; ///< Maximum collective thrust allowed as output [-1,0] e.g. -0.1
	float _lim_tilt{}; ///< Maximum tilt from level the output attitude is allowed to have

	float _hover_thrust{}; ///< Thrust [0,1] with which the vehicle hovers not accelerating down or up with level orientation

	// States
	matrix::Vector3f _pos; /**< current position */
	matrix::Vector3f _vel; /**< current velocity */
	matrix::Vector3f _vel_dot; /**< velocity derivative (replacement for acceleration estimate) */
	matrix::Vector3f _vel_int; /**< integral term of the velocity controller */
	float _yaw{}; /**< current heading */

	vehicle_constraints_s _constraints{}; /**< variable constraints */

	// Setpoints
	matrix::Vector3f _pos_sp; /**< desired position */
	matrix::Vector3f _vel_sp; /**< desired velocity */
	matrix::Vector3f _acc_sp; /**< desired acceleration */
	matrix::Vector3f _thr_sp; /**< desired thrust */
	float _yaw_sp{}; /**< desired heading */
	float _yawspeed_sp{}; /** desired yaw-speed */

	// New RCAC_Class_Variables
	// RCAC _rcac_pos_x;
	// RCAC _rcac_pos_y;
	// RCAC _rcac_pos_z;
	// RCAC _rcac_vel_x;
	// RCAC _rcac_vel_y;
	// RCAC _rcac_vel_z;
	// std::vector<RCAC> _rcac_pos;
	// std::vector<RCAC> _rcac_vel;
	//RCAC vel_RCAC;

	// RCAC
	int ii_Pr_R = 0;
	bool RCAC_Pr_ON=1;

	matrix::Matrix<RCAC, 1, 3> _rcac_r;
	matrix::Matrix<RCAC, 1, 3> _rcac_v;

	float P_11_r;	// spjohn -- P_11_r, P_11_vx are necessary for proper referencing in get_RCAC_P11_Pos()
	float P_11_vx;	//	     and get_RCAC_P11_Velx() -- fix later

	// matrix::SquareMatrix<float, 3> P_Pr_R;
	// matrix::Matrix<float, 3,3> phi_k_Pr_R, phi_km1_Pr_R;
	// matrix::Matrix<float, 3,1> theta_k_Pr_R;
  	// matrix::Matrix<float, 3,1> z_k_Pr_R, z_km1_Pr_R,u_k_Pr_R, u_km1_Pr_R;
	// matrix::SquareMatrix<float, 3> Gamma_Pr_R;

	// const TODO: make really const.
	matrix::SquareMatrix<float, 3> I3, N1_Pr;

	int ii_Pv_R = 0;
	bool RCAC_Pv_ON=1;
	bool _rcac_logging = true; /**< True if logging the aircraft state variable */ //TODO: MAV integration

	// matrix::SquareMatrix<float, 9> P_Pv_R;
	// matrix::Matrix<float, 3,9> phi_k_Pv_R, phi_km1_Pv_R;
	// matrix::Matrix<float, 9,1> theta_k_Pv_R,theta_k_Pv_PID;
  	// matrix::Matrix<float, 3,1> z_k_Pv_R, z_km1_Pv_R,u_k_Pv_R, u_km1_Pv_R;
	// matrix::SquareMatrix<float, 3> Gamma_Pv_R, N1_Pv;

	// matrix::SquareMatrix<float, 3> P_vel_x,P_vel_y,P_vel_z;
	// matrix::Matrix<float, 1,3> phi_k_vel_x, phi_km1_vel_x;
	// matrix::Matrix<float, 1,3> phi_k_vel_y, phi_km1_vel_y;
	// matrix::Matrix<float, 1,3> phi_k_vel_z, phi_km1_vel_z;
	// matrix::Vector3f theta_k_vel_x, theta_k_vel_y, theta_k_vel_z;
  	// matrix::Vector3f z_k_vel, z_km1_vel, u_k_vel, u_km1_vel;
	// matrix::Vector3f N1_vel, Gamma_vel;
	// matrix::Matrix<float, 1,1> dummy1,dummy2,dummy3;

	//float alpha_PID = 1.0f;
	float alpha_PID_pos = 1.0f;
	float alpha_PID_vel = 1.0f;

	matrix::Vector3f Pv_intg;
};
