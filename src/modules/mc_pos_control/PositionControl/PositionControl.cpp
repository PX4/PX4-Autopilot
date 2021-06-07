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
 * @file PositionControl.cpp
 */

#include "PositionControl.hpp"
#include "ControlMath.hpp"
#include <float.h>
#include <px4_platform_common/defines.h>
#include <ecl/geo/geo.h>

using namespace matrix;

void PositionControl::setVelocityGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_vel_p = P;
	_gain_vel_i = I;
	_gain_vel_d = D;
}

void PositionControl::setVelocityLimits(const float vel_horizontal, const float vel_up, const float vel_down)
{
	_lim_vel_horizontal = vel_horizontal;
	_lim_vel_up = vel_up;
	_lim_vel_down = vel_down;
}

void PositionControl::setThrustLimits(const float min, const float max)
{
	// make sure there's always enough thrust vector length to infer the attitude
	_lim_thr_min = math::max(min, 10e-4f);
	_lim_thr_max = max;
}

void PositionControl::updateHoverThrust(const float hover_thrust_new)
{
	_vel_int(2) += (hover_thrust_new - _hover_thrust) * (CONSTANTS_ONE_G / hover_thrust_new);
	setHoverThrust(hover_thrust_new);
}

void PositionControl::setState(const PositionControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_yaw = states.yaw;
	_vel_dot = states.acceleration;
}

void PositionControl::setInputSetpoint(vehicle_local_position_setpoint_s setpoint)
{
	_pos_sp = Vector3f(setpoint.x, setpoint.y, setpoint.z);
	_vel_sp = Vector3f(setpoint.vx, setpoint.vy, setpoint.vz);
	_acc_sp = Vector3f(setpoint.acceleration);
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;

	// kcc ++;

	// if (kcc >= 50){
	// 	kcc = 0;
	// 	for (int i = 0; i <= 2; i++) {
	// 		// PX4_INFO("_pos(%d) = \t%8.6f", i, (double)_pos(i));
	// 		PX4_INFO("_vel_sp(%d) = \t%8.6f", i, (double)_vel_sp(i));
	// 	}
	// }
}

void PositionControl::setConstraints(const vehicle_constraints_s &constraints)
{
	_constraints = constraints;

	// For safety check if adjustable constraints are below global constraints. If they are not stricter than global
	// constraints, then just use global constraints for the limits.
	if (!PX4_ISFINITE(constraints.tilt) || (constraints.tilt > _lim_tilt)) {
		_constraints.tilt = _lim_tilt;
	}

	if (!PX4_ISFINITE(constraints.speed_up) || (constraints.speed_up > _lim_vel_up)) {
		_constraints.speed_up = _lim_vel_up;
	}

	if (!PX4_ISFINITE(constraints.speed_down) || (constraints.speed_down > _lim_vel_down)) {
		_constraints.speed_down = _lim_vel_down;
	}

	// ignore _constraints.speed_xy TODO: remove it completely as soon as no task uses it anymore to avoid confusion
}

bool PositionControl::update(const float dt)
{
	// x and y input setpoints always have to come in pairs
	const bool valid = (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1)))
			   && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1)))
			   && (PX4_ISFINITE(_acc_sp(0)) == PX4_ISFINITE(_acc_sp(1)));

	_positionControl();
	_velocityControl(dt);

	_yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
	_yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw; // TODO: better way to disable yaw control

	return valid && _updateSuccessful();
}

void PositionControl::_positionControl()
{
	// P-position controller
	Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);
	// PX4_INFO("vel_sp_ff = \t%8.6f \t%8.6f \t%8.6f", (double)vel_sp_position(0), (double)vel_sp_position(1), (double)vel_sp_position(2));
	// Vector3f pos_error_ = _pos_sp - _pos;
	// Vector3f u_k_pos;


	for (int i = 0; i <=2; i++)
	{
		if (isnan(_pos_sp(i)) || isnan(_vel_sp(i)))
		{
			islanded = true;
			since_takeoff = 0;
			break;
		}
		else
		{
			islanded = false;
		}
	}

	z_k_r = _pos_sp - _pos;
	u_k_r.setZero();

	if ((RCAC_Pr_ON) && (!islanded))
	{
		if (since_takeoff == 0)
		{
			init_RCAC();
		}

		for (int i = 0; i <= 2; i++)
		{
			u_k_r(i) = _rcac_r(0,i).compute_uk(z_k_r(i), 0, 0, u_km1_r(i));
		}
		u_km1_r = u_k_r;

		ii_Pr_R++;
		since_takeoff++;
	}

	// for (int i = 0; i <= 2; i++) {
	// 	// PX4_INFO("_pos(%d) = \t%8.6f", i, (double)_pos(i));
	// 	PX4_INFO("_pos_sp(%d) = \t%8.6f", i, (double)_pos_sp(i));
	// }

	// z_k_Pr_R = (_pos_sp - _pos);
	// u_k_Pr_R.setZero();
	// if (RCAC_Pr_ON)
	// {
	//         ii_Pr_R += 1;

	// 	// Regressor
	// 	for (int i=0; i<3; i++) {
	// 		phi_k_Pr_R(i, i) = z_k_Pr_R(i,0);
	// 	}
	// 	Gamma_Pr_R 	= phi_km1_Pr_R * P_Pr_R * phi_km1_Pr_R.T() + I3;
	// 	Gamma_Pr_R 	= Gamma_Pr_R.I();
	// 	P_Pr_R 	-= (P_Pr_R * phi_km1_Pr_R.T()) * Gamma_Pr_R * (phi_km1_Pr_R * P_Pr_R);
	// 	theta_k_Pr_R += (P_Pr_R * phi_km1_Pr_R.T()) * N1_Pr *
	// 				(z_k_Pr_R + N1_Pr*(phi_km1_Pr_R * theta_k_Pr_R - u_km1_Pr_R) );
	// 	u_k_Pr_R 	= phi_k_Pr_R * theta_k_Pr_R;
	// 	u_km1_Pr_R 	= u_k_Pr_R;
	// 	phi_km1_Pr_R 	= phi_k_Pr_R;

	// 	// PX4_INFO("Pos Control u :\t%8.6f\t%8.6f\t%8.6f", (double)P_Pr_R(0,0), (double)P_Pr_R(1,1), (double)P_Pr_R(2,2));
	// 	// PX4_INFO("Pos Control u :\t%8.6f\t%8.6f\t%8.6f", (double)N1_vel(0), (double)N1_vel(1), (double)N1_vel(2));
	// 	// PX4_INFO("Pos Control P0:\t%8.6f\t%8.6f", (double)_param_mpc_rcac_pos_p0.get(), (double)P_Pr_R(0,0) );
	// }

	//vel_sp_position = alpha_PID*vel_sp_position + u_k_Pr_R;
	vel_sp_position = alpha_PID_pos*vel_sp_position + u_k_r;
	// PX4_INFO("vel_sp_ff = \t%8.6f \t%8.6f \t%8.6f", (double)vel_sp_position(0), (double)vel_sp_position(1), (double)vel_sp_position(2));

	// Position and feed-forward velocity setpoints or position states being NAN results in them not having an influence
	ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
	// make sure there are no NAN elements for further reference while constraining
	ControlMath::setZeroIfNanVector3f(vel_sp_position);

	// Constrain horizontal velocity by prioritizing the velocity component along the
	// the desired position setpoint over the feed-forward term.
	_vel_sp.xy() = ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal);
	// Constrain velocity in z-direction.
	_vel_sp(2) = math::constrain(_vel_sp(2), -_constraints.speed_up, _constraints.speed_down);
}

void PositionControl::_velocityControl(const float dt)
{
	// PID velocity control
	Vector3f vel_error = _vel_sp - _vel;
	Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d);
	// Vector3f u_k_vel;

	z_k_v = _vel_sp - _vel;
	u_k_v.setZero();

	if ((RCAC_Pv_ON) && (!islanded))
	{
		ii_Pv_R += 1;
		for (int i = 0; i <= 2; i++)
		{
			u_k_v(i) = _rcac_v(0,i).compute_uk(z_k_v(i), _vel_int(i), _vel_dot(i), u_km1_v(i));
		}
		u_km1_v = u_k_v;
	}

	// z_k_vel = vel_error;
	//u_k_vel.setZero();
	// if (RCAC_Pv_ON)
	// {
	// 	ii_Pv_R += 1;
	// 		// Ankit 01 30 2020:New SISO implementation
	// 		z_k_vel = vel_error;

	// 		phi_k_vel_x(0,0) = vel_error(0);
	// 		phi_k_vel_x(0,1) = _vel_int(0);
	// 		phi_k_vel_x(0,2) = _vel_dot(0) * 0;

	// 		phi_k_vel_y(0,0) = vel_error(1);
	// 		phi_k_vel_y(0,1) = _vel_int(1);
	// 		phi_k_vel_y(0,2) = _vel_dot(1) * 0;

	// 		phi_k_vel_z(0,0) = vel_error(2);
	// 		phi_k_vel_z(0,1) = _vel_int(2);
	// 		phi_k_vel_z(0,2) = _vel_dot(2) * 0;

	// 		dummy1 = phi_km1_vel_x * P_vel_x * phi_km1_vel_x.T() + 1.0f;
	// 		dummy2 = phi_km1_vel_y * P_vel_y * phi_km1_vel_y.T() + 1.0f;
	// 		dummy3 = phi_km1_vel_z * P_vel_z * phi_km1_vel_z.T() + 1.0f;
	// 		Gamma_vel(0) 	= dummy1(0,0);
	// 		Gamma_vel(1) 	= dummy2(0,0);
	// 		Gamma_vel(2) 	= dummy3(0,0);

	// 		P_vel_x = P_vel_x - (P_vel_x * phi_km1_vel_x.T()) * (phi_km1_vel_x * P_vel_x) / Gamma_vel(0);
	// 		P_vel_y = P_vel_y - (P_vel_y * phi_km1_vel_y.T()) * (phi_km1_vel_y * P_vel_y) / Gamma_vel(1);
	// 		P_vel_z = P_vel_z - (P_vel_z * phi_km1_vel_z.T()) * (phi_km1_vel_z * P_vel_z) / Gamma_vel(2);

	// 		dummy1 = N1_vel(0)*(phi_km1_vel_x * theta_k_vel_x - u_km1_vel(0));
	// 		dummy2 = N1_vel(1)*(phi_km1_vel_y * theta_k_vel_y - u_km1_vel(1));
	// 		dummy3 = N1_vel(2)*(phi_km1_vel_z * theta_k_vel_z - u_km1_vel(2));
	// 		theta_k_vel_x 	= theta_k_vel_x + (P_vel_x * phi_km1_vel_x.T()) * N1_vel(0) *(z_k_vel(0) + dummy1(0,0));
	// 		theta_k_vel_y 	= theta_k_vel_y + (P_vel_y * phi_km1_vel_y.T()) * N1_vel(1) *(z_k_vel(1) + dummy2(0,0));
	// 		theta_k_vel_z 	= theta_k_vel_z + (P_vel_z * phi_km1_vel_z.T()) * N1_vel(2) *(z_k_vel(2) + dummy3(0,0));

	// 		dummy1 = phi_k_vel_x * theta_k_vel_x;
	// 		dummy2 = phi_k_vel_y * theta_k_vel_y;
	// 		dummy3 = phi_k_vel_z * theta_k_vel_z;
	// 		u_k_vel(0) = dummy1(0,0);
	// 		u_k_vel(1) = dummy2(0,0);
	// 		u_k_vel(2) = dummy3(0,0);

	// 		u_km1_vel = u_k_vel;

	// 		phi_km1_vel_x = phi_k_vel_x;
	// 		phi_km1_vel_y = phi_k_vel_y;
	// 		phi_km1_vel_z = phi_k_vel_z;

	// }

	acc_sp_velocity = alpha_PID_vel*acc_sp_velocity + u_k_v;

	// No control input from setpoints or corresponding states which are NAN
	ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

	_accelerationControl();

	// Integrator anti-windup in vertical direction
	if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.0f) ||
	    (_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.0f)) {
		vel_error(2) = 0.f;
	}

	// Saturate maximal vertical thrust
	_thr_sp(2) = math::max(_thr_sp(2), -_lim_thr_max);

	// Get allowed horizontal thrust after prioritizing vertical control
	const float thrust_max_squared = _lim_thr_max * _lim_thr_max;
	const float thrust_z_squared = _thr_sp(2) * _thr_sp(2);
	const float thrust_max_xy_squared = thrust_max_squared - thrust_z_squared;
	float thrust_max_xy = 0;

	if (thrust_max_xy_squared > 0) {
		thrust_max_xy = sqrtf(thrust_max_xy_squared);
	}

	// Saturate thrust in horizontal direction
	const Vector2f thrust_sp_xy(_thr_sp);
	const float thrust_sp_xy_norm = thrust_sp_xy.norm();

	if (thrust_sp_xy_norm > thrust_max_xy) {
		_thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
	}

	// Use tracking Anti-Windup for horizontal direction: during saturation, the integrator is used to unsaturate the output
	// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
	const Vector2f acc_sp_xy_limited = Vector2f(_thr_sp) * (CONSTANTS_ONE_G / _hover_thrust);
	const float arw_gain = 2.f / _gain_vel_p(0);
	vel_error.xy() = Vector2f(vel_error) - (arw_gain * (Vector2f(_acc_sp) - acc_sp_xy_limited));

	// Make sure integral doesn't get NAN
	ControlMath::setZeroIfNanVector3f(vel_error);
	// Update integral part of velocity control
	_vel_int += vel_error.emult(_gain_vel_i) * dt;

	// limit thrust integral
	_vel_int(2) = math::min(fabsf(_vel_int(2)), CONSTANTS_ONE_G) * sign(_vel_int(2));
}

void PositionControl::_accelerationControl()
{
	// Assume standard acceleration due to gravity in vertical direction for attitude generation
	Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), CONSTANTS_ONE_G).normalized();
	ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _constraints.tilt);
	// Scale thrust assuming hover thrust produces standard gravity
	float collective_thrust = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
	// Project thrust to planned body attitude
	collective_thrust /= (Vector3f(0, 0, 1).dot(body_z));
	collective_thrust = math::min(collective_thrust, -_lim_thr_min);
	_thr_sp = body_z * collective_thrust;
}

bool PositionControl::_updateSuccessful()
{
	bool valid = true;

	// For each controlled state the estimate has to be valid
	for (int i = 0; i <= 2; i++) {
		if (PX4_ISFINITE(_pos_sp(i))) {
			valid = valid && PX4_ISFINITE(_pos(i));
		}

		if (PX4_ISFINITE(_vel_sp(i))) {
			valid = valid && PX4_ISFINITE(_vel(i)) && PX4_ISFINITE(_vel_dot(i));
		}
	}

	// There has to be a valid output accleration and thrust setpoint otherwise there was no
	// setpoint-state pair for each axis that can get controlled
	valid = valid && PX4_ISFINITE(_acc_sp(0)) && PX4_ISFINITE(_acc_sp(1)) && PX4_ISFINITE(_acc_sp(2));
	valid = valid && PX4_ISFINITE(_thr_sp(0)) && PX4_ISFINITE(_thr_sp(1)) && PX4_ISFINITE(_thr_sp(2));
	return valid;
}

void PositionControl::getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const
{
	local_position_setpoint.x = _pos_sp(0);
	local_position_setpoint.y = _pos_sp(1);
	local_position_setpoint.z = _pos_sp(2);
	local_position_setpoint.yaw = _yaw_sp;
	local_position_setpoint.yawspeed = _yawspeed_sp;
	local_position_setpoint.vx = _vel_sp(0);
	local_position_setpoint.vy = _vel_sp(1);
	local_position_setpoint.vz = _vel_sp(2);
	_acc_sp.copyTo(local_position_setpoint.acceleration);
	_thr_sp.copyTo(local_position_setpoint.thrust);
}

void PositionControl::getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const
{
	ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, attitude_setpoint);
	attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;
}

const matrix::Vector3f PositionControl::get_RCAC_pos_z()
{
	matrix::Vector3f RCAC_z{};

	for (int i = 0; i <= 2; i++) {
		// RCAC_z(i) = z_k_Pr_R(i,0);			// spjohn -- sub in rcac class
		RCAC_z(i) = _rcac_r(0,i).get_rcac_zk();
	}

	return RCAC_z;
}

const matrix::Vector3f PositionControl::get_RCAC_pos_u()
{
	matrix::Vector3f RCAC_u{};

	for (int i = 0; i <= 2; i++) {
		// RCAC_u(i) = u_k_Pr_R(i,0);			// spjohn -- sub in rcac class
		RCAC_u(i) = _rcac_r(0,i).get_rcac_uk();
	}

	return RCAC_u;
}

const matrix::Vector3f PositionControl::get_RCAC_pos_theta()
{
	matrix::Vector3f RCAC_theta{};

	for (int i = 0; i <= 2; i++) {
		// RCAC_theta(i) = theta_k_Pr_R(i,0);		// spjohn -- sub in rcac class
		RCAC_theta(i) = _rcac_r(0,i).get_rcac_theta(0);

		// for (int j = 0; j <= 2; j++){
		// 	PX4_INFO("RCAC_theta(%d):\t%8.6f", i, (double)_rcac_r(0,i).get_rcac_theta(j));
		// }
	}

	return RCAC_theta;
}

const matrix::Vector3f PositionControl::get_PX4_pos_theta()
{
	matrix::Vector3f PX4_theta{};
	PX4_theta(0) = _gain_pos_p(0);
	PX4_theta(1) = _gain_pos_p(1);
	PX4_theta(2) = _gain_pos_p(2);
	return PX4_theta;
}

const matrix::Matrix<float, 9,1> PositionControl::get_PX4_ol_theta()
{
	matrix::Matrix<float, 9,1> PX4_theta{};
	PX4_theta(0,0) = _gain_vel_p(0);
	PX4_theta(1,0) = _gain_vel_i(0);
	PX4_theta(2,0) = _gain_vel_d(0);

	PX4_theta(3,0) = _gain_vel_p(1);
	PX4_theta(4,0) = _gain_vel_i(1);
	PX4_theta(5,0) = _gain_vel_d(1);

	PX4_theta(6,0) = _gain_vel_p(2);
	PX4_theta(7,0) = _gain_vel_i(2);
	PX4_theta(8,0) = _gain_vel_d(2);

	return PX4_theta;
}

const matrix::Vector3f PositionControl::get_RCAC_vel_z()
{
	matrix::Vector3f RCAC_z{};

	for (int i = 0; i <= 2; i++) {
		// RCAC_z(i) = z_k_vel(i); //z_k_Pv_R(i,0);	// spjohn -- sub in rcac class
		RCAC_z(i) = _rcac_v(0,i).get_rcac_zk();
	}

	return RCAC_z;
}

const matrix::Vector3f PositionControl::get_RCAC_vel_u()
{
	matrix::Vector3f RCAC_u{};

	for (int i = 0; i <= 2; i++) {
		// RCAC_u(i) = u_k_vel(i); //u_k_Pv_R(i,0);	// spjohn -- sub in rcac class
		RCAC_u(i) = _rcac_v(0,i).get_rcac_uk();
	}

	return RCAC_u;
}

const matrix::Matrix<float, 9,1> PositionControl::get_RCAC_vel_theta()
{
	matrix::Matrix<float, 9,1> RCAC_vel_theta{};
	RCAC_vel_theta.setZero();

	for (int i = 0; i <= 2; i++)
	{
		// //RCAC_theta(i,0) = theta_k_Pv_R(i,0);	// spjohn -- sub in rcac class
		// RCAC_theta(i,0) = theta_k_vel_x(i);
		// RCAC_theta(i+3,0) = theta_k_vel_y(i);
		// RCAC_theta(i+6,0) = theta_k_vel_z(i);

		RCAC_vel_theta(i,0) = _rcac_v(0,0).get_rcac_theta(i);
		RCAC_vel_theta(i+3,0) = _rcac_v(0,1).get_rcac_theta(i);
		RCAC_vel_theta(i+6,0) = _rcac_v(0,2).get_rcac_theta(i);
	}

	return RCAC_vel_theta;
}

void PositionControl::set_RCAC_pos_switch(float switch_RCAC)
{
	// RCAC_Pr_ON = 1;
	if (switch_RCAC<0.0f) {
		RCAC_Pr_ON = 0;
	}
}

void PositionControl::set_RCAC_vel_switch(float switch_RCAC)
{
	// RCAC_Pv_ON = 1;
	if (switch_RCAC<0.0f) {
		RCAC_Pv_ON = 0;
	}
}

void PositionControl::set_PID_pv_factor(float PID_factor, float pos_alpha, float vel_alpha)
{
	// alpha_PID_pos = 1.0f;
	// alpha_PID_vel = 1.0f;
	//alpha_PID = 1.0f;

	if (PID_factor<0.0f) {
		//alpha_PID = 0.5;
		alpha_PID_pos = pos_alpha;
		alpha_PID_vel = vel_alpha;
	}

}

void PositionControl::init_RCAC()
{
	I3 = eye<float, 3>();
	N1_Pr = eye<float, 3>() * (1.0f);

	u_k_r.setZero();
	u_km1_r.setZero();
	z_k_r.setZero();
	z_km1_r.setZero();

	u_k_v.setZero();
	u_km1_v.setZero();
	z_k_v.setZero();
	z_km1_v.setZero();

	for (int i = 0; i <= 2; i++) {
		_rcac_r(0,i) = RCAC(p0_r);
		_rcac_v(0,i) = RCAC(p0_v);
	}

	// for (int i = 0; i <= 2; i++) {
	// 	N1_vel(i) = 1;
	// }
	// P_Pr_R = eye<float, 3>() * 0.010 ;

	// phi_k_Pr_R.setZero();			// spjohn -- sub in rcac class
	// phi_km1_Pr_R.setZero();
	// theta_k_Pr_R.setZero();
	// z_k_Pr_R.setZero();
	// z_km1_Pr_R.setZero();
	// u_k_Pr_R.setZero();
	// u_km1_Pr_R.setZero();
	// Gamma_Pr_R.setZero();

	// P_vel_x = eye<float, 3>() * 0.0010;
	// P_vel_y = eye<float, 3>() * 0.0010;
	// P_vel_z = eye<float, 3>() * 0.0010;
	// phi_k_vel_x.setZero();
	// phi_k_vel_y.setZero();
	// phi_k_vel_z.setZero();
	// phi_km1_vel_x.setZero();
	// phi_km1_vel_y.setZero();
	// phi_km1_vel_z.setZero();
	// theta_k_vel_x.setZero();
	// theta_k_vel_y.setZero();
	// theta_k_vel_z.setZero();
	// u_k_vel.setZero();
	// z_k_vel.setZero();

	// P_Pr_R = eye<float, 3>() * rcac_pos_p0;
	// P_vel_x = eye<float, 3>() * rcac_vel_p0;
	// P_vel_y = eye<float, 3>() * rcac_vel_p0;
	// P_vel_z = eye<float, 3>() * rcac_vel_p0;
	// _rcac_pos_x = RCAC(rcac_pos_p0);
	// _rcac_pos_y = RCAC(rcac_pos_p0);
	// _rcac_pos_z = RCAC(rcac_pos_p0);
	// _rcac_vel_x = RCAC(rcac_vel_p0);
	// _rcac_vel_y = RCAC(rcac_vel_p0);
	// _rcac_vel_z = RCAC(rcac_vel_p0);


	// P_11_r = rcac_pos_p0;
	// P_11_vx = rcac_vel_p0;

	PX4_INFO("Pos Control P0:\t%8.6f", (double)p0_r);
	PX4_INFO("Vel Control P0:\t%8.6f", (double)p0_v);
}

void PositionControl::resetRCAC()
{
	u_k_r.setZero();
	u_km1_r.setZero();
	z_k_r.setZero();
	z_km1_r.setZero();

	u_k_v.setZero();
	u_km1_v.setZero();
	z_k_v.setZero();
	z_km1_v.setZero();

	for (int i = 0; i <= 2; i++) {
		_rcac_r(0,i) = RCAC(p0_r);
		_rcac_v(0,i) = RCAC(p0_v);
	}

	// P_vel_x = eye<float, 3>() * 0.0010;
	// P_vel_y = eye<float, 3>() * 0.0010;
	// P_vel_z = eye<float, 3>() * 0.0010;
	// P_Pr_R = eye<float, 3>() * 0.010 * alpha_P;

	// P_vel_x.setZero();
	// P_vel_y.setZero();
	// P_vel_z.setZero();
	// P_Pr_R.setZero();
	// for (int i = 0; i <= 2; i++) {
	// 	P_vel_x(i,i) = 0.001;
	// 	P_vel_y(i,i) = 0.001;
	// 	P_vel_z(i,i) = 0.001;
	// 	P_Pr_R(i,i) = 0.01;
	// 	P_vel_x(i,i) = rcac_vel_p0;
	// 	P_vel_y(i,i) = rcac_vel_p0;
	// 	P_vel_z(i,i) = rcac_vel_p0;
	// 	P_Pr_R(i,i) = rcac_pos_p0;
	// }
	// phi_k_vel_x.setZero();
	// phi_k_vel_y.setZero();
	// phi_k_vel_z.setZero();
	// phi_km1_vel_x.setZero();
	// phi_km1_vel_y.setZero();
	// phi_km1_vel_z.setZero();
	// theta_k_vel_x.setZero();
	// theta_k_vel_y.setZero();
	// theta_k_vel_z.setZero();
	// u_k_vel.setZero();
	// z_k_vel.setZero();

	// phi_km1_Pr_R.setZero();
	// theta_k_Pr_R.setZero();
	// z_k_Pr_R.setZero();
	// z_km1_Pr_R.setZero();
	// u_k_Pr_R.setZero();
	// u_km1_Pr_R.setZero();

	// P_11_r = rcac_pos_p0;
	// P_11_vx = rcac_vel_p0;

	ii_Pr_R = 0;
	ii_Pv_R = 0;
}
