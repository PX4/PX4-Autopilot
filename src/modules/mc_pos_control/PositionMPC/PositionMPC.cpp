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
 * @file PositionMPC.cpp
 */

#include "PositionMPC.hpp"
#include "ControlMath.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <geo/geo.h>
#include <lib/tinympc/tinympc.h>
#include <iostream>


// TinyMPC variables
// Macro variables
#define DT 0.002f    // dt
#define NSTATES 12   // no. of states (error state)
#define NINPUTS 4    // no. of controls
#define NHORIZON 20   // horizon steps (NHORIZON states and NHORIZON-1 controls)

#define PUBLISH_RATE 1 // 1 for true, 0 for false. If false, publishes Torque and Thrust setpoints at 500Hz

#include "params/params_500hz.h"
// #include "params/traj_fig8.h"

/* Allocate global variables for MPC */
static float f_data[NSTATES] = {0};

// Create data array, all zero initialization
static float x0_data[NSTATES] = {0.0f};       // initial state
static float xg_data[NSTATES] = {0.0f};       // goal state (if not tracking)
static float ug_data[NINPUTS] = {0.0f};       // goal input
// static float Xref_data[NSTATES * NHORIZON] = {0};
static float X_data[NSTATES * NHORIZON] = {0.0f};        // X in MPC solve
static float U_data[NINPUTS * (NHORIZON - 1)] = {0.0f};  // U in MPC solve
static float d_data[NINPUTS * (NHORIZON - 1)] = {0.0f};
static float p_data[NSTATES * NHORIZON] = {0.0f};
static float q_data[NSTATES*(NHORIZON-1)] = {0.0f};
static float r_data[NINPUTS*(NHORIZON-1)] = {0.0f};
static float r_tilde_data[NINPUTS*(NHORIZON-1)] = {0.0f};
static float Acu_data[NINPUTS * NINPUTS] = {0.0f};
static float YU_data[NINPUTS * (NHORIZON - 1)] = {0.0f};
static float umin_data[NINPUTS] = {0.0f};
static float umax_data[NINPUTS] = {0.0f};
static float temp_data[NINPUTS + 2*NINPUTS*(NHORIZON - 1)] = {0.0f};

// Created matrices
static Matrix Xref[NHORIZON];
static Matrix Uref[NHORIZON - 1];
static Matrix X[NHORIZON];
static Matrix U[NHORIZON - 1];
static Matrix d_[NHORIZON - 1];
static Matrix p[NHORIZON];
static Matrix YU[NHORIZON - 1];
static Matrix ZU[NHORIZON - 1];
static Matrix ZU_new[NHORIZON - 1];
static Matrix q_[NHORIZON-1];
static Matrix r[NHORIZON-1];
static Matrix r_tilde[NHORIZON-1];
static Matrix A;
static Matrix B;
static Matrix f;

//Create TinyMPC struct
static tiny_Model mpc_model;
static tiny_AdmmSettings stgs;
static tiny_AdmmData mpc_data;
static tiny_AdmmInfo mpc_info;
static tiny_AdmmSolution mpc_soln;
static tiny_AdmmWorkspace mpc_work;

// Helper variables
static float u_hover = 0.67f;

const trajectory_setpoint_s PositionMPC::empty_trajectory_setpoint  = {0, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, NAN, NAN};

PositionMPC::PositionMPC(){
	// Initialize MPC
	tiny_InitModel(&mpc_model, NSTATES, NINPUTS, NHORIZON, 0, 0, DT);
	tiny_InitSettings(&stgs);

	stgs.rho_init = 250.0f;  // IMPORTANT (select offline, associated with precomp.)

	tiny_InitWorkspace(&mpc_work, &mpc_info, &mpc_model, &mpc_data, &mpc_soln, &stgs);

	// Fill in the remaining struct
	tiny_InitWorkspaceTempData(&mpc_work, ZU, ZU_new, 0, 0, temp_data);
	tiny_InitPrimalCache(&mpc_work, Quu_inv_data, AmBKt_data, coeff_d2p_data);

	tiny_InitModelFromArray(&mpc_model, &A, &B, &f, A_data, B_data, f_data);
	tiny_InitSolnTrajFromArray(&mpc_work, X, U, X_data, U_data);
	tiny_InitSolnGainsFromArray(&mpc_work, d_, p, d_data, p_data, Kinf_data, Pinf_data);
	tiny_InitSolnDualsFromArray(&mpc_work, 0, YU, 0, YU_data, 0);

	tiny_SetInitialState(&mpc_work, x0_data);
	tiny_SetGoalReference(&mpc_work, Xref, Uref, xg_data, ug_data);

	// Set up LQR cost
	tiny_InitDataQuadCostFromArray(&mpc_work, Q_data, R_data);
	slap_AddIdentity(mpc_data.R, mpc_work.rho); // \tilde{R}
	tiny_InitDataLinearCostFromArray(&mpc_work, q_, r, r_tilde, q_data, r_data, r_tilde_data);

	// Set up constraints
	tiny_SetInputBound(&mpc_work, Acu_data, umin_data, umax_data);
	slap_SetConst(mpc_data.ucu, (1 - u_hover));   // UPPER CONTROL BOUND
	slap_SetConst(mpc_data.lcu, (-u_hover));  // LOWER CONTROL BOUND

	// Initialize linear cost (for tracking)
	tiny_UpdateLinearCost(&mpc_work);

	// Solver settings
	stgs.en_cstr_goal = 0;
	stgs.en_cstr_inputs = 1;
	stgs.en_cstr_states = 0;
	stgs.max_iter = 6;           // limit this if needed
	stgs.verbose = 0;
	stgs.check_termination = 2;
	stgs.tol_abs_dual = 5e-2;
	stgs.tol_abs_prim = 5e-2;
}

void PositionMPC::setVelocityGains(const matrix::Vector3f &P, const matrix::Vector3f &I, const matrix::Vector3f &D)
{
	_gain_vel_p = P;
	_gain_vel_i = I;
	_gain_vel_d = D;
}

void PositionMPC::setVelocityLimits(const float vel_horizontal, const float vel_up, const float vel_down)
{
	_lim_vel_horizontal = vel_horizontal;
	_lim_vel_up = vel_up;
	_lim_vel_down = vel_down;
}

void PositionMPC::setThrustLimits(const float min, const float max)
{
	// make sure there's always enough thrust vector length to infer the attitude
	_lim_thr_min = math::max(min, 10e-4f);
	_lim_thr_max = max;
}

void PositionMPC::setHorizontalThrustMargin(const float margin)
{
	_lim_thr_xy_margin = margin;
}

void PositionMPC::updateHoverThrust(const float hover_thrust_new)
{
	// Given that the equation for thrust is T = a_sp * Th / g - Th
	// with a_sp = desired acceleration, Th = hover thrust and g = gravity constant,
	// we want to find the acceleration that needs to be added to the integrator in order obtain
	// the same thrust after replacing the current hover thrust by the new one.
	// T' = T => a_sp' * Th' / g - Th' = a_sp * Th / g - Th
	// so a_sp' = (a_sp - g) * Th / Th' + g
	// we can then add a_sp' - a_sp to the current integrator to absorb the effect of changing Th by Th'
	const float previous_hover_thrust = _hover_thrust;
	setHoverThrust(hover_thrust_new);

	_vel_int(2) += (_acc_sp(2) - CONSTANTS_ONE_G) * previous_hover_thrust / _hover_thrust
		       + CONSTANTS_ONE_G - _acc_sp(2);
}

void PositionMPC::setInputSetpoint(const trajectory_setpoint_s &setpoint)
{
	_pos_sp = matrix::Vector3f(setpoint.position);
	_vel_sp = matrix::Vector3f(setpoint.velocity);
	_acc_sp = matrix::Vector3f(setpoint.acceleration);
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;
}

bool PositionMPC::update(const float dt)
{
	bool valid = _inputValid();

	if (valid) {
		// Set starting state (x0)
		x0_data[0] = PX4_ISFINITE(_pos(0)) ? _pos(0) : 0.f;
		x0_data[1] = -PX4_ISFINITE(_pos(1)) ? _pos(1) : 0.f;
		x0_data[2] = -PX4_ISFINITE(_pos(2)) ? _pos(2) : 0.f;
		// Body velocity error, [m/s]
		x0_data[6] = PX4_ISFINITE(_vel(0)) ? _vel(0) : 0.f;
		x0_data[7] = -PX4_ISFINITE(_vel(1)) ? _vel(1) : 0.f;
		x0_data[8] = -PX4_ISFINITE(_vel(2)) ? _vel(2) : 0.f;
		// Angular rate error, [rad/s]
		x0_data[9]  = PX4_ISFINITE(_ang_vel(0)) ? _ang_vel(0) : 0.f;
		x0_data[10] = PX4_ISFINITE(_ang_vel(1)) ? _ang_vel(1) : 0.f;
		x0_data[11] = PX4_ISFINITE(_ang_vel(2)) ? _ang_vel(2) : 0.f;
		// Attitude error
		matrix::Quatf quat_from_lib = matrix::Quatf(_att(0), _att(1), -_att(2), -_att(3));
		matrix::Eulerf q_to_euler(quat_from_lib);
		x0_data[3] = q_to_euler.phi();
		x0_data[4] = q_to_euler.theta();
		x0_data[5] = _yaw;

		// Debug
		xg_data[0] = PX4_ISFINITE(_pos_sp(0)) ? _pos_sp(0) : 0.f;
		xg_data[1] = -PX4_ISFINITE(_pos_sp(1)) ? _pos_sp(1) : 0.f;
		xg_data[2] = -PX4_ISFINITE(_pos_sp(2)) ? _pos_sp(2) : 0.f;

		xg_data[3] = 0.0f;
		xg_data[4] = 0.0f;
		xg_data[5] = 0.0f;

		xg_data[6] = PX4_ISFINITE(_vel_sp(0)) ? _vel_sp(0) : 0.f;
		xg_data[7] = -PX4_ISFINITE(_vel_sp(1)) ? _vel_sp(1) : 0.f;
		xg_data[8] = -PX4_ISFINITE(_vel_sp(2)) ? _vel_sp(2) : 0.f;

		xg_data[9] = 0.0f;
		xg_data[10] = 0.0f;
		xg_data[11] = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
		tiny_SetGoalReference(&mpc_work, Xref, Uref, xg_data, ug_data);
		std::cout << "X Goal: \n" << xg_data[0] << " " << xg_data[1] << " " << xg_data[2] << " \n"
			<< xg_data[3] << " " << xg_data[4] << " " << xg_data[5] << " \n"
			<< xg_data[6] << " " << xg_data[7] << " " << xg_data[8] << " \n"
			<< xg_data[9] << " " << xg_data[10] << " " << xg_data[11] << std::endl;
		std::cout << "X0: \n" << x0_data[0] << " " << x0_data[1] << " " << x0_data[2] << " \n"
			<< x0_data[3] << " " << x0_data[4] << " " << x0_data[5] << " \n"
			<< x0_data[6] << " " << x0_data[7] << " " << x0_data[8] << " \n"
			<< x0_data[9] << " " << x0_data[10] << " " << x0_data[11] << std::endl;

		/* MPC solve */
		tiny_UpdateLinearCost(&mpc_work);
		tiny_SolveAdmm(&mpc_work);
		std::cout << "Status: " << mpc_info.status_val << " | Iteration: " << mpc_info.iter << std::endl;
		// std::cout << "Inputs: " << U[0].data[0] + u_hover << " " << U[0].data[1] + u_hover  << " " << U[0].data[2] + u_hover  << "  "<< U[0].data[3] + u_hover  << std::endl;
		std::cout << "Inputs to Att Ctl: " << U[0].data[0] + u_hover << " " << X[1].data[5] << " " << X[1].data[11] << std::endl;
		_thr_sp(0) = _thr_sp(1) = 0;
		_thr_sp(2) = U[0].data[0] + u_hover;
		_yaw_sp = X[1].data[5];
		_yawspeed_sp = X[1].data[11];


	}

	// There has to be a valid output acceleration and thrust setpoint otherwise something went wrong
	return valid && _thr_sp.isAllFinite();
}

void PositionMPC::_positionControl()
{
	// P-position controller
	matrix::Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);
	// Position and feed-forward velocity setpoints or position states being NAN results in them not having an influence
	ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
	// make sure there are no NAN elements for further reference while constraining
	ControlMath::setZeroIfNanVector3f(vel_sp_position);

	// Constrain horizontal velocity by prioritizing the velocity component along the
	// the desired position setpoint over the feed-forward term.
	_vel_sp.xy() = ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal);
	// Constrain velocity in z-direction.
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);
}

void PositionMPC::_velocityControl(const float dt)
{
	// Constrain vertical velocity integral
	_vel_int(2) = math::constrain(_vel_int(2), -CONSTANTS_ONE_G, CONSTANTS_ONE_G);

	// PID velocity control
	matrix::Vector3f vel_error = _vel_sp - _vel;
	matrix::Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d);

	// No control input from setpoints or corresponding states which are NAN
	ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

	_accelerationControl();

	// Integrator anti-windup in vertical direction
	if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.f) ||
	    (_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.f)) {
		vel_error(2) = 0.f;
	}

	// Prioritize vertical control while keeping a horizontal margin
	const matrix::Vector2f thrust_sp_xy(_thr_sp);
	const float thrust_sp_xy_norm = thrust_sp_xy.norm();
	const float thrust_max_squared = math::sq(_lim_thr_max);

	// Determine how much vertical thrust is left keeping horizontal margin
	const float allocated_horizontal_thrust = math::min(thrust_sp_xy_norm, _lim_thr_xy_margin);
	const float thrust_z_max_squared = thrust_max_squared - math::sq(allocated_horizontal_thrust);

	// Saturate maximal vertical thrust
	_thr_sp(2) = math::max(_thr_sp(2), -sqrtf(thrust_z_max_squared));

	// Determine how much horizontal thrust is left after prioritizing vertical control
	const float thrust_max_xy_squared = thrust_max_squared - math::sq(_thr_sp(2));
	float thrust_max_xy = 0.f;

	if (thrust_max_xy_squared > 0.f) {
		thrust_max_xy = sqrtf(thrust_max_xy_squared);
	}

	// Saturate thrust in horizontal direction
	if (thrust_sp_xy_norm > thrust_max_xy) {
		_thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
	}

	// Use tracking Anti-Windup for horizontal direction: during saturation, the integrator is used to unsaturate the output
	// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
	const matrix::Vector2f acc_sp_xy_produced = matrix::Vector2f(_thr_sp) * (CONSTANTS_ONE_G / _hover_thrust);
	const float arw_gain = 2.f / _gain_vel_p(0);

	// The produced acceleration can be greater or smaller than the desired acceleration due to the saturations and the actual vertical thrust (computed independently).
	// The ARW loop needs to run if the signal is saturated only.
	const matrix::Vector2f acc_sp_xy = _acc_sp.xy();
	const matrix::Vector2f acc_limited_xy = (acc_sp_xy.norm_squared() > acc_sp_xy_produced.norm_squared())
					? acc_sp_xy_produced
					: acc_sp_xy;
	vel_error.xy() = matrix::Vector2f(vel_error) - arw_gain * (acc_sp_xy - acc_limited_xy);

	// Make sure integral doesn't get NAN
	ControlMath::setZeroIfNanVector3f(vel_error);
	// Update integral part of velocity control
	_vel_int += vel_error.emult(_gain_vel_i) * dt;
}

void PositionMPC::_accelerationControl()
{
	// Assume standard acceleration due to gravity in vertical direction for attitude generation
	float z_specific_force = -CONSTANTS_ONE_G;

	if (!_decouple_horizontal_and_vertical_acceleration) {
		// Include vertical acceleration setpoint for better horizontal acceleration tracking
		z_specific_force += _acc_sp(2);
	}

	matrix::Vector3f body_z = matrix::Vector3f(-_acc_sp(0), -_acc_sp(1), -z_specific_force).normalized();
	ControlMath::limitTilt(body_z, matrix::Vector3f(0, 0, 1), _lim_tilt);
	// Convert to thrust assuming hover thrust produces standard gravity
	const float thrust_ned_z = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
	// Project thrust to planned body attitude
	const float cos_ned_body = (matrix::Vector3f(0, 0, 1).dot(body_z));
	const float collective_thrust = math::min(thrust_ned_z / cos_ned_body, -_lim_thr_min);
	_thr_sp = body_z * collective_thrust;
}

bool PositionMPC::_inputValid()
{
	bool valid = true;

	// Every axis x, y, z needs to have some setpoint
	for (int i = 0; i <= 2; i++) {
		valid = valid && (PX4_ISFINITE(_pos_sp(i)) || PX4_ISFINITE(_vel_sp(i)));
	}

	// x and y input setpoints always have to come in pairs
	valid = valid && (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1)));
	valid = valid && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1)));

	// For each controlled state the estimate has to be valid
	for (int i = 0; i <= 2; i++) {
		if (PX4_ISFINITE(_pos_sp(i))) {
			valid = valid && PX4_ISFINITE(_pos(i));
		}

		if (PX4_ISFINITE(_vel_sp(i))) {
			valid = valid && PX4_ISFINITE(_vel(i)) && PX4_ISFINITE(_vel_dot(i));
		}
	}

	return valid;
}

void PositionMPC::getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const
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

void PositionMPC::getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const
{
	ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, attitude_setpoint);
	attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;
}
