/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file AttitudeControl.hpp
 *
 * A quaternion based attitude controller.
 *
 * @author Matthias Grob	<maetugr@gmail.com>
 *
 * Publication documenting the implemented Quaternion Attitude Control:
 * Nonlinear Quadrocopter Attitude Control (2013)
 * by Dario Brescianini, Markus Hehn and Raffaello D'Andrea
 * Institute for Dynamic Systems and Control (IDSC), ETH Zurich
 *
 * https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>
#include <mathlib/mathlib.h>

class AttitudeControl
{
public:
	AttitudeControl() = default;
	~AttitudeControl() = default;

	/**
	 * Set proportional attitude control gain
	 * @param proportional_gain 3D vector containing gains for roll, pitch, yaw
	 * @param yaw_weight A fraction [0,1] deprioritizing yaw compared to roll and pitch
	 */
	void setProportionalGain(const matrix::Vector3f &proportional_gain, const float yaw_weight);

	/**
	 * Set hard limit for output rate setpoints
	 * @param rate_limit [rad/s] 3D vector containing limits for roll, pitch, yaw
	 */
	void setRateLimit(const matrix::Vector3f &rate_limit) { _rate_limit = rate_limit; }

	/**
	 * Set a new attitude setpoint replacing the one tracked before
	 * @param qd desired vehicle attitude setpoint
	 * @param yawspeed_setpoint [rad/s] yaw feed forward angular rate in world frame
	 */
	void setAttitudeSetpoint(const matrix::Quatf &qd, const float yawspeed_setpoint) { _attitude_setpoint_q = qd; _attitude_setpoint_q.normalize(); _yawspeed_setpoint = yawspeed_setpoint; }

	/**
	 * Adjust last known attitude setpoint by a delta rotation
	 * Optional use to avoid glitches when attitude estimate reference e.g. heading changes.
	 * @param q_delta delta rotation to apply
	 */
	void adaptAttitudeSetpoint(const matrix::Quatf &q_delta) { _attitude_setpoint_q = q_delta * _attitude_setpoint_q; }

	/**
	 * Run one control loop cycle calculation
	 * @param q estimation of the current vehicle attitude unit quaternion
	 * @return [rad/s] body frame 3D angular rate setpoint vector to be executed by the rate controller
	 */
	//matrix::Vector3f update(const matrix::Quatf &q) const;
	matrix::Vector3f update(const matrix::Quatf &q, const bool landed);

	/**
	 * 	Get the
	 * 	@see z_k_Pr_R
	 * 	@return The z variable used by RCAC in the PID+FF controller
	 */
	const matrix::Vector3f get_RCAC_att_z()
	{
		matrix::Vector3f RCAC_z{};

		for (int i = 0; i <= 2; i++) {
			RCAC_z(i) = z_k_Pq_R(i);
		}

		return RCAC_z;
	}

	/**
	 * 	Get the
	 * 	@see u_k_Pr_R
	 * 	@return The u variable computed by RCAC in the PID+FF controller
	 */
	const matrix::Vector3f get_RCAC_att_u()
	{
		matrix::Vector3f RCAC_u{};

		for (int i = 0; i <= 2; i++) {
			RCAC_u(i) = u_k_Pq_R(i);
		}

		return RCAC_u;
	}

	/**
	 * 	Get the
	 * 	@see theta_k_Pr_R
	 * 	@return The theta variable computed by RCAC in the PID+FF controller
	 */
	const matrix::Vector3f get_RCAC_att_theta()
	{
		matrix::Vector3f RCAC_theta{};

		for (int i = 0; i <= 2; i++) {
			RCAC_theta(i) = theta_k_Pq_R(i);
		}

		return RCAC_theta;
	}

	/**
	 * 	Get the
	 * 	@see _proportional_gain
	 * 	@return PX4 PID gains for the attitude controller
	 */
	const matrix::Vector3f get_PX4_att_theta()
	{
		matrix::Vector3f PX4_att_theta{};

		for (int i = 0; i <= 2; i++) {
			PX4_att_theta(i) = _proportional_gain(i);
		}

		return PX4_att_theta;
	}

	/**
	 * 	Get the
	 * 	@see ii
	 * 	@return Iteration step of the RCAC attitude controller
	 */
	const int &get_RCAC_att_ii() { return ii_Pq_R; }

	/**
	 * 	Get the
	 * 	@see RCAC_Aq_ON
	 * 	@return Get RCAC attitude controller switch
	 */
	const bool &get_RCAC_att_switch() {return RCAC_Aq_ON;}

	/**
	 * 	Get the
	 * 	@see alpha_PID_att
	 * 	@return Get the gain that multiplies the attitude PID gains.
	 */
	const float &get_alpha_PID_att() {return alpha_PID_att;}

	/**
	 * 	Set the RCAC Attitude switch.
	 * 	@see RCAC_Aq_ON
	 */
	void set_RCAC_att_switch(float switch_RCAC)
	{
		RCAC_Aq_ON = 1;
		if (switch_RCAC<0.0f) {
			RCAC_Aq_ON = 0;
		}
	}

	/**
	 * 	Set the PID scaling factor.
	 * 	@see alpha_PID
	 */
	void set_PID_att_factor(float PID_factor, float PID_val)
	{
		//alpha_PID = 1;
		alpha_PID_att = 1.0f;
		if (PID_factor<0.0f) {
			//alpha_PID = 0.5;
			alpha_PID_att = PID_val;
		}
	}

	/**
	 * 	Get the
	 * 	@see P_Pq_R
	 * 	@return RCAC P(1,1) of the Attitude controller
	 */
	const float &get_RCAC_P11_Att() { return P_Pq_R(0,0); }

	/**
	 * 	Set RCAC variables.
	 * 	@see all RCAC variables
	 */
	void init_RCAC_att()
	{
		// P_Pq_R = eye<float, 3>() * 0.010;
		// N1_Pq = eye<float, 3>();
		// I3 = eye<float, 3>();
		P_Pq_R.setZero();
		N1_Pq.setZero();
		I3.setZero();
		for (int i = 0; i <= 2; i++) {
			P_Pq_R(i,i) = 0.01f;
			P_Pq_R(i,i) = rcac_att_P0;
			N1_Pq(i,i) = 1.0f;
			I3(i,i) = 1.0f;
		}
		phi_k_Pq_R.setZero();
		phi_km1_Pq_R.setZero();
		theta_k_Pq_R.setZero();
		z_k_Pq_R.setZero();
		z_km1_Pq_R.setZero();
		u_k_Pq_R.setZero();
		u_km1_Pq_R.setZero();
		Gamma_Pq_R.setZero();
	}

	/**
	 * 	Set the RCAC Attiude Controller P0.
	 * 	@see rcac_att_P0
	 */
	void set_RCAC_att_P0(float att_P0)
	{
		rcac_att_P0 = att_P0;
	}
	// const float &get_RCAC_att_P0() {return P_Pq_R(0,0);}

private:
	matrix::Vector3f _proportional_gain;
	matrix::Vector3f _rate_limit;
	float _yaw_w{0.f}; ///< yaw weight [0,1] to deprioritize caompared to roll and pitch

	matrix::Quatf _attitude_setpoint_q; ///< latest known attitude setpoint e.g. from position control
	float _yawspeed_setpoint{0.f}; ///< latest known yawspeed feed-forward setpoint

	int ii_Pq_R = 0;
  	bool RCAC_Aq_ON=1;
	matrix::SquareMatrix<float, 3> P_Pq_R;
	matrix::Matrix<float, 3,3> phi_k_Pq_R, phi_km1_Pq_R;
	//matrix::Matrix<float, 3,1> theta_k_Pq_R,theta_k_Pq_PID;
  	//matrix::Matrix<float, 3,1> z_k_Pq_R, z_km1_Pq_R,u_k_Pq_R, u_km1_Pq_R;
	matrix::Vector3f theta_k_Pq_R,theta_k_Pq_PID;
  	matrix::Vector3f z_k_Pq_R, z_km1_Pq_R,u_k_Pq_R, u_km1_Pq_R;
	matrix::SquareMatrix<float, 3> Gamma_Pq_R, I3, N1_Pq;

	// // New RCAC_Class_Variables
	// RCAC _rcac_att_x;
	// RCAC _rcac_att_y;
	// RCAC _rcac_att_z;

	//float alpha_PID = 1.0f;
	float alpha_PID_att = 1.0f;
	float rcac_att_P0 = 0.011f;

};
