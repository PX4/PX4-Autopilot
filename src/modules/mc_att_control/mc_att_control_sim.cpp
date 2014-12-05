/* Copyright (c) 2014 PX4 Development Team. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in
* the documentation and/or other materials provided with the
* distribution.
* 3. Neither the name PX4 nor the names of its contributors may be
* used to endorse or promote products derived from this software
* without specific prior written permission.
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
 * @file mc_att_control_sim.cpp
 *
 * MC Attitude Controller Interface for usage in a simulator
 *
 * @author Roman Bapst <bapstr@ethz.ch>
 *
 */

#include "mc_att_control_sim.h"
#include <geo/geo.h>
#include <math.h>

#ifdef CONFIG_ARCH_ARM
#else
#include <cmath>
using namespace std;
#endif

MulticopterAttitudeControlSim::MulticopterAttitudeControlSim()
{
	/* setup standard gains */
	//XXX: make these configurable
	_params.att_p(0)      = 5.0;
	_params.rate_p(0)     = 0.05;
	_params.rate_i(0)     = 0.0;
	_params.rate_d(0)     = 0.003;
	/* pitch gains */
	_params.att_p(1)      = 5.0;
	_params.rate_p(1)     = 0.05;
	_params.rate_i(1)     = 0.0;
	_params.rate_d(1)     = 0.003;
	/* yaw gains */
	_params.att_p(2)      = 2.8;
	_params.rate_p(2)     = 0.2;
	_params.rate_i(2)     = 0.1;
	_params.rate_d(2)     = 0.0;
	_params.yaw_rate_max  = 0.5;
	_params.yaw_ff        = 0.5;
	_params.man_roll_max  = 0.6;
	_params.man_pitch_max = 0.6;
	_params.man_yaw_max   = 0.6;
}

MulticopterAttitudeControlSim::~MulticopterAttitudeControlSim()
{
}

void MulticopterAttitudeControlSim::set_attitude(const Eigen::Quaternion<double> attitude)
{
	math::Quaternion quat;
	quat(0) = (float)attitude.w();
	quat(1) = (float)attitude.x();
	quat(2) = (float)attitude.y();
	quat(3) = (float)attitude.z();

	_v_att.q[0] = quat(0);
	_v_att.q[1] = quat(1);
	_v_att.q[2] = quat(2);
	_v_att.q[3] = quat(3);

	math::Matrix<3, 3> Rot = quat.to_dcm();
	_v_att.R[0][0] = Rot(0, 0);
	_v_att.R[1][0] = Rot(1, 0);
	_v_att.R[2][0] = Rot(2, 0);
	_v_att.R[0][1] = Rot(0, 1);
	_v_att.R[1][1] = Rot(1, 1);
	_v_att.R[2][1] = Rot(2, 1);
	_v_att.R[0][2] = Rot(0, 2);
	_v_att.R[1][2] = Rot(1, 2);
	_v_att.R[2][2] = Rot(2, 2);

	_v_att.R_valid = true;
}

void MulticopterAttitudeControlSim::set_attitude_rates(const Eigen::Vector3d &angular_rate)
{
	// check if this is consistent !!!
	_v_att.rollspeed  = angular_rate(0);
	_v_att.pitchspeed = angular_rate(1);
	_v_att.yawspeed   = angular_rate(2);
}

void MulticopterAttitudeControlSim::set_attitude_reference(const Eigen::Vector4d &control_attitude_thrust_reference)
{
	_v_att_sp.roll_body  = control_attitude_thrust_reference(0);
	_v_att_sp.pitch_body = control_attitude_thrust_reference(1);
	_v_att_sp.yaw_body   = control_attitude_thrust_reference(2);
	_v_att_sp.thrust     = (control_attitude_thrust_reference(3) - 30) * (-1) / 30;

	// setup rotation matrix
	math::Matrix<3, 3> Rot_sp;
	Rot_sp.from_euler(_v_att_sp.roll_body, _v_att_sp.pitch_body, _v_att_sp.yaw_body);
	_v_att_sp.R_body[0][0] = Rot_sp(0, 0);
	_v_att_sp.R_body[1][0] = Rot_sp(1, 0);
	_v_att_sp.R_body[2][0] = Rot_sp(2, 0);
	_v_att_sp.R_body[0][1] = Rot_sp(0, 1);
	_v_att_sp.R_body[1][1] = Rot_sp(1, 1);
	_v_att_sp.R_body[2][1] = Rot_sp(2, 1);
	_v_att_sp.R_body[0][2] = Rot_sp(0, 2);
	_v_att_sp.R_body[1][2] = Rot_sp(1, 2);
	_v_att_sp.R_body[2][2] = Rot_sp(2, 2);
}

void MulticopterAttitudeControlSim::get_mixer_input(Eigen::Vector4d &motor_inputs)
{
	motor_inputs(0) = _actuators.control[0];
	motor_inputs(1) = _actuators.control[1];
	motor_inputs(2) = _actuators.control[2];
	motor_inputs(3) = _actuators.control[3];
}
