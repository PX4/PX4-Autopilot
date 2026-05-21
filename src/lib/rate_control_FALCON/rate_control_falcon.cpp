/****************************************************************************
 *
 *   Copyright (c) 2019-2023 PX4 Development Team. All rights reserved.
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
 * @file rate_control_falcon.cpp
 */

#include "rate_control_falcon.hpp"
#include <px4_platform_common/defines.h>
#include <iostream>



using namespace matrix;

/* RateControl::RateControl()
{
	// default gains for testing
} */

void RateControlFalcon::setPidGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{

	// _roll_controller.set_gains(P(0), I(0), D(0));
	// _pitch_controller.set_gains(P(1), I(1), D(1));
	// _yaw_controller.set_gains(P(2), I(2), D(2));
	// _gain_p = P;
	// _gain_i = I;
	// _gain_d = D;

	_roll_controller.set_gains(0.2f, 0.00075f, 0.0f);
	_pitch_controller.set_gains(0.2f, 0.00075f, 0.0f);
	_yaw_controller.set_gains(0.5f, 0.0075f, 0.0f);

	/* _roll_controller = RSLQR(_gain_p(0), _gain_i(0), 1.0f, -1.0f, _lim_int(0));
	_pitch_controller = RSLQR(_gain_p(1), _gain_i(1), 1.0f, -1.0f, _lim_int(1));
	_yaw_controller = RSLQR(_gain_p(2), _gain_i(2), 1.0f, -1.0f, _lim_int(2)); */


	//_roll_controller = new RSLQR("roll");
	//_pitch_controller = new RSLQR("pitch");
	//_yaw_controller = new RSLQR("yaw");

	// 4/21 4:18  Sluggish, overdamped
/* 	_roll_controller = RSLQR(0.15500f, 0.75f, 1.0f, -1.0f, _lim_int(0));
	_pitch_controller = RSLQR(0.15500f, 0.75f, 1.0f, -1.0f, _lim_int(1));
	_yaw_controller = RSLQR(0.10f, 0.75f, 1.0f, -1.0f, _lim_int(2));
 */
	// 4:40			Strange Yaw behavior
	/* _roll_controller 	= RSLQR(0.15500f, 0.75f, 1.0f, -1.0f, _lim_int(0));
	_pitch_controller 	= RSLQR(0.15500f, 0.75f, 1.0f, -1.0f, _lim_int(1));
	_yaw_controller 	= RSLQR(0.10f	, 1.25f, 1.0f, -1.0f, _lim_int(2)); */

	// 4:45 Faught back less on yaw



 // 5:10 Death Spiral: Was miuch less responsive in yaw rate. recommend moving kp back to previous value and adjusting integral gain
 //possibly increase the proportional gain. It ended up suddenly spiraling/decending in yaw rate. not sure why

	/* _roll_controller 	= RSLQR(0.15500f, 0.75f, 1.0f, -1.0f, _lim_int(0));
	_pitch_controller 	= RSLQR(0.15500f, 0.75f, 1.0f, -1.0f, _lim_int(1));
	_yaw_controller 	= RSLQR(0.010f	, 1.00f, 1.0f, -1.0f, _lim_int(2)); */

	// DAY 2

// Trouble taking off
/* 	_roll_controller 	= RSLQR(0.15500f, 0.75f, 1.0f, -1.0f, _lim_int(0));
	_pitch_controller 	= RSLQR(0.15500f, 0.75f, 1.0f, -1.0f, _lim_int(1));
	_yaw_controller 	= RSLQR(0.150f	, 1.75f, 1.0f, -1.0f, _lim_int(2));
 */
// Sluggish, slower but more stable

/* _roll_controller = RSLQR(0.15500f, 0.75f, 1.0f, -1.0f, _lim_int(0));
	_pitch_controller = RSLQR(0.15500f, 0.75f, 1.0f, -1.0f, _lim_int(1));
	_yaw_controller = RSLQR(0.05f, 0.35f, 1.0f, -1.0f, _lim_int(2)); */

	/* _roll_controller 	= RSLQR(0.15500f, 0.75f	, 1.0f, -1.0f, _lim_int(0));
	_pitch_controller 	= RSLQR(0.15500f, 0.75f	, 1.0f, -1.0f, _lim_int(1));
	_yaw_controller 	= RSLQR(0.05f	, 0.5f	, 10.0f, -10.0f, _lim_int(2)); */


}

void RateControlFalcon::setSaturationStatus(const Vector3<bool> &saturation_positive,
				      const Vector3<bool> &saturation_negative)
{
	_control_allocator_saturation_positive = saturation_positive;
	_control_allocator_saturation_negative = saturation_negative;
}

void RateControlFalcon::setPositiveSaturationFlag(size_t axis, bool is_saturated)
{
	if (axis < 3) {
		_control_allocator_saturation_positive(axis) = is_saturated;
	}
}


void RateControlFalcon::setNegativeSaturationFlag(size_t axis, bool is_saturated)
{
	if (axis < 3) {
		_control_allocator_saturation_negative(axis) = is_saturated;
	}
}

Vector3f RateControlFalcon::update(const Vector3f &rate, const Vector3f &rate_sp, const Vector3f &angular_accel,
			     const float dt, const bool landed)
{

	//(roll, pitch, yaw)
	float p = rate(0);
	float q = rate(1);
	float r = rate(2);

	float p_sp = rate_sp(0);
	float q_sp = rate_sp(1);
	float r_sp = rate_sp(2);

	float p_ang = angular_accel(0);
	float q_ang = angular_accel(1);
	float r_ang = angular_accel(2);

	// float p_int_lim = _lim_int(0);
	// float q_int_lim = _lim_int(1);
	// float r_int_lim = _lim_int(2);

	float p_int_lim = 10.0;
	float q_int_lim = 10.0;
	float r_int_lim = 10.0;



	float roll_torque 	= _roll_controller.update(p, p_sp, p_ang, p_int_lim, dt, landed);
	float pitch_torque 	= _pitch_controller.update(q, q_sp, q_ang, q_int_lim, dt, landed);
	float yaw_torque 	= _yaw_controller.update(r, r_sp, r_ang, r_int_lim, dt, landed);



	Vector3f f_torque = {roll_torque, pitch_torque, yaw_torque};

	std::cout << "TIMESTAMP: " << dt << std::endl;
	std::cout << "Roll: " << _roll_controller._rate_int<< std::endl;
	std::cout << "Pitch: " << _pitch_controller._rate_int<< std::endl;
	std::cout << "Yaw: " << _yaw_controller._rate_int<< std::endl;

	return f_torque;
}


void RateControlFalcon::getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status)
{
	rate_ctrl_status.rollspeed_integ = _rate_int(0);
	rate_ctrl_status.pitchspeed_integ = _rate_int(1);
	rate_ctrl_status.yawspeed_integ = _rate_int(2);
}

/* void RateControlFalcon::exportToCSV(const std::string& filename, const std::vector<float>& data) {
	std::ifstream infile(filename);

    bool exists = infile.good();

    std::ofstream file(filename, std::ios::app);

    if (!file.is_open()) {
        throw std::runtime_error("Could not open file");
    }

    // Write header only if file didn't exist
    if (!exists) {
        file << "p,q,r,p_sp,q_sp,r_sp,angular_accel_x,angular_accel_y,angular_accel_z,torque_x,torque_y,torque_z\n";
    }

    for (const auto& value : data) {
        file << value << ",";
    }
    file << "\n";
} */
