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
 * @file rate_control.cpp
 */

#include "rate_control_falcon.hpp"
#include <px4_platform_common/defines.h>



using namespace matrix;

/* RateControl::RateControl()
{
	// default gains for testing
} */

void RateControlFalcon::setPidGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	// Load bearing variables: LEAVE
	_gain_p = P;
	_gain_i = I;
	_gain_d = D;

	/* _roll_controller = RSLQR(_gain_p(0), _gain_i(0), 1.0f, -1.0f, _lim_int(0));
	_pitch_controller = RSLQR(_gain_p(1), _gain_i(1), 1.0f, -1.0f, _lim_int(1));
	_yaw_controller = RSLQR(_gain_p(2), _gain_i(2), 1.0f, -1.0f, _lim_int(2)); */


	//_roll_controller = new RSLQR("roll");
	//_pitch_controller = new RSLQR("pitch");
	//_yaw_controller = new RSLQR("yaw");

	_roll_controller = RSLQR(0.15500f, 0.75f, 1.0f, -1.0f, _lim_int(0));
	_pitch_controller = RSLQR(0.15500f, 0.75f, 1.0f, -1.0f, _lim_int(1));
	_yaw_controller = RSLQR(0.10f, 0.75f, 1.0f, -1.0f, _lim_int(2));




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
	float p_sp = rate_sp(0);
	float q_sp = rate_sp(1);
	float r_sp = rate_sp(2);

	float p = rate(0);
	float q = rate(1);
	float r = rate(2);

	float roll_torque 	= _roll_controller.update(p, p_sp, dt, landed);
	float pitch_torque 	= _pitch_controller.update(q, q_sp, dt, landed);
	float yaw_torque 	= _yaw_controller.update(r, r_sp, dt, landed);

	Vector3f torque = {roll_torque, pitch_torque, yaw_torque};

	// Export controller state to CSV for analysis
	/* if (_logControllerState) {
		std::vector<float> data = {
			{rate(0), rate(1), rate(2), 
			rate_sp(0), rate_sp(1), rate_sp(2), 
			angular_accel(0), angular_accel(1), angular_accel(2),
			torque(0), torque(1), torque(2)}
		};
		exportToCSV("controller_state.csv", data);
	} */
	
	return torque;
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