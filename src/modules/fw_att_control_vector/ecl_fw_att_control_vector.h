/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file ecl_fw_att_control_vector.cpp
 *
 * Fixed wing attitude controller
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Tobias Naegeli <naegelit@student.ethz.ch>
 *
 */

#include <mathlib/mathlib.h>

class ECL_FWAttControlVector {

public:
	ECL_FWAttControlVector();
	void control(float dt, float airspeed, float airspeed_scaling, const math::Dcm &R_nb, float roll, float pitch, float yaw, const math::Vector &F_des_in,
                                const math::Vector &angular_rates,
                                math::Vector &moment_des, float &thrust);

	void set_imax(float integral_max) {
		_integral_max(0) = integral_max;
		_integral_max(1) = integral_max;
	}

	void set_tconst(float tconst) {
		_p_tconst = tconst;
	}

	void set_k_p(float roll, float pitch, float yaw) {
		_k_p(0) = roll;
		_k_p(1) = pitch;
		_k_p(2) = yaw;
	}

	void set_k_d(float roll, float pitch, float yaw) {
		_k_d(0) = roll;
		_k_d(1) = pitch;
		_k_d(2) = yaw;
	}

	void set_k_i(float roll, float pitch, float yaw) {
		_k_i(0) = roll;
		_k_i(1) = pitch;
		_k_i(2) = yaw;
	}

	void reset_integral() {
		_integral_error(0) = 0.0f;
		_integral_error(1) = 0.0f;
	}

	void lock_integral(bool lock) {
		_integral_lock = lock;
	}

	bool airspeed_enabled() {
		return _airspeed_enabled;
	}

	void enable_airspeed(bool airspeed) {
		_airspeed_enabled = airspeed;
	}

	math::Vector3 get_rates_des() {
		return _rates_demanded;
	}

protected:
    math::Vector2f _integral_error;
    math::Vector2f _integral_max;
    math::Vector3 _rates_demanded;
    math::Vector3 _k_p;
    math::Vector3 _k_d;
    math::Vector3 _k_i;
    bool _integral_lock;
    float _p_airspeed_min;
    float _p_airspeed_max;
    float _p_tconst;
    float _p_roll_ffd;
    bool _airspeed_enabled;
};
