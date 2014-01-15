/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author:    Tobias Naegeli
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
 * @file ecl_mc_att_control_vector.cpp
 *
 * Multirotor attitude controller based on concepts in:
 *
 * Minimum Snap Trajectory Generation and Control for Quadrotors
 * http://www.seas.upenn.edu/~dmel/mellingerICRA11.pdf
 *
 * @author Tobias Naegeli <naegelit@student.ethz.ch>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 *
 */

#include <mathlib/mathlib.h>

class ECL_MCAttControlVector {

public:
	ECL_MCAttControlVector();
	void control(float dt, const math::Dcm &R_nb, float yaw, const math::Vector &F_des,
                                float Kp, float Kd, float Ki, const math::Vector &angular_rates,
                                math::Vector &rates_des, float &thrust);

	void set_imax(float integral_max) {
		_integral_max(0) = integral_max;
		_integral_max(1) = integral_max;
	}

	void reset_integral() {
		_integral_error(0) = 0.0f;
		_integral_error(1) = 0.0f;
	}

	void lock_integrator(bool lock) {
		_integral_lock = lock;
	}

protected:
    math::Vector2f _integral_error;
    math::Vector2f _integral_max;
    bool _integral_lock;
};
