/****************************************************************************
 *
 *   Copyright (c) 2020-2023 PX4 Development Team. All rights reserved.
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
 * @file fw_wheel_controller.h
 * Definition of a simple wheel controller.
 */
#ifndef FW_WHEEL_CONTROLLER_H
#define FW_WHEEL_CONTROLLER_H

class WheelController
{
public:
	WheelController() = default;
	~WheelController() = default;

	/**
	 * @brief Calculates wheel body rate setpoint.
	 *
	 * @param yaw_setpoint yaw setpoint [rad]
	 * @param yaw estimated yaw [rad]
	 * @return Wheel body rate setpoint [rad/s]
	 */
	float control_attitude(float yaw_setpoint, float yaw);

	float control_bodyrate(float dt, float body_z_rate, float groundspeed, float groundspeed_scaler);

	void set_time_constant(float time_constant) { _tc = time_constant; }
	void set_k_p(float k_p) { _k_p = k_p; }
	void set_k_i(float k_i) { _k_i = k_i; }
	void set_k_ff(float k_ff) { _k_ff = k_ff; }
	void set_integrator_max(float max) { _integrator_max = max; }
	void set_max_rate(float max_rate) { _max_rate = max_rate; }

	void reset_integrator() { _integrator = 0.f; }

private:
	float _tc;
	float _k_p;
	float _k_i;
	float _k_ff;
	float _integrator_max;
	float _max_rate;
	float _last_output;
	float _integrator;
	float _body_rate_setpoint;
};

#endif // FW_WHEEL_CONTROLLER_H
