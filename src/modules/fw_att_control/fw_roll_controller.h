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
 * @file fw_roll_controller.h
 * Definition of a simple roll P controller.
 */

#ifndef FW_ROLL_CONTROLLER_H
#define FW_ROLL_CONTROLLER_H

class RollController
{
public:
	RollController() = default;
	~RollController() = default;

	/**
	 * @brief Calculates both euler and body roll rate setpoints.
	 *
	 * @param roll_setpoint roll setpoint [rad]
	 * @param euler_yaw_rate_setpoint euler yaw rate setpoint [rad/s]
	 * @param roll estimated roll [rad]
	 * @param pitch estimated pitch [rad]
	 * @return Roll body rate setpoint [rad/s]
	 */
	float control_roll(float roll_setpoint, float euler_yaw_rate_setpoint, float roll, float pitch);

	void set_time_constant(float time_constant) { _tc = time_constant; }
	void set_max_rate(float max_rate) { _max_rate = max_rate; }

	float get_euler_rate_setpoint() { return _euler_rate_setpoint; }
	float get_body_rate_setpoint() { return _body_rate_setpoint; }

private:
	float _tc;
	float _max_rate;
	float _euler_rate_setpoint;
	float _body_rate_setpoint;
};

#endif // FW_ROLL_CONTROLLER_H
