/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
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
 * @file fw_yaw_controller.h
 * Definition of a simple coordinated turn controller.
 *
 * Acknowledgements:
 *
 *   The control design is based on a design
 *   by Paul Riseborough and Andrew Tridgell, 2013,
 *   which in turn is based on initial work of
 *   Jonathan Challinger, 2012.
 */

#ifndef FW_YAW_CONTROLLER_H
#define FW_YAW_CONTROLLER_H

class YawController
{
public:
	YawController() = default;
	~YawController() = default;

	/**
	 * @brief Calculates both euler and body yaw rate setpoints for coordinated turn based on current attitude and airspeed
	 *
	 * @param roll_setpoint roll setpoint [rad]
	 * @param euler_pitch_rate_setpoint euler pitch rate setpoint [rad/s]
	 * @param roll estimated roll [rad]
	 * @param pitch estimated pitch [rad]
	 * @param airspeed airspeed [m/s]
	 * @return Roll body rate setpoint [rad/s]
	 */
	float control_yaw(float roll_setpoint, float euler_pitch_rate_setpoint, float roll, float pitch,
			  float airspeed);

	void set_max_rate(float max_rate) { _max_rate = max_rate; }

	float get_euler_rate_setpoint() { return _euler_rate_setpoint; }
	float get_body_rate_setpoint() { return _body_rate_setpoint; }

private:
	float _max_rate;
	float _euler_rate_setpoint;
	float _body_rate_setpoint;
};

#endif // FW_YAW_CONTROLLER_H
