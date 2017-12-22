/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskOrbit.hpp
 *
 * Flight task for manual controlled altitude.
 *
 */

#pragma once

#include "FlightTaskManual.hpp"

class FlightTaskManualAltitude : public FlightTaskManual
{
public:
	FlightTaskManualAltitude(control::SuperBlock *parent, const char *name);

	virtual ~FlightTaskManualAltitude() = default;

	bool activate() override;

	bool update() override;

protected:
	float _yaw_rate_sp{0.0f}; /** scaled yaw rate directly from stick. NAN if yaw is locked */
	float _yaw_sp{0.0f}; /** yaw setpoint once locked. otherwise NAN */
	float _yaw_sp_predicted{0.0f}; /** _yaw_sp during lock; predicted yaw through _yaw_rate_sp demand */
	float _vel_sp_z{0.0f}; /**< scaled velocity directly from stick. During altitude lock is equal to NAN */
	float _pos_sp_z{0.0f}; /**< position setpoint in z during lock. Otherwise NAN. */

	control::BlockParamFloat _vel_max_down; /**< maximum speed allowed to go up */
	control::BlockParamFloat _vel_max_up; /**< maximum speed allowed to go down */
	control::BlockParamFloat _yaw_rate_scaling; /**< scaling factor from stick to yaw rate */
	control::BlockParamFloat _acc_max_up; /**< maximum acceleration upward */
	control::BlockParamFloat _acc_max_down; /**< maximum acceleration downward */

	virtual void updateSetpoints(); /**< updates all setpoints */
	virtual void scaleSticks(); /**< scales sticks to velocity */

private:
	void updateHeadingSetpoints(); /**< sets yaw or yaw speed */
	void updateZsetpoints(); /**< sets position or velocity setpoint */

	float _lock_time_max{0.0f}; /**< defines time when altitude lock occurs */
	float _lock_time{0.0f}; /**< time after stick are at zero position */
};
