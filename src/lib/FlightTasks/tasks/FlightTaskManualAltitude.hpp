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

	float _vel_sp_z{0.0f}; /**< scaled velocity directly from stick */
	float _yaw_rate_sp{0.0f}; /** scaled yaw rate directly from stick */
	float _pos_sp_z{0.0f};
	float _yaw_sp{0.0f};

	control::BlockParamFloat _vel_max_down; /**< maximum speed allowed to go up */
	control::BlockParamFloat _vel_max_up; /**< maximum speed allowed to go down */
	control::BlockParamFloat _yaw_rate_scaling; /**< scaling factor from stick to yaw rate */

	virtual void update_setpoints(); /**< updates all setpoints */
	virtual void scale_sticks(); /**< scales sticks to velocity */

private:
	void update_heading_setpoints(); /**< sets yaw or yaw speed */
	void update_z_setpoints(); /**< sets position or velocity setpoint */

	float _pos_sp_predicted{0.0f}; /**< position setpoint computed in set_z_setpoints */
	float _yaw_sp_predicted{0.0f}; /**< yaw setpoint computed in set_heading_setpoints */

};
