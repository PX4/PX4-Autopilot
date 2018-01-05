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
 * @file FlightManualAltitude.hpp
 *
 * Flight task for manual controlled altitude.
 */

#pragma once

#include "FlightTaskManualStabilized.hpp"

class FlightTaskManualAltitude : public FlightTaskManualStabilized
{
public:
	FlightTaskManualAltitude(control::SuperBlock *parent, const char *name);

	virtual ~FlightTaskManualAltitude() = default;

	bool activate() override;

	bool update() override;

protected:
	float _vel_sp_z{}; /**< Scaled velocity from stick. During altitude lock it is equal to NAN. */
	float _pos_sp_z{}; /**< Setpoint in z during lock. Otherwise NAN. */

	control::BlockParamFloat _vel_max_down; /**< Maximum speed allowed to go up. */
	control::BlockParamFloat _vel_max_up; /**< Maximum speed allowed to go down. */
	control::BlockParamFloat _vel_z_dz; /**< velocity threshold/deadzone to switch into vertical position hold */

	void _updateAltitudeLock(); /**< Checks for position lock. */
	void _updateSetpoints() override; /**< Updates all setpoints. */
	void _scaleSticks() override; /**< Scales sticks to velocity in z. */

};
