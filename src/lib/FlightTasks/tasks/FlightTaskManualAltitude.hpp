/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
	FlightTaskManualAltitude() = default;

	virtual ~FlightTaskManualAltitude() = default;

protected:
	void _updateAltitudeLock(); /**< checks for position lock */
	void _updateSetpoints() override; /**< updates all setpoints */
	void _scaleSticks() override; /**< scales sticks to velocity in z */

	DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTaskManualStabilized,
					(ParamFloat<px4::params::MPC_Z_VEL_MAX_DN>) _vel_max_down, /**< maximum speed allowed to go up */
					(ParamFloat<px4::params::MPC_Z_VEL_MAX_UP>) _vel_max_up, /**< maximum speed allowed to go down */
					(ParamFloat<px4::params::MPC_HOLD_MAX_Z>)
					_vel_hold_thr_z /**< velocity threshold to switch back into vertical position hold */
				       )

};
