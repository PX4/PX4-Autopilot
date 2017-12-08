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
 * @file FlightManualStabilized.hpp
 *
 * Flight task for manual controlled attitude.
 * It generates thrust and yaw setpoints.
 */

#pragma once

#include "FlightTaskManual.hpp"

class FlightTaskManualStabilized : public FlightTaskManual
{
public:
	FlightTaskManualStabilized(control::SuperBlock *parent, const char *name);

	virtual ~FlightTaskManualStabilized() = default;

	bool activate() override;

	bool update() override;

protected:
	float _yaw_rate_sp{}; /**< Scaled yaw rate from stick. NAN if yaw is locked. */
	float _yaw_sp{}; /**< Yaw setpoint once locked. Otherwise NAN. */
	matrix::Vector3f _thr_sp{}; /**< Thrust setpoint from sticks */

	virtual void _updateSetpoints(); /**< Updates all setpoints. */
	virtual void _scaleSticks(); /**< Scales sticks to yaw and thrust. */

private:

	float _throttle{}; /** Mapped from stick z. */

	void _updateHeadingSetpoints(); /**< Sets yaw or yaw speed. */
	void _updateThrustSetpoints(); /**< Sets thrust setpoint */
	float _throttleCurve(); /**< Piecewise linear mapping from stick to throttle. */

	control::BlockParamFloat _yaw_rate_scaling; /**< Scaling factor from stick to yaw rate. */
	control::BlockParamFloat _tilt_max_man; /**< Maximum tilt allowed for manual flight */
	control::BlockParamFloat _throttle_min; /**< Minimum throttle that always has to be satisfied in flight*/
	control::BlockParamFloat _throttle_max; /**< Maximum throttle that always has to be satisfied in flight*/
	control::BlockParamFloat _throttle_hover; /**< Throttle value at which vehicle is at hover equilibrium */
};
