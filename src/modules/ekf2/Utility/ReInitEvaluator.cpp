/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file ReInitEvaluator.cpp
 */

#include "ReInitEvaluator.hpp"

ReInitEvaluator::ReInitEvaluator()
{
	_vehicle_moving_hysteresis.set_hysteresis_time_from(false,
			_moved_time_threshold);
	_vehicle_moving_a_lot_hysteresis.set_hysteresis_time_from(false,
			_moved_a_lot_time_threshold);
}

void ReInitEvaluator::reset()
{
	const hrt_abstime t_now = hrt_absolute_time();
	_vehicle_moving_hysteresis.set_state_and_update(false, t_now);
	_vehicle_moving_a_lot_hysteresis.set_state_and_update(false, t_now);
	_vehicle_moved = false;
	_vehicle_moved_a_lot = false;
}

void ReInitEvaluator::update(const bool is_vehicle_at_rest)
{
	const hrt_abstime t_now = hrt_absolute_time();
	_vehicle_moving_hysteresis.set_state_and_update(!is_vehicle_at_rest, t_now);
	_vehicle_moving_a_lot_hysteresis.set_state_and_update(!is_vehicle_at_rest, t_now);
	updateMotionDetection();
}

void ReInitEvaluator::updateMotionDetection()
{
	if (_vehicle_moving_hysteresis.get_state()) {
		_vehicle_moved = true;
	}

	if (_vehicle_moving_a_lot_hysteresis.get_state()) {
		_vehicle_moved_a_lot = true;
	}
}
