/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file hysteresis.h
 *
 * Hysteresis of a boolean value.
 *
 * @author Julian Oes <julian@oes.ch>
 */

#pragma once

#include <drivers/drv_hrt.h>

namespace systemlib
{

class Hysteresis
{
public:
	Hysteresis(bool init_state) :
		_state(init_state),
		_requested_state(init_state),
		_hysteresis_time_from_true_us(0),
		_hysteresis_time_from_false_us(0),
		_last_time_to_change_state(0)
	{}

	~Hysteresis()
	{}

	void set_hysteresis_time_from(const bool from_state, const hrt_abstime new_hysteresis_time_us)
	{
		if (from_state == true) {
			_hysteresis_time_from_true_us = new_hysteresis_time_us;

		} else {
			_hysteresis_time_from_false_us = new_hysteresis_time_us;
		}
	}

	bool get_state() const
	{
		return _state;
	}

	void set_state_and_update(const bool new_state);

	void update();

private:

	bool _state;
	bool _requested_state;
	hrt_abstime _hysteresis_time_from_true_us;
	hrt_abstime _hysteresis_time_from_false_us;
	hrt_abstime _last_time_to_change_state;
};

} // namespace systemlib
