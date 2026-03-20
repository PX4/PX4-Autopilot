/****************************************************************************
 *
 *	Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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

#include "green_tech.hpp"

namespace payload_board
{

GreenTech::GreenTech(const uint8_t charging_time) : _charge_time{charging_time} { _singlewire = true; }

int GreenTech::update_state()
{
	_countdown = 0;

	if (_msp_message.copyDataTo(_rx_buf, sizeof(_rx_buf)) == PX4_ERROR) {
		return PX4_ERROR;
	}

	if (memcmp(_rx_buf, "INACTIVE", sizeof("INACTIVE")) == 0) {
		_last_state = payload_response_s::STATE_INACTIVE;

	} else if (memcmp(_rx_buf, "CHARGING", sizeof("CHARGING")) == 0) {
		if (_last_state != payload_response_s::STATE_CHARGING) { _charging_start = hrt_absolute_time(); }

		_last_state = payload_response_s::STATE_CHARGING;
		const int16_t x = _charge_time - (hrt_absolute_time() - _charging_start) / 1e+6;
		_countdown = x < 0 ? 0 : x;

	} else if (memcmp(_rx_buf, "READY", sizeof("READY")) == 0) {
		_last_state = payload_response_s::STATE_READY;

	} else if (memcmp(_rx_buf, "ACTIVE", sizeof("ACTIVE")) == 0) {
		_last_state = payload_response_s::STATE_ACTIVE;

	} else {
		_last_state = payload_response_s::STATE_ERR;
	}

	return PX4_OK;
}

int GreenTech::get_custom_message(uint8_t *message, const size_t size)
{
	return _msp_message.copyDataTo(message, size) >= 0 ? PX4_OK : PX4_ERROR;
}

void GreenTech::begin_read()
{
	Protocol::begin_read();
	_msp_message.clearData();
}
}  // namespace payload_board
