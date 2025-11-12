 /****************************************************************************
  *
  *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

 #include "auxReader.hpp"

#include <lib/mathlib/mathlib.h>

namespace
{
constexpr float kPwmMinUs{1000.f};
constexpr float kPwmMaxUs{2000.f};
constexpr float kPwmMidUs{(kPwmMinUs + kPwmMaxUs) * 0.5f};
constexpr float kPwmHalfRange{(kPwmMaxUs - kPwmMinUs) * 0.5f};
}

bool AuxReader::update()
{
	input_rc_s input_rc{};

	if (_input_rc_sub.update(&input_rc)) {
		const bool input_valid = !input_rc.rc_lost && !input_rc.rc_failsafe && (input_rc.channel_count > 0);

		if (input_valid) {
			const uint8_t channel_count_limited = math::min(input_rc.channel_count, static_cast<uint8_t>(CHANNEL_COUNT));

			for (uint8_t i = 0; i < channel_count_limited; ++i) {
				_channel_values(i) = normalizePwmValue(input_rc.values[i]);
			}

			for (uint8_t i = channel_count_limited; i < CHANNEL_COUNT; ++i) {
				_channel_values(i) = NAN;
			}

		} else {
			for (int i = 0; i < CHANNEL_COUNT; ++i) {
				_channel_values(i) = 0.f;
			}
		}

		_available = input_valid;
		return true;
	}

	return false;
}

float AuxReader::getChannel(int channel_index) const
{
	if ((channel_index < 0) || (channel_index >= CHANNEL_COUNT)) {
		return NAN;
	}

	return _available ? _channel_values(channel_index) : NAN;
}

float AuxReader::getFromParameterMapping(int parameter_value) const
{
	if (parameter_value <= 0) {
		return 0.f;
	}

	const int sanitized_index = math::constrain(parameter_value - 1, 0, CHANNEL_COUNT - 1);
	return getChannel(sanitized_index);
}

float AuxReader::normalizePwmValue(uint16_t pwm) const
{
	const float pwm_f = static_cast<float>(pwm);
	const float normalized = (pwm_f - kPwmMidUs) / kPwmHalfRange;
	return math::constrain(normalized, -1.f, 1.f);
}
