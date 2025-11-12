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

 #pragma once

#include <matrix/math.hpp>
#include <px4_platform_common/defines.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/input_rc.h>

 /**
  * @class AuxReader
  *
 * @brief Utility helper to expose RC channel values (normalized).
  *
 * The class subscribes to `input_rc` and keeps the latest raw RC channel values,
 * scaled to approximately [-1, 1] assuming a PWM range of [1000, 2000] µs.
  */
 class AuxReader
 {
 public:
	static constexpr int CHANNEL_COUNT{input_rc_s::RC_INPUT_MAX_CHANNELS};

 	AuxReader() = default;
 	~AuxReader() = default;

 	/**
 	 * @brief Poll the underlying subscription for updates.
 	 *
 	 * @return true if new data has been retrieved, false otherwise.
 	 */
 	bool update();

 	/**
 	 * @return true if a valid manual control input is currently available.
 	 */
 	bool isAvailable() const { return _available; }

 	/**
 	 * @brief Get a specific AUX channel value.
 	 *
 	 * @param channel_index zero-based index (0 corresponds to AUX1).
 	 * @return the requested channel value in range [-1, 1] if available,
 	 *         NAN otherwise.
 	 */
 	float getChannel(int channel_index) const;

 	/**
 	 * @brief Helper for parameters mapping AUX channels.
 	 *
 	 * Parameters typically store AUX channels with 1-based indexing:
 	 *   0 -> unmapped
 	 *   1 -> AUX1
 	 *   ...
 	 *   6 -> AUX6
 	 *
 	 * @param parameter_value value coming from a parameter (0 meaning unmapped).
 	 * @return channel value in range [-1, 1] when mapped, otherwise 0.
 	 */
 	float getFromParameterMapping(int parameter_value) const;

 	/**
 	 * @return a copy of all AUX channel values.
 	 */
	matrix::Vector<float, CHANNEL_COUNT> getAllChannels() const { return _channel_values; }

 private:
	float normalizePwmValue(uint16_t pwm) const;

	uORB::Subscription _input_rc_sub{ORB_ID(input_rc)};
	matrix::Vector<float, CHANNEL_COUNT> _channel_values{};
 	bool _available{false};
 };
