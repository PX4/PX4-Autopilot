/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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
 * @file signal_generator.cpp
 *
 * @author Mehmet Enes AVCU <avcupyrz@gmail.com>
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>

#define pi 3.14
class SignalGenerator final
{
public:
	SignalGenerator() = default;
	~SignalGenerator() = default;

	/**
	 * @brief Set the Signal Ampiltude Function
	 *
	 * @param amplitude Amplitude od Signal
	 */
	void setSignalAmpiltude(float amplitude) { _amplitude = amplitude;}
	/**
	 * @brief Set the Signal Frequency Function
	 *
	 * @param frequency Frequecny of Signal
	 */
	void setSignalFrequency(float frequency) { _frequency = frequency;}
	/**
	 * @brief Set the Signal Phase Function
	 *
	 * @param phase Phase Angle
	 */
	void setSignalPhase(float phase) { _phase = phase;}
	/**
	 * @brief Set the Signal Start Frequency Function
	 *
	 * @param phase Start Frequency
	 */
	void setSignalStartFrequency(float phase) { _start_frequency = phase;}
	/**
	 * @brief Set the Signal End Frequency Function
	 *
	 * @param phase End Frequency
	 */
	void setSignalEndFrequency(float phase) { _end_frequency = phase;}
	/**
	 * @brief Set the Signal Duration Function
	 *
	 * @param duration Duration of Generated Signal
	 */
	void setSignalDuration(float duration) { _duration = duration;}



	/**
	 * @brief To Generate Sinüs Signal
	 *
	 * @param dt Delta Time
	 * @return Sinüs Signal
	 */
	float generateSinusSignal(const float dt);

	/**
	 * @brief To Generate Chirp Signal
	 *
	 * @param dt Delta Time
	 * @return Chirp Signal
	 */
	float generateChirpSignal(const float dt);


private:
	// Define variables
	float _amplitude{0.0f};
	float _frequency{0.0f};
	float _phase{0.0f};
	float _start_frequency{0.0f};
	float _end_frequency{0.0f};
	float _duration{0.0f};

};
