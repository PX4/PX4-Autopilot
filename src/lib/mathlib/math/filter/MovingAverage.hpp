/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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

/*
 * @file MovingAverage.hpp
 *
 * @brief Implementation of a moving average of fixed window size.
 *
 */

#pragma once

#include <stdint.h>

template<typename T, uint32_t WINDOW = 3>
class MovingAverage
{
public:
	static_assert(WINDOW >= 2, "Window must be two or more samples");

	MovingAverage() = default;

	void insert(const T &sample)
	{
		_buffer[_head] = sample;
		_head = (_head + 1) % WINDOW;
	}

	T getState()
	{
		T sum{0};

		for (uint32_t i = 0; i < WINDOW; i++) {
			sum += _buffer[i];
		}

		return sum / static_cast<T>(WINDOW);
	}

	T apply(const T &sample)
	{
		insert(sample);
		return getState();
	}

	void reset(const T &sample)
	{
		for (uint32_t i = 0; i < WINDOW; i++) {
			_buffer[i] = sample;
		}
	}

private:
	T _buffer[WINDOW] {};
	uint32_t _head{0};
};
