/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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
 * @file mavlink_simple_analyzer.h
 *
 * @author Achermann Florian <acfloria@ethz.ch>
 */

#pragma once

#include <float.h>
#include <stdint.h>

/**
 * SimpleAnalyzer
 *
 * Class used for simple analysis of data streams.
 * The data can be analyzed in the following three modes:
 *
 * AVERAGE:
 * 		The average value is computed at the beginning. Based on the number of analyzed values,
 * 		the update rate and the window size the switch to the moving average is determined.
 *
 * MIN:
 * 		The minimum value is tracked.
 *
 * MAX:
 * 		The maximum value is tracked.
 */
class SimpleAnalyzer
{
public:
	enum Mode {
		AVERAGE = 0,
		MIN,
		MAX,
	};

	/**
	 * Constructor
	 *
	 * Defines the mode of the analyzer and the window size in case of the
	 * averaging mode.
	 *
	 * @param[in] mode: The mode of the analyzer
	 * @param[in] window: The window size in seconds. Only used in the averaging mode.
	 */
	SimpleAnalyzer(Mode mode, float window = 60.0f);

	/**
	 * Reset the analyzer to the initial state.
	 */
	void reset();

	/**
	 * Add a new value to the analyzer and update the result according to the mode.
	 *
	 * @param[in] val: The value to process
	 * @param[in] update_rate: The update rate in [1/s] for which new value are added.
	 * 		Used in the averaging mode to determine when to switch from averaging to the moving average.
	 */
	void add_value(float val, float update_rate);

	/**
	 * Returns true if at least one value has been added to the analyzer.
	 */
	bool valid() const;

	/**
	 * Get the current result of the analyzer.
	 */
	float get() const;

	/**
	 * Get the scaled value of the current result of the analyzer.
	 *
	 * @param[in] scalingfactor: The factor used to scale the result.
	 */
	float get_scaled(float scalingfactor) const;

	/**
	 * Get the rounded scaled value casted to the input template type.
	 * Should only be used to return integer types.
	 *
	 * @param[out] ret: The scaled and rounded value of the current analyzer result.
	 * @parma[in] scalingfactor: The factor which is used to scale the result.
	 */
	void get_scaled(uint8_t &ret, float scalingfactor) const
	{
		float avg = get_scaled(scalingfactor);
		int_round(avg);
		check_limits(avg, 0, UINT8_MAX);

		ret = static_cast<uint8_t>(avg);
	}

	void get_scaled(int8_t &ret, float scalingfactor) const
	{
		float avg = get_scaled(scalingfactor);
		int_round(avg);
		check_limits(avg, INT8_MIN, INT8_MAX);

		ret = static_cast<int8_t>(avg);
	}

private:
	unsigned int _n = 0; /**< The number of added samples */
	float _window = 60.0f; /**< The window size for the moving average filter [s] */
	Mode _mode = AVERAGE; /**< The mode of the simple analyzer */
	float _result = 0.0f; /**< The result of the analyzed data. */

	void check_limits(float &x, float min, float max) const;
	void int_round(float &x) const;
};

void convert_limit_safe(float in, uint16_t &out);
void convert_limit_safe(float in, int16_t &out);
