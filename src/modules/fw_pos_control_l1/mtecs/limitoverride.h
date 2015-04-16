/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: 	@author Thomas Gubler <thomasgubler@gmail.com>
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
 * @file limitoverride.h
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */


#ifndef LIMITOVERRIDE_H_
#define LIMITOVERRIDE_H_

#include "mTecs_blocks.h"

namespace fwPosctrl
{

/* A small class which provides helper functions to override control output limits which are usually set by
* parameters in special cases
*/
class LimitOverride
{
public:
	LimitOverride() :
		overrideThrottleMinEnabled(false),
		overrideThrottleMaxEnabled(false),
		overridePitchMinEnabled(false),
		overridePitchMaxEnabled(false)
	{};

	~LimitOverride() {};

	/*
	* Override the limits of the outputlimiter instances given by the arguments with the limits saved in
	* this class (if enabled)
	* @return true if the limit was applied
	*/
	bool applyOverride(BlockOutputLimiter &outputLimiterThrottle,
			BlockOutputLimiter &outputLimiterPitch);

	/* Functions to enable or disable the override */
	void enableThrottleMinOverride(float value) { enable(&overrideThrottleMinEnabled,
			&overrideThrottleMin, value); }
	void disableThrottleMinOverride() { disable(&overrideThrottleMinEnabled); }
	void enableThrottleMaxOverride(float value) { enable(&overrideThrottleMaxEnabled,
			&overrideThrottleMax, value); }
	void disableThrottleMaxOverride() { disable(&overrideThrottleMaxEnabled); }
	void enablePitchMinOverride(float value) { enable(&overridePitchMinEnabled,
			&overridePitchMin, value); }
	void disablePitchMinOverride() { disable(&overridePitchMinEnabled); }
	void enablePitchMaxOverride(float value) { enable(&overridePitchMaxEnabled,
			&overridePitchMax, value); }
	void disablePitchMaxOverride() { disable(&overridePitchMaxEnabled); }

protected:
	bool overrideThrottleMinEnabled;
	float overrideThrottleMin;
	bool overrideThrottleMaxEnabled;
	float overrideThrottleMax;
	bool overridePitchMinEnabled;
	float overridePitchMin; //in degrees (replaces param values)
	bool overridePitchMaxEnabled;
	float overridePitchMax; //in degrees (replaces param values)

	/* Enable a specific limit override */
	void enable(bool *flag, float *limit, float value) { *flag = true; *limit = value; };

	/* Disable a specific limit override */
	void disable(bool *flag) { *flag = false; };
};

} /* namespace fwPosctrl */

#endif /* LIMITOVERRIDE_H_ */
