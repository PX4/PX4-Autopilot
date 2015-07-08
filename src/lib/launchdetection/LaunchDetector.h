/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file LaunchDetector.h
 * Auto Detection for different launch methods (e.g. catapult)
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#ifndef LAUNCHDETECTOR_H
#define LAUNCHDETECTOR_H

#include <stdbool.h>
#include <stdint.h>

#include "LaunchMethod.h"
#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

namespace launchdetection
{

class __EXPORT LaunchDetector : public control::SuperBlock
{
public:
	LaunchDetector();
	~LaunchDetector();
	void reset();

	void update(float accel_x);
	LaunchDetectionResult getLaunchDetected();
	bool launchDetectionEnabled() { return (bool)launchdetection_on.get(); };

	float getThrottlePreTakeoff() {return throttlePreTakeoff.get(); }

	/* Returns a maximum pitch in deg. Different launch methods may impose upper pitch limits during launch */
	float getPitchMax(float pitchMaxDefault);

//	virtual bool getLaunchDetected();
protected:
private:
	int activeLaunchDetectionMethodIndex; /**< holds a index to the launchMethod in the array launchMethods
					       which detected a Launch. If no launchMethod has detected a launch yet the
					       value is -1. Once one launchMetthod has detected a launch only this
					       method is checked for further adavancing in the state machine (e.g. when
					       to power up the motors) */

	LaunchMethod *launchMethods[1];
	control::BlockParamInt launchdetection_on;
	control::BlockParamFloat throttlePreTakeoff;


};

}

#endif // LAUNCHDETECTOR_H
