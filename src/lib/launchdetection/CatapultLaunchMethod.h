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
 * @file CatapultLaunchMethod.h
 * Catpult Launch detection
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#ifndef CATAPULTLAUNCHMETHOD_H_
#define CATAPULTLAUNCHMETHOD_H_

#include "LaunchMethod.h"

#include <drivers/drv_hrt.h>
#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

namespace launchdetection
{

class CatapultLaunchMethod : public LaunchMethod, public control::SuperBlock
{
public:
	CatapultLaunchMethod(SuperBlock *parent);
	~CatapultLaunchMethod();

	void update(float accel_x);
	LaunchDetectionResult getLaunchDetected() const;
	void reset();
	float getPitchMax(float pitchMaxDefault);

private:
	hrt_abstime last_timestamp;
	float integrator;
	float motorDelayCounter;

	LaunchDetectionResult state;

	control::BlockParamFloat thresholdAccel;
	control::BlockParamFloat thresholdTime;
	control::BlockParamFloat motorDelay;
	control::BlockParamFloat pitchMaxPreThrottle; /**< Upper pitch limit before throttle is turned on.
						       Can be used to make sure that the AC does not climb
						       too much while attached to a bungee */

};

#endif /* CATAPULTLAUNCHMETHOD_H_ */

}
