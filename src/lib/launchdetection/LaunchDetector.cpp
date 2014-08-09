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
 * @file launchDetection.cpp
 * Auto Detection for different launch methods (e.g. catapult)
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#include "LaunchDetector.h"
#include "CatapultLaunchMethod.h"
#include <systemlib/err.h>

namespace launchdetection
{

LaunchDetector::LaunchDetector() :
	SuperBlock(NULL, "LAUN"),
	launchdetection_on(this, "ALL_ON"),
	throttlePreTakeoff(this, "THR_PRE")
{
	/* init all detectors */
	launchMethods[0] = new CatapultLaunchMethod(this);


	/* update all parameters of all detectors */
	updateParams();
}

LaunchDetector::~LaunchDetector()
{

}

void LaunchDetector::reset()
{
	/* Reset all detectors */
	launchMethods[0]->reset();
}

void LaunchDetector::update(float accel_x)
{
	if (launchdetection_on.get() == 1) {
		for (uint8_t i = 0; i < sizeof(launchMethods)/sizeof(LaunchMethod); i++) {
			launchMethods[i]->update(accel_x);
		}
	}
}

bool LaunchDetector::getLaunchDetected()
{
	if (launchdetection_on.get() == 1) {
		for (uint8_t i = 0; i < sizeof(launchMethods)/sizeof(LaunchMethod); i++) {
			if(launchMethods[i]->getLaunchDetected()) {
				return true;
			}
		}
	}

	return false;
}

}
