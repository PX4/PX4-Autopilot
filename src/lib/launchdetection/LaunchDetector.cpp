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

#include "CatapultLaunchMethod.h"
#include "LaunchDetector.h"

namespace launchdetection
{

LaunchDetector::LaunchDetector() :
	SuperBlock(nullptr, "LAUN"),
	launchdetection_on(this, "ALL_ON")
{
	/* init all detectors */
	launchMethods[0] = new CatapultLaunchMethod(this);

	/* update all parameters of all detectors */
	updateParams();
}

LaunchDetector::~LaunchDetector()
{
	delete launchMethods[0];
}

void LaunchDetector::reset()
{
	/* Reset all detectors */
	for (const auto launchMethod : launchMethods) {
		launchMethod->reset();
	}

	/* Reset active launchdetector */
	activeLaunchDetectionMethodIndex = -1;
}

void LaunchDetector::update(float accel_x)
{
	if (launchDetectionEnabled()) {
		for (const auto launchMethod : launchMethods) {
			launchMethod->update(accel_x);
		}
	}
}

LaunchDetectionResult LaunchDetector::getLaunchDetected()
{
	if (launchDetectionEnabled()) {
		if (activeLaunchDetectionMethodIndex < 0) {
			/* None of the active launchmethods has detected a launch, check all launchmethods */
			for (unsigned i = 0; i < (sizeof(launchMethods) / sizeof(launchMethods[0])); i++) {
				if (launchMethods[i]->getLaunchDetected() != LAUNCHDETECTION_RES_NONE) {
					PX4_WARN("selecting launchmethod %d", i);
					activeLaunchDetectionMethodIndex = i; // from now on only check this method
					return launchMethods[i]->getLaunchDetected();
				}
			}

		} else {
			return launchMethods[activeLaunchDetectionMethodIndex]->getLaunchDetected();
		}
	}

	return LAUNCHDETECTION_RES_NONE;
}

float LaunchDetector::getPitchMax(float pitchMaxDefault)
{
	if (!launchDetectionEnabled()) {
		return pitchMaxDefault;
	}

	/* if a lauchdetectionmethod is active or only one exists return the pitch limit from this method,
	 * otherwise use the default limit */
	if (activeLaunchDetectionMethodIndex < 0) {
		if (sizeof(launchMethods) / sizeof(LaunchMethod *) > 1) {
			return pitchMaxDefault;

		} else {
			return launchMethods[0]->getPitchMax(pitchMaxDefault);
		}

	} else {
		return launchMethods[activeLaunchDetectionMethodIndex]->getPitchMax(pitchMaxDefault);
	}
}

} // namespace launchdetection
