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
 * @file CatapultLaunchMethod.cpp
 * Catapult Launch detection
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 *
 */

#include "CatapultLaunchMethod.h"
#include <systemlib/err.h>

namespace launchdetection
{

CatapultLaunchMethod::CatapultLaunchMethod(SuperBlock *parent) :
	SuperBlock(parent, "CAT"),
	last_timestamp(hrt_absolute_time()),
	integrator(0.0f),
	launchDetected(false),
	threshold_accel(this, "A"),
	threshold_time(this, "T")
{

}

CatapultLaunchMethod::~CatapultLaunchMethod() {

}

void CatapultLaunchMethod::update(float accel_x)
{
	float dt = (float)hrt_elapsed_time(&last_timestamp) * 1e-6f;
	last_timestamp = hrt_absolute_time();

	if (accel_x > threshold_accel.get()) {
		integrator += accel_x * dt;
//		warnx("*** integrator %.3f, threshold_accel %.3f, threshold_time %.3f, accel_x %.3f, dt %.3f",
//				(double)integrator, (double)threshold_accel.get(),  (double)threshold_time.get(), (double)accel_x, (double)dt);
		if (integrator > threshold_accel.get() * threshold_time.get()) {
			launchDetected = true;
		}

	} else {
//		warnx("integrator %.3f, threshold_accel %.3f, threshold_time %.3f, accel_x %.3f, dt %.3f",
//				(double)integrator, (double)threshold_accel.get(),  (double)threshold_time.get(), (double)accel_x, (double)dt);
		/* reset integrator */
		integrator = 0.0f;
		launchDetected = false;
	}

}

bool CatapultLaunchMethod::getLaunchDetected()
{
	return launchDetected;
}


void CatapultLaunchMethod::reset()
{
	integrator = 0.0f;
	launchDetected = false;
}

}
