/****************************************************************************
 * pid.c
 *
 *   Copyright (C) 2012 Ivan Ovinnikov. All rights reserved.
 *   Authors: Ivan Ovinnikov <oivan@ethz.ch>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "pid.h"
#include "fixedwing_control.h"

/*******************************************************************************
 * pid()
 *
 * Calculates the PID control output given an error
 *
 * Input: float error, uint16_t dt, float scaler, float K_p, float K_i, float K_d
 *
 * Output: PID control value
 *
 ******************************************************************************/

static float pid(float error, float error_deriv, uint16_t dt, float scaler, float K_p, float K_i, float K_d, float intmax)
{
	// PID parameters

	float Kp = K_p;
	float Ki = K_i;
	float Kd = K_d;
	float delta_time = dt;	// delta time
	float lerror;		// last error value
	float imax = intmax;			// max integral value
	float integrator;
	float derivative;
	float lderiv;
	int fCut = 20;		// anything above 20 Hz is considered noise - low pass filter for the derivative
	float output = 0;		// the output of the PID controller

	output += error * Kp;

	if ((fabs(Kd) > 0) && (dt > 0)) {

		if (PID_DERIVMODE_CALC) {
			derivative = (error - lerror) / delta_time;

			// discrete low pass filter, cuts out the
			// high frequency noise that can drive the controller crazy
			float RC = 1 / (2 * M_PI * fCut);
			derivative = lderiv +
				     (delta_time / (RC + delta_time)) * (derivative - lderiv);

			// update state
			lerror 	= error;
			lderiv  = derivative;

		} else {
			derivative = error_deriv;
		}

		// add in derivative component
		output 	+= Kd * derivative;
	}

	printf("PID derivative %i\n", (int)(1000 * derivative));

	// scale the P and D components
	output *= scaler;

	// Compute integral component if time has elapsed
	if ((fabs(Ki) > 0) && (dt > 0)) {
		integrator 		+= (error * Ki) * scaler * delta_time;

		if (integrator < -imax) {
			integrator = -imax;

		} else if (integrator > imax) {
			integrator = imax;
		}

		output += integrator;
	}

	printf("PID Integrator %i\n", (int)(1000 * integrator));

	return output;
}

