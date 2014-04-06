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
 * @file mTecs.cpp
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#include "mTecs.h"

#include <lib/geo/geo.h>
#include <systemlib/err.h>

namespace fwPosctrl {

mTecs::mTecs() :
	SuperBlock(NULL, "MT"),
	/* Parameters */
	_mTecsEnabled(this, "ENABLED"),
	/* control blocks */
	_controlTotalEnergy(this, "THR"),
	_controlEnergyDistribution(this, "PIT", true),
	_controlAltitude(this, "FPA", true),
	_controlAirSpeed(this, "ACC"),
	_airspeedDerivative(this, "AD"),
	_throttleSp(0.0f),
	_pitchSp(0.0f),
	BlockOutputLimiterTakeoffThrottle(this, "TKF_THR"),
	BlockOutputLimiterTakeoffPitch(this, "TKF_PIT", true),
	timestampLastIteration(hrt_absolute_time()),
	_firstIterationAfterReset(true),
	dtCalculated(false),
	_counter(0)
{
}

mTecs::~mTecs()
{
}

void mTecs::updateAltitudeSpeed(float flightPathAngle, float altitude, float altitudeSp, float airspeed, float airspeedSp, tecs_mode mode)
{

	/* time measurement */
	updateTimeMeasurement();

	float flightPathAngleSp = _controlAltitude.update(altitudeSp - altitude);
	if (_counter % 10 == 0) {
		warnx("***");
		warnx("updateAltitudeSpeed: altitudeSp %.4f, altitude %.4f, flightPathAngleSp %.4f", (double)altitudeSp, (double)altitude, (double)flightPathAngleSp);
	}
	updateFlightPathAngleSpeed(flightPathAngle, flightPathAngleSp, airspeed, airspeedSp, mode);
}

void mTecs::updateFlightPathAngleSpeed(float flightPathAngle, float flightPathAngleSp, float airspeed, float airspeedSp, tecs_mode mode) {

	/* time measurement */
	updateTimeMeasurement();

	float accelerationLongitudinalSp = _controlAirSpeed.update(airspeedSp - airspeed);
	if (_counter % 10 == 0) {
		warnx("updateFlightPathAngleSpeed airspeedSp %.4f, airspeed %.4f, accelerationLongitudinalSp%.4f", (double)airspeedSp, (double)airspeed, (double)accelerationLongitudinalSp);
	}
	updateFlightPathAngleAcceleration(flightPathAngle, flightPathAngleSp, airspeed, accelerationLongitudinalSp, mode);
}

void mTecs::updateFlightPathAngleAcceleration(float flightPathAngle, float flightPathAngleSp, float airspeed, float accelerationLongitudinalSp, tecs_mode mode)
{
	/* time measurement */
	updateTimeMeasurement();

	/* update parameters first */
	updateParams();

	/* calculate values (energies) */
	float flightPathAngleError = flightPathAngleSp - flightPathAngle;
	float airspeedDerivative = 0.0f;
	if(_airspeedDerivative.getDt() > 0.0f) {
		airspeedDerivative = _airspeedDerivative.update(airspeed);
	}
	float airspeedDerivativeNorm = airspeedDerivative / CONSTANTS_ONE_G;
	float airspeedSpDerivative = accelerationLongitudinalSp;
	float airspeedSpDerivativeNorm = airspeedSpDerivative / CONSTANTS_ONE_G;
	float airspeedSpDerivativeNormError = airspeedSpDerivativeNorm - airspeedDerivativeNorm;

	float totalEnergyRate = flightPathAngle + airspeedDerivativeNorm;
	float totalEnergyRateError = flightPathAngleError + airspeedSpDerivativeNormError;
	float totalEnergyRateSp = flightPathAngleSp + airspeedSpDerivativeNorm;
	float totalEnergyRateError2 = totalEnergyRateSp - totalEnergyRate;

	float energyDistributionRate = flightPathAngle - airspeedDerivativeNorm;
	float energyDistributionRateError = flightPathAngleError - airspeedSpDerivativeNormError;
	float energyDistributionRateSp = flightPathAngleSp - airspeedSpDerivativeNorm;
	float energyDistributionRateError2 = energyDistributionRateSp - energyDistributionRate;

	if (_counter % 10 == 0) {
		warnx("totalEnergyRateSp %.2f, totalEnergyRate %.2f, totalEnergyRateError %.2f totalEnergyRateError2 %.2f airspeedDerivativeNorm %.4f",
				(double)totalEnergyRateSp, (double)totalEnergyRate, (double)totalEnergyRateError, (double)totalEnergyRateError2, (double)airspeedDerivativeNorm);
		warnx("energyDistributionRateSp %.2f, energyDistributionRate %.2f, energyDistributionRateError %.2f energyDistributionRateError2 %.2f",
				(double)energyDistributionRateSp, (double)energyDistributionRate, (double)energyDistributionRateError, (double)energyDistributionRateError2);
	}

	/* Set special ouput limiters if we are not in TECS_MODE_NORMAL */
	BlockOutputLimiter *outputLimiterThrottle = NULL; // NULL --> use standard inflight limits
	BlockOutputLimiter *outputLimiterPitch = NULL; // NULL --> use standard inflight limits
	if (mode == TECS_MODE_TAKEOFF) {
		outputLimiterThrottle = &BlockOutputLimiterTakeoffThrottle; //XXX: accept prelaunch values from launchdetector
		outputLimiterPitch = &BlockOutputLimiterTakeoffPitch;
	}

	/** update control blocks **/
	/* update total energy rate control block */
	_throttleSp = _controlTotalEnergy.update(totalEnergyRateSp, totalEnergyRateError, outputLimiterThrottle);

	/* update energy distribution rate control block */
	_pitchSp = _controlEnergyDistribution.update(energyDistributionRateSp, energyDistributionRateError, outputLimiterPitch);


	if (_counter % 10 == 0) {
		warnx("_throttleSp %.1f, _pitchSp %.1f, flightPathAngleSp %.1f, flightPathAngle %.1f accelerationLongitudinalSp %.1f, airspeedDerivative %.1f",
				(double)_throttleSp, (double)_pitchSp,
				(double)flightPathAngleSp, (double)flightPathAngle,
				(double)accelerationLongitudinalSp, (double)airspeedDerivative);
	}


	/* clean up */
	_firstIterationAfterReset = false;
	dtCalculated = false;

	_counter++;
}

void mTecs::resetIntegrators()
{
	_controlTotalEnergy.getIntegral().setY(0.0f);
	_controlEnergyDistribution.getIntegral().setY(0.0f);
	timestampLastIteration = hrt_absolute_time();
	_firstIterationAfterReset = true;
}

void mTecs::updateTimeMeasurement()
{
	if (!dtCalculated) {
		float deltaTSeconds = 0.0f;
		if (!_firstIterationAfterReset) {
			hrt_abstime timestampNow = hrt_absolute_time();
			deltaTSeconds = (float)(timestampNow - timestampLastIteration) * 1e-6f;
			timestampLastIteration = timestampNow;
		}
		setDt(deltaTSeconds);

		dtCalculated = true;
	}
}

} /* namespace fwPosctrl */

