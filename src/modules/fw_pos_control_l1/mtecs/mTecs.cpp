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
#include <stdio.h>

namespace fwPosctrl {

mTecs::mTecs() :
	SuperBlock(NULL, "MT"),
	/* Parameters */
	_mTecsEnabled(this, "ENABLED"),
	_airspeedMin(this, "FW_AIRSPD_MIN", false),
	/* Publications */
	_status(ORB_ID(tecs_status), &getPublications()),
	/* control blocks */
	_controlTotalEnergy(this, "THR"),
	_controlEnergyDistribution(this, "PIT", true),
	_controlAltitude(this, "FPA", true),
	_controlAirSpeed(this, "ACC"),
	_flightPathAngleLowpass(this, "FPA_LP"),
	_altitudeLowpass(this, "ALT_LP"),
	_airspeedLowpass(this, "A_LP"),
	_airspeedDerivative(this, "AD"),
	_throttleSp(0.0f),
	_pitchSp(0.0f),
	_BlockOutputLimiterTakeoffThrottle(this, "TKF_THR"),
	_BlockOutputLimiterTakeoffPitch(this, "TKF_PIT", true),
	_BlockOutputLimiterUnderspeedThrottle(this, "USP_THR"),
	_BlockOutputLimiterUnderspeedPitch(this, "USP_PIT", true),
	_BlockOutputLimiterLandThrottle(this, "LND_THR"),
	_BlockOutputLimiterLandPitch(this, "LND_PIT", true),
	timestampLastIteration(hrt_absolute_time()),
	_firstIterationAfterReset(true),
	_dtCalculated(false),
	_counter(0),
	_debug(false)
{
}

mTecs::~mTecs()
{
}

int mTecs::updateAltitudeSpeed(float flightPathAngle, float altitude, float altitudeSp, float airspeed,
		float airspeedSp, tecs_mode mode, LimitOverride limitOverride)
{
	/* check if all input arguments are numbers and abort if not so */
	if (!isfinite(flightPathAngle) || !isfinite(altitude) ||
			!isfinite(altitudeSp) || !isfinite(airspeed) || !isfinite(airspeedSp) || !isfinite(mode)) {
		return -1;
	}

	/* time measurement */
	updateTimeMeasurement();

	/* Filter altitude */
	float altitudeFiltered = _altitudeLowpass.update(altitude);


	/* calculate flight path angle setpoint from altitude setpoint */
	float flightPathAngleSp = _controlAltitude.update(altitudeSp - altitudeFiltered);

	/* Debug output */
	if (_counter % 10 == 0) {
		debug("***");
		debug("updateAltitudeSpeed: altitudeSp %.4f, altitude %.4f, altitude filtered %.4f, flightPathAngleSp %.4f", (double)altitudeSp, (double)altitude, (double)altitudeFiltered, (double)flightPathAngleSp);
	}

	/* Write part of the status message */
	_status.altitudeSp = altitudeSp;
	_status.altitude_filtered = altitudeFiltered;


	/* use flightpath angle setpoint for total energy control */
	return updateFlightPathAngleSpeed(flightPathAngle, flightPathAngleSp, airspeed,
			airspeedSp, mode, limitOverride);
}

int mTecs::updateFlightPathAngleSpeed(float flightPathAngle, float flightPathAngleSp, float airspeed,
		float airspeedSp, tecs_mode mode, LimitOverride limitOverride)
{
	/* check if all input arguments are numbers and abort if not so */
	if (!isfinite(flightPathAngle) || !isfinite(flightPathAngleSp) ||
			!isfinite(airspeed) || !isfinite(airspeedSp) || !isfinite(mode)) {
		return -1;
	}

	/* time measurement */
	updateTimeMeasurement();

	/* Filter airspeed */
	float airspeedFiltered = _airspeedLowpass.update(airspeed);

	/* calculate longitudinal acceleration setpoint from airspeed setpoint*/
	float accelerationLongitudinalSp = _controlAirSpeed.update(airspeedSp - airspeedFiltered);

	/* Debug output */
	if (_counter % 10 == 0) {
		debug("updateFlightPathAngleSpeed airspeedSp %.4f, airspeed %.4f airspeedFiltered %.4f,"
				"accelerationLongitudinalSp%.4f",
				(double)airspeedSp, (double)airspeed,
				(double)airspeedFiltered, (double)accelerationLongitudinalSp);
	}

	/* Write part of the status message */
	_status.airspeedSp = airspeedSp;
	_status.airspeed_filtered = airspeedFiltered;


	/* use longitudinal acceleration setpoint for total energy control */
	return updateFlightPathAngleAcceleration(flightPathAngle, flightPathAngleSp, airspeedFiltered,
			accelerationLongitudinalSp, mode, limitOverride);
}

int mTecs::updateFlightPathAngleAcceleration(float flightPathAngle, float flightPathAngleSp, float airspeedFiltered,
		float accelerationLongitudinalSp, tecs_mode mode, LimitOverride limitOverride)
{
	/* check if all input arguments are numbers and abort if not so */
	if (!isfinite(flightPathAngle) || !isfinite(flightPathAngleSp) ||
			!isfinite(airspeedFiltered) || !isfinite(accelerationLongitudinalSp) || !isfinite(mode)) {
		return -1;
	}
	/* time measurement */
	updateTimeMeasurement();

	/* update parameters first */
	updateParams();

	/* Filter flightpathangle */
	float flightPathAngleFiltered = _flightPathAngleLowpass.update(flightPathAngle);

	/* calculate values (energies) */
	float flightPathAngleError = flightPathAngleSp - flightPathAngleFiltered;
	float airspeedDerivative = 0.0f;
	if(_airspeedDerivative.getDt() > 0.0f) {
		airspeedDerivative = _airspeedDerivative.update(airspeedFiltered);
	}
	float airspeedDerivativeNorm = airspeedDerivative / CONSTANTS_ONE_G;
	float airspeedDerivativeSp = accelerationLongitudinalSp;
	float airspeedDerivativeNormSp = airspeedDerivativeSp / CONSTANTS_ONE_G;
	float airspeedDerivativeNormError = airspeedDerivativeNormSp - airspeedDerivativeNorm;

	float totalEnergyRate = flightPathAngleFiltered + airspeedDerivativeNorm;
	float totalEnergyRateError = flightPathAngleError + airspeedDerivativeNormError;
	float totalEnergyRateSp = flightPathAngleSp + airspeedDerivativeNormSp;
	float totalEnergyRateError2 = totalEnergyRateSp - totalEnergyRate;

	float energyDistributionRate = flightPathAngleFiltered - airspeedDerivativeNorm;
	float energyDistributionRateError = flightPathAngleError - airspeedDerivativeNormError;
	float energyDistributionRateSp = flightPathAngleSp - airspeedDerivativeNormSp;
	float energyDistributionRateError2 = energyDistributionRateSp - energyDistributionRate;

	/* Debug output */
	if (_counter % 10 == 0) {
		debug("totalEnergyRateSp %.2f, totalEnergyRate %.2f, totalEnergyRateError %.2f totalEnergyRateError2 %.2f airspeedDerivativeNorm %.4f",
				(double)totalEnergyRateSp, (double)totalEnergyRate, (double)totalEnergyRateError, (double)totalEnergyRateError2, (double)airspeedDerivativeNorm);
		debug("energyDistributionRateSp %.2f, energyDistributionRate %.2f, energyDistributionRateError %.2f energyDistributionRateError2 %.2f",
				(double)energyDistributionRateSp, (double)energyDistributionRate, (double)energyDistributionRateError, (double)energyDistributionRateError2);
	}

	/* Check airspeed: if below safe value switch to underspeed mode (if not in land or takeoff mode) */
	if (mode != TECS_MODE_LAND && mode != TECS_MODE_TAKEOFF && airspeedFiltered < _airspeedMin.get()) {
		mode = TECS_MODE_UNDERSPEED;
	}

	/* Set special ouput limiters if we are not in TECS_MODE_NORMAL */
	BlockOutputLimiter *outputLimiterThrottle = &_controlTotalEnergy.getOutputLimiter();
	BlockOutputLimiter *outputLimiterPitch = &_controlEnergyDistribution.getOutputLimiter();
	if (mode == TECS_MODE_TAKEOFF) {
		outputLimiterThrottle = &_BlockOutputLimiterTakeoffThrottle;
		outputLimiterPitch = &_BlockOutputLimiterTakeoffPitch;
	} else if (mode == TECS_MODE_LAND) {
		// only limit pitch but do not limit throttle
		outputLimiterPitch = &_BlockOutputLimiterLandPitch;
	} else if (mode == TECS_MODE_LAND_THROTTLELIM) {
		outputLimiterThrottle = &_BlockOutputLimiterLandThrottle;
		outputLimiterPitch = &_BlockOutputLimiterLandPitch;
	} else if (mode == TECS_MODE_UNDERSPEED) {
		outputLimiterThrottle = &_BlockOutputLimiterUnderspeedThrottle;
		outputLimiterPitch = &_BlockOutputLimiterUnderspeedPitch;
	}

	/* Apply overrride given by the limitOverride argument (this is used for limits which are not given by
	 * parameters such as pitch limits with takeoff waypoints or throttle limits when the launchdetector
	 * is running) */
	limitOverride.applyOverride(*outputLimiterThrottle, *outputLimiterPitch);

	/* Write part of the status message */
	_status.flightPathAngleSp = flightPathAngleSp;
	_status.flightPathAngle = flightPathAngle;
	_status.flightPathAngleFiltered = flightPathAngleFiltered;
	_status.airspeedDerivativeSp = airspeedDerivativeSp;
	_status.airspeedDerivative = airspeedDerivative;
	_status.totalEnergyRateSp = totalEnergyRateSp;
	_status.totalEnergyRate = totalEnergyRate;
	_status.energyDistributionRateSp = energyDistributionRateSp;
	_status.energyDistributionRate = energyDistributionRate;
	_status.mode = mode;

	/** update control blocks **/
	/* update total energy rate control block */
	_throttleSp = _controlTotalEnergy.update(totalEnergyRateSp, totalEnergyRateError, outputLimiterThrottle);

	/* update energy distribution rate control block */
	_pitchSp = _controlEnergyDistribution.update(energyDistributionRateSp, energyDistributionRateError, outputLimiterPitch);


	if (_counter % 10 == 0) {
		debug("_throttleSp %.1f, _pitchSp %.1f, flightPathAngleSp %.1f, flightPathAngle %.1f accelerationLongitudinalSp %.1f, airspeedDerivative %.1f",
				(double)_throttleSp, (double)_pitchSp,
				(double)flightPathAngleSp, (double)flightPathAngle,
				(double)accelerationLongitudinalSp, (double)airspeedDerivative);
	}

	/* publish status messge */
	_status.update();

	/* clean up */
	_firstIterationAfterReset = false;
	_dtCalculated = false;

	_counter++;

	return 0;
}

void mTecs::resetIntegrators()
{
	_controlTotalEnergy.getIntegral().setY(0.0f);
	_controlEnergyDistribution.getIntegral().setY(0.0f);
	timestampLastIteration = hrt_absolute_time();
	_firstIterationAfterReset = true;
}

void mTecs::resetDerivatives(float airspeed)
{
	_airspeedDerivative.setU(airspeed);
}


void mTecs::updateTimeMeasurement()
{
	if (!_dtCalculated) {
		float deltaTSeconds = 0.0f;
		if (!_firstIterationAfterReset) {
			hrt_abstime timestampNow = hrt_absolute_time();
			deltaTSeconds = (float)(timestampNow - timestampLastIteration) * 1e-6f;
			timestampLastIteration = timestampNow;
		}
		setDt(deltaTSeconds);

		_dtCalculated = true;
	}
}

void mTecs::debug_print(const char *fmt, va_list args)
{
	fprintf(stderr, "%s: ", "[mtecs]");
	vfprintf(stderr, fmt, args);

	fprintf(stderr, "\n");
}

void mTecs::debug(const char *fmt, ...) {

	if (!_debug) {
		return;
	}

	va_list args;

	va_start(args, fmt);
	debug_print(fmt, args);
}

} /* namespace fwPosctrl */
