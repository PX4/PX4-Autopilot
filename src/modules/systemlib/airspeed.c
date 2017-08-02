/****************************************************************************
 *
 *   Copyright (C) 2012-2013 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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
 * @file airspeed.c
 * Airspeed estimation
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 *
 */

#include <stdio.h>
#include <math.h>
#include <geo/geo.h>
#include "airspeed.h"

/**
 * Calculate indicated airspeed.
 *
 * Note that the indicated airspeed is not the true airspeed because it
 * lacks the air density compensation. Use the calc_true_airspeed functions to get
 * the true airspeed.
 *
 * @param differential_pressure total_ pressure - static pressure
 * @return indicated airspeed in m/s
 */
float calc_indicated_airspeed_corrected(enum AIRSPEED_PITOT_MODEL pmodel, enum AIRSPEED_SENSOR_MODEL smodel,
					float tube_len, float differential_pressure, float pressure_ambient, float temperature_celsius)
{

	// air density in kg/m3
	double rho_air = get_air_density(pressure_ambient, temperature_celsius);

	double dp = fabsf(differential_pressure);
	// additional dp through pitot tube
	float dp_pitot;
	float dp_tube;
	float dv;

	switch (smodel) {

	case AIRSPEED_SENSOR_MODEL_MEMBRANE: {
			dp_pitot = 0.0f;
			dp_tube = 0.0f;
			dv = 0.0f;
		}
		break;

	case AIRSPEED_SENSOR_MODEL_SDP3X: {
			// flow through sensor
			double flow_SDP33 = (300.805 - 300.878 / (0.00344205 * pow(dp, 0.68698) + 1)) * 1.29 / rho_air;

			// for too small readings the compensation might result in a negative flow which causes numerical issues
			if (flow_SDP33 < 0.0) {
				flow_SDP33 = 0.0;
			}

			switch (pmodel) {
			case AIRSPEED_PITOT_MODEL_HB:
				dp_pitot = 28557670.0 - 28557670.0 / (1 + pow((flow_SDP33 / 5027611.0), 1.227924));
				break;

			default:
				dp_pitot = 0.0f;
				break;
			}

			// pressure drop through tube
			dp_tube = flow_SDP33 * 0.000746124 * (double)tube_len * rho_air;

			// speed at pitot-tube tip due to flow through sensor
			dv = 0.0331582 * flow_SDP33;
		}
		break;

	default: {
			dp_pitot = 0.0f;
			dp_tube = 0.0f;
			dv = 0.0f;
		}
		break;
	}

	// if (!PX4_ISFINITE(dp_tube)) {
	// 	dp_tube = 0.0f;
	// }

	// if (!PX4_ISFINITE(dp_pitot)) {
	// 	dp_pitot = 0.0f;
	// }

	// if (!PX4_ISFINITE(dv)) {
	// 	dv = 0.0f;
	// }

	// sum of all pressure drops
	float dp_tot = (float)dp + dp_tube + dp_pitot;

	// computed airspeed without correction for inflow-speed at tip of pitot-tube
	float airspeed_uncorrected = sqrtf(2 * dp_tot / CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C);

	// corrected airspeed
	float airspeed_corrected = airspeed_uncorrected + dv;

	// return result with correct sign
	return (differential_pressure > 0.0f) ? airspeed_corrected : -airspeed_corrected;
}


/**
 * Calculate indicated airspeed.
 *
 * Note that the indicated airspeed is not the true airspeed because it
 * lacks the air density compensation. Use the calc_true_airspeed functions to get
 * the true airspeed.
 *
 * @param differential_pressure total_ pressure - static pressure
 * @return indicated airspeed in m/s
 */
float calc_indicated_airspeed(float differential_pressure)
{


	if (differential_pressure > 0.0f) {
		return sqrtf((2.0f * differential_pressure) / CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C);

	} else {
		return -sqrtf((2.0f * fabsf(differential_pressure)) / CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C);
	}

}

/**
 * Calculate true airspeed from indicated airspeed.
 *
 * Note that the true airspeed is NOT the groundspeed, because of the effects of wind
 *
 * @param speed_indicated current indicated airspeed
 * @param pressure_ambient pressure at the side of the tube/airplane
 * @param temperature_celsius air temperature in degrees celcius
 * @return true airspeed in m/s
 */
float calc_true_airspeed_from_indicated(float speed_indicated, float pressure_ambient, float temperature_celsius)
{
	return speed_indicated * sqrtf(CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C / get_air_density(pressure_ambient,
				       temperature_celsius));
}

/**
 * Directly calculate true airspeed
 *
 * Note that the true airspeed is NOT the groundspeed, because of the effects of wind
 *
 * @param total_pressure pressure inside the pitot/prandtl tube
 * @param static_pressure pressure at the side of the tube/airplane
 * @param temperature_celsius air temperature in degrees celcius
 * @return true airspeed in m/s
 */
float calc_true_airspeed(float total_pressure, float static_pressure, float temperature_celsius)
{
	float density = get_air_density(static_pressure, temperature_celsius);

	if (density < 0.0001f || !isfinite(density)) {
		density = CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C;
	}

	float pressure_difference = total_pressure - static_pressure;

	if (pressure_difference > 0) {
		return sqrtf((2.0f * (pressure_difference)) / density);

	} else {
		return -sqrtf((2.0f * fabsf(pressure_difference)) / density);
	}
}

float get_air_density(float static_pressure, float temperature_celsius)
{
	return static_pressure / (CONSTANTS_AIR_GAS_CONST * (temperature_celsius - CONSTANTS_ABSOLUTE_NULL_CELSIUS));
}
