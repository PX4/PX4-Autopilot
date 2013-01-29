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

#include "math.h"
#include "conversions.h"
#include "airspeed.h"


float calc_indicated_airspeed(float pressure_front, float pressure_ambient, float temperature)
{
	return sqrtf((2.0f*(pressure_front - pressure_ambient)) / CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C);
}
 
/**
 * Calculate true airspeed from indicated airspeed.
 *
 * Note that the true airspeed is NOT the groundspeed, because of the effects of wind
 *
 * @param speed current indicated airspeed
 * @param pressure_ambient pressure at the side of the tube/airplane
 * @param temperature air temperature in degrees celcius
 * @return true airspeed in m/s
 */
float calc_true_airspeed_from_indicated(float speed, float pressure_ambient, float temperature)
{
	return speed * sqrtf(CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C / get_air_density(pressure_ambient, temperature));
}
 
/**
 * Directly calculate true airspeed
 *
 * Note that the true airspeed is NOT the groundspeed, because of the effects of wind
 *
 * @param pressure_front pressure inside the pitot/prandl tube
 * @param pressure_ambient pressure at the side of the tube/airplane
 * @param temperature air temperature in degrees celcius
 * @return true airspeed in m/s
 */
float calc_true_airspeed(float pressure_front, float pressure_ambient, float temperature)
{
	return sqrtf((2.0f*(pressure_front - pressure_ambient)) / get_air_density(pressure_ambient, temperature));
}
