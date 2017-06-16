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
 * @file airspeed.h
 * Airspeed estimation declarations
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 *
 */

#ifndef AIRSPEED_H_
#define AIRSPEED_H_

#include "math.h"
#include "conversions.h"

__BEGIN_DECLS

/**
 * Calculate indicated airspeed.
 *
 * Note that the indicated airspeed is not the true airspeed because it
 * lacks the air density compensation. Use the calc_true_airspeed functions to get
 * the true airspeed.
 *
 * @param total_pressure pressure inside the pitot/prandtl tube
 * @param static_pressure pressure at the side of the tube/airplane
 * @return indicated airspeed in m/s
 */
__EXPORT float calc_indicated_airspeed(float differential_pressure);

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
__EXPORT float calc_true_airspeed_from_indicated(float speed_indicated, float pressure_ambient,
		float temperature_celsius);

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
__EXPORT float calc_true_airspeed(float total_pressure, float static_pressure, float temperature_celsius);

/**
* Calculates air density.
*
* @param static_pressure ambient pressure in millibar
* @param temperature_celcius air / ambient temperature in celcius
*/
__EXPORT float get_air_density(float static_pressure, float temperature_celsius);

__END_DECLS

#endif
