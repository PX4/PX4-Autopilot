/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file conversions.h
 * Definition of commonly used conversions.
 *
 * Includes bit / byte / geo representation and unit conversions.
 */

#ifndef CONVERSIONS_H_
#define CONVERSIONS_H_
#include <float.h>
#include <stdint.h>

#define CONSTANTS_ONE_G				9.80665f
#define CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C	1.225f
#define CONSTANTS_AIR_GAS_CONST 8.31432f
#define CONSTANTS_ABSOLUTE_NULL_KELVIN 273.15f

__BEGIN_DECLS

/**
 * Converts a signed 16 bit integer from big endian to little endian.
 *
 * This function is for commonly used 16 bit big endian sensor data,
 * delivered by driver routines as two 8 bit numbers in big endian order.
 * Common vendors with big endian representation are Invense, Bosch and
 * Honeywell. ST micro devices tend to use a little endian representation.
 */
__EXPORT int16_t int16_t_from_bytes(uint8_t bytes[]);

/**
 * Converts a 3 x 3 rotation matrix to an unit quaternion.
 *
 * All orientations are expressed in NED frame.
 *
 * @param R rotation matrix to convert
 * @param Q quaternion to write back to
 */
__EXPORT void rot2quat(const float R[9], float Q[4]);

/**
 * Converts an unit quaternion to a 3 x 3 rotation matrix.
 *
 * All orientations are expressed in NED frame.
 *
 * @param Q quaternion to convert
 * @param R rotation matrix to write back to
 */
__EXPORT void quat2rot(const float Q[4], float R[9]);

/**
 * Calculates air density.
 *
 * @param static_pressure ambient pressure in millibar
 * @param temperature_celcius air / ambient temperature in celcius
 */
__EXPORT float get_air_density(float static_pressure, float temperature_celsius);

__END_DECLS

#endif /* CONVERSIONS_H_ */
