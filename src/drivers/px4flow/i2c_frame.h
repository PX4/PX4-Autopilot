/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file i2c_frame.h
 * Definition of i2c frames.
 * There are different I2C readouts available: i2c_frame and i2c_integral_frame.
 * The ic2_frame can be requested via the register 0x0, the i2c_integral_frame via 0x16.
 *
 * @author Thomas Boehm <thomas.boehm@fortiss.org>
 * @author James Goppert <james.goppert@gmail.com>
 */

#pragma once

#include <inttypes.h>

typedef  struct i2c_frame {
	uint16_t frame_count;       /**< counts created I2C frames [#frames] */
	int16_t pixel_flow_x_sum;   /**< latest x flow measurement in pixels*10 [pixels] */
	int16_t pixel_flow_y_sum;   /**< latest y flow measurement in pixels*10 [pixels] */
	int16_t flow_comp_m_x;      /**< x velocity*1000 [meters/sec] */
	int16_t flow_comp_m_y;      /**< y velocity*1000 [meters/sec] */
	int16_t qual;               /**< Optical flow quality / confidence [0: bad, 255: maximum quality] */
	int16_t gyro_x_rate;        /**< latest gyro x rate [rad/sec] */
	int16_t gyro_y_rate;        /**< latest gyro y rate [rad/sec] */
	int16_t gyro_z_rate;        /**< latest gyro z rate [rad/sec] */
	uint8_t gyro_range;         /**< gyro range [0 .. 7] equals [50 deg/sec .. 2000 deg/sec] */
	uint8_t sonar_timestamp;    /**< time since last sonar update [milliseconds] */
	int16_t ground_distance;    /**< Ground distance in meters*1000 [meters]. Positive value: distance known. Negative value: Unknown distance */
} i2c_frame;

#define I2C_FRAME_SIZE (sizeof(i2c_frame))


typedef struct i2c_integral_frame {
	uint16_t frame_count_since_last_readout; /**< number of flow measurements since last I2C readout [#frames] */
	int16_t pixel_flow_x_integral;           /**< accumulated flow in radians*10000 around x axis since last I2C readout [rad*10000] */
	int16_t pixel_flow_y_integral;           /**< accumulated flow in radians*10000 around y axis since last I2C readout [rad*10000] */
	int16_t gyro_x_rate_integral;            /**< accumulated gyro x rates in radians*10000 since last I2C readout [rad*10000] */
	int16_t gyro_y_rate_integral;            /**< accumulated gyro y rates in radians*10000 since last I2C readout [rad*10000] */
	int16_t gyro_z_rate_integral;            /**< accumulated gyro z rates in radians*10000 since last I2C readout [rad*10000] */
	uint32_t integration_timespan;           /**< accumulation timespan in microseconds since last I2C readout [microseconds] */
	uint32_t sonar_timestamp;                /**< time since last sonar update [microseconds] */
	uint16_t ground_distance;                /**< Ground distance in meters*1000 [meters*1000] */
	int16_t gyro_temperature;                /**< Temperature * 100 in centi-degrees Celsius [degcelsius*100] */
	uint8_t qual;                            /**< averaged quality of accumulated flow values [0:bad quality;255: max quality] */
} i2c_integral_frame;

#define I2C_INTEGRAL_FRAME_SIZE (sizeof(i2c_integral_frame))
