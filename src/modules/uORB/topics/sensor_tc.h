/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
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
 * @file sensor_tc.h
 *
 * topic which holds the temperature compensated values of an sensor.
 * @author Hessel van der Molen <hmolen.science@gmail.com>
 */

#ifndef TOPIC_SENSOR_TC_H
#define TOPIC_SENSOR_TC_H

#include <stdint.h>
#include "../uORB.h"

/**
 * Temperature compensated sensor readings in SI-unit form.
 */

struct sensor_tc_s {
    uint64_t timestamp;             /**< Timestamp in microseconds since boot, from gyro         */

    float gyro_rad_s[3];			/**< Angular velocity in radian per seconds       */
    float gyro_temp;                /**< Temperature of gyro 0 */
    float accelerometer_m_s2[3];    /**< Acceleration in NED body frame, in m/s^2     */
    float accelerometer_temp;		/**< Temperature of accel 0 */
    float magnetometer_ga[3];		/**< Magnetic field in NED body frame, in Gauss   */
    float magnetometer_temp;		/**< Temperature of mag 0 */
};

/* register this as object request broker structure */
ORB_DECLARE(sensor_tc);

#endif