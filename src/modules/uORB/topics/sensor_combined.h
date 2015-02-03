/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file sensor_combined.h
 * Definition of the sensor_combined uORB topic.
 *
 * @author Thomas Gubler <thomas@px4.io>
 * @author Julian Oes <julian@px4.io>
 * @author Lorenz Meier <lorenz@px4.io>
 */

#ifndef SENSOR_COMBINED_H_
#define SENSOR_COMBINED_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

enum MAGNETOMETER_MODE {
	MAGNETOMETER_MODE_NORMAL = 0,
	MAGNETOMETER_MODE_POSITIVE_BIAS,
	MAGNETOMETER_MODE_NEGATIVE_BIAS
};

/**
 * @addtogroup topics
 * @{
 */

/**
 * Sensor readings in raw and SI-unit form.
 *
 * These values are read from the sensors. Raw values are in sensor-specific units,
 * the scaled values are in SI-units, as visible from the ending of the variable
 * or the comments. The use of the SI fields is in general advised, as these fields
 * are scaled and offset-compensated where possible and do not change with board
 * revisions and sensor updates.
 *
 */
struct sensor_combined_s {

	/*
	 * Actual data, this is specific to the type of data which is stored in this struct
	 * A line containing L0GME will be added by the Python logging code generator to the
	 * logged dataset.
	 */

	/* NOTE: Ordering of fields optimized to align to 32 bit / 4 bytes Change with consideration only   */

	uint64_t timestamp;			/**< Timestamp in microseconds since boot, from gyro         */

	int16_t	gyro_raw[3];			/**< Raw sensor values of angular velocity        */
	float gyro_rad_s[3];			/**< Angular velocity in radian per seconds       */
	unsigned gyro_errcount;			/**< Error counter for gyro 0 */

	int16_t accelerometer_raw[3];		/**< Raw acceleration in NED body frame           */
	float accelerometer_m_s2[3];		/**< Acceleration in NED body frame, in m/s^2     */
	int accelerometer_mode;			/**< Accelerometer measurement mode */
	float accelerometer_range_m_s2;		/**< Accelerometer measurement range in m/s^2 */
	uint64_t accelerometer_timestamp;	/**< Accelerometer timestamp        */
	unsigned accelerometer_errcount;	/**< Error counter for accel 0 */

	int16_t	magnetometer_raw[3];		/**< Raw magnetic field in NED body frame         */
	float magnetometer_ga[3];		/**< Magnetic field in NED body frame, in Gauss   */
	int magnetometer_mode;			/**< Magnetometer measurement mode */
	float magnetometer_range_ga;		/**< ± measurement range in Gauss */
	float magnetometer_cuttoff_freq_hz;	/**< Internal analog low pass frequency of sensor */
	uint64_t magnetometer_timestamp;	/**< Magnetometer timestamp         */
	unsigned magnetometer_errcount;		/**< Error counter for mag 0 */

	int16_t	gyro1_raw[3];			/**< Raw sensor values of angular velocity        */
	float gyro1_rad_s[3];			/**< Angular velocity in radian per seconds       */
	uint64_t gyro1_timestamp;		/**< Gyro timestamp */
	unsigned gyro1_errcount;		/**< Error counter for gyro 1 */

	int16_t accelerometer1_raw[3];		/**< Raw acceleration in NED body frame           */
	float accelerometer1_m_s2[3];		/**< Acceleration in NED body frame, in m/s^2     */
	uint64_t accelerometer1_timestamp;	/**< Accelerometer timestamp        */
	unsigned accelerometer1_errcount;	/**< Error counter for accel 1 */

	int16_t	magnetometer1_raw[3];		/**< Raw magnetic field in NED body frame         */
	float magnetometer1_ga[3];		/**< Magnetic field in NED body frame, in Gauss   */
	uint64_t magnetometer1_timestamp;	/**< Magnetometer timestamp         */
	unsigned magnetometer1_errcount;		/**< Error counter for mag 0 */

	int16_t	gyro2_raw[3];			/**< Raw sensor values of angular velocity        */
	float gyro2_rad_s[3];			/**< Angular velocity in radian per seconds       */
	uint64_t gyro2_timestamp;		/**< Gyro timestamp */
	unsigned gyro2_errcount;		/**< Error counter for gyro 1 */

	int16_t accelerometer2_raw[3];		/**< Raw acceleration in NED body frame           */
	float accelerometer2_m_s2[3];		/**< Acceleration in NED body frame, in m/s^2     */
	uint64_t accelerometer2_timestamp;	/**< Accelerometer timestamp        */
	unsigned accelerometer2_errcount;	/**< Error counter for accel 2 */

	int16_t	magnetometer2_raw[3];		/**< Raw magnetic field in NED body frame         */
	float magnetometer2_ga[3];		/**< Magnetic field in NED body frame, in Gauss   */
	uint64_t magnetometer2_timestamp;	/**< Magnetometer timestamp         */
	unsigned magnetometer2_errcount;	/**< Error counter for mag 2 */

	float baro_pres_mbar;			/**< Barometric pressure, already temp. comp.     */
	float baro_alt_meter;			/**< Altitude, already temp. comp.                */
	float baro_temp_celcius;		/**< Temperature in degrees celsius               */
	uint64_t baro_timestamp;		/**< Barometer timestamp        */

	float baro1_pres_mbar;			/**< Barometric pressure, already temp. comp.     */
	float baro1_alt_meter;			/**< Altitude, already temp. comp.                */
	float baro1_temp_celcius;		/**< Temperature in degrees celsius               */
	uint64_t baro1_timestamp;		/**< Barometer timestamp        */

	float adc_voltage_v[10];		/**< ADC voltages of ADC Chan 10/11/12/13 or -1   */
	unsigned adc_mapping[10];		/**< Channel indices of each of these values */
	float mcu_temp_celcius;			/**< Internal temperature measurement of MCU */

	float differential_pressure_pa;			/**< Airspeed sensor differential pressure */
	uint64_t differential_pressure_timestamp;	/**< Last measurement timestamp */
	float differential_pressure_filtered_pa;	/**< Low pass filtered airspeed sensor differential pressure reading */

	float differential_pressure1_pa;			/**< Airspeed sensor differential pressure */
	uint64_t differential_pressure1_timestamp;	/**< Last measurement timestamp */
	float differential_pressure1_filtered_pa;	/**< Low pass filtered airspeed sensor differential pressure reading */

};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(sensor_combined);

#endif
