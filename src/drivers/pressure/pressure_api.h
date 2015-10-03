/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
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
#ifndef PRESSURE_API_H_
#define PRESSURE_API_H_

#include <stdbool.h>

__BEGIN_DECLS

/**
 * @file
 * The functions in this file provide the top level, device independent interface
 * to retrieve sensor data.
 */

/**
 * @section Pressure Sensor API
 * The functions which follow are called by the flight stack code to obtain sensor
 * data for publication.
 */

/**
 * The sensor independent data structure containing pressure values.
 */
struct pressure_sensor_data
{
   int32_t t_fine; /*! used internally to calculate a temperature compensated pressure value. */
   uint32_t pressure_in_pa; /*! current pressure in Pascals */
   float temperature_in_c; /*! current temperature in C at which the pressure was read */
   uint32_t sensor_read_counter; /*! the total number of pressure sensor readings since the system was started */
   uint64_t last_read_time_in_usecs; /*! time stamp indicating the time at which the pressure in this data structure was read */
   uint64_t error_count; /*! the total number of errors detected when reading the pressure, since the system was started */
};

/**
 * Retrieves a detailed error code for the last pressure sensor function executed.
 * This is typically called after a general error of -1 is returned by the
 * pressure sensor function.
 * @param handle
 * Handle returned from a previous call to to pressure_open().
 * @return
 * 0 for success
 * -1 for failure
 */
int pressure_api_get_last_error(uint32_t handle);

/**
 * Sets the pressure at sea level for the location of the sensor.  This is used
 * internally to convert pressure to altitude.
 * @note
 * TODO: This function is currently not implemented.
 * @param handle
 * @param altimeter_setting_in_mbars
 * @return
 * 0 for success
 * -1 for failure
 */
int pressure_api_set_altimeter(uint32_t handle, float altimeter_setting_in_mbars);

/**
 * Opens access to the pressure sensor on the I2C bus specified by the device
 * path parameter.
 * @param i2c_device_path
 * Device path for the I2C bus used by the pressure sensor, e.g.: /dev/i2c-3
 * @return
 * 0 for success
 * -1 for failure
 */
int pressure_api_open(const char *i2c_device_path,uint32_t* handle);

/**
 * Close the connection to the bus used by this sensor, and release any
 * associated resources.
 * @param handle
 */
void pressure_api_close(uint32_t handle);

/**
 * Read the pressure sensor data from the from the previously opened sensor connection.
 * @param handle
 * Handle used identify the pressure sensor previously opened with the pressure_open() function.
 * @param out_data
 * Reference to the structure that will be filled with pressure sensor data.
 * @param is_new_data_required
 * - true: The caller will block until new sensor data is retrieved in the next sensor polling interval.
 * - false: Retrieves the last sensor value generated and cached by the driver.
 * @return
 * 0 for success
 * -1 for failure
 */
int pressure_api_get_sensor_data(uint32_t handle, struct pressure_sensor_data *out_data, bool is_new_data_required);

__END_DECLS

#endif /* PRESSURE_API_H_ */
