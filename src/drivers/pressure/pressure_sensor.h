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
#ifndef _PRESSURE_SENSOR_H_
#define _PRESSURE_SENSOR_H_

/**
 * @file
 * The functions in this file provide device specific values of all types, including addresses,
 * bit masks, and data lengths.
 */

/**
 * @section Pressure Sensor Device API
 * The functions which follow are called by the device independent code to obtain device
 * specific data.
 */

#define MAX_INTERVAL_BETWEEN_SAMPLES_IN_USECS 20000
#define MAX_LEN_TRANSMIT_BUFFER_IN_BYTES 128

/**
 * Data types for callback functions used by the device specific code to read/write
 * register values from/to the sensor.
 * @param fildes
 * @param address
 * @param out_buffer
 * @param length
 * @return
 */
typedef int (*read_reg_func_t)(int fildes, uint8_t address, uint8_t *out_buffer, int length);
typedef int (*write_reg_func_t)(int fildes, uint8_t address, uint8_t *in_buffer, int length);

/**
 * Initiate communication with the pressure sensor on the I2C bus identified by the
 * specified device path.  A device path of "/dev/i2c-1" would begin communication with
 * pressure on I2C bus number 1.  Use the specified callback to read/write registers to
 * the I2C bus.
 * @param i2c_device_path
 * POSIX device path identifying the I2C bus to be used to communciate with the sensor
 * device.
 * @param read_reg_func_ptr
 * Address of a callback function to be used to read the specified data to the
 * specified registers.
 * @param write_reg_func_ptr
 * Address of a callback function to be used to write the specified data to the
 * specified registers.
 * @param handle
 * Out variable containing the handle, later used with other functions which must reference
 * this port.
 * @return
 * 0 - indicates that the pressure sensor at the specified I2C bus was successfully opened.
 * -1 - indicates that an error occurred.
 */
int pressure_sensor_open(const char *i2c_device_path, read_reg_func_t read_reg_func_ptr,
		write_reg_func_t write_reg_func_ptr, uint32_t* handle);

/**
 * Close the sensor previously opened with pressure_sensor_open().
 * @param handle
 * Handle to the pressure sensor resource previously opened with the pressure_sensor_open() function.
 */
void pressure_sensor_close(uint32_t handle);

/**
 * Read the pressure data from the sensor previously open opened with the pressure_sensor_open
 * function. The pressure sensor values are retained in a local buffer.  Call the
 * pressure_sensor_get_pressure_in_pa() and pressure_sensor_get_temperature_in_c() functions
 * to retrieve the pressure and temperature values read by this function.
 * @param handle
 * Handle to the pressure sensor resource previously opened with the pressure_sensor_open function.
 * @return
 * 0 - indicates that the pressure sensor at the specified I2C bus was successfully read.
 * -1 - indicates that an error occurred.
 */
int pressure_sensor_read_data(uint32_t handle);

/**
 * Retrieve the pressure value in Pascals from the data read during a previous call
 * to the pressure_sensor_read_data() function.
 * @param handle
 * Handle to the pressure sensor resource previously opened with the pressure_sensor_open function.*
 * @return
 * pressure detected by the sensor in Pascals
 */
uint32_t pressure_sensor_get_pressure_in_pa(uint32_t handle);

/**
 * Retrieve the temperature value in C from a previous call to the pressure_sensor_read_data()
 * function.
 * @param handle
 * Handle to the pressure sensor resource previously opened with the pressure_sensor_open function.
 * @return
 * temperature indicated at the time the pressure was read from the sensor
 */
float pressure_sensor_get_temperature_in_c(uint32_t handle);

#endif // _PRESSURE_SENSOR_H_
