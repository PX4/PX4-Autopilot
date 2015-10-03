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

#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>

#include <px4_log.h>

#include "pressure.h"
#include "pressure_api.h"
#include "test_util.h"

#define PRESSURE_DEVICE_PATH "/dev/i2c-2"

/**
 * @brief Prints the bmp280_sensor_data struct to the log
 *
 * @param   sensor_data[in]   pointer to bmp280_sensor_data struct to print to log
 *
 */
void print_pressure_values(struct pressure_sensor_data *sensor_data) {
	PX4_INFO(
			"bmp280 data [cntr: %d, time stamp: %lld, pressure (pascals): %d, temp (C): %f]",
			sensor_data->sensor_read_counter, sensor_data->last_read_time_in_usecs, sensor_data->pressure_in_pa, sensor_data->temperature_in_c);
}

/**
 * @brief Open and read the bmp280 device.  Dumps data to the log file.
 *
 * @par
 * Test: 
 * 1) Attempt to open bmp280 device ('/dev/i2c-2')
 * 2) Read from the bmp280 device a couple of times 
 * 3) Close the device
 *
 * @return
 * TEST_PASS ------ Was able to open and read device data
 * TEST_FAIL ------ Open was not successful
 */

int pressure_run_test(void) {
	uint32_t handle;
	int status = 0;

	PX4_INFO("entering: pressure_run_test");

	/*
	 * Open the pressure sensor at the specified device_path.
	 */
	status = pressure_api_open(PRESSURE_DEVICE_PATH, &handle);
	if (status == 0) {
		PX4_INFO("successfully opened pressure, handle: 0x%X", handle);
	} else {
		PX4_ERR("error: unable to obtain a valid handle for the receiver at: %s",
				PRESSURE_DEVICE_PATH);
		return TEST_FAIL;
	}

	uint32_t read_attempts = 0;
	uint32_t read_counter = 0;
	struct pressure_sensor_data sensor_data;
	sensor_data.sensor_read_counter = 0;

	while ((read_counter < 1000) && (read_attempts < 2000)) {
		status = pressure_api_get_sensor_data(handle, &sensor_data, true);

		if ((status == 0) && (read_counter != sensor_data.sensor_read_counter)) {
			read_counter = sensor_data.sensor_read_counter;
			print_pressure_values(&sensor_data);
		} else {
			PX4_ERR("error: unable to read the pressure sensor device.");
		}
		usleep(1000);
	}

	PX4_INFO("closing pressure sensor, handle 0x%X", handle);
	pressure_api_close(handle);

	return TEST_PASS;
}

