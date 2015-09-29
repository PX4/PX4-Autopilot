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

#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <math.h>
#include <assert.h>

#include <px4_tasks.h>
#include <px4_log.h>
#include <px4_getopt.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_baro.h>

#include <semaphore.h>
#include <pressure_api.h>

/** driver 'main' command */
extern "C" {
__EXPORT int pressure_main(int argc, char *argv[]);
}

#define WAIT_FOR_NEW_BARO_DATA true
#define MAX_LEN_DEV_PATH 32

namespace pressure_sensor {

/** SPI bus that the pressure sensor is connected to */
static char _device[MAX_LEN_DEV_PATH];

/** flag indicating if the pressure sensor application is running */
static bool _is_running = false;

/** flag indicating if measurement thread should exit */
static bool _task_should_exit = false;

/** handle to the task main thread */
static px4_task_t _task_handle = (px4_task_t) - 1;

/** uORB topic to report the pressure sensor data */
static orb_advert_t _baro_pub = (orb_advert_t) -1;

/** data structure associated with the baro uORB topic */
static struct sensor_baro_s _baro_report;

/** data structure used for local copy of the last baro sample */
static struct pressure_sensor_data _data;

/** Print out the usage information */
static void usage();

/** start taking measurements */
static void start(const char *device);

/** stop taking measurements */
static void stop();

/** task main trampoline function */
static void task_main_trampoline(int argc, char *argv[]);

/** measurement thread primary entry point */
static void task_main(int argc, char *argv[]);

/**
 * create and advertise all publications
 * @return   true on success, false otherwise
 */
static bool create_pubs();

/** update all sensor reports with the latest sensor reading */
static void update_reports();

/** publish all sensor reports */
static void publish_reports();

bool create_pubs() {
	/* initialize the reports */
	memset(&_baro_report, 0, sizeof(_baro_report));
	_baro_pub = orb_advertise(ORB_ID(sensor_baro), &_baro_report);

	if (_baro_pub == (orb_advert_t) (-1)) {
		/* advertise the existence of a baro topic */
		PX4_WARN("failed to advertise sensor_baro topic");
		return false;
	}

	return true;
}

/* Converts pressure in mbar to altitude in meters, using 1976 US Standard
 * Atmosphere model (note that this formula only applies to a height of 11 km,
 * or about 36000 ft).  If altimeter setting (QNH, barometric pressure adjusted
 * to sea level) is given, this function returns an indicated altitude compensated
 * for actual regional pressure; otherwise, it returns the pressure altitude above
 * the standard pressure level of 1013.25 mbar or 29.9213 inHg
 */
float convert_mbars_to_meters(float pressure_mbar,
		float altimeter_setting_mbar) {
	if (altimeter_setting_mbar > 0)
		return (1.0
				- pow((float) (pressure_mbar / altimeter_setting_mbar),
						(float) 0.190263)) * 44330.8;
	else
		return (1.0 - pow(pressure_mbar / 1013.25, 0.190263)) * 44330.8;
}

void update_reports() {
	PX4_DEBUG(
			"Baro Data: timestamp: %llu, error count: %llu, temperature in C: %f, pressure: %d",
			_data.last_read_time_in_usecs, _data.error_count, _data.temperature_in_c, _data.pressure_in_pa);
	_baro_report.timestamp = _data.last_read_time_in_usecs;
	_baro_report.error_count = _data.error_count;
	_baro_report.temperature = _data.temperature_in_c;

	/*
	 * Save the pressure to millibars for the baro report, using a conversion
	 * factor of 1 pascal = .01 millibars
	 */
	_baro_report.pressure = (float) _data.pressure_in_pa * .01;

	/*
	 * Use the pressure in millibars to generate an altitude in meters, assuming
	 * no compensation for regional pressure difference at MSL.
	 */
	_baro_report.altitude = convert_mbars_to_meters(_baro_report.pressure, 0);
}

void publish_reports() {
	if (orb_publish(ORB_ID(sensor_baro), _baro_pub, &_baro_report) != 0) {
		PX4_WARN("failed to publish gyro report");
	} else {
		PX4_DEBUG("Baro Report: %f (meters), %f (mbars)",
				_baro_report.altitude, _baro_report.pressure);
	}
}

void task_main(int argc, char *argv[]) {
	PX4_WARN("entering task_main");

	sigset_t set;
	int sig = 0;
	int rv;
	uint32_t device_handle;

	if (pressure_api_open(_device, &device_handle) != 0) {
		PX4_WARN("error: initializing pressure sensor");
		goto exit;
	}

	/* Create all uORB publications. */
	if (!create_pubs()) {
		goto exit;
	}

	while (!_task_should_exit) {
		/* read the current baro pressure sample */
		if (pressure_api_get_sensor_data(device_handle, &_data,
				WAIT_FOR_NEW_BARO_DATA) != 0) {
			PX4_WARN("error: reading data from the pressure sensor");
			continue;
		}

		/* Process the data obtained above. */
		update_reports();

		/* Publish the processed data for subscribers to use. */
		publish_reports();
	}

	exit:
	PX4_WARN("closing pressure sensor");
	pressure_api_close(device_handle);
}

void task_main_trampoline(int argc, char *argv[]) {
	PX4_WARN("task_main_trampoline");
	task_main(argc, argv);
}

void start() {
	assert(_task_handle == (px4_task_t)-1);

	/* start the task */
	_task_handle = px4_task_spawn_cmd("pressure_sensor_main", SCHED_DEFAULT,
			SCHED_PRIORITY_MAX, 1500, (px4_main_t) &task_main_trampoline, nullptr);

	if (_task_handle < (px4_task_t) 0) {
		PX4_WARN("task start failed");
		return;
	}

	_is_running = true;
}

void stop() {
	_task_should_exit = true;
	_is_running = false;
	_task_handle = -1;
}

void usage() {
	PX4_WARN("missing command: try 'start', 'stop', 'status'");
	PX4_WARN("options:");
	PX4_WARN("    -D device");
}

}; /* namespace pressure_sensor */

int pressure_main(int argc, char *argv[]) {
	const char *device = NULL;
	int ch;
	int opt_ind = 1;
	const char *opt_arg = NULL;

	while ((ch = px4_getopt(argc, argv, "D:", &opt_ind, &opt_arg)) != EOF) {
		switch (ch) {
		case 'D':
			device = opt_arg;
			break;

		default:
			pressure_sensor::usage();
			return 1;
		}
	}

	// Check on required arguments
	if (device == NULL || strlen(device) == 0) {
		pressure_sensor::usage();
		return 1;
	}

	memset(pressure_sensor::_device, 0, MAX_LEN_DEV_PATH);
	strncpy(pressure_sensor::_device, device, strlen(device));

	const char *verb = argv[opt_ind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		if (pressure_sensor::_is_running) {
			PX4_WARN("pressure sensor is already running");
			return 1;
		}

		PX4_WARN("pressure sensor is starting");
		pressure_sensor::start();
	} else if (!strcmp(verb, "stop")) {
		if (pressure_sensor::_is_running) {
			PX4_WARN("pressure sensor is not running");
			return 1;
		}

		pressure_sensor::stop();
	} else if (!strcmp(verb, "status")) {
		PX4_WARN("pressure sensor is %s",
				pressure_sensor::_is_running ? "running" : "stopped");
		return 0;
	} else {
		pressure_sensor::usage();
		return 1;
	}

	return 0;
}
