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
* @file PreflightCheck.cpp
*
* Preflight check for main system components
*
* @author Lorenz Meier <lorenz@px4.io>
* @author Johan Jansen <jnsn.johan@gmail.com>
*/

#include <nuttx/config.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>

#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <systemlib/rc_check.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_baro.h>

#include <mavlink/mavlink_log.h>

#include "PreflightCheck.h"

namespace Commander
{
static bool magnometerCheck(int mavlink_fd, unsigned instance, bool optional)
{
	bool success = true;

	char s[30];
	sprintf(s, "%s%u", MAG_BASE_DEVICE_PATH, instance);
	int fd = open(s, 0);

	if (fd < 0) {
		if (!optional) {
			mavlink_and_console_log_critical(mavlink_fd,
							 "PREFLIGHT FAIL: NO MAG SENSOR #%u", instance);
		}

		return false;
	}

	int calibration_devid;
	int ret;
	int devid = ioctl(fd, DEVIOCGDEVICEID, 0);
	sprintf(s, "CAL_MAG%u_ID", instance);
	param_get(param_find(s), &(calibration_devid));

	if (devid != calibration_devid) {
		mavlink_and_console_log_critical(mavlink_fd,
						 "PREFLIGHT FAIL: MAG #%u UNCALIBRATED (NO ID)", instance);
		success = false;
		goto out;
	}

	ret = ioctl(fd, MAGIOCSELFTEST, 0);

	if (ret != OK) {
		mavlink_and_console_log_critical(mavlink_fd,
						 "PREFLIGHT FAIL: MAG #%u SELFTEST FAILED", instance);
		success = false;
		goto out;
	}

out:
	close(fd);
	return success;
}

static bool accelerometerCheck(int mavlink_fd, unsigned instance, bool optional)
{
	bool success = true;

	char s[30];
	sprintf(s, "%s%u", ACCEL_BASE_DEVICE_PATH, instance);
	int fd = open(s, O_RDONLY);

	if (fd < 0) {
		if (!optional) {
			mavlink_and_console_log_critical(mavlink_fd,
							 "PREFLIGHT FAIL: NO ACCEL SENSOR #%u", instance);
		}

		return false;
	}

	int calibration_devid;
	int ret;
	int devid = ioctl(fd, DEVIOCGDEVICEID, 0);
	sprintf(s, "CAL_ACC%u_ID", instance);
	param_get(param_find(s), &(calibration_devid));

	if (devid != calibration_devid) {
		mavlink_and_console_log_critical(mavlink_fd,
						 "PREFLIGHT FAIL: ACCEL #%u UNCALIBRATED (NO ID)", instance);
		success = false;
		goto out;
	}

	ret = ioctl(fd, ACCELIOCSELFTEST, 0);

	if (ret != OK) {
		mavlink_and_console_log_critical(mavlink_fd,
						 "PREFLIGHT FAIL: ACCEL #%u SELFTEST FAILED", instance);
		success = false;
		goto out;
	}

out:
	close(fd);
	return success;
}

static bool gyroCheck(int mavlink_fd, unsigned instance, bool optional)
{
	bool success = true;

	char s[30];
	sprintf(s, "%s%u", GYRO_BASE_DEVICE_PATH, instance);
	int fd = open(s, 0);

	if (fd < 0) {
		if (!optional) {
			mavlink_and_console_log_critical(mavlink_fd,
							 "PREFLIGHT FAIL: NO GYRO SENSOR #%u", instance);
		}

		return false;
	}

	int calibration_devid;
	int ret;
	int devid = ioctl(fd, DEVIOCGDEVICEID, 0);
	sprintf(s, "CAL_GYRO%u_ID", instance);
	param_get(param_find(s), &(calibration_devid));

	if (devid != calibration_devid) {
		mavlink_and_console_log_critical(mavlink_fd,
						 "PREFLIGHT FAIL: GYRO #%u UNCALIBRATED (NO ID)", instance);
		success = false;
		goto out;
	}

	ret = ioctl(fd, GYROIOCSELFTEST, 0);

	if (ret != OK) {
		mavlink_and_console_log_critical(mavlink_fd,
						 "PREFLIGHT FAIL: GYRO #%u SELFTEST FAILED", instance);
		success = false;
		goto out;
	}

out:
	close(fd);
	return success;
}

static bool baroCheck(int mavlink_fd, unsigned instance, bool optional)
{
	bool success = true;

	char s[30];
	sprintf(s, "%s%u", BARO_BASE_DEVICE_PATH, instance);
	int fd = open(s, 0);

	if (fd < 0) {
		if (!optional) {
			mavlink_and_console_log_critical(mavlink_fd,
							 "PREFLIGHT FAIL: NO BARO SENSOR #%u", instance);
		}

		return false;
	}

	close(fd);
	return success;
}

bool preflightCheck(int mavlink_fd, bool checkMag, bool checkAcc, bool checkGyro, bool checkBaro, bool checkRC)
{
	bool failed = false;

//Magnetometer
	if (checkMag) {
		/* check all sensors, but fail only for mandatory ones */
		for (unsigned i = 0; i < max_optional_mag_count; i++) {
			bool required = (i < max_mandatory_mag_count);

			if (!magnometerCheck(mavlink_fd, i, !required) && required) {
				failed = true;
			}
		}
	}

//Accelerometer
	if (checkAcc) {
		/* check all sensors, but fail only for mandatory ones */
		for (unsigned i = 0; i < max_optional_accel_count; i++) {
			bool required = (i < max_mandatory_accel_count);

			if (!accelerometerCheck(mavlink_fd, i, !required) && required) {
				failed = true;
			}
		}
	}

// ---- GYRO ----
	if (checkGyro) {
		/* check all sensors, but fail only for mandatory ones */
		for (unsigned i = 0; i < max_optional_gyro_count; i++) {
			bool required = (i < max_mandatory_gyro_count);

			if (!gyroCheck(mavlink_fd, i, !required) && required) {
				failed = true;
			}
		}
	}

// ---- BARO ----
	if (checkBaro) {
		/* check all sensors, but fail only for mandatory ones */
		for (unsigned i = 0; i < max_optional_baro_count; i++) {
			bool required = (i < max_mandatory_baro_count);

			if (!baroCheck(mavlink_fd, i, !required) && required) {
				failed = true;
			}
		}
	}

// ---- RC CALIBRATION ----
	if (checkRC) {
		if (rc_calibration_check(mavlink_fd) != OK) {
			failed = true;
		}
	}

// Report status
	return !failed;
}
}
