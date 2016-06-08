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

#include <px4_config.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>

#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <systemlib/rc_check.h>
#include <systemlib/mavlink_log.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_baro.h>
#include <drivers/drv_airspeed.h>

#include <uORB/topics/airspeed.h>
#include <uORB/topics/vehicle_gps_position.h>


#include "PreflightCheck.h"

#include "DevMgr.hpp"

using namespace DriverFramework;

namespace Commander
{

static int check_calibration(DevHandle &h, const char* param_template, int &devid)
{
	bool calibration_found;

	/* new style: ask device for calibration state */
	int ret = h.ioctl(SENSORIOCCALTEST, 0);

	calibration_found = (ret == OK);

	devid = h.ioctl(DEVIOCGDEVICEID, 0);

	char s[20];
	int instance = 0;

	/* old style transition: check param values */
	while (!calibration_found) {
		sprintf(s, param_template, instance);
		param_t parm = param_find(s);

		/* if the calibration param is not present, abort */
		if (parm == PARAM_INVALID) {
			break;
		}

		/* if param get succeeds */
		int calibration_devid;
		if (!param_get(parm, &(calibration_devid))) {

			/* if the devid matches, exit early */
			if (devid == calibration_devid) {
				calibration_found = true;
				break;
			}
		}
		instance++;
	}

	return !calibration_found;
}

static bool magnometerCheck(orb_advert_t *mavlink_log_pub, unsigned instance, bool optional, int &device_id, bool report_fail)
{
	bool success = true;

	char s[30];
	sprintf(s, "%s%u", MAG_BASE_DEVICE_PATH, instance);
	DevHandle h;
	DevMgr::getHandle(s, h);

	if (!h.isValid()) {
		if (!optional) {
			if (report_fail) {
				mavlink_and_console_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: NO MAG SENSOR #%u", instance);
			}
		}

		return false;
	}

	int ret = check_calibration(h, "CAL_MAG%u_ID", device_id);

	if (ret) {
		if (report_fail) {
			mavlink_and_console_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: MAG #%u UNCALIBRATED", instance);
		}
		success = false;
		goto out;
	}

	ret = h.ioctl(MAGIOCSELFTEST, 0);

	if (ret != OK) {
		if (report_fail) {
			mavlink_and_console_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: MAG #%u SELFTEST FAILED", instance);
		}
		success = false;
		goto out;
	}

out:
	DevMgr::releaseHandle(h);
	return success;
}

static bool accelerometerCheck(orb_advert_t *mavlink_log_pub, unsigned instance, bool optional, bool dynamic, int &device_id, bool report_fail)
{
	bool success = true;

	char s[30];
	sprintf(s, "%s%u", ACCEL_BASE_DEVICE_PATH, instance);
	DevHandle h;
	DevMgr::getHandle(s, h);

	if (!h.isValid()) {
		if (!optional) {
			if (report_fail) {
				mavlink_and_console_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: NO ACCEL SENSOR #%u", instance);
			}
		}

		return false;
	}

	int ret = check_calibration(h, "CAL_ACC%u_ID", device_id);

	if (ret) {
		if (report_fail) {
			mavlink_and_console_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: ACCEL #%u UNCALIBRATED", instance);
		}
		success = false;
		goto out;
	}

	ret = h.ioctl(ACCELIOCSELFTEST, 0);

	if (ret != OK) {
		if (report_fail) {
			mavlink_and_console_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: ACCEL #%u TEST FAILED: %d", instance, ret);
		}
		success = false;
		goto out;
	}

#ifdef __PX4_NUTTX
	if (dynamic) {
		/* check measurement result range */
		struct accel_report acc;
		ret = h.read(&acc, sizeof(acc));

		if (ret == sizeof(acc)) {
			/* evaluate values */
			float accel_magnitude = sqrtf(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);

			if (accel_magnitude < 4.0f || accel_magnitude > 15.0f /* m/s^2 */) {
				if (report_fail) {
					mavlink_and_console_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: ACCEL RANGE, hold still on arming");
				}
				/* this is frickin' fatal */
				success = false;
				goto out;
			}
		} else {
			if (report_fail) {
				mavlink_and_console_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: ACCEL READ");
			}
			/* this is frickin' fatal */
			success = false;
			goto out;
		}
	}
#endif

out:
	DevMgr::releaseHandle(h);
	return success;
}

static bool gyroCheck(orb_advert_t *mavlink_log_pub, unsigned instance, bool optional, int &device_id, bool report_fail)
{
	bool success = true;

	char s[30];
	sprintf(s, "%s%u", GYRO_BASE_DEVICE_PATH, instance);
	DevHandle h;
	DevMgr::getHandle(s, h);

	if (!h.isValid()) {
		if (!optional) {
			if (report_fail) {
				mavlink_and_console_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: NO GYRO SENSOR #%u", instance);
			}
		}

		return false;
	}

	int ret = check_calibration(h, "CAL_GYRO%u_ID", device_id);

	if (ret) {
		if (report_fail) {
			mavlink_and_console_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: GYRO #%u UNCALIBRATED", instance);
		}
		success = false;
		goto out;
	}

	ret = h.ioctl(GYROIOCSELFTEST, 0);

	if (ret != OK) {
		if (report_fail) {
			mavlink_and_console_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: GYRO #%u SELFTEST FAILED", instance);
		}
		success = false;
		goto out;
	}

out:
	DevMgr::releaseHandle(h);
	return success;
}

static bool baroCheck(orb_advert_t *mavlink_log_pub, unsigned instance, bool optional, int &device_id, bool report_fail)
{
	bool success = true;

	char s[30];
	sprintf(s, "%s%u", BARO_BASE_DEVICE_PATH, instance);
	DevHandle h;
	DevMgr::getHandle(s, h);

	if (!h.isValid()) {
		if (!optional) {
			if (report_fail) {
				mavlink_and_console_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: NO BARO SENSOR #%u", instance);
			}
		}

		return false;
	}

	device_id = -1000;

	// TODO: There is no baro calibration yet, since no external baros exist
	// int ret = check_calibration(fd, "CAL_BARO%u_ID");

	// if (ret) {
	// 	mavlink_and_console_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: BARO #%u UNCALIBRATED", instance);
	// 	success = false;
	// 	goto out;
	// }

//out:

	DevMgr::releaseHandle(h);
	return success;
}

static bool airspeedCheck(orb_advert_t *mavlink_log_pub, bool optional, bool report_fail)
{
	bool success = true;
	int ret;
	int fd = orb_subscribe(ORB_ID(airspeed));

	struct airspeed_s airspeed;

	if ((ret = orb_copy(ORB_ID(airspeed), fd, &airspeed)) ||
	    (hrt_elapsed_time(&airspeed.timestamp) > (500 * 1000))) {
		if (report_fail) {
			mavlink_and_console_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: AIRSPEED SENSOR MISSING");
		}
		success = false;
		goto out;
	}

	if (fabsf(airspeed.confidence) < 0.99f) {
		if (report_fail) {
			mavlink_and_console_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: AIRSPEED SENSOR COMM ERROR");
		}
		success = false;
		goto out;
	}

	if (fabsf(airspeed.indicated_airspeed_m_s) > 6.0f) {
		if (report_fail) {
			mavlink_and_console_log_critical(mavlink_log_pub, "AIRSPEED WARNING: WIND OR CALIBRATION ISSUE");
		}
		// XXX do not make this fatal yet
	}

out:
	px4_close(fd);
	return success;
}

static bool gnssCheck(orb_advert_t *mavlink_log_pub, bool report_fail)
{
	bool success = true;

	int gpsSub = orb_subscribe(ORB_ID(vehicle_gps_position));

	//Wait up to 2000ms to allow the driver to detect a GNSS receiver module
	px4_pollfd_struct_t fds[1];
	fds[0].fd = gpsSub;
	fds[0].events = POLLIN;
	if(px4_poll(fds, 1, 2000) <= 0) {
		success = false;
	}
	else {
		struct vehicle_gps_position_s gps;
		if ( (OK != orb_copy(ORB_ID(vehicle_gps_position), gpsSub, &gps)) ||
		    (hrt_elapsed_time(&gps.timestamp_position) > 1000000)) {
			success = false;
		}
	}

	//Report failure to detect module
	if (!success) {
		if (report_fail) {
			mavlink_and_console_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: GPS RECEIVER MISSING");
		}
	}

	px4_close(gpsSub);
	return success;
}

bool preflightCheck(orb_advert_t *mavlink_log_pub, bool checkMag, bool checkAcc, bool checkGyro,
		    bool checkBaro, bool checkAirspeed, bool checkRC, bool checkGNSS, bool checkDynamic, bool reportFailures)
{

#ifdef __PX4_QURT
	// WARNING: Preflight checks are important and should be added back when
	// all the sensors are supported
	PX4_WARN("Preflight checks always pass on Snapdragon.");
	return true;
#endif

	bool failed = false;

	/* ---- MAG ---- */
	if (checkMag) {
		bool prime_found = false;
		int32_t prime_id = 0;
		param_get(param_find("CAL_MAG_PRIME"), &prime_id);

		/* check all sensors, but fail only for mandatory ones */
		for (unsigned i = 0; i < max_optional_mag_count; i++) {
			bool required = (i < max_mandatory_mag_count);
			int device_id = -1;

			if (!magnometerCheck(mavlink_log_pub, i, !required, device_id, reportFailures) && required) {
				failed = true;
			}

			if (device_id == prime_id) {
				prime_found = true;
			}
		}

		/* check if the primary device is present */
		if (!prime_found && prime_id != 0) {
			if (reportFailures) {
				mavlink_and_console_log_critical(mavlink_log_pub, "Warning: Primary compass not found");
			}
			failed = true;
		}
	}

	/* ---- ACCEL ---- */
	if (checkAcc) {
		bool prime_found = false;
		int32_t prime_id = 0;
		param_get(param_find("CAL_ACC_PRIME"), &prime_id);

		/* check all sensors, but fail only for mandatory ones */
		for (unsigned i = 0; i < max_optional_accel_count; i++) {
			bool required = (i < max_mandatory_accel_count);
			int device_id = -1;

			if (!accelerometerCheck(mavlink_log_pub, i, !required, checkDynamic, device_id, reportFailures) && required) {
				failed = true;
			}

			if (device_id == prime_id) {
				prime_found = true;
			}
		}

		/* check if the primary device is present */
		if (!prime_found && prime_id != 0) {
			if (reportFailures) {
				mavlink_and_console_log_critical(mavlink_log_pub, "Warning: Primary accelerometer not found");
			}
			failed = true;
		}
	}

	/* ---- GYRO ---- */
	if (checkGyro) {
		bool prime_found = false;
		int32_t prime_id = 0;
		param_get(param_find("CAL_GYRO_PRIME"), &prime_id);

		/* check all sensors, but fail only for mandatory ones */
		for (unsigned i = 0; i < max_optional_gyro_count; i++) {
			bool required = (i < max_mandatory_gyro_count);
			int device_id = -1;

			if (!gyroCheck(mavlink_log_pub, i, !required, device_id, reportFailures) && required) {
				failed = true;
			}

			if (device_id == prime_id) {
				prime_found = true;
			}
		}

		/* check if the primary device is present */
		if (!prime_found && prime_id != 0) {
			if (reportFailures) {
				mavlink_and_console_log_critical(mavlink_log_pub, "Warning: Primary gyro not found");
			}
			failed = true;
		}
	}

	/* ---- BARO ---- */
	if (checkBaro) {
		bool prime_found = false;
		int32_t prime_id = 0;
		param_get(param_find("CAL_BARO_PRIME"), &prime_id);

		/* check all sensors, but fail only for mandatory ones */
		for (unsigned i = 0; i < max_optional_baro_count; i++) {
			bool required = (i < max_mandatory_baro_count);
			int device_id = -1;

			if (!baroCheck(mavlink_log_pub, i, !required, device_id, reportFailures) && required) {
				failed = true;
			}

			if (device_id == prime_id) {
				prime_found = true;
			}
		}

		// TODO there is no logic in place to calibrate the primary baro yet
		// // check if the primary device is present
		if (!prime_found && prime_id != 0) {
			if (reportFailures) {
				mavlink_and_console_log_critical(mavlink_log_pub, "warning: primary barometer not operational");
			}
			failed = true;
		}
	}

	/* ---- AIRSPEED ---- */
	if (checkAirspeed) {
		if (!airspeedCheck(mavlink_log_pub, true, reportFailures)) {
			failed = true;
		}
	}

	/* ---- RC CALIBRATION ---- */
	if (checkRC) {
		if (rc_calibration_check(mavlink_log_pub, reportFailures) != OK) {
			if (reportFailures) {
				mavlink_and_console_log_critical(mavlink_log_pub, "RC calibration check failed");
			}
			failed = true;
		}
	}

	/* ---- Global Navigation Satellite System receiver ---- */
	if (checkGNSS) {
		if (!gnssCheck(mavlink_log_pub, reportFailures)) {
			failed = true;
		}
	}

	/* Report status */
	return !failed;
}

}
