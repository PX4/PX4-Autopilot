/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/console_buffer.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>

#include <lib/calibration/MagnetometerCalibration.hpp>
#include <lib/ecl/geo_lookup/geo_mag_declination.h>
#include <lib/matrix/matrix/math.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_gps_position.h>

#include <poll.h>
#include <stdio.h>

using namespace matrix;
using namespace time_literals;

static constexpr int MAX_SENSOR_COUNT = 4;

static void print_usage();

extern "C" __EXPORT int mag_test_main(int argc, char *argv[])
{
	bool once = false;

	if (argc > 1) {
		if (!strcmp(argv[1], "once")) {
			once = true;

		} else {
			print_usage();
			return 0;
		}
	}

	uORB::Subscription sensor_sub[MAX_SENSOR_COUNT] {
		{ORB_ID(sensor_mag), 0},
		{ORB_ID(sensor_mag), 1},
		{ORB_ID(sensor_mag), 2},
		{ORB_ID(sensor_mag), 3}
	};

	sensors::MagnetometerCalibration calibration[MAX_SENSOR_COUNT];

	uORB::Subscription sensor_combined_sub{ORB_ID(sensor_combined)};
	uORB::Subscription vehicle_gps_position_sub {ORB_ID(vehicle_gps_position)};

	for (;;) {

		float mag_declination = 0.f;
		vehicle_gps_position_s gps;

		if (vehicle_gps_position_sub.copy(&gps)) {
			if ((hrt_elapsed_time(&gps.timestamp) < 1_s) && (gps.eph < 1000)) {
				const double lat = gps.lat / 1.e7;
				const double lon = gps.lon / 1.e7;

				// set the magnetic field data returned by the geo library using the current GPS position
				float declination = math::radians(get_mag_declination(lat, lon));

				if (fabsf(declination - mag_declination) > 0.5f) {
					PX4_INFO("declination updated %.3f -> %.3f", (double)mag_declination, (double)declination);
				}

				mag_declination = declination;
			}
		}

		sensor_combined_s imu{};
		sensor_combined_sub.copy(&imu);
		const Vector3f accel{imu.accelerometer_m_s2};

		for (int mag_instance = 0; mag_instance < MAX_SENSOR_COUNT; mag_instance++) {
			sensor_mag_s mag{};

			if ((hrt_elapsed_time(&imu.timestamp) < 1_s) && sensor_sub[mag_instance].copy(&mag)) {

				calibration[mag_instance].set_external(mag.is_external);
				calibration[mag_instance].set_device_id(mag.device_id);

				const Vector3f m = calibration[mag_instance].Correct(Vector3f{mag.x, mag.y, mag.z});

				// Rotation matrix can be easily constructed from acceleration and mag field vectors
				// 'k' is Earth Z axis (Down) unit vector in body frame
				Vector3f k = -accel;
				k.normalize();

				// 'i' is Earth X axis (North) unit vector in body frame, orthogonal with 'k'
				Vector3f i = (m - k * (m * k));
				i.normalize();

				// 'j' is Earth Y axis (East) unit vector in body frame, orthogonal with 'k' and 'i'
				Vector3f j = k % i;

				// Fill rotation matrix
				Dcmf R;
				R.row(0) = i;
				R.row(1) = j;
				R.row(2) = k;

				// Convert to quaternion
				Quatf q = R;

				// Compensate for magnetic declination
				Quatf decl_rotation = Eulerf(0.0f, 0.0f, mag_declination);
				q = q * decl_rotation;
				q.normalize();
				const Eulerf euler{q};

				printf("| Mag #%u (%d) Heading:%4.0fº ", mag_instance, mag.device_id, (double)roundf(math::degrees(euler(2))));
			}
		}

		printf("\n");

		if (!once) {


			struct pollfd fds {};
			fds.fd = STDIN_FILENO;
			fds.events = POLLIN;

			if (poll(&fds, 1, 0) > 0) {
				char c;
				int ret = read(0, &c, 1);

				if (ret) {
					return 1;
				}

				switch (c) {
				case 0x03: // ctrl-c
				case 0x1b: // esc
				case 'c':
				case 'q':
					return 0;
				}
			}

			px4_usleep(70000);

		} else {
			return 0;
		}
	}

	return 0;
}

void print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This command is used to read and write GPIOs

gpio read <PORT> <PIN> [PULLDOWN|PULLUP] [--force]
gpio write <PORT> <PIN> <VALUE> [PUSHPULL|OPENDRAIN] [--force]

### Examples
Read the value on port H pin 4 configured as pullup, and it is high
$ mag_test
| Mag #0 (396825) Heading: 126º | Mag #1 (396809) Heading:  42º

)DESCR_STR");

	PRINT_MODULE_DESCRIPTION("This command computes the heading for each currently available magnetometer.");

	PRINT_MODULE_USAGE_NAME_SIMPLE("mag_test", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("once", "print load only once");
}
