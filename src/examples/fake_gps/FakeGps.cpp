/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "FakeGps.hpp"

using namespace time_literals;

ModuleBase::Descriptor FakeGps::desc{task_spawn, custom_command, print_usage};

FakeGps::FakeGps(double latitude_deg, double longitude_deg, double altitude_m) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
	_latitude(latitude_deg),
	_longitude(longitude_deg),
	_altitude(altitude_m)
{
}

bool FakeGps::init()
{
	ScheduleOnInterval(SENSOR_INTERVAL_US);
	return true;
}

void FakeGps::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	sensor_gnss_s sensor_gnss{};
	sensor_gnss.time_utc_usec = hrt_absolute_time() + 1613692609599954;
	sensor_gnss.latitude = _latitude;
	sensor_gnss.longitude = _longitude;
	sensor_gnss.altitude_msl = _altitude;
	sensor_gnss.altitude_ellipsoid = _altitude;
	sensor_gnss.speed_accuracy = 0.3740f;
	sensor_gnss.course_accuracy = 0.6737f;
	sensor_gnss.eph = 2.1060f;
	sensor_gnss.epv = 3.8470f;
	sensor_gnss.hdop = 0.8800f;
	sensor_gnss.vdop = 1.3300f;
	sensor_gnss.noise = 101;
	sensor_gnss.jamming_indicator = 35;
	sensor_gnss.ground_speed = 0.0420f;
	sensor_gnss.vel_north = 0.0370f;
	sensor_gnss.vel_east = 0.0200f;
	sensor_gnss.vel_down = -0.0570f;
	sensor_gnss.course = 0.3988f;
	sensor_gnss.timestamp_time_relative = 0;
	sensor_gnss.fix_type = 4;
	sensor_gnss.jamming_state = 0;
	sensor_gnss.spoofing_state = 0;
	sensor_gnss.vel_ned_valid = true;
	sensor_gnss.satellites_used = 14;
	sensor_gnss.timestamp = hrt_absolute_time();
	_sensor_gnss_pub.publish(sensor_gnss);

	sensor_gnss_status_s sensor_gnss_status{};
	sensor_gnss_status.quality_corrections = 0;
	sensor_gnss_status.quality_receiver = 9;
	sensor_gnss_status.quality_gnss_signals = 10;
	sensor_gnss_status.quality_post_processing = 255;
	sensor_gnss_status.timestamp = hrt_absolute_time();
	_sensor_gnss_status_pub.publish(sensor_gnss_status);
}

int FakeGps::task_spawn(int argc, char *argv[])
{
	FakeGps *instance = new FakeGps();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int FakeGps::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FakeGps::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fake_gps", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int fake_gps_main(int argc, char *argv[])
{
	return ModuleBase::main(FakeGps::desc, argc, argv);
}
