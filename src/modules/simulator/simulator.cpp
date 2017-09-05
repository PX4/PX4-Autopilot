/****************************************************************************
 *
 *   Copyright (c) 2015 Mark Charlebois. All rights reserved.
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
 * @file simulator.cpp
 * A device simulator
 */

#include <px4_log.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <pthread.h>
#include <poll.h>
#include <systemlib/err.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <drivers/drv_board_led.h>

#include "simulator.h"

using namespace simulator;

static px4_task_t g_sim_task = -1;

Simulator *Simulator::_instance = nullptr;

Simulator *Simulator::getInstance()
{
	return _instance;
}

bool Simulator::getGyroReport(simulator::RawGyroData *report)
{
	return _gyro.copyData(report);
}

bool Simulator::getAccelReport(simulator::RawAccelData *report)
{
	return _accel.copyData(report);
}

bool Simulator::getMagReport(simulator::RawMagData *report)
{
	return _mag.copyData(report);
}

bool Simulator::getBaroSample(simulator::RawBaroData *report)
{
	return _baro.copyData(report);
}

bool Simulator::getGPSSample(simulator::RawGPSData *report)
{
	return _gps.copyData(report);
}

bool Simulator::getAirspeedSample(simulator::RawAirspeedData *report)
{
	return _airspeed.copyData(report);
}

void Simulator::write_gyro_data(const simulator::RawGyroData *report)
{
	_gyro.writeData(report);
}

void Simulator::write_accel_data(const simulator::RawAccelData *report)
{
	_accel.writeData(report);
}

void Simulator::write_mag_data(const simulator::RawMagData *report)
{
	_mag.writeData(report);
}

void Simulator::write_baro_data(const simulator::RawBaroData *report)
{
	_baro.writeData(report);
}

void Simulator::write_gps_data(const simulator::RawGPSData *report)
{
	_gps.writeData(report);
}

void Simulator::write_airspeed_data(const simulator::RawAirspeedData *report)
{
	_airspeed.writeData(report);
}

void Simulator::parameters_update(bool force)
{
	bool updated;
	struct parameter_update_s param_upd;

	orb_check(_param_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), _param_sub, &param_upd);
	}

	if (updated || force) {
		// update C++ param system
		updateParams();
	}
}

int Simulator::start(int argc, char *argv[])
{
	int ret = 0;
	int udp_port = 0;
	_instance = new Simulator();

	if (_instance) {
		drv_led_start();

		if (argc == 5 && strcmp(argv[3], "-u") == 0) {
			udp_port = atoi(argv[4]);
		}

		if (argv[2][1] == 's') {
			_instance->initializeSensorData();
#ifndef __PX4_QURT
			// Update sensor data
			_instance->pollForMAVLinkMessages(false, udp_port);
#endif

		} else if (argv[2][1] == 'p') {
			// Update sensor data
			_instance->pollForMAVLinkMessages(true, udp_port);

		} else {
			_instance->initializeSensorData();
			_instance->_initialized = true;
		}

	} else {
		PX4_WARN("Simulator creation failed");
		ret = 1;
	}

	return ret;
}

static void usage()
{
	PX4_WARN("Usage: simulator {start -[spt] [-u udp_port] |stop}");
	PX4_WARN("Simulate raw sensors:     simulator start -s");
	PX4_WARN("Publish sensors combined: simulator start -p");
	PX4_WARN("Dummy unit test data:     simulator start -t");
}

__BEGIN_DECLS
extern int simulator_main(int argc, char *argv[]);
__END_DECLS

extern "C" {

	int simulator_main(int argc, char *argv[])
	{
		int ret = 0;

		if (argc > 2 && strcmp(argv[1], "start") == 0) {
			if (strcmp(argv[2], "-s") == 0 ||
			    strcmp(argv[2], "-p") == 0 ||
			    strcmp(argv[2], "-t") == 0) {

				if (g_sim_task >= 0) {
					warnx("Simulator already started");
					return 0;
				}

				// enable lockstep support
				px4_enable_sim_lockstep();

				g_sim_task = px4_task_spawn_cmd("simulator",
								SCHED_DEFAULT,
								SCHED_PRIORITY_MAX,
								1500,
								Simulator::start,
								argv);

				// now wait for the command to complete
				while (!px4_exit_requested()) {
					if (Simulator::getInstance() && Simulator::getInstance()->isInitialized()) {
						break;

					} else {
						usleep(100000);
					}
				}

			} else {
				usage();
				ret = -EINVAL;
			}

		} else if (argc == 2 && strcmp(argv[1], "stop") == 0) {
			if (g_sim_task < 0) {
				PX4_WARN("Simulator not running");

			} else {
				px4_task_delete(g_sim_task);
				g_sim_task = -1;
			}

		} else {
			usage();
			ret = -EINVAL;
		}

		return ret;
	}

}
