/****************************************************************************
 *
 *   Copyright (c) 2015-2016 PX4 Development Team. All rights reserved.
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
 * @file tempcal_main.cpp
 * Implementation of the Temperature Calibration for onboard sensors.
 *
 * @author Siddharth Bharat Purohit
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <float.h>

#include <arch/board/board.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <platforms/px4_defines.h>
#include <drivers/drv_hrt.h>
#include <controllib/uorb/blocks.hpp>

#include <uORB/topics/sensor_gyro.h>
#include "polyfit.h"

#define SENSOR_COUNT_MAX		3

extern "C" __EXPORT int tempcal_main(int argc, char *argv[]);


class Tempcal;

namespace tempcal
{
Tempcal *instance = nullptr;
}


class Tempcal : public control::SuperBlock
{
public:
	/**
	 * Constructor
	 */
	Tempcal();

	/**
	 * Destructor, also kills task.
	 */
	~Tempcal();

	/**
	 * Start task.
	 *
	 * @return		OK on success.
	 */
	int		start();

	static void	task_main_trampoline(int argc, char *argv[]);

	void		task_main();

	void print_status();

	void exit() { _task_should_exit = true; }

private:
	bool	_task_should_exit = false;
	int	_control_task = -1;		// task handle for task

	/* Low pass filter for attitude rates */
	math::LowPassFilter2p _lp_roll_rate;
	math::LowPassFilter2p _lp_pitch_rate;
	math::LowPassFilter2p _lp_yaw_rate;
};

Tempcal::Tempcal():
	SuperBlock(NULL, "EKF"),
	_lp_roll_rate(250.0f, 1.0f),
	_lp_pitch_rate(250.0f, 1.0f),
	_lp_yaw_rate(250.0f, 1.0f)
{

}

Tempcal::~Tempcal()
{

}

void Tempcal::print_status()
{
	//TODO: Implement Print Status
}

void Tempcal::task_main()
{
	// subscribe to relevant topics
	int gyro_sub[SENSOR_COUNT_MAX];
	float gyro_sample_filt[SENSOR_COUNT_MAX][4];
	polyfitter P[SENSOR_COUNT_MAX][3];
	px4_pollfd_struct_t fds[SENSOR_COUNT_MAX] = {};
	uint8_t _hot_soak_sat[SENSOR_COUNT_MAX] = {};
	unsigned num_gyro = orb_group_count(ORB_ID(sensor_gyro));
	uint16_t num_samples[SENSOR_COUNT_MAX] = {0};

	bool _cold_soaked[SENSOR_COUNT_MAX] = {false};
	bool _hot_soaked[SENSOR_COUNT_MAX] = {false};

	float _low_temp[SENSOR_COUNT_MAX], _high_temp[SENSOR_COUNT_MAX];

	for (unsigned i = 0; i < num_gyro; i++) {
		if (gyro_sub[i] < 0) {
			gyro_sub[i] = orb_subscribe_multi(ORB_ID(sensor_gyro), i);
		}
	}

	for (uint8_t i = 0; i < num_gyro; i++) {
		fds[i].fd = gyro_sub[i];
		fds[i].events = POLLIN;
		P[i][0].init(3);
		P[i][1].init(3);
		P[i][2].init(3);
	}

	// initialize data structures outside of loop
	// because they will else not always be
	// properly populated
	sensor_gyro_s gyro_data = {};

	while (!_task_should_exit) {
		int ret = px4_poll(fds, sizeof(fds) / sizeof(fds[0]), 1000);

		if (ret < 0) {
			// Poll error, sleep and try again
			usleep(10000);
			continue;

		} else if (ret == 0) {
			// Poll timeout or no new data, do nothing
			continue;
		}

		for (uint8_t i = 0; i < num_gyro; i++) {
			if (_hot_soaked[i]) {
				continue;
			}

			if (fds[i].revents & POLLIN) {
				orb_copy(ORB_ID(sensor_gyro), gyro_sub[i], &gyro_data);
				gyro_sample_filt[i][0] = _lp_roll_rate.apply(gyro_data.x);
				gyro_sample_filt[i][1] = _lp_roll_rate.apply(gyro_data.y);
				gyro_sample_filt[i][2] = _lp_roll_rate.apply(gyro_data.z);
				gyro_sample_filt[i][3] = gyro_data.temperature;

				if (!_cold_soaked[i]) {
					_cold_soaked[i] = true;
					_low_temp[i] = gyro_data.temperature;	//Record the low temperature
				}

				num_samples[i]++;
			}
		}

		bool _collection_complete = true;

		for (uint8_t i = 0; i < num_gyro; i++) {
			if (_hot_soaked[i]) {
				continue;

			} else {
				_collection_complete = false;
			}

			if (num_samples[i] < 250) {
				continue;
			}

			if (gyro_sample_filt[i][3] <= _low_temp[i]) {
				//We are not hot soaking increment hot soak saturation count
				_hot_soak_sat[i]++;
			}

			if (_hot_soak_sat[i] == 10) {
				_hot_soaked[i] = true;
				_high_temp[i] = gyro_sample_filt[i][3];
			}

			printf("Got Here!! %.6f %.6f %.6f %.6f\n", (double)gyro_sample_filt[i][0], (double)gyro_sample_filt[i][1],
			       (double)gyro_sample_filt[i][2], (double)gyro_sample_filt[i][3]);
			//update linear fit matrices
			P[i][0].update(gyro_sample_filt[i][0], gyro_sample_filt[i][3]);
			P[i][1].update(gyro_sample_filt[i][1], gyro_sample_filt[i][3]);
			P[i][2].update(gyro_sample_filt[i][2], gyro_sample_filt[i][3]);
			num_samples[i] = 0;
		}

		if (_collection_complete) {
			for (uint8_t i = 0; i < num_gyro; i++) {
				if (_high_temp[i] - _low_temp[i] > 20) {
					PX4_WARN("Cal Failed for Gyro %d", i);

				} else {
					float res[4];
					P[i][0].fit(res);
					PX4_WARN("Result Gyro %d Axis 0: %.6f %.6f %.6f %.6f", i, (double)res[0], (double)res[1], (double)res[2],
						 (double)res[3]);
					P[i][1].fit(res);
					PX4_WARN("Result Gyro %d Axis 1: %.6f %.6f %.6f %.6f", i, (double)res[0], (double)res[1], (double)res[2],
						 (double)res[3]);
					P[i][2].fit(res);
					PX4_WARN("Result Gyro %d Axis 2: %.6f %.6f %.6f %.6f", i, (double)res[0], (double)res[1], (double)res[2],
						 (double)res[3]);
				}
			}

			break; //complete temp cal
		}
	}

	for (uint8_t i = 0; i < num_gyro; i++) {
		orb_unsubscribe(gyro_sub[i]);
	}

	delete tempcal::instance;
	tempcal::instance = nullptr;
	PX4_WARN("Tempcal process stopped");
}


void Tempcal::task_main_trampoline(int argc, char *argv[])
{
	tempcal::instance->task_main();
}

int Tempcal::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("tempcal",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   5800,
					   (px4_main_t)&Tempcal::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		PX4_WARN("task start failed");
		return -errno;

	} else {
		PX4_WARN("Tempcal proc started");
	}

	return OK;
}

int tempcal_main(int argc, char *argv[])
{
	if (argc < 2) {
		PX4_WARN("usage: tempcal {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (tempcal::instance != nullptr) {
			PX4_WARN("already running");
			return 1;
		}

		tempcal::instance = new Tempcal();

		if (tempcal::instance == nullptr) {
			PX4_WARN("alloc failed");
			return 1;
		}

		if (OK != tempcal::instance->start()) {
			delete tempcal::instance;
			tempcal::instance = nullptr;
			PX4_WARN("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (tempcal::instance == nullptr) {
			PX4_WARN("not running");
			return 1;
		}

		tempcal::instance->exit();

		// wait for the destruction of the instance
		while (tempcal::instance != nullptr) {
			usleep(50000);
		}

		return 0;
	}

	if (!strcmp(argv[1], "print")) {
		if (tempcal::instance != nullptr) {

			return 0;
		}

		return 1;
	}

	if (!strcmp(argv[1], "status")) {
		if (tempcal::instance) {
			PX4_WARN("running");
			tempcal::instance->print_status();
			return 0;

		} else {
			PX4_WARN("not running");
			return 1;
		}
	}

	PX4_WARN("unrecognized command");
	return 1;
}
