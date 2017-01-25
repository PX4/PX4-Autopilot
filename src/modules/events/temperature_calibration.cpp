/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
#include <vector>
#include <arch/board/board.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <platforms/px4_defines.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_gyro.h>
#include <controllib/uorb/blocks.hpp>

#include <uORB/topics/sensor_gyro.h>
#include "polyfit.hpp"
#include "temperature_calibration.h"

#define TC_PRINT_DEBUG 0
#if TC_PRINT_DEBUG
#define TC_DEBUG(fmt, ...) printf(fmt, ##__VA_ARGS__);
#else
#define TC_DEBUG(fmt, ...)
#endif


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

	static void do_temperature_calibration(int argc, char *argv[]);

	void		task_main();

	void print_status();

	void exit() { _task_should_exit = true; }

private:
	bool	_task_should_exit = false;
	int	_control_task = -1;		// task handle for task
};

Tempcal::Tempcal():
	SuperBlock(NULL, "Tempcal")
{
}

Tempcal::~Tempcal()
{

}

void Tempcal::task_main()
{
	// subscribe to relevant topics
	int gyro_sub[SENSOR_COUNT_MAX];
	float gyro_sample_filt[SENSOR_COUNT_MAX][4];
	polyfitter<4> P[SENSOR_COUNT_MAX][3];
	px4_pollfd_struct_t fds[SENSOR_COUNT_MAX] = {};
	unsigned _hot_soak_sat[SENSOR_COUNT_MAX] = {};
	unsigned num_gyro = orb_group_count(ORB_ID(sensor_gyro));
	unsigned num_samples[SENSOR_COUNT_MAX] = {0};
	uint32_t device_ids[SENSOR_COUNT_MAX] = {};

	if (num_gyro > SENSOR_COUNT_MAX) {
		num_gyro = SENSOR_COUNT_MAX;
	}

	bool _cold_soaked[SENSOR_COUNT_MAX] = {false};
	bool _hot_soaked[SENSOR_COUNT_MAX] = {false};
	bool _tempcal_complete[SENSOR_COUNT_MAX] = {false};
	float _low_temp[SENSOR_COUNT_MAX];
	float _high_temp[SENSOR_COUNT_MAX] = {0};
	float _ref_temp[SENSOR_COUNT_MAX];

	for (unsigned i = 0; i < num_gyro; i++) {
		gyro_sub[i] = orb_subscribe_multi(ORB_ID(sensor_gyro), i);
		fds[i].fd = gyro_sub[i];
		fds[i].events = POLLIN;
	}

	// initialize data structures outside of loop
	// because they will else not always be
	// properly populated
	sensor_gyro_s gyro_data = {};

	while (!_task_should_exit) {
		int ret = px4_poll(fds, num_gyro, 1000);

		if (ret < 0) {
			// Poll error, sleep and try again
			usleep(10000);
			continue;

		} else if (ret == 0) {
			// Poll timeout or no new data, do nothing
			continue;
		}

		for (unsigned i = 0; i < num_gyro; i++) {
			if (_hot_soaked[i]) {
				continue;
			}

			if (fds[i].revents & POLLIN) {
				orb_copy(ORB_ID(sensor_gyro), gyro_sub[i], &gyro_data);

				device_ids[i] = gyro_data.device_id;

				gyro_sample_filt[i][0] = gyro_data.x;
				gyro_sample_filt[i][1] = gyro_data.y;
				gyro_sample_filt[i][2] = gyro_data.z;
				gyro_sample_filt[i][3] = gyro_data.temperature;

				if (!_cold_soaked[i]) {
					_cold_soaked[i] = true;
					_low_temp[i] = gyro_sample_filt[i][3];	//Record the low temperature
					_ref_temp[i] = gyro_sample_filt[i][3] + 12.0f;
				}

				num_samples[i]++;
			}
		}

		for (unsigned i = 0; i < num_gyro; i++) {
			if (_hot_soaked[i]) {
				continue;
			}

			if (gyro_sample_filt[i][3] > _high_temp[i]) {
				_high_temp[i] = gyro_sample_filt[i][3];
				_hot_soak_sat[i] = 0;

			} else {
				continue;
			}

			//TODO: Hot Soak Saturation
			if (_hot_soak_sat[i] == 10 || (_high_temp[i] - _low_temp[i]) > 24.0f) {
				_hot_soaked[i] = true;
			}

			if (i == 0) {
				TC_DEBUG("\n%.20f,%.20f,%.20f,%.20f, %.6f, %.6f, %.6f\n\n", (double)gyro_sample_filt[i][0],
					 (double)gyro_sample_filt[i][1],
					 (double)gyro_sample_filt[i][2], (double)gyro_sample_filt[i][3], (double)_low_temp[i], (double)_high_temp[i],
					 (double)(_high_temp[i] - _low_temp[i]));
			}

			//update linear fit matrices
			gyro_sample_filt[i][3] -= _ref_temp[i];
			P[i][0].update((double)gyro_sample_filt[i][3], (double)gyro_sample_filt[i][0]);
			P[i][1].update((double)gyro_sample_filt[i][3], (double)gyro_sample_filt[i][1]);
			P[i][2].update((double)gyro_sample_filt[i][3], (double)gyro_sample_filt[i][2]);
			num_samples[i] = 0;
		}

		for (unsigned i = 0; i < num_gyro; i++) {
			if (_hot_soaked[i] && !_tempcal_complete[i]) {
				double res[3][4] = {0.0f};
				P[i][0].fit(res[0]);
				PX4_WARN("Result Gyro %d Axis 0: %.20f %.20f %.20f %.20f", i, (double)res[0][0], (double)res[0][1], (double)res[0][2],
					 (double)res[0][3]);
				P[i][1].fit(res[1]);
				PX4_WARN("Result Gyro %d Axis 1: %.20f %.20f %.20f %.20f", i, (double)res[1][0], (double)res[1][1], (double)res[1][2],
					 (double)res[1][3]);
				P[i][2].fit(res[2]);
				PX4_WARN("Result Gyro %d Axis 2: %.20f %.20f %.20f %.20f", i, (double)res[2][0], (double)res[2][1], (double)res[2][2],
					 (double)res[2][3]);
				_tempcal_complete[i] = true;

				char str[30];
				float param = 0.0f;
				int result = PX4_OK;

				sprintf(str, "TC_G%d_ID", i);
				result = param_set(param_find(str), &device_ids[i]);

				if (result != PX4_OK) {
					PX4_ERR("unable to reset %s", str);
				}

				for (unsigned j = 0; j < 3; j++) {
					for (unsigned m = 0; m <= 3; m++) {
						sprintf(str, "TC_G%d_X%d_%d", i, m, j);
						param = (float)res[j][m];
						result = param_set(param_find(str), &param);

						if (result != PX4_OK) {
							PX4_ERR("unable to reset %s", str);
						}
					}

					sprintf(str, "TC_G%d_TMAX", i);
					param = _high_temp[i];
					result = param_set(param_find(str), &param);

					if (result != PX4_OK) {
						PX4_ERR("unable to reset %s", str);
					}

					sprintf(str, "TC_G%d_TMIN", i);
					param = _low_temp[i];
					result = param_set(param_find(str), &param);

					if (result != PX4_OK) {
						PX4_ERR("unable to reset %s", str);
					}

					sprintf(str, "TC_G%d_TREF", i);
					param = _ref_temp[i];
					result = param_set(param_find(str), &param);

					if (result != PX4_OK) {
						PX4_ERR("unable to reset %s", str);
					}
				}

			}
		}

	}

	for (unsigned i = 0; i < num_gyro; i++) {
		orb_unsubscribe(gyro_sub[i]);
	}

	delete tempcal::instance;
	tempcal::instance = nullptr;
	PX4_INFO("Tempcal process stopped");
}

void Tempcal::do_temperature_calibration(int argc, char *argv[])
{
	tempcal::instance->task_main();
}

int Tempcal::start()
{

	ASSERT(_control_task == -1);
	_control_task = px4_task_spawn_cmd("temperature_calib",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   5800,
					   (px4_main_t)&Tempcal::do_temperature_calibration,
					   nullptr);

	if (_control_task < 0) {
		delete tempcal::instance;
		tempcal::instance = nullptr;
		PX4_ERR("start failed");
		return -errno;
	}

	return 0;
}

int run_temperature_calibration()
{
	PX4_INFO("Starting Temperature calibration task");
	tempcal::instance = new Tempcal();

	if (tempcal::instance == nullptr) {
		PX4_ERR("alloc failed");
		return 1;
	}

	return tempcal::instance->start();
}
