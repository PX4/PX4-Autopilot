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
 * @file temperature_accel_calibration.cpp
 * Implementation of the Temperature Calibration for onboard accelerometer sensors.
 *
 * @author Siddharth Bharat Purohit
 * @author Paul Riseborough
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
#include <drivers/drv_accel.h>
#include <controllib/uorb/blocks.hpp>

#include <uORB/topics/sensor_accel.h>
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


class Tempcalaccel;

namespace tempcalaccel
{
Tempcalaccel *instance = nullptr;
}


class Tempcalaccel : public control::SuperBlock
{
public:
	/**
	 * Constructor
	 */
	Tempcalaccel();

	/**
	 * Destructor, also kills task.
	 */
	~Tempcalaccel();

	/**
	 * Start task.
	 *
	 * @return		OK on success.
	 */
	int		start();

	static void do_temperature_accel_calibration(int argc, char *argv[]);

	void		task_main();

	void print_status();

	void exit() { _force_task_exit = true; }

private:
	bool	_force_task_exit = false;
	int	_control_task = -1;		// task handle for task
};

Tempcalaccel::Tempcalaccel():
	SuperBlock(NULL, "Tempcalaccel")
{
}

Tempcalaccel::~Tempcalaccel()
{

}

void Tempcalaccel::task_main()
{
	// subscribe to relevant topics
	int accel_sub[SENSOR_COUNT_MAX];
	float accel_sample_filt[SENSOR_COUNT_MAX][4];
	polyfitter<4> P[SENSOR_COUNT_MAX][3];
	px4_pollfd_struct_t fds[SENSOR_COUNT_MAX] = {};
	unsigned hot_soak_sat[SENSOR_COUNT_MAX] = {};
	unsigned num_accel = orb_group_count(ORB_ID(sensor_accel));
	unsigned num_samples[SENSOR_COUNT_MAX] = {0};
	uint32_t device_ids[SENSOR_COUNT_MAX] = {};
	int param_set_result = PX4_OK;
	char param_str[30];
	int num_completed = 0; // number of completed sensors

	if (num_accel > SENSOR_COUNT_MAX) {
		num_accel = SENSOR_COUNT_MAX;
	}

	bool cold_soaked[SENSOR_COUNT_MAX] = {false};
	bool hot_soaked[SENSOR_COUNT_MAX] = {false};
	bool tempcal_complete[SENSOR_COUNT_MAX] = {false};
	float low_temp[SENSOR_COUNT_MAX];
	float high_temp[SENSOR_COUNT_MAX] = {0};
	float ref_temp[SENSOR_COUNT_MAX];

	for (unsigned i = 0; i < num_accel; i++) {
		accel_sub[i] = orb_subscribe_multi(ORB_ID(sensor_accel), i);
		fds[i].fd = accel_sub[i];
		fds[i].events = POLLIN;
	}

	// initialize data structures outside of loop
	// because they will else not always be
	// properly populated
	sensor_accel_s accel_data = {};

	/* reset all driver level calibrations */
	float offset = 0.0f;
	float scale = 1.0f;
	for (unsigned s = 0; s < num_accel; s++) {
		(void)sprintf(param_str, "CAL_ACC%u_XOFF", s);
		param_set_result = param_set_no_notification(param_find(param_str), &offset);
		if (param_set_result != PX4_OK) {
			PX4_ERR("unable to reset %s", param_str);
		}
		(void)sprintf(param_str, "CAL_ACC%u_YOFF", s);
		param_set_result = param_set_no_notification(param_find(param_str), &offset);
		if (param_set_result != PX4_OK) {
			PX4_ERR("unable to reset %s", param_str);
		}
		(void)sprintf(param_str, "CAL_ACC%u_ZOFF", s);
		param_set_result = param_set_no_notification(param_find(param_str), &offset);
		if (param_set_result != PX4_OK) {
			PX4_ERR("unable to reset %s", param_str);
		}
		(void)sprintf(param_str, "CAL_ACC%u_XSCALE", s);
		param_set_result = param_set_no_notification(param_find(param_str), &scale);
		if (param_set_result != PX4_OK) {
			PX4_ERR("unable to reset %s", param_str);
		}
		(void)sprintf(param_str, "CAL_ACC%u_YSCALE", s);
		param_set_result = param_set_no_notification(param_find(param_str), &scale);
		if (param_set_result != PX4_OK) {
			PX4_ERR("unable to reset %s", param_str);
		}
		(void)sprintf(param_str, "CAL_ACC%u_ZSCALE", s);
		param_set_result = param_set_no_notification(param_find(param_str), &scale);
		if (param_set_result != PX4_OK) {
			PX4_ERR("unable to reset %s", param_str);
		}
	}

	while (!_force_task_exit) {
		int ret = px4_poll(fds, num_accel, 1000);

		if (ret < 0) {
			// Poll error, sleep and try again
			usleep(10000);
			continue;

		} else if (ret == 0) {
			// Poll timeout or no new data, do nothing
			continue;
		}

		for (unsigned uorb_index = 0; uorb_index < num_accel; uorb_index++) {
			if (hot_soaked[uorb_index]) {
				continue;
			}

			if (fds[uorb_index].revents & POLLIN) {
				orb_copy(ORB_ID(sensor_accel), accel_sub[uorb_index], &accel_data);

				device_ids[uorb_index] = accel_data.device_id;

				accel_sample_filt[uorb_index][0] = accel_data.x;
				accel_sample_filt[uorb_index][1] = accel_data.y;
				accel_sample_filt[uorb_index][2] = accel_data.z;
				accel_sample_filt[uorb_index][3] = accel_data.temperature;

				if (!cold_soaked[uorb_index]) {
					cold_soaked[uorb_index] = true;
					low_temp[uorb_index] = accel_sample_filt[uorb_index][3];	//Record the low temperature
					ref_temp[uorb_index] = accel_sample_filt[uorb_index][3] + 12.0f;
				}

				num_samples[uorb_index]++;
			}
		}

		for (unsigned sensor_index = 0; sensor_index < num_accel; sensor_index++) {
			if (hot_soaked[sensor_index]) {
				continue;
			}

			if (accel_sample_filt[sensor_index][3] > high_temp[sensor_index]) {
				high_temp[sensor_index] = accel_sample_filt[sensor_index][3];
				hot_soak_sat[sensor_index] = 0;

			} else {
				continue;
			}

			//TODO: Hot Soak Saturation
			if (hot_soak_sat[sensor_index] == 10 || (high_temp[sensor_index] - low_temp[sensor_index]) > 24.0f) {
				hot_soaked[sensor_index] = true;
			}

			if (sensor_index == 0) {
				TC_DEBUG("\n%.20f,%.20f,%.20f,%.20f, %.6f, %.6f, %.6f\n\n", (double)accel_sample_filt[sensor_index][0],
					 (double)accel_sample_filt[sensor_index][1],
					 (double)accel_sample_filt[sensor_index][2], (double)accel_sample_filt[sensor_index][3], (double)low_temp[sensor_index], (double)high_temp[sensor_index],
					 (double)(high_temp[sensor_index] - low_temp[sensor_index]));
			}

			//update linear fit matrices
			accel_sample_filt[sensor_index][3] -= ref_temp[sensor_index];
			P[sensor_index][0].update((double)accel_sample_filt[sensor_index][3], (double)accel_sample_filt[sensor_index][0]);
			P[sensor_index][1].update((double)accel_sample_filt[sensor_index][3], (double)accel_sample_filt[sensor_index][1]);
			P[sensor_index][2].update((double)accel_sample_filt[sensor_index][3], (double)accel_sample_filt[sensor_index][2]);
			num_samples[sensor_index] = 0;
		}

		for (unsigned sensor_index = 0; sensor_index < num_accel; sensor_index++) {
			if (hot_soaked[sensor_index] && !tempcal_complete[sensor_index]) {
				double res[3][4] = {0.0f};
				P[sensor_index][0].fit(res[0]);
				res[0][3] = 0.0; // normalise the correction to be zero at the reference temperature
				PX4_WARN("Result Accel %u Axis 0: %.20f %.20f %.20f %.20f", sensor_index, (double)res[0][0], (double)res[0][1], (double)res[0][2],
					 (double)res[0][3]);
				P[sensor_index][1].fit(res[1]);
				res[1][3] = 0.0; // normalise the correction to be zero at the reference temperature
				PX4_WARN("Result Accel %u Axis 1: %.20f %.20f %.20f %.20f", sensor_index, (double)res[1][0], (double)res[1][1], (double)res[1][2],
					 (double)res[1][3]);
				P[sensor_index][2].fit(res[2]);
				res[2][3] = 0.0; // normalise the correction to be zero at the reference temperature
				PX4_WARN("Result Accel %u Axis 2: %.20f %.20f %.20f %.20f", sensor_index, (double)res[2][0], (double)res[2][1], (double)res[2][2],
					 (double)res[2][3]);
				tempcal_complete[sensor_index] = true;
				++num_completed;

				float param = 0.0f;

				sprintf(param_str, "TC_A%d_ID", sensor_index);
				param_set_result = param_set_no_notification(param_find(param_str), &device_ids[sensor_index]);

				if (param_set_result != PX4_OK) {
					PX4_ERR("unable to reset %s", param_str);
				}

				for (unsigned axis_index = 0; axis_index < 3; axis_index++) {
					for (unsigned coef_index = 0; coef_index <= 3; coef_index++) {
						sprintf(param_str, "TC_A%d_X%d_%d", sensor_index, (3-coef_index), axis_index);
						param = (float)res[axis_index][coef_index];
						param_set_result = param_set_no_notification(param_find(param_str), &param);

						if (param_set_result != PX4_OK) {
							PX4_ERR("unable to reset %s", param_str);
						}
					}

					sprintf(param_str, "TC_A%d_TMAX", sensor_index);
					param = high_temp[sensor_index];
					param_set_result = param_set_no_notification(param_find(param_str), &param);

					if (param_set_result != PX4_OK) {
						PX4_ERR("unable to reset %s", param_str);
					}

					sprintf(param_str, "TC_A%d_TMIN", sensor_index);
					param = low_temp[sensor_index];
					param_set_result = param_set_no_notification(param_find(param_str), &param);

					if (param_set_result != PX4_OK) {
						PX4_ERR("unable to reset %s", param_str);
					}

					sprintf(param_str, "TC_A%d_TREF", sensor_index);
					param = ref_temp[sensor_index];
					param_set_result = param_set_no_notification(param_find(param_str), &param);

					if (param_set_result != PX4_OK) {
						PX4_ERR("unable to reset %s", param_str);
					}
				}

			}
		}

		// Check if completed and enable use of the thermal compensation
		if (num_completed >= num_accel) {
			sprintf(param_str, "TC_A_ENABLE");
			int32_t enabled = 1;
			param_set_result = param_set(param_find(param_str), &enabled);

			if (param_set_result != PX4_OK) {
				PX4_ERR("unable to reset %s", param_str);
			}

			// exit the while loop
			break;

		}
	}

	for (unsigned i = 0; i < num_accel; i++) {
		orb_unsubscribe(accel_sub[i]);
	}

	delete tempcalaccel::instance;
	tempcalaccel::instance = nullptr;
	PX4_INFO("Tempcalaccel process stopped");
}

void Tempcalaccel::do_temperature_accel_calibration(int argc, char *argv[])
{
	tempcalaccel::instance->task_main();
}

int Tempcalaccel::start()
{

	ASSERT(_control_task == -1);
	_control_task = px4_task_spawn_cmd("accel_temp_calib",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   5800,
					   (px4_main_t)&Tempcalaccel::do_temperature_accel_calibration,
					   nullptr);

	if (_control_task < 0) {
		delete tempcalaccel::instance;
		tempcalaccel::instance = nullptr;
		PX4_ERR("start failed");
		return -errno;
	}

	return 0;
}

int run_temperature_accel_calibration()
{
	PX4_INFO("Starting accel thermal calibration task");
	tempcalaccel::instance = new Tempcalaccel();

	if (tempcalaccel::instance == nullptr) {
		PX4_ERR("alloc failed");
		return 1;
	}

	return tempcalaccel::instance->start();
}
