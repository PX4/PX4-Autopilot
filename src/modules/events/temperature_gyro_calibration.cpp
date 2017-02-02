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
 * @file temperature_gyro_calibration.cpp
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

extern "C" __EXPORT int tempcalgyro_main(int argc, char *argv[]);


class Tempcalgyro;

namespace tempcalgyro
{
Tempcalgyro *instance = nullptr;
}


class Tempcalgyro : public control::SuperBlock
{
public:
	/**
	 * Constructor
	 */
	Tempcalgyro();

	/**
	 * Destructor, also kills task.
	 */
	~Tempcalgyro();

	/**
	 * Start task.
	 *
	 * @return		OK on success.
	 */
	int		start();

	static void do_temperature_calibration(int argc, char *argv[]);

	void		task_main();

	void print_status();

	void exit() { _force_task_exit = true; }

private:
	bool	_force_task_exit = false;
	int	_control_task = -1;		// task handle for task
};

Tempcalgyro::Tempcalgyro():
	SuperBlock(NULL, "Tempcalgyro")
{
}

Tempcalgyro::~Tempcalgyro()
{

}

void Tempcalgyro::task_main()
{
	// subscribe to relevant topics
	int gyro_sub[SENSOR_COUNT_MAX];
	float gyro_sample_filt[SENSOR_COUNT_MAX][4];
	polyfitter<4> P[SENSOR_COUNT_MAX][3];
	px4_pollfd_struct_t fds[SENSOR_COUNT_MAX] = {};
	unsigned hot_soak_sat[SENSOR_COUNT_MAX] = {};
	unsigned num_gyro = orb_group_count(ORB_ID(sensor_gyro));
	unsigned num_samples[SENSOR_COUNT_MAX] = {0};
	uint32_t device_ids[SENSOR_COUNT_MAX] = {};
	int param_set_result;
	char param_str[30];
	int num_completed = 0; // number of completed gyros

	if (num_gyro > SENSOR_COUNT_MAX) {
		num_gyro = SENSOR_COUNT_MAX;
	}

	bool cold_soaked[SENSOR_COUNT_MAX] = {false};
	bool hot_soaked[SENSOR_COUNT_MAX] = {false};
	bool tempcal_complete[SENSOR_COUNT_MAX] = {false};
	float low_temp[SENSOR_COUNT_MAX];
	float high_temp[SENSOR_COUNT_MAX] = {0};
	float ref_temp[SENSOR_COUNT_MAX];

	for (unsigned i = 0; i < num_gyro; i++) {
		gyro_sub[i] = orb_subscribe_multi(ORB_ID(sensor_gyro), i);
		fds[i].fd = gyro_sub[i];
		fds[i].events = POLLIN;
	}

	// initialize data structures outside of loop
	// because they will else not always be
	// properly populated
	sensor_gyro_s gyro_data = {};

	/* reset all driver level calibrations */
	float offset = 0.0f;
	float scale = 1.0f;
	for (unsigned s = 0; s < num_gyro; s++) {
		(void)sprintf(param_str, "CAL_GYRO%u_XOFF", s);
		param_set_result = param_set_no_notification(param_find(param_str), &offset);
		if (param_set_result != PX4_OK) {
			PX4_ERR("unable to reset %s", param_str);
		}
		(void)sprintf(param_str, "CAL_GYRO%u_YOFF", s);
		param_set_result = param_set_no_notification(param_find(param_str), &offset);
		if (param_set_result != PX4_OK) {
			PX4_ERR("unable to reset %s", param_str);
		}
		(void)sprintf(param_str, "CAL_GYRO%u_ZOFF", s);
		param_set_result = param_set_no_notification(param_find(param_str), &offset);
		if (param_set_result != PX4_OK) {
			PX4_ERR("unable to reset %s", param_str);
		}
		(void)sprintf(param_str, "CAL_GYRO%u_XSCALE", s);
		param_set_result = param_set_no_notification(param_find(param_str), &scale);
		if (param_set_result != PX4_OK) {
			PX4_ERR("unable to reset %s", param_str);
		}
		(void)sprintf(param_str, "CAL_GYRO%u_YSCALE", s);
		param_set_result = param_set_no_notification(param_find(param_str), &scale);
		if (param_set_result != PX4_OK) {
			PX4_ERR("unable to reset %s", param_str);
		}
		(void)sprintf(param_str, "CAL_GYRO%u_ZSCALE", s);
		param_set_result = param_set_no_notification(param_find(param_str), &scale);
		if (param_set_result != PX4_OK) {
			PX4_ERR("unable to reset %s", param_str);
		}
	}

	int32_t min_temp_rise = 24;
	param_get(param_find("SYS_CAL_TEMP"), &min_temp_rise);

	while (!_force_task_exit) {
		int ret = px4_poll(fds, num_gyro, 1000);

		if (ret < 0) {
			// Poll error, sleep and try again
			usleep(10000);
			continue;

		} else if (ret == 0) {
			// Poll timeout or no new data, do nothing
			continue;
		}

		for (unsigned uorb_index = 0; uorb_index < num_gyro; uorb_index++) {
			if (hot_soaked[uorb_index]) {
				continue;
			}

			if (fds[uorb_index].revents & POLLIN) {
				orb_copy(ORB_ID(sensor_gyro), gyro_sub[uorb_index], &gyro_data);

				device_ids[uorb_index] = gyro_data.device_id;

				gyro_sample_filt[uorb_index][0] = gyro_data.x;
				gyro_sample_filt[uorb_index][1] = gyro_data.y;
				gyro_sample_filt[uorb_index][2] = gyro_data.z;
				gyro_sample_filt[uorb_index][3] = gyro_data.temperature;

				if (!cold_soaked[uorb_index]) {
					cold_soaked[uorb_index] = true;
					low_temp[uorb_index] = gyro_sample_filt[uorb_index][3];	//Record the low temperature
					ref_temp[uorb_index] = gyro_sample_filt[uorb_index][3] + 0.5f * (float)min_temp_rise;
				}

				num_samples[uorb_index]++;
			}
		}

		for (unsigned sensor_index = 0; sensor_index < num_gyro; sensor_index++) {
			if (hot_soaked[sensor_index]) {
				continue;
			}

			if (gyro_sample_filt[sensor_index][3] > high_temp[sensor_index]) {
				high_temp[sensor_index] = gyro_sample_filt[sensor_index][3];
				hot_soak_sat[sensor_index] = 0;

			} else {
				continue;
			}

			//TODO: Detect when temperature has stopped rising for more than TBD seconds
			if (hot_soak_sat[sensor_index] == 10 || (high_temp[sensor_index] - low_temp[sensor_index]) > (float)min_temp_rise) {
				hot_soaked[sensor_index] = true;
			}

			if (sensor_index == 0) {
				TC_DEBUG("\n%.20f,%.20f,%.20f,%.20f, %.6f, %.6f, %.6f\n\n", (double)gyro_sample_filt[sensor_index][0],
					 (double)gyro_sample_filt[sensor_index][1],
					 (double)gyro_sample_filt[sensor_index][2], (double)gyro_sample_filt[sensor_index][3], (double)low_temp[sensor_index], (double)high_temp[sensor_index],
					 (double)(high_temp[sensor_index] - low_temp[sensor_index]));
			}

			//update linear fit matrices
			gyro_sample_filt[sensor_index][3] -= ref_temp[sensor_index];
			P[sensor_index][0].update((double)gyro_sample_filt[sensor_index][3], (double)gyro_sample_filt[sensor_index][0]);
			P[sensor_index][1].update((double)gyro_sample_filt[sensor_index][3], (double)gyro_sample_filt[sensor_index][1]);
			P[sensor_index][2].update((double)gyro_sample_filt[sensor_index][3], (double)gyro_sample_filt[sensor_index][2]);
			num_samples[sensor_index] = 0;
		}

		for (unsigned sensor_index = 0; sensor_index < num_gyro; sensor_index++) {
			if (hot_soaked[sensor_index] && !tempcal_complete[sensor_index]) {
				double res[3][4] = {0.0f};
				P[sensor_index][0].fit(res[0]);
				PX4_WARN("Result Gyro %d Axis 0: %.20f %.20f %.20f %.20f", sensor_index, (double)res[0][0], (double)res[0][1], (double)res[0][2],
					 (double)res[0][3]);
				P[sensor_index][1].fit(res[1]);
				PX4_WARN("Result Gyro %d Axis 1: %.20f %.20f %.20f %.20f", sensor_index, (double)res[1][0], (double)res[1][1], (double)res[1][2],
					 (double)res[1][3]);
				P[sensor_index][2].fit(res[2]);
				PX4_WARN("Result Gyro %d Axis 2: %.20f %.20f %.20f %.20f", sensor_index, (double)res[2][0], (double)res[2][1], (double)res[2][2],
					 (double)res[2][3]);
				tempcal_complete[sensor_index] = true;
				++num_completed;

				char str[30];
				float param = 0.0f;
				int result = PX4_OK;

				sprintf(str, "TC_G%d_ID", sensor_index);
				result = param_set(param_find(str), &device_ids[sensor_index]);

				if (result != PX4_OK) {
					PX4_ERR("unable to reset %s", str);
				}

				for (unsigned axis_index = 0; axis_index < 3; axis_index++) {
					for (unsigned coef_index = 0; coef_index <= 3; coef_index++) {
						sprintf(str, "TC_G%d_X%d_%d", sensor_index, (3-coef_index), axis_index);
						param = (float)res[axis_index][coef_index];
						result = param_set_no_notification(param_find(str), &param);

						if (result != PX4_OK) {
							PX4_ERR("unable to reset %s", str);
						}
					}

					sprintf(str, "TC_G%d_TMAX", sensor_index);
					param = high_temp[sensor_index];
					result = param_set_no_notification(param_find(str), &param);

					if (result != PX4_OK) {
						PX4_ERR("unable to reset %s", str);
					}

					sprintf(str, "TC_G%d_TMIN", sensor_index);
					param = low_temp[sensor_index];
					result = param_set_no_notification(param_find(str), &param);

					if (result != PX4_OK) {
						PX4_ERR("unable to reset %s", str);
					}

					sprintf(str, "TC_G%d_TREF", sensor_index);
					param = ref_temp[sensor_index];
					result = param_set_no_notification(param_find(str), &param);

					if (result != PX4_OK) {
						PX4_ERR("unable to reset %s", str);
					}
				}

			}
		}

		// Check if completed and enable use of the thermal compensation
		if (num_completed >= num_gyro) {
			sprintf(param_str, "TC_G_ENABLE");
			int32_t enabled = 1;
			param_set_result = param_set(param_find(param_str), &enabled);

			if (param_set_result != PX4_OK) {
				PX4_ERR("unable to reset %s", param_str);
			}

			break;

		}
	}

	for (unsigned i = 0; i < num_gyro; i++) {
		orb_unsubscribe(gyro_sub[i]);
	}

	delete tempcalgyro::instance;
	tempcalgyro::instance = nullptr;
	PX4_INFO("Tempcalgyro process exited");
}

void Tempcalgyro::do_temperature_calibration(int argc, char *argv[])
{
	tempcalgyro::instance->task_main();
}

int Tempcalgyro::start()
{

	ASSERT(_control_task == -1);
	_control_task = px4_task_spawn_cmd("temperature_calib",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   5800,
					   (px4_main_t)&Tempcalgyro::do_temperature_calibration,
					   nullptr);

	if (_control_task < 0) {
		delete tempcalgyro::instance;
		tempcalgyro::instance = nullptr;
		PX4_ERR("start failed");
		return -errno;
	}

	return 0;
}

int run_temperature_gyro_calibration()
{
	PX4_INFO("Starting Temperature calibration task");
	tempcalgyro::instance = new Tempcalgyro();

	if (tempcalgyro::instance == nullptr) {
		PX4_ERR("alloc failed");
		return 1;
	}

	return tempcalgyro::instance->start();
}
