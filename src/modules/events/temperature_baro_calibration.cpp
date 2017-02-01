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
 * @file temperature_baro_calibration.cpp
 * Implementation of the Temperature Calibration for onboard baroerometer sensors.
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
#include <drivers/drv_baro.h>
#include <controllib/uorb/blocks.hpp>

#include <uORB/topics/sensor_baro.h>
#include "polyfit.hpp"
#include "temperature_calibration.h"

#define TC_PRINT_DEBUG 0
#if TC_PRINT_DEBUG
#define TC_DEBUG(fmt, ...) printf(fmt, ##__VA_ARGS__);
#else
#define TC_DEBUG(fmt, ...)
#endif


#define SENSOR_COUNT_MAX		3
#define POLYFIT_ORDER			5

extern "C" __EXPORT int tempcal_main(int argc, char *argv[]);


class Tempcalbaro;

namespace tempcalbaro
{
Tempcalbaro *instance = nullptr;
}


class Tempcalbaro : public control::SuperBlock
{
public:
	/**
	 * Constructor
	 */
	Tempcalbaro();

	/**
	 * Destructor, also kills task.
	 */
	~Tempcalbaro();

	/**
	 * Start task.
	 *
	 * @return		OK on success.
	 */
	int		start();

	static void do_temperature_baro_calibration(int argc, char *argv[]);

	void		task_main();

	void print_status();

	void exit() { _force_task_exit = true; }

private:
	bool	_force_task_exit = false;
	int	_control_task = -1;		// task handle for task
};

Tempcalbaro::Tempcalbaro():
	SuperBlock(NULL, "Tempcalbaro")
{
}

Tempcalbaro::~Tempcalbaro()
{

}

void Tempcalbaro::task_main()
{
	// subscribe to relevant topics
	int baro_sub[SENSOR_COUNT_MAX];
	float baro_sample_filt[SENSOR_COUNT_MAX][2];
	polyfitter<POLYFIT_ORDER+1> P[SENSOR_COUNT_MAX];
	px4_pollfd_struct_t fds[SENSOR_COUNT_MAX] = {};
	unsigned hot_soak_sat[SENSOR_COUNT_MAX] = {};
	unsigned num_baro = orb_group_count(ORB_ID(sensor_baro));
	unsigned num_samples[SENSOR_COUNT_MAX] = {0};
	uint32_t device_ids[SENSOR_COUNT_MAX] = {};

	if (num_baro > SENSOR_COUNT_MAX) {
		num_baro = SENSOR_COUNT_MAX;
	}

	bool cold_soaked[SENSOR_COUNT_MAX] = {false};
	bool hot_soaked[SENSOR_COUNT_MAX] = {false};
	bool tempcal_complete[SENSOR_COUNT_MAX] = {false};
	float low_temp[SENSOR_COUNT_MAX];
	float high_temp[SENSOR_COUNT_MAX] = {0};
	float ref_temp[SENSOR_COUNT_MAX];

	for (unsigned i = 0; i < num_baro; i++) {
		baro_sub[i] = orb_subscribe_multi(ORB_ID(sensor_baro), i);
		fds[i].fd = baro_sub[i];
		fds[i].events = POLLIN;
	}

	// initialize data structures outside of loop
	// because they will else not always be
	// properly populated
	sensor_baro_s baro_data = {};

	int param_set_result;
	char param_str[30];
	int num_completed = 0; // number of completed sensors

	while (!_force_task_exit) {
		int ret = px4_poll(fds, num_baro, 1000);

		if (ret < 0) {
			// Poll error, sleep and try again
			usleep(10000);
			continue;

		} else if (ret == 0) {
			// Poll timeout or no new data, do nothing
			continue;
		}

		for (unsigned uorb_index = 0; uorb_index < num_baro; uorb_index++) {
			if (hot_soaked[uorb_index]) {
				continue;
			}

			if (fds[uorb_index].revents & POLLIN) {
				orb_copy(ORB_ID(sensor_baro), baro_sub[uorb_index], &baro_data);

				device_ids[uorb_index] = baro_data.device_id;

				baro_sample_filt[uorb_index][0] = 100.0f * baro_data.pressure; // convert from hPA to Pa
				baro_sample_filt[uorb_index][1] = baro_data.temperature;

				if (!cold_soaked[uorb_index]) {
					cold_soaked[uorb_index] = true;
					low_temp[uorb_index] = baro_sample_filt[uorb_index][1];	//Record the low temperature
					ref_temp[uorb_index] = baro_sample_filt[uorb_index][1] + 12.0f;
				}

				num_samples[uorb_index]++;
			}
		}

		for (unsigned sensor_index = 0; sensor_index < num_baro; sensor_index++) {
			if (hot_soaked[sensor_index]) {
				continue;
			}

			if (baro_sample_filt[sensor_index][1] > high_temp[sensor_index]) {
				high_temp[sensor_index] = baro_sample_filt[sensor_index][1];
				hot_soak_sat[sensor_index] = 0;

			} else {
				continue;
			}

			//TODO: Hot Soak Saturation
			if (hot_soak_sat[sensor_index] == 10 || (high_temp[sensor_index] - low_temp[sensor_index]) > 24.0f) {
				hot_soaked[sensor_index] = true;
			}

			if (sensor_index == 0) {
				TC_DEBUG("\n%.20f,%.20f,%.20f,%.20f, %.6f, %.6f, %.6f\n\n", (double)baro_sample_filt[sensor_index][0],
					 (double)baro_sample_filt[sensor_index][1], (double)low_temp[sensor_index], (double)high_temp[sensor_index],
					 (double)(high_temp[sensor_index] - low_temp[sensor_index]));
			}

			//update linear fit matrices
			baro_sample_filt[sensor_index][1] -= ref_temp[sensor_index];
			P[sensor_index].update((double)baro_sample_filt[sensor_index][1], (double)baro_sample_filt[sensor_index][0]);
			num_samples[sensor_index] = 0;
		}

		for (unsigned sensor_index = 0; sensor_index < num_baro; sensor_index++) {
			if (hot_soaked[sensor_index] && !tempcal_complete[sensor_index]) {
				double res[POLYFIT_ORDER+1] = {0.0f};
				P[sensor_index].fit(res);
				res[POLYFIT_ORDER] = 0.0; // normalise the correction to be zero at the reference temperature by setting the X^0 coefficient to zero
				PX4_WARN("Result baro %u %.20f %.20f %.20f %.20f %.20f %.20f", sensor_index, (double)res[0], (double)res[1], (double)res[2], (double)res[3], (double)res[4], (double)res[5]);
				tempcal_complete[sensor_index] = true;
				++num_completed;

				float param_val = 0.0f;

				sprintf(param_str, "TC_B%d_ID", sensor_index);
				param_set_result = param_set_no_notification(param_find(param_str), &device_ids[sensor_index]);

				if (param_set_result != PX4_OK) {
					PX4_ERR("unable to reset %s", param_str);
				}

				for (unsigned coef_index = 0; coef_index <= POLYFIT_ORDER; coef_index++) {
					sprintf(param_str, "TC_B%d_X%d", sensor_index, (POLYFIT_ORDER - coef_index));
					param_val = (float)res[coef_index];
					param_set_result = param_set_no_notification(param_find(param_str), &param_val);

					if (param_set_result != PX4_OK) {
						PX4_ERR("unable to reset %s", param_str);
					}
				}

				sprintf(param_str, "TC_B%d_TMAX", sensor_index);
				param_val = high_temp[sensor_index];
				param_set_result = param_set_no_notification(param_find(param_str), &param_val);

				if (param_set_result != PX4_OK) {
					PX4_ERR("unable to reset %s", param_str);
				}

				sprintf(param_str, "TC_B%d_TMIN", sensor_index);
				param_val = low_temp[sensor_index];
				param_set_result = param_set_no_notification(param_find(param_str), &param_val);

				if (param_set_result != PX4_OK) {
					PX4_ERR("unable to reset %s", param_str);
				}

				sprintf(param_str, "TC_B%d_TREF", sensor_index);
				param_val = ref_temp[sensor_index];
				param_set_result = param_set_no_notification(param_find(param_str), &param_val);

				if (param_set_result != PX4_OK) {
					PX4_ERR("unable to reset %s", param_str);
				}
			}
		}

		// Check if completed and enable use of the thermal compensation
		if (num_completed >= num_baro) {
			sprintf(param_str, "TC_B_ENABLE");
			int32_t enabled = 1;
			param_set_result = param_set(param_find(param_str), &enabled);

			if (param_set_result != PX4_OK) {
				PX4_ERR("unable to reset %s", param_str);
			}

			break;

		}
	}

	for (unsigned i = 0; i < num_baro; i++) {
		orb_unsubscribe(baro_sub[i]);
	}

	delete tempcalbaro::instance;
	tempcalbaro::instance = nullptr;
	PX4_INFO("Tempcalbaro process stopped");
}

void Tempcalbaro::do_temperature_baro_calibration(int argc, char *argv[])
{
	tempcalbaro::instance->task_main();
}

int Tempcalbaro::start()
{

	ASSERT(_control_task == -1);
	_control_task = px4_task_spawn_cmd("baro_temp_calib",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   5800,
					   (px4_main_t)&Tempcalbaro::do_temperature_baro_calibration,
					   nullptr);

	if (_control_task < 0) {
		delete tempcalbaro::instance;
		tempcalbaro::instance = nullptr;
		PX4_ERR("start failed");
		return -errno;
	}

	return 0;
}

int run_temperature_baro_calibration()
{
	PX4_INFO("Starting baro thermal calibration task");
	tempcalbaro::instance = new Tempcalbaro();

	if (tempcalbaro::instance == nullptr) {
		PX4_ERR("alloc failed");
		return 1;
	}

	return tempcalbaro::instance->start();
}
