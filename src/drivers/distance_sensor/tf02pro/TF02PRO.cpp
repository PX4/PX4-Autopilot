/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include "TF02PRO.hpp"

/**
 * @brief Construct a new TF02PRO::TF02PRO object
 *
 * @param config
 */
TF02PRO::TF02PRO(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_px4_rangefinder(get_device_id(), config.rotation)
{
	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_TF02PRO);
	_px4_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_LASER);
	_px4_rangefinder.set_max_distance(TF02PRO_MAX_DISTANCE);
	_px4_rangefinder.set_min_distance(TF02PRO_MIN_DISTANCE);
	_px4_rangefinder.set_fov(math::radians(3.0f));
}

/**
 * @brief Destroy the TF02PRO::TF02PRO object
 *
 */
TF02PRO::~TF02PRO()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

/**
 * @brief
 *
 */
void TF02PRO::start()
{
	_collect_phase = false;

	ScheduleDelayed(5);  // 5 us
}

/**
 * @brief
 *
 * @return int
 */
int TF02PRO::init()
{
	if (I2C::init() != OK) {
		return PX4_ERROR;
	}

	px4_usleep(100000);

	int ret = measure();

	if (ret == PX4_OK) {
		start();
	}

	return ret;
}

/**
 * @brief
 *
 * @return int
 * @Note
 *   Receive Frame
 *   Byte0: 0x59, frame header, same for each frame
 *   Byte1: 0x59, frame header, same for each frame
 *   Byte2: Dist_L distance value low 8 bits
 *   Byte3: Dist_H distance value high 8 bits
 *   Byte4: Strength_L low 8 bits
 *   Byte5: Strength_H high 8 bits
 *   Byte6: Temp_L low 8 bits
 *   Byte7: Temp_H high 8 bits
 *   Byte8: Checksum is the lower 8 bits of the cumulative sum of the number of the first 8 bytes
 *
 */
int TF02PRO::collect()
{
	uint8_t recv_data[9] {};
	perf_begin(_sample_perf);

	const hrt_abstime timestamp_sample = hrt_absolute_time();
	int ret = transfer(nullptr, 0, recv_data, sizeof(recv_data));

	if (ret < 0) {
		PX4_DEBUG("error reading from sensor: %d", ret);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	uint16_t strength = recv_data[5] << 8 | recv_data[4];
	uint16_t distance_mm = recv_data[3] << 8 | recv_data[2];

	if (strength >= 60u && distance_mm < 45000u) {
		float distance_m = float(distance_mm) * 1e-3f;

		_px4_rangefinder.update(timestamp_sample, distance_m);
	}

	perf_end(_sample_perf);
	return PX4_OK;
}

/**
 * @brief
 *
 * @return int
 */
int TF02PRO::measure()
{
	uint8_t obtain_Data_mm[5] = {0x5A, 0x05, 0x00, 0x06, 0x65};

	int ret = transfer(obtain_Data_mm, sizeof(obtain_Data_mm), nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		PX4_DEBUG("i2c::transfer returned %d", ret);
		return ret;
	}

	return PX4_OK;
}

/**
 * @brief
 *
 */
void TF02PRO::RunImpl()
{
	if (_collect_phase) {
		if (OK != collect()) {
			PX4_DEBUG("collection error");
			start();
			return;
		}

		_collect_phase = false;
	}

	if (OK != measure()) {
		PX4_DEBUG("measure error I2C adress");
	}

	_collect_phase = true;

	ScheduleDelayed(_interval);
}

/**
 * @brief
 *
 */
void TF02PRO::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
