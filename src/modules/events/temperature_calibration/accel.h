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

#pragma once

#include "common.h"
#include "polyfit.hpp"

class TemperatureCalibrationAccel : public TemperatureCalibrationBase
{
public:
	TemperatureCalibrationAccel(float min_temperature_rise);
	virtual ~TemperatureCalibrationAccel();

	/**
	 * @see TemperatureCalibrationBase::update()
	 */
	int update();

	/**
	 * @see TemperatureCalibrationBase::finish()
	 */
	int finish();

	/**
	 * @see TemperatureCalibrationBase::reset_calibration()
	 */
	void reset_calibration();

private:

	struct PerSensorData {
		float sensor_sample_filt[4];
		polyfitter<4> P[3];
		unsigned hot_soak_sat = 0;
		uint32_t device_id = 0;
		bool cold_soaked = false;
		bool hot_soaked = false;
		bool tempcal_complete = false;
		float low_temp = 0.f;
		float high_temp = 0.f;
		float ref_temp = 0.f;
	};

	PerSensorData _data[SENSOR_COUNT_MAX];

	/**
	 * update a single sensor instance
	 * @return 0 when done, 1 not finished yet
	 */
	inline int update_sensor_instance(PerSensorData &data, int sensor_sub);

	inline int finish_sensor_instance(PerSensorData &data, int sensor_index);

	int _num_sensor_instances;
	int _sensor_subs[SENSOR_COUNT_MAX];
};
