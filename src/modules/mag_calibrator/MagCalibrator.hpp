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

#pragma once

#include "TRICAL.h"

#include <lib/sensor_calibration/Magnetometer.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/estimator_mag_cal.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_gps_position.h>

class MagCalibrator : public ModuleBase<MagCalibrator>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	MagCalibrator();
	~MagCalibrator() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:
	void Run() override;

	static constexpr int MAX_SENSOR_COUNT = 4;

	TRICAL_instance_t _trical_instance[MAX_SENSOR_COUNT];

	bool _initialized[MAX_SENSOR_COUNT] {};

	bool _mag_earth_available{false};

	float _mag_declination_gps{0.f}; // magnetic declination returned by the geo library using the last valid GPS position (rad)
	float _mag_inclination_gps{0.f}; // magnetic inclination returned by the geo library using the last valid GPS position (rad)
	float _mag_strength_gps{0.f};    // magnetic strength returned by the geo library using the last valid GPS position (T)

	matrix::Vector3f _mag_earth_pred{};

	calibration::Magnetometer _calibration[MAX_SENSOR_COUNT];

	uORB::PublicationMulti<estimator_mag_cal_s> _estimator_mag_cal_pub[MAX_SENSOR_COUNT] {
		ORB_ID(estimator_mag_cal), ORB_ID(estimator_mag_cal), ORB_ID(estimator_mag_cal), ORB_ID(estimator_mag_cal)
	};

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};

	uORB::SubscriptionMultiArray<sensor_mag_s, MAX_SENSOR_COUNT> _sensor_mag_subs{ORB_ID::sensor_mag};

	perf_counter_t	_loop_perf;			/**< loop duration performance counter */
};
