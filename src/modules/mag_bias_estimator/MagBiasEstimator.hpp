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

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/field_sensor_bias_estimator/FieldSensorBiasEstimator.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <lib/sensor_calibration/Magnetometer.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/magnetometer_bias_estimate.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_status_flags.h>

namespace mag_bias_estimator
{

class MagBiasEstimator : public ModuleBase<MagBiasEstimator>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	MagBiasEstimator();
	~MagBiasEstimator() override;

	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[])
	{
		return print_usage("unknown command");
	}

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	void start();

private:
	void Run() override;
	void publishMagBiasEstimate();

	static constexpr int MAX_SENSOR_COUNT = 4;

	FieldSensorBiasEstimator _bias_estimator[MAX_SENSOR_COUNT];
	hrt_abstime _timestamp_last_update[MAX_SENSOR_COUNT] {};

	uORB::SubscriptionMultiArray<sensor_mag_s, MAX_SENSOR_COUNT> _sensor_mag_subs{ORB_ID::sensor_mag};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_status_flags_sub{ORB_ID(vehicle_status_flags)};

	uORB::Publication<magnetometer_bias_estimate_s> _magnetometer_bias_estimate_pub{ORB_ID(magnetometer_bias_estimate)};

	calibration::Magnetometer _calibration[MAX_SENSOR_COUNT];

	hrt_abstime _time_valid[MAX_SENSOR_COUNT] {};

	bool _reset_field_estimator[MAX_SENSOR_COUNT] {};
	bool _valid[MAX_SENSOR_COUNT] {};

	uint8_t _arming_state{0};
	bool _system_calibrating{false};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MBE_LEARN_GAIN>) _param_mbe_learn_gain
	)
};

} // namespace mag_bias_estimator
