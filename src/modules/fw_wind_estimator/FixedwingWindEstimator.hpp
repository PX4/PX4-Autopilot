/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#include <lib/rate_control/rate_control.hpp>

#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <lib/slew_rate/SlewRate.hpp>
#include <matrix/math.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/airflow.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_local_position.h>
#include <geo/geo.h> // CONSTANTS_ONE_G

using uORB::SubscriptionData;

using namespace time_literals;

class FixedwingWindEstimator final : public ModuleBase<FixedwingWindEstimator>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	FixedwingWindEstimator();
	~FixedwingWindEstimator() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};     	// linear acceleration
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_rates_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _airspeed_validated_sub{ORB_ID(airspeed_validated)};

	uORB::Publication<airflow_s> _airflow_pub{ORB_ID(airflow)};

	vehicle_control_mode_s			_vcontrol_mode{};
	vehicle_status_s			_vehicle_status{};

	perf_counter_t _loop_perf;

	hrt_abstime _last_run{0};

	float _calibrated_airspeed{0.0f};
	float _true_airspeed{15.0f};
	matrix::Quatf _attitude{};
	matrix::Vector3f _acceleration{0.f, 0.f, 0.f};
	matrix::Vector3f _local_velocity{0.f, 0.f, 0.f};

	bool _landed{true};

	//Vehicle parameters


	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::FW_W_MASS>) _param_fw_w_mass,
		(ParamFloat<px4::params::FW_W_AREA>) _param_fw_w_area,
		(ParamFloat<px4::params::FW_W_C_B1>) _param_fw_w_c_b1,
		(ParamFloat<px4::params::FW_W_C_A0>) _param_fw_w_c_a0,
		(ParamFloat<px4::params::FW_W_C_A1>) _param_fw_w_c_a1,
		(ParamFloat<px4::params::FW_AIRSPD_STALL>) _param_fw_airspd_stall
	)

	/**
	 * Update our local parameter cache.
	 */
	int parameters_update();
	void vehicle_land_detected_poll();

	void airspeed_poll();
	void vehicle_acceleration_poll();
	void vehicle_attitude_poll();
	void vehicle_local_position_poll();
	matrix::Vector3f predictBodyAirVelocity();
};
