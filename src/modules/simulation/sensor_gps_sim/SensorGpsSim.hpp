/****************************************************************************
 *
 *   Copyright (c) 2021-2026 PX4 Development Team. All rights reserved.
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

#include <GaussianNoise.hpp>
#include <lib/failure_injection/FailureInjection.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/failure_injection.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>

using namespace time_literals;

class SensorGpsSim : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	SensorGpsSim();
	~SensorGpsSim() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	static constexpr int GPS_MAX_INSTANCES = 2;

	void Run() override;

	void updateFailureConfig();

	void publishWithFailures(int instance, sensor_gps_s gps, sensor_gps_s &snapshot,
				 uORB::PublicationMulti<sensor_gps_s> &pub);

	// instance is 0-based here; the failure_injection topic addresses 1-based instances.
	failure_injection::Mode failureMode(int instance) const
	{
		return _failure_config.mode(failure_injection_s::FAILURE_UNIT_SENSOR_GPS, instance + 1);
	}
	bool isBlocked(int instance) const { return failureMode(instance) == failure_injection::Mode::Off; }
	bool isStuck(int instance)   const { return failureMode(instance) == failure_injection::Mode::Stuck; }
	bool isWrong(int instance)   const { return failureMode(instance) == failure_injection::Mode::Wrong; }


	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _vehicle_global_position_sub{ORB_ID(vehicle_global_position_groundtruth)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position_groundtruth)};

	uORB::PublicationMulti<sensor_gps_s> _sensor_gps_pub{ORB_ID(sensor_gps)};
	uORB::PublicationMulti<sensor_gps_s> _sensor_gps_pub2{ORB_ID(sensor_gps)};

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	failure_injection::Config _failure_config;

	sensor_gps_s _last_gps0{};
	sensor_gps_s _last_gps1{};

	// GPS Markov process noise state
	float _gps_pos_noise_n{0.0f};
	float _gps_pos_noise_e{0.0f};
	float _gps_pos_noise_d{0.0f};
	float _gps_vel_noise_n{0.0f};
	float _gps_vel_noise_e{0.0f};
	float _gps_vel_noise_d{0.0f};

	// Gauss-Markov noise parameters, rate-corrected from GZBridge (30 Hz) to SIH (8 Hz)
	static constexpr float _pos_noise_amplitude{0.8f};
	static constexpr float _pos_random_walk{0.02f};
	static constexpr float _pos_markov_time{0.76f};
	static constexpr float _vel_noise_amplitude{0.05f};
	static constexpr float _vel_noise_density{0.4f};
	static constexpr float _vel_markov_time{0.54f};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SIM_GPS_USED>)      _sim_gps_used,
		(ParamFloat<px4::params::SENS_GPS1_OFFX>)  _param_gps1_offx,
		(ParamFloat<px4::params::SENS_GPS1_OFFY>)  _param_gps1_offy
	)
};
