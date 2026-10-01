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

#include <lib/failure_injection/FailureInjection.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/failure_injection.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rtcm_data.h>
#include <uORB/topics/sensor_gnss.h>
#include <uORB/topics/sensor_gnss_relative.h>
#include <uORB/topics/vehicle_attitude.h>
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

	// Each receiver has its own Gauss-Markov error, so that two receivers disagree by more than their biases
	struct ReceiverNoise {
		matrix::Vector3f position{};
		matrix::Vector3f velocity{};
	};

	void Run() override;

	void updateFailureConfig();

	// True while rtcm_corrections messages keep arriving (stale after RTCM_TIMEOUT, like the gps driver).
	bool updateRtcmCorrections();

	void publishWithFailures(int instance, sensor_gnss_s gnss);

	// Dual antenna or moving base rover heading of the receiver, as configured by SENS_GNSSn_HDG
	void publishRelativeHeading(int instance, const sensor_gnss_s &gnss, const matrix::Dcmf &body_to_ned);

	// Antenna position (SENS_GNSSn_OFFX/Y/Z) of the receiver
	matrix::Vector3f antennaOffset(int instance) const;

	// generate white Gaussian noise sample with std=1
	static float generate_wgn();

	// generate white Gaussian noise sample as a 3D vector with specified std
	matrix::Vector3f noiseGauss3f(float stdx, float stdy, float stdz) { return matrix::Vector3f(generate_wgn() * stdx, generate_wgn() * stdy, generate_wgn() * stdz); }

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude_groundtruth)};
	uORB::Subscription _vehicle_global_position_sub{ORB_ID(vehicle_global_position_groundtruth)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position_groundtruth)};
	uORB::SubscriptionMultiArray<rtcm_data_s, rtcm_data_s::MAX_INSTANCES> _rtcm_corrections_sub{ORB_ID::rtcm_corrections};

	uORB::PublicationMulti<sensor_gnss_s> _sensor_gnss_pub[GPS_MAX_INSTANCES] {{ORB_ID(sensor_gnss)}, {ORB_ID(sensor_gnss)}};
	uORB::PublicationMulti<sensor_gnss_relative_s> _sensor_gnss_relative_pub[GPS_MAX_INSTANCES] {
		{ORB_ID(sensor_gnss_relative)}, {ORB_ID(sensor_gnss_relative)}
	};

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	// Failure injection (FAILURE_UNIT_SENSOR_GPS): active config + per-instance last-good sample.
	failure_injection::Config _failure_config;
	failure_injection::Stuck<sensor_gnss_s> _stuck[GPS_MAX_INSTANCES];
	failure_injection::Stuck<sensor_gnss_relative_s> _stuck_relative[GPS_MAX_INSTANCES];

	static constexpr hrt_abstime RTCM_TIMEOUT{5_s};
	hrt_abstime _last_rtcm_time{0};

	matrix::Quatf _attitude{};

	ReceiverNoise _noise[GPS_MAX_INSTANCES] {};

	// Gauss-Markov noise parameters, rate-corrected from GZBridge (30 Hz) to SIH (8 Hz)
	static constexpr float _pos_noise_amplitude{0.8f};
	static constexpr float _pos_random_walk{0.02f};
	static constexpr float _pos_markov_time{0.76f};
	static constexpr float _vel_noise_amplitude{0.05f};
	static constexpr float _vel_noise_density{0.4f};
	static constexpr float _vel_markov_time{0.54f};

	// Baseline error of a carrier phase fixed heading solution
	static constexpr float _baseline_noise{0.005f}; // [m]

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SIM_GPS_USED>)      _sim_gps_used,
		(ParamInt<px4::params::SIM_GNSS_NUM>)      _sim_gnss_num,
		(ParamFloat<px4::params::SIM_GNSS0_BIAS_N>) _param_sim_gnss0_bias_n,
		(ParamFloat<px4::params::SIM_GNSS0_BIAS_E>) _param_sim_gnss0_bias_e,
		(ParamFloat<px4::params::SIM_GNSS0_BIAS_D>) _param_sim_gnss0_bias_d,
		(ParamFloat<px4::params::SIM_GNSS1_BIAS_N>) _param_sim_gnss1_bias_n,
		(ParamFloat<px4::params::SIM_GNSS1_BIAS_E>) _param_sim_gnss1_bias_e,
		(ParamFloat<px4::params::SIM_GNSS1_BIAS_D>) _param_sim_gnss1_bias_d,
		(ParamFloat<px4::params::SENS_GNSS0_OFFX>) _param_gnss0_offx,
		(ParamFloat<px4::params::SENS_GNSS0_OFFY>) _param_gnss0_offy,
		(ParamFloat<px4::params::SENS_GNSS0_OFFZ>) _param_gnss0_offz,
		(ParamFloat<px4::params::SENS_GNSS1_OFFX>) _param_gnss1_offx,
		(ParamFloat<px4::params::SENS_GNSS1_OFFY>) _param_gnss1_offy,
		(ParamFloat<px4::params::SENS_GNSS1_OFFZ>) _param_gnss1_offz
#if defined(CONFIG_SENSORS_VEHICLE_GNSS_HEADING)
		,
		(ParamInt<px4::params::SENS_GNSS0_HDG>) _param_gnss0_hdg,
		(ParamFloat<px4::params::SENS_GNSS0_AUXX>) _param_gnss0_auxx,
		(ParamFloat<px4::params::SENS_GNSS0_AUXY>) _param_gnss0_auxy,
		(ParamFloat<px4::params::SENS_GNSS0_AUXZ>) _param_gnss0_auxz,
		(ParamInt<px4::params::SENS_GNSS1_HDG>) _param_gnss1_hdg,
		(ParamFloat<px4::params::SENS_GNSS1_AUXX>) _param_gnss1_auxx,
		(ParamFloat<px4::params::SENS_GNSS1_AUXY>) _param_gnss1_auxy,
		(ParamFloat<px4::params::SENS_GNSS1_AUXZ>) _param_gnss1_auxz
#endif // CONFIG_SENSORS_VEHICLE_GNSS_HEADING
	)
};
