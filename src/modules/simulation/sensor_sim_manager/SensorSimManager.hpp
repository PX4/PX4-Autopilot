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

#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/airspeed_validated.h>
#include <simulation/failure_injection/failureInjection.hpp>
#include <drivers/drv_sensor.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <lib/world_magnetic_model/geo_mag_declination.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/lat_lon_alt/lat_lon_alt.hpp>
#include <random>

using namespace time_literals;

class SensorSimManager : public ModuleBase<SensorSimManager>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	SensorSimManager();
	~SensorSimManager() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	// Sensor simulation methods
	void updateGPS();
	void updateBarometer();
	void updateMagnetometer();
	void updateAirspeed();
	void updateAGP();

	// Utility methods
	static float generate_wgn();
	matrix::Vector3f noiseGauss3f(float stdx, float stdy, float stdz);

	// Sensor timing structure
	struct SensorTiming {
		hrt_abstime interval_us;
		hrt_abstime next_update_time;
		hrt_abstime offset_us;
		bool enabled;
	};

	// Sensor timings with random offsets
	SensorTiming _gps_timing;
	SensorTiming _baro_timing;
	SensorTiming _mag_timing;
	SensorTiming _airspeed_timing;
	SensorTiming _agp_timing;

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _vehicle_global_position_sub{ORB_ID(vehicle_global_position_groundtruth)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position_groundtruth)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude_groundtruth)};

	// Publications
	uORB::PublicationMulti<sensor_gps_s> _sensor_gps_pub{ORB_ID(sensor_gps)};
	uORB::PublicationMulti<sensor_baro_s> _sensor_baro_pub{ORB_ID(sensor_baro)};
	uORB::PublicationMulti<differential_pressure_s> _differential_pressure_pub{ORB_ID(differential_pressure)};
	uORB::PublicationMulti<vehicle_global_position_s> _aux_global_position_pub{ORB_ID(aux_global_position)};

	// Magnetometer uses PX4Magnetometer publisher
	PX4Magnetometer _px4_mag{1, ROTATION_NONE};

	FailureInjection _failure_injection{};

	// TODO: needed?
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _gps_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": gps")};
	perf_counter_t _baro_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": baro")};
	perf_counter_t _mag_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": mag")};
	perf_counter_t _airspeed_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": airspeed")};
	perf_counter_t _agp_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": agp")};

	// Random number generator for offsets and noise
	std::random_device _rd;
	std::mt19937 _gen;
	std::uniform_real_distribution<float> _uniform_dist;

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SENS_EN_GPSSIM>) _param_sens_en_gpssim,
		(ParamInt<px4::params::SENS_EN_BAROSIM>) _param_sens_en_barosim,
		(ParamInt<px4::params::SENS_EN_MAGSIM>) _param_sens_en_magsim,
		(ParamInt<px4::params::SENS_EN_ARSPDSIM>) _param_sens_en_arspdsim,
		(ParamInt<px4::params::SENS_EN_AGPSIM>) _param_sens_en_agpsim,
		(ParamInt<px4::params::SIM_GPS_USED>) _sim_gps_used,
		(ParamFloat<px4::params::SIM_BARO_OFF_P>) _sim_baro_off_p,
		(ParamFloat<px4::params::SIM_BARO_OFF_T>) _sim_baro_off_t,
		(ParamFloat<px4::params::SIM_MAG_OFFSET_X>) _sim_mag_offset_x,
		(ParamFloat<px4::params::SIM_MAG_OFFSET_Y>) _sim_mag_offset_y,
		(ParamFloat<px4::params::SIM_MAG_OFFSET_Z>) _sim_mag_offset_z,
		(ParamInt<px4::params::SIM_AGP_FAIL>) _sim_agp_fail
	)

	// Barometer simulation state
	hrt_abstime _last_baro_update_time{0};
	float _baro_drift_pa{0.0f};
	float _baro_drift_pa_per_sec{0.1f}; // TODO, was 0
	bool _baro_rnd_use_last{false};
	double _baro_rnd_y2{0.0};

	// Magnetometer simulation state
	matrix::Vector3f _mag_earth_pred{};
	bool _mag_earth_available{false};

	// AGP simulation state
	LatLonAlt _agp_measured_lla{};
	matrix::Vector3f _agp_position_bias{};
	uint64_t _agp_time_last_update{0};

	// Air constants
	static constexpr float TEMPERATURE_MSL = 288.0f; // [K]
	static constexpr float PRESSURE_MSL = 101325.0f; // [Pa]
	static constexpr float LAPSE_RATE = 0.0065f; // [K/m]
	static constexpr float AIR_DENSITY_MSL = 1.225f; // [kg/m^3]

};
