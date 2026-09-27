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

#include <lib/mathlib/math/Limits.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/sensor_gnss_relative.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_gnss_heading.h>
#include <uORB/topics/pps_capture.h>

#include "GnssHeadingBaseline.hpp"
#include "gps_blending.hpp"
#include "PpsTimeSync.hpp"

using namespace time_literals;

namespace sensors
{
class VehicleGPSPosition : public ModuleParams, public px4::ScheduledWorkItem
{
public:

	VehicleGPSPosition();
	~VehicleGPSPosition() override;

	bool Start();
	void Stop();

	void PrintStatus();

private:
	void Run() override;

	void ParametersUpdate(bool force = false);

	// define max number of GPS receivers supported
	static constexpr int GPS_MAX_RECEIVERS = 2;
	static_assert(GPS_MAX_RECEIVERS == GpsBlending::GPS_MAX_RECEIVERS_BLEND,
		      "GPS_MAX_RECEIVERS must match to GPS_MAX_RECEIVERS_BLEND");

	static constexpr hrt_abstime kDefaultDelay{110_ms}; // matches SENS_GPS*_DELAY default
	static constexpr hrt_abstime kHeadingSourceTimeout{3_s};
	static constexpr hrt_abstime kHeadingSettleTime{1_s};

	struct GpsParamSlot {
		uint32_t device_id{0};
		matrix::Vector3f offset{};
		hrt_abstime delay_us{kDefaultDelay};
		matrix::Vector3f baseline{}; // antenna baseline the heading is measured along, body frame (m)
		float baseline_length{0.f};
		float heading_offset{0.f};   // yaw of the baseline in the body frame (rad)
	};

	// SENS_GPSn_* slot for a receiver, by device_id or (when no IDs are configured) by sensor_gps instance
	const GpsParamSlot *findParamSlot(uint32_t device_id, int instance) const;

#if defined(CONFIG_SENSORS_VEHICLE_GNSS_HEADING)
	struct HeadingSample {
		uint64_t timestamp_sample;
		uint32_t device_id;
		float heading;          // measured baseline heading (rad), NAN when not valid
		float heading_accuracy;
		float baseline_length;  // reported baseline (m), NAN when not reported
		float baseline_down;
		uint8_t jamming_state;
		uint8_t spoofing_state;
		bool from_relative;     // sensor_gnss_relative, else sensor_gps
	};

	void UpdateGnssHeading(const sensor_gps_s gps_data[GPS_MAX_RECEIVERS], const bool gps_updated[GPS_MAX_RECEIVERS],
			       const float measured_heading[GPS_MAX_RECEIVERS]);
	// Returns true if the sample passed the checks and is from the source in use (published once settled)
	bool handleHeadingSample(const HeadingSample &sample, const GpsParamSlot *slot);

	// sensor_gps instance publishing this device_id, or -1, with its latest sample in gps_data (zeroed when not found)
	int findGpsInstance(uint32_t device_id, sensor_gps_s &gps_data);
	// Measured baseline heading rotated into the body frame; NAN without a configured baseline
	static float bodyHeading(const GpsParamSlot *slot, float heading);
#endif // CONFIG_SENSORS_VEHICLE_GNSS_HEADING

	static uint64_t resolveSampleTimestamp(uint64_t driver_timestamp_sample, uint64_t driver_timestamp,
					       hrt_abstime delay_us);

	// defines used to specify the mask position for use of different accuracy metrics in the GPS blending algorithm
	static constexpr uint8_t BLEND_MASK_USE_SPD_ACC  = 1;
	static constexpr uint8_t BLEND_MASK_USE_HPOS_ACC = 2;
	static constexpr uint8_t BLEND_MASK_USE_VPOS_ACC = 4;

	uORB::Publication<sensor_gps_s> _vehicle_gps_position_pub{ORB_ID(vehicle_gps_position)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionCallbackWorkItem _sensor_gps_sub[GPS_MAX_RECEIVERS] {	/**< sensor data subscription */
		{this, ORB_ID(sensor_gps), 0},
		{this, ORB_ID(sensor_gps), 1},
	};

	uORB::Subscription _pps_capture_sub{ORB_ID(pps_capture)};

#if defined(CONFIG_SENSORS_VEHICLE_GNSS_HEADING)
	uORB::Publication<vehicle_gnss_heading_s> _vehicle_gnss_heading_pub {ORB_ID(vehicle_gnss_heading)};

	uORB::SubscriptionCallbackWorkItem _sensor_gnss_relative_sub[GPS_MAX_RECEIVERS] {
		{this, ORB_ID(sensor_gnss_relative), 0},
		{this, ORB_ID(sensor_gnss_relative), 1},
	};

	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};

	struct HeadingSource {
		uint32_t device_id{0};
		bool from_relative{false};
		hrt_abstime last_pass{0};     // last sample that passed the checks
		hrt_abstime settled_since{0}; // start of the current run of samples passing the checks
	} _heading_source{};

	bool _heading_unconfigured_reported{false};
#endif // CONFIG_SENSORS_VEHICLE_GNSS_HEADING

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	GpsBlending _gps_blending;
	PpsTimeSync _pps_time_sync;

	GpsParamSlot _gps_param_slots[GPS_MAX_RECEIVERS] {};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SENS_GPS_MASK>) _param_sens_gps_mask,
		(ParamFloat<px4::params::SENS_GPS_TAU>) _param_sens_gps_tau,
		(ParamInt<px4::params::SENS_GPS_PRIME>) _param_sens_gps_prime,
		(ParamInt<px4::params::SENS_GPS0_ID>) _param_sens_gps0_id,
		(ParamFloat<px4::params::SENS_GPS0_OFFX>) _param_sens_gps0_offx,
		(ParamFloat<px4::params::SENS_GPS0_OFFY>) _param_sens_gps0_offy,
		(ParamFloat<px4::params::SENS_GPS0_OFFZ>) _param_sens_gps0_offz,
		(ParamInt<px4::params::SENS_GPS1_ID>) _param_sens_gps1_id,
		(ParamFloat<px4::params::SENS_GPS1_OFFX>) _param_sens_gps1_offx,
		(ParamFloat<px4::params::SENS_GPS1_OFFY>) _param_sens_gps1_offy,
		(ParamFloat<px4::params::SENS_GPS1_OFFZ>) _param_sens_gps1_offz,
#if defined(CONFIG_SENSORS_VEHICLE_GNSS_HEADING)
		(ParamInt<px4::params::SENS_GPS0_HDG>) _param_sens_gps0_hdg,
		(ParamFloat<px4::params::SENS_GPS0_BLX>) _param_sens_gps0_blx,
		(ParamFloat<px4::params::SENS_GPS0_BLY>) _param_sens_gps0_bly,
		(ParamFloat<px4::params::SENS_GPS0_BLZ>) _param_sens_gps0_blz,
		(ParamInt<px4::params::SENS_GPS1_HDG>) _param_sens_gps1_hdg,
		(ParamFloat<px4::params::SENS_GPS1_BLX>) _param_sens_gps1_blx,
		(ParamFloat<px4::params::SENS_GPS1_BLY>) _param_sens_gps1_bly,
		(ParamFloat<px4::params::SENS_GPS1_BLZ>) _param_sens_gps1_blz,
#endif // CONFIG_SENSORS_VEHICLE_GNSS_HEADING
		(ParamInt<px4::params::SENS_GPS0_DELAY>) _param_sens_gps0_delay,
		(ParamInt<px4::params::SENS_GPS1_DELAY>) _param_sens_gps1_delay
	)
};
}; // namespace sensors
