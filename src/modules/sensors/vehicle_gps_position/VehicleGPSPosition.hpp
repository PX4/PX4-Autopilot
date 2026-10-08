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
#include <uORB/topics/sensor_gnss.h>
#include <uORB/topics/vehicle_gnss.h>
#include <uORB/topics/sensor_gnss_relative.h>
#include <uORB/topics/vehicle_gnss_heading.h>
#include <uORB/topics/pps_capture.h>
#include <uORB/topics/sensors_status_gnss.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>
#include <lib/gnss/gnss_checks.hpp>

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
	void UpdateVehicleState();
	void PublishStatus();

	// define max number of GPS receivers supported
	static constexpr int GPS_MAX_RECEIVERS = 2;
	static_assert(GPS_MAX_RECEIVERS == GpsBlending::GPS_MAX_RECEIVERS_BLEND,
		      "GPS_MAX_RECEIVERS must match to GPS_MAX_RECEIVERS_BLEND");

	static constexpr hrt_abstime kDefaultDelay{110_ms}; // matches SENS_GNSS*_DELAY default
	static constexpr hrt_abstime kHeadingSourceTimeout{3_s};
	static constexpr hrt_abstime kHeadingSettleTime{1_s};

	struct GpsParamSlot {
		uint32_t device_id{0};
		matrix::Vector3f offset{};
		hrt_abstime delay_us{kDefaultDelay};
		float baseline_length{0.f};  // of the antenna baseline the heading is measured along (m)
		float heading_offset{0.f};   // yaw of the baseline in the body frame (rad)
		bool heading_enabled{false}; // SENS_GNSSn_HDG isn't Disabled
	};

	// SENS_GNSSn_* slot for a receiver, by device_id or (when no IDs are configured) by sensor_gnss instance
	const GpsParamSlot *findParamSlot(uint32_t device_id, int instance) const;

	// sensor_gnss instance of the receiver SENS_GNSS_PRIME designates, -1 for none or not yet published
	int resolvePreferredInstance() const;

	// SENS_GNSS_PRIME names a receiver, whether or not it has published yet
	bool hasConfiguredPreference() const;

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
	};

	void UpdateGnssHeading();
	void handleHeadingSample(const HeadingSample &sample, const GpsParamSlot *slot);

	// Keeps the furthest sensors_status_gnss_s::HEADING_* state a heading sample reached within kHeadingSourceTimeout
	void reportHeadingState(uint8_t state, hrt_abstime now);

	// sensor_gnss instance publishing this device_id, or -1, with its latest sample in gnss_data (zeroed when not found)
	int findGnssInstance(uint32_t device_id, sensor_gnss_s &gnss_data);
#endif // CONFIG_SENSORS_VEHICLE_GNSS_HEADING

	static uint64_t resolveSampleTimestamp(uint64_t driver_timestamp_sample, uint64_t driver_timestamp,
					       hrt_abstime delay_us);

	uORB::Publication<vehicle_gnss_s> _vehicle_gnss_pub{ORB_ID(vehicle_gnss)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionCallbackWorkItem _sensor_gnss_sub[GPS_MAX_RECEIVERS] {	/**< sensor data subscription */
		{this, ORB_ID(sensor_gnss), 0},
		{this, ORB_ID(sensor_gnss), 1},
	};

	uORB::Subscription _pps_capture_sub{ORB_ID(pps_capture)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	uORB::Publication<sensors_status_gnss_s> _sensors_status_gnss_pub{ORB_ID(sensors_status_gnss)};

#if defined(CONFIG_SENSORS_VEHICLE_GNSS_HEADING)
	uORB::Publication<vehicle_gnss_heading_s> _vehicle_gnss_heading_pub {ORB_ID(vehicle_gnss_heading)};

	uORB::SubscriptionCallbackWorkItem _sensor_gnss_relative_sub[GPS_MAX_RECEIVERS] {
		{this, ORB_ID(sensor_gnss_relative), 0},
		{this, ORB_ID(sensor_gnss_relative), 1},
	};

	struct HeadingSource {
		uint32_t device_id{0};
		hrt_abstime last_pass{0};     // last sample that passed the checks
		hrt_abstime settled_since{0}; // first passing sample since the receiver last had no heading
	} _heading_source{};

	uint8_t _heading_state{sensors_status_gnss_s::HEADING_NONE};
	hrt_abstime _heading_state_time{0};
#endif // CONFIG_SENSORS_VEHICLE_GNSS_HEADING

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	GpsBlending _gps_blending;

	GnssChecks _gnss_checks[GPS_MAX_RECEIVERS] {};
	uint32_t _receiver_device_id[GPS_MAX_RECEIVERS] {};
	hrt_abstime _receiver_timestamp[GPS_MAX_RECEIVERS] {};
	uint32_t _selected_device_id{0};
	int8_t _preferred_instance{-1};
	uint8_t _first_publication[GPS_MAX_RECEIVERS] {}; ///< 1 for the first receiver to publish, 2 for the next, 0 before
	uint8_t _receivers_published{0};

	// The checks run the strict thresholds while disarmed on the ground and the drift checks only at rest
	bool _armed{false};
	bool _in_air{false};
	bool _at_rest{false};
	PpsTimeSync _pps_time_sync;

	GpsParamSlot _gnss_param_slots[GPS_MAX_RECEIVERS] {};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SENS_GNSS_PRIME>) _param_sens_gnss_prime,
		(ParamInt<px4::params::SENS_GNSS0_ID>) _param_sens_gnss0_id,
		(ParamFloat<px4::params::SENS_GNSS0_OFFX>) _param_sens_gnss0_offx,
		(ParamFloat<px4::params::SENS_GNSS0_OFFY>) _param_sens_gnss0_offy,
		(ParamFloat<px4::params::SENS_GNSS0_OFFZ>) _param_sens_gnss0_offz,
		(ParamInt<px4::params::SENS_GNSS1_ID>) _param_sens_gnss1_id,
		(ParamFloat<px4::params::SENS_GNSS1_OFFX>) _param_sens_gnss1_offx,
		(ParamFloat<px4::params::SENS_GNSS1_OFFY>) _param_sens_gnss1_offy,
		(ParamFloat<px4::params::SENS_GNSS1_OFFZ>) _param_sens_gnss1_offz,
#if defined(CONFIG_SENSORS_VEHICLE_GNSS_HEADING)
		(ParamInt<px4::params::SENS_GNSS0_HDG>) _param_sens_gnss0_hdg,
		(ParamFloat<px4::params::SENS_GNSS0_AUXX>) _param_sens_gnss0_auxx,
		(ParamFloat<px4::params::SENS_GNSS0_AUXY>) _param_sens_gnss0_auxy,
		(ParamFloat<px4::params::SENS_GNSS0_AUXZ>) _param_sens_gnss0_auxz,
		(ParamInt<px4::params::SENS_GNSS1_HDG>) _param_sens_gnss1_hdg,
		(ParamFloat<px4::params::SENS_GNSS1_AUXX>) _param_sens_gnss1_auxx,
		(ParamFloat<px4::params::SENS_GNSS1_AUXY>) _param_sens_gnss1_auxy,
		(ParamFloat<px4::params::SENS_GNSS1_AUXZ>) _param_sens_gnss1_auxz,
#endif // CONFIG_SENSORS_VEHICLE_GNSS_HEADING
		(ParamInt<px4::params::SENS_GNSS0_DELAY>) _param_sens_gnss0_delay,
		(ParamInt<px4::params::SENS_GNSS1_DELAY>) _param_sens_gnss1_delay,
		(ParamInt<px4::params::GNSS_CHECK>) _param_gnss_check,
		(ParamInt<px4::params::GNSS_REQ_NSATS>) _param_gnss_req_nsats,
		(ParamFloat<px4::params::GNSS_REQ_PDOP>) _param_gnss_req_pdop,
		(ParamFloat<px4::params::GNSS_REQ_EPH>) _param_gnss_req_eph,
		(ParamFloat<px4::params::GNSS_REQ_EPV>) _param_gnss_req_epv,
		(ParamFloat<px4::params::GNSS_REQ_SACC>) _param_gnss_req_sacc,
		(ParamFloat<px4::params::GNSS_REQ_HDRIFT>) _param_gnss_req_hdrift,
		(ParamFloat<px4::params::GNSS_REQ_VDRIFT>) _param_gnss_req_vdrift,
		(ParamInt<px4::params::GNSS_REQ_FIX>) _param_gnss_req_fix,
		(ParamFloat<px4::params::GNSS_REQ_TIME>) _param_gnss_req_time
	)
};
}; // namespace sensors
