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
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/sensor_gps_checks.h>
#include <uORB/topics/pps_capture.h>
#include <uORB/topics/vehicle_land_detected.h>

#include "gnss_checks.hpp"
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

	// defines used to specify the mask position for use of different accuracy metrics in the GPS blending algorithm
	static constexpr uint8_t BLEND_MASK_USE_SPD_ACC  = 1;
	static constexpr uint8_t BLEND_MASK_USE_HPOS_ACC = 2;
	static constexpr uint8_t BLEND_MASK_USE_VPOS_ACC = 4;

	// define max number of GPS receivers supported
	static constexpr int GPS_MAX_RECEIVERS = 2;
	static_assert(GPS_MAX_RECEIVERS == GpsBlending::GPS_MAX_RECEIVERS_BLEND,
		      "GPS_MAX_RECEIVERS must match to GPS_MAX_RECEIVERS_BLEND");

	uORB::Publication<sensor_gps_s> _vehicle_gps_position_pub{ORB_ID(vehicle_gps_position)};
	uORB::PublicationMulti<sensor_gps_checks_s> _sensor_gps_checks_pub[GPS_MAX_RECEIVERS] {
		ORB_ID(sensor_gps_checks),
		ORB_ID(sensor_gps_checks)
	};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionCallbackWorkItem _sensor_gps_sub[GPS_MAX_RECEIVERS] {	/**< sensor data subscription */
		{this, ORB_ID(sensor_gps), 0},
		{this, ORB_ID(sensor_gps), 1},
	};

	uORB::Subscription _pps_capture_sub{ORB_ID(pps_capture)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};

	vehicle_land_detected_s _vehicle_land_detected;

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	GnssChecks _gnss_checks[GPS_MAX_RECEIVERS];
	GpsBlending _gps_blending;
	PpsTimeSync _pps_time_sync;

	struct GpsParamSlot {
		uint32_t device_id{0};
		matrix::Vector3f offset{};
		hrt_abstime delay_us{110_ms};
	} _gps_param_slots[GPS_MAX_RECEIVERS] {};

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
		(ParamInt<px4::params::SENS_GPS0_DELAY>) _param_sens_gps0_delay,
		(ParamInt<px4::params::SENS_GPS1_DELAY>) _param_sens_gps1_delay,
		(ParamInt<px4::params::GPS_CHECK>)	 _param_gps_check,
		(ParamInt<px4::params::REQ_NSATS>) 	_param_req_nsats,
		(ParamFloat<px4::params::REQ_HDOP>) 	_param_req_hdop,
		(ParamFloat<px4::params::REQ_VDOP>) 	_param_req_vdop,
		(ParamFloat<px4::params::REQ_EPH>) 	_param_req_eph,
		(ParamFloat<px4::params::REQ_EPV>) 	_param_req_epv,
		(ParamFloat<px4::params::REQ_SACC>) 	_param_req_sacc,
		(ParamFloat<px4::params::REQ_HDRIFT>)	_param_req_hdrift,
		(ParamFloat<px4::params::REQ_VDRIFT>) 	_param_req_vdrift,
		(ParamInt<px4::params::REQ_FIX>) 	_param_req_fix,
		(ParamFloat<px4::params::REQ_GPS_H>) 	_param_req_gps_h,
		(ParamFloat<px4::params::EKF2_VEL_LIM>)	_param_ekf2_vel_lim
	)
};
}; // namespace sensors
