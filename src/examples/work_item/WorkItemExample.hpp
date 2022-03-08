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

#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/topics/orb_test.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/togan_hedef.h>
#include <uORB/topics/vehicle_trajectory_waypoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/wind_estimate.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include "TOGANConfig.hpp"
#include <termios.h>

class WorkItemExample : public ModuleBase<WorkItemExample>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	WorkItemExample();
	~WorkItemExample() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:

	bool isSwitchOn;
	int _serial_fd{-2};
	vehicle_trajectory_waypoint_s vehicle_trajectory_wp;
	vehicle_global_position_s my_global_pos;
	vehicle_gps_position_s my_gps_pos;
	wind_estimate_s  wind_est_rec;
	vehicle_air_data_s vehicle_air_data_d;
	position_setpoint_triplet_s position_setpoint_triplet;
	togan_hedef_s	togan_hedef_data{0};
	void CalculateAndPublishTOGANLAR();
	void Run() override;
#if 1
	uORB::Publication<orb_test_s> _orb_test_pub{ORB_ID(orb_test)};
	
	uORB::SubscriptionData<sensor_accel_s> _sensor_accel_sub{ORB_ID(sensor_accel)};
	uORB::SubscriptionData<vehicle_global_position_s> _my_global_pos{ORB_ID(vehicle_global_position)};
	uORB::SubscriptionData<vehicle_gps_position_s> _my_gps_pos{ORB_ID(vehicle_gps_position)};
	uORB::SubscriptionData<wind_estimate_s> _wind_est_rec{ORB_ID(wind_estimate)};
	uORB::SubscriptionData<position_setpoint_triplet_s> _position_setpoint_triplet_s{ORB_ID(position_setpoint_triplet)};
	uORB::SubscriptionData<togan_hedef_s> _togan_hedef{ORB_ID(togan_hedef)};

#endif

	uORB::SubscriptionData<vehicle_air_data_s> _vehicle_air_data{ORB_ID(vehicle_air_data)};

	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

void firecontrol_pointmass(FP32* ret_val, FP32* xhat, FP32* airspeed_ms, FP32 airDensity_kgm3, FP32 cd, FP32 latitude_rad);
void firecontrol_forwardsimulation(FP32* initstate, FP32 targetheight_m, FP32 temperatureCorrection, FP32 pressureCorrection, FP32 densityCorrection, FP32* windspeed_ms, FP32 timestep_s, FP32 message2R_latitude_deg);
FP32 firecontrol_getairtemperature(FP32 altitude_m);
FP32 firecontrol_getairpressure(FP32 airTemperature_K);
FP32 firecontrol_getairdensity(FP32 airTemperature_K, FP32 airPressure_Pa);
FP32 firecontrol_getmachnumber(FP32 trueairspeed_kt, FP32 airTemperature_K);
FP32 firecontrol_getcd(FP32 machnumber);
void firecontrol_llh2cart(FP32* konumenu, FP32 targetLat_deg, FP32 targetLon_deg, FP32 targetheight_m, FP32 aircraftLat_deg, FP32 aircraftLon_deg, FP32 aircraftHeight_m);
void firecontrol_eastnorth2downcross(FP32* downCross, FP32 posEast_m, FP32 posNorth_m, FP32 aircraftGroundTrack_der);
void firecontrol_wander2northeast(FP32* velne_ms, FP32 wander_deg, FP32 velX_ms, FP32 velY_ms);
void firecontrol_momentarm2ned(FP32* deltaMomentArm_m, FP32 momentArmX_m, FP32 momentArmY_m, FP32 momentArmZ_m, FP32 roll_deg, FP32 pitch_deg, FP32 trueheading_deg);

	const static INT32U MIN_HIZ_KT = 135;
	const static INT32U HIZ_P1_KT = 160;
	const static INT32U HIZ_P2_KT = 165;
	const static INT32U HIZ_P3_KT = 175;
	const static INT32U HIZ_P4_KT = 185;
	const static INT32U MAX_HIZ_KT = 200;

	const static INT32U MIN_IRTIFA_ABOVE_TARGET_FT = 1000;

	const static INT32U MIN_IRTIFA_MIN_P1_FT = 500;
	const static INT32U MAX_IRTIFA_MIN_P1_FT = 10000;

	const static INT32U MIN_IRTIFA_P1_P2_FT = 500;
	const static INT32U MAX_IRTIFA_P1_P2_FT = 10000;

	const static INT32U MIN_IRTIFA_P2_P3_FT = 500;
	const static INT32U MAX_IRTIFA_P2_P3_FT = 6500;

	const static INT32U MIN_IRTIFA_P3_P4_FT = 500;
	const static INT32U MAX_IRTIFA_P3_P4_FT = 4000;

	const static INT32U MIN_IRTIFA_P4_MAX_FT = 500;
	const static INT32U MAX_IRTIFA_P4_MAX_FT = 2500;

	const static INT32U MAX_IRTIFA_FT = 30000;

	const static INT32U ALT_HIGH_FT = 30000;//ft system altitude;
	const static INT32U ALT_LOW_FT = 1000;// ft above target level;

	const static INT32U PITCH_LIMIT_HIGH_DEG = 10;
	const static INT32S PITCH_LIMIT_LOW_DEG = -10;
	const static INT32U ROLL_LIMIT_HIGH_DEG = 3;
	const static INT32S ROLL_LIMIT_LOW_DEG = -3;
};
