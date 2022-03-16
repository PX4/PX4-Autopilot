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

/**
 * @file RLSWrenchEstimator.hpp
 * @brief Interface class for a RLS parameter identification and
 * external wrench estimator
 *
 * @author Pedro Mendes <pmen817@aucklanduni.ac.nz>
 */

#pragma once

#include "RLSIdentification/RLSIdentification.hpp"
#include "WrenchEstimator/WrenchEstimator.hpp"

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <lib/hysteresis/hysteresis.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/rls_wrench_estimator.h>
#include <uORB/topics/debug_vect.h>
#include <uORB/topics/battery_status.h>

using namespace time_literals;

class RLSWrenchEstimator : public ModuleBase<RLSWrenchEstimator>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	RLSWrenchEstimator();
	~RLSWrenchEstimator() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;
	void updateParams() override;

	void publishStatus();
	void publishInvalidStatus();
	bool copyAndCheckAllFinite(vehicle_acceleration_s &accel, actuator_outputs_s &actuator_outputs,
					vehicle_attitude_s &v_att, vehicle_angular_velocity_s &v_ang_vel, battery_status_s &batt_stat);

	RLSIdentification _identification{};
	WrenchEstimator _wrench_estimator{};

	// Publications
	// uORB::Publication<debug_vect_s> _debug_vect_pub{ORB_ID(debug_vect)};
	uORB::Publication<rls_wrench_estimator_s> _rls_wrench_estimator_pub{ORB_ID(rls_wrench_estimator)};

	// Subscriptions
	uORB::SubscriptionCallbackWorkItem _vehicle_acceleration_sub{this, ORB_ID(vehicle_acceleration)};        // subscription that schedules RLSWrenchEstimator when updated

	uORB::SubscriptionInterval  _parameter_update_sub{ORB_ID(parameter_update), 1_s}; // subscription limited to 1 Hz updates

	uORB::Subscription _actuator_outputs_sub{ORB_ID(actuator_outputs)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};          // regular subscription for additional data
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _debug_vect_sub{ORB_ID(debug_vect)};
	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};

	hrt_abstime _timestamp_last{0};
	systemlib::Hysteresis _valid_hysteresis{false};

	// Performance (perf) counter
	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RLS_EST_MASS>) _param_rls_mass,
		(ParamFloat<px4::params::RLS_EST_TILT>) _param_rls_tilt,
		(ParamFloat<px4::params::RLS_EST_LPF_M>) _param_rls_lpf_motor,
		(ParamFloat<px4::params::RLS_EST_KF_INIT>) _param_rls_kf_init,
		(ParamFloat<px4::params::RLS_EST_KR_INIT>) _param_rls_kr_init,
		(ParamFloat<px4::params::RLS_EST_KF_CONF>) _param_rls_kf_conf,
		(ParamFloat<px4::params::RLS_EST_KR_CONF>) _param_rls_kr_conf,
		(ParamFloat<px4::params::RLS_EST_XY_NOISE>) _param_rls_xy_noise,
		(ParamFloat<px4::params::RLS_EST_Z_NOISE>) _param_rls_z_noise,
		(ParamInt<px4::params::RLS_EST_N_ROTORS>) _param_rls_n_rotors,
		(ParamFloat<px4::params::RLS_EST_SPE_P1>) _param_rls_speed_p1,
		(ParamFloat<px4::params::RLS_EST_SPE_P2>) _param_rls_speed_p2,
		(ParamFloat<px4::params::RLS_EST_SPE_V1>) _param_rls_speed_v1,
		(ParamFloat<px4::params::RLS_EST_SPE_V2>) _param_rls_speed_v2,
		(ParamFloat<px4::params::RLS_EST_TAU_F>) _param_rls_lpf_force,
		(ParamFloat<px4::params::RLS_EST_TAU_M>) _param_rls_lpf_moment,
		(ParamFloat<px4::params::RLS_EST_XO_INIT>) _param_rls_xo_init,
		(ParamFloat<px4::params::RLS_EST_YO_INIT>) _param_rls_yo_init,
		(ParamFloat<px4::params::RLS_EST_XO_CONF>) _param_rls_xo_conf,
		(ParamFloat<px4::params::RLS_EST_YO_CONF>) _param_rls_yo_conf,
		(ParamFloat<px4::params::RLS_EST_F_NOISE>) _param_rls_f_noise,
		(ParamFloat<px4::params::RLS_EST_KM>) _param_rls_km,
		(ParamFloat<px4::params::RLS_EST_D>) _param_rls_diameter,
		(ParamFloat<px4::params::RLS_EST_TOP_H>) _param_rls_top_height,
		(ParamFloat<px4::params::RLS_EST_BOT_H>) _param_rls_bot_height,
		(ParamFloat<px4::params::RLS_EST_IXX>) _param_rls_inertia_x,
		(ParamFloat<px4::params::RLS_EST_IYY>) _param_rls_inertia_y,
		(ParamFloat<px4::params::RLS_EST_IZZ>) _param_rls_inertia_z,
		(ParamInt<px4::params::BAT1_N_CELLS>) _param_n_cells
	)

	bool _armed{false};
	bool _landed{false};
	bool _in_air{false};
	bool _valid{false};
	bool _finite{false};
	bool _interaction_flag{false};
	hrt_abstime _debug_timestamp_last{};
	float _voltage{11.7f};
};
