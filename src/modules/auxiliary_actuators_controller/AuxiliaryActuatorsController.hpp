/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <lib/slew_rate/SlewRate.hpp>

#include <uORB/topics/parameter_update.h>

#include <uORB/topics/action_request.h>
#include <uORB/topics/landing_gear_setpoint.h>
#include <uORB/topics/landing_gear_auto_setpoint.h>

#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/position_setpoint_triplet.h>

#include <uORB/topics/flaps_auto_setpoint.h>
#include <uORB/topics/flaps_setpoint.h>
#include <uORB/topics/spoilers_auto_setpoint.h>
#include <uORB/topics/spoilers_setpoint.h>

using namespace time_literals;

static constexpr float kDtMin = 0.002f;
static constexpr float kDtMax = 0.04f;
static constexpr float kFlapSlewRate = 1.f; //minimum time from none to full flap deflection [s]
static constexpr float kSpoilerSlewRate = 1.f; //minimum time from none to full spoiler deflection [s]

namespace auxiliary_actuators_controller
{

class AuxiliaryActuatorsController : public ModuleBase<AuxiliaryActuatorsController>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	AuxiliaryActuatorsController();
	~AuxiliaryActuatorsController() override;

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

	void action_request_poll();
	void landing_gear_auto_setpoint_poll();

	landing_gear_setpoint_s landing_gear_;
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	manual_control_setpoint_s manual_control_setpoint_;
	vehicle_control_mode_s vehicle_control_mode_;
	vehicle_status_s vehicle_status_;
	position_setpoint_triplet_s position_setpoint_triplet_;

	uORB::Subscription action_request_sub_{ORB_ID(action_request)};

	uORB::Subscription vehicle_status_sub_{ORB_ID(vehicle_status)};
	uORB::Subscription position_setpoint_triplet_sub_{ORB_ID(position_setpoint_triplet)};
	uORB::Subscription landing_gear_auto_setpoint_sub_{ORB_ID(landing_gear_auto_setpoint)};
	uORB::Subscription flaps_auto_setpoint_sub_{ORB_ID(flaps_auto_setpoint)};
	uORB::Subscription spoilers_auto_setpoint_sub_{ORB_ID(spoilers_auto_setpoint)};
	uORB::Subscription vehicle_control_mode_sub_{ORB_ID(vehicle_control_mode)};
	uORB::Subscription manual_control_setpoint_sub_{ORB_ID(manual_control_setpoint)};
	uORB::Publication<landing_gear_setpoint_s> landing_gear_pub_{ORB_ID(landing_gear_setpoint)};
	uORB::Publication<flaps_setpoint_s> flaps_setpoint_pub_{ORB_ID(flaps_setpoint)};
	uORB::Publication<spoilers_setpoint_s> spoilers_setpoint_pub_{ORB_ID(spoilers_setpoint)};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	hrt_abstime _last_run{0};

	int lading_gear_auto_setpoint_old_{landing_gear_auto_setpoint_s::GEAR_KEEP};

	SlewRate<float> spoilers_setpoint_with_slewrate_;
	SlewRate<float> flaps_setpoint_with_slewrate_;

	/**
	 * @brief Update flap control setting
	 *
	 * @param dt Current time delta [s]
	 */
	void controlFlaps(const float dt);

	/**
	 * @brief Update spoiler control setting
	 *
	 * @param dt Current time delta [s]
	 */
	void controlSpoilers(const float dt);

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::FW_FLAPS_TO_SCL>) _param_fw_flaps_to_scl,
		(ParamFloat<px4::params::FW_FLAPS_LND_SCL>) _param_fw_flaps_lnd_scl,
		(ParamFloat<px4::params::FW_SPOILERS_LND>) _param_fw_spoilers_lnd,
		(ParamFloat<px4::params::FW_SPOILERS_DESC>) _param_fw_spoilers_desc,
		(ParamInt<px4::params::FW_SPOILERS_MAN>) _param_fw_spoilers_man,
		(ParamFloat<px4::params::VT_SPOILER_MC_LD>) _param_vt_spoiler_mc_ld

		// (ParamFloat<px4::params::ACTL_>) _param_fw_flaps_lnd_scl,
		// (ParamFloat<px4::params::ACTL_>) _param_fw_flaps_lnd_scl,
	)

	// TODO: add param to retract landing gear on the ground or not
};

} // namespace auxiliary_actuators_controller
