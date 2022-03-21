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
 * @file AdmittanceControlModule.hpp
 * @brief Interface class for a RLS parameter identification and
 * external wrench estimator
 *
 * @author Pedro Mendes <pmen817@aucklanduni.ac.nz>
 */

#pragma once

#include "AdmittanceControl/AdmittanceControl.hpp"

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/rls_wrench_estimator.h>
#include <uORB/topics/debug_vect.h>

using namespace time_literals;

class AdmittanceControlModule : public ModuleBase<AdmittanceControlModule>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	AdmittanceControlModule();
	~AdmittanceControlModule() override;

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
	bool copyAndCheckAllFinite(rls_wrench_estimator_s &wrench, actuator_outputs_s &actuator_outputs,
					vehicle_attitude_setpoint_s &v_att_sp, vehicle_local_position_setpoint_s &setpoint);


	AdmittanceControl _control{};

	// Publications
	// uORB::Publication<debug_vect_s> _debug_vect_pub{ORB_ID(debug_vect)};
	uORB::Publication<vehicle_local_position_setpoint_s> _admittance_setpoint_pub{ORB_ID(admittance_setpoint)};

	// Subscriptions
	uORB::SubscriptionCallbackWorkItem _rls_wrench_estimator_sub{this, ORB_ID(rls_wrench_estimator)};        // subscription that schedules AdmittanceControlModule when updated

	uORB::SubscriptionInterval  _parameter_update_sub{ORB_ID(parameter_update), 1_s}; // subscription limited to 1 Hz updates
	uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _actuator_outputs_sub{ORB_ID(actuator_outputs)};
	uORB::Subscription _debug_vect_sub{ORB_ID(debug_vect)};

	hrt_abstime _timestamp_last{0};

	// Performance (perf) counter
	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::RLS_EST_N_ROTORS>) _param_rls_n_rotors,

		(ParamFloat<px4::params::ADM_CTR_BEL_AX>) _param_adm_ctr_ax,
		(ParamFloat<px4::params::ADM_CTR_BEL_AY>) _param_adm_ctr_ay,
		(ParamFloat<px4::params::ADM_CTR_BEL_AZ>) _param_adm_ctr_az,
		(ParamFloat<px4::params::ADM_CTR_BEL_AW>) _param_adm_ctr_ayaw,

		(ParamFloat<px4::params::ADM_CTR_BEL_B1X>) _param_adm_ctr_b1x,
		(ParamFloat<px4::params::ADM_CTR_BEL_B1Y>) _param_adm_ctr_b1y,
		(ParamFloat<px4::params::ADM_CTR_BEL_B1Z>) _param_adm_ctr_b1z,
		(ParamFloat<px4::params::ADM_CTR_BEL_B1W>) _param_adm_ctr_b1yaw,

		(ParamFloat<px4::params::ADM_CTR_BEL_B2X>) _param_adm_ctr_b2x,
		(ParamFloat<px4::params::ADM_CTR_BEL_B2Y>) _param_adm_ctr_b2y,
		(ParamFloat<px4::params::ADM_CTR_BEL_B2Z>) _param_adm_ctr_b2z,
		(ParamFloat<px4::params::ADM_CTR_BEL_B2W>) _param_adm_ctr_b2yaw,

		(ParamFloat<px4::params::ADM_CTR_BEL_B3X>) _param_adm_ctr_b3x,
		(ParamFloat<px4::params::ADM_CTR_BEL_B3Y>) _param_adm_ctr_b3y,
		(ParamFloat<px4::params::ADM_CTR_BEL_B3Z>) _param_adm_ctr_b3z,
		(ParamFloat<px4::params::ADM_CTR_BEL_B3W>) _param_adm_ctr_b3yaw,

		(ParamFloat<px4::params::ADM_CTR_BEL_MIX>) _param_adm_ctr_mminx,
		(ParamFloat<px4::params::ADM_CTR_BEL_MIY>) _param_adm_ctr_mminy,
		(ParamFloat<px4::params::ADM_CTR_BEL_MIZ>) _param_adm_ctr_mminz,
		(ParamFloat<px4::params::ADM_CTR_BEL_MIW>) _param_adm_ctr_mminyaw,

		(ParamFloat<px4::params::ADM_CTR_BEL_KIX>) _param_adm_ctr_kminx,
		(ParamFloat<px4::params::ADM_CTR_BEL_KIY>) _param_adm_ctr_kminy,
		(ParamFloat<px4::params::ADM_CTR_BEL_KIZ>) _param_adm_ctr_kminz,
		(ParamFloat<px4::params::ADM_CTR_BEL_KIW>) _param_adm_ctr_kminyaw,

		(ParamFloat<px4::params::ADM_CTR_BEL_MAX>) _param_adm_ctr_mmaxx,
		(ParamFloat<px4::params::ADM_CTR_BEL_MAY>) _param_adm_ctr_mmaxy,
		(ParamFloat<px4::params::ADM_CTR_BEL_MAZ>) _param_adm_ctr_mmaxz,
		(ParamFloat<px4::params::ADM_CTR_BEL_MAW>) _param_adm_ctr_mmaxyaw,

		(ParamFloat<px4::params::ADM_CTR_BEL_KAX>) _param_adm_ctr_kmaxx,
		(ParamFloat<px4::params::ADM_CTR_BEL_KAY>) _param_adm_ctr_kmaxy,
		(ParamFloat<px4::params::ADM_CTR_BEL_KAZ>) _param_adm_ctr_kmaxz,
		(ParamFloat<px4::params::ADM_CTR_BEL_KAW>) _param_adm_ctr_kmaxyaw,
		(ParamFloat<px4::params::ADM_CTR_BEL_LPF>) _param_adm_ctr_lpf,

		(ParamFloat<px4::params::ADM_CTR_WRE_DZX>) _param_adm_ctr_dzx,
		(ParamFloat<px4::params::ADM_CTR_WRE_DZY>) _param_adm_ctr_dzy,
		(ParamFloat<px4::params::ADM_CTR_WRE_DZZ>) _param_adm_ctr_dzz,
		(ParamFloat<px4::params::ADM_CTR_WRE_DZW>) _param_adm_ctr_dzw,

		(ParamFloat<px4::params::ADM_CTR_WRE_SAX>) _param_adm_ctr_sax,
		(ParamFloat<px4::params::ADM_CTR_WRE_SAY>) _param_adm_ctr_say,
		(ParamFloat<px4::params::ADM_CTR_WRE_SAZ>) _param_adm_ctr_saz,
		(ParamFloat<px4::params::ADM_CTR_WRE_SAW>) _param_adm_ctr_saw
	)

	bool _finite{false};
	bool _valid{false};
	bool _sp_updated{false};
	bool _admittance_flag{false};  //Need to receive value from mavlink
	float _target_dist{};//Need to receive value from mavlink
	hrt_abstime _debug_timestamp_last{};
};
