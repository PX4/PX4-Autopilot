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

#include <drivers/drv_hrt.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/internal_combustion_engine_control.h>
#include <uORB/topics/internal_combustion_engine_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/rpm.h>
#include <uORB/topics/actuator_motors.h>

#include <lib/slew_rate/SlewRate.hpp>

namespace internal_combustion_engine_control
{

class InternalCombustionEngineControl : public ModuleBase<InternalCombustionEngineControl>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	InternalCombustionEngineControl();
	~InternalCombustionEngineControl() override;

	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[])
	{
		return print_usage("unknown command");
	}

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	void start();

private:
	void Run() override;

	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _rpm_sub{ORB_ID(rpm)};
	uORB::Subscription _actuator_motors{ORB_ID(actuator_motors)};

	uORB::Publication<internal_combustion_engine_control_s> _internal_combustion_engine_control_pub{ORB_ID(internal_combustion_engine_control)};
	uORB::Publication<internal_combustion_engine_status_s> _internal_combustion_engine_status_pub{ORB_ID(internal_combustion_engine_status)};


	// has to mirror internal_combustion_engine_status_s::State
	enum class State {
		Stopped,
		Starting,
		Running,
		Fault
	} _state{State::Stopped};

	enum class SubState {
		Run,
		Rest
	};

	enum class UserOnOffRequest {
		Off,
		On
	};

	enum class ICESource {
		ArmingState,
		Aux1,
		Aux2,
	};

	hrt_abstime _state_start_time{0};
	hrt_abstime _last_time_run{0};

	bool _ignition_on{false};
	float _choke_control{1.f};
	float _starter_engine_control{0.f};
	float _throttle_control{0.f};

	SlewRate<float> _throttle_control_slew_rate;

	bool isEngineRunning(const hrt_abstime now);
	void controlEngineRunning(float throttle_in);
	void controlEngineStop();
	void controlEngineStartup(const hrt_abstime now);
	void controlEngineFault();
	bool maximumAttemptsReached();
	void publishControl(const hrt_abstime now, const UserOnOffRequest user_request);

	// Starting state specifics
	static constexpr float DELAY_BEFORE_RESTARTING{1.f};
	int _starting_retry_cycle{0};
	hrt_abstime _starting_rest_time{0};
	SubState _starting_sub_state{SubState::Run};

	/**
	 * @brief Currently the ICE starting state is permitted after resting
	 * DELAY_BEFORE_RESTARTING s to reduce wear on the starter motor.
	 * @param now current time
	 * @return if true, otherwise false
	 */
	bool isStartingPermitted(const hrt_abstime now);

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::ICE_ON_SOURCE>) _param_ice_on_source,
		(ParamFloat<px4::params::ICE_CHOKE_ST_DUR>) _param_ice_choke_st_dur,
		(ParamFloat<px4::params::ICE_STRT_DUR>) _param_ice_strt_dur,
		(ParamFloat<px4::params::ICE_MIN_RUN_RPM>) _param_ice_min_run_rpm,
		(ParamInt<px4::params::ICE_STRT_ATTEMPT>) _param_ice_strt_attempts,
		(ParamInt<px4::params::ICE_RUN_FAULT_D>) _param_ice_running_fault_detection,
		(ParamFloat<px4::params::ICE_STRT_THR>) _param_ice_strt_thr,
		(ParamInt<px4::params::ICE_STOP_CHOKE>) _param_ice_stop_choke,
		(ParamFloat<px4::params::ICE_THR_SLEW>) _param_ice_thr_slew,
		(ParamFloat<px4::params::ICE_IGN_DELAY>) _param_ice_ign_delay
	)
};

} // namespace internal_combustion_engine_control
