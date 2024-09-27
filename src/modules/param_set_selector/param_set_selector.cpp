/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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
 * @file sensors.cpp
 *
 * @author Daniel M. Sahu <>
 */

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/parameters/param.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/parameter_selector.h>

using namespace time_literals;

class ParamSetSelector : public ModuleBase<ParamSetSelector>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	ParamSetSelector();
	~ParamSetSelector() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** perform all initialization processes */
	bool init();

private:
	/**< hardcoded parameter sets */
	enum class ParameterSet {
		DISABLED = 0,
                ALT_FAST = 1,
		ALT_MEDIUM = 2,
                ALT_SLOW = 3,
		ACRO_FAST = 4,
                RESERVED5 = 5
	};

	/** Core loop method. */
	void Run() override;

	/** Check if we should update from rc input.
	 * Returns true if the selector state has changed
	*/
	bool set_from_rc_input();

	/** Check if we should update from params.
	 * Returns true if the selector state has changed
	*/
	bool set_from_params();

	/** switch to the given set. */
	void switchSet(const ParameterSet& set);

	/**< notification of parameter updates */
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	/**< subscription to RC Inputs */
	uORB::Subscription _input_rc_sub{ORB_ID(input_rc)};

	/**< loop performance counter */
	perf_counter_t	_loop_perf;

	/**< RC channel index (-1 means unused) */
	int _rc_channel_index {-1};

	/**< Currently selected parameter set */
	ParameterSet _current_set {ParameterSet::DISABLED};

	/**< define *our* parameters */
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::PSET_MODE>)		_param_pset_mode,
		(ParamInt<px4::params::PSET_CHANNEL>)		_param_param_set_channel,
		(ParamFloat<px4::params::PSET_FST_TILT>)	_param_pset_fst_tilt,
		(ParamFloat<px4::params::PSET_FST_VELZ>) 	_param_pset_fst_velz,
		(ParamFloat<px4::params::PSET_FST_TAU>)		_param_pset_fst_tau,
		(ParamFloat<px4::params::PSET_MED_TILT>)	_param_pset_med_tilt,
		(ParamFloat<px4::params::PSET_MED_VELZ>) 	_param_pset_med_velz,
		(ParamFloat<px4::params::PSET_MED_TAU>)		_param_pset_med_tau,
		(ParamFloat<px4::params::PSET_SLW_TILT>)	_param_pset_slw_tilt,
		(ParamFloat<px4::params::PSET_SLW_VELZ>) 	_param_pset_slw_velz,
		(ParamFloat<px4::params::PSET_SLW_TAU>)		_param_pset_slw_tau,
		(ParamFloat<px4::params::FLGT_VZ_MAX>)		_param_flgt_vz_max
	)

	/* Parameter uorb publication */
	uORB::Publication<parameter_selector_s> _parameter_selector_pub{ORB_ID(parameter_selector)};
	parameter_selector_s _parameter_selector;
	uint32_t _instance; 	// Number of times the uorb parameter_selector has been published
};

ParamSetSelector::ParamSetSelector() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME))
{
	// get initial params
	updateParams();

	// get the channel to listen on for RC inputs
	_rc_channel_index = static_cast<int>(_param_param_set_channel.get()) - 1;
}

ParamSetSelector::~ParamSetSelector()
{
	ScheduleClear();
}

bool ParamSetSelector::set_from_rc_input()
{
	// exit early if we aren't updated
	if (!_input_rc_sub.updated())
		return false;

	// get channel value
	const uint8_t idx = static_cast<uint8_t>(_rc_channel_index);
	if (_rc_channel_index >= input_rc_s::RC_INPUT_MAX_CHANNELS) {
		// if we get here something really bad happened. like this should be an assert
		PX4_ERR("Invalid channel index requested: %i", idx);
		return false;
	}

	// get latest RC input value
	input_rc_s input_rc;
	_input_rc_sub.copy(&input_rc);
	uint16_t pwm = input_rc.values[idx];

	// map PWM values (~1000-2000) to our acceptable range ([3,2,1])
	// @TODO make this configurable somehow? users could choose
	float normalized = (static_cast<float>(pwm) - 1000.0f) / 1000.0f;
	int requested = round(2.0f * (1.0f - math::constrain(normalized, 0.0f, 1.0f)) + 1.0f);

	// if this matches our current value do nothing
	if (static_cast<ParameterSet>(requested) == _current_set)
		return false;

	// update local variables and perform parameter set switch
	PX4_DEBUG("Switching from Parameter Set #%i to #%i", static_cast<int>(_current_set), requested);
	_current_set = static_cast<ParameterSet>(requested);
	switchSet(_current_set);

	// also set the corresponding param to prevent confusing disconnects
	param_set(param_find("PSET_MODE"), &requested);

	return true;
}

bool ParamSetSelector::set_from_params()
{
	// update parameter set, if modified
	ParameterSet requested = static_cast<ParameterSet>(_param_pset_mode.get());
	if (requested == _current_set)
	{
		return false;
	}

	_current_set = requested;
	switchSet(requested);
	return true;
}

void ParamSetSelector::switchSet(const ParameterSet& set)
{
	// initialize set of parameters we support - defaults to a baseline ALT_MED
	float mpc_tiltmax_air {20.0};
	float mpc_man_tilt_max {20.0};
	int mc_airmode {0};
	float mc_man_tilt_tau {20.0};
	float param_flgt_vz_max {1.5};

	// switch to the new parameter set
	_parameter_selector.state = uint8_t(set);
	switch (set)
	{
		case ParameterSet::DISABLED:
			return;
		case ParameterSet::ACRO_FAST:
		{
			strncpy(_parameter_selector.short_string, "ACR", sizeof(_parameter_selector.short_string));

			// hardcoded params for ACRO_FAST
			mpc_man_tilt_max = 65.0;
			mpc_tiltmax_air = 65.0;
			mc_airmode = 0; // typically 2
			PX4_INFO("Updating to ACRO_FAST params.");
			break;
		}
		case ParameterSet::ALT_FAST:
		{
			strncpy(_parameter_selector.short_string, "FST", sizeof(_parameter_selector.short_string));

			// hardcoded params for ALT_FAST
			mc_airmode = 0;

			mpc_man_tilt_max = _param_pset_fst_tilt.get();
			mpc_tiltmax_air  = _param_pset_fst_tilt.get();
			param_flgt_vz_max = _param_pset_fst_velz.get();
			mc_man_tilt_tau  = _param_pset_fst_tau.get();

			PX4_INFO("Updating to ALT_FAST params.");
			break;
		}
		case ParameterSet::ALT_MEDIUM:
		{
			strncpy(_parameter_selector.short_string, "MED", sizeof(_parameter_selector.short_string));

			// hardcoded params for ALT_MEDIUM
			mc_airmode = 0;

			mpc_man_tilt_max = _param_pset_med_tilt.get();
			mpc_tiltmax_air  = _param_pset_med_tilt.get();
			param_flgt_vz_max = _param_pset_med_velz.get();
			mc_man_tilt_tau  = _param_pset_med_tau.get();

			PX4_INFO("Updating to ALT_MEDIUM params.");
			break;
		}
		case ParameterSet::ALT_SLOW:
		{
			strncpy(_parameter_selector.short_string, "SLW", sizeof(_parameter_selector.short_string));

			// hardcoded params for ALT_SLOW
			mc_airmode = 0;

			mpc_man_tilt_max = _param_pset_slw_tilt.get();
			mpc_tiltmax_air  = _param_pset_slw_tilt.get();
			param_flgt_vz_max = _param_pset_slw_velz.get();
			mc_man_tilt_tau  = _param_pset_slw_tau.get();

			PX4_INFO("Updating to ALT_SLOW params.");
			break;
		}
		case ParameterSet::RESERVED5:
			strncpy(_parameter_selector.short_string, "RS5", sizeof(_parameter_selector.short_string));
			break;
		default:
			// Should not get here
			strncpy(_parameter_selector.short_string, "ERR", sizeof(_parameter_selector.short_string));
			return;
	}

	// if we got this far, set each param
	// FIXME  grab the handles at init instead of using string look ups
	param_set_no_notification(param_find("MPC_TILTMAX_AIR"), &mpc_tiltmax_air);
	param_set_no_notification(param_find("MPC_MAN_TILT_MAX"), &mpc_man_tilt_max);
	param_set_no_notification(param_find("MC_AIRMODE"), &mc_airmode);
	param_set_no_notification(param_find("MC_MAN_TILT_TAU"), &mc_man_tilt_tau);

	// Update Parameters
	param_set_no_notification(param_find("FLGT_VZ_MAX"), &param_flgt_vz_max);

	// batch notify other modules that these have changed
	param_notify_changes();
}

void ParamSetSelector::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	// begin perf counter
	perf_begin(_loop_perf);

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}

	// try to update from an RC input
	bool state_has_changed = false;
	if (_rc_channel_index >= 0)
		state_has_changed = set_from_rc_input();
	else
		state_has_changed = set_from_params();

	// Publish parameter_selector uorb if the selector state has changed
	if (state_has_changed) {
		_parameter_selector.instance = _instance;
		_instance ++;

		_parameter_selector_pub.publish(_parameter_selector);
	}


	// schedule next iteration
	ScheduleDelayed(100_ms);

	// end perf counter
	perf_end(_loop_perf);
}

int ParamSetSelector::task_spawn(int argc, char *argv[])
{
	ParamSetSelector *instance = new ParamSetSelector();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

bool ParamSetSelector::init()
{
	ScheduleNow();
	return true;
}

int ParamSetSelector::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ParamSetSelector::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

This module toggles between disjoint collections (sets) of parameters.

### Implementation

Monitors the 'PSET_MODE' parameter. If the parameter is updated, switch
to the set defined by that parameter and update all corresponding parameters.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("param_set_selector", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int param_set_selector_main(int argc, char *argv[])
{
	return ParamSetSelector::main(argc, argv);
}
