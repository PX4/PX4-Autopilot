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

#include <lib/battery/battery.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

class BatterySimulator : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	BatterySimulator();
	~BatterySimulator() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;
	void updateParams() override;

	// Average current in A drawn from the given battery in the current flight mode, negative if not simulated
	float avgCurrent(int battery_index, bool is_fw) const;

	// Lowest state of charge in percent the given battery drains down to
	float minimumPercentage(int battery_index) const;

	static constexpr uint32_t BATTERY_SIMLATOR_SAMPLE_FREQUENCY_HZ = 100; // Hz
	static constexpr uint32_t BATTERY_SIMLATOR_SAMPLE_INTERVAL_US = 1_s / BATTERY_SIMLATOR_SAMPLE_FREQUENCY_HZ;

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	// A battery only publishes if its BAT<N>_SOURCE is set to "Power Module / Analog".
	// Allocated once in the constructor: Battery is not copyable and older compilers (GCC < 9) do not elide the copy
	// when constructing an array of them in place.
	Battery *_batteries[battery_status_s::MAX_INSTANCES] {};

	// Per-battery overrides of SIM_BAT_I_FW, SIM_BAT_I_MC, and SIM_BAT_MIN_PCT, negative when unset
	param_t _i_fw_handles[battery_status_s::MAX_INSTANCES] {};
	param_t _i_mc_handles[battery_status_s::MAX_INSTANCES] {};
	param_t _min_pct_handles[battery_status_s::MAX_INSTANCES] {};
	float _i_fw_override_s[battery_status_s::MAX_INSTANCES] {};
	float _i_mc_override_s[battery_status_s::MAX_INSTANCES] {};
	float _min_pct_override[battery_status_s::MAX_INSTANCES] {};

	float _last_min_percentage[battery_status_s::MAX_INSTANCES] {};
	bool _capacity_warned[battery_status_s::MAX_INSTANCES] {};
	bool _armed{false};
	bool _was_armed{false};
	bool _is_fw{false};

	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::SIM_BAT_I_FW>) _param_sim_bat_i_fw, ///< battery avg current in fixed wings
		(ParamFloat<px4::params::SIM_BAT_I_MC>) _param_sim_bat_i_mc, ///< battery avg current in multicopter
		(ParamFloat<px4::params::SIM_BAT_MIN_PCT>) _param_bat_min_pct //< minimum battery percentage
	)
};
