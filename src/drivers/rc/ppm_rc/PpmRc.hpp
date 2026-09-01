/****************************************************************************
 *
 *   Copyright (c) 2012-2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <board_config.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/parameter_update.h>

#if defined(HRT_PPM_CHANNEL)
# include <systemlib/ppm_decode.h>
#endif

using namespace time_literals;

class PpmRc : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	PpmRc();
	virtual ~PpmRc();

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;

private:
	void Run() override;

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::PublicationMulti<input_rc_s> _input_rc_pub{ORB_ID(input_rc)};
	perf_counter_t _cycle_perf;
	perf_counter_t _publish_interval_perf;
#if defined(HRT_PPM_CHANNEL)
	hrt_abstime _timestamp_last_signal {0};
	bool _locked{false};
#endif

	static constexpr unsigned _current_update_interval{4000};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::RC_RSSI_PWM_CHAN>) _param_rc_rssi_pwm_chan,
		(ParamInt<px4::params::RC_RSSI_PWM_MIN>) _param_rc_rssi_pwm_min,
		(ParamInt<px4::params::RC_RSSI_PWM_MAX>) _param_rc_rssi_pwm_max
	)
};
