/****************************************************************************
 *
 *   Copyright (c) 2012-2024 PX4 Development Team. All rights reserved.
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

#include <float.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <lib/rc/ghst.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/parameter_update.h>

#include "ghst_telemetry.hpp"

using namespace time_literals;

class GhstRc : public ModuleBase<GhstRc>, public ModuleParams, public px4::ScheduledWorkItem
{
public:

	GhstRc(const char *device);
	virtual ~GhstRc();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:

	void Run() override;

	hrt_abstime _rc_scan_begin{0};
	hrt_abstime _timestamp_last_signal{0};

	bool _rc_scan_locked{false};

	static constexpr unsigned	_current_update_interval{4000}; // 250 Hz

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::PublicationMulti<input_rc_s> _input_rc_pub{ORB_ID(input_rc)};

	int		_rcs_fd{-1};
	char		_device[20] {};					///< device / serial port path

	GHSTTelemetry *_ghst_telemetry{nullptr};

	perf_counter_t	_cycle_perf;
	perf_counter_t	_publish_interval_perf;
	uint32_t	_bytes_rx{0};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::RC_GHST_TEL_EN>) _param_rc_ghst_tel_en
	)
};
