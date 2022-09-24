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

#pragma once

#include <float.h>

#include "ghst_parser.hpp"

#include <board_config.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/sensor_gps.h>

class RcGhst : public ModuleBase<RcGhst>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	RcGhst(const char *device);
	virtual ~RcGhst();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	int init();

private:
	void Run() override;

	void fill_rc_in(uint16_t raw_rc_count_local,
			uint16_t raw_rc_values_local[input_rc_s::RC_INPUT_MAX_CHANNELS],
			hrt_abstime now, bool frame_drop, bool failsafe,
			unsigned frame_drops, int rssi);

	void rc_io_invert(bool invert);

	hrt_abstime _rc_scan_begin{0};

	bool _initialized{false};
	bool _rc_scan_locked{false};

	static constexpr unsigned	_current_update_interval{4000}; // 250 Hz

	input_rc_s	_rc_in{};

	uORB::PublicationMulti<input_rc_s>	_to_input_rc{ORB_ID(input_rc)};

	int		_rcs_fd{-1};
	char		_device[20] {};					///< device / serial port path

	static constexpr size_t RC_MAX_BUFFER_SIZE{sizeof(ghst_frame_t)};
	uint8_t _rcs_buf[RC_MAX_BUFFER_SIZE] {};

	uint16_t _raw_rc_values[input_rc_s::RC_INPUT_MAX_CHANNELS] {};
	uint16_t _raw_rc_count{};

	perf_counter_t	_cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")};
	perf_counter_t	_publish_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": publish interval")};
	uint32_t	_bytes_rx{0};

	// telemetry
	bool send_battery_status();
	bool send_gps1_status();
	bool send_gps2_status();

	uORB::Subscription _vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};
	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};

	hrt_abstime _last_update{};
	uint32_t _next_type{};

	static constexpr uint32_t NUM_DATA_TYPES {3U};	// number of different telemetry data types
	static constexpr uint32_t UPDATE_RATE_HZ {10U};	// update rate [Hz]

	// Factors that should be applied to get correct values
	static constexpr float FACTOR_VOLTS_TO_10MV {100.0F};
	static constexpr float FACTOR_AMPS_TO_10MA {100.0F};
	static constexpr float FACTOR_MAH_TO_10MAH {0.1F};
};
