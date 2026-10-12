/*
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include "ptgl100_parser.hpp"

class PTGL100 : public ModuleBase, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	PTGL100(const char *port, uint8_t rotation);
	~PTGL100() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;
	int init();

private:
	static constexpr size_t PORT_NAME_SIZE = 64;

	void Run() override;
	int open_port();
	void close_port();
	void collect();
	void handle_reply(const ptgl100::Reply &reply, hrt_abstime now);
	void start_measurement(hrt_abstime now);
	void transmit();
	void stop_measurement();

	PX4Rangefinder _rangefinder;
	ptgl100::Parser _parser;
	char _port[PORT_NAME_SIZE] {};
	int _fd{-1};
	uint8_t _tx_buffer[ptgl100::FRAME_SIZE] {};
	size_t _tx_offset{ptgl100::FRAME_SIZE};

	hrt_abstime _last_open_attempt{0};
	hrt_abstime _last_start_command{0};
	hrt_abstime _last_measurement_reply{0};
	hrt_abstime _last_sample{0};
	float _last_distance{0.f};
	uint8_t _last_measurement_status{0};
	uint8_t _self_test_status{0};
	uint8_t _self_test_error{0};
	bool _self_test_received{false};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME ": comms errors")};
	perf_counter_t _checksum_errors{perf_alloc(PC_COUNT, MODULE_NAME ": checksum errors")};
	perf_counter_t _unknown_commands{perf_alloc(PC_COUNT, MODULE_NAME ": unknown replies")};
	perf_counter_t _measurement_failures{perf_alloc(PC_COUNT, MODULE_NAME ": measurement failures")};
	perf_counter_t _start_commands{perf_alloc(PC_COUNT, MODULE_NAME ": start commands")};
	perf_counter_t _samples{perf_alloc(PC_COUNT, MODULE_NAME ": samples")};
};
