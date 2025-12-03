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

#include <float.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/Serial.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>

#include "config.h"

/**
 * @author Niklas Hauser <niklas@auterion.com>
 */
#ifdef CONFIG_VTXTABLE_UORB_CONFIG
class VtxTable : public ModuleBase<VtxTable>, public px4::ScheduledWorkItem
#else
class VtxTable : public ModuleBase<VtxTable>
#endif
{
public:
	static vtx::Config data;

#ifdef CONFIG_VTXTABLE_UORB_CONFIG
	VtxTable();
#else
	VtxTable() = default;
#endif
	virtual ~VtxTable();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	inline int print_status() override
	{
		print_info();
#ifdef CONFIG_VTXTABLE_UORB_CONFIG
		perf_print_counter(_perf_cycle);
#endif
		return 0;
	}

	static void print_info();
	static void print_power_levels();
	static void print_frequencies();
#ifdef CONFIG_VTXTABLE_AUX_MAP
	static void print_aux_map();
#endif

#ifdef CONFIG_VTXTABLE_USE_STORAGE
	static int store(const char *filename = nullptr);
	static int load(const char *filename = nullptr);
#endif

private:
#ifdef CONFIG_VTXTABLE_UORB_CONFIG
	void Run() override;
	uORB::Subscription _vtx_table_sub {ORB_ID(vtx_table)};
	uORB::Subscription _vtx_aux_map_sub{ORB_ID(vtx_aux_map)};
	perf_counter_t _perf_cycle;
	perf_counter_t _perf_error;
#endif
#ifdef CONFIG_VTXTABLE_USE_STORAGE
	static constexpr const char *_config_file {CONFIG_VTXTABLE_CONFIG_FILE};
#endif
};
