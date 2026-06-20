/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include <array>
#include <vector>

#include <px4_rerun/px4_rerun.hpp>
#include <rerun.hpp>

#include <lib/geo/geo.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/home_position.h>
#include <uORB/topics/log_message.h>
#include <uORB/topics/navigator_mission_item.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/mag_worker_data.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>

using namespace time_literals;

class RerunLogger : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	explicit RerunLogger(const char *sim_name);
	~RerunLogger() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	static constexpr uint32_t SAMPLE_FREQUENCY_HZ = 60;
	static constexpr uint32_t SAMPLE_INTERVAL_US = 1_s / SAMPLE_FREQUENCY_HZ;

	// Rerun
	rerun::RecordingStream _rec;
	bool _rerun_connected{false};

	// Geo projection for lat/lon to local NED
	MapProjection _map_projection{};
	bool _map_ref_initialized{false};

	// uORB subscriptions
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _home_position_sub{ORB_ID(home_position)};
	uORB::Subscription _log_message_sub{ORB_ID(log_message)};
	uORB::Subscription _navigator_mission_item_sub{ORB_ID(navigator_mission_item)};
	static constexpr uint8_t MAX_MAGS = mag_worker_data_s::MAX_MAGS;
	uORB::SubscriptionMultiArray<sensor_mag_s, MAX_MAGS> _sensor_mag_subs{ORB_ID::sensor_mag};
	uORB::Subscription _mag_worker_data_sub{ORB_ID(mag_worker_data)};

	// Cached state
	float _ref_alt{0.f};
	uint32_t _mag_cal_counter[MAX_MAGS]{};
	uint32_t _mag_cal_sides_flushed{0};
	uint64_t _mag_cal_last_timestamp{0};
	std::vector<std::array<float, 3>> _mag_cal_samples[MAX_MAGS];
};
