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

#pragma once

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
#include <uORB/topics/state_sharing_msg.h>
#include <uORB/topics/state_sharing_control.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_global_position.h>

class StateSharing;
class PublisherStateSharing : public px4::ScheduledWorkItem
{
	friend class StateSharing;
public:
	static constexpr const char *kPublisherWorkItemName = "publisher_state_sharing";

	PublisherStateSharing(StateSharing *parent, const px4::wq_config_t &config);

	/**
	 * @brief Start the state sharing publisher with specified timing parameters.
	 *
	 * @param period_us The period between state sharing messages in microseconds
	 * @param delay_us The initial delay before starting to publish in microseconds
	 * @param sync_epoch_us Real-time (UTC) epoch in microseconds at which the schedule is anchored.
	 *                      The delay is added on top of the time remaining until this epoch.
	 *                      0 starts immediately (only the delay applies).
	 */
	void start(double period_us, double delay_us, uint64_t sync_epoch_us);

	/**
	 * @brief Stop the state sharing publisher.
	 *
	 * Stops the publisher and clears the scheduled work item.
	 */
	void stop();

	/**
	 * @brief Check if the publisher is currently running.
	 *
	 * @return true if the publisher is started and running, false otherwise
	 */
	bool is_started();

private:
	bool _first_time_publish{false};
	bool _started{false}; ///< True if sharing is started

	void Run() override;

	// Publications
	uORB::Publication<state_sharing_msg_s> _incoming_state_sharing_pub{ORB_ID(incoming_state_sharing)};
	uORB::Publication<state_sharing_msg_s> _outgoing_state_sharing_pub{ORB_ID(outgoing_state_sharing)};

	StateSharing *_parent{nullptr};
};

class StateSharing : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	StateSharing();
	~StateSharing() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/**
	 * @brief Initialize the state sharing module.
	 *
	 * This function initializes the state sharing module by:
	 * - Registering callbacks for vehicle odometry and state sharing control
	 * - Setting up the frame ID from system parameters
	 * - Optionally starting the publisher if start_publishing is true
	 *
	 * @param start_publishing If true, starts the publisher immediately after initialization
	 * @return true if initialization successful, false otherwise
	 */
	bool init(bool start_publishing);

	/**
	 * @brief Get the current state sharing message.
	 *
	 * Only the fields enabled in the active data flags are populated, the rest are zeroed
	 * so that MAVLink 2 zero-truncation strips them from the wire.
	 *
	 * @return Current state sharing message with the telemetry selected by the data flags
	 */
	state_sharing_msg_s getStateSharing() const;

	/**
	 * @brief Start the state sharing publisher.
	 *
	 * The publisher will begin sending state sharing messages at the specified interval.
	 *
	 * @param period_s Period between messages in seconds
	 * @param delay_s Initial delay before the first message in seconds
	 * @param sync_epoch_us Real-time epoch in microseconds to anchor the schedule (0 = immediate)
	 */
	void start_publisher(float period_s, float delay_s, uint64_t sync_epoch_us);

	/**
	 * @brief Start the state sharing publisher with the configured parameters.
	 *
	 * Uses SHARING_PERIOD and DELAY_START with no synchronization epoch.
	 */
	void start_publisher();

	/**
	 * @brief Stop the state sharing publisher.
	 *
	 * Stops the publisher from sending state sharing messages.
	 */
	void stop_publisher();

	int print_status() override;

	/**
	 * @brief Get current real time in nanoseconds.
	 *
	 * @return Current CLOCK_REALTIME time in nanoseconds
	 */
	inline static uint64_t getRealTimeNs()
	{
		timespec tv = {};
		px4_clock_gettime(CLOCK_REALTIME, &tv);
		return static_cast<uint64_t>(tv.tv_sec) * 1000000000ULL + static_cast<uint64_t>(tv.tv_nsec);
	}

	/**
	 * @brief Get current real time in microseconds.
	 *
	 * @return Current CLOCK_REALTIME time in microseconds
	 */
	inline static uint64_t getRealTimeUs()
	{
		return getRealTimeNs() / 1000ULL;
	}

private:
	void Run() override;

	/**
	 * @brief Update data_flags in _state_sharing and zero out inactive fields.
	 *
	 * Zeroed trailing fields are stripped on the wire via MAVLink 2 zero-truncation.
	 *
	 * @param flags Bitmask of active telemetry fields
	 */
	void update_data_flags(uint16_t flags);

	// Subscriptions
	uORB::SubscriptionCallbackWorkItem _vehicle_odometry_sub{this, ORB_ID(vehicle_odometry)};
	uORB::Subscription                 _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription                 _vehicle_global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::SubscriptionCallbackWorkItem _state_sharing_control_sub{this, ORB_ID(incoming_state_sharing_control)};
	uORB::Subscription                 _parameter_update_sub{ORB_ID(parameter_update)};

	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MAV_SYS_ID>) _param_mav_sys_id,
		(ParamFloat<px4::params::SHARING_PERIOD>) _param_sharing_period,
		(ParamFloat<px4::params::DELAY_START>) _param_delay_start,
		(ParamInt<px4::params::SHARING_FLAGS>) _param_sharing_flags
	)

	state_sharing_msg_s _state_sharing{};

	// Publisher helper
	PublisherStateSharing _publisher_state_sharing;
};
