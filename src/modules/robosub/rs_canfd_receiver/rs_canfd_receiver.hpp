/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <sys/socket.h>
#include <net/if.h>
#include <netpacket/can.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <sys/socket.h>
#include <nuttx/can.h>
#include <sys/time.h>
#include <uORB/Publication.hpp>

#include <uORB/topics/water_detection.h>
#include <uORB/topics/raw_canfd.h>
#include <uORB/topics/internal_sensors.h>

using namespace time_literals;

extern "C" __EXPORT int rs_canfd_receiver_main(int argc, char *argv[]);
class RoboSubCANFDReceiver : public ModuleBase<RoboSubCANFDReceiver>, public ModuleParams, public px4::WorkItem
{
public:
	__attribute__((optimize(0))) RoboSubCANFDReceiver();

	virtual ~RoboSubCANFDReceiver() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void __attribute__((optimize(0))) Run();

	/** @see ModuleBase::print_status() */
	int print_status() override;

	bool init();

private:
	// perf_counter_t	_loop_perf;
	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(bool force = false);

	/**
	 * Send a test message
	 */
	void send_0x01();
	void send_0x02();

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,   /**< example parameter */
		(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig  /**< another parameter */
	)

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::SubscriptionCallbackWorkItem _raw_canfd_sub{this, ORB_ID(raw_canfd)}; /**< raw canfd subscription */

	raw_canfd_s _raw_canfd_msg{}; /**< raw canfd message */
	raw_canfd_s _send_raw_canfd_msg{};


	union can_id_u {
		uint32_t id;
		struct  {
            		uint32_t command_type : 4;
            		uint32_t session : 3;
            		uint32_t client_id_src : 5;
            		uint32_t module_id_src : 5;
			uint32_t client_id_des : 5;
            		uint32_t module_id_des : 5;
			uint32_t emergency : 2; // Emergency status
			uint32_t rest : 3;
		} can_id_seg;
	};

	can_id_u received_id;
	uORB::Publication<px4::msg::waterDetection> water_detection_pub{ORB_ID(water_detection)};
	uORB::Publication<px4::msg::RawCanfd> send_raw_canfd_pub{ORB_ID(send_raw_canfd)};
	uORB::Publication<px4::msg::InternalSensors> internal_sensors_pub{ORB_ID(internal_sensors)};

};
