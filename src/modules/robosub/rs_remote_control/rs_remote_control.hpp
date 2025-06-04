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
 #include <uORB/topics/parameter_update.h>
 #include <uORB/Subscription.hpp>
 #include <uORB/SubscriptionCallback.hpp>
 #include <uORB/Publication.hpp>
 #include <lib/perf/perf_counter.h>
 #include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
 #include <uORB/topics/input_rc.h>
 #include <uORB/topics/water_detection.h>
 #include <uORB/topics/drone_task.h>

 using namespace time_literals;

 extern "C" __EXPORT int rs_remote_control_main(int argc, char *argv[]);


 class RobosubRemoteControl : public ModuleBase<RobosubRemoteControl>,  public ModuleParams, public px4::ScheduledWorkItem
 {
 public:
	 RobosubRemoteControl();
	~RobosubRemoteControl();

	void receiver();

	 /** @see ModuleBase */
	 static int task_spawn(int argc, char *argv[]);

	 bool init();

	 /** @see ModuleBase */
	 static int custom_command(int argc, char *argv[]);

	 /** @see ModuleBase */
	 static int print_usage(const char *reason = nullptr);

	 /** @see ModuleBase::run() */
	 void Run() override;


	 /** @see ModuleBase::print_status() */
	 int print_status() override;

 private:

	 /**
	  * Check for parameter changes and update them if needed.
	  * @param parameter_update_sub uorb subscription to parameter_update
	  * @param force for a parameter update

	  */

	#define TASK_INIT 0b000
	#define TASK_DEFAULT 0b001
	#define TASK_REMOTE_CONTROLLED 0b111

	perf_counter_t	_loop_perf;

	uORB::SubscriptionCallbackWorkItem _water_detection_sub{this, ORB_ID(water_detection)};

	water_detection_s 	_water_detection{};

	void taskStat();

	 void parameters_update(bool force = false);


	 DEFINE_PARAMETERS(
		 (ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,   /**< example parameter */
		 (ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig  /**< another parameter */
	 )

	 // Subscriptions
	 uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	 uORB::Subscription _input_rc_sub{ORB_ID(input_rc)};

	 uORB::Publication<drone_task_s>    _drone_task_pub{ORB_ID(drone_task)};

	 drone_task_s _drone_task{};
	 input_rc_s _input_rc{};

 };
