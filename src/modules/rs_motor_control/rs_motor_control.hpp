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

/**
 *
 * This module is a modification of the rover attitide control module and is designed for the
 * TUHH hippocampus.
 *
 * All the acknowledgments and credits for the fw wing app are reported in those files.
 *
 * @author Daniel Duecker <daniel.duecker@tuhh.de>
 * @author Philipp Hastedt <philipp.hastedt@tuhh.de>
 * @author Tim Hansen <t.hansen@tuhh.de>
 */

 #include <float.h>

 #include <drivers/drv_hrt.h>
 #include <lib/geo/geo.h>
 #include <lib/mathlib/mathlib.h>
 #include <lib/perf/perf_counter.h>
 #include <matrix/math.hpp>
 #include <px4_platform_common/px4_config.h>
 #include <px4_platform_common/defines.h>
 #include <px4_platform_common/posix.h>
 #include <px4_platform_common/tasks.h>
 #include <px4_platform_common/module.h>
 #include <px4_platform_common/module_params.h>
 #include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
 #include <uORB/Subscription.hpp>
 #include <uORB/SubscriptionCallback.hpp>
 #include <uORB/Publication.hpp>
 #include <uORB/topics/parameter_update.h>
 #include <uORB/topics/input_rc.h>
#include <uORB/topics/actuator_test.h>


 #include <uORB/uORB.h>

 using matrix::Eulerf;
 using matrix::Quatf;
 using matrix::Matrix3f;
 using matrix::Vector3f;
 using matrix::Dcmf;

 using uORB::SubscriptionData;

 using namespace time_literals;

 class RobosubMotorControl: public ModuleBase<RobosubMotorControl>, public ModuleParams, public px4::WorkItem
 {
 public:
 	RobosubMotorControl();
	 ~RobosubMotorControl();

	 /** @see ModuleBase */
	 static int task_spawn(int argc, char *argv[]);

	 static int custom_command(int argc, char *argv[]);

	 /** @see ModuleBase */
	 static int print_usage(const char *reason = nullptr);

	 bool init();

 private:
	 uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	 uORB::Subscription _input_rc_sub{ORB_ID(input_rc)};

	 uORB::SubscriptionCallbackWorkItem _rs_input_rc_sub{this, ORB_ID(input_rc)};

	 input_rc_s _input_rc{};

	 perf_counter_t	_loop_perf;


	 void Run() override;
	 /**
	  * Update our local parameter cache.
	  */
	 void parameters_update(bool force = false);

	 void actuator_test(int function, float value, int timeout_ms, bool release_control);

 };
