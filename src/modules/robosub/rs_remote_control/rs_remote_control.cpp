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

 #include "rs_remote_control.hpp"

 #include "../rs_motor_control/rs_motor_control.hpp"

 #include <px4_platform_common/getopt.h>
 #include <px4_platform_common/log.h>
 #include <px4_platform_common/posix.h>

 #include <uORB/topics/parameter_update.h>
 #include <uORB/topics/sensor_combined.h>

extern "C" __EXPORT int rs_remote_control_main(int argc, char *argv[]);

 int RobosubRemoteControl::print_status()
 {
	 PX4_INFO("Running");
	 // TODO: print additional runtime information about the state of the module

	 return 0;
 }

 int RobosubRemoteControl::custom_command(int argc, char *argv[])
 {
	 /*
	 if (!is_running()) {
		 print_usage("not running");
		 return 1;
	 }

	 // additional custom commands can be handled like this:
	 if (!strcmp(argv[0], "do-something")) {
		 get_instance()->do_something();
		 return 0;
	 }
	  */

	 return print_usage("unknown command");
 }


 int RobosubRemoteControl::task_spawn(int argc, char *argv[])
 {
	 RobosubRemoteControl *instance = new RobosubRemoteControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
 }

bool RobosubRemoteControl::init()
{
	// Execute the Run() function everytime an input_rc is publiced
	// if (!.registerCallback()) {
	// 	PX4_ERR("callback registration failed");
	// 	return true;
	// }

	ScheduleOnInterval(100_ms);
	PX4_DEBUG("RobosubRemoteControl::init()");
	return true;
}

RobosubRemoteControl::RobosubRemoteControl() :
ModuleParams(nullptr),
ScheduledWorkItem(MODULE_NAME, px4:: wq_configurations::nav_and_controllers),
_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
 {

 }

 RobosubRemoteControl::~RobosubRemoteControl()
{
    // clean up if necessary
}

 void RobosubRemoteControl::Run()
 {
	perf_begin(_loop_perf);

	taskStat();

	receiver();

	// Schedule();
	perf_end(_loop_perf);
 }

 float normalized[8];
 float range = 1.0f;
 uint8_t bitReg = 0;
 uint8_t update1 = 0;

 void RobosubRemoteControl::taskStat()
 {
	update1 = 0;
	if(_input_rc_sub.update(&_input_rc))
	{
		update1 = 1;
		input_rc_s rc_data{};
		_input_rc_sub.copy(&rc_data);
		normalized[4] = (rc_data.values[4] - 1500) / 400.0f;
		normalized[5] = (rc_data.values[5] - 1500) / 400.0f;
		normalized[6] = (rc_data.values[6] - 1500) / 400.0f;
 		normalized[7] = (rc_data.values[7] - 1500) / 400.0f;

		normalized[4] = math::constrain(normalized[4], -range, range);
		normalized[5] = math::constrain(normalized[5], -range, range);
		normalized[6] = math::constrain(normalized[6], -range, range);
		normalized[7] = math::constrain(normalized[7], -range, range);

		uint8_t stateEnable((normalized[4] > 0.0f) ? 1 : 0);

		drone_task_s drone_task{};

		if(stateEnable == 1)
		{
			bitReg =
    			((normalized[5] > 0.0f) ? 1 : 0) |
    			((normalized[6] > 0.0f) ? 1 : 0) << 1 |
    			((normalized[7] > 0.0f) ? 1 : 0) << 2;
			switch(bitReg)
			{
				case 0b1000:
					drone_task.task = TASK_INIT;
				break;
				case 0b1001:
					drone_task.task = TASK_DEFAULT;
				break;
				case 0b1010:
					drone_task.task = TASK_AUTONOMOUS;
				break;
				case 0b1111:
					drone_task.task = TASK_REMOTE_CONTROLLED;
				break;
				default:

				break;
			}

			drone_task.timestamp = hrt_absolute_time();

			_drone_task_pub.publish(drone_task);

		}

	}
 }

 uint8_t sensor1 = false;
 uint8_t sensor2 = false;

 void RobosubRemoteControl::receiver()
 {
	RobosubMotorControl robosub_motor_control;

		if (update1)
		{
			if(bitReg == TASK_REMOTE_CONTROLLED)
			{
				input_rc_s rc_data {};
				_input_rc_sub.copy(&rc_data);

				_water_detection_sub.update(&_water_detection);

				sensor1 	= 1;//= _water_detection.power_module_sensor;
				sensor2 	= 1;//= _water_detection.mainbrain_sensor;


				if (!sensor1 && ! sensor2)
				{
					range = 0.1f;

				} else if (sensor1 && ! sensor2)
				{
					range = 0.2f;

				} else if (sensor1 &&  sensor2)
				{
					range = 1.0f;
				}

				// Normalize the rc data to a value between -1 and 1
				normalized[0] = (rc_data.values[1] - 1500) / 400.0f;
				normalized[1] = (rc_data.values[2] - 1500) / 400.0f;
				normalized[2] = (rc_data.values[3] - 1500) / 400.0f;
				normalized[3] = (rc_data.values[0] - 1500) / 400.0f;

				normalized[0] = math::constrain(normalized[0], -range, range);
				normalized[1] = math::constrain(normalized[1], -range, range);
				normalized[2] = math::constrain(normalized[2], -range, range);
				normalized[3] = math::constrain(normalized[3], -range, range);


				robosub_motor_control.actuator_test(MOTOR_FORWARDS1, 	normalized[0], 0, false);
				robosub_motor_control.actuator_test(MOTOR_FORWARDS2, 	normalized[0], 0, false);
				robosub_motor_control.actuator_test(MOTOR_UP1, 		normalized[1], 0, false);
				robosub_motor_control.actuator_test(MOTOR_UP2, 		(normalized[1] * 0.5f), 0, false);
				robosub_motor_control.actuator_test(MOTOR_UP3, 		(normalized[1] * 0.5f), 0, false);
				if(normalized[2] > 0.1f || normalized[2] < -0.1f)
				{
					robosub_motor_control.actuator_test(MOTOR_SIDE1, normalized[2], 0, false);
					robosub_motor_control.actuator_test(MOTOR_SIDE2, normalized[2], 0, false);
				}
				 else
				{
					robosub_motor_control.actuator_test(MOTOR_SIDE1, -normalized[3], 0, false);
					robosub_motor_control.actuator_test(MOTOR_SIDE2, normalized[3], 0, false);
				}
			}
			update1 = 0;
		}

 }


 void RobosubRemoteControl::parameters_update(bool force)
 {
	 // check for parameter updates
	 if (_parameter_update_sub.updated() || force) {
		 // clear update
		 parameter_update_s update;
		 _parameter_update_sub.copy(&update);

		 // update parameters from storage
		 updateParams();
	 }
 }

 int RobosubRemoteControl::print_usage(const char *reason)
 {
	 if (reason) {
		 PX4_WARN("%s\n", reason);
	 }

	 PRINT_MODULE_DESCRIPTION(
		 R"DESCR_STR(
 ### Description
 Section that describes the provided module functionality.

 This is a template for a module running as a task in the background with start/stop/status functionality.

 ### Implementation
 Section describing the high-level implementation of this module.

 ### Examples
 CLI usage example:
 $ module start -f -p 42

 )DESCR_STR");

	 PRINT_MODULE_USAGE_NAME("module", "rs arm control");
	 PRINT_MODULE_USAGE_COMMAND("start");
	 PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	 PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	 PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	 return 0;
 }


 int rs_remote_control_main(int argc, char *argv[])
 {
	 return RobosubRemoteControl::main(argc, argv);
 }
