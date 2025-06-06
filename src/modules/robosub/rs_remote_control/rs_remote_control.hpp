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
 #include <uORB/topics/internal_sensors.h>

 using namespace time_literals;

 extern "C" __EXPORT int rs_remote_control_main(int argc, char *argv[]);


 class RobosubRemoteControl : public ModuleBase<RobosubRemoteControl>,  public ModuleParams, public px4::ScheduledWorkItem
 {
 public:

	#define TASK_INIT 0b000
	#define TASK_DEFAULT 0b001
	#define TASK_AUTONOMOUS 0b010
	#define TASK_REMOTE_CONTROLLED 0b111

	enum MotorID {
    	MOTOR_FORWARDS1  = 101,
    	MOTOR_FORWARDS2	 = 106,
    	MOTOR_UP1   	 = 103,
    	MOTOR_UP2 	 = 104,
    	MOTOR_UP3 	 = 102,
    	MOTOR_SIDE1 	 = 105,
    	MOTOR_SIDE2  	 = 107
	};

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

	perf_counter_t	_loop_perf;

	uORB::Subscription _water_detection_sub{ORB_ID(water_detection)};

	water_detection_s 	_water_detection{};
	water_detection_s 	water_detection_msg{}; // create the temp message struct

	float calculate_absolute_humidity(float rel_humidity, float temperature);

	void taskStat();

	void parameters_update(bool force = false);

	void check_internal_state();

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::OFF_A_HUMIDITY>) _param_offset_abs_humidity,
		(ParamFloat<px4::params::OFF_TEMPERATURE>) _param_offset_temperature,
		(ParamFloat<px4::params::OFF_PRESSURE>) _param_offset_pressure,
		(ParamFloat<px4::params::OFF_R_HUMIDITY>) _param_offset_rel_humidity
	)

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::SubscriptionCallbackWorkItem _input_rc_sub{this, ORB_ID(input_rc)};
	uORB::SubscriptionCallbackWorkItem _internal_sensors_sub{this, ORB_ID(internal_sensors)};


	uORB::Publication<drone_task_s>    _drone_task_pub{ORB_ID(drone_task)};

	drone_task_s _drone_task{};
	input_rc_s _input_rc{};

	float normalized[8];
	float range = 1.0f;
	uint8_t bitReg = 0;
	uint8_t update1 = 0;

	bool force_overide = false;

	bool sensor_mainbrain = false;
	bool sensor_power = false;

	// Running average filter variables
	static constexpr size_t FILTER_SIZE = 10;
	static constexpr size_t N_MODULES = 2;


	struct SensorFilter {
		float values[FILTER_SIZE];
		size_t index;
		size_t count;
		float sum;
		float initial_average = 0.0f;
		bool updated = false;
		uint64_t last_update = 0;

		SensorFilter() : index(0), count(0), sum(0.0f), last_update(0) {
		for (size_t i = 0; i < FILTER_SIZE; i++) {
			values[i] = 0.0f;
		}
		}
	};

	SensorFilter _humidity_filter[N_MODULES];
	SensorFilter _temperature_filter[N_MODULES];
	SensorFilter _pressure_filter[N_MODULES];
	SensorFilter _absolute_humidity_filter[N_MODULES]; // A bit overkill, but it makes sense to have it for consistency


	float _filtered_humidity[N_MODULES] = {0.0f};
	float _filtered_temperature[N_MODULES] = {0.0f};
	float _filtered_pressure[N_MODULES] = {0.0f};
	float _filtered_absolute_humidity[N_MODULES] = {0.0f};


	// Helper function to update running average
	float update_running_average(SensorFilter& filter, float new_value);

 };
