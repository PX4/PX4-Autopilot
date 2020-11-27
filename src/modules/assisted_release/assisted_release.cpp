/****************************************************************************
 *
 *   Copyright (c) 2020 ThunderFly s.r.o. All rights reserved.
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

#include "assisted_release.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/Publication.hpp>
#include <drivers/drv_hrt.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>

#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/rpm.h>


int AssistedRelease::print_status()
{
	PX4_INFO("Running");
    PX4_INFO("State: frequency=%d, airspeed=%d, throttle=%d, switch=%d, pitch_sp=%d", _rpm.indicated_frequency_rpm > (float)_param_min_rpm.get(), _airspeed.indicated_airspeed_m_s > (float)_param_min_aspd.get(), _throttle >= 0.5f, _rc_channel >= 0.5f, _pitch_setpoint > 0);

	return 0;
}

int AssistedRelease::custom_command(int argc, char *argv[])
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


int AssistedRelease::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("assisted_release",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

AssistedRelease *AssistedRelease::instantiate(int argc, char *argv[])
{
	int example_param = 0;
	bool example_flag = false;

	AssistedRelease *instance = new AssistedRelease(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

AssistedRelease::AssistedRelease(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}
void AssistedRelease::play_happy_tone()
{
  		tune_control_s tune_control{};
  		tune_control.tune_id = tune_control_s::TUNE_ID_NOTIFY_NEUTRAL;
  		tune_control.volume = tune_control_s::VOLUME_LEVEL_DEFAULT;
  		tune_control.timestamp = hrt_absolute_time();
  		_tune_control.publish(tune_control);
}

void AssistedRelease::play_sad_tone()
{
  		tune_control_s tune_control{};
  		tune_control.tune_id = tune_control_s::TUNE_ID_NOTIFY_NEGATIVE;
  		tune_control.volume = tune_control_s::VOLUME_LEVEL_DEFAULT;
  		tune_control.timestamp = hrt_absolute_time();
  		_tune_control.publish(tune_control);

}

void AssistedRelease::play_final_tone()
{
  		tune_control_s tune_control{};
  		tune_control.tune_id = tune_control_s::TUNE_ID_NOTIFY_POSITIVE;
  		tune_control.volume = tune_control_s::VOLUME_LEVEL_DEFAULT;
  		tune_control.timestamp = hrt_absolute_time();
  		_tune_control.publish(tune_control);

}


void AssistedRelease::run()
{

	// int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));

	_rpm_sub = orb_subscribe(ORB_ID(rpm));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_airspeed_sub = orb_subscribe(ORB_ID(airspeed));
	_actuator_controls_0_sub = orb_subscribe(ORB_ID(actuator_controls_0));
	_input_rc_sub = orb_subscribe(ORB_ID(input_rc));
	_manual_control_setpoint_sub = orb_subscribe(ORB_ID(manual_control_setpoint));

	orb_set_interval(_rpm_sub, 100);
	orb_set_interval(_vehicle_status_sub, 100);
	orb_set_interval(_airspeed_sub, 100);
	orb_set_interval(_actuator_controls_0_sub, 100);
	orb_set_interval(_input_rc_sub, 100);
    orb_set_interval(_manual_control_setpoint_sub, 100);

	rpm_poll();
	airspeed_poll();
	vehicle_status_poll();
	actuator_controls_poll();
	input_rc_poll();
    manual_control_setpoint_poll();

	actuator_controls_s control{};
	uORB::Publication<actuator_controls_s> control_pub{ORB_ID(actuator_controls_1)};


	px4_pollfd_struct_t fds[] = {
	    { .fd = _rpm_sub,   .events = POLLIN },
	    { .fd = _vehicle_status_sub,   .events = POLLIN },
	    { .fd = _airspeed_sub,   .events = POLLIN },
	};

  // hrt_abstime _state_changed{0};

	parameters_update();


    uint64_t set_time = 0;
    uint64_t timestamp_us = hrt_absolute_time();
    int input_port = _param_rc_chan.get();
    int output_port = _param_out_chan.get();

	control.control[4] = -1.0f;

	while (!should_exit()) {

		int pret = px4_poll(fds, 1, 300);
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		rpm_poll();
		airspeed_poll();
		vehicle_status_poll();
    	actuator_controls_poll();
    	input_rc_poll();
        manual_control_setpoint_poll();

        _rc_channel = (1500-_input_rc.values[input_port])/500.0f;
        _throttle = _actuator_controls.control[actuator_controls_s::INDEX_THROTTLE];
        _pitch_setpoint = _manual_control_setpoint.x;

        timestamp_us = hrt_absolute_time();

        bool rpm_flag = (state_flags >> state_types::RPM_OK) & 1U;

        if(_rpm.indicated_frequency_rpm > _param_min_rpm.get()){
          if(!rpm_flag){
              state_flags |= 1UL << state_types::RPM_OK;
              play_happy_tone();
          }
        } else {
          if(rpm_flag){
              state_flags &= ~(1UL << state_types::RPM_OK);
              play_sad_tone();
          }
        }

        bool rc_flag = (state_flags >> state_types::RC_OK) & 1U;
        if(_rc_channel > 0.5f){
          if(!rc_flag){
              state_flags |= 1UL << state_types::RC_OK;
              play_happy_tone();
          }
        } else {
          if(rc_flag){
              state_flags &= ~(1UL << state_types::RC_OK);
              play_sad_tone();
          }
        }


        if (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED){
    		if (_rpm.indicated_frequency_rpm > _param_min_rpm.get() && _airspeed.indicated_airspeed_m_s > _param_min_aspd.get() && _throttle >= 0.5f && _rc_channel >= 0.5f && _pitch_setpoint > 0){
                if(control.control[output_port] < 0.0f){
                    set_time = timestamp_us;
        			control.control[output_port] = 1.0f;
                }
    		} else {
                if ( (set_time + _param_latch_time.get()*1000) <  timestamp_us){
                    control.control[output_port] = -1.0f;
                    set_time = timestamp_us;
                }
    		}
        } else { // Pokud neni naarmovano
            set_time = timestamp_us;
            control.control[output_port] = _rc_channel;
        }

		control.timestamp = timestamp_us;
		control.timestamp_sample = timestamp_us;

		control_pub.publish(control);

		parameters_update();
	}

	//orb_unsubscribe(sensor_combined_sub);
	orb_unsubscribe(_rpm_sub);
	orb_unsubscribe(_airspeed_sub);
	orb_unsubscribe(_vehicle_status_sub);
	orb_unsubscribe(_actuator_controls_0_sub);
	orb_unsubscribe(_input_rc_sub);
}



void
AssistedRelease::rpm_poll()
{
	bool updated;
	orb_check(_rpm_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(rpm), _rpm_sub, &_rpm);
	}
}


void
AssistedRelease::airspeed_poll()
{
	bool updated;
	orb_check(_airspeed_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(airspeed), _airspeed_sub, &_airspeed);
	}
}

void
AssistedRelease::actuator_controls_poll()
{
	bool updated;
	orb_check(_actuator_controls_0_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_controls_0), _actuator_controls_0_sub, &_actuator_controls);
	}
}

void
AssistedRelease::vehicle_status_poll()
{
	bool updated;
	orb_check(_vehicle_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);
	}
}


void
AssistedRelease::input_rc_poll()
{
	bool updated;
	orb_check(_input_rc_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(input_rc), _input_rc_sub, &_input_rc);
	}
}


void
AssistedRelease::manual_control_setpoint_poll()
{
	bool updated;
	orb_check(_manual_control_setpoint_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_setpoint_sub, &_manual_control_setpoint);
	}
}


void AssistedRelease::parameters_update(bool force)
{
	if (_parameter_update_sub.updated() || force) {
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		updateParams();
	}
}


int AssistedRelease::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

This application is used to release the drone from the take-off platform.
Release conditions are based on certain parameters set in parameters.

### Implementation
When the parameters are exceeded. Application sets specific output.

### Examples
CLI usage example:
$ assisted_release start

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("assisted_release", "");
	PRINT_MODULE_USAGE_COMMAND("start");
	// PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	// PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int assisted_release_main(int argc, char *argv[])
{
	return AssistedRelease::main(argc, argv);
}
