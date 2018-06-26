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



#include "rc_loss_alarm.h"

#include <px4_defines.h>

#include <drivers/drv_hrt.h>
#include <errno.h>
#include <stdint.h>

#include <uORB/topics/tune_control.h>

// Init static members
work_s RC_Loss_Alarm::_work = {};
bool RC_Loss_Alarm::_was_armed = false;
orb_advert_t RC_Loss_Alarm::_tune_control_pub = nullptr;


RC_Loss_Alarm::RC_Loss_Alarm()
{
}

RC_Loss_Alarm::~RC_Loss_Alarm()
{
	work_cancel(LPWORK, &_work);

	orb_unsubscribe(_vehicle_status_sub);

	orb_unadvertise(_tune_control_pub);
}

/** @see ModuleBase */
int RC_Loss_Alarm::task_spawn(int argc, char *argv[])
{
	// Instantiate task
	RC_Loss_Alarm *rc_loss_alarm = nullptr;

	if (_object == nullptr) {
		rc_loss_alarm = new RC_Loss_Alarm();

		if (rc_loss_alarm == nullptr) {
			PX4_ERR("failed to start flow");
			return PX4_ERROR;
		}
	}

	_object = rc_loss_alarm;

	// schedule a cycle to start things
	int ret = work_queue(LPWORK, &_work,
			     (worker_t)&RC_Loss_Alarm::cycle_trampoline, rc_loss_alarm, 0);

	if (ret < 0) {
		return ret;
	}

	// Since this task runs on the work-queue rather than in a separate thread,
	// we specify the workqueue task_id
	_task_id = task_id_is_work_queue;

	// wait until task is up & running (the mode_* commands depend on it)
	if (wait_until_running() < 0) {
		return -1;
	}

	return PX4_OK;
}

/** @see ModuleBase */
int RC_Loss_Alarm::custom_command(int argc, char *argv[])
{
	if (!strcmp(argv[0], "test")) {
		RC_Loss_Alarm::pub_tune();
		return PX4_OK;

	} else if (!strcmp(argv[0], "reset")) {
		return RC_Loss_Alarm::reset_module();
	}

	return print_usage("unknown command");
}

/** @see ModuleBase */
int RC_Loss_Alarm::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
RC Loss Alarm plays a loud error tune in the event that the drone loses RC
link after disarming. Ideal for finding a lost drone.
### Example
The module is typically started with:
rc_loss_alarm start
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rc_loss_alarm", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the task");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop the task");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Display running status of task");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test", "Start alarm tune, can be stopped with 'reset'");
	PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Resets internal state and stops an eventual alarm");
	return PX4_OK;
}

void RC_Loss_Alarm::cycle_trampoline(void *arg)
{
	RC_Loss_Alarm *dev = reinterpret_cast<RC_Loss_Alarm *>(arg);
	dev->cycle();
}

void RC_Loss_Alarm::cycle()
{
  // Subscribe if necessary
  if(_vehicle_status_sub == -1){
    _vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
  }

  // Check armed status
  bool updated = false;

  orb_check(_vehicle_status_sub, &updated);
  if(updated){
    orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);

    if (!_was_armed &&
        _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED){

      _was_armed = true;  // Once true, impossible to go back to false
    }

    if (_was_armed && _vehicle_status.rc_signal_lost &&
        _vehicle_status.arming_state != vehicle_status_s::ARMING_STATE_ARMED){

      pub_tune();
    }
  }

  // Schedule next cycle unless module is shutting down
	if (!should_exit()) {
			work_queue(LPWORK, &_work, (worker_t)&RC_Loss_Alarm::cycle_trampoline, this,
				   USEC2TICK(UPDATE_RATE));
	}
}

void RC_Loss_Alarm::pub_tune()
{
  struct tune_control_s tune_control = {};
	tune_control.tune_id = static_cast<int>(TuneID::ERROR_TUNE);
	tune_control.strength = tune_control_s::STRENGTH_MAX;
	tune_control.tune_override = 1;
	tune_control.timestamp = hrt_absolute_time();

  if (_tune_control_pub == nullptr) {
    _tune_control_pub = orb_advertise(ORB_ID(tune_control), &tune_control);
  }else{
    orb_publish(ORB_ID(tune_control), _tune_control_pub, &tune_control);
  }
}

void RC_Loss_Alarm::stop_tune()
{
  struct tune_control_s tune_control = {};
  tune_control.tune_id = static_cast<int>(TuneID::CUSTOM);
  tune_control.frequency = 0;
  tune_control.duration = 0;
  tune_control.silence = 0;
  tune_control.tune_override = true;

  if (_tune_control_pub == nullptr) {
    _tune_control_pub = orb_advertise(ORB_ID(tune_control), &tune_control);
  }else{
    orb_publish(ORB_ID(tune_control), _tune_control_pub, &tune_control);
  }
}

int RC_Loss_Alarm::reset_module(){
  RC_Loss_Alarm::_was_armed = false;
  RC_Loss_Alarm::stop_tune();
  return PX4_OK;
}

// module 'main' command
extern "C" __EXPORT int rc_loss_alarm_main(int argc, char *argv[]);
int rc_loss_alarm_main(int argc, char *argv[])
{
	return RC_Loss_Alarm::main(argc, argv);
}
