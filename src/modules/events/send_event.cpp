/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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

#include "send_event.h"
#include "temperature_calibration/temperature_calibration.h"

#include <math.h>

#include <px4_getopt.h>
#include <px4_log.h>
#include <drivers/drv_hrt.h>

namespace events
{

struct work_s SendEvent::_work = {};

// Run it at 30 Hz.
const unsigned SEND_EVENT_INTERVAL_US = 33000;


int SendEvent::task_spawn(int argc, char *argv[])
{
	int ret = work_queue(LPWORK, &_work, (worker_t)&SendEvent::initialize_trampoline, nullptr, 0);

	if (ret < 0) {
		return ret;
	}

	ret = wait_until_running();

	if (ret < 0) {
		return ret;
	}

	_task_id = task_id_is_work_queue;

	return 0;
}

SendEvent::SendEvent() : ModuleParams(nullptr)
{
	if (_param_ev_tsk_stat_dis.get()) {
		_status_display = new status::StatusDisplay(_subscriber_handler);
	}

	if (_param_ev_tsk_rc_loss.get()) {
		_rc_loss_alarm = new rc_loss::RC_Loss_Alarm(_subscriber_handler);
	}
}

SendEvent::~SendEvent()
{
	if (_status_display != nullptr) {
		delete _status_display;
	}

	if (_rc_loss_alarm != nullptr) {
		delete _rc_loss_alarm;
	}
}

int SendEvent::start()
{
	if (is_running()) {
		return 0;
	}

	// Subscribe to the topics.
	_subscriber_handler.subscribe();

	// Kick off the cycling. We can call it directly because we're already in the work queue context.
	cycle();

	return 0;
}

void SendEvent::initialize_trampoline(void *arg)
{
	SendEvent *send_event = new SendEvent();

	if (!send_event) {
		PX4_ERR("alloc failed");
		return;
	}

	send_event->start();
	_object.store(send_event);
}

void SendEvent::cycle_trampoline(void *arg)
{
	SendEvent *obj = reinterpret_cast<SendEvent *>(arg);

	obj->cycle();
}

void SendEvent::cycle()
{
	if (should_exit()) {
		_subscriber_handler.unsubscribe();
		exit_and_cleanup();
		return;
	}

	_subscriber_handler.check_for_updates();

	process_commands();

	if (_status_display != nullptr) {
		_status_display->process();
	}

	if (_rc_loss_alarm != nullptr) {
		_rc_loss_alarm->process();
	}

	work_queue(LPWORK, &_work, (worker_t)&SendEvent::cycle_trampoline, this,
		   USEC2TICK(SEND_EVENT_INTERVAL_US));
}

void SendEvent::process_commands()
{
	if (!_subscriber_handler.vehicle_command_updated()) {
		return;
	}

	struct vehicle_command_s cmd;

	orb_copy(ORB_ID(vehicle_command), _subscriber_handler.get_vehicle_command_sub(), &cmd);

	bool got_temperature_calibration_command = false, accel = false, baro = false, gyro = false;

	switch (cmd.command) {
	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION:
		if ((int)(cmd.param1) == vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION) {
			gyro = true;
			got_temperature_calibration_command = true;
		}

		if ((int)(cmd.param5) == vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION) {
			accel = true;
			got_temperature_calibration_command = true;
		}

		if ((int)(cmd.param7) == vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION) {
			baro = true;
			got_temperature_calibration_command = true;
		}

		if (got_temperature_calibration_command) {
			if (run_temperature_calibration(accel, baro, gyro) == 0) {
				answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);

			} else {
				answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_FAILED);
			}
		}

		break;
	}

}

void SendEvent::answer_command(const vehicle_command_s &cmd, unsigned result)
{
	/* publish ACK */
	vehicle_command_ack_s command_ack = {};
	command_ack.timestamp = hrt_absolute_time();
	command_ack.command = cmd.command;
	command_ack.result = (uint8_t)result;
	command_ack.target_system = cmd.source_system;
	command_ack.target_component = cmd.source_component;

	if (_command_ack_pub != nullptr) {
		orb_publish(ORB_ID(vehicle_command_ack), _command_ack_pub, &command_ack);

	} else {
		_command_ack_pub = orb_advertise_queue(ORB_ID(vehicle_command_ack), &command_ack,
						       vehicle_command_ack_s::ORB_QUEUE_LENGTH);
	}
}

int SendEvent::print_usage(const char *reason)
{
	if (reason) {
		printf("%s\n\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Background process running periodically on the LP work queue to perform housekeeping tasks.
It is currently only responsible for temperature calibration and tone alarm on RC Loss.

The tasks can be started via CLI or uORB topics (vehicle_command from MAVLink, etc.).
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("send_event", "system");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the background task");
	PRINT_MODULE_USAGE_COMMAND_DESCR("temperature_calibration", "Run temperature calibration process");
	PRINT_MODULE_USAGE_PARAM_FLAG('g', "calibrate the gyro", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('a', "calibrate the accel", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('b', "calibrate the baro (if none of these is given, all will be calibrated)", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int SendEvent::custom_command(int argc, char *argv[])
{
	if (!strcmp(argv[0], "temperature_calibration")) {

		if (!is_running()) {
			PX4_ERR("background task not running");
			return -1;
		}

		bool gyro_calib = false, accel_calib = false, baro_calib = false;
		bool calib_all = true;
		int myoptind = 1;
		int ch;
		const char *myoptarg = nullptr;

		while ((ch = px4_getopt(argc, argv, "abg", &myoptind, &myoptarg)) != EOF) {
			switch (ch) {
			case 'a':
				accel_calib = true;
				calib_all = false;
				break;

			case 'b':
				baro_calib = true;
				calib_all = false;
				break;

			case 'g':
				gyro_calib = true;
				calib_all = false;
				break;

			default:
				print_usage("unrecognized flag");
				return 1;
			}
		}

		vehicle_command_s vcmd = {};
		vcmd.timestamp = hrt_absolute_time();
		vcmd.param1 = (float)((gyro_calib || calib_all) ? vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION : NAN);
		vcmd.param2 = NAN;
		vcmd.param3 = NAN;
		vcmd.param4 = NAN;
		vcmd.param5 = ((accel_calib || calib_all) ? vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION : (double)NAN);
		vcmd.param6 = (double)NAN;
		vcmd.param7 = (float)((baro_calib || calib_all) ? vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION : NAN);
		vcmd.command = vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION;

		orb_advertise_queue(ORB_ID(vehicle_command), &vcmd, vehicle_command_s::ORB_QUEUE_LENGTH);

	} else {
		print_usage("unrecognized command");
	}

	return 0;
}

int send_event_main(int argc, char *argv[])
{
	return SendEvent::main(argc, argv);
}

} /* namespace events */
