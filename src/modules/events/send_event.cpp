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

#include <px4_getopt.h>
#include <px4_log.h>
#include <drivers/drv_hrt.h>

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

SendEvent::SendEvent()
{
}

int SendEvent::start()
{
	if (is_running()) {
		return 0;
	}

	_vehicle_command_sub = orb_subscribe(ORB_ID(vehicle_command));

	// Kick off the cycling. We can call it directly because we're already in the work queue context
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
	_object = send_event;
}

void
SendEvent::cycle_trampoline(void *arg)
{
	SendEvent *obj = reinterpret_cast<SendEvent *>(arg);

	obj->cycle();
}

void SendEvent::cycle()
{
	if (should_exit()) {
		if (_vehicle_command_sub >= 0) {
			orb_unsubscribe(_vehicle_command_sub);
			_vehicle_command_sub = -1;
		}

		exit_and_cleanup();
		return;
	}

	process_commands();

	work_queue(LPWORK, &_work, (worker_t)&SendEvent::cycle_trampoline, this,
		   USEC2TICK(SEND_EVENT_INTERVAL_US));
}

void SendEvent::process_commands()
{
	struct vehicle_command_s cmd;
	bool updated;
	orb_check(_vehicle_command_sub, &updated);

	if (!updated) {
		return;
	}

	orb_copy(ORB_ID(vehicle_command), _vehicle_command_sub, &cmd);

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
	struct vehicle_command_ack_s command_ack;

	/* publish ACK */
	command_ack.command = cmd.command;
	command_ack.result = result;
	command_ack.timestamp = hrt_absolute_time();

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
It is currently only responsible for temperature calibration.

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


int send_event_main(int argc, char *argv[])
{
	return SendEvent::main(argc, argv);
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

		vehicle_command_s cmd = {};
		cmd.target_system = -1;
		cmd.target_component = -1;

		cmd.command = vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION;
		cmd.param1 = (gyro_calib || calib_all) ? vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION : NAN;
		cmd.param2 = NAN;
		cmd.param3 = NAN;
		cmd.param4 = NAN;
		cmd.param5 = (accel_calib || calib_all) ? vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION : NAN;
		cmd.param6 = NAN;
		cmd.param7 = (baro_calib || calib_all) ? vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION : NAN;

		orb_advert_t h = orb_advertise_queue(ORB_ID(vehicle_command), &cmd, vehicle_command_s::ORB_QUEUE_LENGTH);
		(void)orb_unadvertise(h);

	} else {
		print_usage("unrecognized command");
	}

	return 0;
}
