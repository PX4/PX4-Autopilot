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
#include "calibration/calibration.h"
#include "calibration/gyro_calibration.h"
#include "calibration/mag_calibration.h"
#include "calibration/accelerometer_calibration.h"
#include "calibration/esc_calibration.h"
#include "calibration/airspeed_calibration.h"
#include "calibration/rc_calibration.h"
#include "calibration/calibration_helper.h"
#include "temperature_calibration/temperature_calibration.h"

#include <px4_getopt.h>
#include <px4_log.h>
#include <drivers/drv_hrt.h>
#include <systemlib/mavlink_log.h>
#include <uORB/topics/calibration_status.h>
#include <uORB/topics/vehicle_status.h>

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
	: _status_display(_subscriber_handler)
{
}

int SendEvent::start()
{
	if (is_running()) {
		return 0;
	}

	// subscribe to the topics
	_subscriber_handler.subscribe();

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
		_subscriber_handler.unsubscribe();
		exit_and_cleanup();
		return;
	}

	if (blink_msg_state() == 2) {
		/* blinking LED message completed, restore normal state */
		rgbled_set_color_and_mode(led_control_s::COLOR_BLUE, led_control_s::MODE_BREATHE);
	}

	_subscriber_handler.check_for_updates();

	process_commands();

	_status_display.process();

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

		const hrt_abstime t_start = hrt_absolute_time();
		static constexpr hrt_abstime timeout = 2000000; /* 2secs */

		vehicle_status_s vehicle_status = {};

		do {
			if (_subscriber_handler.vehicle_status_updated()) {
				orb_copy(ORB_ID(vehicle_status), _subscriber_handler.get_vehicle_status_sub(), &vehicle_status);
			}

			if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_INIT) {
				break;
			}

			usleep(100);
		} while (hrt_elapsed_time(&t_start) > timeout);

		if (vehicle_status.arming_state != vehicle_status_s::ARMING_STATE_INIT) {
			answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_FAILED);
			break;
		}

		/* wait until the arming state is moved to ARMING_STATE_INIT */
		if ((int)(cmd.param1) == 1) {

			if (run_calibration("gyro calibration", do_gyro_calibration) == calibration_status_s::CALIBRATION_OK) {
				answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);

			} else {
				answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_FAILED);
			}

			break;

		} else if ((int)(cmd.param2) == 1) {
			if (run_calibration("mag calibration", do_mag_calibration) == calibration_status_s::CALIBRATION_OK) {
				answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);

			} else {
				answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_FAILED);
			}

			break;

		} else if ((int)(cmd.param4) == 2) {
			if (run_calibration("trim calibration", do_trim_calibration) == calibration_status_s::CALIBRATION_OK) {
				answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);

			} else {
				answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_FAILED);
			}

			break;

		} else if ((int)(cmd.param5) == 1) {
			if (run_calibration("accel calibration", do_accel_calibration) == calibration_status_s::CALIBRATION_OK) {
				answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);

			} else {
				answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_FAILED);
			}

			break;

		} else if ((int)(cmd.param5) == 2) {
			// board offset calibration
			if (run_calibration("level calibration", do_level_calibration) == calibration_status_s::CALIBRATION_OK) {
				answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);

			} else {
				answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_FAILED);
			}

		} else if ((int)(cmd.param6) == 1 || (int)(cmd.param6) == 2) {
			// TODO: param6 == 1 is deprecated, but we still accept it for a while (feb 2017)
			if (run_calibration("airspeed calibration", do_airspeed_calibration) == calibration_status_s::CALIBRATION_OK) {
				answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);

			} else {
				answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_FAILED);
			}

			break;

		} else if ((int)(cmd.param7) == 1) {
			if (run_calibration("esc calibration", do_esc_calibration) == calibration_status_s::CALIBRATION_OK) {
				answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);

			} else {
				answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_FAILED);
			}

			break;
		}

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
	struct vehicle_command_ack_s command_ack = {
		.timestamp = hrt_absolute_time(),
		.result_param2 = 0,
		.command = cmd.command,
		.result = (uint8_t)result,
		.from_external = false,
		.result_param1 = 0,
		.target_system = cmd.source_system,
		.target_component = cmd.source_component
	};

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
	PRINT_MODULE_USAGE_COMMAND_DESCR("gyro_calibration", "Run gyro calibration process");
	PRINT_MODULE_USAGE_COMMAND_DESCR("mag_calibration", "Run mag calibration process");
	PRINT_MODULE_USAGE_COMMAND_DESCR("accel_calibration", "Run accel calibration process");
	PRINT_MODULE_USAGE_COMMAND_DESCR("level_calibration", "Run level calibration process");
	PRINT_MODULE_USAGE_COMMAND_DESCR("esc_calibration", "Run esc calibration process");
	PRINT_MODULE_USAGE_COMMAND_DESCR("airspeed_calibration", "Run airspeed calibration process");
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

		struct vehicle_command_s cmd = {
			.timestamp = 0,
			.param5 = (float)((accel_calib || calib_all) ? vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION : NAN),
			.param6 = NAN,
			.param1 = (float)((gyro_calib || calib_all) ? vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION : NAN),
			.param2 = NAN,
			.param3 = NAN,
			.param4 = NAN,
			.param7 = (float)((baro_calib || calib_all) ? vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION : NAN),
			.command = vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION
		};

		orb_advert_t h = orb_advertise_queue(ORB_ID(vehicle_command), &cmd, vehicle_command_s::ORB_QUEUE_LENGTH);
		(void)orb_unadvertise(h);

	} else if (!strcmp(argv[0], "gyro_calibration")) {
		if (!is_running()) {
			PX4_ERR("background task not running");
			return -1;
		}

		run_calibration("gyro calibration", do_gyro_calibration);
	} else if (!strcmp(argv[0], "mag_calibration")) {
		if (!is_running()) {
			PX4_ERR("background task not running");
			return -1;
		}

		run_calibration("mag calibration", do_mag_calibration);
	} else if (!strcmp(argv[0], "accel_calibration")) {
		if (!is_running()) {
			PX4_ERR("background task not running");
			return -1;
		}

		run_calibration("accel calibration", do_accel_calibration);
	} else if (!strcmp(argv[0], "level_calibration")) {
		if (!is_running()) {
			PX4_ERR("background task not running");
			return -1;
		}

		run_calibration("level calibration", do_level_calibration);
	} else if (!strcmp(argv[0], "esc_calibration")) {
		if (!is_running()) {
			PX4_ERR("background task not running");
			return -1;
		}

		run_calibration("esc calibration", do_esc_calibration);
	} else if (!strcmp(argv[0], "airspeed_calibration")) {
		if (!is_running()) {
			PX4_ERR("background task not running");
			return -1;
		}

		run_calibration("airspeed calibration", do_airspeed_calibration);
	} else {
		print_usage("unrecognized command");
	}

	return 0;
}
