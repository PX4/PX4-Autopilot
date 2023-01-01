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

#include "uwb_sr150driver.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>

// Timeout between bytes. If there is more time than this between bytes, then this driver assumes
// that it is the boundary between messages.
// See uwb_sr150::run() for more detailed explanation.
#define BYTE_TIMEOUT_US 5000

// Amount of time to wait for a new message. If more time than this passes between messages, then this
// driver assumes that the UWB_SR150 module is disconnected.
// (Right now it does not do anything about this)
#define MESSAGE_TIMEOUT_S 10  //wait 10 seconds.
#define MESSAGE_TIMEOUT_US 1

// The default baudrate of the uwb_sr150 module before configuration
#define DEFAULT_BAUD B115200

extern "C" __EXPORT int uwb_sr150_main(int argc, char *argv[]);

// Unchanged
UWB_SR150::UWB_SR150(const char *device_name, speed_t baudrate, bool uwb_pos_debug):
	ModuleParams(nullptr),
	_read_count_perf(perf_alloc(PC_COUNT, "uwb_sr150_count")),
	_read_err_perf(perf_alloc(PC_COUNT, "uwb_sr150_err"))
{
	_uwb_pos_debug = uwb_pos_debug;
	// start serial port
	_uart = open(device_name, O_RDWR | O_NOCTTY);

	if (_uart < 0) { err(1, "could not open %s", device_name); }

	int ret = 0;
	struct termios uart_config {};
	ret = tcgetattr(_uart, &uart_config);

	if (ret < 0) { err(1, "failed to get attr"); }

	uart_config.c_oflag &= ~ONLCR; // no CR for every LF
	ret = cfsetispeed(&uart_config, baudrate);

	if (ret < 0) { err(1, "failed to set input speed"); }

	ret = cfsetospeed(&uart_config, baudrate);

	if (ret < 0) { err(1, "failed to set output speed"); }

	ret = tcsetattr(_uart, TCSANOW, &uart_config);

	if (ret < 0) { err(1, "failed to set attr"); }
}

//TODO: Make sure this works with new stop command
UWB_SR150::~UWB_SR150(){
	printf("UWB: Ranging Stopped\t\n");

	stop{};
	perf_free(_read_err_perf);
	perf_free(_read_count_perf);

	close(_uart);
}

//TODO: Contains part of destructor from 4A_uwb_sr150
UWB_SR150::Stop{
	//TODO enable stop part
	//int written = write(_uart, &CMD_APP_STOP, sizeof(CMD_APP_STOP));

	//if (written < (int) sizeof(CMD_APP_STOP)) {
	//PX4_ERR("Only wrote %d bytes out of %d.", written, (int) sizeof(CMD_APP_STOP));
	//}
	break;
}

// TODO
void UWB_SR150::run(){
}

// Unchanged
int UWB_SR150::custom_command(int argc, char *argv[])
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

	return print_usage("Unrecognized command.");
}

// Unchanged
int UWB_SR150::print_usage(const char *reason)
{
	if (reason) {
		printf("%s\n\n", reason);
	}

	PRINT_MODULE_USAGE_NAME("uwb", "driver");
	PRINT_MODULE_DESCRIPTION(R"DESC_STR(
### Description

Driver for NXP UWB_SR150 UWB positioning system. This driver publishes a `uwb_distance` message
whenever the UWB_SR150 has a position measurement available.

### Example

Start the driver with a given device:

$ uwb start -d /dev/ttyS2
	)DESC_STR");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<file:dev>", "Name of device for serial communication with UWB", false);
	PRINT_MODULE_USAGE_PARAM_STRING('b', nullptr, "<int>", "Baudrate for serial communication", false);
	PRINT_MODULE_USAGE_PARAM_STRING('p', nullptr, "<int>", "Position Debug: displays errors in Multilateration", false);
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("status");
	return 0;
}

// Unchanged
int UWB_SR150::task_spawn(int argc, char *argv[])
{
	int task_id = px4_task_spawn_cmd(
			      "uwb_driver",
			      SCHED_DEFAULT,
			      SCHED_PRIORITY_DEFAULT,
			      2048,
			      &run_trampoline,
			      argv
		      );

	if (task_id < 0) {
		return -errno;

	} else {
		_task_id = task_id;
		return 0;
	}
}

// Unchanged
UWB_SR150 *UWB_SR150::instantiate(int argc, char *argv[])
{
	int ch;
	int option_index = 1;
	const char *option_arg;
	const char *device_name = nullptr;
	bool error_flag = false;
	int baudrate = 0;
	bool uwb_pos_debug = false; // Display UWB position calculation debug Messages

	while ((ch = px4_getopt(argc, argv, "d:b:p", &option_index, &option_arg)) != EOF) {
		switch (ch) {
		case 'd':
			device_name = option_arg;
			break;

		case 'b':
			px4_get_parameter_value(option_arg, baudrate);
			break;

		case 'p':

			uwb_pos_debug = true;
			break;

		default:
			PX4_WARN("Unrecognized flag: %c", ch);
			error_flag = true;
			break;
		}
	}

	if (!error_flag && device_name == nullptr) {
		print_usage("Device name not provided. Using default Device: TEL1:/dev/ttyS4 \n");
		device_name = "TEL2";
		error_flag = true;
	}

	if (!error_flag && baudrate == 0) {
		printf("Baudrate not provided. Using default Baud: 115200 \n");
		baudrate = B115200;
	}

	if (!error_flag && uwb_pos_debug == true) {
		printf("UWB Position algorithm Debugging \n");
	}

	if (error_flag) {
		PX4_WARN("Failed to start UWB driver. \n");
		return nullptr;

	} else {
		PX4_INFO("Constructing UWB_SR150. Device: %s", device_name);
		return new UWB_SR150(device_name, baudrate, uwb_pos_debug);
	}
}

// TODO
int UWB_SR150::distance(){
}

// TODO
UWB_POS_ERROR_CODES UWB_SR150::localization(){

}

// Unchanged
int uwb_sr150_main(int argc, char *argv[])
{
	return UWB_SR150::main(argc, argv);
}

// Unchanged
void UWB_SR150::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// If any parameter updated, call updateParams() to check if
		// this class attributes need updating (and do so).
		updateParams();
	}
}

// TODO: Not included in either uwb driver.... do I need this? Print whether it's 4anchor mode or single anchor mode? (e.g. gps.cpp)
int UWB_SR150::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}
