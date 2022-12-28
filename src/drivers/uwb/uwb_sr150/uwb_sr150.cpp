/****************************************************************************
 *
 *   Copyright (c) 2020-2023 PX4 Development Team. All rights reserved.
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


/* This is a driver for the NXP SR150 UWB Chip on MK UWB Shield 2
 *	This Driver handles the Communication to the UWB Board.
 *	For Information about HW and SW contact Mobile Knowledge:
 *	https://www.themobileknowledge.com
 * */

#include "uwb_sr150.h"
#include <px4_platform_common/log.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/cli.h>
#include <errno.h>
#include <fcntl.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <ctype.h>
#include <string.h>

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

UWB_SR150::UWB_SR150(const char *port):
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_read_count_perf(perf_alloc(PC_COUNT, "uwb_sr150_count")),
	_read_err_perf(perf_alloc(PC_COUNT, "uwb_sr150_err"))
{
	/* store port name */
	strncpy(_port, port, sizeof(_port) - 1);
	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';
}

UWB_SR150::~UWB_SR150()
{
	printf("UWB: Ranging Stopped\t\n");

	// stop{}; will be implemented when this is changed to a scheduled work task
	perf_free(_read_err_perf);
	perf_free(_read_count_perf);

	close(_uart);
}

bool UWB_SR150::init()
{
	// execute Run() on every sensor_accel publication
	if (!_sensor_uwb_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	// alternatively, Run on fixed interval
	// ScheduleOnInterval(5000_us); // 2000 us interval, 200 Hz rate

	return true;
}

void UWB_SR150::start()
{
	/* schedule a cycle to start things */
	ScheduleNow();
}

void UWB_SR150::stop()
{
	ScheduleClear();
}

void UWB_SR150::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	if (_uart < 0) {
		/* open fd */
		_uart = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

		if (_uart < 0) {
			PX4_ERR("open failed (%i)", errno);
			return;
		}

		struct termios uart_config;

		int termios_state;

		/* fill the struct for the new configuration */
		tcgetattr(_uart, &uart_config);

		/* clear ONLCR flag (which appends a CR for every LF) */
		uart_config.c_oflag &= ~ONLCR;

		//TODO: should I keep this?
		/* no parity, one stop bit */
		uart_config.c_cflag &= ~(CSTOPB | PARENB);

		unsigned speed = DEFAULT_BAUD;

		/* set baud rate */
		if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d ISPD", termios_state);
		}

		if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d OSPD", termios_state);
		}

		if ((termios_state = tcsetattr(_uart, TCSANOW, &uart_config)) < 0) {
			PX4_ERR("baud %d ATTR", termios_state);
		}
	}

	/* perform collection */
	int collect_ret = collectData();

	if (collect_ret == -EAGAIN) {
		/* reschedule to grab the missing bits, time to transmit 8 bytes @ 9600 bps */
		ScheduleDelayed(1042 * 8);

		return;
	}

	if (OK != collect_ret) {

		/* we know the sensor needs about four seconds to initialize */
		if (hrt_absolute_time() > 5 * 1000 * 1000LL && _consecutive_fail_count < 5) {
			PX4_ERR("collection error #%u", _consecutive_fail_count);
		}

		_consecutive_fail_count++;

		/* restart the measurement state machine */
		start();
		return;

	} else {
		/* apparently success */
		_consecutive_fail_count = 0;
	}
}

int UWB_SR150::custom_command(int argc, char *argv[])
{
	return print_usage("Unrecognized command.");
}

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
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("status");
	return 0;
}

int UWB_SR150::task_spawn(int argc, char *argv[])
{
	int ch;
	int option_index = 1;
	const char *option_arg;
	const char *device_name = UWB_DEFAULT_PORT;
	int baudrate = 0;

	while ((ch = px4_getopt(argc, argv, "d:b", &option_index, &option_arg)) != EOF) {
		switch (ch) {
		case 'd':
			device_name = option_arg;
			break;

		case 'b':
			px4_get_parameter_value(option_arg, baudrate);
			break;

		default:
			PX4_WARN("Unrecognized flag: %c", ch);
			break;
		}
	}

	UWB_SR150 *instance = new UWB_SR150(device_name);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		instance->ScheduleOnInterval(5000_us);

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

int uwb_sr150_main(int argc, char *argv[])
{
	return UWB_SR150::main(argc, argv);
}

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

int UWB_SR150::collectData()
{
	uint8_t *buffer = (uint8_t *) &_distance_result_msg;

	FD_ZERO(&_uart_set);
	FD_SET(_uart, &_uart_set);
	_uart_timeout.tv_sec = MESSAGE_TIMEOUT_S ;
	_uart_timeout.tv_usec = MESSAGE_TIMEOUT_US;

	size_t buffer_location = 0;
	// There is a atleast 2000 clock cycles between 2 msg (20000/80mhz = 200uS)
	// Messages are only delimited by time. There is a chance that this driver starts up in the middle
	// of a message, with no way to know this other than time. There is also always the possibility of
	// transmission errors causing a dropped byte.
	// Here is the process for dealing with that:
	//  - Wait up to 1 second to start receiving a message
	//  - Once receiving a message, keep going until EITHER:
	//    - There is too large of a gap between bytes (Currently set to 5ms).
	//      This means the message is incomplete. Throw it out and start over.
	//    - 46 bytes are received (the size of the whole message).

	while (buffer_location < sizeof(_distance_result_msg)
	       && select(_uart + 1, &_uart_set, nullptr, nullptr, &_uart_timeout) > 0) {

		int bytes_read = read(_uart, &buffer[buffer_location], sizeof(_distance_result_msg) - buffer_location);

		if (bytes_read > 0) {
			buffer_location += bytes_read;

		} else {
			break;
		}

		FD_ZERO(&_uart_set);
		FD_SET(_uart, &_uart_set);
		_uart_timeout.tv_sec = 0;
		// Setting this timeout too high (> 37ms) will cause problems because the next message will start
		//  coming in, and overlap with the current message.
		// Setting this timeout too low (< 1ms) will cause problems because there is some delay between
		//  the individual bytes of a message, and a too-short timeout will cause the message to be truncated.
		// The current value of 5ms was found experimentally to never cut off a message prematurely.
		// Strictly speaking, there are no downsides to setting this timeout as high as possible (Just under 37ms),
		// because if this process is waiting, it means that the last message was incomplete, so there is no current
		// data waiting to be published. But we would rather set this timeout lower in case the UWB_SR150 board is
		// updated to publish data faster.
		_uart_timeout.tv_usec = BYTE_TIMEOUT_US;
	}

	perf_count(_read_count_perf);

	// All of the following criteria must be met for the message to be acceptable:
	//  - Size of message == sizeof(distance_msg_t) (36 bytes)
	//  - status == 0x00
	bool ok = (buffer_location == sizeof(distance_msg_t) && _distance_result_msg.stop == 0x1b);

	if (ok) {

		/* Ranging Message*/
		_sensor_uwb.timestamp 		= hrt_absolute_time();

		_sensor_uwb.sessionid 		= _distance_result_msg.sessionId;
		_sensor_uwb.time_offset 	= _distance_result_msg.range_interval;
		_sensor_uwb.counter 		= _distance_result_msg.seq_ctr;
		_sensor_uwb.mac			= _distance_result_msg.MAC;

		_sensor_uwb.mac_dest		= _distance_result_msg.measurements.MAC;
		_sensor_uwb.status		= _distance_result_msg.measurements.status;
		_sensor_uwb.nlos 		= _distance_result_msg.measurements.nLos;
		_sensor_uwb.distance 		= double(_distance_result_msg.measurements.distance) / 100;


		/*Angle of Arrival has Format Q9.7; dividing by 2^7 and negating results in the correct value*/
		_sensor_uwb.aoa_azimuth_dev 	= - double(_distance_result_msg.measurements.aoa_azimuth) / 128;
		_sensor_uwb.aoa_elevation_dev 	= - double(_distance_result_msg.measurements.aoa_elevation) / 128;
		_sensor_uwb.aoa_azimuth_resp 	= - double(_distance_result_msg.measurements.aoa_dest_azimuth) / 128;
		_sensor_uwb.aoa_elevation_resp 	= - double(_distance_result_msg.measurements.aoa_dest_elevation) / 128;


		/*Angle of Arrival has Format Q9.7; dividing by 2^7 and negating results in the correct value*/
		_sensor_uwb.aoa_azimuth_fom 		= - double(_distance_result_msg.measurements.aoa_azimuth) / 128;
		_sensor_uwb.aoa_elevation_fom 		= - double(_distance_result_msg.measurements.aoa_elevation) / 128;
		_sensor_uwb.aoa_dest_azimuth_fom	= - double(_distance_result_msg.measurements.aoa_dest_azimuth) / 128;
		_sensor_uwb.aoa_dest_elevation_fom	= - double(_distance_result_msg.measurements.aoa_dest_elevation) / 128;

		/* Sensor physical offset*/ //for now we propagate the physical configuration via Uorb
		_sensor_uwb.orientation		= _sensor_rot.get();
		_sensor_uwb.offset_x		= _offset_x.get();
		_sensor_uwb.offset_y		= _offset_y.get();
		_sensor_uwb.offset_z		= _offset_z.get();

		_sensor_uwb_pub.publish(_sensor_uwb);

	} else {
		perf_count(_read_err_perf);

		if (buffer_location == 0) {
			PX4_WARN("UWB module is not responding.");
		}
	}

	return 1;
}
