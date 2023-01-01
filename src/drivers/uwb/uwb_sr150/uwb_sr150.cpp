/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
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

UWB_SR150::~UWB_SR150()
{
	printf("UWB: Ranging Stopped\t\n");

	//TODO enable stop part
	//int written = write(_uart, &CMD_APP_STOP, sizeof(CMD_APP_STOP));

	//if (written < (int) sizeof(CMD_APP_STOP)) {
	//PX4_ERR("Only wrote %d bytes out of %d.", written, (int) sizeof(CMD_APP_STOP));
	//}

	perf_free(_read_err_perf);
	perf_free(_read_count_perf);

	close(_uart);
}

void UWB_SR150::run()
{
	// Subscribe to parameter_update message
	parameters_update();
	param_timestamp = hrt_absolute_time();
	_uwb_mode = (_uwb_driver_mode)_uwb_mode_p.get();

	/* Ranging  Command */
	int status = FALSE;

	while (!should_exit()) {
		status = UWB_SR150::distance(); //evaluate Ranging Messages until Stop
	}

	if (!status) { printf("ERROR: Distance Failed"); }

	// Automatic Stop. This should not be reachable
	//
	//status = write(_uart, &CMD_RANGING_STOP, UWB_CMD_LEN);

	//if (status < (int) sizeof(CMD_RANGING_STOP)) {
	//	PX4_ERR("Only wrote %d bytes out of %d.", status, (int) sizeof(CMD_RANGING_STOP));
	//}
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
	PRINT_MODULE_USAGE_PARAM_STRING('p', nullptr, "<int>", "Position Debug: displays errors in Multilateration", false);
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("status");
	return 0;
}

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

int UWB_SR150::distance()
{

	if (hrt_elapsed_time(&param_timestamp) > 1000_ms) {
		parameters_update();
		param_timestamp = hrt_absolute_time();
	}

	_uwb_init_offset = matrix::Vector3d(_uwb_init_off_x.get(), _uwb_init_off_y.get(),
					    _uwb_init_off_z.get()); //set offset at the start
	_uwb_init_attitude = matrix::Vector3d(_uwb_init_off_yaw.get(), _uwb_init_off_pitch.get(), 0.0); //set UWB attitude
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
		_uwb_distance.timestamp = hrt_absolute_time();
		_uwb_distance.counter = _distance_result_msg.seq_ctr;
		_uwb_distance.sessionid = _distance_result_msg.sessionId;
		_uwb_distance.time_offset = _distance_result_msg.range_interval;
		//_uwb_distance.mac = _distance_result_msg.MAC


		_uwb_distance.distance = double(_distance_result_msg.measurements.distance) / 100;
		_uwb_distance.nlos = _distance_result_msg.measurements.nLos;
		/*Angle of Arrival has Format Q9.7; dividing by 2^7 and negating results in the correct value*/
		_uwb_distance.aoa_azimuth_dev 	= - double(_distance_result_msg.measurements.aoa_azimuth) / 128;
		_uwb_distance.aoa_elevation_dev = - double(_distance_result_msg.measurements.aoa_elevation) / 128;
		_uwb_distance.aoa_azimuth_resp 	= - double(_distance_result_msg.measurements.aoa_dest_azimuth) / 128;
		_uwb_distance.aoa_elevation_resp = - double(_distance_result_msg.measurements.aoa_dest_elevation) / 128;

		//TODO doe something with the AoA FOM
		//FOM is angle measurement quality estimation

		switch (_uwb_mode) {
		case data: {
				_uwb_distance.status = 9;
				break;
			}

		case prec_nav: { //Precision landing mode
				_uwb_distance.status = 10;
				_rel_pos = UWB_SR150::localization(_uwb_distance.distance, _uwb_distance.aoa_azimuth_dev,
								   _uwb_distance.aoa_elevation_dev);
				_uwb_distance.position[0] = _rel_pos(0);
				_uwb_distance.position[1] = _rel_pos(1);
				_uwb_distance.position[2] = _rel_pos(2);
				break;
			}

		case follow_me: { // Follow me mode
				_uwb_distance.status = 11;
				actuator_control(_uwb_distance.distance, _uwb_distance.aoa_azimuth_dev,
						 _uwb_distance.aoa_elevation_dev);
				break;
			}

		default:
			_uwb_distance.status = _uwb_mode;
			break;
		}

		_uwb_distance_pub.publish(_uwb_distance);

	} else {
		perf_count(_read_err_perf);

		if (buffer_location == 0) {
			PX4_WARN("UWB module is not responding.");
		}
	}

	return 1;
}

void UWB_SR150::actuator_control(double distance, double azimuth, double elevation)
{
	/* UWB_SR150::actuator_control takes distance and angle measurements and publishes proportional thrust commands.
	can be used to make a rover follow an UWB receiver
	*/

	//Lots of Params to finetune the Follow me behavior
	double follow_distance_max =	_uwb_follow_distance_max.get();
	double follow_distance = 	_uwb_follow_distance.get();
	double follow_distance_min =	_uwb_follow_distance_min.get();
	double throttle_max =		_uwb_throttle.get();
	double thrust_heading =		_uwb_thrust_head.get(); // heading thrust multiplier
	double thrust_heading_min =		_uwb_thrust_head_min.get();
	double thrust_heading_max =		_uwb_thrust_head_max.get();
	double throttle_reverse =	_uwb_throttle_reverse.get();

	double heading = azimuth / 70; //normalize the AoA to -1..+1
	double throttle = 0;

	//simple heading control loop
	if (azimuth >= 0) { // Heading deadzone
		heading = heading * thrust_heading;
		heading = math::constrain(heading, thrust_heading_min, thrust_heading_max) ; //Limit heading to -1..+1

	} else if (azimuth <= 0) { // Heading deadzone
		heading = heading * thrust_heading;
		heading = math::constrain(heading, -thrust_heading_max, -thrust_heading_min) ;

	} else {
		heading = 0;
	}

	//simple Throttle control loop
	if (distance > follow_distance_max) {
		throttle = 0;

	} else if (distance <= follow_distance_min) {
		throttle =	throttle_reverse;

	} else if ((distance >= follow_distance_min) && (distance < follow_distance)) {
		throttle = 0;

	} else if ((distance >= follow_distance) && (distance < follow_distance_max)) {
		throttle = (distance - follow_distance) / 2; //Throttle will reach 100% over 2 meters

	}

	throttle = math::constrain(throttle, -throttle_max, throttle_max); //limit throttle to -1..1

	offboard_control_mode_s offboard_control_mode{}; //publish offboard control mode to enable it
	offboard_control_mode.timestamp = hrt_absolute_time();
	offboard_control_mode.actuator = true;
	_offboard_control_mode_pub.publish(offboard_control_mode);

	vehicle_status_s vehicle_status{};
	_vehicle_status_sub.copy(&vehicle_status);

	// Publish actuator controls only once in OFFBOARD
	if (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
		//TODO query frame type and disable if not rover

		/*
		vehicle_rates_setpoint_s vehicle_rates_setpoint{};
		vehicle_rates_setpoint.timestamp = hrt_absolute_time();
		vehicle_rates_setpoint.roll = heading;
		vehicle_rates_setpoint.pitch = heading;
		vehicle_rates_setpoint.yaw = heading;
		vehicle_rates_setpoint.thrust_body[0] = throttle * max_throttle * timeout;
		vehicle_rates_setpoint.thrust_body[1] = throttle * max_throttle * timeout;
		vehicle_rates_setpoint.thrust_body[2] = throttle * max_throttle * timeout;
		_vehicle_rates_setpoint_pub.publish(vehicle_rates_setpoint);
		*/

		actuator_controls_s actuator_controls{};
		actuator_controls.timestamp = hrt_absolute_time();
		actuator_controls.control[2] = heading; //yaw
		actuator_controls.control[3] = throttle; //Thrust
		_actuator_controls_pubs[0].publish(actuator_controls); //flight controls
	}

}

matrix::Vector3d UWB_SR150::localization(double distance, double azimuth_dev, double elevation_dev)
{
	/* UWB_SR150::localization takes distance and angle measurements and publishes position data.
	can be used to make a rover follow an UWB receiver
	*/
	double deg2rad = M_PI / 180.0;
	// Catch angle measurements at the end of the range and discard them
	/*
	if(60.0 > azimuth_dev  || -60.0 < azimuth_dev){
		return;
	}
	if(60.0 > elevation_dev  || -60.0 < elevation_dev){
		return;
	}*/


	double azimuth 	 = azimuth_dev * deg2rad; 	//subtract yaw offset and convert to rad
	double elevation = elevation_dev  * deg2rad; 	//subtract pitch offset and convert to rad

	matrix::Vector3d position(	sin(azimuth) * sin(elevation),
					cos(azimuth) * sin(elevation),
					-cos(elevation));

	position *= distance; //scale the vector to the distance
	//Output is the Coordinates of the Initiator in relation to the UWB Receiver in NED (North-East-Down) Framing

	// Now the position is the landing point relative to the vehicle.
	// so the only thing left is to add the Initiator offset
	position +=  matrix::Vector3d(_uwb_init_offset);

	return position;
}
