/****************************************************************************
 *
 *   Copyright (c) 2013-2023 PX4 Development Team. All rights reserved.
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

/**
 * @file RoboClaw.cpp
 *
 * RoboClaw Motor Driver
 *
 * references:
 * http://downloads.orionrobotics.com/downloads/datasheets/motor_controller_robo_claw_R0401.pdf
 *
 */


#include "RoboClaw.hpp"
#include <poll.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>

#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>

#include <uORB/Publication.hpp>
#include <drivers/drv_hrt.h>
#include <math.h>
#include <mathlib/mathlib.h>

// The RoboClaw has a serial communication timeout of 10ms.
// Add a little extra to account for timing inaccuracy
#define TIMEOUT_US 10500

// If a timeout occurs during serial communication, it will immediately try again this many times
#define TIMEOUT_RETRIES 5

// If a timeout occurs while disarmed, it will try again this many times. This should be a higher number,
// because stopping when disarmed is pretty important.
#define STOP_RETRIES 10

// Number of bytes returned by the Roboclaw when sending command 78, read both encoders
#define ENCODER_MESSAGE_SIZE 10

// Number of bytes for commands 18 and 19, read speeds.
#define ENCODER_SPEED_MESSAGE_SIZE 7

using namespace time_literals;

RoboClaw::RoboClaw(const char *deviceName, const char *baudRateParam) :
	// ModuleParams(nullptr),
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default)
	// ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	strncpy(_storedDeviceName, deviceName, sizeof(_storedDeviceName) - 1);
	_storedDeviceName[sizeof(_storedDeviceName) - 1] = '\0'; // Ensure null-termination

	strncpy(_storedBaudRateParam, baudRateParam, sizeof(_storedBaudRateParam) - 1);
	_storedBaudRateParam[sizeof(_storedBaudRateParam) - 1] = '\0'; // Ensure null-termination
}

RoboClaw::~RoboClaw()
{
	close(_uart_fd);
}

int RoboClaw::init() {
	_uart_fd_timeout = { .tv_sec = 0, .tv_usec = TIMEOUT_US };

	int32_t baud_rate_parameter_value{0};
	int32_t baud_rate_posix{0};
	param_get(param_find(_storedBaudRateParam), &baud_rate_parameter_value);

	switch (baud_rate_parameter_value) {
	case 0: // Auto
	default:
		PX4_ERR("Please configure the port's baud_rate_parameter_value");
		break;

	case 2400:
		baud_rate_posix = B2400;
		break;

	case 9600:
		baud_rate_posix = B9600;
		break;

	case 19200:
		baud_rate_posix = B19200;
		break;

	case 38400:
		baud_rate_posix = B38400;
		break;

	case 57600:
		baud_rate_posix = B57600;
		break;

	case 115200:
		baud_rate_posix = B115200;
		break;

	case 230400:
		baud_rate_posix = B230400;
		break;

	case 460800:
		baud_rate_posix = B460800;
		break;
	}

	// start serial port
	_uart_fd = open(_storedDeviceName, O_RDWR | O_NOCTTY);

	if (_uart_fd < 0) { err(1, "could not open %s", _storedDeviceName); }

	int ret = 0;
	struct termios uart_config {};
	ret = tcgetattr(_uart_fd, &uart_config);

	if (ret < 0) { err(1, "failed to get attr"); }

	uart_config.c_oflag &= ~ONLCR; // no CR for every LF
	uart_config.c_cflag &= ~CRTSCTS;

	// Set baud rate
	ret = cfsetispeed(&uart_config, baud_rate_posix);
	if (ret < 0) { err(1, "failed to set input speed"); }

	ret = cfsetospeed(&uart_config, baud_rate_posix);
	if (ret < 0) { err(1, "failed to set output speed"); }

	ret = tcsetattr(_uart_fd, TCSANOW, &uart_config);
	if (ret < 0) { err(1, "failed to set attr"); }

	FD_ZERO(&_uart_fd_set);
	FD_SET(_uart_fd, &_uart_fd_set);

	// Make sure the Roboclaw is actually connected, so I don't just spam errors if it's not.
	uint8_t rbuff[6]; // Number of bytes for command 90 status message, read reads status of the roboclaw.
	int err_code = receiveTransaction(CMD_READ_STATUS, &rbuff[0], sizeof(rbuff));

	if (err_code <= 0) {
		PX4_ERR("Shutting down Roboclaw driver.");
		return PX4_ERROR;
	} else {
		PX4_INFO("Successfully connected");
		/* Schedule a cycle to start things. */
		_successfully_connected = true;
		return PX4_OK;
	}
}

int RoboClaw::task_spawn(int argc, char *argv[])
{
	const char *deviceName = argv[1];
	const char *baud_rate_parameter_value = argv[2];

	RoboClaw *instance = new RoboClaw(deviceName, baud_rate_parameter_value);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;
		// instance->ScheduleOnInterval(10_ms); // 100 Hz
		instance->ScheduleNow();
		return PX4_OK;

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	printf("Ending task_spawn");

	return PX4_ERROR;
}

bool RoboClaw::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			unsigned num_outputs, unsigned num_control_groups_updated)
{
	float left_motor_output = -((float)outputs[1] - 128.0f)/127.f;
	float right_motor_output = ((float)outputs[0] - 128.0f)/127.f;

	if(stop_motors){
		setMotorSpeed(MOTOR_1, 0.f);
		setMotorSpeed(MOTOR_2, 0.f);
	} else {
		// PX4_ERR("Left and Right motor speed %f %f", (double)left_motor_output, (double)right_motor_output);
		setMotorSpeed(MOTOR_1, left_motor_output);
		setMotorSpeed(MOTOR_2, right_motor_output);
	}
	return true;
}

void RoboClaw::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		_mixing_output.unregister();
		return;
	}

	_mixing_output.update();

	if(!_initialized){
		init();
		_initialized = true;
	}

		// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		updateParams();
	}

	_actuator_armed_sub.update();

	_mixing_output.updateSubscriptions(false);

	if (readEncoder() < 0) {
		PX4_ERR("Error reading encoders");
	}

}

int RoboClaw::readEncoder()
{
	uint8_t buffer_positon[ENCODER_MESSAGE_SIZE];
	uint8_t buffer_speed_right[ENCODER_SPEED_MESSAGE_SIZE];
	uint8_t buffer_speed_left[ENCODER_SPEED_MESSAGE_SIZE];

	if (receiveTransaction(CMD_READ_BOTH_ENCODERS, buffer_positon, ENCODER_MESSAGE_SIZE) < ENCODER_MESSAGE_SIZE) {
		return -1;
	}

	if (receiveTransaction(CMD_READ_SPEED_1, buffer_speed_right, ENCODER_SPEED_MESSAGE_SIZE) < ENCODER_SPEED_MESSAGE_SIZE) {
		return -1;
	}

	if (receiveTransaction(CMD_READ_SPEED_2, buffer_speed_left, ENCODER_SPEED_MESSAGE_SIZE) < ENCODER_SPEED_MESSAGE_SIZE) {
		return -1;
	}

	int32_t position_right = reverseInt32(&buffer_positon[0]);
	int32_t position_left = reverseInt32(&buffer_positon[4]);
	int32_t speed_right = reverseInt32(&buffer_speed_right[0]);
	int32_t speed_left = reverseInt32(&buffer_speed_left[0]);

	wheel_encoders_s wheel_encoders{};
	wheel_encoders.wheel_angle[0] = static_cast<float>(position_right) / _param_rbclw_counts_rev.get() * M_TWOPI_F;
	wheel_encoders.wheel_angle[1] = static_cast<float>(position_left) / _param_rbclw_counts_rev.get() * M_TWOPI_F;
	wheel_encoders.wheel_speed[0] = static_cast<float>(speed_right) / _param_rbclw_counts_rev.get() * M_TWOPI_F;
	wheel_encoders.wheel_speed[1] = static_cast<float>(speed_left) / _param_rbclw_counts_rev.get() * M_TWOPI_F;
	wheel_encoders.timestamp = hrt_absolute_time();
	_wheel_encoders_pub.publish(wheel_encoders);

	return 1;
}

int32_t RoboClaw::reverseInt32(uint8_t *buffer)
{
	return (buffer[0] << 24)
	       | (buffer[1] << 16)
	       | (buffer[2] << 8)
	       | buffer[3];
}

void RoboClaw::setMotorSpeed(e_motor motor, float value)
{
	e_command command;

	// send command
	if (motor == MOTOR_1) {
		if (value > 0) {
			command = CMD_DRIVE_FWD_1;

		} else {
			command = CMD_DRIVE_REV_1;
		}

	} else if (motor == MOTOR_2) {
		if (value > 0) {
			command = CMD_DRIVE_FWD_2;

		} else {
			command = CMD_DRIVE_REV_2;
		}

	} else {
		return;
	}

	_sendUnsigned7Bit(command, value);
}

void RoboClaw::setMotorDutyCycle(e_motor motor, float value)
{
	e_command command;

	// send command
	if (motor == MOTOR_1) {
		command = CMD_SIGNED_DUTYCYCLE_1;

	} else if (motor == MOTOR_2) {
		command = CMD_SIGNED_DUTYCYCLE_2;

	} else {
		return;
	}

	return _sendSigned16Bit(command, value);
}

void RoboClaw::drive(float value)
{
	e_command command = value >= 0 ? CMD_DRIVE_FWD_MIX : CMD_DRIVE_REV_MIX;
	_sendUnsigned7Bit(command, value);
}

void RoboClaw::turn(float value)
{
	e_command command = value >= 0 ? CMD_TURN_LEFT : CMD_TURN_RIGHT;
	_sendUnsigned7Bit(command, value);
}

void RoboClaw::resetEncoders()
{
	sendTransaction(CMD_RESET_ENCODERS, nullptr, 0);
}

void RoboClaw::_sendUnsigned7Bit(e_command command, float data)
{
	data = fabs(data);

	if (data >= 1.0f) {
		data = 0.99f;
	}

	auto byte = (uint8_t)(data * INT8_MAX);
	sendTransaction(command, &byte, 1);
}

void RoboClaw::_sendSigned16Bit(e_command command, float data)
{
	int16_t duty = math::constrain(data, -1.f, 1.f) * INT16_MAX;
	uint8_t buff[2];
	buff[0] = (duty >> 8) & 0xFF; // High byte
	buff[1] = duty & 0xFF; // Low byte
	sendTransaction(command, (uint8_t *) &buff, 2);
}

uint16_t RoboClaw::_calcCRC(const uint8_t *buf, size_t n, uint16_t init)
{
	uint16_t crc = init;

	for (size_t byte = 0; byte < n; byte++) {
		crc = crc ^ (((uint16_t) buf[byte]) << 8);

		for (uint8_t bit = 0; bit < 8; bit++) {
			if (crc & 0x8000) {
				crc = (crc << 1) ^ 0x1021;

			} else {
				crc = crc << 1;
			}
		}
	}

	return crc;
}

int RoboClaw::receiveTransaction(e_command cmd, uint8_t *read_buffer, size_t bytes_to_read)
{
	if (writeCommand(cmd) != RoboClawError::Success) {
		return -1;
	}

	size_t total_bytes_read = 0;

	while (total_bytes_read < bytes_to_read) {
		int select_status = select(_uart_fd + 1, &_uart_fd_set, nullptr, nullptr, &_uart_fd_timeout);

		if (select_status <= 0) {
			PX4_ERR("Select timeout %d\n", select_status);
			return -1;
		}

		int bytes_read = read(_uart_fd, &read_buffer[total_bytes_read], bytes_to_read - total_bytes_read);

		if (bytes_read <= 0) {
			PX4_ERR("Read timeout %d\n", select_status);
			return -1;
		}

		total_bytes_read += bytes_read;
	}

	if (total_bytes_read < 2) {
		PX4_ERR("Too short payload received\n");
		return -1;
	}

	// Verify checksum
	uint8_t address = static_cast<uint8_t>(_param_rbclw_address.get());
	uint8_t command = static_cast<uint8_t>(cmd);
	uint16_t checksum_calc = _calcCRC(&address, 1); // address
	checksum_calc = _calcCRC(&command, 1, checksum_calc); // command
	checksum_calc = _calcCRC(read_buffer, total_bytes_read - 2, checksum_calc); // received payload
	uint16_t checksum_recv = (read_buffer[total_bytes_read - 2] << 8) + read_buffer[total_bytes_read - 1];

	if (checksum_calc != checksum_recv) {
		PX4_ERR("Checksum mismatch\n");
		return -1;
	}

	return total_bytes_read;
}

void RoboClaw::sendTransaction(e_command cmd, uint8_t *write_buffer, size_t bytes_to_write)
{
	writeCommandWithPayload(cmd, write_buffer, bytes_to_write);
	int select_status = select(_uart_fd + 1, &_uart_fd_set, nullptr, nullptr, &_uart_fd_timeout);

	if (select_status <= 0) {
		PX4_ERR("ACK timeout");
		return;
	}

	uint8_t acknowledgement{0};
	int bytes_read = read(_uart_fd, &acknowledgement, 1);

	if ((bytes_read != 1) || (acknowledgement != 0xFF)) {
		PX4_ERR("ACK wrong");
		return;
	}
}

RoboClaw::RoboClawError RoboClaw::writeCommand(e_command cmd)
{
	uint8_t buffer[2];

	buffer[0] = (uint8_t)_param_rbclw_address.get();
	buffer[1] = cmd;

	size_t count = write(_uart_fd, buffer, 2);

	if (count < 2) {
		PX4_ERR("Only wrote %d out of %zu bytes", count, 2);
		return RoboClawError::WriteError;
	}

	return RoboClawError::Success;
}

RoboClaw::RoboClawError RoboClaw::writeCommandWithPayload(e_command cmd, uint8_t *wbuff, size_t bytes_to_write)
{
	size_t packet_size = 2 + bytes_to_write + 2;
	uint8_t buffer[packet_size];

	// Add address + command ID
	buffer[0] = (uint8_t) _param_rbclw_address.get();
	buffer[1] = cmd;

	// Add payload
	if (bytes_to_write > 0 && wbuff) {
		memcpy(&buffer[2], wbuff, bytes_to_write);
	}

	// Add checksum
	uint16_t sum = _calcCRC(buffer, packet_size - 2);
	buffer[packet_size - 2] = (sum >> 8) & 0xFF;
	buffer[packet_size - 1] = sum & 0xFFu;

	// Write to device
	size_t count = write(_uart_fd, buffer, packet_size);

	// Not all bytes sent
	if (count < packet_size) {
		PX4_ERR("Only wrote %d out of %d bytes", count, bytes_to_write);
		return RoboClawError::WriteError;
	}

	return RoboClawError::Success;
}

int RoboClaw::custom_command(int argc, char *argv[])
{
	return 0;
}

int RoboClaw::print_usage(const char *reason)
{
	return 0;
}

int RoboClaw::print_status()
{
	PX4_ERR("Running, Initialized: %f", (double)_initialized);
	return 0;
}

extern "C" __EXPORT int roboclaw_main(int argc, char *argv[])
{
	return RoboClaw::main(argc, argv);
}
