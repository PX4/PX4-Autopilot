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
 * @file Roboclaw.cpp
 *
 * Roboclaw Motor Driver
 *
 * references:
 * http://downloads.orionrobotics.com/downloads/datasheets/motor_controller_robo_claw_R0401.pdf
 *
 */

#include "Roboclaw.hpp"
#include <termios.h>

Roboclaw::Roboclaw(const char *device_name, const char *bad_rate_parameter) :
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default)
{
	strncpy(_stored_device_name, device_name, sizeof(_stored_device_name) - 1);
	_stored_device_name[sizeof(_stored_device_name) - 1] = '\0'; // Ensure null-termination

	strncpy(_stored_baud_rate_parameter, bad_rate_parameter, sizeof(_stored_baud_rate_parameter) - 1);
	_stored_baud_rate_parameter[sizeof(_stored_baud_rate_parameter) - 1] = '\0'; // Ensure null-termination
}

Roboclaw::~Roboclaw()
{
	close(_uart_fd);
}

int Roboclaw::initializeUART()
{
	// The Roboclaw has a serial communication timeout of 10ms
	// Add a little extra to account for timing inaccuracy
	static constexpr int TIMEOUT_US = 11_ms;
	_uart_fd_timeout = { .tv_sec = 0, .tv_usec = TIMEOUT_US };

	int32_t baud_rate_parameter_value{0};
	int32_t baud_rate_posix{0};
	param_get(param_find(_stored_baud_rate_parameter), &baud_rate_parameter_value);

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
	_uart_fd = open(_stored_device_name, O_RDWR | O_NOCTTY);

	if (_uart_fd < 0) { err(1, "could not open %s", _stored_device_name); }

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

	// Make sure the device does respond
	static constexpr int READ_STATUS_RESPONSE_SIZE = 6;
	uint8_t response_buffer[READ_STATUS_RESPONSE_SIZE];

	if (receiveTransaction(Command::ReadStatus, response_buffer, READ_STATUS_RESPONSE_SIZE) < READ_STATUS_RESPONSE_SIZE) {
		PX4_ERR("No valid response, stopping driver");
		request_stop();
		return ERROR;

	} else {
		PX4_INFO("Successfully connected");
		return OK;
	}
}

bool Roboclaw::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			     unsigned num_outputs, unsigned num_control_groups_updated)
{
	float right_motor_output = ((float)outputs[0] - 128.0f) / 127.f;
	float left_motor_output = ((float)outputs[1] - 128.0f) / 127.f;

	if (stop_motors) {
		setMotorSpeed(Motor::Right, 0.f);
		setMotorSpeed(Motor::Left, 0.f);

	} else {
		setMotorSpeed(Motor::Right, right_motor_output);
		setMotorSpeed(Motor::Left, left_motor_output);
	}

	return true;
}

void Roboclaw::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		_mixing_output.unregister();
		return;
	}

	_mixing_output.update();

	if (!_uart_initialized) {
		initializeUART();
		_uart_initialized = true;
	}

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// Read from topic to clear updated flag
		parameter_update_s parameter_update;
		_parameter_update_sub.copy(&parameter_update);

		updateParams();
	}

	_actuator_armed_sub.update();
	_mixing_output.updateSubscriptions(false);

	if (readEncoder() != OK) {
		PX4_ERR("Error reading encoders");
	}
}

int Roboclaw::readEncoder()
{
	static constexpr int ENCODER_MESSAGE_SIZE = 10; // response size for ReadEncoderCounters
	static constexpr int ENCODER_SPEED_MESSAGE_SIZE = 7; // response size for CMD_READ_SPEED_{1,2}

	uint8_t buffer_positon[ENCODER_MESSAGE_SIZE];
	uint8_t buffer_speed_right[ENCODER_SPEED_MESSAGE_SIZE];
	uint8_t buffer_speed_left[ENCODER_SPEED_MESSAGE_SIZE];

	if (receiveTransaction(Command::ReadSpeedMotor1, buffer_speed_right,
			       ENCODER_SPEED_MESSAGE_SIZE) < ENCODER_SPEED_MESSAGE_SIZE) {
		return ERROR;
	}

	if (receiveTransaction(Command::ReadSpeedMotor2, buffer_speed_left,
			       ENCODER_SPEED_MESSAGE_SIZE) < ENCODER_SPEED_MESSAGE_SIZE) {
		return ERROR;
	}

	if (receiveTransaction(Command::ReadEncoderCounters, buffer_positon, ENCODER_MESSAGE_SIZE) < ENCODER_MESSAGE_SIZE) {
		return ERROR;
	}

	int32_t speed_right = swapBytesInt32(&buffer_speed_right[0]);
	int32_t speed_left = swapBytesInt32(&buffer_speed_left[0]);
	int32_t position_right = swapBytesInt32(&buffer_positon[0]);
	int32_t position_left = swapBytesInt32(&buffer_positon[4]);

	wheel_encoders_s wheel_encoders{};
	wheel_encoders.wheel_speed[0] = static_cast<float>(speed_right) / _param_rbclw_counts_rev.get() * M_TWOPI_F;
	wheel_encoders.wheel_speed[1] = static_cast<float>(speed_left) / _param_rbclw_counts_rev.get() * M_TWOPI_F;
	wheel_encoders.wheel_angle[0] = static_cast<float>(position_right) / _param_rbclw_counts_rev.get() * M_TWOPI_F;
	wheel_encoders.wheel_angle[1] = static_cast<float>(position_left) / _param_rbclw_counts_rev.get() * M_TWOPI_F;
	wheel_encoders.timestamp = hrt_absolute_time();
	_wheel_encoders_pub.publish(wheel_encoders);

	return OK;
}

void Roboclaw::setMotorSpeed(Motor motor, float value)
{
	Command command;

	// send command
	if (motor == Motor::Right) {
		if (value > 0) {
			command = Command::DriveForwardMotor1;

		} else {
			command = Command::DriveBackwardsMotor1;
		}

	} else if (motor == Motor::Left) {
		if (value > 0) {
			command = Command::DriveForwardMotor2;

		} else {
			command = Command::DriveBackwardsMotor2;
		}

	} else {
		return;
	}

	sendUnsigned7Bit(command, value);
}

void Roboclaw::setMotorDutyCycle(Motor motor, float value)
{
	Command command;

	// send command
	if (motor == Motor::Right) {
		command = Command::DutyCycleMotor1;

	} else if (motor == Motor::Left) {
		command = Command::DutyCycleMotor2;

	} else {
		return;
	}

	return sendSigned16Bit(command, value);
}

void Roboclaw::resetEncoders()
{
	sendTransaction(Command::ResetEncoders, nullptr, 0);
}

void Roboclaw::sendUnsigned7Bit(Command command, float data)
{
	data = fabs(data);

	if (data >= 1.0f) {
		data = 0.99f;
	}

	auto byte = (uint8_t)(data * INT8_MAX);
	sendTransaction(command, &byte, 1);
}

void Roboclaw::sendSigned16Bit(Command command, float data)
{
	int16_t value = math::constrain(data, -1.f, 1.f) * INT16_MAX;
	uint8_t buff[2];
	buff[0] = (value >> 8) & 0xFF; // High byte
	buff[1] = value & 0xFF; // Low byte
	sendTransaction(command, (uint8_t *) &buff, 2);
}

int Roboclaw::sendTransaction(Command cmd, uint8_t *write_buffer, size_t bytes_to_write)
{
	if (writeCommandWithPayload(cmd, write_buffer, bytes_to_write) != OK) {
		return ERROR;
	}

	return readAcknowledgement();
}

int Roboclaw::writeCommandWithPayload(Command command, uint8_t *wbuff, size_t bytes_to_write)
{
	size_t packet_size = 2 + bytes_to_write + 2;
	uint8_t buffer[packet_size];

	// Add address + command ID
	buffer[0] = (uint8_t) _param_rbclw_address.get();
	buffer[1] = static_cast<uint8_t>(command);

	// Add payload
	if (bytes_to_write > 0 && wbuff) {
		memcpy(&buffer[2], wbuff, bytes_to_write);
	}

	// Add checksum
	uint16_t sum = _calcCRC(buffer, packet_size - 2);
	buffer[packet_size - 2] = (sum >> 8) & 0xFF;
	buffer[packet_size - 1] = sum & 0xFFu;

	// Write to device
	size_t bytes_written = write(_uart_fd, buffer, packet_size);

	// Not all bytes sent
	if (bytes_written < packet_size) {
		PX4_ERR("Only wrote %d out of %d bytes", bytes_written, bytes_to_write);
		return ERROR;
	}

	return OK;
}

int Roboclaw::readAcknowledgement()
{
	int select_status = select(_uart_fd + 1, &_uart_fd_set, nullptr, nullptr, &_uart_fd_timeout);

	if (select_status <= 0) {
		PX4_ERR("ACK timeout");
		return ERROR;
	}

	uint8_t acknowledgement{0};
	int bytes_read = read(_uart_fd, &acknowledgement, 1);

	if ((bytes_read != 1) || (acknowledgement != 0xFF)) {
		PX4_ERR("ACK wrong");
		return ERROR;
	}

	return OK;
}

int Roboclaw::receiveTransaction(Command command, uint8_t *read_buffer, size_t bytes_to_read)
{
	if (writeCommand(command) != OK) {
		return ERROR;
	}

	return readResponse(command, read_buffer, bytes_to_read);
}

int Roboclaw::writeCommand(Command command)
{
	uint8_t buffer[2];

	// Just address + command ID
	buffer[0] = (uint8_t)_param_rbclw_address.get();
	buffer[1] = static_cast<uint8_t>(command);

	size_t bytes_written = write(_uart_fd, buffer, 2);

	if (bytes_written < 2) {
		PX4_ERR("Only wrote %d out of %d bytes", bytes_written, 2);
		return ERROR;
	}

	return OK;
}

int Roboclaw::readResponse(Command command, uint8_t *read_buffer, size_t bytes_to_read)
{
	size_t total_bytes_read = 0;

	while (total_bytes_read < bytes_to_read) {
		int select_status = select(_uart_fd + 1, &_uart_fd_set, nullptr, nullptr, &_uart_fd_timeout);

		if (select_status <= 0) {
			PX4_ERR("Select timeout %d\n", select_status);
			return ERROR;
		}

		int bytes_read = read(_uart_fd, &read_buffer[total_bytes_read], bytes_to_read - total_bytes_read);

		if (bytes_read <= 0) {
			PX4_ERR("Read timeout %d\n", select_status);
			return ERROR;
		}

		total_bytes_read += bytes_read;
	}

	if (total_bytes_read < 2) {
		PX4_ERR("Too short payload received\n");
		return ERROR;
	}

	// Verify response checksum
	uint8_t address = static_cast<uint8_t>(_param_rbclw_address.get());
	uint8_t command_byte = static_cast<uint8_t>(command);
	uint16_t crc_calculated = _calcCRC(&address, 1); // address
	crc_calculated = _calcCRC(&command_byte, 1, crc_calculated); // command
	crc_calculated = _calcCRC(read_buffer, total_bytes_read - 2, crc_calculated); // received payload
	uint16_t crc_received = (read_buffer[total_bytes_read - 2] << 8) + read_buffer[total_bytes_read - 1];

	if (crc_calculated != crc_received) {
		PX4_ERR("Checksum mismatch\n");
		return ERROR;
	}

	return total_bytes_read;
}

uint16_t Roboclaw::_calcCRC(const uint8_t *buffer, size_t bytes, uint16_t init)
{
	uint16_t crc = init;

	for (size_t byte = 0; byte < bytes; byte++) {
		crc = crc ^ (((uint16_t) buffer[byte]) << 8);

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

int32_t Roboclaw::swapBytesInt32(uint8_t *buffer)
{
	return (buffer[0] << 24)
	       | (buffer[1] << 16)
	       | (buffer[2] << 8)
	       | buffer[3];
}

int Roboclaw::task_spawn(int argc, char *argv[])
{
	const char *device_name = argv[1];
	const char *baud_rate_parameter_value = argv[2];

	Roboclaw *instance = new Roboclaw(device_name, baud_rate_parameter_value);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;
		instance->ScheduleNow();
		return OK;

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	printf("Ending task_spawn");

	return ERROR;
}

int Roboclaw::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int Roboclaw::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
### Description

This driver communicates over UART with the [Roboclaw motor driver](https://www.basicmicro.com/motor-controller).
It performs two tasks:

 - Control the motors based on the OutputModuleInterface.
 - Read the wheel encoders and publish the raw data in the `wheel_encoders` uORB topic

In order to use this driver, the Roboclaw should be put into Packet Serial mode (see the linked documentation), and
your flight controller's UART port should be connected to the Roboclaw as shown in the documentation.
The driver needs to be enabled using the parameter `RBCLW_SER_CFG`, the baudrate needs to be set correctly and
the address `RBCLW_ADDRESS` needs to match the ESC configuration.

The command to start this driver is: `$ roboclaw start <UART device> <baud rate>`
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("roboclaw", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

int Roboclaw::print_status()
{
	return 0;
}

extern "C" __EXPORT int roboclaw_main(int argc, char *argv[])
{
	return Roboclaw::main(argc, argv);
}
