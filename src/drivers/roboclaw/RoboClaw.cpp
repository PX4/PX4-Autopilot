/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
#include <arch/board/board.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/debug_key_value.h>
#include <drivers/drv_hrt.h>
#include <math.h>

// The RoboClaw has a serial communication timeout of 10ms.
#define TIMEOUT_US 10000

// TODO: Make this a parameter
#define FAILED_TRANSACTION_RETRIES 1

// The RoboClaw determines the change in the wheel encoder value when it overflows
#define OVERFLOW_AMOUNT 0x100000000LL

// TODO: Delete this
//void printbytes(const char *msg, uint8_t *bytes, int numbytes)
//{
//	if (numbytes < 0) {
//		numbytes = 0;
//	}
//
//	size_t msglen = strlen(msg);
//	char buff[msglen + (4 * numbytes) + 1];
//	char *cur = &buff[0];
//	cur += sprintf(cur, "%s ", msg);
//
//	for (int i = 0; i < numbytes; i++) {
//		cur += sprintf(cur, "0x%02X ", bytes[i]);
//	}
//
//	PX4_INFO("%s", buff);
//}

RoboClaw::RoboClaw(const char *deviceName, uint16_t address, uint16_t pulsesPerRev):
	ScheduledWorkItem(px4::wq_configurations::hp_default),
	_address(address),
	_pulsesPerRev(pulsesPerRev),
	_uart(0),
	_uart_timeout{.tv_sec = 0, .tv_usec = TIMEOUT_US},
	_uart_mutex(PTHREAD_MUTEX_INITIALIZER),
	_controlPoll(),
	_actuators(ORB_ID(actuator_controls_0), 20),
	_lastEncoderCount{0, 0},
	_encoderCounts{0, 0},
	_motorSpeeds{0, 0}

{
	// setup control polling
	_controlPoll.fd = _actuators.getHandle();
	_controlPoll.events = POLLIN;

	// start serial port
	_uart = open(deviceName, O_RDWR | O_NOCTTY);

	if (_uart < 0) { err(1, "could not open %s", deviceName); }

	int ret = 0;
	struct termios uart_config {};
	ret = tcgetattr(_uart, &uart_config);

	if (ret < 0) { err(1, "failed to get attr"); }

	uart_config.c_oflag &= ~ONLCR; // no CR for every LF
	ret = cfsetispeed(&uart_config, B38400);

	if (ret < 0) { err(1, "failed to set input speed"); }

	ret = cfsetospeed(&uart_config, B38400);

	if (ret < 0) { err(1, "failed to set output speed"); }

	ret = tcsetattr(_uart, TCSANOW, &uart_config);

	if (ret < 0) { err(1, "failed to set attr"); }

	FD_ZERO(&_uart_set);

	pthread_mutex_init(&_uart_mutex, nullptr);

	// setup default settings, reset encoders
	resetEncoders();
}

RoboClaw::~RoboClaw()
{
	setMotorDutyCycle(MOTOR_1, 0.0);
	setMotorDutyCycle(MOTOR_2, 0.0);
	close(_uart);

	pthread_mutex_destroy(&_uart_mutex);
}

void RoboClaw::Run()
{
	readEncoder();
	//readEncoder(MOTOR_2);

	//PX4_INFO("Motor1: (%d, %d), Motor2: (%d, %d)", _motor1EncoderCounts, _motor1Revolutions, _motor2EncoderCounts,
	//	 _motor2Revolutions);
}

int RoboClaw::readEncoder()
{

	uint8_t rbuff[10];
	int nread = 0;

	for (int retry = 0; retry < FAILED_TRANSACTION_RETRIES && nread == 0; retry++) {
		nread =  _transaction(CMD_READ_BOTH_ENCODERS, nullptr, 0, &rbuff[0], 10, false, true);
	}

	if (nread < 10) {
		PX4_ERR("Error reading encoders: %d", nread);
		return -1;
	}

	uint32_t count;
	uint8_t *count_bytes;

	for (int motor = 0; motor <= 1; motor++) {
		count = 0;
		count_bytes = &rbuff[motor * 4];

		// Data from the roboclaw is big-endian. This converts the bytes to an integer, regardless of the
		// endianness of the system this code is running on.
		for (int byte = 0; byte < 4; byte++) {
			count = (count << 8) + count_bytes[byte];
		}

		// The Roboclaw stores encoder counts as unsigned 32-bit ints. This can overflow, especially when starting
		// at 0 and moving backward. The Roboclaw has overflow flags for this, but in my testing, they don't seem
		// to work. This code detects overflow manually.
		// These diffs are the difference between the count I just read from the Roboclaw and the last
		// count that was read from the roboclaw for this motor. fwd_diff assumes that the wheel moved forward,
		// and rev_diff assumes it moved backward. If the motor actually moved forward, then rev_diff will be close
		// to 2^32 (UINT32_MAX). If the motor actually moved backward, then fwd_diff will be close to 2^32.
		// To detect and account for overflow, I just assume that the smaller diff is correct.
		// Strictly speaking, if the wheel rotated more than 2^31 encoder counts since the last time I checked, this
		// will be wrong. But that's 1.7 million revolutions, so it probably won't come up.
		uint32_t fwd_diff = count - _lastEncoderCount[motor];
		uint32_t rev_diff = _lastEncoderCount[motor] - count;
		// At this point, abs(diff) is always <= 2^31, so this cast from unsigned to signed is safe.
		int32_t diff = fwd_diff <= rev_diff ? fwd_diff : -int32_t(rev_diff);
		_encoderCounts[motor] += diff;
//		PX4_INFO("Motor %d - LastCount: %7u, Count: %7u, Diff1: %8u, Diff2: %8u, diff: %4d, End counts: %4lld",
//			 motor, _lastEncoderCount[motor], count, fwd_diff, rev_diff, diff, _encoderCounts[motor]);
		_lastEncoderCount[motor] = count;
	}

	return 1;
}

void RoboClaw::printStatus(char *string, size_t n)
{
	snprintf(string, n, "pos1,spd1,pos2,spd2: %10.2f %10.2f %10.2f %10.2f\n",
		 double(getMotorPosition(MOTOR_1)),
		 double(getMotorSpeed(MOTOR_1)),
		 double(getMotorPosition(MOTOR_2)),
		 double(getMotorSpeed(MOTOR_2)));
}

float RoboClaw::getMotorPosition(e_motor motor)
{
	if (motor == MOTOR_1) {
		return _encoderCounts[0];

	} else if (motor == MOTOR_2) {
		return _encoderCounts[1];

	} else {
		warnx("Unknown motor value passed to RoboClaw::getMotorPosition");
		return NAN;
	}
}

float RoboClaw::getMotorSpeed(e_motor motor)
{
	if (motor == MOTOR_1) {
		return _motorSpeeds[0];

	} else if (motor == MOTOR_2) {
		return _motorSpeeds[1];

	} else {
		warnx("Unknown motor value passed to RoboClaw::getMotorPosition");
		return NAN;
	}
}

int RoboClaw::setMotorSpeed(e_motor motor, float value)
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
		return -1;
	}

	return _sendUnsigned7Bit(command, value);
}

int RoboClaw::setMotorDutyCycle(e_motor motor, float value)
{

	e_command command;

	// send command
	if (motor == MOTOR_1) {
		command = CMD_SIGNED_DUTYCYCLE_1;

	} else if (motor == MOTOR_2) {
		command = CMD_SIGNED_DUTYCYCLE_2;

	} else {
		return -1;
	}

	return _sendSigned16Bit(command, value);
}

int RoboClaw::drive(float value)
{
	e_command command = value >= 0 ? CMD_DRIVE_FWD_MIX : CMD_DRIVE_REV_MIX;
	return _sendUnsigned7Bit(command, value);
}

int RoboClaw::turn(float value)
{
	e_command command = value >= 0 ? CMD_TURN_LEFT : CMD_TURN_RIGHT;
	return _sendUnsigned7Bit(command, value);
}

int RoboClaw::resetEncoders()
{
	return _sendNothing(CMD_RESET_ENCODERS);
}

int RoboClaw::update()
{
	//TODO: Also update motor locations and speeds here

	// wait for an actuator publication,
	// check for exit condition every second
	// note "::poll" is required to distinguish global
	// poll from member function for driver
	if (::poll(&_controlPoll, 1, 1000) < 0) { return -1; } // poll error

	// if new data, send to motors
	if (_actuators.updated()) {
		_actuators.update();
		// setMotorDutyCycle(MOTOR_1, _actuators.get().control[actuator_controls_s::INDEX_]);
		// setMotorDutyCycle(MOTOR_2, _actuators.get().control[CH_VOLTAGE_RIGHT]);
		int drive_ret = drive(_actuators.get().control[actuator_controls_s::INDEX_THROTTLE]);
		int turn_ret = turn(_actuators.get().control[actuator_controls_s::INDEX_YAW]);

		if (drive_ret <= 0 || turn_ret <= 0) {
			PX4_ERR("Error controlling RoboClaw. Drive err: %d. Turn err: %d", drive_ret, turn_ret);
		}
	}

	Run();

	return 0;
}

int RoboClaw::_sendUnsigned7Bit(e_command command, float data)
{
	data = fabs(data);

	if (data > 1.0f) {
		data = 1.0f;
	}

	auto byte = (uint8_t)(data * 127);
	uint8_t recv_byte;
	return _transaction(command, &byte, 1, &recv_byte, 1);
}

int RoboClaw::_sendSigned16Bit(e_command command, float data)
{
	if (data > 1.0f) {
		data = 1.0f;

	} else if (data < -1.0f) {
		data = -1.0f;
	}

	auto buff = (uint16_t)(data * 32767);
	uint8_t recv_buff;
	return _transaction(command, (uint8_t *) &buff, 2, &recv_buff, 1);
}

int RoboClaw::_sendNothing(e_command command)
{
	uint8_t recv_buff;
	return _transaction(command, nullptr, 0, &recv_buff, 1);
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

int RoboClaw::_transaction(e_command cmd, uint8_t *wbuff, size_t wbytes,
			   uint8_t *rbuff, size_t rbytes, bool send_checksum, bool recv_checksum)
{
	// WRITE

	pthread_mutex_lock(&_uart_mutex);

	tcflush(_uart, TCIOFLUSH); // flush  buffers
	uint8_t buf[wbytes + 4];
	buf[0] = _address;
	buf[1] = cmd;

	if (wbuff) {
		memcpy(&buf[2], wbuff, wbytes);
	}

	wbytes = wbytes + (send_checksum ? 4 : 2);

	if (send_checksum) {
		uint16_t sum = _calcCRC(buf, wbytes - 2);
		buf[wbytes - 2] = (sum >> 8) & 0xFF;
		buf[wbytes - 1] = sum & 0xFFu;
	}

	int count = write(_uart, buf, wbytes);

	if (count < (int) wbytes) { // Did not successfully send all bytes.
		PX4_ERR("Only wrote %d out of %d bytes", count, (int) wbytes);
		pthread_mutex_unlock(&_uart_mutex);
		return -1;
	}

	// READ

	FD_ZERO(&_uart_set);
	FD_SET(_uart, &_uart_set);

	int rv = select(_uart + 1, &_uart_set, nullptr, nullptr, &_uart_timeout);

	//TODO: Clean up this mess of IFs and returns

	if (rv > 0) {
		// select() returns as soon as ANY bytes are available. I need to wait until ALL of the bytes are available.
		// TODO: Make sure this is not a busy wait.
		usleep(2000);
		int bytes_read = read(_uart, rbuff, rbytes);

		if (recv_checksum) {
			if (bytes_read < 2) {
				pthread_mutex_unlock(&_uart_mutex);
				return -1;
			}

			// The checksum sent back by the roboclaw is calculated based on the address and command bytes as well
			// as the data returned.
			uint16_t checksum_calc = _calcCRC(buf, 2);
			checksum_calc = _calcCRC(rbuff, bytes_read - 2, checksum_calc);
			uint16_t checksum_recv = (rbuff[bytes_read - 2] << 8) + rbuff[bytes_read - 1];

			if (checksum_calc == checksum_recv) {
				pthread_mutex_unlock(&_uart_mutex);
				return bytes_read;

			} else {
				//PX4_ERR("Invalid checksum. Expected 0x%04X, got 0x%04X", checksum_calc, checksum_recv);
				pthread_mutex_unlock(&_uart_mutex);
				return -10;
			}

		} else {
			if (bytes_read == 1 && rbuff[0] == 0xFF) {
				pthread_mutex_unlock(&_uart_mutex);
				return 1;

			} else {
				pthread_mutex_unlock(&_uart_mutex);
				return -11;
			}
		}

	} else {
		pthread_mutex_unlock(&_uart_mutex);
		return rv;
	}
}

int RoboClaw::roboclawTest(int argc, char *argv[])
{
	printf("roboclaw test: starting\n");

	// setup
	RoboClaw roboclaw("/dev/ttyS3", 128, 1200);
	roboclaw.setMotorDutyCycle(RoboClaw::MOTOR_1, 0.3);
	roboclaw.setMotorDutyCycle(RoboClaw::MOTOR_2, 0.3);
	char buf[200];

	for (int i = 0; i < 10; i++) {
		usleep(100000);
		roboclaw.readEncoder();
		//roboclaw.readEncoder(RoboClaw::MOTOR_2);
		roboclaw.printStatus(buf, 200);
		printf("%s", buf);
	}

	roboclaw.setMotorDutyCycle(RoboClaw::MOTOR_1, -0.3);
	roboclaw.setMotorDutyCycle(RoboClaw::MOTOR_2, -0.3);

	for (int i = 0; i < 10; i++) {
		usleep(100000);
		roboclaw.readEncoder();
		//roboclaw.readEncoder(RoboClaw::MOTOR_2);
		roboclaw.printStatus(buf, 200);
		printf("%s", buf);
	}

	roboclaw.setMotorDutyCycle(RoboClaw::MOTOR_1, 0.0);
	roboclaw.setMotorDutyCycle(RoboClaw::MOTOR_2, 0.0);

	printf("Test complete\n");

	return 0;
}
