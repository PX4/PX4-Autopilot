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

// The RoboClaw has a serial communication timeout of 10ms.
#define TIMEOUT_US 10000

// The RoboClaw determines the change in the wheel encoder value when it overflows
#define OVERFLOW_AMOUNT 0x100000000LL

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
	_controlPoll(),
	_actuators(ORB_ID(actuator_controls_0), 20),
	_motor1EncoderCounts(0),
	_motor1Revolutions(0),
	_motor1Overflow(0),
	_motor1Speed(0),
	_motor2EncoderCounts(0),
	_motor2Revolutions(0),
	_motor2Overflow(0),
	_motor2Speed(0),
	_lastEncoderCount{0, 0},
	_localPosition{0, 0},
	_revolutions{0, 0}

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

	_uart_timeout.tv_sec = 0;
	_uart_timeout.tv_usec = TIMEOUT_US;

	pthread_mutex_init(&_uart_mutex, NULL);

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

	PX4_INFO("Motor1: (%d, %d), Motor2: (%d, %d)", _motor1EncoderCounts, _motor1Revolutions, _motor2EncoderCounts,
		 _motor2Revolutions);
}

int RoboClaw::readEncoder()
{

	uint8_t rbuff[10];
	int nread = _transaction(CMD_READ_BOTH_ENCODERS, nullptr, 0, &rbuff[0], 10, false, true);

	if (nread < 10) {
		PX4_ERR("Error reading encoders: %d", nread);
		return -1;
	}

	int32_t count;
	uint8_t *count_bytes;

	for (int motor = 0; motor <= 1; motor++) {
		count = 0;
		count_bytes = &rbuff[motor * 4];

		// Data from the roboclaw is big-endian. This converts the bytes to an integer, regardless of the
		// endianness of the system this code is running on.
		for (int byte = 0; byte < 4; byte++) {
			count = (count << 8) + count_bytes[byte];
		}

		int overflow = 0;

		if (count - _lastEncoderCount[motor] > OVERFLOW_AMOUNT / 2) {
			overflow = -1;

		} else if (_lastEncoderCount[motor] - count > OVERFLOW_AMOUNT / 2) {
			overflow = +1;
		}

		int64_t adjusted_count = int64_t(count) + (overflow * int64_t(OVERFLOW_AMOUNT));
		int32_t diff = int32_t(adjusted_count - int64_t(_lastEncoderCount[motor]));
		_localPosition[motor] += diff;
		_revolutions[motor] += _localPosition[motor] / _pulsesPerRev;
		_localPosition[motor] %= _pulsesPerRev;
		PX4_INFO("Motor %d - LastCount: %7d, Count: %7d, Overflow: %+1d, adjusted count: %+8lld, local: %4d, revs: %2lld",
			 motor, _lastEncoderCount[motor], count, overflow, adjusted_count, _localPosition[motor], _revolutions[motor]);
		_lastEncoderCount[motor] = count;


	}

	return 1;

//	e_command cmd = motor == MOTOR_1 ? CMD_READ_ENCODER_1 : CMD_READ_ENCODER_2;
//	uint8_t rbuf[7];
//
//	int nread = _transaction(cmd, nullptr, 0, rbuf, 7, false, true);
//
//	if (nread < 7) {
//		PX4_ERR("Error reading RoboClaw encoders: %d", nread);
//		return -1;
//	}
//
//	uint32_t count = 0;
//	auto countBytes = (uint8_t *)(&count);
//	countBytes[3] = rbuf[0];
//	countBytes[2] = rbuf[1];
//	countBytes[1] = rbuf[2];
//	countBytes[0] = rbuf[3];
//	uint8_t status = rbuf[4];
//
//	int overflowFlag = 0;
//
//	if (status & STATUS_UNDERFLOW) {
//		overflowFlag = -1;
//		PX4_INFO("=====UNDERFLOW=====");
//
//	} else if (status & STATUS_OVERFLOW) {
//		PX4_INFO("=====OVERFLOW=====");
//		overflowFlag = +1;
//	}
//
//	int32_t *encoderCount;
//	int32_t *revolutions;
//	int32_t *overflow;
//
//	if (motor == MOTOR_1) {
//		encoderCount = &_motor1EncoderCounts;
//		revolutions = &_motor1Revolutions;
//		overflow = &_motor1Overflow;
//
//	} else {
//		encoderCount = &_motor2EncoderCounts;
//		revolutions = &_motor2Revolutions;
//		overflow = &_motor2Overflow;
//	}
//
//	PX4_INFO("COUNT: %08X STATUS: %02X", count, status);
//
//	*overflow += overflowFlag;
//	int64_t totalCounts = int64_t(count) + (int64_t(*overflow) * OVERFLOW_AMOUNT);
//	PX4_INFO("Total counts: %lld", totalCounts);
//	*encoderCount = int32_t(totalCounts % _pulsesPerRev);
//	*revolutions = int32_t(totalCounts / _pulsesPerRev);
//
//	return 0;
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
		return _motor1EncoderCounts;

	} else if (motor == MOTOR_2) {
		return _motor2EncoderCounts;

	} else {
		warnx("Unknown motor value passed to RoboClaw::getMotorPosition");
		return NAN;
	}
}

float RoboClaw::getMotorSpeed(e_motor motor)
{
	if (motor == MOTOR_1) {
		return _motor1Speed;

	} else if (motor == MOTOR_2) {
		return _motor2Speed;

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

	uint8_t byte = (uint8_t)(data * 127);
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

	uint16_t buff = (uint16_t)(data * 32767);
	uint8_t recv_buff;
	return _transaction(command, (uint8_t *) &buff, 2, &recv_buff, 1);
}

int RoboClaw::_sendNothing(e_command command)
{
	uint8_t recv_buff;
	return _transaction(command, nullptr, 0, &recv_buff, 1);
}

uint16_t RoboClaw::_sumBytes(uint8_t *buf, size_t n, uint16_t init)
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
		uint16_t sum = _sumBytes(buf, wbytes - 2);
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

	int rv = select(_uart + 1, &_uart_set, NULL, NULL, &_uart_timeout);

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
			uint16_t checksum_calc = _sumBytes(buf, 2);
			checksum_calc = _sumBytes(rbuff, bytes_read - 2, checksum_calc);
			uint16_t checksum_recv = (rbuff[bytes_read - 2] << 8) + rbuff[bytes_read - 1];

			if (checksum_calc == checksum_recv) {
				pthread_mutex_unlock(&_uart_mutex);
				return bytes_read;

			} else {
				PX4_ERR("Invalid checksum. Expected 0x%04X, got 0x%04X", checksum_calc, checksum_recv);
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
