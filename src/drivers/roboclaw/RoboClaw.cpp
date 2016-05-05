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
#include <uORB/topics/actuator_controls_0.h>
#include <uORB/topics/debug_key_value.h>
#include <drivers/drv_hrt.h>

uint8_t RoboClaw::checksum_mask = 0x7f;

RoboClaw::RoboClaw(const char *deviceName, uint16_t address,
		   uint16_t pulsesPerRev):
	_address(address),
	_pulsesPerRev(pulsesPerRev),
	_uart(0),
	_controlPoll(),
	_actuators(ORB_ID(actuator_controls_0), 20),
	_motor1Position(0),
	_motor1Speed(0),
	_motor1Overflow(0),
	_motor2Position(0),
	_motor2Speed(0),
	_motor2Overflow(0)
{
	// setup control polling
	_controlPoll.fd = _actuators.getHandle();
	_controlPoll.events = POLLIN;

	// start serial port
	_uart = open(deviceName, O_RDWR | O_NOCTTY);

	if (_uart < 0) { err(1, "could not open %s", deviceName); }

	int ret = 0;
	struct termios uart_config;
	ret = tcgetattr(_uart, &uart_config);

	if (ret < 0) { err(1, "failed to get attr"); }

	uart_config.c_oflag &= ~ONLCR; // no CR for every LF
	ret = cfsetispeed(&uart_config, B38400);

	if (ret < 0) { err(1, "failed to set input speed"); }

	ret = cfsetospeed(&uart_config, B38400);

	if (ret < 0) { err(1, "failed to set output speed"); }

	ret = tcsetattr(_uart, TCSANOW, &uart_config);

	if (ret < 0) { err(1, "failed to set attr"); }

	// setup default settings, reset encoders
	resetEncoders();
}

RoboClaw::~RoboClaw()
{
	setMotorDutyCycle(MOTOR_1, 0.0);
	setMotorDutyCycle(MOTOR_2, 0.0);
	close(_uart);
}

int RoboClaw::readEncoder(e_motor motor)
{
	uint16_t sum = 0;

	if (motor == MOTOR_1) {
		_sendCommand(CMD_READ_ENCODER_1, nullptr, 0, sum);

	} else if (motor == MOTOR_2) {
		_sendCommand(CMD_READ_ENCODER_2, nullptr, 0, sum);
	}

	uint8_t rbuf[50];
	usleep(5000);
	int nread = read(_uart, rbuf, 50);

	if (nread < 6) {
		printf("failed to read\n");
		return -1;
	}

	//printf("received: \n");
	//for (int i=0;i<nread;i++) {
	//printf("%d\t", rbuf[i]);
	//}
	//printf("\n");
	uint32_t count = 0;
	uint8_t *countBytes = (uint8_t *)(&count);
	countBytes[3] = rbuf[0];
	countBytes[2] = rbuf[1];
	countBytes[1] = rbuf[2];
	countBytes[0] = rbuf[3];
	uint8_t status = rbuf[4];
	uint8_t checksum = rbuf[5];
	uint8_t checksum_computed = (sum + _sumBytes(rbuf, 5)) &
				    checksum_mask;

	// check if checksum is valid
	if (checksum != checksum_computed) {
		printf("checksum failed: expected %d got %d\n",
		       checksum, checksum_computed);
		return -1;
	}

	int overFlow = 0;

	if (status & STATUS_REVERSE) {
		//printf("roboclaw: reverse\n");
	}

	if (status & STATUS_UNDERFLOW) {
		//printf("roboclaw: underflow\n");
		overFlow = -1;

	} else if (status & STATUS_OVERFLOW) {
		//printf("roboclaw: overflow\n");
		overFlow = +1;
	}

	static int64_t overflowAmount = 0x100000000LL;

	if (motor == MOTOR_1) {
		_motor1Overflow += overFlow;
		_motor1Position = float(int64_t(count) +
					_motor1Overflow * overflowAmount) / _pulsesPerRev;

	} else if (motor == MOTOR_2) {
		_motor2Overflow += overFlow;
		_motor2Position = float(int64_t(count) +
					_motor2Overflow * overflowAmount) / _pulsesPerRev;
	}

	return 0;
}

void RoboClaw::printStatus(char *string, size_t n)
{
	snprintf(string, n,
		 "pos1,spd1,pos2,spd2: %10.2f %10.2f %10.2f %10.2f\n",
		 double(getMotorPosition(MOTOR_1)),
		 double(getMotorSpeed(MOTOR_1)),
		 double(getMotorPosition(MOTOR_2)),
		 double(getMotorSpeed(MOTOR_2)));
}

float RoboClaw::getMotorPosition(e_motor motor)
{
	if (motor == MOTOR_1) {
		return _motor1Position;

	} else if (motor == MOTOR_2) {
		return _motor2Position;

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
	uint16_t sum = 0;

	// bound
	if (value > 1) { value = 1; }

	if (value < -1) { value = -1; }

	uint8_t speed = fabs(value) * 127;

	// send command
	if (motor == MOTOR_1) {
		if (value > 0) {
			return _sendCommand(CMD_DRIVE_FWD_1, &speed, 1, sum);

		} else {
			return _sendCommand(CMD_DRIVE_REV_1, &speed, 1, sum);
		}

	} else if (motor == MOTOR_2) {
		if (value > 0) {
			return _sendCommand(CMD_DRIVE_FWD_2, &speed, 1, sum);

		} else {
			return _sendCommand(CMD_DRIVE_REV_2, &speed, 1, sum);
		}
	}

	return -1;
}

int RoboClaw::setMotorDutyCycle(e_motor motor, float value)
{
	uint16_t sum = 0;

	// bound
	if (value > 1) { value = 1; }

	if (value < -1) { value = -1; }

	int16_t duty = 1500 * value;

	// send command
	if (motor == MOTOR_1) {
		return _sendCommand(CMD_SIGNED_DUTYCYCLE_1,
				    (uint8_t *)(&duty), 2, sum);

	} else if (motor == MOTOR_2) {
		return _sendCommand(CMD_SIGNED_DUTYCYCLE_2,
				    (uint8_t *)(&duty), 2, sum);
	}

	return -1;
}

int RoboClaw::resetEncoders()
{
	uint16_t sum = 0;
	return _sendCommand(CMD_RESET_ENCODERS,
			    nullptr, 0, sum);
}

int RoboClaw::update()
{
	// wait for an actuator publication,
	// check for exit condition every second
	// note "::poll" is required to distinguish global
	// poll from member function for driver
	if (::poll(&_controlPoll, 1, 1000) < 0) { return -1; } // poll error

	// if new data, send to motors
	if (_actuators.updated()) {
		_actuators.update();
		setMotorDutyCycle(MOTOR_1, _actuators.get().control[CH_VOLTAGE_LEFT]);
		setMotorDutyCycle(MOTOR_2, _actuators.get().control[CH_VOLTAGE_RIGHT]);
	}

	return 0;
}

uint16_t RoboClaw::_sumBytes(uint8_t *buf, size_t n)
{
	uint16_t sum = 0;

	//printf("sum\n");
	for (size_t i = 0; i < n; i++) {
		sum += buf[i];
		//printf("%d\t", buf[i]);
	}

	//printf("total sum %d\n", sum);
	return sum;
}

int RoboClaw::_sendCommand(e_command cmd, uint8_t *data,
			   size_t n_data, uint16_t &prev_sum)
{
	tcflush(_uart, TCIOFLUSH); // flush  buffers
	uint8_t buf[n_data + 3];
	buf[0] = _address;
	buf[1] = cmd;

	for (size_t i = 0; i < n_data; i++) {
		buf[i + 2] = data[n_data - i - 1]; // MSB
	}

	uint16_t sum = _sumBytes(buf, n_data + 2);
	prev_sum += sum;
	buf[n_data + 2] = sum & checksum_mask;
	//printf("\nmessage:\n");
	//for (size_t i=0;i<n_data+3;i++) {
	//printf("%d\t", buf[i]);
	//}
	//printf("\n");
	return write(_uart, buf, n_data + 3);
}

int roboclawTest(const char *deviceName, uint8_t address,
		 uint16_t pulsesPerRev)
{
	printf("roboclaw test: starting\n");

	// setup
	RoboClaw roboclaw(deviceName, address, pulsesPerRev);
	roboclaw.setMotorDutyCycle(RoboClaw::MOTOR_1, 0.3);
	roboclaw.setMotorDutyCycle(RoboClaw::MOTOR_2, 0.3);
	char buf[200];

	for (int i = 0; i < 10; i++) {
		usleep(100000);
		roboclaw.readEncoder(RoboClaw::MOTOR_1);
		roboclaw.readEncoder(RoboClaw::MOTOR_2);
		roboclaw.printStatus(buf, 200);
		printf("%s", buf);
	}

	roboclaw.setMotorDutyCycle(RoboClaw::MOTOR_1, -0.3);
	roboclaw.setMotorDutyCycle(RoboClaw::MOTOR_2, -0.3);

	for (int i = 0; i < 10; i++) {
		usleep(100000);
		roboclaw.readEncoder(RoboClaw::MOTOR_1);
		roboclaw.readEncoder(RoboClaw::MOTOR_2);
		roboclaw.printStatus(buf, 200);
		printf("%s", buf);
	}

	printf("Test complete\n");
	return 0;
}

// vi:noet:smarttab:autoindent:ts=4:sw=4:tw=78
