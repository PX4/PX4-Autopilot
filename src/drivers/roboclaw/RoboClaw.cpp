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
#include <arch/board/board.h>
#include <mavlink/mavlink_log.h>

#include <controllib/uorb/UOrbPublication.hpp>
#include <uORB/topics/debug_key_value.h>
#include <drivers/drv_hrt.h>

uint8_t RoboClaw::checksum_mask = 0x7f;

RoboClaw::RoboClaw(const char *deviceName, uint16_t address):
	_address(address),
	_uart(0),
	_controlPoll(),
	_actuators(NULL, ORB_ID(actuator_controls_0), 20),
	_motor1Position(0),
	_motor1Speed(0),
	_motor2Position(0),
	_motor2Speed(0)
{
	// setup control polling
	_controlPoll.fd = _actuators.getHandle();
	_controlPoll.events = POLLIN;

	// start serial port
	_uart = open(deviceName, O_RDWR | O_NOCTTY);
	struct termios uart_config;
	tcgetattr(_uart, &uart_config);
	uart_config.c_oflag &= ~ONLCR; // no CR for every LF
	cfsetispeed(&uart_config, B38400);
	cfsetospeed(&uart_config, B38400);
	tcsetattr(_uart, TCSANOW, &uart_config);

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
	uint8_t rbuf[6];
	int ret = read(_uart, rbuf, 6);
	uint32_t count = 0;
	uint8_t * countBytes = (uint8_t *)(&count);
	countBytes[3] = rbuf[0];
	countBytes[2] = rbuf[1];
	countBytes[1] = rbuf[2];
	countBytes[0] = rbuf[3];
	uint8_t status = rbuf[4];
	if ((sum + _sumBytes(rbuf,5)) & 
			checksum_mask == rbuf[5]) return -1;
	int overFlow = 0;
	if (status & STATUS_UNDERFLOW) {
		overFlow = -1;
	} else if (status & STATUS_OVERFLOW) {
		overFlow = +1;
	}
	if (motor == MOTOR_1) {
		_motor1Overflow += overFlow;
	} else if (motor == MOTOR_2) {
		_motor2Overflow += overFlow;
	}
	return ret;
}

void RoboClaw::status(char *string, size_t n)
{
	snprintf(string, n,
		 "motor 1 position:\t%10.2f\tspeed:\t%10.2f\n"
		 "motor 2 position:\t%10.2f\tspeed:\t%10.2f\n",
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
	}
}

float RoboClaw::getMotorSpeed(e_motor motor)
{
	if (motor == MOTOR_1) {
		return _motor1Speed;
	} else if (motor == MOTOR_2) {
		return _motor2Speed;
	}
}

int RoboClaw::setMotorSpeed(e_motor motor, float value)
{
	uint16_t sum = 0;
	// bound
	if (value > 1) value = 1;
	if (value < -1) value = -1;
	uint8_t speed = fabs(value)*127;
	// send command
	if (motor == MOTOR_1) {
		if (value > 0) {
			_sendCommand(CMD_DRIVE_FWD_1, &speed, 1, sum);
		} else {
			_sendCommand(CMD_DRIVE_REV_1, &speed, 1, sum);
		}
	} else if (motor == MOTOR_2) {
		if (value > 0) {
			_sendCommand(CMD_DRIVE_FWD_2, &speed, 1, sum);
		} else {
			_sendCommand(CMD_DRIVE_REV_2, &speed, 1, sum);
		}
	}
}

int RoboClaw::setMotorDutyCycle(e_motor motor, float value)
{
	uint16_t sum = 0;
	// bound
	if (value > 1) value = 1;
	if (value < -1) value = -1;
	int16_t duty = 1500*value;
	// send command
	if (motor == MOTOR_1) {
		_sendCommand(CMD_SIGNED_DUTYCYCLE_1,
				(uint8_t *)(&duty), 2, sum);
	} else if (motor == MOTOR_2) {
		_sendCommand(CMD_SIGNED_DUTYCYCLE_2,
				(uint8_t *)(&duty), 2, sum);
	}
}

void RoboClaw::update()
{
	// wait for an actuator publication,
	// check for exit condition every second
	// note "::poll" is required to distinguish global
	// poll from member function for driver
	if (::poll(&_controlPoll, 1, 1000) < 0) return; // poll error

	// if new data, send to motors
	if (_actuators.updated()) {
		_actuators.update();
		setMotorSpeed(MOTOR_1,_actuators.control[CH_VOLTAGE_LEFT]);
		setMotorSpeed(MOTOR_2,_actuators.control[CH_VOLTAGE_RIGHT]);
	}
}

uint16_t RoboClaw::_sumBytes(uint8_t * buf, size_t n)
{
	uint16_t sum = 0;
	for (size_t i=0;i<n;i++) {
		sum += buf[i];
	}
	return sum;
}

int RoboClaw::_sendCommand(e_command cmd, uint8_t * data, 
		size_t n_data, uint16_t & prev_sum)
{
	tcflush(_uart, TCIOFLUSH);
	uint8_t buf[n_data + 3];
	buf[0] = _address;
	buf[1] = cmd;
	for (int i=0;i<n_data;i++) {
		buf[i+2] = data[n_data - i - 1]; // MSB
	}
	uint16_t sum = _sumBytes(buf, n_data + 2);
	prev_sum += sum;
	buf[n_data + 2] = sum & checksum_mask;
	return write(_uart, buf, n_data + 3);
}

int roboclawTest(const char *deviceName, uint8_t address)
{
	printf("roboclaw test: starting\n");

	// setup
	RoboClaw roboclaw(deviceName, address);

	printf("Test complete\n");
	return 0;
}

// vi:noet:smarttab:autoindent:ts=4:sw=4:tw=78
