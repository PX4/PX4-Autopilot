/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
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
 * @file RoboClas.hpp
 *
 * RoboClaw Motor Driver
 *
 * references:
 * http://downloads.orionrobotics.com/downloads/datasheets/motor_controller_robo_claw_R0401.pdf
 *
 */

#pragma once

#include <poll.h>
#include <stdio.h>
#include <termios.h>
#include <lib/parameters/param.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/wheel_encoders.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <drivers/device/i2c.h>
#include <sys/select.h>
#include <sys/time.h>
#include <pthread.h>

/**
 * This is a driver for the RoboClaw motor controller
 */
class RoboClaw
{
public:

	void taskMain();
	static bool taskShouldExit;

	/** control channels */
	enum e_channel {
		CH_VOLTAGE_LEFT = 0,
		CH_VOLTAGE_RIGHT
	};

	/**  motors */
	enum e_motor {
		MOTOR_1 = 0,
		MOTOR_2
	};

	/**
	 * constructor
	 * @param deviceName the name of the
	 * 	serial port e.g. "/dev/ttyS2"
	 * @param address the adddress  of the motor
	 * 	(selectable on roboclaw)
	 * @param baudRateParam Name of the parameter that holds the baud rate of this serial port
	 */
	RoboClaw(const char *deviceName, const char *baudRateParam);

	/**
	 * deconstructor
	 */
	virtual ~RoboClaw();

	/**
	 * @return position of a motor, rev
	 */
	float getMotorPosition(e_motor motor);

	/**
	 * @return speed of a motor, rev/sec
	 */
	float getMotorSpeed(e_motor motor);

	/**
	 * set the speed of a motor, rev/sec
	 */
	int setMotorSpeed(e_motor motor, float value);

	/**
	 * set the duty cycle of a motor
	 */
	int setMotorDutyCycle(e_motor motor, float value);

	/**
	 * Drive both motors. +1 = full forward, -1 = full backward
	 */
	int drive(float value);

	/**
	 * Turn. +1 = full right, -1 = full left
	 */
	int turn(float value);

	/**
	 * reset the encoders
	 * @return status
	 */
	int resetEncoders();

	/**
	 * read data from serial
	 */
	int readEncoder();

	/**
	 * print status
	 */
	void printStatus(char *string, size_t n);

private:

	// commands
	// We just list the commands we want from the manual here.
	enum e_command {

		// simple
		CMD_DRIVE_FWD_1 = 0,
		CMD_DRIVE_REV_1 = 1,
		CMD_DRIVE_FWD_2 = 4,
		CMD_DRIVE_REV_2 = 5,

		CMD_DRIVE_FWD_MIX = 8,
		CMD_DRIVE_REV_MIX = 9,
		CMD_TURN_RIGHT = 10,
		CMD_TURN_LEFT = 11,

		// encoder commands
		CMD_READ_ENCODER_1 = 16,
		CMD_READ_ENCODER_2 = 17,
		CMD_READ_SPEED_1 = 18,
		CMD_READ_SPEED_2 = 19,
		CMD_RESET_ENCODERS = 20,
		CMD_READ_BOTH_ENCODERS = 78,
		CMD_READ_BOTH_SPEEDS = 79,

		// advanced motor control
		CMD_READ_SPEED_HIRES_1 = 30,
		CMD_READ_SPEED_HIRES_2 = 31,
		CMD_SIGNED_DUTYCYCLE_1 = 32,
		CMD_SIGNED_DUTYCYCLE_2 = 33,

		CMD_READ_STATUS = 90
	};

	struct {
		speed_t serial_baud_rate;
		int32_t counts_per_rev;
		int32_t encoder_read_period_ms;
		int32_t actuator_write_period_ms;
		int32_t address;
	} _parameters{};

	struct {
		param_t serial_baud_rate;
		param_t counts_per_rev;
		param_t encoder_read_period_ms;
		param_t actuator_write_period_ms;
		param_t address;
	} _param_handles{};

	int _uart;
	fd_set _uart_set;
	struct timeval _uart_timeout;

	/** actuator controls subscription */
	int _actuatorsSub{-1};
	actuator_controls_s _actuatorControls;

	int _armedSub{-1};
	actuator_armed_s _actuatorArmed;

	int _paramSub{-1};
	parameter_update_s _paramUpdate;

	uORB::PublicationMulti<wheel_encoders_s> _wheelEncodersAdv[2] { ORB_ID(wheel_encoders), ORB_ID(wheel_encoders)};
	wheel_encoders_s _wheelEncoderMsg[2];

	uint32_t _lastEncoderCount[2] {0, 0};
	int64_t _encoderCounts[2] {0, 0};
	int32_t _motorSpeeds[2] {0, 0};

	void _parameters_update();

	static uint16_t _calcCRC(const uint8_t *buf, size_t n, uint16_t init = 0);
	int _sendUnsigned7Bit(e_command command, float data);
	int _sendSigned16Bit(e_command command, float data);
	int _sendNothing(e_command);

	/**
	 * Perform a round-trip write and read.
	 *
	 * NOTE: This function is not thread-safe.
	 *
	 * @param cmd Command to send to the Roboclaw
	 * @param wbuff Write buffer. Must not contain command, address, or checksum. For most commands, this will be
	 *   one or two bytes. Can be null iff wbytes == 0.
	 * @param wbytes Number of bytes to write. Can be 0.
	 * @param rbuff Read buffer. Will be filled with the entire response, including a checksum if the Roboclaw sends
	 *   a checksum for this command.
	 * @param rbytes Maximum number of bytes to read.
	 * @param send_checksum If true, then the checksum will be calculated and sent to the Roboclaw.
	 *   This is an option because some Roboclaw commands expect no checksum.
	 * @param recv_checksum If true, then this function will calculate the checksum of the returned data and compare
	 *   it to the checksum received. If they are not equal, OR if fewer than 2 bytes were received, then an
	 *   error is returned.
	 *   If false, then this function will expect to read exactly one byte, 0xFF, and will return an error otherwise.
	 * @return If successful, then the number of bytes read from the Roboclaw is returned. If there is a timeout
	 *   reading from the Roboclaw, then 0 is returned. If there is an IO error, then a negative value is returned.
	 */
	int _transaction(e_command cmd, uint8_t *wbuff, size_t wbytes,
			 uint8_t *rbuff, size_t rbytes, bool send_checksum = true, bool recv_checksum = false);
};
