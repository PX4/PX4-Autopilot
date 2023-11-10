/****************************************************************************
 *
 *   Copyright (C) 2013-2023 PX4 Development Team. All rights reserved.
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
#include <drivers/device/i2c.h>
#include <sys/select.h>
#include <sys/time.h>
#include <pthread.h>
#include <unistd.h>

#include <uORB/Subscription.hpp>

#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/wheel_encoders.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>

#include <uORB/topics/differential_drive_control.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <lib/mixer_module/mixer_module.hpp>


/**
 * This is a driver for the RoboClaw motor controller
 */
class RoboClaw : public ModuleBase<RoboClaw>, public OutputModuleInterface
{
public:
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

	/** error handeling*/
	enum class RoboClawError {
		Success,
		WriteError,
		ReadError,
		ChecksumError,
		ChecksumMismatch,
		UnexpectedError,
		ReadTimeout
	};

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	void Run() override;

	int init();

	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			unsigned num_outputs, unsigned num_control_groups_updated) override;

	/**
	 * set the speed of a motor, rev/sec
	 */
	void setMotorSpeed(e_motor motor, float value);

	/**
	 * set the duty cycle of a motor
	 */
	void setMotorDutyCycle(e_motor motor, float value);

	/**
	 * Drive both motors. +1 = full forward, -1 = full backward
	 */
	void drive(float value);

	/**
	 * Turn. +1 = full right, -1 = full left
	 */
	void turn(float value);

	/**
	 * reset the encoders
	 * @return status
	 */
	void resetEncoders();

	/**
	 * read data from serial
	 */
	int readEncoder();

private:
	static constexpr int MAX_ACTUATORS = 2;

	MixingOutput _mixing_output{"ROBOCLAW", MAX_ACTUATORS, *this, MixingOutput::SchedulingPolicy::Auto, false};

	char _storedDeviceName[256]; // Adjust size as necessary
	char _storedBaudRateParam[256]; // Adjust size as necessary

	int _timeout_counter = 0;

	bool _successfully_connected{false};

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

	int _uart_fd{0};
	fd_set _uart_fd_set;
	struct timeval _uart_fd_timeout;

	uORB::SubscriptionData<actuator_armed_s> _actuator_armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _differential_drive_control_sub{ORB_ID(differential_drive_control)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	differential_drive_control_s 		_diff_drive_control{};

	matrix::Vector2f _motor_control{0.0f, 0.0f};

	uORB::Publication<wheel_encoders_s> _wheel_encoders_pub {ORB_ID(wheel_encoders)};

	void _parameters_update();

	static uint16_t _calcCRC(const uint8_t *buf, size_t n, uint16_t init = 0);
	void _sendUnsigned7Bit(e_command command, float data);
	void _sendSigned16Bit(e_command command, float data);

	int32_t reverseInt32(uint8_t *buffer);

	bool _initialized{false};

	RoboClawError writeCommand(e_command cmd);
	RoboClawError writeCommandWithPayload(e_command cmd, uint8_t *wbuff, size_t bytes_to_write);

	void sendTransaction(e_command cmd, uint8_t *write_buffer, size_t bytes_to_write);
	int receiveTransaction(e_command cmd, uint8_t *read_buffer, size_t bytes_to_read);


	DEFINE_PARAMETERS(
		(ParamInt<px4::params::RBCLW_ADDRESS>) _param_rbclw_address,
		(ParamInt<px4::params::RBCLW_COUNTS_REV>) _param_rbclw_counts_rev
	)
};
