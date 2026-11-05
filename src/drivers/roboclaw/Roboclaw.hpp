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
 * @file Roboclaw.hpp
 *
 * Roboclaw motor control driver
 *
 * Product page: https://www.basicmicro.com/motor-controller
 * Manual: https://downloads.basicmicro.com/docs/roboclaw_user_manual.pdf
 */

#pragma once

#include <lib/mixer_module/mixer_module.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/Serial.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>

#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/wheel_encoders.h>

class Roboclaw : public ModuleBase, public OutputModuleInterface
{
public:
	static Descriptor desc;

	Roboclaw(const char *device_name);
	~Roboclaw() override;

	static int task_spawn(int argc, char *argv[]); ///< @see ModuleBase
	static int custom_command(int argc, char *argv[]); ///< @see ModuleBase
	static int print_usage(const char *reason = nullptr); ///< @see ModuleBase
	int print_status() override; ///< @see ModuleBase

	void Run() override;

	void reversibleMaskChanged(uint32_t reversible_mask) override;
	void mixerChanged() override;

	/** @see OutputModuleInterface */
	bool updateOutputs(float outputs[MAX_ACTUATORS], unsigned num_outputs, unsigned num_control_groups_updated) override;

private:
	enum class Command : uint8_t {
		ReadStatus = 90,

		DriveForwardMotor1 = 0,
		DriveBackwardsMotor1 = 1,
		DriveForwardMotor2 = 4,
		DriveBackwardsMotor2 = 5,

		ReadSpeedMotor1 = 18,
		ReadSpeedMotor2 = 19,
		ReadEncoderCounters = 78,
	};

	static constexpr unsigned MAX_CONTROLLERS = 8;
	static constexpr unsigned CHANNELS_PER_CONTROLLER = 2;
	static constexpr unsigned NUM_OUTPUTS = MAX_CONTROLLERS * CHANNELS_PER_CONTROLLER;
	static constexpr uint32_t TRANSACTION_TIMEOUT_MS = 100; // Allows a complete encoder response even at 2400 baud.
	MixingOutput _mixing_output{"RBCLW", NUM_OUTPUTS, *this, MixingOutput::SchedulingPolicy::Auto, false};

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::PublicationMulti<wheel_encoders_s> _wheel_encoders_pub[MAX_CONTROLLERS] {
		{ORB_ID(wheel_encoders)}, {ORB_ID(wheel_encoders)}, {ORB_ID(wheel_encoders)}, {ORB_ID(wheel_encoders)},
		{ORB_ID(wheel_encoders)}, {ORB_ID(wheel_encoders)}, {ORB_ID(wheel_encoders)}, {ORB_ID(wheel_encoders)}
	};
	device::Serial _uart;
	bool _initialized{false};
	uint8_t _address{128};
	uint8_t _base_address{128};
	unsigned _controller_count{1};
	unsigned _encoder_controller{0};
	hrt_abstime _last_encoder_read{0};
	uint32_t _communication_errors{0};
	uint32_t _encoder_updates[MAX_CONTROLLERS] {};

	bool init();
	int readEncoder(unsigned controller);
	int setMotorSpeed(unsigned motor, float value);
	void stopMotors();

	// Roboclaw protocol
	int sendTransaction(Command cmd, uint8_t *write_buffer, size_t bytes_to_write);
	int writeCommandWithPayload(Command cmd, uint8_t *wbuff, size_t bytes_to_write);
	int readAcknowledgement();

	int receiveTransaction(Command cmd, uint8_t *read_buffer, size_t bytes_to_read);
	int writeCommand(Command cmd);
	int readResponse(Command command, uint8_t *read_buffer, size_t bytes_to_read);

	static uint16_t _calcCRC(const uint8_t *buf, size_t n, uint16_t init = 0);
	int32_t swapBytesInt32(uint8_t *buffer);

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::RBCLW_ADDRESS>) _param_rbclw_address,
		(ParamInt<px4::params::RBCLW_BAUD>) _param_rbclw_baud,
		(ParamInt<px4::params::RBCLW_COUNT>) _param_rbclw_count,
		(ParamInt<px4::params::RBCLW_COUNTS_REV>) _param_rbclw_counts_rev
	)
};
