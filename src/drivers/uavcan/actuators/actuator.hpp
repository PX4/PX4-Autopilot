/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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
 * @file actuator.hpp
 *
 * UAVCAN <--> ORB bridge for actuator (servo) messages:
 *     uavcan.equipment.actuator.ArrayCommand
 *     uavcan.equipment.actuator.Status
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 * @author Jacob Crabill <jacob@flyvoly.com>
 */

#pragma once

#include <uavcan/uavcan.hpp>
#include <uavcan/equipment/actuator/ArrayCommand.hpp>
#include <uavcan/equipment/actuator/Status.hpp>

#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/servo_status.h>

#include <drivers/drv_hrt.h>

#include <lib/mixer_module/mixer_module.hpp>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>


class UavcanActuatorController
{
public:
	static constexpr int MAX_ACTUATORS = MixingOutput::MAX_ACTUATORS;
	static constexpr uint16_t DISARMED_OUTPUT_VALUE = UINT16_MAX;

	UavcanActuatorController(uavcan::INode &node);
	~UavcanActuatorController();

	int init();

	void update_outputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs);

	/**
	 * Sets the number of rotors
	 */
	void set_actuator_count(uint8_t count) { _actuator_count = count; }

	static int max_output_value() { return uavcan::equipment::actuator::ArrayCommand::FieldTypes::cmd::RawValueType::max(); }

private:
	/**
	 * ESC status message reception will be reported via this callback.
	 */
	void actuator_status_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::actuator::Status> &msg);

	/**
	 * ESC status will be published to ORB from this callback (fixed rate).
	 */
	void orb_pub_timer_cb(const uavcan::TimerEvent &event);

	/**
	 * Checks all the actuators's freshness based on timestamp, if an actuator exceeds the timeout then is flagged offline.
	 */
	uint8_t check_actuators_status();

	static constexpr unsigned SERVO_STATUS_UPDATE_RATE_HZ = 10;
	static constexpr unsigned UAVCAN_COMMAND_TRANSFER_PRIORITY = 5;	///< 0..31, inclusive, 0 - highest, 31 - lowest

	typedef uavcan::MethodBinder<UavcanActuatorController *,
		void (UavcanActuatorController::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::actuator::Status>&)>
		StatusCbBinder;

	typedef uavcan::MethodBinder<UavcanActuatorController *,
		void (UavcanActuatorController::*)(const uavcan::TimerEvent &)> TimerCbBinder;

	servo_status_s	_servo_status{};

	uORB::PublicationMulti<servo_status_s> _servo_status_pub{ORB_ID(servo_status)};

	uint8_t		_actuator_count{0};

	/*
	 * libuavcan related things
	 */
	uavcan::MonotonicTime							_prev_cmd_pub;   ///< rate limiting
	uavcan::INode								&_node;
	uavcan::Publisher<uavcan::equipment::actuator::ArrayCommand>		_uavcan_pub_raw_cmd;
	uavcan::Subscriber<uavcan::equipment::actuator::Status, StatusCbBinder>	_uavcan_sub_status;
	uavcan::TimerEventForwarder<TimerCbBinder>				_orb_timer;

	/*
	 * Servo states
	 */
	uint8_t				_max_number_of_nonzero_outputs{0};

	int 				_esc_update_rate{0};
};
