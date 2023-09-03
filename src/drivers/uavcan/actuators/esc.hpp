/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
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
 * @file esc.hpp
 *
 * UAVCAN <--> ORB bridge for ESC messages:
 *     uavcan.equipment.esc.RawCommand
 *     uavcan.equipment.esc.RPMCommand
 *     uavcan.equipment.esc.Status
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/uavcan.hpp>
#include <uavcan/equipment/esc/RawCommand.hpp>
#include <uavcan/equipment/esc/Status.hpp>
#include <lib/perf/perf_counter.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/esc_status.h>
#include <drivers/drv_hrt.h>
#include <lib/mixer_module/mixer_module.hpp>

#include "../third_party_protocols/kdecan.hpp"

class UavcanEscController
{
public:
	static constexpr int MAX_ACTUATORS = esc_status_s::CONNECTED_ESC_MAX;
	static constexpr unsigned MAX_RATE_HZ = 400;
	static constexpr uint16_t DISARMED_OUTPUT_VALUE = UINT16_MAX;

	static_assert(uavcan::equipment::esc::RawCommand::FieldTypes::cmd::MaxSize >= MAX_ACTUATORS, "Too many actuators");


	UavcanEscController(uavcan::INode &node, uavcan::Protocol can_protocol);
	~UavcanEscController() = default;

	int init();

	void update_outputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs);

	/**
	 * Sets the number of rotors and enable timer
	 */
	void set_rotor_count(uint8_t count);

	static int max_output_value() { return uavcan::equipment::esc::RawCommand::FieldTypes::cmd::RawValueType::max(); }

	bool setCANProtocol(uavcan::Protocol can_protocol) {
		if (can_protocol != uavcan::Protocol::Invalid) {
			_can_protocol = can_protocol;

			return true;
		} else {
			return false;
		}
	}

	uavcan::Protocol getCANProtocol() { return _can_protocol; }

	void kdeCanSetExtendedId(bool extended_id) { _extended_id = extended_id; }

	void kdeCanSetMotorPoles(uint8_t motor_poles) { _kdecan_motor_poles = motor_poles; }

	esc_status_s &esc_status() { return _esc_status; }

private:
	/**
	 * ESC status message reception will be reported via this callback.
	 */
	void esc_status_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::Status> &msg);

	/**
	 * KDECAN ESC status message reception will be reported via these callbacks.
	 */
	void kdeesc_status_sub_cb(const kdecan::EscStatus&);
	void kdeesc_input_throttle_sub_cb(const kdecan::InputThrottle&);
	void kdeesc_output_throttle_sub_cb(const kdecan::OutputThrottle&);

	/**
	 * ESC status requests will be triggered from this callback (fixed rate).
	 */
	void kdeesc_status_timer_cb(const uavcan::TimerEvent &event);

	/**
	 * Checks all the ESCs freshness based on timestamp, if an ESC exceeds the timeout then is flagged offline.
	 */
	uint8_t check_escs_status();

	static constexpr unsigned ESC_STATUS_UPDATE_RATE_HZ = 10;

	typedef uavcan::MethodBinder<UavcanEscController *,
		void (UavcanEscController::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::Status>&)> StatusCbBinder;

	typedef uavcan::MethodBinder<UavcanEscController *,
		void (UavcanEscController::*)(const kdecan::EscStatus&)>  KdeStatusCbBinder;

	typedef uavcan::MethodBinder<UavcanEscController *,
		void (UavcanEscController::*)(const kdecan::InputThrottle&)>  KdeInputThrottleCbBinder;

	typedef uavcan::MethodBinder<UavcanEscController *,
		void (UavcanEscController::*)(const kdecan::OutputThrottle&)>  KdeOutputThrottleCbBinder;

	typedef uavcan::MethodBinder<UavcanEscController *,
		void (UavcanEscController::*)(const uavcan::TimerEvent &)> TimerCbBinder;

	esc_status_s	_esc_status{};

	uORB::PublicationMulti<esc_status_s> _esc_status_pub{ORB_ID(esc_status)};

	uint8_t		_rotor_count{0};

	uint8_t 	_kdecan_motor_poles{1};

	/*
	 * libuavcan related things
	 */
	uavcan::MonotonicTime							_prev_cmd_pub;   ///< rate limiting
	uavcan::INode								&_node;
	uavcan::Publisher<uavcan::equipment::esc::RawCommand>			_uavcan_pub_raw_cmd;
	uavcan::Subscriber<uavcan::equipment::esc::Status, StatusCbBinder>	_uavcan_sub_status;
	uavcan::TimerEventForwarder<TimerCbBinder>				_orb_timer;

	kdecan::Publisher<kdecan::EscStatus>					_kde_status_pub;
	kdecan::Subscriber<kdecan::EscStatus, KdeStatusCbBinder>		_kde_status_sub;
	kdecan::Subscriber<kdecan::InputThrottle, KdeInputThrottleCbBinder>	_kde_ith_sub;
	kdecan::Publisher<kdecan::InputThrottle>				_kde_ith_pub;
	kdecan::Subscriber<kdecan::OutputThrottle, KdeOutputThrottleCbBinder>	_kde_oth_sub;
	kdecan::Publisher<kdecan::OutputThrottle>				_kde_oth_pub;
	kdecan::Publisher<kdecan::PwmThrottle>					_kde_pwm_pub;

	uavcan::Protocol							_can_protocol;
	bool 									_extended_id;

	/*
	 * ESC states
	 */
	uint8_t				_max_number_of_nonzero_outputs{0};
};
