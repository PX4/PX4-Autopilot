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
 * @file esc.cpp
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include "esc.hpp"
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#define MOTOR_BIT(x) (1<<(x))

using namespace time_literals;

UavcanEscController::UavcanEscController(uavcan::INode &node, uavcan::Protocol can_protocol) :
	_node(node),
	_uavcan_pub_raw_cmd(node),
	_uavcan_sub_status(node),
	_orb_timer(node),
	_kde_status_pub(node),
	_kde_status_sub(node),
	_kde_ith_sub(node),
	_kde_ith_pub(node),
	_kde_oth_sub(node),
	_kde_oth_pub(node),
	_kde_pwm_pub(node),
	_can_protocol(can_protocol),
	_extended_id(true)
{
	_uavcan_pub_raw_cmd.setPriority(uavcan::TransferPriority::NumericallyMin); // Highest priority
}

int
UavcanEscController::init()
{
	// ESC status subscription (only enabled if in UAVCAN mode)
	if (_can_protocol == uavcan::Protocol::Standard) {
		int res = _uavcan_sub_status.start(StatusCbBinder(this, &UavcanEscController::esc_status_sub_cb));

		if (res < 0) {
			PX4_ERR("ESC status sub failed %i", res);
			return res;
		}

		_esc_status_pub.advertise();

	} else if (_can_protocol == kdecan::protocolID) {
		_kde_status_sub.setCallback(KdeStatusCbBinder(this, &UavcanEscController::kdeesc_status_sub_cb));
		_kde_ith_sub.setCallback(KdeInputThrottleCbBinder(this, &UavcanEscController::kdeesc_input_throttle_sub_cb));
		_kde_oth_sub.setCallback(KdeOutputThrottleCbBinder(this, &UavcanEscController::kdeesc_output_throttle_sub_cb));
		_node.getDispatcher().registerCustomCanListener(_kde_status_sub.getKdeListener());
		_node.getDispatcher().registerCustomCanListener(_kde_ith_sub.getKdeListener());
		_node.getDispatcher().registerCustomCanListener(_kde_oth_sub.getKdeListener());

		// Callback needed to trigger status queries
		_orb_timer.setCallback(TimerCbBinder(this, &UavcanEscController::kdeesc_status_timer_cb));
		_orb_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(1000 / ESC_STATUS_UPDATE_RATE_HZ));
	}

	return OK;
}

void
UavcanEscController::update_outputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs)
{
	/*
	 * Rate limiting - we don't want to congest the bus
	 */
	const auto timestamp = _node.getMonotonicTime();

	if ((timestamp - _prev_cmd_pub).toUSec() < (1000000 / MAX_RATE_HZ)) {
		return;
	}

	_prev_cmd_pub = timestamp;

	if (_can_protocol == uavcan::Protocol::Standard) {\

		/*
		 * Fill the command message
		 * If unarmed, we publish an empty message anyway
		 */
		uavcan::equipment::esc::RawCommand msg;

		for (unsigned i = 0; i < num_outputs; i++) {
			if (stop_motors || outputs[i] == DISARMED_OUTPUT_VALUE) {
				msg.cmd.push_back(static_cast<unsigned>(0));

			} else {
				msg.cmd.push_back(static_cast<int>(outputs[i]));
			}
		}

		/*
		 * Remove channels that are always zero.
		 * transfer would be enough. This is a valid optimization as the UAVCAN specification implies that all
		 * non-specified ESC setpoints should be considered zero.
		 * The positive outcome is a (marginally) lower bus traffic and lower CPU load.
		 *
		 * From the standpoint of the PX4 architecture, however, this is a hack. It should be investigated why
		 * the mixer returns more outputs than are actually used.
		 */
		for (int index = int(msg.cmd.size()) - 1; index >= _max_number_of_nonzero_outputs; index--) {
			if (msg.cmd[index] != 0) {
				_max_number_of_nonzero_outputs = index + 1;
				break;
			}
		}

		msg.cmd.resize(_max_number_of_nonzero_outputs);

		/*
		 * Publish the command message to the bus
		 * Note that for a quadrotor it takes one CAN frame
		 */
		_uavcan_pub_raw_cmd.broadcast(msg);

	} else if (_can_protocol == kdecan::protocolID) {
		// Fill and publish the command message - one command per each ESC
		for (unsigned i = 0; i < num_outputs; i++) {
			uint16_t kdecan_output = 0;

			if (stop_motors || outputs[i] == DISARMED_OUTPUT_VALUE) {
				kdecan_output = kdecan::minPwmValue;

			} else {
				kdecan_output = kdecan::minPwmValue + (uint16_t)(((float)outputs[i]/max_output_value()) * (kdecan::maxPwmValue - kdecan::minPwmValue));
			}

			_kde_pwm_pub.publish(kdecan::PwmThrottle(kdecan::escNodeIdOffset + i, kdecan_output), _extended_id);
		}
	}
}

void
UavcanEscController::set_rotor_count(uint8_t count)
{
	_rotor_count = count;
}

void
UavcanEscController::esc_status_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::Status> &msg)
{
	if (msg.esc_index < esc_status_s::CONNECTED_ESC_MAX) {
		auto &ref = _esc_status.esc[msg.esc_index];

		ref.timestamp       = hrt_absolute_time();
		ref.esc_address = msg.getSrcNodeID().get();
		ref.esc_voltage     = msg.voltage;
		ref.esc_current     = msg.current;
		ref.esc_temperature = msg.temperature;
		ref.esc_rpm         = msg.rpm;
		ref.esc_errorcount  = msg.error_count;

		_esc_status.esc_count = _rotor_count;
		_esc_status.counter += 1;
		_esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_CAN;
		_esc_status.esc_online_flags = check_escs_status();
		_esc_status.esc_armed_flags = (1 << _rotor_count) - 1;
		_esc_status.timestamp = hrt_absolute_time();
		_esc_status_pub.publish(_esc_status);
	}
}

void
UavcanEscController::kdeesc_status_sub_cb(const kdecan::EscStatus& received_structure)
{
	if (received_structure.source_address_ - kdecan::escNodeIdOffset < esc_status_s::CONNECTED_ESC_MAX) {
		auto &ref = _esc_status.esc[received_structure.source_address_  - kdecan::escNodeIdOffset];

		/*PX4_INFO("status received: id=%d V=%.4f C=%.4F RPM=%.4f T=%d Warn=%d",
			received_structure.source_address_,
			(double)received_structure.voltage_,
			(double)received_structure.current_,
			(double)received_structure.erpm_,
			received_structure.temperature_,
			received_structure.warnings_);*/

		ref.esc_address     = received_structure.source_address_;
		ref.timestamp       = hrt_absolute_time();
		ref.esc_voltage     = received_structure.voltage_;
		ref.esc_current     = received_structure.current_;
		ref.esc_temperature = received_structure.temperature_;
		ref.esc_rpm         = received_structure.erpm_ / _kdecan_motor_poles;
		ref.esc_errorcount  = received_structure.warnings_;
	}
}

void
UavcanEscController::kdeesc_input_throttle_sub_cb(const kdecan::InputThrottle& received_structure)
{
	if (received_structure.source_address_ - kdecan::escNodeIdOffset < esc_status_s::CONNECTED_ESC_MAX) {
		auto &ref = _esc_status.esc[received_structure.source_address_  - kdecan::escNodeIdOffset];

		/*PX4_INFO("in throttle: id = %d Th=%d",
			received_structure.source_address_,
			received_structure.input_throttle_);*/

		ref.input_throttle = received_structure.input_throttle_;
	}
}

void
UavcanEscController::kdeesc_output_throttle_sub_cb(const kdecan::OutputThrottle& received_structure)
{
	if (received_structure.source_address_ - kdecan::escNodeIdOffset < esc_status_s::CONNECTED_ESC_MAX) {
		auto &ref = _esc_status.esc[received_structure.source_address_  - kdecan::escNodeIdOffset];

		/*PX4_INFO("out throttle: id = %d Th=%.4f",
			received_structure.source_address_,
			(double)received_structure.output_throttle_);*/

		ref.output_throttle = received_structure.output_throttle_;
	}
}

void
UavcanEscController::kdeesc_status_timer_cb(const uavcan::TimerEvent &)
{
	// we need to actively send a request for the KDE protocol
	if (_can_protocol == kdecan::protocolID) {
		_kde_status_pub.publish(kdecan::EscStatus(kdecan::escNodeIdBroadcast), _extended_id);
		_kde_ith_pub.publish(kdecan::InputThrottle(kdecan::escNodeIdBroadcast), _extended_id);
		_kde_oth_pub.publish(kdecan::OutputThrottle(kdecan::escNodeIdBroadcast), _extended_id);
	}
}


uint8_t
UavcanEscController::check_escs_status()
{
	int esc_status_flags = 0;
	const hrt_abstime now = hrt_absolute_time();

	for (int index = 0; index < esc_status_s::CONNECTED_ESC_MAX; index++) {

		if (_esc_status.esc[index].timestamp > 0 && now - _esc_status.esc[index].timestamp < 1200_ms) {
			esc_status_flags |= (1 << index);
		}

	}

	return esc_status_flags;
}
