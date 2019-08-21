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
 * @file UavcanServoController.cpp
 *
 * @author Richard Kirby rkirby@kspresearch.com>
 */

#include "servo.hpp"
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#define MOTOR_BIT(x) (1<<(x))

UavcanServoController::UavcanServoController(uavcan::INode &node) :
	_node(node),
        _uavcan_pub_array_cmd(node),
	_uavcan_sub_status(node),
	_orb_timer(node)
{
        //_uavcan_pub_raw_cmd.setPriority(UAVCAN_COMMAND_TRANSFER_PRIORITY);
        _uavcan_pub_array_cmd.setPriority(UAVCAN_COMMAND_TRANSFER_PRIORITY);

	if (_perfcnt_invalid_input == nullptr) {
		errx(1, "uavcan: couldn't allocate _perfcnt_invalid_input");
	}

	if (_perfcnt_scaling_error == nullptr) {
		errx(1, "uavcan: couldn't allocate _perfcnt_scaling_error");
	}
}

UavcanServoController::~UavcanServoController()
{
	perf_free(_perfcnt_invalid_input);
	perf_free(_perfcnt_scaling_error);
}

int UavcanServoController::init()
{
//	// ESC status subscription
//	int res = _uavcan_sub_status.start(StatusCbBinder(this, &UavcanEscController::esc_status_sub_cb));

//	if (res < 0) {
//		warnx("ESC status sub failed %i", res);
//		return res;
//	}

//	// ESC status will be relayed from UAVCAN bus into ORB at this rate
//	_orb_timer.setCallback(TimerCbBinder(this, &UavcanEscController::orb_pub_timer_cb));
//	_orb_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(1000 / ESC_STATUS_UPDATE_RATE_HZ));

//	return res;
        return 0;
}

void UavcanServoController::update_outputs(uint8_t actuator_id, float value)
{
//	if ((outputs == nullptr) ||
//	    (num_outputs > uavcan::equipment::esc::RawCommand::FieldTypes::cmd::MaxSize) ||
//	    (num_outputs > esc_status_s::CONNECTED_ESC_MAX)) {
//		perf_count(_perfcnt_invalid_input);
//		return;
//	}

        /*
         * Rate limiting - we don't want to congest the bus
         */

        if (_armed_mask & SERVO_BIT(actuator_id)) {
                const auto timestamp = _node.getMonotonicTime();

                //TODO: Determine if we need to rate limit
        //        if ((timestamp - _prev_cmd_pub).toUSec() < (1000000 / MAX_RATE_HZ)) {
        //                return;
        //        }

                _prev_cmd_pub = timestamp;

                /*
                * Fill the command message
                */
                uavcan::equipment::actuator::ArrayCommand msg;
                uavcan::equipment::actuator::Command cmd;

                cmd.actuator_id = actuator_id;
                cmd.command_type = uavcan::equipment::actuator::Command::COMMAND_TYPE_UNITLESS;
                cmd.command_value = value;
                msg.commands.push_back(cmd);

                /*
                * Publish the command message to the bus
                */
                (void)_uavcan_pub_array_cmd.broadcast(msg);
        }

        // TODO: figure out if I need to Publish actuator outputs
//	if (_actuator_outputs_pub != nullptr) {
//		orb_publish(ORB_ID(actuator_outputs), _actuator_outputs_pub, &actuator_outputs);

//	} else {
//		int instance;
//		_actuator_outputs_pub = orb_advertise_multi(ORB_ID(actuator_outputs), &actuator_outputs,
//					&instance, ORB_PRIO_DEFAULT);
//	}

}

void UavcanServoController::servo_status_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::Status> &msg)
{
//	if (msg.esc_index < esc_status_s::CONNECTED_ESC_MAX) {
//		_esc_status.esc_count = uavcan::max<int>(_esc_status.esc_count, msg.esc_index + 1);
//		_esc_status.timestamp = hrt_absolute_time();

//		auto &ref = _esc_status.esc[msg.esc_index];

//		ref.esc_address = msg.getSrcNodeID().get();

//		ref.esc_voltage     = msg.voltage;
//		ref.esc_current     = msg.current;
//		ref.esc_temperature = msg.temperature;
//		ref.esc_setpoint    = msg.power_rating_pct;
//		ref.esc_rpm         = msg.rpm;
//		ref.esc_errorcount  = msg.error_count;
//	}
}

void UavcanServoController::orb_pub_timer_cb(const uavcan::TimerEvent &)
{
//	_esc_status.counter += 1;
//	_esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_CAN;

//	if (_esc_status_pub != nullptr) {
//		(void)orb_publish(ORB_ID(esc_status), _esc_status_pub, &_esc_status);

//	} else {
//		_esc_status_pub = orb_advertise(ORB_ID(esc_status), &_esc_status);
//	}
}


void UavcanServoController::arm_all_servos(bool arm)
{
	if (arm) {
		_armed_mask = -1;

	} else {
		_armed_mask = 0;
	}
}

void UavcanServoController::arm_single_servo(int num, bool arm)
{
	if (arm) {
		_armed_mask = MOTOR_BIT(num);

	} else {
		_armed_mask = 0;
	}
}
