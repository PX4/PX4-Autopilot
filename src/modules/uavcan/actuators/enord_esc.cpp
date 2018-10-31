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

#include "enord_esc.hpp"
#include <systemlib/err.h>

#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <lib/ecl/geo/geo.h>
#include <parameters/param.h>

EnordEscController::EnordEscController(uavcan::INode &node) :
	_node(node),
        _enord_pub_rpm_cmd(node),
        _enord_esc_sub_status(node),
        _enord_esc_sub_echo(node),
        _orb_timer(node)
{
    _enord_pub_rpm_cmd.setPriority(UAVCAN_COMMAND_TRANSFER_PRIORITY);
}

EnordEscController::~EnordEscController()
{
}

int EnordEscController::init()
{
	// ESC status subscription
        int res = _enord_esc_sub_status.start(StatusCbBinder(this, &EnordEscController::esc_status_sub_cb));

        // ESC Echo subscription
        res = _enord_esc_sub_echo.start(EchoCbBinder(this, &EnordEscController::esc_echo_sub_cb));

	if (res < 0) {
		warnx("ESC status sub failed %i", res);
		return res;
	}

	// ESC status will be relayed from UAVCAN bus into ORB at this rate
        _orb_timer.setCallback(TimerCbBinder(this, &EnordEscController::orb_pub_timer_cb));
	_orb_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(1000 / ESC_STATUS_UPDATE_RATE_HZ));

        _pub_enord_report = orb_advertise(ORB_ID(enord_report), &_enord_report);

        _enord_report.balance = 0;
        _enord_report.ground_speed = 0.0f;
        _enord_report.pump += 0;

	return res;
}

void EnordEscController::update_outputs(float ground_speed, float altitude)
{
    /*
     * Rate limiting - we don't want to congest the bus
     */
    const auto timestamp = _node.getMonotonicTime();

    if ((timestamp - _prev_cmd_pub).toUSec() < ((1000000) / MAX_RATE_HZ)) {
            return;
    }

    _prev_cmd_pub = timestamp;

    enord::esc::RawCommand msg = GetOperationRawCommand(ground_speed, altitude);

    /*
     * Publish the command message to the bus
     * Note that for a quadrotor it takes one CAN frame
     */
    (void)_enord_pub_rpm_cmd.broadcast(msg);
}

enord::esc::RawCommand
EnordEscController::GetOperationRawCommand(float ground_speed, float altitude)
{
    int32_t operation_min = 0;
    int32_t operation_max = 0;

    enord::esc::RawCommand msg;
    msg.command = 0x01;

    msg.start_duty1 = 0;
    msg.start_duty2 = 0;
    msg.start_stop_delay = 0;
    msg.rising_falling_delay = 0;
    msg.operating_duty2 = 0;    

    (void)param_get(param_find("ENORD_OP_MAX"), &operation_max);
    (void)param_get(param_find("ENORD_OP_MIN"), &operation_min);

    switch(_switch_value)
    {
    case ENORD_ESC_OPERATION_MAX:
        msg.operating_duty1 = operation_max;
        break;
    case ENORD_ESC_OPERATION_IDLE:
        msg.operating_duty1 = 0;
        break;
    case ENORD_ESC_OPERATION_OPERATION:
        if(_roll_value > 0.0f || _roll_value < 0.0f)
        {
            msg.operating_duty1 = 0;
        }else{
            msg.operating_duty1 = calculate_operation_duty(ground_speed, altitude, operation_min, operation_max);
        }
        break;
    }

    msg.operating_duty1 = (msg.operating_duty1 > 250) ? 250 : msg.operating_duty1;

    return msg;
}

int
EnordEscController::calculate_operation_duty(float ground_speed, float altitude, int32_t operation_min, int32_t operation_max)
{
    float velocity_min = 0.0f;    
    float velocity_max = 0.0f;

    (void)param_get(param_find("ENORD_V_MIN"), &velocity_min);
    (void)param_get(param_find("ENORD_V_MAX"), &velocity_max);

    if(ground_speed < velocity_min) return 0;
    if(ground_speed > velocity_max) return operation_max;


    // operation duty slope for speed only use one pump
    float _op1_slope = (operation_max - operation_min) / (velocity_max - velocity_min);
    int _constant_1 = (int)(operation_min - (_op1_slope * velocity_min));

    return (int)(_op1_slope * ground_speed + _constant_1);
}

void EnordEscController::esc_echo_sub_cb(const uavcan::ReceivedDataStructure<enord::esc::RawCommand> &msg)
{
    // echo check
    PX4_WARN("\t node id : %d\n", msg.getSrcNodeID().get());
    PX4_WARN("operating_duty front:\t%d!\n", msg.operating_duty1);
}

void EnordEscController::esc_status_sub_cb(const uavcan::ReceivedDataStructure<enord::esc::RawResponse> &msg)
{
    // echo check
    //printf("\t node id : %d\n", msg.getSrcNodeID().get());
    //PX4_INFO("operating_duty front:\t%d!\n", msg.balance);
    //PX4_INFO("motor 1 operation duty:\t%d!\n", msg.temp_2);
}

void EnordEscController::orb_pub_timer_cb(const uavcan::TimerEvent &)
{
    // echo check value publish
    _enord_report.balance += 1;
    _enord_report.ground_speed += 0.01f;
    _enord_report.pump += 1;
    orb_publish(ORB_ID(enord_report), _pub_enord_report, &_enord_report);
}
