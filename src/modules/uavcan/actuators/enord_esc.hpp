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
#include <enord/esc/RawCommand.hpp>
#include <enord/esc/RawResponse.hpp>
#include <uavcan/protocol/NodeStatus.hpp>
#include <perf/perf_counter.h>
#include <px4_config.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/enord_report.h>


class EnordEscController
{
public:
        EnordEscController(uavcan::INode &node);
        ~EnordEscController();

	int init();

        void update_outputs(float ground_speed, float altitude);

        int32_t _switch_value = 0;
        float _roll_value = 0.0f;
private:
	/**
	 * ESC status message reception will be reported via this callback.
	 */
        void esc_status_sub_cb(const uavcan::ReceivedDataStructure<enord::esc::RawResponse> &msg);
        void esc_echo_sub_cb(const uavcan::ReceivedDataStructure<enord::esc::RawCommand> &msg);

	/**
	 * ESC status will be published to ORB from this callback (fixed rate).
	 */
	void orb_pub_timer_cb(const uavcan::TimerEvent &event);

        /**
         * RawCommand Data for operation
         */
        enord::esc::RawCommand GetOperationRawCommand(float ground_speed, float altitude);

        static constexpr unsigned MAX_RATE_HZ = 5;			///< XXX make this configurable
        static constexpr unsigned ESC_STATUS_UPDATE_RATE_HZ = 5;
	static constexpr unsigned UAVCAN_COMMAND_TRANSFER_PRIORITY = 5;	///< 0..31, inclusive, 0 - highest, 31 - lowest

        static constexpr unsigned ENORD_ESC_OPERATION_MAX = -1;
        static constexpr unsigned ENORD_ESC_OPERATION_IDLE = 0;
        static constexpr unsigned ENORD_ESC_OPERATION_OPERATION = 1;

        typedef uavcan::MethodBinder<EnordEscController *,
                void (EnordEscController::*)(const uavcan::ReceivedDataStructure<enord::esc::RawCommand>&)>
                EchoCbBinder;

        typedef uavcan::MethodBinder<EnordEscController *,
                void (EnordEscController::*)(const uavcan::ReceivedDataStructure<enord::esc::RawResponse>&)>
		StatusCbBinder;

        typedef uavcan::MethodBinder<EnordEscController *, void (EnordEscController::*)(const uavcan::TimerEvent &)>
	TimerCbBinder;
	/*
	 * libuavcan related things
	 */
	uavcan::MonotonicTime							_prev_cmd_pub;   ///< rate limiting
	uavcan::INode								&_node;
        uavcan::Publisher<enord::esc::RawCommand>			        _enord_pub_rpm_cmd;

        uavcan::Subscriber<enord::esc::RawResponse, StatusCbBinder>	        _enord_esc_sub_status;
        uavcan::Subscriber<enord::esc::RawCommand, EchoCbBinder>	        _enord_esc_sub_echo;

	uavcan::TimerEventForwarder<TimerCbBinder>				_orb_timer;
        orb_advert_t                                                            _pub_enord_report;
        enord_report_s                                                          _enord_report;

        int calculate_operation_duty(float ground_speed, float altitude, int32_t operation_min, int32_t operation_max);
};
