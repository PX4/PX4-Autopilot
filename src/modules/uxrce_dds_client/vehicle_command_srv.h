/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#pragma once

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>

#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>

#include "srv_base.h"

/**
 * @see SrvBase
 * @class VehicleCommandSrv The VehicleCommandSrv class implement the VehicleCommand service server.
 */
class VehicleCommandSrv : public SrvBase
{
private:
	uORB::Publication<vehicle_command_s> vehicle_command_pub_{ORB_ID(vehicle_command)};
	uORB::Subscription vehicle_command_ack_sub_{ORB_ID(vehicle_command_ack)};
	uint32_t last_command_sent_{0};
	hrt_abstime last_command_sent_timestamp_{0};
public:
	/**
	 * @brief Constructor.
	 * @see SrvBase.
	 * @param client_namespace namespace for the client service.
	 * @param index index used to create the replier id.
	 * @return Returns false iff successful, otherwise false.
	 */
	VehicleCommandSrv(uxrSession *session, uxrStreamId reliable_out_stream_id, uxrStreamId input_stream_id,
			  uxrObjectId participant_id, const char *client_namespace, const uint8_t index);

	~VehicleCommandSrv();

	/** @see SrvBase */
	bool process_request(ucdrBuffer *ub, const int64_t time_offset_us);

	/** @see SrvBase */
	bool process_reply();
};
