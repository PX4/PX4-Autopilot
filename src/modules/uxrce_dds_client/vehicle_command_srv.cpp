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

#include "vehicle_command_srv.h"
#include <uORB/ucdr/vehicle_command.h>
#include <uORB/ucdr/vehicle_command_ack.h>

VehicleCommandSrv::VehicleCommandSrv(uxrSession *session, uxrStreamId reliable_out_stream_id,
				     uxrStreamId input_stream_id, uxrObjectId participant_id, const char *client_namespace, const uint8_t index) :
	SrvBase(session, reliable_out_stream_id, input_stream_id, participant_id)
{
	uint16_t queue_depth = orb_get_queue_size(ORB_ID(vehicle_command)) * 2; // use a bit larger queue size than internal
	create_replier(input_stream_id, participant_id, index, client_namespace, "vehicle_command", "VehicleCommand",
		       queue_depth);
};

VehicleCommandSrv::~VehicleCommandSrv()
{

};

bool VehicleCommandSrv::process_request(ucdrBuffer *ub, const int64_t time_offset_us)
{
	vehicle_command_s data;

	if (ucdr_deserialize_vehicle_command(*ub, data, time_offset_us)) {
		vehicle_command_pub_.publish(data);
		is_reply_pending_ = true;
		last_command_sent_ = data.command;
		last_command_sent_timestamp_ = hrt_absolute_time();
	}

	return 0;
}

bool VehicleCommandSrv::process_reply()
{
	vehicle_command_ack_s cmd_ack;

	if (is_reply_pending_ && vehicle_command_ack_sub_.update(&cmd_ack)) {
		if (cmd_ack.command == last_command_sent_ && cmd_ack.timestamp > last_command_sent_timestamp_) {
			last_command_sent_ = 0;
			is_reply_pending_ = false;

			ucdrBuffer reply_ub;
			const uint32_t topic_size = ucdr_topic_size_vehicle_command_ack();
			uint8_t reply_buffer[topic_size] = {
				0

			};
			const int64_t time_offset_us = session_->time_offset / 1000; // ns -> us
			ucdr_init_buffer(&reply_ub, reply_buffer, sizeof(reply_buffer));
			ucdr_serialize_vehicle_command_ack(&cmd_ack, reply_ub, time_offset_us);

			uxr_buffer_reply(session_, reliable_out_stream_id_, replier_id_, &sample_id_, reply_buffer, sizeof(reply_buffer));
		}
	}

	return 0;
}
