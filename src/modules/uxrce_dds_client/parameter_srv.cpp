/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#include "parameter_srv.h"
#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/log.h>
#include <uORB/ucdr/parameter_get_request.h>
#include <uORB/ucdr/parameter_get_reply.h>
#include <uORB/ucdr/parameter_set_request.h>
#include <uORB/ucdr/parameter_set_reply.h>

static param_t resolve_param(int16_t param_index, const char *param_id)
{
	param_t param = PARAM_INVALID;

	if (param_index >= 0) {
		// Get parameter by index
		param = param_for_used_index((unsigned)param_index);
	} else {
		// Get parameter by name
		param = param_find_no_notification(param_id);
	}

	return param;
}

template <typename ReplyType>
static void populate_reply(ReplyType &reply, param_t param)
{
	reply.param_index = param_get_used_index(param);

	// Copy name
	const char *name = param_name(param);
	strncpy(reply.param_id, name, sizeof(reply.param_id));

	// Get type and value
	param_type_t type = param_type(param);

	if (type == PARAM_TYPE_INT32) {
		reply.param_type = ReplyType::TYPE_INT32;
		int32_t val;

		if (param_get(param, &val) == 0) {
			reply.int_value = val;
			reply.success = true;
		}

	} else if (type == PARAM_TYPE_FLOAT) {
		reply.param_type = ReplyType::TYPE_FLOAT;
		float val;

		if (param_get(param, &val) == 0) {
			reply.real_value = val;
			reply.success = true;
		}
	}
}

// ParameterGetSrv Implementation

ParameterGetSrv::ParameterGetSrv(uxrSession *session, uxrStreamId reliable_out_stream_id,
				 uxrStreamId input_stream_id, uxrObjectId participant_id, const char *client_namespace, const uint8_t index) :
	SrvBase(session, reliable_out_stream_id, input_stream_id, participant_id)
{
	uint16_t queue_depth = orb_get_queue_size(ORB_ID(parameter_get_request));
	create_replier(input_stream_id, participant_id, index, client_namespace, "param_get", "ParameterGet",
		       queue_depth);
};

bool ParameterGetSrv::process_request(ucdrBuffer *ub, const int64_t time_offset_us)
{
	parameter_get_request_s req;

	if (ucdr_deserialize_parameter_get_request(*ub, req, time_offset_us)) {

		parameter_get_reply_s reply{};
		reply.timestamp = hrt_absolute_time();
		reply.success = false;
		reply.param_count = param_count_used();

		param_t param = resolve_param(req.param_index, req.param_id);

		if (param != PARAM_INVALID) {
			populate_reply(reply, param);

		} else {
			// Parameter not found
			reply.success = false;

			// Even if not found, we might want to populate param_id if it was a by-name lookup
			// But if invalid name, we can't do much.
			if (req.param_index < 0) {
				strncpy(reply.param_id, req.param_id, sizeof(reply.param_id));
			}
		}

		// Send reply
		ucdrBuffer reply_ub;
		const uint32_t topic_size = ucdr_topic_size_parameter_get_reply();
		uint8_t reply_buffer[topic_size];
		// Zero out buffer for safety
		memset(reply_buffer, 0, sizeof(reply_buffer));

		const int64_t reply_time_offset_us = session_->time_offset / 1000; // ns -> us
		ucdr_init_buffer(&reply_ub, reply_buffer, sizeof(reply_buffer));

		if (ucdr_serialize_parameter_get_reply(&reply, reply_ub, reply_time_offset_us)) {
			uxr_buffer_reply(session_, reliable_out_stream_id_, replier_id_, &sample_id_, reply_buffer, sizeof(reply_buffer));
		} else {
			PX4_ERR("Param Get Reply serialization failed");
		}
	} else {
		PX4_ERR("Param Get Request deserialization failed");
	}

	return true;
}

bool ParameterGetSrv::process_reply()
{
	return false;
}

// ParameterSetSrv Implementation

ParameterSetSrv::ParameterSetSrv(uxrSession *session, uxrStreamId reliable_out_stream_id,
				 uxrStreamId input_stream_id, uxrObjectId participant_id, const char *client_namespace, const uint8_t index) :
	SrvBase(session, reliable_out_stream_id, input_stream_id, participant_id)
{
	uint16_t queue_depth = orb_get_queue_size(ORB_ID(parameter_set_request));
	create_replier(input_stream_id, participant_id, index, client_namespace, "param_set", "ParameterSet",
		       queue_depth);
};

bool ParameterSetSrv::process_request(ucdrBuffer *ub, const int64_t time_offset_us)
{
	parameter_set_request_s req;

	if (ucdr_deserialize_parameter_set_request(*ub, req, time_offset_us)) {
		parameter_set_reply_s reply{};
		reply.timestamp = hrt_absolute_time();
		reply.success = false;

		param_t param = resolve_param(req.param_index, req.param_id);

		if (param != PARAM_INVALID) {
			param_type_t type = param_type(param);

			bool set_success = false;
			bool read_success = false;

			if (type == PARAM_TYPE_INT32) {
				if (req.param_type == parameter_set_request_s::TYPE_INT32) {
					int32_t val = req.int_value;

					if (param_set(param, &val) == 0) {
						set_success = true;
					}
				}

			} else if (type == PARAM_TYPE_FLOAT) {
				if (req.param_type == parameter_set_request_s::TYPE_FLOAT) {
					float val = req.real_value;

					if (param_set(param, &val) == 0) {
						set_success = true;
					}
				}
			}

			// Populate reply with current value (updated or not)
			populate_reply(reply, param);
			// populate_reply sets success=true only if it can READ the value.
			read_success = reply.success;

			// For a set request we only want success=true if the SET worked
			// and we successfully read back the current value.
			reply.success = set_success && read_success;

		} else {
			// Not found, fill ID from request for context
			if (req.param_index < 0) {
				strncpy(reply.param_id, req.param_id, sizeof(reply.param_id));
			}
		}

		// Send reply
		ucdrBuffer reply_ub;
		const uint32_t topic_size = ucdr_topic_size_parameter_set_reply();
		uint8_t reply_buffer[topic_size];
		// Zero out buffer
		memset(reply_buffer, 0, sizeof(reply_buffer));

		const int64_t reply_time_offset_us = session_->time_offset / 1000; // ns -> us
		ucdr_init_buffer(&reply_ub, reply_buffer, sizeof(reply_buffer));

		if (ucdr_serialize_parameter_set_reply(&reply, reply_ub, reply_time_offset_us)) {
			uxr_buffer_reply(session_, reliable_out_stream_id_, replier_id_, &sample_id_, reply_buffer, sizeof(reply_buffer));
		} else {
			PX4_ERR("Param Set Reply serialization failed");
		}
	} else {
		PX4_ERR("Param Set Request deserialization failed");
	}

	return true;
}

bool ParameterSetSrv::process_reply()
{
	return false;
}
