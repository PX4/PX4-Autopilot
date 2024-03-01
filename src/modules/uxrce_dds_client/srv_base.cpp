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

#include "srv_base.h"

#include<cstdio>
//#include "utilities.hpp"


#define TOPIC_NAME_SIZE 128
#define REQUEST_TYPE_SIZE 128
#define REPLY_TYPE_SIZE 128

static bool generate_request_name(char *request, const char *client_namespace, const char *name)
{
	if (client_namespace != nullptr) {
		int ret = snprintf(request, TOPIC_NAME_SIZE, "rq/%s/fmu/%sRequest", client_namespace, name);
		return (ret > 0 && ret < TOPIC_NAME_SIZE);
	}

	int ret = snprintf(request, TOPIC_NAME_SIZE, "rq/fmu/%sRequest", name);
	return (ret > 0 && ret < TOPIC_NAME_SIZE);
}

static bool generate_reply_name(char *reply, const char *client_namespace, const char *name)
{
	if (client_namespace != nullptr) {
		int ret = snprintf(reply, TOPIC_NAME_SIZE, "rr/%s/fmu/%sReply", client_namespace, name);
		return (ret > 0 && ret < TOPIC_NAME_SIZE);
	}

	int ret = snprintf(reply, TOPIC_NAME_SIZE, "rr/fmu/%sReply", name);
	return (ret > 0 && ret < TOPIC_NAME_SIZE);
}

static bool generate_request_type_name(char *request, const char *name)
{
	int ret = snprintf(request, REQUEST_TYPE_SIZE, "px4_msgs::srv::dds_::%s_Request_", name);
	return (ret > 0 && ret < REQUEST_TYPE_SIZE);
}

static bool generate_reply_type_name(char *reply, const char *name)
{
	int ret = snprintf(reply, REPLY_TYPE_SIZE, "px4_msgs::srv::dds_::%s_Response_", name);
	return (ret > 0 && ret < REPLY_TYPE_SIZE);
}

SrvBase::SrvBase(uxrSession *session, uxrStreamId reliable_out_stream_id, uxrStreamId input_stream_id,
		 uxrObjectId participant_id) :
	session_(session),
	reliable_out_stream_id_(reliable_out_stream_id)

{

}

bool SrvBase::create_replier(uxrStreamId input_stream_id,
			     uxrObjectId participant_id, uint16_t index, const char *client_namespace, const char *service_name_simple,
			     const char *service_type_name_simple, uint16_t queue_depth)
{
// request and reply names
	char request_name[TOPIC_NAME_SIZE];
	char reply_name[TOPIC_NAME_SIZE];

	if (!generate_request_name(request_name, client_namespace, service_name_simple)) {
		return false;
	}

	if (!generate_reply_name(reply_name, client_namespace, service_name_simple)) {
		return false;
	}

	// request and reply types
	char request_type_name[REQUEST_TYPE_SIZE];
	char reply_type_name[REPLY_TYPE_SIZE];

	if (!generate_request_type_name(request_type_name, service_type_name_simple)) {
		return false;
	}

	if (!generate_reply_type_name(reply_type_name, service_type_name_simple)) {
		return false;
	}


	// Use the second half of the available ID space.
	// Add 1 so that we get a nice hex starting number: 0x800 instead of 0x7ff.
	uint16_t id = index + (65535U / 32U) + 1;

	replier_id_ = uxr_object_id(id, UXR_REPLIER_ID);

	//char service_name[TOPIC_NAME_SIZE];

	const uxrQoS_t qos = {
		.durability = UXR_DURABILITY_PERSISTENT,
		.reliability = UXR_RELIABILITY_RELIABLE,
		.history = UXR_HISTORY_KEEP_LAST,
		.depth = 1,
	};

	uint16_t replier_req = uxr_buffer_create_replier_bin(session_, reliable_out_stream_id_, replier_id_, participant_id,
			       service_name_simple, request_type_name, reply_type_name, request_name, reply_name, qos, UXR_REPLACE);
	uint8_t status;

	if (!uxr_run_session_until_all_status(session_, 1000, &replier_req, &status, 1)) {
		return false;
	}


	// Request  requests
	uxrDeliveryControl delivery_control = {
		0
	};
	delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
	uint16_t read_data_req =
		uxr_buffer_request_data(session_, reliable_out_stream_id_, replier_id_, input_stream_id, &delivery_control);
	(void) read_data_req;

	return true;
}
