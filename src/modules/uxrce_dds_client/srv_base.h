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

#include <uxr/client/client.h>

/** @class SrvBase The SrvBase class defines the common properties and methods of a service requester. */
class SrvBase
{
public:
	/**
	 * @brief Constructor.
	 * @param session pointer to the micro xrce-dds session.
	 * @param reliable_out_stream_id output stream ID.
	 * @param input_stream_id input stream ID.
	 * @param participant_id participant ID.
	 * @return Returns false iff successful, otherwise false.
	 */
	SrvBase(uxrSession *session, uxrStreamId reliable_out_stream_id, uxrStreamId input_stream_id,
		uxrObjectId participant_id);

	virtual ~SrvBase()
	{

	};

	/**
	 * @brief Virtual method that process an incoming request from xrce_dds.
	 * @param ub Buffer that stores the incoming request message.
	 * @param time_offset_us time offset between agent and client.
	 * @return Returns false iff successful, otherwise false.
	 */
	virtual bool process_request(ucdrBuffer *ub, const int64_t time_offset_us) = 0;

	/**
	 * @brief Virtual method that process and send a reply.
	 * @return Returns false iff successful, otherwise false.
	 */
	virtual bool process_reply() = 0;

	/** @var is_reply_pending_ Flag for pending replies */
	bool is_reply_pending_;

	/** @var session_ xrce_dds session pointer */
	uxrSession *session_;

	/** @var reliable_out_stream_id_ output stream */
	uxrStreamId reliable_out_stream_id_;

	/** @var replier_id_ uxrce_dds replier */
	uxrObjectId replier_id_;

	/** @var sample_id_ uxrce_dds sample identifier to link request and reply */
	SampleIdentity sample_id_;

protected:
	/**
	 * @brief xrce_dds replier creator.
	 * @param input_stream_id input stream.
	 * @param participant_id partecipant id.
	 * @param index index used to create the replier id.
	 * @param client_namespace namespace of the client.
	 * @param service_name_simple name of the service.
	 * @param service_type_name_simple name of the service type.
	 * @param queue_depth lenght of the queue.
	 * @return Returns false iff successful, otherwise false.
	 */
	bool create_replier(uxrStreamId input_stream_id, uxrObjectId participant_id, uint16_t index,
			    const char *client_namespace, const char *service_name_simple, const char *service_type_name_simple,
			    uint16_t queue_depth);
};
