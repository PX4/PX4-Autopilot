/****************************************************************************
 *
 * Copyright (C) 2015 Mark Charlebois. All rights reserved.
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
#include <stdint.h>
#include "px4muorb_KraitRpcWrapper.hpp"
#include "px4muorb.h"

using namespace px4muorb;

/**
 * Constructor
 */
KraitRpcWrapper::KraitRpcWrapper() {}

/**
 * destructor
 */
KraitRpcWrapper::~KraitRpcWrapper() {}

/**
 * Initiatizes the rpc channel px4 muorb
 */
bool KraitRpcWrapper::Initialize()
{ 
	return (px4muorb_orb_initialize() == 0);
}

/**
 * Terminate to clean up the resources.  This should be called at program exit
 */
bool KraitRpcWrapper::Terminate()
{
	// FIXME - TBD
	return true;
}

/**
 * Muorb related functions to pub/sub of orb topic from krait to adsp
 */
int32_t KraitRpcWrapper::AddSubscriber(const char *topic)
{
	return px4muorb_add_subscriber(topic);
}

int32_t KraitRpcWrapper::RemoveSubscriber(const char *topic)
{
	return px4muorb_remove_subscriber(topic);
}

int32_t KraitRpcWrapper::SendData(const char *topic, int32_t length_in_bytes, const uint8_t *data)
{
	return px4muorb_send_topic_data(topic, data, length_in_bytes);
}

int32_t KraitRpcWrapper::ReceiveData(int32_t *msg_type, char **topic, int32_t *length_in_bytes, uint8_t **data)
{
	// FIXME ??
	int topic_nameLen = 0;
	int rv = px4muorb_receive_msg(msg_type, *topic, topic_nameLen, *data, *length_in_bytes, length_in_bytes);
	return rv;
}

int32_t KraitRpcWrapper::IsSubscriberPresent(const char *topic, int32_t *status)
{
	return px4muorb_is_subscriber_present(topic, status);
}

int32_t KraitRpcWrapper::ReceiveBulkData(uint8_t **bulk_data, int32_t *length_in_bytes, int32_t *topic_count)
{
	return px4muorb_receive_bulk_data(*bulk_data, *length_in_bytes, length_in_bytes, topic_count);
}

int32_t KraitRpcWrapper::UnblockReceiveData()
{
	return px4muorb_unblock_recieve_msg();
}

