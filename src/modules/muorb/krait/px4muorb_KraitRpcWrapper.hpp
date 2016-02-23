/****************************************************************************
 *
 * Copyright (c) 2016 Ramakrishna Kintada. All rights reserved.
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
#ifndef _px4muorb_KraitRpcWrapper_hpp_
#define _px4muorb_KraitRpcWrapper_hpp_
#include <stdint.h>

namespace px4muorb
{
class KraitRpcWrapper;
}

class px4muorb::KraitRpcWrapper
{
public:
	/**
	 * Constructor
	 */
	KraitRpcWrapper();

	/**
	 * destructor
	 */
	~KraitRpcWrapper();

	/**
	 * Initiatizes the rpc channel px4 muorb
	 */
	bool Initialize();

	/**
	 * Terminate to clean up the resources.  This should be called at program exit
	 */
	bool Terminate();

	/**
	 * Muorb related functions to pub/sub of orb topic from krait to adsp
	 */
	int32_t AddSubscriber(const char *topic);
	int32_t RemoveSubscriber(const char *topic);
	int32_t SendData(const char *topic, int32_t length_in_bytes, const uint8_t *data);
	int32_t ReceiveData(int32_t *msg_type, char **topic, int32_t *length_in_bytes, uint8_t **data);
	int32_t IsSubscriberPresent(const char *topic, int32_t *status);
	int32_t ReceiveBulkData(uint8_t **bulk_data, int32_t *length_in_bytes, int32_t *topic_count);
	int32_t UnblockReceiveData();
};
#endif // _px4muorb_KraitWrapper_hpp_
