/****************************************************************************
 *
 * Copyright (C) 2022 ModalAI, Inc. All rights reserved.
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

#ifndef _uORBAppsProtobufChannel_hpp_
#define _uORBAppsProtobufChannel_hpp_

#include <stdint.h>
#include <string>
#include "MUORBTest.hpp"
#include <px4_platform_common/log.h>

namespace uORB
{
class AppsProtobufChannel;
}

class uORB::AppsProtobufChannel
{
public:
	/**
	 * static method to get the IChannel Implementor.
	 */
	static uORB::AppsProtobufChannel *GetInstance()
	{
		if (_InstancePtr == nullptr) {
			_InstancePtr = new uORB::AppsProtobufChannel();
		}

		return _InstancePtr;
	}

	/**
	 * Static method to check if there is an instance.
	 */
	static bool isInstance()
	{
		return (_InstancePtr != nullptr);
	}

	bool Initialize(bool enable_debug);

	bool Test();

private: // data members
	static uORB::AppsProtobufChannel           *_InstancePtr;

private://class members.
	/// constructor.
	AppsProtobufChannel() {};

	bool Test(MUORBTestType test_type);

	static bool test_flag;

	static void ReceiveCallback(const char *topic,
				    const uint8_t *data,
				    uint32_t length_in_bytes);
	static void AdvertiseCallback(const char *topic);
	static void SubscribeCallback(const char *topic);
	static void UnsubscribeCallback(const char *topic);

};

#endif /* _uORBAppsProtobufChannel_hpp_ */
