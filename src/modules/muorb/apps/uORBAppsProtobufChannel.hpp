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
#include <map>

#include <px4_platform_common/log.h>

#include "MUORBTest.hpp"
#include "uORB/uORBCommunicator.hpp"
#include "mUORBAggregator.hpp"

namespace uORB
{
class AppsProtobufChannel;
}

class uORB::AppsProtobufChannel : public uORBCommunicator::IChannel
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

	/**
	 * @brief Method used to initialize the fc sensor callback on the apps side.
	 *
	 * @param enable_debug
	 * 	This represents the debug boolean. If this is set to true, then specific
	 *	log information will be printed out.
	 * @return
	 * 	1 = success; This means the initialization of the fc_sensor data has happened.
	 *  otherwise = failure.
	 */
	bool Initialize(bool enable_debug);

	/**
	 * @brief Interface to notify the remote entity of a topic being advertised.
	 *
	 * @param messageName
	 * 	This represents the uORB message name(aka topic); This message name should be
	 * 	globally unique.
	 * @return
	 * 	0 = success; This means the messages is successfully sent to the receiver
	 * 		Note: This does not mean that the receiver as received it.
	 *  otherwise = failure.
	 */
	int16_t topic_advertised(const char *messageName);

	/**
	 * @brief Interface to notify the remote entity of interest of a
	 * subscription for a message.
	 *
	 * @param messageName
	 * 	This represents the uORB message name; This message name should be
	 * 	globally unique.
	 * @param msgRate
	 * 	The max rate at which the subscriber can accept the messages. This value is unused.
	 * @return
	 * 	0 = success; This means the messages is successfully sent to the receiver
	 * 		Note: This does not mean that the receiver as received it.
	 *  otherwise = failure.
	 */
	int16_t add_subscription(const char *messageName, int msgRateInHz);

	/**
	 * @brief Interface to notify the remote entity of removal of a subscription
	 *
	 * @param messageName
	 * 	This represents the uORB message name; This message name should be
	 * 	globally unique.
	 * @return
	 * 	0 = success; This means the messages is successfully sent to the receiver
	 * 		Note: This does not necessarily mean that the receiver as received it.
	 *  otherwise = failure.
	 */
	int16_t remove_subscription(const char *messageName);

	/**
	 * Register Message Handler.  This is internal for the IChannel implementer*
	 */
	int16_t register_handler(uORBCommunicator::IChannelRxHandler *handler);

	/**
	 * @brief Sends the data message over the communication link.
	 * @param messageName
	 * 	This represents the uORB message name; This message name should be
	 * 	globally unique.
	 * @param length
	 * 	The length of the data buffer to be sent.
	 * @param data
	 * 	The actual data to be sent.
	 * @return
	 *  0 = success; This means the messages is successfully sent to the receiver
	 * 		Note: This does not mean that the receiver as received it.
	 *  otherwise = failure.
	 */
	int16_t send_message(const char *messageName, int length, uint8_t *data);

	/**
	 * @brief Interface to test the functions of the protobuf channel.
	 *
	 * @return
	 * 	1 = success; This means all callbacks have been tested and return the correct value.
	 *  otherwise = failure.
	 */
	bool Test();

private:
	/**
	 * Data Members
	 */
	static uORB::AppsProtobufChannel           *_InstancePtr;
	static uORBCommunicator::IChannelRxHandler *_RxHandler;
	static mUORB::Aggregator					_Aggregator;
	static std::map<std::string, int>           _SlpiSubscriberCache;
	static pthread_mutex_t                      _tx_mutex;
	static pthread_mutex_t                      _rx_mutex;
	static bool                                 _Debug;

	bool                                        _Initialized;
	uint32_t                                    _MessageCounter;

private:
	/**
	 * Class Members
	 */
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

#endif
