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

#include "uORBKraitFastRpcChannel.hpp"
#include "px4_log.h"


#define LOG_TAG "uORBKraitFastRpcChannel.cpp"

// static intialization.
uORB::KraitFastRpcChannel uORB::KraitFastRpcChannel::_Instance;

//==============================================================================
//==============================================================================
uORB::KraitFastRpcChannel::KraitFastRpcChannel()
	: _RxHandler(nullptr)
	, _ThreadStarted(false)
	, _ShouldExit(false)
{
	_KraitWrapper.Initialize();
}

//==============================================================================
//==============================================================================
int16_t uORB::KraitFastRpcChannel::add_subscription(const char *messageName, int32_t msgRateInHz)
{
	int16_t rc = 0;
	// invoke fast_rpc call. From Idl.
	PX4_DEBUG("Before calling AddSubscriber for [%s]\n", messageName);
	rc = _KraitWrapper.AddSubscriber(messageName);
	PX4_DEBUG("Response for AddSubscriber for [%s], rc[%d]\n", messageName, rc);
	return rc;
}

//==============================================================================
//==============================================================================
int16_t uORB::KraitFastRpcChannel::remove_subscription(const char *messageName)
{
	int16_t rc = 0;
	// invoke the fast_rpc call defined in idl.
	PX4_DEBUG("Before calling RemoveSubscriber for [%s]\n", messageName);
	rc = _KraitWrapper.RemoveSubscriber(messageName);
	PX4_DEBUG("Response for RemoveSubscriber for [%s], rc[%d]\n", messageName, rc);
	return rc;
}

//==============================================================================
//==============================================================================
int16_t uORB::KraitFastRpcChannel::register_handler(uORBCommunicator::IChannelRxHandler *handler)
{
	_RxHandler = handler;
	return 0;
}


//==============================================================================
//==============================================================================
int16_t uORB::KraitFastRpcChannel::send_message(const char *messageName, int32_t length, uint8_t *data)
{
	int16_t rc = 0;
	// invoke the fast rpc call to send data defined in idl.
	//PX4_DEBUG( "Before calling send_data for [%s] len[%d]\n", messageName.c_str(), length );
	int32_t status = 0;

	if (_KraitWrapper.IsSubscriberPresent(messageName, &status) == 0) {
		if (status > 0) { // there are remote subscribers
			rc = _KraitWrapper.SendData(messageName, length, data);
			//PX4_DEBUG( "***** SENDING[%s] topic to remote....\n", messageName.c_str() );

		} else {
			//PX4_DEBUG( "******* NO SUBSCRIBER PRESENT ON THE REMOTE FOR topic[%s] \n", messageName.c_str() );
		}
	} else {
		PX4_ERR("Error returned for KraitWrapper.IsSubscriberPresent(%s)\n", messageName);
	}

	//PX4_DEBUG( "Response for SendMessage for [%s],len[%d] rc[%d]\n", messageName.c_str(), length, rc );
	return rc;
}

void uORB::KraitFastRpcChannel::Start()
{
	_ThreadStarted = true;
	pthread_create(&_RecvThread, NULL, thread_start, this);
}

void uORB::KraitFastRpcChannel::Stop()
{
	_ShouldExit = true;
	_KraitWrapper.UnblockReceiveData();
	PX4_DEBUG("After calling krait_wrapper_unlock_receive_Data...\n");
	pthread_join(_RecvThread, NULL);
	PX4_DEBUG("*** After calling thread wait...\n");
	_ThreadStarted = false;
	_ShouldExit = false;
}


void uORB::KraitFastRpcChannel::thread_start(void *handler)
{
	if (handler != nullptr) {
		((uORB::KraitFastRpcChannel *)handler)->fastrpc_recv_thread();
	}
}

void uORB::KraitFastRpcChannel::fastrpc_recv_thread()
{
	// sit in while loop.
	int32_t rc = 0;
	int32_t  type = 0;
	char    *name = nullptr;
	int32_t  data_length = 0;
	uint8_t *data = nullptr;

	while (!_ShouldExit) {
		// call the fastrpc recv data call.
		//uorb_fastrpc_recieve( &type, &name_len, name, &data_length, data );
		rc = _KraitWrapper.ReceiveData(&type, &name, &data_length, &data);

		if (rc == 0) {
			switch (type) {
			case _CONTROL_MSG_TYPE_ADD_SUBSCRIBER:
				if (_RxHandler != nullptr) {
					_RxHandler->process_add_subscription(name, 1);
					PX4_DEBUG("Received add subscriber control message for: [%s]\n", name);
				}

				break;

			case _CONTROL_MSG_TYPE_REMOVE_SUBSCRIBER:
				if (_RxHandler != nullptr) {
					_RxHandler->process_remove_subscription(name);
					PX4_DEBUG("Received remove subscriber control message for: [%s]\n", name);
				}

				break;

			case _DATA_MSG_TYPE:
				if (_RxHandler != nullptr) {
					_RxHandler->process_received_message(name,
									     data_length, data);
					//PX4_DEBUG( "Received topic data for control message for: [%s] len[%d]\n", name, data_length );
				}

				break;

			default:
				// error condition.
				break;
			}

		} else {
			PX4_DEBUG("Error: Getting data over fastRPC channel\n");
			break;
		}
	}

	PX4_DEBUG("[uORB::KraitFastRpcChannel::fastrpc_recv_thread] Exiting fastrpc_recv_thread\n");
}

