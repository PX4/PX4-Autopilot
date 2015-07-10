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
#include "uORBFastRpcChannel.hpp"
#include "px4_log.h"
#include <algorithm>

// static intialization.
uORB::FastRpcChannel uORB::FastRpcChannel::_Instance;

//==============================================================================
//==============================================================================
uORB::FastRpcChannel::FastRpcChannel()
	: _RxHandler(0)
	, _DataQInIndex(0)
	, _DataQOutIndex(0)
	, _ControlQInIndex(0)
	, _ControlQOutIndex(0)
{
	for (int32_t i = 0; i < _MAX_MSG_QUEUE_SIZE; ++ i) {
		_DataMsgQueue[i]._MaxBufferSize = 0;
		_DataMsgQueue[i]._Length = 0;
		_DataMsgQueue[i]._Buffer = 0;
	}

	_RemoteSubscribers.clear();
}

//==============================================================================
//==============================================================================
int16_t uORB::FastRpcChannel::add_subscription(const char *messageName, int32_t msgRateInHz)
{
	int16_t rc = 0;
	_Subscribers.push_back(messageName);
	PX4_DEBUG("Adding message[%s] to subscriber queue...", messageName);
	return rc;
}

//==============================================================================
//==============================================================================
int16_t uORB::FastRpcChannel::remove_subscription(const char *messageName)
{
	int16_t rc = 0;
	_Subscribers.remove(messageName);

	return rc;
}

int16_t uORB::FastRpcChannel::is_subscriber_present(const char *messageName, int32_t *status)
{
	int16_t rc = 0;

	if (std::find(_Subscribers.begin(), _Subscribers.end(), messageName) != _Subscribers.end()) {
		*status = 1;
		PX4_DEBUG("******* Found subscriber for message[%s]....", messageName);

	} else {
		*status = 0;
		//PX4_WARN("@@@@@ Subscriber not found for[%s]...numSubscribers[%d]", messageName, _Subscribers.size());
		int i = 0;

		for (std::list<std::string>::iterator it = _Subscribers.begin(); it != _Subscribers.end(); ++it) {
			if (*it == messageName) {
				PX4_DEBUG("##### Found the message[%s] in the subscriber list-index[%d]", messageName, i);
			}

			++i;
		}
	}

	return rc;
}

int16_t uORB::FastRpcChannel::unblock_get_data_method()
{
	PX4_DEBUG("[unblock_get_data_method] calling post method for _DataAvailableSemaphore()");
	_DataAvailableSemaphore.post();
	return 0;
}
//==============================================================================
//==============================================================================
int16_t uORB::FastRpcChannel::register_handler(uORBCommunicator::IChannelRxHandler *handler)
{
	_RxHandler = handler;
	return 0;
}


//==============================================================================
//==============================================================================
int16_t uORB::FastRpcChannel::send_message(const char *messageName, int32_t length, uint8_t *data)
{
	int16_t rc = 0;

	if (_RemoteSubscribers.find(messageName) == _RemoteSubscribers.end()) {
		//there is no-remote subscriber. So do not queue the message.
		return rc;
	}

	_QueueMutex.lock();
	bool overwriteData = false;

	if (IsDataQFull()) {
		// queue is full.  Overwrite the oldest data.
		PX4_WARN("[send_message] Queue Full Overwrite the oldest data. in[%ld] out[%ld] max[%ld]",
			 _DataQInIndex, _DataQOutIndex, _MAX_MSG_QUEUE_SIZE);
		_DataQOutIndex++;

		if (_DataQOutIndex == _MAX_MSG_QUEUE_SIZE) {
			_DataQOutIndex = 0;
		}

		overwriteData = true;
	}

	// now check to see if the data queue's buffer size if large enough to memcpy the data.
	// if not, delete the old buffer and re-create a new buffer of larger size.
	check_and_expand_data_buffer(_DataQInIndex, length);

	// now memcpy the data to the buffer.
	memcpy(_DataMsgQueue[ _DataQInIndex ]._Buffer, data, length);
	_DataMsgQueue[ _DataQInIndex ]._Length = length;
	_DataMsgQueue[ _DataQInIndex ]._MsgName = messageName;

	_DataQInIndex++;

	if (_DataQInIndex == _MAX_MSG_QUEUE_SIZE) {
		_DataQInIndex = 0;
	}

	// the assumption here is that each caller reads only one data from either control or data queue.
	if (!overwriteData) {
		_DataAvailableSemaphore.post();
	}

	_QueueMutex.unlock();
	return rc;
}

//==============================================================================
//==============================================================================
void uORB::FastRpcChannel::check_and_expand_data_buffer(int32_t index, int32_t length)
{
	if (_DataMsgQueue[ index ]._MaxBufferSize < length) {
		// create a new buffer of size length and delete old buffer.
		if (_DataMsgQueue[ index ]._Buffer != 0) {
			delete _DataMsgQueue[ index ]._Buffer;
		}

		_DataMsgQueue[ index ]._Buffer = new uint8_t[ length ];

		if (_DataMsgQueue[ index ]._Buffer == 0) {
			PX4_ERR("Error[check_and_expand_data_buffer] Failed to allocate data queue buffer of size[%ld]", length);
			_DataMsgQueue[ index ]._MaxBufferSize = 0;
			return;
		}

		_DataMsgQueue[ index ]._MaxBufferSize = length;
	}
}

int32_t uORB::FastRpcChannel::DataQSize()
{
	int32_t rc;
	rc = (_DataQInIndex - _DataQOutIndex) + _MAX_MSG_QUEUE_SIZE;
	rc %= _MAX_MSG_QUEUE_SIZE;
	return rc;
}

int32_t uORB::FastRpcChannel::ControlQSize()
{
	int32_t rc;
	rc = (_ControlQInIndex - _ControlQOutIndex) + _MAX_MSG_QUEUE_SIZE;
	rc %= _MAX_MSG_QUEUE_SIZE;
	return rc;
}

bool uORB::FastRpcChannel::IsControlQFull()
{
	return (ControlQSize() == (_MAX_MSG_QUEUE_SIZE - 1));
}

bool uORB::FastRpcChannel::IsControlQEmpty()
{
	return (ControlQSize() == 0);
}

bool uORB::FastRpcChannel::IsDataQFull()
{
	return (DataQSize() == (_MAX_MSG_QUEUE_SIZE - 1));
}

bool uORB::FastRpcChannel::IsDataQEmpty()
{
	return (DataQSize() == 0);
}

int16_t uORB::FastRpcChannel::get_data
(
	int32_t *msg_type,
	char *topic_name,
	int32_t topic_name_len,
	uint8_t *data,
	int32_t data_len_in_bytes,
	int32_t *bytes_returned
)
{
	int16_t rc = 0;
	// wait for data availability
	_DataAvailableSemaphore.wait();
	_QueueMutex.lock();

	if (DataQSize() != 0 || ControlQSize() != 0) {
		if (ControlQSize() > 0) {
			// read the first element of the Control Queue.
			*msg_type = _ControlMsgQueue[ _ControlQOutIndex ]._Type;

			if ((int)_ControlMsgQueue[ _ControlQOutIndex ]._MsgName.size() < (int)topic_name_len) {
				memcpy
				(
					topic_name,
					_ControlMsgQueue[ _ControlQOutIndex ]._MsgName.c_str(),
					_ControlMsgQueue[ _ControlQOutIndex ]._MsgName.size()
				);

				topic_name[_ControlMsgQueue[ _ControlQOutIndex ]._MsgName.size()] = 0;

				*bytes_returned = 0;

				_ControlQOutIndex++;

				if (_ControlQOutIndex == _MAX_MSG_QUEUE_SIZE) {
					_ControlQOutIndex = 0;
				}

			} else {
				PX4_ERR("Error[get_data-CONTROL]: max topic_name_len[%ld] < controlMsgLen[%d]",
					topic_name_len,
					_ControlMsgQueue[ _ControlQOutIndex ]._MsgName.size()
				       );
				rc = -1;
			}

		} else {
			// read the first element of the Control Queue.
			*msg_type = _DATA_MSG_TYPE;

			if (((int)_DataMsgQueue[ _DataQOutIndex ]._MsgName.size() < topic_name_len) ||
			    (_DataMsgQueue[ _DataQOutIndex ]._Length < data_len_in_bytes)) {
				memcpy
				(
					topic_name,
					_DataMsgQueue[ _DataQOutIndex ]._MsgName.c_str(),
					_DataMsgQueue[ _DataQOutIndex ]._MsgName.size()
				);

				topic_name[_DataMsgQueue[ _DataQOutIndex ]._MsgName.size()] = 0;

				*bytes_returned = _DataMsgQueue[ _DataQOutIndex ]._Length;
				memcpy(data, _DataMsgQueue[ _DataQOutIndex ]._Buffer, _DataMsgQueue[ _DataQOutIndex ]._Length);

				_DataQOutIndex++;

				if (_DataQOutIndex == _MAX_MSG_QUEUE_SIZE) {
					_DataQOutIndex = 0;
				}

			} else {
				PX4_ERR("Error:[get_data-DATA] type msg max topic_name_len[%ld] > dataMsgLen[%d] ",
					topic_name_len,
					_DataMsgQueue[ _DataQOutIndex ]._MsgName.size()
				       );
				PX4_ERR("Error:[get_data-DATA] Or data_buffer_len[%ld] > message_size[%ld] ",
					data_len_in_bytes,
					_DataMsgQueue[ _DataQOutIndex ]._Length
				       );

				rc = -1;
			}
		}

	} else {
		PX4_ERR("[get_data] Error: Semaphore is up when there is no data on the control/data queues");
		rc = -1;
	}

	_QueueMutex.unlock();
	return rc;
}
