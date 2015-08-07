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
#include <drivers/drv_hrt.h>

// static intialization.
uORB::FastRpcChannel uORB::FastRpcChannel::_Instance;

static hrt_abstime _check_time;
static unsigned long _dropped_pkts;
static unsigned long _get_min = 0xFFFFFF;
static unsigned long _get_max = 0;
static unsigned long _min_q = 200;
static unsigned long _max_q = 0;
static unsigned long _avg_q = 0;
static unsigned long _count = 0;
static unsigned long _get_bulk_min = 0xFFFFFF;
static unsigned long _get_bulk_max = 0;
static unsigned long _bulk_topic_count_min = 0xFFFFFF;
static unsigned long _bulk_topic_count_max = 0;

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
		//PX4_DEBUG("******* Found subscriber for message[%s]....", messageName);

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
	hrt_abstime t1, t2;
	static hrt_abstime check_time = 0;
	int32_t initial_queue_size = 0;

	if (_RemoteSubscribers.find(messageName) == _RemoteSubscribers.end()) {
		//there is no-remote subscriber. So do not queue the message.
		return rc;
	}

	t1 = hrt_absolute_time();
	_QueueMutex.lock();
	bool overwriteData = false;

	if (IsDataQFull()) {
		// queue is full.  Overwrite the oldest data.
		//PX4_WARN("[send_message] Queue Full Overwrite the oldest data. in[%ld] out[%ld] max[%ld]",
		//	 _DataQInIndex, _DataQOutIndex, _MAX_MSG_QUEUE_SIZE);
		_DataQOutIndex++;

		if (_DataQOutIndex == _MAX_MSG_QUEUE_SIZE) {
			_DataQOutIndex = 0;
		}

		overwriteData = true;
		_dropped_pkts++;
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
	//if (!overwriteData) {
	if (DataQSize() == 1) {  // post it only of the queue moves from empty to available.
		_DataAvailableSemaphore.post();
	}

	if ((unsigned long)DataQSize() < _min_q) { _min_q = (unsigned long)DataQSize(); }
	if ((unsigned long)DataQSize() > _max_q) { _max_q = (unsigned long)DataQSize(); }

	_count++;
	_avg_q = ((double)((_avg_q * (_count - 1)) + (unsigned long)(DataQSize()))) / (double)(_count);

	_QueueMutex.unlock();
	t2 = hrt_absolute_time();

	if ((unsigned long)(t2 - check_time) > 10000000) {
		//PX4_DEBUG("MsgName: %20s, t1: %lu, t2: %lu, dt: %lu",messageName, (unsigned long) t1, (unsigned long) t2, (unsigned long) (t2-t1));
		//PX4_DEBUG("Q. Stats: min: %lu, max : %lu, avg: %lu count: %lu ", _min_q, _max_q, (unsigned long)(_avg_q * 1000.0),  _count);
		_min_q = _MAX_MSG_QUEUE_SIZE * 2;
		_max_q = 0;
		_avg_q = 0;
		_count = 0;
		check_time = t2;
	}

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
	PX4_DEBUG("Get data should not be called...");
	return -1;
	// wait for data availability
	static hrt_abstime check_time = 0;
	hrt_abstime t1 = hrt_absolute_time();
	_DataAvailableSemaphore.wait();
	hrt_abstime t2 = hrt_absolute_time();
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
	hrt_abstime t3 = hrt_absolute_time();

	if ((unsigned long)(t3 - t1) > _get_max) { _get_max = (unsigned long)(t3 - t1); }

	if ((unsigned long)(t3 - t1) < _get_min) { _get_min = (unsigned long)(t3 - t1); }

	if ((unsigned long)(t3 - check_time) > 1000000) {
		if (rc != 0) {
			topic_name[0] = '\0';
		}
		/*
		PX4_DEBUG("GetData: %30s: t1: %lu t2: %lu t3: %lu", topic_name, (unsigned long)t1, (unsigned long)t2,
			  (unsigned long)t3);
		PX4_DEBUG(".... dt1: %7lu dt2: %7lu Q: %d", (unsigned long)(t2 - t1), (unsigned long)(t3 - t2), DataQSize());
		PX4_DEBUG("ADSP RPC Stats: _get_min: %lu _get_max: %lu _dropped_pkts: %lu", _get_min, _get_max, _dropped_pkts);
		*/
		check_time = t3;
	}

	return rc;
}


int16_t uORB::FastRpcChannel::get_bulk_data
(
	uint8_t *buffer,
	int32_t  max_buffer_in_bytes,
	int32_t *returned_bytes,
	int32_t *topic_count
)
{
	int16_t rc = 0;
	// wait for data availability
	static hrt_abstime check_time = 0;
	hrt_abstime t1 = hrt_absolute_time();
	_DataAvailableSemaphore.wait();
	hrt_abstime t2 = hrt_absolute_time();

	_QueueMutex.lock();

	int32_t bytes_copied = 0;
	int32_t copy_result = 0;
	*returned_bytes = 0;
	*topic_count = 0;
	int32_t topic_count_to_return = 0;

	if (DataQSize() != 0) {
		//PX4_DEBUG( "get_bulk_data: QSize: %d", DataQSize() );
		topic_count_to_return = DataQSize();

		while (DataQSize() != 0) {
			// this is a hack as we are using a counting semaphore.  Should be re-implemented with cond_variable and wait.
			//_DataAvailableSemaphore.wait();
			if (get_data_msg_size_at(_DataQOutIndex) < (max_buffer_in_bytes - bytes_copied)) {
				// there is enough space in the buffer, copy the data.
				//PX4_DEBUG( "Coping Data to buffer..." );
				copy_result = copy_data_to_buffer(_DataQOutIndex, buffer, bytes_copied, max_buffer_in_bytes);

				if (copy_result == -1) {
					if (bytes_copied == 0) {
						rc = -1;
					}

					break;

				} else {
					//PX4_DEBUG( "[%d] %02x %02x %02x %02x", *topic_count,\
					//      buffer[bytes_copied], \
					//      buffer[bytes_copied+1], \
					//      buffer[bytes_copied+2], \
					//      buffer[bytes_copied+3] );
					bytes_copied += copy_result;
					(*topic_count)++;
					*returned_bytes = bytes_copied;
					_DataQOutIndex++;

					if (_DataQOutIndex == _MAX_MSG_QUEUE_SIZE) {
						_DataQOutIndex = 0;
					}
				}

			} else {
				if (bytes_copied == 0) {
					rc = -1;
					PX4_WARN("ERROR: Insufficent space in data buffer, no topics returned");

				} else {
					PX4_DEBUG("Exiting out of the while loop...");
				}

				break;
			}
		}

	} else {
		PX4_ERR("[get_data_bulk] Error: Semaphore is up when there is no data on the control/data queues");
		rc = -1;
	}

	if (topic_count_to_return != *topic_count) {
		PX4_WARN("Not sending all topics: topics_to_return:[%ld] topics_returning:[%ld]", topic_count_to_return, *topic_count);
	}

	_QueueMutex.unlock();
	hrt_abstime t3 = hrt_absolute_time();

	if ((unsigned long)(t3 - t1) > _get_bulk_max) { _get_bulk_max = (unsigned long)(t3 - t1); }
	if ((unsigned long)(t3 - t1) < _get_bulk_min) { _get_bulk_min = (unsigned long)(t3 - t1); }
	if ((unsigned long)(*topic_count) > _bulk_topic_count_max) { _bulk_topic_count_max = (unsigned long)(*topic_count); }
	if ((unsigned long)(*topic_count) < _bulk_topic_count_min) { _bulk_topic_count_min = (unsigned long)(*topic_count); }
	if ((unsigned long)(t3 - check_time) > 10000000) {
		//PX4_DEBUG("GetData: t1: %lu t2: %lu t3: %lu", (unsigned long)t1, (unsigned long)t2, (unsigned long)t3);
		//PX4_DEBUG(".... dt1: %7lu dt2: %7lu Q: %d", (unsigned long)(t2 - t1), (unsigned long)(t3 - t2), DataQSize());
		//PX4_DEBUG("ADSP RPC Stats: _get_bulk_min: %lu _get_bulk_max: %lu _dropped_pkts: %lu", _get_bulk_min, _get_bulk_max,
		//	  _dropped_pkts);
		//PX4_DEBUG(" .... topic_count_min: %lu topic_count_max: %lu", _bulk_topic_count_min, _bulk_topic_count_max);
		_get_bulk_max = 0;
		_get_bulk_min = 0xFFFFFF;
		_bulk_topic_count_min = 0xFFFFFF;
		_bulk_topic_count_max = 0;
		check_time = t3;
	}

	//PX4_DEBUG( "Returning topics: %d bytes_returned: %d", *topic_count, *returned_bytes );
	return rc;
}


int32_t uORB::FastRpcChannel::get_data_msg_size_at(int32_t index)
{
	// the assumption here is that this is called within the context of semaphore,
	// hence lock/unlock is not needed.
	int32_t rc = 0;
	rc += _DataMsgQueue[ index ]._Length;
	rc += _DataMsgQueue[ index ]._MsgName.size() + 1;
	rc += _PACKET_HEADER_SIZE;
	return rc;
}


int32_t uORB::FastRpcChannel::copy_data_to_buffer(int32_t src_index, uint8_t *dst_buffer, int32_t offset,
		int32_t dst_buffer_len)
{
	int32_t rc = -1;

	// before calling this method the following are assumed:
	// *  sem_lock is acquired for data protection
	// *  the dst_buffer is validated to

	// compute the different offsets to pack the packets.
	int32_t field_header_offset = offset;
	int32_t field_topic_name_offset = field_header_offset + sizeof(struct BulkTransferHeader);
	int32_t field_data_offset       = field_topic_name_offset + _DataMsgQueue[ src_index ]._MsgName.size() + 1;

	struct BulkTransferHeader header = { (uint16_t)(_DataMsgQueue[ src_index ]._MsgName.size() + 1), (uint16_t)(_DataMsgQueue[ src_index ]._Length) };


	//PX4_DEBUG( "Offsets: header[%d] name[%d] data[%d]",
	//           field_header_offset,
	//           field_topic_name_offset,
	//           field_data_offset );

	if ((field_data_offset + _DataMsgQueue[ src_index ]._Length) < dst_buffer_len) {
		memmove(&(dst_buffer[field_header_offset]), (char *)(&header), sizeof(header));
		// pack the data here.
		memmove
		(
			&(dst_buffer[field_topic_name_offset]),
			_DataMsgQueue[ src_index ]._MsgName.c_str(),
			_DataMsgQueue[ src_index ]._MsgName.size()
		);

		if (_DataMsgQueue[ src_index ]._MsgName.size() == 0) {
			PX4_WARN("##########  Error MsgName cannot be zero: ");
		}

		dst_buffer[ field_topic_name_offset + _DataMsgQueue[ src_index ]._MsgName.size()] = '\0';
		memmove(&(dst_buffer[field_data_offset]), _DataMsgQueue[ src_index ]._Buffer, _DataMsgQueue[ src_index ]._Length);
		rc = field_data_offset + _DataMsgQueue[ src_index ]._Length - offset;

	} else {
		PX4_WARN("Error coping the DataMsg to dst buffer, insuffienct space. ");
		PX4_WARN("... offset[%ld] len[%ld] data_msg_len[%ld]",
			 offset, dst_buffer_len, (field_data_offset - offset) + _DataMsgQueue[ src_index ]._Length);
	}

	return rc;
}
