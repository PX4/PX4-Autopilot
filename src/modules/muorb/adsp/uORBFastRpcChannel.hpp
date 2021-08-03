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
#ifndef _uORBFastRpcChannel_hpp_
#define _uORBFastRpcChannel_hpp_

#include <stdint.h>
#include <string>
#include <list>
#include "uORB/uORBCommunicator.hpp"
#include <semaphore.h>
#include <set>
#include <px4_platform_common/sem.h>

namespace uORB
{
class FastRpcChannel;
}

class uORB::FastRpcChannel : public uORBCommunicator::IChannel
{
public:
	/**
	 * static method to get the IChannel Implementor.
	 */
	static uORB::FastRpcChannel *GetInstance()
	{
		return &(_Instance);
	}

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
	virtual int16_t topic_advertised(const char *messageName);

	/**
	 * @brief Interface to notify the remote entity of a topic being unadvertised
	 * and is no longer publishing messages.
	 *
	 * @param messageName
	 * 	This represents the uORB message name(aka topic); This message name should be
	 * 	globally unique.
	 * @return
	 * 	0 = success; This means the messages is successfully sent to the receiver
	 * 		Note: This does not mean that the receiver as received it.
	 *  otherwise = failure.
	 */
	virtual int16_t topic_unadvertised(const char *messageName);

	/**
	 * @brief Interface to notify the remote entity of interest of a
	 * subscription for a message.
	 *
	 * @param messageName
	 * 	This represents the uORB message name; This message name should be
	 * 	globally unique.
	 * @param msgRate
	 * 	The max rate at which the subscriber can accept the messages.
	 * @return
	 * 	0 = success; This means the messages is successfully sent to the receiver
	 * 		Note: This does not mean that the receiver as received it.
	 *  otherwise = failure.
	 */
	virtual int16_t add_subscription(const char *messageName, int32_t msgRateInHz);


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
	virtual int16_t remove_subscription(const char *messageName);

	/**
	 * Register Message Handler.  This is internal for the IChannel implementer*
	 */
	virtual int16_t register_handler(uORBCommunicator::IChannelRxHandler *handler);


	//=========================================================================
	//     INTERFACES FOR Data messages
	//=========================================================================

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
	virtual int16_t send_message(const char *messageName, int32_t length, uint8_t *data);

	//Function to return the data to krait.
	int16_t get_data
	(
		int32_t *msg_type,
		char *topic_name,
		int32_t topic_name_len,
		uint8_t *data,
		int32_t data_len_in_bytes,
		int32_t *bytes_returned
	);

	int16_t get_bulk_data(uint8_t *buffer, int32_t max_size_in_bytes, int32_t *returned_bytes, int32_t *topic_count);

	// function to check if there are subscribers for a topic on adsp.
	int16_t is_subscriber_present(const char *messageName, int32_t *status);

	// function to release the blocking semaphore for get_data method.
	int16_t unblock_get_data_method();

	uORBCommunicator::IChannelRxHandler *GetRxHandler()
	{
		return _RxHandler;
	}

	void AddRemoteSubscriber(const std::string &messageName)
	{
		_RemoteSubscribers.insert(messageName);
	}
	void RemoveRemoteSubscriber(const std::string &messageName)
	{
		_RemoteSubscribers.erase(messageName);
	}

private: // data members
	static uORB::FastRpcChannel _Instance;
	uORBCommunicator::IChannelRxHandler *_RxHandler;

	/// data structure to store the messages to be retrived by Krait.
	static const int32_t _MAX_MSG_QUEUE_SIZE = 100;
	static const int32_t _CONTROL_MSG_TYPE_ADD_SUBSCRIBER = 1;
	static const int32_t _CONTROL_MSG_TYPE_REMOVE_SUBSCRIBER = 2;
	static const int32_t _DATA_MSG_TYPE = 3;
	static const int32_t _CONTROL_MSG_TYPE_ADVERTISE = 4;
	static const int32_t _CONTROL_MSG_TYPE_UNADVERTISE = 5;

	static const int32_t _PACKET_FIELD_TOPIC_NAME_LEN_SIZE_IN_BYTES = 2;
	static const int32_t _PACKET_FIELD_DATA_LEN_IN_BYTES = 2;
	static const int32_t _PACKET_HEADER_SIZE = 1 + //first byte is the MSG Type
			_PACKET_FIELD_TOPIC_NAME_LEN_SIZE_IN_BYTES + _PACKET_FIELD_DATA_LEN_IN_BYTES;

	struct FastRpcDataMsg {
		int32_t     _MaxBufferSize;
		int32_t     _Length;
		uint8_t    *_Buffer;
		std::string _MsgName;
	};

	struct FastRpcControlMsg {
		int32_t _Type;
		std::string _MsgName;
	};

	struct BulkTransferHeader {
		uint16_t _MsgType;
		uint16_t _MsgNameLen;
		uint16_t _DataLen;
	};


	struct FastRpcDataMsg _DataMsgQueue[ _MAX_MSG_QUEUE_SIZE ];
	int32_t _DataQInIndex;
	int32_t _DataQOutIndex;

	struct FastRpcControlMsg _ControlMsgQueue[ _MAX_MSG_QUEUE_SIZE ];
	int32_t _ControlQInIndex;
	int32_t _ControlQOutIndex;

	std::list<std::string> _Subscribers;

	//utility classes
	class Mutex
	{
	public:
		Mutex()
		{
			sem_init(&_Sem, 0, 1);
		}
		~Mutex()
		{
			sem_destroy(&_Sem);
		}
		void lock()
		{
			sem_wait(&_Sem);
		}
		void unlock()
		{
			sem_post(&_Sem);
		}
	private:
		sem_t _Sem;

		Mutex(const Mutex &);

		Mutex &operator=(const Mutex &);
	};

	class Semaphore
	{
	public:
		Semaphore()
		{
			sem_init(&_Sem, 0, 0);
			/* _Sem use case is a signal */
			px4_sem_setprotocol(&_Sem, SEM_PRIO_NONE);
		}
		~Semaphore()
		{
			sem_destroy(&_Sem);
		}
		void post()
		{
			sem_post(&_Sem);
		}
		void wait()
		{
			sem_wait(&_Sem);
		}
	private:
		sem_t _Sem;
		Semaphore(const Semaphore &);
		Semaphore &operator=(const Semaphore &);

	};

	Mutex _QueueMutex;
	Semaphore _DataAvailableSemaphore;

private://class members.
	/// constructor.
	FastRpcChannel();

	void check_and_expand_data_buffer(int32_t index, int32_t length);

	bool IsControlQFull();
	bool IsControlQEmpty();
	bool IsDataQFull();
	bool IsDataQEmpty();
	int32_t DataQSize();
	int32_t ControlQSize();

	int32_t get_msg_size_at(bool isData, int32_t index);
	int32_t copy_msg_to_buffer(bool isData, int32_t src_index, uint8_t *dst_buffer, int32_t offset, int32_t dst_buffer_len);
	int16_t control_msg_queue_add(int32_t msgtype, const char *messageName);

	std::set<std::string> _RemoteSubscribers;
};

#endif /* _uORBFastRpcChannel_hpp_ */
