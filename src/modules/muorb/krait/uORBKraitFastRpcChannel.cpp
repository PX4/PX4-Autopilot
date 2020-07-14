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
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <drivers/drv_hrt.h>
#include <cstdio>
#include <pthread.h>
#include <string.h>

#define LOG_TAG "uORBKraitFastRpcChannel.cpp"

uORB::KraitFastRpcChannel *uORB::KraitFastRpcChannel::_InstancePtr = nullptr;

static void DumpData(uint8_t *buffer, int32_t length, int32_t num_topics);

// static initialization.
static std::string _log_file_name = "./hex_dump.txt";

static unsigned long _snd_msg_min = 0xFFFFFF;
static unsigned long _snd_msg_max = 0;
static double        _snd_msg_avg = 0.0;
static unsigned long _snd_msg_count = 0;
static unsigned long _overall_snd_min = 0xFFFFFF;
static unsigned long _overall_snd_max = 0;
static double        _overall_snd_avg = 0.0;
static unsigned long _overall_snd_count = 0;
static hrt_abstime   _log_check_time = 0;
static hrt_abstime   _log_check_interval = 10000000;


uORB::KraitFastRpcChannel::KraitFastRpcChannel() :
	_RxHandler(nullptr),
	_ThreadStarted(false),
	_ThreadShouldExit(false)
{
	_KraitWrapper.Initialize();
}

int16_t uORB::KraitFastRpcChannel::topic_advertised(const char *messageName)
{
	int16_t rc = 0;
	PX4_DEBUG("Before calling TopicAdvertised for [%s]\n", messageName);
	rc = _KraitWrapper.TopicAdvertised(messageName);
	PX4_DEBUG("Response for TopicAdvertised for [%s], rc[%d]\n", messageName, rc);
	return rc;
}

int16_t uORB::KraitFastRpcChannel::topic_unadvertised(const char *messageName)
{
	int16_t rc = 0;
	PX4_DEBUG("Before calling TopicUnadvertised for [%s]\n", messageName);
	rc = _KraitWrapper.TopicUnadvertised(messageName);
	PX4_DEBUG("Response for TopicUnadvertised for [%s], rc[%d]\n", messageName, rc);
	return rc;
}

int16_t uORB::KraitFastRpcChannel::add_subscription(const char *messageName, int32_t msgRateInHz)
{
	int16_t rc = 0;
	//PX4_DEBUG("Before calling AddSubscriber for [%s]\n", messageName);
	rc = _KraitWrapper.AddSubscriber(messageName);
	//PX4_DEBUG("Response for AddSubscriber for [%s], rc[%d]\n", messageName, rc);
	return rc;
}

int16_t uORB::KraitFastRpcChannel::remove_subscription(const char *messageName)
{
	int16_t rc = 0;
	//PX4_DEBUG("Before calling RemoveSubscriber for [%s]\n", messageName);
	rc = _KraitWrapper.RemoveSubscriber(messageName);
	//PX4_DEBUG("Response for RemoveSubscriber for [%s], rc[%d]\n", messageName, rc);
	return rc;
}

int16_t uORB::KraitFastRpcChannel::register_handler(uORBCommunicator::IChannelRxHandler *handler)
{
	_RxHandler = handler;
	return 0;
}

int16_t uORB::KraitFastRpcChannel::send_message(const char *messageName, int32_t length, uint8_t *data)
{
	int16_t rc = 0;
	int32_t status = 0;
	hrt_abstime t1, t4;
	hrt_abstime t2 = 0;
	hrt_abstime t3 = 0;
	t1 = hrt_absolute_time();

	if (_AdspSubscriberCache.find(std::string(messageName)) == _AdspSubscriberCache.end()) {
		// check the status from adsp. as it is not cached.
		if (_KraitWrapper.IsSubscriberPresent(messageName, &status) == 0) {
			_AdspSubscriberCache[messageName] = status;
			_AdspSubscriberSampleTimestamp[messageName] = hrt_absolute_time();
		}

	} else {
		if ((hrt_absolute_time() - _AdspSubscriberSampleTimestamp[messageName]) > _SubCacheRefreshRate) {
			if (_KraitWrapper.IsSubscriberPresent(messageName, &status) == 0) {
				_AdspSubscriberCache[messageName] = status;
				_AdspSubscriberSampleTimestamp[messageName] = hrt_absolute_time();
			}
		}
	}

	if (_AdspSubscriberCache[messageName] > 0) {// there are remote subscribers
		t2 = hrt_absolute_time();
		rc = _KraitWrapper.SendData(messageName, length, data);
		t3 = hrt_absolute_time();
		_snd_msg_count++;
		//PX4_DEBUG( "***** SENDING[%s] topic to remote....\n", messageName.c_str() );

	} else {
		//PX4_DEBUG( "******* NO SUBSCRIBER PRESENT ON THE REMOTE FOR topic[%s] \n", messageName.c_str() );
	}

	t4 = hrt_absolute_time();
	_overall_snd_count++;

	if ((t4 - t1) < _overall_snd_min) { _overall_snd_min = (t4 - t1); }

	if ((t4 - t1) > _overall_snd_max) { _overall_snd_max = (t4 - t1); }

	if (_AdspSubscriberCache[messageName] > 0) {
		if ((t3 - t2) < _snd_msg_min) { _snd_msg_min = (t3 - t2); }

		if ((t3 - t2) > _snd_msg_max) { _snd_msg_max = (t3 - t2); }

		_snd_msg_avg = ((double)((_snd_msg_avg * (_snd_msg_count - 1)) +
					 (unsigned long)(t3 - t2))) / (double)(_snd_msg_count);
	}

	_overall_snd_avg = ((double)((_overall_snd_avg * (_overall_snd_count - 1)) +
				     (unsigned long)(t4 - t1))) / (double)(_overall_snd_count);

	if ((t4 - _log_check_time) > _log_check_interval) {
		/*
		PX4_DEBUG("SndMsgStats: overall_min: %lu overall_max: %lu snd_msg_min: %lu snd_msg_max: %lu",
			  _overall_snd_min, _overall_snd_max,
			  _snd_msg_min, _snd_msg_max);
		PX4_DEBUG(".... overall_avg: %f (%lu) snd_msg_avg: %f (%lu)",
			  _overall_snd_avg, _overall_snd_count, _snd_msg_avg, _snd_msg_count);
		*/
		_log_check_time = t4;
		_overall_snd_min = _snd_msg_min = 0xFFFFFFF;
		_overall_snd_max = _snd_msg_max = 0;
		_overall_snd_count = _snd_msg_count = 0;
		_overall_snd_avg = _snd_msg_avg = 0.0;
	}

	//PX4_DEBUG( "Response for SendMessage for [%s],len[%d] rc[%d]\n", messageName.c_str(), length, rc );
	return rc;
}

void uORB::KraitFastRpcChannel::Start()
{
	_ThreadStarted = true;
	_ThreadShouldExit = false;
	pthread_attr_t recv_thread_attr;
	pthread_attr_init(&recv_thread_attr);

	struct sched_param param;
	(void)pthread_attr_getschedparam(&recv_thread_attr, &param);
	param.sched_priority = SCHED_PRIORITY_MAX - 80;
	(void)pthread_attr_setschedparam(&recv_thread_attr, &param);

	pthread_attr_setstacksize(&recv_thread_attr, 4096);

	if (pthread_create(&_RecvThread, &recv_thread_attr, thread_start, (void *)this) != 0) {
		PX4_ERR("Error  creating the receive thread for muorb");

	} else {
		pthread_setname_np(_RecvThread, "muorb_krait_receiver");
	}

	pthread_attr_destroy(&recv_thread_attr);
}

void uORB::KraitFastRpcChannel::Stop()
{
	_ThreadShouldExit = true;
	_KraitWrapper.UnblockReceiveData();
	//PX4_DEBUG("After calling UnblockReceiveData()...\n");
	pthread_join(_RecvThread, NULL);
	//PX4_DEBUG("*** After calling pthread_join...\n");
	_ThreadStarted = false;
}

void  *uORB::KraitFastRpcChannel::thread_start(void *handler)
{
	if (handler != nullptr) {
		((uORB::KraitFastRpcChannel *)handler)->fastrpc_recv_thread();
	}

	return 0;
}

void uORB::KraitFastRpcChannel::fastrpc_recv_thread()
{
	// sit in while loop.
	int32_t rc = 0;
	int32_t  data_length = 0;
	uint8_t *data = nullptr;
	unsigned long rpc_min, rpc_max;
	unsigned long orb_min, orb_max;
	double rpc_avg, orb_avg;
	unsigned long count = 0;
	rpc_max = orb_max = 0;
	rpc_min = orb_min = 0xFFFFFFFF;
	rpc_avg = orb_avg = 0.0;

	int32_t num_topics = 0;

	hrt_abstime check_time = 0;

	while (!_ThreadShouldExit) {
		hrt_abstime t1, t2, t3;
		t1 = hrt_absolute_time();
		rc = _KraitWrapper.ReceiveBulkData(&data, &data_length, &num_topics);

		t2 = hrt_absolute_time();

		if (rc == 0) {
			//PX4_DEBUG( "Num of topics Received: %d", num_topics );
			int32_t bytes_processed = 0;

			for (int i = 0; i < num_topics; ++i) {
				uint8_t *new_pkt = &(data[bytes_processed]);
				struct BulkTransferHeader *header = (struct BulkTransferHeader *)new_pkt;
				char *messageName = (char *)(new_pkt + sizeof(struct BulkTransferHeader));
				uint16_t check_msg_len = strlen(messageName);

				if (header->_MsgNameLen != (check_msg_len + 1)) {
					PX4_ERR("Error: Packing error.  Sent Msg Len. of[%d] but strlen returned:[%d]", header->_MsgNameLen, check_msg_len);
					PX4_ERR("Error: NumTopics: %d processing topic: %d msgLen[%d] dataLen[%d] data_len[%d] bytes processed: %d",
						num_topics, i, header->_MsgNameLen, header->_DataLen, data_length, bytes_processed);
					DumpData(data, data_length, num_topics);
					break;
				}

				uint8_t *topic_data = (uint8_t *)(messageName + strlen(messageName) + 1);

				if (_RxHandler != nullptr) {
					if (header->_MsgType == _DATA_MSG_TYPE) {
						//PX4_DEBUG( "Received topic data for: [%s] len[%d]\n", messageName, data_length );
						_RxHandler->process_received_message(messageName,
										     header->_DataLen, topic_data);

					} else if (header->_MsgType == _CONTROL_MSG_TYPE_ADVERTISE) {
						PX4_DEBUG("Received topic advertise message for: [%s] len[%d]\n", messageName, data_length);
						_RxHandler->process_remote_topic(messageName, true);

					} else if (header->_MsgType == _CONTROL_MSG_TYPE_UNADVERTISE) {
						PX4_DEBUG("Received topic unadvertise message for: [%s] len[%d]\n", messageName, data_length);
						_RxHandler->process_remote_topic(messageName, false);
					}
				}

				bytes_processed += header->_MsgNameLen + header->_DataLen + sizeof(struct BulkTransferHeader);
			}

		} else {
			PX4_DEBUG("Error: Getting data over fastRPC channel\n");
			break;
		}

		t3 = hrt_absolute_time();
		count++;

		if ((unsigned long)(t2 - t1) < rpc_min) {
			rpc_min = (unsigned long)(t2 - t1);
		}

		if ((unsigned long)(t2 - t1) > rpc_max) {
			rpc_max = (unsigned long)(t2 - t1);
		}

		if ((unsigned long)(t3 - t2) < orb_min) {
			orb_min = (unsigned long)(t3 - t2);
		}

		if ((unsigned long)(t3 - t2) > orb_max) {
			orb_max = (unsigned long)(t3 - t2);
		}

		rpc_avg = ((double)((rpc_avg * (count - 1)) + (unsigned long)(t2 - t1))) / (double)(count);
		orb_avg = ((double)((orb_avg * (count - 1)) + (unsigned long)(t3 - t2))) / (double)(count);

		if ((unsigned long)(t3 - check_time) >= 10000000) {
			//PX4_DEBUG("Krait RPC Stats     : rpc_min: %lu rpc_max: %lu rpc_avg: %f", rpc_min, rpc_max, rpc_avg);
			//PX4_DEBUG("Krait RPC(orb) Stats: orb_min: %lu orb_max: %lu orb_avg: %f", orb_min, orb_max, orb_avg);
			check_time = t3;
			rpc_max = orb_max = 0;
			rpc_min = orb_min = 0xFFFFFF;
			orb_avg = 0;
			rpc_avg = 0;
			count = 0;
		}

		//PX4_DEBUG("MsgName: %30s, t1: %lu, t2: %lu, t3: %lu, dt1: %lu, dt2: %lu",name, (unsigned long) t1, (unsigned long) t2, (unsigned long) t3,
		//  (unsigned long) (t2-t1), (unsigned long) (t3-t2));
	}

	PX4_DEBUG("[uORB::KraitFastRpcChannel::fastrpc_recv_thread] Exiting fastrpc_recv_thread\n");
}

void DumpData(uint8_t *buffer, int32_t length, int32_t num_topics)
{
	FILE *fp = fopen(_log_file_name.c_str(), "a+");

	if (fp == nullptr) {
		PX4_ERR("Error unable to open log file[%s]", _log_file_name.c_str());
		return;
	}

	fprintf(fp, "===== Data Len[%d] num_topics[%d]  ======\n", length, num_topics);

	for (int i = 0; i < length; i += 16) {
		int remaining_chars = length - i;
		remaining_chars = (remaining_chars >= 16) ? 16 : remaining_chars;

		fprintf(fp, "%p  - ", &(buffer[i]));

		for (int j = 0; j < remaining_chars; j++) {
			fprintf(fp, " %02X", buffer[i + j]);

			if (j == 7) {
				fprintf(fp, " -");
			}
		}

		fprintf(fp, "  ");

		for (int j = 0; j < remaining_chars; j++) {
			fprintf(fp, "%c", (char)buffer[i + j ]);
		}

		fprintf(fp, "\n");
	}

	fclose(fp);
}

