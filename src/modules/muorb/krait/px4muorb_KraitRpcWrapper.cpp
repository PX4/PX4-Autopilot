/****************************************************************************
 *
 * Copyright (C) 2016 Ramakrishna Kintada. All rights reserved.
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

#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include "px4muorb_KraitRpcWrapper.hpp"
#include <rpcmem.h>
#include "px4muorb.h"
#include "px4_log.h"

using namespace px4muorb;

/* Flags applied to the allocation of the shared memory for RPC */
#define MUORB_KRAIT_FASTRPC_MEM_FLAGS         0

/* The ID of the HEAP to be used when allocating shared memory */
//TODO This heap id is used for test purposes. We need to find out the correct one.
#define MUORB_KRAIT_FASTRPC_HEAP_ID           22

static char *_TopicNameBuffer = 0;
static const int32_t _MAX_TOPIC_NAME_BUFFER = 256;

static uint8_t *_DataBuffer  = 0;
static const uint32_t _MAX_DATA_BUFFER_SIZE = 2048;

static bool _Initialized = false;

// These numbers are based off the fact each fastrpc call for 64K packet is 94us.
// hence we are trying to allocation 64K of byte buffers.
static const uint32_t _MAX_TOPIC_DATA_BUFFER_SIZE = 1024;
static const uint32_t _MAX_TOPICS = 64;
static const uint32_t _MAX_BULK_TRANSFER_BUFFER_SIZE = _MAX_TOPIC_DATA_BUFFER_SIZE * _MAX_TOPICS;
static uint8_t *_BulkTransferBuffer = 0;

// The DSP timer can be read from this file.
#define DSP_TIMER_FILE "/sys/kernel/boot_adsp/qdsp_qtimer"

/**
 * Helper function to get timer difference between time on DSP and appsproc side.
 * Usually the DSP gets started around 2s before the appsproc (Linux) side and
 * therefore the clocks are not in sync. We change the clock on the DSP side but
 * for this we need to find the offset first and then tell code on the DSP side.
 *
 * @param time_diff_us: pointer to time offset to set.
 * @return: 0 on success, < 0 on error.
 */
int calc_timer_diff_to_dsp_us(int32_t *time_diff_us);

int calc_timer_diff_to_dsp_us(int32_t *time_diff_us)
{
	int fd = open(DSP_TIMER_FILE, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("Could not open DSP timer file %s.", DSP_TIMER_FILE);
		return -1;
	}

	char buffer[21];
	memset(buffer, 0, sizeof(buffer));
	int bytes_read = read(fd, buffer, sizeof(buffer));

	if (bytes_read < 0) {
		PX4_ERR("Could not read DSP timer file %s.", DSP_TIMER_FILE);
		close(fd);
		return -2;
	}

	// Do this call right after reading to avoid latency here.
	timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	uint64_t time_appsproc = ((uint64_t)ts.tv_sec) * 1000000llu + (ts.tv_nsec / 1000);

	close(fd);

	uint64_t time_dsp;
	int ret  = sscanf(buffer, "%llx", &time_dsp);

	if (ret < 0) {
		PX4_ERR("Could not parse DSP timer.");
		return -3;
	}

	// The clock count needs to get converted to us.
	// The magic value of 19.2 was provided by Qualcomm.
	time_dsp /= 19.2;

	// Before casting to in32_t, check if it fits.
	uint64_t abs_diff = (time_appsproc > time_dsp)
			    ? (time_appsproc - time_dsp) : (time_dsp - time_appsproc);

	if (abs_diff > INT32_MAX) {
		PX4_ERR("Timer difference too big");
		return -4;
	}

	*time_diff_us = time_appsproc - time_dsp;

	PX4_DEBUG("found time_dsp: %llu us, time_appsproc: %llu us", time_dsp, time_appsproc);
	PX4_DEBUG("found time_diff: %li us, %.6f s", *time_diff_us, ((double)*time_diff_us) / 1e6);

	return 0;
}


px4muorb::KraitRpcWrapper::KraitRpcWrapper()
{
}

px4muorb::KraitRpcWrapper::~KraitRpcWrapper()
{
}

bool px4muorb::KraitRpcWrapper::Initialize()
{
	bool rc = true;

	PX4_DEBUG("%s Now calling rpcmem_init...", __FUNCTION__);
	rpcmem_init();

	PX4_DEBUG("%s Now calling rpcmem_alloc...", __FUNCTION__);

	_BulkTransferBuffer = (uint8_t *) rpcmem_alloc(MUORB_KRAIT_FASTRPC_HEAP_ID, MUORB_KRAIT_FASTRPC_MEM_FLAGS,
			      _MAX_BULK_TRANSFER_BUFFER_SIZE * sizeof(uint8_t));
	rc = (_BulkTransferBuffer != NULL) ? true : false;

	if (!rc) {
		PX4_ERR("%s rpcmem_alloc failed! for bulk transfer buffers", __FUNCTION__);
		return rc;

	} else {
		PX4_DEBUG("%s rpcmem_alloc passed for Bulk transfer buffers buffer_size: %d addr: %p",
			  __FUNCTION__, (_MAX_BULK_TRANSFER_BUFFER_SIZE * sizeof(uint8_t)), _BulkTransferBuffer);
	}

	_TopicNameBuffer = (char *) rpcmem_alloc(MUORB_KRAIT_FASTRPC_HEAP_ID,
			   MUORB_KRAIT_FASTRPC_MEM_FLAGS, _MAX_TOPIC_NAME_BUFFER * sizeof(char));

	rc = (_TopicNameBuffer != NULL) ? true : false;

	if (!rc) {
		PX4_ERR("%s rpcmem_alloc failed! for topic_name_buffer", __FUNCTION__);
		rpcmem_free(_BulkTransferBuffer);
		return rc;

	} else {
		PX4_DEBUG("%s rpcmem_alloc passed for topic_name_buffer", __FUNCTION__);
	}

	// now allocate the data buffer.
	_DataBuffer = (uint8_t *) rpcmem_alloc(MUORB_KRAIT_FASTRPC_HEAP_ID,
					       MUORB_KRAIT_FASTRPC_MEM_FLAGS, _MAX_DATA_BUFFER_SIZE * sizeof(uint8_t));

	rc = (_DataBuffer != NULL) ? true : false;

	if (!rc) {
		PX4_ERR("%s rpcmem_alloc failed! for DataBuffer", __FUNCTION__);
		// free the topic name buffer;
		rpcmem_free(_BulkTransferBuffer);
		rpcmem_free(_TopicNameBuffer);
		_TopicNameBuffer = 0;
		return rc;

	} else {
		PX4_DEBUG("%s rpcmem_alloc passed for data_buffer", __FUNCTION__);
	}

	int32_t time_diff_us;

	if (calc_timer_diff_to_dsp_us(&time_diff_us) != 0) {
		rc = false;
		return rc;
	}

	// call muorb initialize routine.
	if (px4muorb_orb_initialize() != 0) {
		PX4_ERR("%s Error calling the uorb fastrpc initalize method..", __FUNCTION__);
		rc = false;
		return rc;
	}

	// TODO FIXME: remove this check or make it less verbose later
	px4muorb_set_absolute_time_offset(time_diff_us);

	uint64_t time_dsp;
	px4muorb_get_absolute_time(&time_dsp);

	uint64_t time_appsproc = hrt_absolute_time();

	int diff = (time_dsp - time_appsproc);

	PX4_DEBUG("time_dsp: %llu us, time appsproc: %llu us, diff: %d us", time_dsp, time_appsproc, diff);

	_Initialized = true;
	return rc;
}

bool px4muorb::KraitRpcWrapper::Terminate()
{
	if (_BulkTransferBuffer != NULL) {
		rpcmem_free(_BulkTransferBuffer);
		_BulkTransferBuffer = 0;
	}

	if (_TopicNameBuffer != NULL) {
		rpcmem_free(_TopicNameBuffer);
		_TopicNameBuffer = 0;
	}

	if (_DataBuffer != NULL) {
		rpcmem_free(_DataBuffer);
		_DataBuffer = 0;
	}

	_Initialized = false;
	return true;
}

int32_t px4muorb::KraitRpcWrapper::AddSubscriber(const char *topic)
{
	return ((_Initialized) ? px4muorb_add_subscriber(topic) : -1);
}

int32_t px4muorb::KraitRpcWrapper::RemoveSubscriber(const char *topic)
{
	return (_Initialized ? px4muorb_remove_subscriber(topic) : -1);
}

int32_t px4muorb::KraitRpcWrapper::IsSubscriberPresent(const char *topic, int32_t *status)
{
	return (_Initialized ? px4muorb_is_subscriber_present(topic, status) : -1);
}

int32_t px4muorb::KraitRpcWrapper::SendData(const char *topic, int32_t length_in_bytes, const uint8_t *data)
{
	return (_Initialized ? px4muorb_send_topic_data(topic, data, length_in_bytes) : -1);
}

int32_t px4muorb::KraitRpcWrapper::ReceiveData(int32_t *msg_type, char **topic, int32_t *length_in_bytes,
		uint8_t **data)
{
	int32_t rc = -1;

	if (_Initialized) {
		rc = px4muorb_receive_msg(msg_type, _TopicNameBuffer, _MAX_TOPIC_NAME_BUFFER, _DataBuffer, _MAX_DATA_BUFFER_SIZE,
					  length_in_bytes);

		if (rc == 0) {
			*topic = _TopicNameBuffer;
			*data  = _DataBuffer;

		} else {
			PX4_ERR("ERROR: Getting data from fastRPC link");
		}

	} else {
		PX4_ERR("ERROR: FastRpcWrapper Not Initialized");
	}

	return rc;
}

int32_t px4muorb::KraitRpcWrapper::ReceiveBulkData(uint8_t **bulk_data, int32_t *length_in_bytes, int32_t *topic_count)
{
	int32_t rc = -1;

	if (_Initialized) {
		//rc = px4muorb_receive_msg( msg_type, _TopicNameBuffer, _MAX_TOPIC_NAME_BUFFER, _DataBuffer, _MAX_DATA_BUFFER_SIZE, length_in_bytes  );
		rc = px4muorb_receive_bulk_data(_BulkTransferBuffer, _MAX_BULK_TRANSFER_BUFFER_SIZE,  length_in_bytes, topic_count);

		if (rc == 0) {
			*bulk_data = _BulkTransferBuffer;

		} else {
			PX4_ERR("ERROR: Getting Bulk data from fastRPC link");
		}

	} else {
		PX4_ERR("ERROR: FastRpcWrapper Not Initialized");
	}

	return rc;
}

int32_t px4muorb::KraitRpcWrapper::UnblockReceiveData()
{
	return (_Initialized ? px4muorb_unblock_recieve_msg() : -1);
}
