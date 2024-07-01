/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#pragma once

#include <canard.h>
#include "o1heap/o1heap.h"
#include "CanardInterface.hpp"

class CanardHandle
{
	/*
	* This memory is allocated for the 01Heap allocator used by
	* libcanard to store incoming/outcoming data
	* Current size of 8192 bytes is arbitrary, should be optimized further
	* when more nodes and messages are on the CAN bus
	*/
	static constexpr unsigned HeapSize = 8192;

public:
	CanardHandle(uint32_t node_id, const size_t capacity, const size_t mtu_bytes);
	~CanardHandle();

	bool init(const char *can_iface_name);

	void receive();
	void transmit();

	int32_t TxPush(const CanardMicrosecond             tx_deadline_usec,
		       const CanardTransferMetadata *const metadata,
		       const size_t                        payload_size,
		       const void *const                   payload);

	int8_t RxSubscribe(const CanardTransferKind    transfer_kind,
			   const CanardPortID          port_id,
			   const size_t                extent,
			   const CanardMicrosecond     transfer_id_timeout_usec,
			   CanardRxSubscription *const out_subscription);
	int8_t RxUnsubscribe(const CanardTransferKind transfer_kind,
			     const CanardPortID       port_id);
	CanardTreeNode *getRxSubscriptions(CanardTransferKind kind);
	O1HeapDiagnostics getO1HeapDiagnostics();

	int32_t mtu();
	CanardNodeID node_id();
	void set_node_id(CanardNodeID id);

private:
	CanardInterface *_can_interface;

	CanardInstance _canard_instance;

	CanardTxQueue _queue;

	void *_cyphal_heap{nullptr};

};
