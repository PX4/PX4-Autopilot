/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "CanardHandle.hpp"

#include <net/if.h>
#include <sys/ioctl.h>
#include <string.h>

#include <px4_platform_common/log.h>

#include "o1heap/o1heap.h"

#include "Subscribers/BaseSubscriber.hpp"

#if defined(__PX4_NUTTX)
# if defined(CONFIG_NET_CAN)
#  include "CanardSocketCAN.hpp"
# elif defined(CONFIG_CAN)
#  include "CanardNuttXCDev.hpp"
# else
#  error "No CAN driver enabled for Cyphal build"
# endif // CONFIG_CAN
#endif // NuttX


O1HeapInstance *cyphal_allocator{nullptr};

static void *memAllocate(CanardInstance *const ins, const size_t amount) { return o1heapAllocate(cyphal_allocator, amount); }
static void memFree(CanardInstance *const ins, void *const pointer) { o1heapFree(cyphal_allocator, pointer); }


CanardHandle::CanardHandle(uint32_t node_id, const size_t capacity, const size_t mtu_bytes)
{
	_cyphal_heap = memalign(O1HEAP_ALIGNMENT, HeapSize);
	cyphal_allocator = o1heapInit(_cyphal_heap, HeapSize, nullptr, nullptr);

	if (cyphal_allocator == nullptr) {
		PX4_ERR("o1heapInit failed with size %u", HeapSize);
	}

	_canard_instance = canardInit(&memAllocate, &memFree);

	_canard_instance.node_id = node_id; // Defaults to anonymous; can be set up later at any point.

	_queue = canardTxInit(capacity, mtu_bytes);

#if defined(__PX4_NUTTX)
# if defined(CONFIG_NET_CAN)
	_can_interface = new CanardSocketCAN();
# elif defined(CONFIG_CAN)
	_can_interface = new CanardNuttXCDev();
# endif // CONFIG_CAN
#endif // NuttX

}

CanardHandle::~CanardHandle()
{
	_can_interface->close();
	delete _can_interface;
	_can_interface = nullptr;

	delete static_cast<uint8_t *>(_cyphal_heap);
	_cyphal_heap = nullptr;

}


bool CanardHandle::init(const char *can_iface_name)
{
	if (_can_interface) {
		if (_can_interface->init(can_iface_name) == PX4_OK) {
			return true;
		}

		//}
	}

	return false;
}

void CanardHandle::receive()
{
	/* Process received messages */

	uint8_t data[64] {};
	CanardRxFrame received_frame{};
	received_frame.frame.payload = &data;

	while (_can_interface->receive(&received_frame) > 0) {
		CanardRxTransfer receive{};
		CanardRxSubscription *subscription = nullptr;
		int32_t result = canardRxAccept(&_canard_instance, received_frame.timestamp_usec, &received_frame.frame, 0, &receive,
						&subscription);

		if (result < 0) {
			// An error has occurred: either an argument is invalid or we've ran out of memory.
			// It is possible to statically prove that an out-of-memory will never occur for a given application if
			// the heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
			// Reception of an invalid frame is NOT an error.
			PX4_ERR("Receive error %" PRId32" \n", result);

		} else if (result == 1) {
			// A transfer has been received, process it.
			// PX4_INFO("received Port ID: %d", receive.metadata.port_id);

			if (subscription != nullptr) {
				UavcanBaseSubscriber *sub_instance = (UavcanBaseSubscriber *)subscription->user_reference;
				sub_instance->callback(receive);

			} else {
				PX4_ERR("No matching sub for %d", receive.metadata.port_id);
			}

			// Deallocate the dynamic memory afterwards.
			_canard_instance.memory_free(&_canard_instance, (void *)receive.payload);

		} else {
			// PX4_INFO("RX canard %li\n", result);
		}
	}

}

void CanardHandle::transmit()
{
	// Look at the top of the TX queue.
	for (const CanardTxQueueItem *ti = NULL; (ti = canardTxPeek(&_queue)) != NULL;) { // Peek at the top of the queue.
		if ((0U == ti->tx_deadline_usec) || (ti->tx_deadline_usec > hrt_absolute_time())) { // Check the deadline.
			// Send the frame. Redundant interfaces may be used here.
			const int tx_res = _can_interface->transmit(*ti);

			if (tx_res < 0) {
				PX4_ERR("Transmit error %d, frame dropped, errno '%s'", tx_res, strerror(errno));

			} else if (tx_res == 0) {
				// Timeout - just exit and try again later
				break;
			}

		}

		// After the frame is transmitted or if it has timed out while waiting, pop it from the queue and deallocate:
		_canard_instance.memory_free(&_canard_instance, canardTxPop(&_queue, ti));
	}
}

int32_t CanardHandle::TxPush(const CanardMicrosecond             tx_deadline_usec,
			     const CanardTransferMetadata *const metadata,
			     const size_t                        payload_size,
			     const void *const                   payload)
{
	return canardTxPush(&_queue, &_canard_instance, tx_deadline_usec, metadata, payload_size, payload);
}

int8_t CanardHandle::RxSubscribe(const CanardTransferKind    transfer_kind,
				 const CanardPortID          port_id,
				 const size_t                extent,
				 const CanardMicrosecond     transfer_id_timeout_usec,
				 CanardRxSubscription *const out_subscription)
{
	return canardRxSubscribe(&_canard_instance, transfer_kind, port_id, extent, transfer_id_timeout_usec, out_subscription);
}

int8_t CanardHandle::RxUnsubscribe(const CanardTransferKind transfer_kind,
				   const CanardPortID       port_id)
{
	return canardRxUnsubscribe(&_canard_instance, transfer_kind, port_id);
}

CanardTreeNode *CanardHandle::getRxSubscriptions(CanardTransferKind kind)
{
	return _canard_instance.rx_subscriptions[kind];
}

O1HeapDiagnostics CanardHandle::getO1HeapDiagnostics()
{
	return o1heapGetDiagnostics(cyphal_allocator);
}

int32_t CanardHandle::mtu()
{
	return _queue.mtu_bytes;
}

CanardNodeID CanardHandle::node_id()
{
	return _canard_instance.node_id;
}

void CanardHandle::set_node_id(CanardNodeID id)
{
	_canard_instance.node_id = id;
}
