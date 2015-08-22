/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *               Pavel Kirienko <pavel.kirienko@zubax.com>
 *               David Sidrane <david_s5@nscdg.com>
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

#include <nuttx/config.h>

#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <fcntl.h>
#include <pthread.h>
#include <semaphore.h>
#include <debug.h>

#include <uavcan_stm32/clock.hpp>
#include <uavcan/node/sub_node.hpp>
#include <uavcan/protocol/node_status_monitor.hpp>

/*
 * General purpose wrapper around os's mutual exclusion
 * mechanism.
 *
 * It supports the
 *
 * Note the naming of thier_mutex_ implies that the underlying
 * mutex is owned by class using the Lock.
 * and this wrapper provides service to initialize and de initialize
 * the mutex.
 */
class Lock
{
	pthread_mutex_t &thier_mutex_;

public:

	Lock(pthread_mutex_t &m) :
		thier_mutex_(m)
	{
		(void)pthread_mutex_lock(&m);
	}

	~Lock()
	{
		(void)pthread_mutex_unlock(&thier_mutex_);
	}

	static int init(pthread_mutex_t &thier_mutex_)
	{
		return pthread_mutex_init(&thier_mutex_, NULL);
	}

	static int deinit(pthread_mutex_t &thier_mutex_)
	{
		return pthread_mutex_destroy(&thier_mutex_);
	}

};

/**
 * Generic queue based on the linked list class defined in libuavcan.
 * This class does not use heap memory.
 */
template <typename T>
class Queue
{
	struct Item : public uavcan::LinkedListNode<Item> {
		T payload;

		template <typename... Args>
		Item(Args... args) : payload(args...) { }
	};

	uavcan::LimitedPoolAllocator allocator_;
	uavcan::LinkedListRoot<Item> list_;

public:
	Queue(uavcan::IPoolAllocator &arg_allocator, std::size_t block_allocation_quota) :
		allocator_(arg_allocator, block_allocation_quota)
	{
		uavcan::IsDynamicallyAllocatable<Item>::check();
	}

	bool isEmpty() const { return list_.isEmpty(); }

	/**
	 * Creates one item in-place at the end of the list.
	 * Returns true if the item was appended successfully, false if there's not enough memory.
	 * Complexity is O(N) where N is queue length.
	 */
	template <typename... Args>
	bool tryEmplace(Args... args)
	{
		// Allocating memory
		void *const ptr = allocator_.allocate(sizeof(Item));

		if (ptr == nullptr) {
			return false;
		}

		// Constructing the new item
		Item *const item = new(ptr) Item(args...);
		assert(item != nullptr);

		// Inserting the new item at the end of the list
		Item *p = list_.get();

		if (p == nullptr) {
			list_.insert(item);

		} else {
			while (p->getNextListNode() != nullptr) {
				p = p->getNextListNode();
			}

			assert(p->getNextListNode() == nullptr);
			p->setNextListNode(item);
			assert(p->getNextListNode()->getNextListNode() == nullptr);
		}

		return true;
	}

	/**
	 * Accesses the first element.
	 * Nullptr will be returned if the queue is empty.
	 * Complexity is O(1).
	 */
	T       *peek()       { return isEmpty() ? nullptr : &list_.get()->payload; }
	const T *peek() const { return isEmpty() ? nullptr : &list_.get()->payload; }

	/**
	 * Removes the first element.
	 * If the queue is empty, nothing will be done and assertion failure will be triggered.
	 * Complexity is O(1).
	 */
	void pop()
	{
		Item *const item = list_.get();
		assert(item != nullptr);

		if (item != nullptr) {
			list_.remove(item);
			item->~Item();
			allocator_.deallocate(item);
		}
	}
};

/**
 * Objects of this class are owned by the sub-node thread.
 * This class does not use heap memory.
 */
class VirtualCanIface : public uavcan::ICanIface,
	uavcan::Noncopyable
{
	/**
	 * This class re-defines uavcan::RxCanFrame with flags.
	 * Simple inheritance or composition won't work here, because the 40 byte limit will be exceeded,
	 * rendering this class unusable with Queue<>.
	 */
	struct RxItem: public uavcan::CanFrame {
		const uavcan::MonotonicTime ts_mono;
		const uavcan::UtcTime ts_utc;
		const uavcan::CanIOFlags flags;
		const uint8_t iface_index;

		RxItem(const uavcan::CanRxFrame &arg_frame, uavcan::CanIOFlags arg_flags) :
			uavcan::CanFrame(arg_frame),
			ts_mono(arg_frame.ts_mono),
			ts_utc(arg_frame.ts_utc),
			flags(arg_flags),
			iface_index(arg_frame.iface_index)
		{
			// Making sure it will fit into a pool block with a pointer prefix
			static_assert(sizeof(RxItem) <= (uavcan::MemPoolBlockSize - 8), "Bad coder, no coffee");
		}
	};

	pthread_mutex_t &common_driver_mutex_;

	uavcan::CanTxQueue prioritized_tx_queue_;
	Queue<RxItem> rx_queue_;

	int16_t send(const uavcan::CanFrame &frame, uavcan::MonotonicTime tx_deadline, uavcan::CanIOFlags flags) override
	{
		Lock lock(common_driver_mutex_);
		prioritized_tx_queue_.push(frame, tx_deadline, uavcan::CanTxQueue::Volatile, flags);
		return 1;
	}

	int16_t receive(uavcan::CanFrame &out_frame, uavcan::MonotonicTime &out_ts_monotonic,
			uavcan::UtcTime &out_ts_utc, uavcan::CanIOFlags &out_flags) override
	{
		Lock lock(common_driver_mutex_);

		if (rx_queue_.isEmpty()) {
			return 0;
		}

		const auto item = *rx_queue_.peek();
		rx_queue_.pop();

		out_frame = item;
		out_ts_monotonic = item.ts_mono;
		out_ts_utc = item.ts_utc;
		out_flags = item.flags;

		return 1;
	}

	int16_t configureFilters(const uavcan::CanFilterConfig *, std::uint16_t) override { return -uavcan::ErrDriver; }
	uint16_t getNumFilters() const override { return 0; }
	uint64_t getErrorCount() const override { return 0; }

public:
	VirtualCanIface(uavcan::IPoolAllocator &allocator, uavcan::ISystemClock &clock,
			pthread_mutex_t &arg_mutex, unsigned quota_per_queue) :
		common_driver_mutex_(arg_mutex),
		prioritized_tx_queue_(allocator, clock, quota_per_queue),
		rx_queue_(allocator, quota_per_queue)
	{
	}

	~VirtualCanIface()
	{
	}

	/**
	 * Note that RX queue overwrites oldest items when overflowed.
	 * Call this from the main thread only.
	 * No additional locking is required.
	 */
	void addRxFrame(const uavcan::CanRxFrame &frame, uavcan::CanIOFlags flags)
	{
		Lock lock(common_driver_mutex_);

		if (!rx_queue_.tryEmplace(frame, flags) && !rx_queue_.isEmpty()) {
			rx_queue_.pop();
			(void)rx_queue_.tryEmplace(frame, flags);
		}
	}

	/**
	 * Call this from the main thread only.
	 * No additional locking is required.
	 */
	void flushTxQueueTo(uavcan::INode &main_node, std::uint8_t iface_index)
	{
		Lock lock(common_driver_mutex_);
		const std::uint8_t iface_mask = static_cast<std::uint8_t>(1U << iface_index);

		while (auto e = prioritized_tx_queue_.peek()) {
			UAVCAN_TRACE("VirtualCanIface", "TX injection [iface=0x%02x]: %s",
				     unsigned(iface_mask), e->toString().c_str());

			const int res = main_node.injectTxFrame(e->frame, e->deadline, iface_mask,
								uavcan::CanTxQueue::Qos(e->qos), e->flags);

			prioritized_tx_queue_.remove(e);

			if (res <= 0) {
				break;
			}

		}
	}

	/**
	 * Call this from the sub-node thread only.
	 * No additional locking is required.
	 */
	bool hasDataInRxQueue()
	{
		Lock lock(common_driver_mutex_);
		return !rx_queue_.isEmpty();
	}
};

/**
 * This interface defines one method that will be called by the main node thread periodically in order to
 * transfer contents of TX queue of the sub-node into the TX queue of the main node.
 */
class ITxQueueInjector
{
public:
	virtual ~ITxQueueInjector() { }

	/**
	 * Flush contents of TX queues into the main node.
	 * @param main_node         Reference to the main node.
	 */
	virtual void injectTxFramesInto(uavcan::INode &main_node) = 0;
};

/**
 * Objects of this class are owned by the sub-node thread.
 * This class does not use heap memory.
 * @tparam SharedMemoryPoolSize         Amount of memory, in bytes, that will be statically allocated for the
 *                                      memory pool that will be shared across all interfaces for RX/TX queues.
 *                                      Typically this value should be no less than 4K per interface.
 */
template <unsigned SharedMemoryPoolSize>
class VirtualCanDriver : public uavcan::ICanDriver,
	public uavcan::IRxFrameListener,
	public ITxQueueInjector,
	uavcan::Noncopyable
{
	class Event
	{
		FAR sem_t sem;


	public:

		int init()
		{
			return sem_init(&sem, 0, 0);
		}

		int deinit()
		{
			return sem_destroy(&sem);
		}


		Event()
		{
		}

		~Event()
		{
		}


		/**
		 */

		void waitFor(uavcan::MonotonicDuration duration)
		{
			static const unsigned NsPerSec = 1000000000;

			if (duration.isPositive()) {
				auto abstime = ::timespec();

				if (clock_gettime(CLOCK_REALTIME, &abstime) >= 0) {
					abstime.tv_nsec += duration.toUSec() * 1000;

					if (abstime.tv_nsec >= NsPerSec) {
						abstime.tv_sec++;
						abstime.tv_nsec -= NsPerSec;
					}

					(void)sem_timedwait(&sem, &abstime);
				}
			}
		}

		void signal()
		{
			int count;
			int rv = sem_getvalue(&sem, &count);

			if (rv > 0 && count <= 0) {
				sem_post(&sem);
			}
		}
	};

	Event event_;               ///< Used to unblock the select() call when IO happens.
	pthread_mutex_t driver_mutex_;    ///< Shared across all ifaces
	uavcan::PoolAllocator<SharedMemoryPoolSize, uavcan::MemPoolBlockSize> allocator_;   ///< Shared across all ifaces
	uavcan::LazyConstructor<VirtualCanIface> ifaces_[uavcan::MaxCanIfaces];
	const unsigned num_ifaces_;
	uavcan::ISystemClock &clock_;

	uavcan::ICanIface *getIface(uint8_t iface_index) override
	{
		return (iface_index < num_ifaces_) ? ifaces_[iface_index].operator VirtualCanIface * () : nullptr;
	}

	uint8_t getNumIfaces() const override { return num_ifaces_; }

	/**
	 * This and other methods of ICanDriver will be invoked by the sub-node thread.
	 */
	int16_t select(uavcan::CanSelectMasks &inout_masks,
		       const uavcan::CanFrame * (&)[uavcan::MaxCanIfaces],
		       uavcan::MonotonicTime blocking_deadline) override
	{
		bool need_block = (inout_masks.write == 0);    // Write queue is infinite

		for (unsigned i = 0; need_block && (i < num_ifaces_); i++) {
			const bool need_read = inout_masks.read & (1U << i);

			if (need_read && ifaces_[i]->hasDataInRxQueue()) {
				need_block = false;
			}
		}

		if (need_block) {
			event_.waitFor(blocking_deadline - clock_.getMonotonic());
		}

		inout_masks = uavcan::CanSelectMasks();

		for (unsigned i = 0; i < num_ifaces_; i++) {
			const std::uint8_t iface_mask = 1U << i;
			inout_masks.write |= iface_mask;           // Always ready to write

			if (ifaces_[i]->hasDataInRxQueue()) {
				inout_masks.read |= iface_mask;
			}
		}

		return num_ifaces_;       // We're always ready to write, hence > 0.
	}

	/**
	 * This handler will be invoked by the main node thread.
	 */
	void handleRxFrame(const uavcan::CanRxFrame &frame, uavcan::CanIOFlags flags) override
	{
		UAVCAN_TRACE("VirtualCanDriver", "RX [flags=%u]: %s", unsigned(flags), frame.toString().c_str());

		if (frame.iface_index < num_ifaces_) {
			ifaces_[frame.iface_index]->addRxFrame(frame, flags);
			event_.signal();

		}
	}

	/**
	 * This method will be invoked by the main node thread.
	 */
	void injectTxFramesInto(uavcan::INode &main_node) override
	{
		for (unsigned i = 0; i < num_ifaces_; i++) {
			ifaces_[i]->flushTxQueueTo(main_node, i);
		}

		event_.signal();
	}

public:
	VirtualCanDriver(unsigned arg_num_ifaces, uavcan::ISystemClock &system_clock) :
		num_ifaces_(arg_num_ifaces),
		clock_(system_clock)
	{
		Lock::init(driver_mutex_);
		event_.init();

		assert(num_ifaces_ > 0 && num_ifaces_ <= uavcan::MaxCanIfaces);

		const unsigned quota_per_iface = allocator_.getNumBlocks() / num_ifaces_;
		const unsigned quota_per_queue = quota_per_iface;             // 2x overcommit

		UAVCAN_TRACE("VirtualCanDriver", "Total blocks: %u, quota per queue: %u",
			     unsigned(allocator_.getNumBlocks()), unsigned(quota_per_queue));

		for (unsigned i = 0; i < num_ifaces_; i++) {
			ifaces_[i].template construct<uavcan::IPoolAllocator &, uavcan::ISystemClock &,
				pthread_mutex_t &, unsigned>(allocator_, clock_, driver_mutex_, quota_per_queue);
		}
	}

	~VirtualCanDriver()
	{
		Lock::deinit(driver_mutex_);
		event_.deinit();
	}

};
