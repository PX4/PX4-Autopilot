/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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

#include <sys/shm.h>
#include <sys/mman.h>

#include "uORBDeviceNode.hpp"

#include "uORBUtils.hpp"
#include "uORBManager.hpp"

#include "SubscriptionCallback.hpp"

#ifdef CONFIG_ORB_COMMUNICATOR
#include "uORBCommunicator.hpp"
#endif /* CONFIG_ORB_COMMUNICATOR */

#if defined(__PX4_NUTTX)
#include <nuttx/arch.h>
#include <nuttx/mm/mm.h>
#include <px4_platform/micro_hal.h>
#endif

#include <px4_platform_common/sem.hpp>
#include <drivers/drv_hrt.h>

// This is a speed optimization for nuttx flat build
#ifdef CONFIG_BUILD_FLAT
#define ATOMIC_ENTER irqstate_t flags = px4_enter_critical_section()
#define ATOMIC_LEAVE px4_leave_critical_section(flags)
#else
#define ATOMIC_ENTER lock()
#define ATOMIC_LEAVE unlock()
#endif

// Every subscriber thread has it's own list of cached subscriptions
uORB::DeviceNode::MappingCache::MappingCacheListItem *uORB::DeviceNode::MappingCache::g_cache =
	nullptr;

// This lock protects the subscription cache list from concurrent accesses by the threads in the same process
px4_sem_t uORB::DeviceNode::MappingCache::g_cache_lock;

orb_advert_t uORB::DeviceNode::MappingCache::get(ORB_ID orb_id, uint8_t instance)
{
	lock();

	MappingCacheListItem *item = g_cache;

	while (item &&
	       (orb_id != node(item->handle)->id() ||
		instance != node(item->handle)->get_instance())) {
		item = item->next;
	}

	unlock();

	return item != nullptr ? item->handle : ORB_ADVERT_INVALID;
}

orb_advert_t uORB::DeviceNode::MappingCache::map_node(ORB_ID orb_id, uint8_t instance, int shm_fd)
{

	// Check if it is already mapped
	orb_advert_t handle = get(orb_id, instance);

	if (orb_advert_valid(handle)) {
		return handle;
	}

	lock();

	// Not mapped yet, map it
	void *ptr = mmap(0, sizeof(uORB::DeviceNode), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);

	if (ptr != MAP_FAILED) {
		// In NuttX flat and protected builds we can just drop the mappings
		// to save some kernel memory. There is no MMU, and the memory is
		// there until the shm object is unlinked
#if defined(CONFIG_BUILD_FLAT)
		munmap(ptr, sizeof(uORB::DeviceNode));
#endif

		// Create a list item and add to the beginning of the list
		handle.node = ptr;
		MappingCacheListItem *item = new MappingCacheListItem{g_cache, handle};

		if (item) {
			g_cache = item;
		}
	}

	unlock();

	return handle;
}

#if !defined(CONFIG_BUILD_FLAT)
orb_advert_t uORB::DeviceNode::MappingCache::map_data(orb_advert_t handle, int shm_fd, size_t size, bool publisher)
{
	lock();

	MappingCacheListItem *item = g_cache;

	while (item &&
	       handle.node != item->handle.node) {
		item = item->next;
	}

	if (item != nullptr) {

		if (item->handle.data != nullptr && item->handle.data_size == size) {
			// Mapped already, return the mapping
			handle = item->handle;

		} else {
			// Drop any old mapping if exists
			if (handle.data != nullptr) {
				munmap(handle.data, handle.data_size);
			}

			// Map the data with new size
			if (shm_fd >= 0 && size > 0) {
				handle.data = mmap(0, size, publisher ? PROT_WRITE : PROT_READ, MAP_SHARED, shm_fd, 0);

				if (handle.data == MAP_FAILED) {
					handle.data = nullptr;
					handle.data_size = 0;
					PX4_ERR("MMAP fail\n");

				} else {
					handle.data_size = size;
				}

			} else {
				handle.data = nullptr;
				handle.data_size = 0;
			}

			item->handle = handle;
		}
	}

	unlock();

	return handle;
}
#endif

bool uORB::DeviceNode::MappingCache::del(const orb_advert_t &handle)
{
	MappingCacheListItem *prev = nullptr;

	lock();

	MappingCacheListItem *item = g_cache;

	while (item &&
	       handle.node != item->handle.node) {
		prev = item;
		item = item->next;
	}

	if (item != nullptr) {
		if (prev == nullptr) {
			// Remove the first item
			g_cache = item->next;

		} else {
			prev->next = item->next;
		}

		munmap(handle.node, sizeof(DeviceNode));

#ifndef CONFIG_BUILD_FLAT

		if (handle.data) {
			munmap(handle.data, handle.data_size);
		}

#endif

		delete (item);
	}

	unlock();

	return item != nullptr ? true : false;
}

// round up to nearest power of two
// Such as 0 => 1, 1 => 1, 2 => 2 ,3 => 4, 10 => 16, 60 => 64, 65...255 => 128
// Note: When the input value > 128, the output is always 128
static inline uint8_t round_pow_of_two_8(uint8_t n)
{
	if (n == 0) {
		return 1;
	}

	// Avoid is already a power of 2
	uint8_t value = n - 1;

	// Fill 1
	value |= value >> 1U;
	value |= value >> 2U;
	value |= value >> 4U;

	// Unable to round-up, take the value of round-down
	if (value == UINT8_MAX) {
		value >>= 1U;
	}

	return value + 1;
}

orb_advert_t uORB::DeviceNode::nodeOpen(const ORB_ID id, const uint8_t instance, bool create)
{
	/*
	 * Generate the path to the node and try to open it.
	 */

	orb_advert_t handle = MappingCache::get(id, instance);

	if (orb_advert_valid(handle)) {
		return handle;
	}

	char nodepath[orb_maxpath];
	int inst = instance;
	int ret = uORB::Utils::node_mkpath(nodepath, get_orb_meta(id), &inst);
	bool created = false;

	if (ret != OK) {
		return handle;
	}

	// First, try to create the node. This will fail if it already exists

	int shm_fd = -1;

	if (create) {
		shm_fd = shm_open(nodepath, O_CREAT | O_RDWR | O_EXCL, 0666);

		if (shm_fd >= 0) {

			// If the creation succeeded, set the size of the shm region
			if (ftruncate(shm_fd, sizeof(uORB::DeviceNode)) != 0) {
				::close(shm_fd);
				shm_fd = -1;
				PX4_ERR("truncate fail!\n");

			} else {
				created = true;
			}
		}
	}

	if (shm_fd < 0) {
		// Now try to open an existing one

		shm_fd = shm_open(nodepath, O_RDWR, 0666);
	}

	if (shm_fd < 0) {
		// We were not able to create a new node or open an existing one
		return handle;
	}

	handle = MappingCache::map_node(id, instance, shm_fd);

	// No need to keep the fd any more, close it

	::close(shm_fd);

	if (orb_advert_valid(handle) && created) {
		// construct the new node in the region
		new (node(handle)) uORB::DeviceNode(id, instance, nodepath);
	}

	return handle;
}

int uORB::DeviceNode::nodeClose(orb_advert_t &handle)
{
	if (!orb_advert_valid(handle)) {
		return PX4_ERROR;
	}

	if (node(handle)->_publisher_count == 0) {
		node(handle)->_queue_size = 0;
		node(handle)->_data_valid = false;

		// Delete the data
#ifdef CONFIG_BUILD_FLAT
		free(node(handle)->_data);
		node(handle)->_data = nullptr;
#else
		shm_unlink(node(handle)->get_devname() + 1);
		MappingCache::map_data(handle, -1, 0, false);
#endif

		// If there are no more subscribers, delete the node and its mapping
		if (node(handle)->_subscriber_count == 0) {

			// Close the Node object
			shm_unlink(node(handle)->get_devname());

			// Uninitialize the node
			delete (node(handle));

			// Delete the mappings for this process
			MappingCache::del(handle);
		}
	}

	handle = ORB_ADVERT_INVALID;

	return PX4_OK;
}

orb_advert_t uORB::DeviceNode::orb_advertise(const ORB_ID id, int instance, unsigned queue_size,
		bool publisher)
{
	/* Open the node, if it exists or create a new one */

	orb_advert_t handle;
	handle = nodeOpen(id, instance, true);

	if (orb_advert_valid(handle)) {
		node(handle)->advertise(publisher, queue_size);
	}

	return handle;
}

int uORB::DeviceNode::advertise(bool publisher, uint8_t queue_size)
{
	int ret = -1;

	ret = ++_advertiser_count;

	if (publisher) {
		ret = ++_publisher_count;
	}

	update_queue_size(queue_size);

	return ret;
}

int uORB::DeviceNode::orb_unadvertise(orb_advert_t &handle, bool publisher)
{
	int ret = -1;

	if (orb_advert_valid(handle)) {
		ret = node(handle)->unadvertise(publisher);
		nodeClose(handle);
	}

	return ret;
}

int uORB::DeviceNode::unadvertise(bool publisher)
{
	int ret = -1;

	ret = --_advertiser_count;

	if (publisher) {
		--_publisher_count;
	}

	return ret;
}

uORB::DeviceNode::DeviceNode(const ORB_ID id, const uint8_t instance, const char *path) :
	_orb_id(id),
	_instance(instance)
{
#if defined(CONFIG_BUILD_FLAT)
	_devname = strdup(path);
#else

	if (strnlen(path, sizeof(_devname)) == sizeof(_devname)) {
		PX4_ERR("node path too long %s", path);
	}

	strncpy(_devname, path, sizeof(_devname));
#endif

	int ret = px4_sem_init(&_lock, 1, 1);

	if (ret != 0) {
		PX4_DEBUG("SEM INIT FAIL: ret %d", ret);
	}
}

uORB::DeviceNode::~DeviceNode()
{
	px4_sem_destroy(&_lock);

#if defined(CONFIG_BUILD_FLAT)

	// Delete all the allocated free callback items.
	// There should not be any left in use, since the node is
	// deleted only if there are no more publishers or subscribers registered

	IndexedStackHandle<CB_LIST_T> callbacks(_callbacks);
	uorb_cb_handle_t handle = callbacks.pop_free();

	while (callbacks.handle_valid(handle)) {
		delete (static_cast<EventWaitItem *>(callbacks.peek(handle)));
		handle = callbacks.pop_free();
	}

	free(_devname);
#endif
}

/* Map the node data to the memory space of publisher or subscriber */

void uORB::DeviceNode::remap_data(orb_advert_t &handle, size_t new_size, bool publisher)
{
	// In NuttX flat and protected builds, just malloc the data (from user heap)
	// and store
	// the pointer. This saves us the inodes in the
	// kernel side. Otherwise the same logic would work

#ifdef CONFIG_BUILD_FLAT

	// Data size has changed, re-allocate (remap) by publisher
	// The remapping may happen only on the first write,
	// when the handle.data_size==0

	if (publisher && handle.data_size == 0) {
		free(_data);
		_data = malloc(new_size);
	}

	if (_data != nullptr) {
		handle.data_size = new_size;

	} else {
		handle.data_size = 0;
	}

#else

	// Open the data, the data shm name is the same as device node's except for leading '_'
	int oflag = publisher ? O_RDWR | O_CREAT : O_RDONLY;
	int shm_fd = shm_open(get_devname() + 1, oflag, 0666);

	// and mmap it
	if (shm_fd >= 0) {

		// For the publisher, set the new data size
		if (publisher && handle.data_size == 0) {
			if (ftruncate(shm_fd, new_size) != 0) {
				::close(shm_fd);
				PX4_ERR("Setting advertise size failed\n");
				return;
			}
		}

		handle = MappingCache::map_data(handle, shm_fd, new_size, publisher);

		// Close the shm, there is no need to leave it open
		::close(shm_fd);
	}

#endif
}

/**
	 * Copies data and the corresponding generation
	 * from a node to the buffer provided.
	 *
	 * @param dst
	 *   The buffer into which the data is copied.
	 * @param generation
	 *   The generation that was copied.
	 * @return bool
	 *   Returns true if the data was copied.
	 */
bool uORB::DeviceNode::copy(void *dst, orb_advert_t &handle, unsigned &generation)
{
	if (dst == nullptr || !_data_valid) {
		return false;
	}

	size_t o_size = get_meta()->o_size;
	size_t data_size = _queue_size * o_size;

	ATOMIC_ENTER;

	if (data_size != handle.data_size) {
		remap_data(handle, data_size, false);

		if (node_data(handle) == nullptr) {
			ATOMIC_LEAVE;
			return false;
		}
	}

	if (_queue_size == 1) {
		memcpy(dst, node_data(handle), o_size);
		generation = _generation.load();

	} else {
		const unsigned current_generation = _generation.load();

		if (current_generation > generation + _queue_size) {
			// Reader is too far behind: some messages are lost
			generation = current_generation - _queue_size;
		}

		if ((current_generation == generation) && (generation > 0)) {
			/* The subscriber already read the latest message, but nothing new was published yet.
			 * Return the previous message
			 */
			--generation;
		}

		memcpy(dst, ((uint8_t *)node_data(handle)) + (o_size * (generation % _queue_size)), o_size);

		if (generation < current_generation) {
			++generation;
		}
	}

	ATOMIC_LEAVE;

	return true;

}

ssize_t
uORB::DeviceNode::write(const char *buffer, const orb_metadata *meta, orb_advert_t &handle)
{
	size_t o_size = meta->o_size;

	/* If data size has changed, re-map the data */
	size_t data_size = _queue_size * o_size;

	/* Perform an atomic copy. */
	ATOMIC_ENTER;

	if (data_size != handle.data_size) {
		remap_data(handle, data_size, true);
	}

	if (node_data(handle) == nullptr) {
		ATOMIC_LEAVE;
		return -ENOMEM;
	}

	/* wrap-around happens after ~49 days, assuming a publisher rate of 1 kHz */
	unsigned generation = _generation.fetch_add(1);

	memcpy(((uint8_t *)node_data(handle)) + o_size * (generation % _queue_size), buffer, o_size);

	/* Mark at least one data has been published */
	_data_valid = true;

	uORB::DeviceNode *n = node(handle);
	IndexedStackHandle<CB_LIST_T> callbacks(n->_callbacks);

	uorb_cb_handle_t cb = callbacks.head();

	while (callbacks.handle_valid(cb)) {
		EventWaitItem *item = callbacks.peek(cb);

		if (item->interval_us == 0 || hrt_elapsed_time(&item->last_update) >= item->interval_us) {
			if (item->subscriber != nullptr) {
#ifdef CONFIG_BUILD_FLAT
				item->subscriber->call();
#else
				Manager::queueCallback(item->subscriber);
#endif
			}

			// Release poll waiters (and callback threads in non-flat builds)
			if (item->lock != -1) {
				Manager::unlockThread(item->lock);
			}
		}

		cb = callbacks.next(cb);
	}

	ATOMIC_LEAVE;

	return o_size;
}

ssize_t
uORB::DeviceNode::publish(const orb_metadata *meta, orb_advert_t &handle, const void *data)
{
	uORB::DeviceNode *devnode = node(handle);
	int ret;

	/* check if the device handle is initialized and data is valid */
	if ((devnode == nullptr) || (meta == nullptr) || (data == nullptr)) {
		errno = EFAULT;
		return PX4_ERROR;
	}

	/* check if the orb meta data matches the publication */
	if (static_cast<uint8_t>(devnode->id()) != meta->o_id) {
		errno = EINVAL;
		return PX4_ERROR;
	}

	/* call the devnode write method */
	ret = devnode->write((const char *)data, meta, handle);

	if (ret != (int)meta->o_size) {
		errno = EIO;
		return PX4_ERROR;
	}

#ifdef CONFIG_ORB_COMMUNICATOR
	/*
	 * if the write is successful, send the data over the Multi-ORB link
	 */
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();

	if (ch != nullptr) {
		if (ch->send_message(meta->o_name, meta->o_size, (uint8_t *)data) != 0) {
			PX4_ERR("Error Sending [%s] topic data over comm_channel", meta->o_name);
			return PX4_ERROR;
		}
	}

#endif /* CONFIG_ORB_COMMUNICATOR */

	return PX4_OK;
}

#ifdef CONFIG_ORB_COMMUNICATOR
int16_t uORB::DeviceNode::topic_advertised(const orb_metadata *meta)
{
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();

	if (ch != nullptr && meta != nullptr) {
		return ch->topic_advertised(meta->o_name);
	}

	return -1;
}
#endif /* CONFIG_ORB_COMMUNICATOR */

bool
uORB::DeviceNode::print_statistics(int max_topic_length)
{

	lock();

	const uint8_t instance = get_instance();
	const int8_t sub_count = subscriber_count();
	const uint8_t queue_size = get_queue_size();

	unlock();

	const orb_metadata *meta = get_meta();

	PX4_INFO_RAW("%-*s %2i %4i %2i %4i %s\n", max_topic_length, meta->o_name, (int)instance, (int)sub_count,
		     queue_size, meta->o_size, get_devname());

	return true;
}

orb_advert_t uORB::DeviceNode::add_subscriber(ORB_ID orb_id, uint8_t instance,
		unsigned *initial_generation, bool advertise)
{
	orb_advert_t handle;

	if (advertise) {
		handle = orb_advertise(orb_id, instance, 0, false);

	} else {
		handle = nodeOpen(orb_id, instance, false);
	}

	if (orb_advert_valid(handle)) {
		node(handle)->_add_subscriber(initial_generation);

	} else {
		*initial_generation = 0;
	}

	return handle;
}


void uORB::DeviceNode::_add_subscriber(unsigned *initial_generation)
{
	*initial_generation = _generation.load() - (_data_valid ? 1 : 0);
	_subscriber_count++;

#ifdef CONFIG_ORB_COMMUNICATOR
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();

	if (ch != nullptr && _subscriber_count > 0) {
		unlock(); //make sure we cannot deadlock if add_subscription calls back into DeviceNode
		ch->add_subscription(get_name(), 1);

	}

#endif /* CONFIG_ORB_COMMUNICATOR */
}


int8_t uORB::DeviceNode::remove_subscriber(orb_advert_t &handle, bool advertiser)
{
	int8_t ret = _subscriber_count--;

	if (advertiser) {
		orb_unadvertise(handle, false);

	} else {
		nodeClose(handle);

	}

#ifdef CONFIG_ORB_COMMUNICATOR
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();

	if (ch != nullptr && ret == 0) {
		ch->remove_subscription(get_meta()->o_name);

	}

#endif /* ORB_COMMUNICATOR */

	return ret;
}

#ifdef CONFIG_ORB_COMMUNICATOR
int16_t uORB::DeviceNode::process_add_subscription(orb_advert_t &handle)
{
	// if there is already data in the node, send this out to
	// the remote entity.
	// send the data to the remote entity.
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();
	const orb_metadata *meta = get_meta();

	if (ch != nullptr) {
		ch->send_message(meta->o_name, meta->o_size, (uint8_t *)node_data(handle));
	}

	return PX4_OK;
}

int16_t uORB::DeviceNode::process_remove_subscription(orb_advert_t &handle)
{
	return PX4_OK;
}

int16_t uORB::DeviceNode::process_received_message(orb_advert_t &handle, int32_t length, uint8_t *data)
{
	int16_t ret = -1;

	if (!orb_advert_valid(handle)) {
		return ret;
	}

	const orb_metadata *meta = get_meta();

	if (length != (int32_t)(meta->o_size)) {
		PX4_ERR("Received '%s' with DataLength[%d] != ExpectedLen[%d]", meta->o_name, (int)length,
			(int)meta->o_size);
		return PX4_ERROR;
	}

	/* call the devnode write method */
	ret = write((const char *)data, meta, handle);

	if (ret < 0) {
		return PX4_ERROR;
	}

	if (ret != (int)meta->o_size) {
		errno = EIO;
		return PX4_ERROR;
	}

	return PX4_OK;
}
#endif /* CONFIG_ORB_COMMUNICATOR */

int uORB::DeviceNode::update_queue_size(unsigned int queue_size)
{
	// subscribers may advertise the node, but not set the queue_size
	if (queue_size == 0) {
		return PX4_OK;
	}

	queue_size = round_pow_of_two_8(queue_size);

	// queue size is limited to 255 for the single reason that we use uint8 to store it
	if (queue_size > 255) {
		return PX4_ERROR;
	}

	_queue_size = queue_size;

	return PX4_OK;
}

//TODO: make this a normal member function
uorb_cb_handle_t
uORB::DeviceNode::register_callback(orb_advert_t &node_handle, uORB::SubscriptionCallback *callback_sub,
				    int8_t poll_lock, hrt_abstime last_update, uint32_t interval_us)
{
	uORB::DeviceNode *n = node(node_handle);

	n->lock();

#ifndef CONFIG_BUILD_FLAT
	// Get the cb lock for this process from the Manager
	int8_t lock = poll_lock == -1 ? Manager::getCallbackLock() : poll_lock;
#else
	int8_t lock = poll_lock;
#endif

	// TODO: Check for duplicate registrations?

	IndexedStackHandle<CB_LIST_T> callbacks(n->_callbacks);
	uorb_cb_handle_t i = callbacks.pop_free();
	EventWaitItem *item = callbacks.peek(i);

#ifdef CONFIG_BUILD_FLAT

	if (!item) {
		item = new EventWaitItem;
		i = item;
	}

#endif

	if (item != nullptr) {
		item->lock = lock;
		item->subscriber = callback_sub;
		item->last_update = last_update;
		item->interval_us = interval_us;

		callbacks.push(i);

	} else {
		PX4_ERR("register fail\n");
	}

	n->unlock();

	return i;
}

//TODO: make this a normal member function?
void
uORB::DeviceNode::unregister_callback(orb_advert_t &node_handle, uorb_cb_handle_t cb_handle)
{
	uORB::DeviceNode *n = node(node_handle);

	n->lock();

	IndexedStackHandle<CB_LIST_T> callbacks(n->_callbacks);

	if (!callbacks.rm(cb_handle)) {
		PX4_ERR("unregister fail\n");

	} else {
		callbacks.push_free(cb_handle);
	}

	n->unlock();
}
