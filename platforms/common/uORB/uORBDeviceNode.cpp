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

#if !defined(POSIX_SHM_DISABLED)
#include <sys/shm.h>
#include <dirent.h>
#endif
#include <stdlib.h>

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

#include <px4_platform_common/mmap.h>
#include <px4_platform_common/sem.hpp>
#include <drivers/drv_hrt.h>

#ifndef CONFIG_FS_SHMFS_VFS_PATH
#define CONFIG_FS_SHMFS_VFS_PATH "/dev/shm"
#endif

uORB::DeviceNode::MappingCache::MappingCacheListItem *&uORB::DeviceNode::MappingCache::cache()
{
	// Every subscriber thread has its own list of cached subscriptions
	static MappingCacheListItem *g_mapping_cache{nullptr};
	return g_mapping_cache;
}

px4_sem_t &uORB::DeviceNode::MappingCache::cache_lock()
{
	// This lock protects the subscription cache list from concurrent accesses by the threads in the same process
	static px4_sem_t g_mapping_cache_lock;
	return g_mapping_cache_lock;
}

bool &uORB::DeviceNode::MappingCache::initialized()
{
	static bool g_mapping_cache_initialized{false};
	return g_mapping_cache_initialized;
}

void uORB::DeviceNode::MappingCache::init()
{
	if (!initialized()) {
		px4_sem_init(&cache_lock(), 0, 1);
		initialized() = true;
	}
}

void uORB::DeviceNode::MappingCache::lock()
{
	init();
	do {} while (px4_sem_wait(&cache_lock()) != 0);
}

void uORB::DeviceNode::MappingCache::unlock()
{
	px4_sem_post(&cache_lock());
}

static bool topic_matches_filter(const uORB::DeviceNode &node, char **topic_filter, int num_filters)
{
	if (num_filters <= 0 || topic_filter == nullptr) {
		return true;
	}

	for (int i = 0; i < num_filters; ++i) {
		if (strstr(node.get_name(), topic_filter[i])) {
			return true;
		}
	}

	return false;
}

static int append_statistics_node(uORB::DeviceNodeStatisticsData **first_node,
				  uORB::DeviceNodeStatisticsData *&last_node,
				  uORB::DeviceNode &node,
				  size_t &max_topic_name_length)
{
	if (last_node) {
		last_node->next = new uORB::DeviceNodeStatisticsData();
		last_node = last_node->next;

	} else {
		*first_node = last_node = new uORB::DeviceNodeStatisticsData();
	}

	if (last_node == nullptr) {
		return -ENOMEM;
	}

	last_node->node = &node;

	const size_t name_length = strlen(last_node->node->get_meta()->o_name);

	if (name_length > max_topic_name_length) {
		max_topic_name_length = name_length;
	}

	last_node->last_pub_msg_count = last_node->node->updates_available(0);
	return PX4_OK;
}

int uORB::DeviceNode::fillStatistics(DeviceNodeStatisticsData **first_node, int &num_topics,
				     size_t &max_topic_name_length, char **topic_filter, int num_filters)
{
	DeviceNodeStatisticsData *cur_node = nullptr;
	DeviceNodeStatisticsData *last_node = *first_node;
	num_topics = 0;

	while (last_node && last_node->next) {
		last_node = last_node->next;
	}

#if defined(POSIX_SHM_DISABLED)
	MappingCache::lock();
	MappingCache::MappingCacheListItem *cache_item = MappingCache::list_head();

	while (cache_item != nullptr) {
		DeviceNode *node = static_cast<DeviceNode *>(cache_item->handle);

		// check if already added
		cur_node = *first_node;

		while (cur_node) {
			if (cur_node->node == node) {
				break;
			}

			cur_node = cur_node->next;
		}

		if (cur_node) {
			cache_item = cache_item->next;
			continue;
		}

		++num_topics;

		if (!topic_matches_filter(*node, topic_filter, num_filters)) {
			cache_item = cache_item->next;
			continue;
		}

		if (append_statistics_node(first_node, last_node, *node, max_topic_name_length) != PX4_OK) {
			MappingCache::unlock();
			return -ENOMEM;
		}

		cache_item = cache_item->next;
	}

	MappingCache::unlock();
	return PX4_OK;

#else
	DIR *shm_dir = opendir(CONFIG_FS_SHMFS_VFS_PATH);

	if (shm_dir == nullptr) {
		return -errno;
	}

	struct dirent *shm = nullptr;

	const char orb_name_prefix[] = "_orb_";

	char orb_name[orb_maxpath];

	while ((shm = readdir(shm_dir)) != nullptr) {

		if (strncmp(orb_name_prefix, shm->d_name, sizeof(orb_name_prefix) - 1) != 0) {
			continue;
		}

		cur_node = *first_node;

		while (cur_node) {
			int instance = cur_node->node->get_instance();

			if (uORB::Utils::node_mkpath(orb_name, cur_node->node->get_meta(), &instance)) {
				closedir(shm_dir);
				PX4_ERR("Can't construct orb name?");
				return PX4_ERROR;
			}

			if (!strcmp(orb_name, shm->d_name)) {
				break;
			}

			cur_node = cur_node->next;
		}

		if (cur_node) {
			continue;
		}

		void *ptr = nullptr;
		DeviceNode *node = nullptr;
		const int shm_fd = shm_open(shm->d_name, O_RDWR, 0666);

		if (shm_fd >= 0) {
			ptr = px4_mmap(nullptr, sizeof(DeviceNode), PROT_READ, MAP_SHARED, shm_fd, 0);
		}

		if (ptr != MAP_FAILED) {
			node = static_cast<DeviceNode *>(ptr);
		}

		if (shm_fd >= 0) {
			close(shm_fd);
		}

		if (node == nullptr) {
			PX4_ERR("Failed to MMAP an existing node");
			continue;
		}

		++num_topics;

		if (!topic_matches_filter(*node, topic_filter, num_filters)) {
			px4_munmap(node, sizeof(DeviceNode));
			continue;
		}

		if (append_statistics_node(first_node, last_node, *node, max_topic_name_length) != PX4_OK) {
			px4_munmap(node, sizeof(DeviceNode));
			closedir(shm_dir);
			return -ENOMEM;
		}
	}

	closedir(shm_dir);
	return PX4_OK;
#endif
}

void uORB::DeviceNode::cleanupStatisticsData(DeviceNodeStatisticsData *first_node)
{
	DeviceNodeStatisticsData *cur_node = first_node;

	while (cur_node) {
		DeviceNodeStatisticsData *next_node = cur_node->next;

#if !defined(POSIX_SHM_DISABLED)
		px4_munmap(cur_node->node, sizeof(DeviceNode));
#endif

		delete cur_node;
		cur_node = next_node;
	}
}

orb_advert_t uORB::DeviceNode::MappingCache::get(ORB_ID orb_id, uint8_t instance)
{
	lock();

	MappingCacheListItem *item = cache();

	while (item &&
	       (orb_id != node(item->handle)->id() ||
		instance != node(item->handle)->get_instance())) {
		item = item->next;
	}

	unlock();

	return item != nullptr ? item->handle : ORB_ADVERT_INVALID;
}

bool uORB::DeviceNode::MappingCache::add(const orb_advert_t &handle)
{
	lock();
	MappingCacheListItem *item = new MappingCacheListItem{cache(), handle};

	if (item) {
		cache() = item;
	}

	unlock();

	return item != nullptr;
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
	void *ptr = px4_mmap(nullptr, get_orb_size(orb_id), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd,
			     0);

	if (ptr != MAP_FAILED) {
		// In NuttX flat and protected builds we can just drop the mappings
		// to save some kernel memory. There is no MMU, and the memory is
		// there until the shm object is unlinked
#if defined(CONFIG_BUILD_FLAT)
		px4_munmap(ptr, get_orb_size(orb_id));
#endif

		// Create a list item and add to the beginning of the list
		handle = ptr;
		MappingCacheListItem *item = new MappingCacheListItem{cache(), handle};

		if (item) {
			cache() = item;
		}
	}

	unlock();

	return handle;
}

bool uORB::DeviceNode::MappingCache::del(const orb_advert_t &handle)
{
	MappingCacheListItem *prev = nullptr;

	lock();

	MappingCacheListItem *item = cache();

	while (item &&
	       handle != item->handle) {
		prev = item;
		item = item->next;
	}

	if (item != nullptr) {
		if (prev == nullptr) {
			// Remove the first item
			cache() = item->next;

		} else {
			prev->next = item->next;
		}

#if defined(POSIX_SHM_DISABLED)
		free((void *)handle);
#else
		px4_munmap(handle, node(handle)->get_size());
#endif

		delete (item);
	}

	unlock();

	return item != nullptr ? true : false;
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

	if (ret == -ENAMETOOLONG) {
		PX4_ERR("Device node name too long! '%s' (max path: %u chars)",
			get_orb_meta(id)->o_name, orb_maxpath - 1);
	}

	if (ret != OK) {
		return handle;
	}

#if defined(POSIX_SHM_DISABLED)

	if (!create) {
		return handle;
	}

	void *ptr = malloc(get_orb_size(id));

	if (ptr == nullptr) {
		return ORB_ADVERT_INVALID;
	}

	memset(ptr, 0, get_orb_size(id));
	handle = ptr;

	// construct the new node in the allocated region
	new (node(handle)) uORB::DeviceNode(id, instance, nodepath);

	// cache mapping for this process
	if (!MappingCache::add(handle)) {
		node(handle)->~DeviceNode();
		free(ptr);
		handle = ORB_ADVERT_INVALID;
	}

	return handle;

#else // !defined(POSIX_SHM_DISABLED)

	// First, try to create the node. This will fail if it already exists

	int shm_fd = -1;
	bool created = false;

	if (create) {
		shm_fd = shm_open(nodepath, O_CREAT | O_RDWR | O_EXCL, 0666);

		if (shm_fd >= 0) {

			// If the creation succeeded, set the size of the shm region
			if (ftruncate(shm_fd, get_orb_size(id)) != 0) {
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

#endif // defined(POSIX_SHM_DISABLED)
}

int uORB::DeviceNode::nodeClose(orb_advert_t &handle)
{
	if (!orb_advert_valid(handle)) {
		return PX4_ERROR;
	}

	if (node(handle)->_publisher_count == 0) {
		node(handle)->_data_valid = false;

		// If there are no more subscribers, delete the node and its mapping
		if (node(handle)->_subscriber_count == 0) {

			// Close the Node object

#if !defined(POSIX_SHM_DISABLED)
			shm_unlink(node(handle)->get_devname());
#endif

			// Uninitialize the node
			node(handle)->~DeviceNode();

			// Delete the mappings for this process
			MappingCache::del(handle);
		}
	}

	handle = ORB_ADVERT_INVALID;

	return PX4_OK;
}

orb_advert_t uORB::DeviceNode::orb_advertise(const ORB_ID id, int instance,
		bool publisher)
{
	/* Open the node, if it exists or create a new one */

	orb_advert_t handle;
	handle = nodeOpen(id, instance, true);

	if (orb_advert_valid(handle)) {
		node(handle)->advertise(publisher);
	}

	return handle;
}

int uORB::DeviceNode::advertise(bool publisher)
{
	int ret = -1;

	ret = ++_advertiser_count;

	if (publisher) {
		ret = ++_publisher_count;
	}

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
	int ret = px4_sem_init(&_lock, 1, 1);
	int ret2 = px4_sem_init(&_cb_lock, 1, 1);

	if (ret != 0 || ret2 != 0) {
		PX4_DEBUG("SEM INIT FAIL: _lock %d, _cb_lock %d", ret, ret2);
	}

#if defined(CONFIG_BUILD_FLAT)
	_devname = strdup(path);
#else

	if (strlen(path) >= sizeof(_devname)) {
		PX4_ERR("node path too long %s", path);
	}

	strncpy(_devname, path, sizeof(_devname) - 1);
	_devname[sizeof(_devname) - 1] = '\0';
#endif
}

uORB::DeviceNode::~DeviceNode()
{
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
	px4_sem_destroy(&_lock);
	px4_sem_destroy(&_cb_lock);
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
	size_t o_queue = get_meta()->o_queue;

	lock();

	if (o_queue == 1) {
		memcpy(dst, node_data(handle), o_size);
		generation = _generation;

	} else {
		const unsigned current_generation = _generation;

		if (current_generation == generation) {
			/* The subscriber already read the latest message, but nothing new was published yet.
			 * Return the previous message
			 */
			--generation;
		}

		/* Compatible with normal and overflow conditions */
		if (current_generation - generation > o_queue) {
			/* Reader is too far behind: some messages are lost */
			generation = current_generation - o_queue;
		}

		memcpy(dst, node_data(handle) + (o_size * (generation % o_queue)), o_size);

		++generation;
	}

	unlock();

	return true;

}

ssize_t
uORB::DeviceNode::write(const char *buffer, const orb_metadata *meta, orb_advert_t &handle)
{
	size_t o_size = meta->o_size;
	size_t o_queue = meta->o_queue;

	/* Perform an atomic copy. */
	lock();

	/* wrap-around happens after ~49 days, assuming a publisher rate of 1 kHz */
	unsigned generation = _generation++;

	memcpy(node_data(handle) + o_size * (generation % o_queue), buffer, o_size);

	/* Mark at least one data has been published */
	_data_valid = true;

	unlock();

	IndexedStackHandle<CB_LIST_T> callbacks(_callbacks);

	/* First quick-check if there might be callbacks to avoid unnecessary call to lock_cb
	 * Note that while this is a race condition, the head is checked again in the while
	 * below, after the callback list is locked
	 */

	if (!callbacks.empty()) {
		lock_cb();

		uorb_cb_handle_t cb = callbacks.head();

		while (callbacks.handle_valid(cb)) {
			EventWaitItem *item = callbacks.peek(cb);

			if (item->interval_us == 0 || hrt_elapsed_time(&item->last_update) >= item->interval_us) {

#ifdef CONFIG_BUILD_FLAT

				if (item->subscriber != nullptr) {
					// execute callback
					item->subscriber->call();

				} else {
					// release poll
					Manager::unlockThread(item->lock);
				}

#else

				// Release poll waiters and callback threads
				if (item->cb_triggered < CB_ALIVE_MAX_VALUE) {
					__atomic_fetch_add(&item->cb_triggered, 1, __ATOMIC_SEQ_CST);
					Manager::unlockThread(item->lock);

				} else if (item->cb_triggered == CB_ALIVE_MAX_VALUE) {
					// Callbacks are not being handled? Post once more and print an error
					__atomic_fetch_add(&item->cb_triggered, 1, __ATOMIC_SEQ_CST);
					Manager::unlockThread(item->lock);
					PX4_ERR("CB triggered from %s is not being handled?\n", get_name());
				}

#endif
			}

			cb = callbacks.next(cb);
		}

		unlock_cb();
	}

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
	if (static_cast<orb_id_size_t>(devnode->id()) != meta->o_id) {
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
	const uint8_t instance = get_instance();
	const int8_t sub_count = subscriber_count();
	const uint8_t queue_size = get_queue_size();
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
		handle = orb_advertise(orb_id, instance, false);

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
	*initial_generation = _generation - (_data_valid ? 1 : 0);
	_subscriber_count++;

#ifdef CONFIG_ORB_COMMUNICATOR
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();

	if (ch != nullptr && _subscriber_count > 0) {
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
		// Only send the most recent data to initialize the remote end.
		if (_data_valid) {
			ch->send_message(meta->o_name, meta->o_size,
					 node_data(handle) + (meta->o_size * ((_generation - 1) % meta->o_queue)));
		}
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

bool
uORB::DeviceNode::_register_callback(uORB::SubscriptionCallback *cb_sub,
				     int8_t poll_lock, hrt_abstime last_update, uint32_t interval_us, uorb_cb_handle_t &cb_handle)
{
	int8_t cb_lock = poll_lock;

	IndexedStackHandle<CB_LIST_T> callbacks(_callbacks);

	lock_cb();
	cb_handle = callbacks.pop_free();
	unlock_cb();

	EventWaitItem *item = callbacks.peek(cb_handle);

#ifdef CONFIG_BUILD_FLAT
	/* With flat mode it is OK to allocate more items for the list */

	if (!item) {
		item = new EventWaitItem;
		cb_handle = item;
	}

#endif

	if (item != nullptr) {
		item->lock = cb_lock;
#ifdef CONFIG_BUILD_FLAT
		item->subscriber = cb_sub;
#else
		item->cb_triggered = 0;
#endif
		item->last_update = last_update;
		item->interval_us = interval_us;
		lock_cb();
		callbacks.push(cb_handle);
		unlock_cb();

	} else {
		PX4_ERR("register fail\n");
	}

	return uorb_cb_handle_valid(cb_handle);
}

int
uORB::DeviceNode::_unregister_callback(uorb_cb_handle_t &cb_handle)
{
	IndexedStackHandle<CB_LIST_T> callbacks(_callbacks);
#ifndef CONFIG_BUILD_FLAT
	EventWaitItem *item = callbacks.peek(cb_handle);
#endif
	int ret = 0;

	lock_cb();

	bool ret_rm = callbacks.rm(cb_handle);

	if (!ret_rm) {
		PX4_ERR("unregister fail\n");

	} else {
#ifndef CONFIG_BUILD_FLAT
		ret = item->cb_triggered;
#endif
		callbacks.push_free(cb_handle);
		cb_handle = UORB_INVALID_CB_HANDLE;
	}

	unlock_cb();

	return ret;
}
