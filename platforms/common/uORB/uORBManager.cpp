/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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

#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/shm.h>
#include <stdarg.h>
#include <fcntl.h>

#include <px4_platform_common/mmap.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>

#include "uORBDeviceNode.hpp"
#include "uORBUtils.hpp"
#include "uORBManager.hpp"
#include "SubscriptionCallback.hpp"

#ifndef CONFIG_FS_SHMFS_VFS_PATH
#define CONFIG_FS_SHMFS_VFS_PATH "/dev/shm"
#endif

static const char uORBManagerName[] = "_uORB_Manager";

#ifdef CONFIG_ORB_COMMUNICATOR
pthread_mutex_t uORB::Manager::_communicator_mutex = PTHREAD_MUTEX_INITIALIZER;
#endif

uORB::Manager *uORB::Manager::_Instance = nullptr;

// This is the per-process lock for callback thread
#ifndef CONFIG_BUILD_FLAT
int8_t uORB::Manager::per_process_lock = -1;
pid_t uORB::Manager::per_process_cb_thread = -1;
#endif

void uORB::Manager::cleanup()
{
	// TODO: This is operating system dependent. Works on linux and NuttX
	DIR *shm_dir = opendir(CONFIG_FS_SHMFS_VFS_PATH);
	struct dirent *next_file;

	// Delete all uorb shm allocations
	while ((next_file = readdir(shm_dir)) != nullptr) {
		// build the path for each file in the folder
		if (!strncmp(next_file->d_name, "orb_", 4) ||
		    !strncmp(next_file->d_name, "_orb_", 5)) {
			shm_unlink(next_file->d_name);
		}
	}

	closedir(shm_dir);

	// Delete manager shm allocations
	shm_unlink(uORBManagerName);
}

bool uORB::Manager::initialize()
{
	if (_Instance == nullptr) {

		// Cleanup from previous execution, in case some shm files are left
		cleanup();

		// Create a shared memory segment for uORB Manager and initialize a new manager into it
		int shm_fd = shm_open(uORBManagerName, O_CREAT | O_RDWR, 0666);

		if (shm_fd >= 0) {
			// If the creation succeeded, set the size
			if (ftruncate(shm_fd, sizeof(uORB::Manager)) == 0) {
				// mmap the shared memory region
				void *ptr = px4_mmap(0, sizeof(uORB::Manager), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
				_Instance = new (ptr) uORB::Manager();

				for (auto &publisher : _Instance->g_has_publisher) {
					publisher = 0;
				}
			}
		}
	}

#if defined(CONFIG_FS_SHMFS_WRPROTECT)
	px4_register_boardct_ioctl(_ORBIOCDEVBASE, orb_ioctl);
#endif

	return _Instance != nullptr;
}

void uORB::Manager::map_instance()
{
	if (_Instance == nullptr) {

		// Open the existing manager
		int shm_fd = shm_open(uORBManagerName, O_RDWR, 0666);

		if (shm_fd >= 0) {
			// mmap the shared memory region
			void *ptr = px4_mmap(0, sizeof(uORB::Manager), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);

			if (ptr != MAP_FAILED) {
				_Instance = (uORB::Manager *)ptr;
			}
		}
	}

	if (_Instance == nullptr) {
		PX4_ERR("FATAL: Can't get uORB manager");
	}
}

bool uORB::Manager::terminate()
{
	// We don't delete or unmap the Manager. Cleanup will
	// unlink the SHM, and all the mappings are dropped when the
	// processes exit

	cleanup();

	if (_Instance != nullptr) {
		return true;
	}

	return false;
}

uORB::Manager::Manager()
{
	int ret;

#ifdef ORB_USE_PUBLISHER_RULES
	const char *file_name = PX4_STORAGEDIR"/orb_publisher.rules";
	ret = readPublisherRulesFromFile(file_name, _publisher_rule);

	if (ret == PX4_OK) {
		_has_publisher_rules = true;
		PX4_INFO("Using orb rules from %s", file_name);

	} else {
		PX4_ERR("Failed to read publisher rules file %s (%s)", file_name, strerror(-ret));
	}

#endif /* ORB_USE_PUBLISHER_RULES */

	ret = px4_sem_init(&_lock, 1, 1);

	if (ret != 0) {
		PX4_DEBUG("SEM INIT FAIL: ret %d", ret);
	}

	ret = px4_sem_init(&_callback_lock, 1, 1);

	if (ret != 0) {
		PX4_DEBUG("SEM INIT FAIL: ret %d", ret);
	}

	g_sem_pool.init();
}

uORB::Manager::~Manager()
{
	px4_sem_destroy(&_lock);
	px4_sem_destroy(&_callback_lock);
}

int uORB::Manager::orb_exists(const struct orb_metadata *meta, int instance)
{
	// instance valid range: [0, ORB_MULTI_MAX_INSTANCES)
	if ((instance < 0) || (instance > (ORB_MULTI_MAX_INSTANCES - 1)) || meta == nullptr) {
		return PX4_ERROR;
	}

	// meta != nullptr
	// orb is advertised by a publisher
	if (meta != nullptr &&
	    has_publisher(static_cast<ORB_ID>(meta->o_id), instance)) {
		return PX4_OK;
	}

	return PX4_ERROR;
}

orb_advert_t uORB::Manager::orb_advertise_multi(const struct orb_metadata *meta, const void *data, int *instance,
		uint8_t queue_size)
{
#ifdef ORB_USE_PUBLISHER_RULES

	// check publisher rule
	if (_has_publisher_rules) {
		const char *prog_name = px4_get_taskname();

		if (strcmp(_publisher_rule.module_name, prog_name) == 0) {
			if (_publisher_rule.ignore_other_topics) {
				if (!findTopic(_publisher_rule, meta->o_name)) {
					PX4_DEBUG("not allowing %s to publish topic %s", prog_name, meta->o_name);
					return ORB_ADVERT_INVALID;
				}
			}

		} else {
			if (findTopic(_publisher_rule, meta->o_name)) {
				PX4_DEBUG("not allowing %s to publish topic %s", prog_name, meta->o_name);
				return ORB_ADVERT_INVALID;
			}
		}
	}

#endif /* ORB_USE_PUBLISHER_RULES */

	// Calculate the wanted instance
	unsigned group_tries = 0;

	lock();

	while (group_tries < ORB_MULTI_MAX_INSTANCES) {

		// is not advertised by a publisher or is a single instance publisher
		if (!has_publisher(static_cast<ORB_ID>(meta->o_id), group_tries) || !instance) {
			break;
		}

		group_tries++;
	}

	if (group_tries == ORB_MULTI_MAX_INSTANCES) {
		unlock();
		PX4_ERR("%s: too many instances (%d)", meta->o_name, group_tries);
		return ORB_ADVERT_INVALID;
	}

	orb_advert_t handle = uORB::DeviceNode::orb_advertise(static_cast<ORB_ID>(meta->o_id), group_tries, queue_size, true);

	if (instance != nullptr) {
		*instance = group_tries;
	}

	// Cache existence of this node instance globally
	if (orb_advert_valid(handle)) {
		set_has_publisher(static_cast<ORB_ID>(meta->o_id), group_tries);

#ifdef CONFIG_ORB_COMMUNICATOR
		// For remote systems call over and inform them
		uORB::DeviceNode::topic_advertised(meta);
#endif /* CONFIG_ORB_COMMUNICATOR */

	} else {
		PX4_ERR("orb_advertise_multi failed %s", meta->o_name);
	}

	unlock();

	/* the advertiser may perform an initial publish to initialise the object */

	if (data != nullptr && orb_advert_valid(handle)) {
		int result = orb_publish(meta, handle, data);

		if (result == PX4_ERROR) {
			PX4_ERR("orb_publish failed %s", meta->o_name);
			orb_unadvertise(handle);
		}
	}

	return handle;
}

int uORB::Manager::orb_unadvertise(orb_advert_t &handle)
{
	if (!orb_advert_valid(handle)) {
		return PX4_ERROR;
	}

	ORB_ID id = static_cast<ORB_ID>(node(handle)->id());
	uint8_t instance = node(handle)->get_instance();

	Manager *manager = get_instance();

	manager->lock();

	bool unadvertised = uORB::DeviceNode::orb_unadvertise(handle, true) >= 0;

	// Node is deleted and handle invalidated, if the last advertiser goes away

	if (!orb_advert_valid(handle) || node(handle)->publisher_count() == 0) {
		manager->unset_has_publisher(id, instance);
	}

	manager->unlock();

	return unadvertised ? PX4_OK : PX4_ERROR;
}

// Should only be called from old interface
orb_sub_t uORB::Manager::orb_subscribe(const struct orb_metadata *meta)
{
	return orb_subscribe_multi(meta, 0);
}

// Should only be called from old interface
orb_sub_t uORB::Manager::orb_subscribe_multi(const struct orb_metadata *meta, unsigned instance)
{
	uORB::SubscriptionInterval *sub = new uORB::SubscriptionPollable(meta, instance);

	if (sub && !sub->valid()) {
		// subscribe and advertise the topic
		sub->subscribe(true);
	}

	return sub;
}

int uORB::Manager::orb_unsubscribe(orb_sub_t handle)
{
	delete (static_cast<SubscriptionCallback *>(handle));
	return PX4_OK;
}

int uORB::Manager::orb_copy(const struct orb_metadata *meta, orb_sub_t handle, void *buffer)
{
	if (!(static_cast<SubscriptionInterval *>(handle))->copy(buffer)) {
		errno = EIO;
		return PX4_ERROR;
	}

	return PX4_OK;
}

int uORB::Manager::orb_check(orb_sub_t handle, bool *updated)
{
	*updated = ((uORB::SubscriptionInterval *)handle)->updated();
	return PX4_OK;
}

int uORB::Manager::orb_set_interval(orb_sub_t handle, unsigned interval)
{
	((uORB::SubscriptionInterval *)handle)->set_interval_us(interval * 1000);
	return PX4_OK;
}

int uORB::Manager::orb_get_interval(orb_sub_t handle, unsigned *interval)
{
	*interval = ((uORB::SubscriptionInterval *)handle)->get_interval_us() / 1000;
	return PX4_OK;
}

int uORB::Manager::orb_poll(orb_poll_struct_t *fds, unsigned int nfds, int timeout)
{
	SubscriptionPollable *sub;

	// Get a poll semaphore from the global pool
	int8_t lock_idx = g_sem_pool.reserve();

	if (lock_idx < 0) {
		PX4_ERR("Out of thread locks");
		return -1;
	}

	// Any orb updated already?
	bool err = false;
	int count = 0;

	for (unsigned i = 0; i < nfds; i++) {
		fds[i].revents = 0;

		if ((fds[i].events & POLLIN) == POLLIN) {
			sub = static_cast<SubscriptionPollable *>(fds[i].fd);
			sub->registerPoll(lock_idx);

			if (sub->updated()) {
				fds[i].revents = POLLIN;
				count++;
			}
		}
	}

	// If none of the orbs were updated before registration, go to sleep.
	// If some orb was updated after registration, but not yet refelected in "updated", the semaphore is already released. So there is no race in here.

	if (count == 0) {

		// First advertiser will wake us up, or it might have happened already
		// during registration above

		int ret;

		if (timeout < 0) {
			// Wait event until interrupted by a signal
			ret = g_sem_pool.take_interruptible(lock_idx);

		} else {
			// Wait event for a maximum timeout time
			struct timespec to;
#if defined(ENABLE_LOCKSTEP_SCHEDULER)
			px4_clock_gettime(CLOCK_MONOTONIC, &to);
#else
			px4_clock_gettime(CLOCK_REALTIME, &to);
#endif
			hrt_abstime now = ts_to_abstime(&to);
			abstime_to_ts(&to, now + (hrt_abstime)timeout * 1000);
			ret = g_sem_pool.take_timedwait(lock_idx, &to);
		}

		if (ret != 0 && errno != ETIMEDOUT && errno != EINTR) {
			PX4_ERR("poll on %d timeout %d FAIL errno %d\n", lock_idx, timeout, errno);
			err = true;
		}
	}

	count = 0;

	for (unsigned i = 0; i < nfds; i++) {
		if ((fds[i].events & POLLIN) == POLLIN) {
			sub = static_cast<SubscriptionPollable *>(fds[i].fd);
			sub->unregisterPoll();

			if (sub->updated()) {
				fds[i].revents |= POLLIN;
				count++;
			}
		}
	}

	// recover from releasing multiple times
	g_sem_pool.set(lock_idx, 0);
	g_sem_pool.free(lock_idx);

	return err ? -1 : count;
}

#ifndef CONFIG_BUILD_FLAT

int8_t
uORB::Manager::launchCallbackThread()
{
	per_process_lock = Manager::getThreadLock();

	if (per_process_lock < 0) {
		PX4_ERR("Out of thread locks\n");
		return -1;
	}

	if (per_process_cb_thread == -1) {
		per_process_cb_thread = px4_task_spawn_cmd("orb_callback",
					SCHED_DEFAULT,
					SCHED_PRIORITY_MAX - 1,
					1024,
					callback_thread,
					nullptr);

		if (per_process_cb_thread < 0) {
			PX4_ERR("callback thread creation failed\n");
			Manager::freeThreadLock(per_process_lock);
			return -1;
		}
	}

	return per_process_lock;
}

int
uORB::Manager::callback_thread(int argc, char *argv[])
{
	while (true) {
		lockThread(per_process_lock);

		SubscriptionCallback *sub = _Instance->_callback_ptr;
		_Instance->unlock_callbacks();

		// Pass nullptr to this thread to exit
		if (sub == nullptr) {
			break;
		}

		sub->call();
	}

	Manager::freeThreadLock(per_process_lock);
	per_process_lock = -1;

	return 0;
}

#endif


void uORB::Manager::GlobalSemPool::init(void)
{
	for (auto &sem : _global_sem) {
		sem.init();
	}

	px4_sem_init(&_semLock, 1, 1);
}

void uORB::Manager::GlobalSemPool::free(int8_t i)
{
	lock();

	_global_sem[i].in_use = false;

	unlock();
}

int8_t uORB::Manager::GlobalSemPool::reserve()
{
	lock();

	// Find the first free lock
	int8_t i;

	for (i = 0; i < NUM_GLOBAL_SEMS; i++) {
		if (!_global_sem[i].in_use) {
			break;
		}
	}

	// Check that we got one
	if (i ==  NUM_GLOBAL_SEMS) {
		PX4_ERR("Out of global locks");
		unlock();
		return -1;
	}

	// Mark this one as in use
	_global_sem[i].in_use = true;

	unlock();

	return i;
}

#ifdef CONFIG_ORB_COMMUNICATOR
void uORB::Manager::set_uorb_communicator(uORBCommunicator::IChannel *channel)
{
	_comm_channel = channel;

	if (_comm_channel != nullptr) {
		_comm_channel->register_handler(this);
	}
}

uORBCommunicator::IChannel *uORB::Manager::get_uorb_communicator()
{
	pthread_mutex_lock(&_communicator_mutex);
	uORBCommunicator::IChannel *temp = _comm_channel;
	pthread_mutex_unlock(&_communicator_mutex);

	return temp;
}

const struct orb_metadata *uORB::Manager::topic_name_to_meta(const char *topic_name)
{
	const struct orb_metadata *const *topic_list = orb_get_topics();
	orb_id_t topic_ptr = nullptr;

	for (size_t i = 0; i < orb_topics_count(); i++) {
		if (strcmp(topic_list[i]->o_name, topic_name) == 0) {
			topic_ptr = topic_list[i];
			break;
		}
	}

	return topic_ptr;
}

int16_t uORB::Manager::process_remote_topic(const char *topic_name)
{
	PX4_DEBUG("entering process_remote_topic: name: %s", topic_name);

	int16_t rc = 0;

	const struct orb_metadata *meta = topic_name_to_meta(topic_name);

	if (meta == nullptr) {
		PX4_INFO("process_remote_topic meta not found for %s\n", topic_name);
		_remote_topics.erase(topic_name);
		return -1;
	}

	_Instance->lock();

	orb_advert_t handle = orb_advertise(meta, nullptr);

	_Instance->unlock();

	if (orb_advert_valid(handle)) {
		PX4_INFO("Advertising remote publisher %s", topic_name);
		_remote_topics.insert(handle);

	} else {
		PX4_INFO("Advertisement failed");
		rc = -1;
	}

	return rc;
}

int16_t uORB::Manager::process_add_subscription(const char *messageName)
{
	PX4_DEBUG("entering Manager_process_add_subscription: name: %s", messageName);

	int16_t rc = -1;

	const struct orb_metadata *meta = topic_name_to_meta(messageName);

	if (meta == nullptr) {
		return rc;
	}

	orb_advert_t handle = DeviceNode::nodeOpen(static_cast<ORB_ID>(meta->o_id), 0, false);

	if (!orb_advert_valid(handle)) {
		PX4_DEBUG("DeviceNode(%s) not created yet", messageName);

	} else {
		// node is present.

		node(handle)->process_add_subscription(handle);
		rc = 0;
	}

	return rc;
}

int16_t uORB::Manager::process_remove_subscription(const char *messageName)
{
	int16_t rc = -1;

	const struct orb_metadata *meta = topic_name_to_meta(messageName);

	if (meta == nullptr) {
		PX4_DEBUG("DeviceNode(%s) meta not found", messageName);
		return rc;
	}

	orb_advert_t handle = DeviceNode::nodeOpen(static_cast<ORB_ID>(meta->o_id), 0, false);

	if (!orb_advert_valid(handle)) {
		PX4_DEBUG("[posix-uORB::Manager::process_remove_subscription(%d)]Error No existing subscriber found for message: [%s]",
			  __LINE__, messageName);

	} else {
		// node is present.
		node(handle)->process_remove_subscription(handle);
		rc = 0;
	}

	return rc;
}

int16_t uORB::Manager::process_received_message(const char *messageName, int32_t length, uint8_t *data)
{
	int16_t rc = -1;

	orb_advert_t handle = _remote_topics.find(messageName);

	if (!orb_advert_valid(handle)) {
		PX4_DEBUG("No existing subscriber found for message: [%s]", messageName);

	} else {
		// node is present.
		node(handle)->process_received_message(handle, length, data);
		rc = 0;
	}

	return rc;
}

#endif /* CONFIG_ORB_COMMUNICATOR */

#ifdef ORB_USE_PUBLISHER_RULES

bool uORB::Manager::startsWith(const char *pre, const char *str)
{
	size_t lenpre = strlen(pre),
	       lenstr = strlen(str);
	return lenstr < lenpre ? false : strncmp(pre, str, lenpre) == 0;
}

bool uORB::Manager::findTopic(const PublisherRule &rule, const char *topic_name)
{
	const char **topics_ptr = rule.topics;

	while (*topics_ptr) {
		if (strcmp(*topics_ptr, topic_name) == 0) {
			return true;
		}

		++topics_ptr;
	}

	return false;
}

void uORB::Manager::strTrim(const char **str)
{
	while (**str == ' ' || **str == '\t') { ++(*str); }
}

int uORB::Manager::readPublisherRulesFromFile(const char *file_name, PublisherRule &rule)
{
	FILE *fp;
	static const int line_len = 1024;
	int ret = PX4_OK;
	char *line = new char[line_len];

	if (!line) {
		return -ENOMEM;
	}

	fp = fopen(file_name, "r");

	if (fp == NULL) {
		delete[](line);
		return -errno;
	}

	const char *restrict_topics_str = "restrict_topics:";
	const char *module_str = "module:";
	const char *ignore_others = "ignore_others:";

	rule.ignore_other_topics = false;
	rule.module_name = nullptr;
	rule.topics = nullptr;

	while (fgets(line, line_len, fp) && ret == PX4_OK) {

		if (strlen(line) < 2 || line[0] == '#') {
			continue;
		}

		if (startsWith(restrict_topics_str, line)) {
			//read topics list
			char *start = line + strlen(restrict_topics_str);
			strTrim((const char **)&start);
			char *topics = strdup(start);
			int topic_len = 0, num_topics = 0;

			for (int i = 0; topics[i]; ++i) {
				if (topics[i] == ',' || topics[i] == '\n') {
					if (topic_len > 0) {
						topics[i] = 0;
						++num_topics;
					}

					topic_len = 0;

				} else {
					++topic_len;
				}
			}

			if (num_topics > 0) {
				rule.topics = new const char *[num_topics + 1];
				int topic = 0;
				strTrim((const char **)&topics);
				rule.topics[topic++] = topics;

				while (topic < num_topics) {
					if (*topics == 0) {
						++topics;
						strTrim((const char **)&topics);
						rule.topics[topic++] = topics;

					} else {
						++topics;
					}
				}

				rule.topics[num_topics] = nullptr;
			}

		} else if (startsWith(module_str, line)) {
			//read module name
			char *start = line + strlen(module_str);
			strTrim((const char **)&start);
			int len = strlen(start);

			if (len > 0 && start[len - 1] == '\n') {
				start[len - 1] = 0;
			}

			rule.module_name = strdup(start);

		} else if (startsWith(ignore_others, line)) {
			const char *start = line + strlen(ignore_others);
			strTrim(&start);

			if (startsWith("true", start)) {
				rule.ignore_other_topics = true;
			}

		} else {
			PX4_ERR("orb rules file: wrong format: %s", line);
			ret = -EINVAL;
		}
	}

	if (ret == PX4_OK && (!rule.module_name || !rule.topics)) {
		PX4_ERR("Wrong format in orb publisher rules file");
		ret = -EINVAL;
	}

	delete[](line);
	fclose(fp);
	return ret;
}

#endif /* ORB_USE_PUBLISHER_RULES */
