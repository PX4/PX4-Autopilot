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

#pragma once

#include <px4_platform_common/atomic.h>
#include <px4_platform_common/mmap.h>
#include <px4_platform_common/sem.h>

#include "uORBDeviceNode.hpp"
#include "uORBCommon.hpp"

#include <uORB/topics/uORBTopics.hpp> // For ORB_ID enum
#include <stdint.h>
#include <px4_platform_common/px4_config.h>

#ifdef CONFIG_ORB_COMMUNICATOR
#include "ORBSet.hpp"
#include "uORBCommunicator.hpp"
#endif /* CONFIG_ORB_COMMUNICATOR */

#define NUM_GLOBAL_SEMS 40

namespace uORB
{

/**
 * This is implemented as a singleton.  This class manages creating the
 * uORB nodes for each uORB topics and also implements the behavor of the
 * uORB Api's.
 */
class Manager
#ifdef CONFIG_ORB_COMMUNICATOR
	: public uORBCommunicator::IChannelRxHandler
#endif /* CONFIG_ORB_COMMUNICATOR */
{
public:
	// public interfaces for this class.

	/**
	 * Initialize the singleton. Call this before everything else.
	 * @return true on success
	 */
	static bool initialize();

	/**
	 * Terminate the singleton. Call this after everything else.
	 * @return true on success
	 */
	static bool terminate();

	/**
	 * Method to get the singleton instance for the uORB::Manager.
	 * @return uORB::Manager*
	 */
	static uORB::Manager *get_instance()
	{
		if (_Instance == nullptr) {
			map_instance();
		}

		return _Instance;
	}

	// ==== uORB interface methods ====
	/**
	 * Advertise as the publisher of a topic.
	 *
	 * This performs the initial advertisement of a topic; it creates the topic
	 * node in /obj if required and publishes the initial data.
	 *
	 * Any number of advertisers may publish to a topic; publications are atomic
	 * but co-ordination between publishers is not provided by the ORB.
	 *
	 * Internally this will call orb_advertise_multi with an instance of 0.
	 *
	 * @param meta    The uORB metadata (usually from the ORB_ID() macro)
	 *      for the topic.
	 * @param data    A pointer to the initial data to be published.
	 *      For topics updated by interrupt handlers, the advertisement
	 *      must be performed from non-interrupt context.
	 * @param queue_size  Maximum number of buffered elements. If this is 1, no queuing is
	 *      used.
	 * @return    nullptr on error, otherwise returns an object pointer
	 *      that can be used to publish to the topic.
	 *      If the topic in question is not known (due to an
	 *      ORB_DEFINE with no corresponding ORB_DECLARE)
	 *      this function will return nullptr and set errno to ENOENT.
	 */
	orb_advert_t orb_advertise(const struct orb_metadata *meta, const void *data, unsigned int queue_size = 1)
	{
		return orb_advertise_multi(meta, data, nullptr, queue_size);
	}

	/**
	 * Advertise as the publisher of a topic.
	 *
	 * This performs the initial advertisement of a topic; it creates the topic
	 * node in /obj if required and publishes the initial data.
	 *
	 * Any number of advertisers may publish to a topic; publications are atomic
	 * but co-ordination between publishers is not provided by the ORB.
	 *
	 * The multi can be used to create multiple independent instances of the same topic
	 * (each instance has its own buffer).
	 * This is useful for multiple publishers who publish the same topic. The subscriber
	 * then subscribes to all instances and chooses which source he wants to use.
	 *
	 * @param meta    The uORB metadata (usually from the ORB_ID() macro)
	 *      for the topic.
	 * @param data    A pointer to the initial data to be published.
	 *      For topics updated by interrupt handlers, the advertisement
	 *      must be performed from non-interrupt context.
	 * @param instance  Pointer to an integer which will yield the instance ID (0-based)
	 *      of the publication. This is an output parameter and will be set to the newly
	 *      created instance, ie. 0 for the first advertiser, 1 for the next and so on.
	 * @param queue_size  Maximum number of buffered elements. If this is 1, no queuing is
	 *      used.
	 * @return    nullptr on error, otherwise returns a handle
	 *      that can be used to publish to the topic.
	 *      If the topic in question is not known (due to an
	 *      ORB_DEFINE with no corresponding ORB_DECLARE)
	 *      this function will return nullptr and set errno to ENOENT.
	 */
	orb_advert_t orb_advertise_multi(const struct orb_metadata *meta, const void *data, int *instance,
					 uint8_t queue_size = 1);

	/**
	 * Unadvertise a topic.
	 *
	 * @param handle  handle returned by orb_advertise or orb_advertise_multi.
	 * @return 0 on success
	 */
	static int orb_unadvertise(orb_advert_t &handle);

	/**
	 * Publish new data to a topic.
	 *
	 * The data is atomically published to the topic and any waiting subscribers
	 * will be notified.  Subscribers that are not waiting can check the topic
	 * for updates using orb_check.
	 *
	 * @param meta    The uORB metadata (usually from the ORB_ID() macro)
	 *      for the topic.
	 * @handle    The handle returned from orb_advertise.
	 * @param data    A pointer to the data to be published.
	 * @return    OK on success, PX4_ERROR otherwise with errno set accordingly.
	 */
	static int  orb_publish(const struct orb_metadata *meta, orb_advert_t &handle, const void *data) {return uORB::DeviceNode::publish(meta, handle, data);}

	/**
	 * Subscribe to a topic.
	 *
	 * The returned value is a handle that can be passed to orb_poll()
	 * in order to wait for updates to a topic, as well as topic_read,
	 * orb_check.
	 *
	 * If there were any publications of the topic prior to the subscription,
	 * an orb_check right after orb_subscribe will return true.
	 *
	 * Subscription will succeed even if the topic has not been advertised;
	 * in this case the topic will have a timestamp of zero, it will never
	 * signal a poll() event, checking will always return false and it cannot
	 * be copied. When the topic is subsequently advertised, poll, check,
	 * stat and copy calls will react to the initial publication that is
	 * performed as part of the advertisement.
	 *
	 * Subscription will fail if the topic is not known to the system, i.e.
	 * there is nothing in the system that has declared the topic and thus it
	 * can never be published.
	 *
	 * Internally this will call orb_subscribe_multi with instance 0.
	 *
	 * @param meta    The uORB metadata (usually from the ORB_ID() macro)
	 *      for the topic.
	 * @return    PX4_ERROR on error, otherwise returns a handle
	 *      that can be used to read and update the topic.
	 */
	orb_sub_t orb_subscribe(const struct orb_metadata *meta);

	/**
	 * Subscribe to a multi-instance of a topic.
	 *
	 * The returned value is a handle that can be passed to orb_poll()
	 * in order to wait for updates to a topic, as well as topic_read,
	 * orb_check.
	 *
	 * If there were any publications of the topic prior to the subscription,
	 * an orb_check right after orb_subscribe_multi will return true.
	 *
	 * Subscription will succeed even if the topic has not been advertised;
	 * in this case the topic will have a timestamp of zero, it will never
	 * signal a poll() event, checking will always return false and it cannot
	 * be copied. When the topic is subsequently advertised, poll, check,
	 * stat and copy calls will react to the initial publication that is
	 * performed as part of the advertisement.
	 *
	 * Subscription will fail if the topic is not known to the system, i.e.
	 * there is nothing in the system that has declared the topic and thus it
	 * can never be published.
	 *
	 * If a publisher publishes multiple instances the subscriber should
	 * subscribe to each instance with orb_subscribe_multi
	 * (@see orb_advertise_multi()).
	 *
	 * @param meta    The uORB metadata (usually from the ORB_ID() macro)
	 *      for the topic.
	 * @param instance  The instance of the topic. Instance 0 matches the
	 *      topic of the orb_subscribe() call, higher indices
	 *      are for topics created with orb_advertise_multi().
	 * @return returns a handle
	 *      that can be used to read and update the topic.
	 *      If the topic in question is not known (due to an
	 *      ORB_DEFINE_OPTIONAL with no corresponding ORB_DECLARE)
	 *      this function will return an invalid handle and set errno to ENOENT.
	 */
	orb_sub_t orb_subscribe_multi(const struct orb_metadata *meta, unsigned instance);

	/**
	 * Unsubscribe from a topic.
	 *
	 * @param handle  A handle returned from orb_subscribe.
	 * @return    OK on success, PX4_ERROR otherwise with errno set accordingly.
	 */
	static int  orb_unsubscribe(orb_sub_t handle);

	/**
	 * Fetch data from a topic.
	 *
	 * This is the only operation that will reset the internal marker that
	 * indicates that a topic has been updated for a subscriber. Once poll
	 * or check return indicating that an update is available, this call
	 * must be used to update the subscription.
	 *
	 * @param meta    The uORB metadata (usually from the ORB_ID() macro)
	 *      for the topic.
	 * @param handle  A handle returned from orb_subscribe.
	 * @param buffer  Pointer to the buffer receiving the data, or NULL
	 *      if the caller wants to clear the updated flag without
	 *      using the data.
	 * @return    OK on success, PX4_ERROR otherwise with errno set accordingly.
	 */
	static int  orb_copy(const struct orb_metadata *meta, orb_sub_t handle, void *buffer);

	/**
	 * Check whether a topic has been published to since the last orb_copy.
	 *
	 * This check can be used to determine whether to copy the topic when
	 * not using poll(), or to avoid the overhead of calling poll() when the
	 * topic is likely to have updated.
	 *
	 * Updates are tracked on a per-handle basis; this call will continue to
	 * return true until orb_copy is called using the same handle.
	 *
	 * @param handle  A handle returned from orb_subscribe.
	 * @param updated Set to true if the topic has been updated since the
	 *      last time it was copied using this handle.
	 * @return    OK if the check was successful, PX4_ERROR otherwise with
	 *      errno set accordingly.
	 */
	static int  orb_check(orb_sub_t handle, bool *updated);

	/**
	 * Check if a topic has already been created and published (advertised)
	 *
	 * @param meta    ORB topic metadata.
	 * @param instance  ORB instance
	 * @return    OK if the topic exists, PX4_ERROR otherwise.
	 */
	static int  orb_exists(const struct orb_metadata *meta, int instance);

	/**
	 * Set the minimum interval between which updates are seen for a subscription.
	 *
	 * If this interval is set, the subscriber will not see more than one update
	 * within the period.
	 *
	 * Specifically, the first time an update is reported to the subscriber a timer
	 * is started. The update will continue to be reported via poll and orb_check, but
	 * once fetched via orb_copy another update will not be reported until the timer
	 * expires.
	 *
	 * This feature can be used to pace a subscriber that is watching a topic that
	 * would otherwise update too quickly.
	 *
	 * @param handle  A handle returned from orb_subscribe.
	 * @param interval  An interval period in milliseconds.
	 * @return    OK on success, PX4_ERROR otherwise with ERRNO set accordingly.
	 */
	static int  orb_set_interval(orb_sub_t handle, unsigned interval);

	/**
	 * Get the minimum interval between which updates are seen for a subscription.
	 *
	 * @see orb_set_interval()
	 *
	 * @param handle  A handle returned from orb_subscribe.
	 * @param interval  The returned interval period in milliseconds.
	 * @return    OK on success, PX4_ERROR otherwise with ERRNO set accordingly.
	 */
	static int	orb_get_interval(orb_sub_t handle, unsigned *interval);

	int orb_poll(orb_poll_struct_t *fds, unsigned int nfds, int timeout);

	static orb_advert_t orb_add_internal_subscriber(ORB_ID orb_id, uint8_t instance, unsigned *initial_generation,
			bool advertise)
	{
		if (!advertise && !has_publisher(orb_id, instance)) {
			return ORB_ADVERT_INVALID;
		}

		_Instance->lock();
		orb_advert_t handle = uORB::DeviceNode::add_subscriber(orb_id, instance, initial_generation, advertise);
		_Instance->unlock();

		return handle;
	}

	static void orb_remove_internal_subscriber(orb_advert_t &node_handle, bool advertiser)
	{
		_Instance->lock();
		node(node_handle)->remove_subscriber(node_handle, advertiser);
		_Instance->unlock();
	}

	static uint8_t orb_get_queue_size(const orb_advert_t &node_handle) {return node(node_handle)->get_queue_size();}

	static bool orb_data_copy(orb_advert_t &node_handle, void *dst, unsigned &generation,
				  bool only_if_updated)
	{
		if (!orb_advert_valid(node_handle) ||
		    (only_if_updated && !node(node_handle)->updates_available(generation))) {
			return false;
		}

		return node(node_handle)->copy(dst, node_handle, generation);
	}

#ifndef CONFIG_BUILD_FLAT
	static uint8_t getCallbackLock()
	{
		uint8_t cbLock;

		// TODO: think about if this needs protection, maybe not use the
		// same lock as for node advertise/subscribe

		_Instance->lock();
		cbLock = per_process_lock >= 0 ? per_process_lock : launchCallbackThread();
		_Instance->unlock();
		return cbLock;
	}
#endif

	static uint8_t orb_get_instance(orb_advert_t &node_handle)
	{
		if (orb_advert_valid(node_handle)) {
			return node(node_handle)->get_instance();
		}

		return -1;
	}

	static unsigned updates_available(const orb_advert_t &node_handle, unsigned last_generation)
	{
		return node(node_handle)->updates_available(last_generation);
	}

	static bool has_publisher(ORB_ID orb_id, uint8_t instance)
	{
		return (get_instance()->g_has_publisher[static_cast<uint8_t>(orb_id)] & (1 << instance)) != 0;
	}

#ifdef CONFIG_ORB_COMMUNICATOR

	/**
	 * Method to set the uORBCommunicator::IChannel instance.
	 * @param comm_channel
	 *  The IChannel instance to talk to remote proxies.
	 * @note:
	 *  Currently this call only supports the use of one IChannel
	 *  Future extensions may include more than one IChannel's.
	 */
	void set_uorb_communicator(uORBCommunicator::IChannel *comm_channel);

	/**
	 * Gets the uORB Communicator instance.
	 */
	uORBCommunicator::IChannel *get_uorb_communicator();

#endif /* CONFIG_ORB_COMMUNICATOR */

	void *operator new (size_t, void *p)
	{
		return p;
	}

	void operator delete (void *p)
	{
		px4_munmap(p, sizeof(uORB::Manager));
	}

	static void lockThread(int idx)
	{
		_Instance->g_sem_pool.take(idx);
	}

	static void unlockThread(int idx)
	{
		_Instance->g_sem_pool.release(idx);
	}

	static void freeThreadLock(int i) {_Instance->g_sem_pool.free(i);}

	static int8_t getThreadLock() {return _Instance->g_sem_pool.reserve();}

	static void queueCallback(class SubscriptionCallback *sub)
	{
		_Instance->lock_callbacks();
		_Instance->_callback_ptr = sub;
		// The manager is unlocked in callback thread
	}

private: // class methods
	inline static uORB::DeviceNode *node(orb_advert_t handle) {return static_cast<uORB::DeviceNode *>(handle.node);}

	static void cleanup();
	static int callback_thread(int argc, char *argv[]);
	static int8_t launchCallbackThread();

private: // data members
	static Manager *_Instance;

#ifdef CONFIG_ORB_COMMUNICATOR
	// the communicator channel instance.
	uORBCommunicator::IChannel *_comm_channel{nullptr};
	static pthread_mutex_t _communicator_mutex;

	// Track the advertisements we get from the remote side
	ORBSet _remote_topics;
#endif /* CONFIG_ORB_COMMUNICATOR */

private: //class methods
	Manager();
	~Manager();

	/**
	 * Lock against node concurrent node creation
	 */
	void		lock() { do {} while (px4_sem_wait(&_lock) != 0); }

	/**
	 * Release the node creation lock
	 */
	void		unlock() { px4_sem_post(&_lock); }

	px4_sem_t	_lock;

#ifdef CONFIG_ORB_COMMUNICATOR
	/**
	 * Helper function to find orb_metadata struct by topic name
	 * @param topic_name
	 * 	This represents the uORB message Name (topic); This message Name should be
	 * 	globally unique.
	 * @return
	 *  pointer to struct orb_metadata
	 *  nullptr = failure
	 */
	const struct orb_metadata *topic_name_to_meta(const char *topic_name);

	/**
	 * Interface to process a received topic advertisement from remote.
	 * @param topic_name
	 * 	This represents the uORB message Name (topic); This message Name should be
	 * 	globally unique.
	 * @return
	 *  0 = success; This means the messages is successfully handled in the
	 *  	handler.
	 *  otherwise = failure.
	 */
	virtual int16_t process_remote_topic(const char *topic_name);

	/**
	   * Interface to process a received AddSubscription from remote.
	   * @param messageName
	   *  This represents the uORB message Name; This message Name should be
	   *  globally unique.
	   * @return
	   *  0 = success; This means the messages is successfully handled in the
	   *    handler.
	   *  otherwise = failure.
	   */
	virtual int16_t process_add_subscription(const char *messageName);

	/**
	 * Interface to process a received control msg to remove subscription
	 * @param messageName
	 *  This represents the uORB message Name; This message Name should be
	 *  globally unique.
	 * @return
	 *  0 = success; This means the messages is successfully handled in the
	 *    handler.
	 *  otherwise = failure.
	 */
	virtual int16_t process_remove_subscription(const char *messageName);

	/**
	 * Interface to process the received data message.
	 * @param messageName
	 *  This represents the uORB message Name; This message Name should be
	 *  globally unique.
	 * @param length
	 *  The length of the data buffer to be sent.
	 * @param data
	 *  The actual data to be sent.
	 * @return
	 *  0 = success; This means the messages is successfully handled in the
	 *    handler.
	 *  otherwise = failure.
	 */
	virtual int16_t process_received_message(const char *messageName, int32_t length, uint8_t *data);
#endif /* CONFIG_ORB_COMMUNICATOR */

#ifdef ORB_USE_PUBLISHER_RULES

	struct PublisherRule {
		const char **topics; //null-terminated list of topic names
		const char *module_name; //only this module is allowed to publish one of the topics
		bool ignore_other_topics;
	};

	/**
	 * test if str starts with pre
	 */
	bool startsWith(const char *pre, const char *str);

	/**
	 * find a topic in a rule
	 */
	bool findTopic(const PublisherRule &rule, const char *topic_name);

	/**
	 * trim whitespace from the beginning of a string
	 */
	void strTrim(const char **str);

	/**
	 * Read publisher rules from a file. It has the format:
	 *
	 * restrict_topics: <topic1>, <topic2>, <topic3>
	 * module: <module_name>
	 * [ignore_others:true]
	 *
	 * @return 0 on success, <0 otherwise
	 */
	int readPublisherRulesFromFile(const char *file_name, PublisherRule &rule);
	PublisherRule _publisher_rule;
	bool _has_publisher_rules = false;

#endif /* ORB_USE_PUBLISHER_RULES */

	/**
	 * Method to map the singleton instance for the uORB::Manager
	 * to the processes address space. Make sure initialize() is called first.
	 */
	static void map_instance();

	void set_has_publisher(ORB_ID orb_id, uint8_t instance)
	{
		static_assert(sizeof(g_has_publisher[0]) * 8 >= ORB_MULTI_MAX_INSTANCES);
		g_has_publisher[static_cast<uint8_t>(orb_id)] |= (1 << instance);
	}

	void unset_has_publisher(ORB_ID orb_id, uint8_t instance)
	{
		static_assert(sizeof(g_has_publisher[0]) * 8 >= ORB_MULTI_MAX_INSTANCES);
		g_has_publisher[static_cast<uint8_t>(orb_id)] &= ~(1 << instance);
	}

	// Global cache for advertised uORB node instances
	uint16_t g_has_publisher[ORB_TOPICS_COUNT + 1];

	// This (system global) variable is used to pass the subsriber
	// pointer to the callback thread. This is in Manager, since
	// it needs to be mapped for both advertisers and the subscribers
	class SubscriptionCallback *_callback_ptr {nullptr};

	// This mutex protects the above pointer for one writer at a time
	px4_sem_t	_callback_lock;

	void		lock_callbacks() { do {} while (px4_sem_wait(&_callback_lock) != 0); }
	void		unlock_callbacks() { px4_sem_post(&_callback_lock); }

	// A global pool of semaphores for
	// 1) poll locks
	// 2) callback thread signalling (except in NuttX flat build)

	class GlobalSemPool
	{
	public:
		void init();
		int8_t reserve();
		void free(int8_t i);

		void set(int8_t i, int val) {_global_sem[i].set(val);}
		void take(int8_t i) { do {} while (_global_sem[i].take() != 0); }
		int take_interruptible(int8_t i) { return _global_sem[i].take(); }
		int take_timedwait(int8_t i, struct timespec *abstime) { return _global_sem[i].take_timedwait(abstime); }
		void release(int8_t i) {_global_sem[i].release(); }

		class GlobalLock
		{
		public:
			void init()
			{
				px4_sem_init(&_sem, 1, 0);
#if __PX4_NUTTX
				sem_setprotocol(&_sem, SEM_PRIO_NONE);
#endif
				in_use = false;
			}
			void set(int val)
			{
				px4_sem_destroy(&_sem);
				px4_sem_init(&_sem, 1, val);
#if __PX4_NUTTX
				sem_setprotocol(&_sem, SEM_PRIO_NONE);
#endif
			}
			int take() {return px4_sem_wait(&_sem);}
			int take_timedwait(struct timespec *abstime) { return px4_sem_timedwait(&_sem, abstime); }
			void release() {px4_sem_post(&_sem); }

			bool in_use{false};

		private:
			struct SubscriptionCallback *subscriber;
			px4_sem_t _sem;
		};
	private:

		void lock()
		{
			do {} while (px4_sem_wait(&_semLock) != 0);
		}
		void unlock() { px4_sem_post(&_semLock); }

		GlobalLock _global_sem[NUM_GLOBAL_SEMS];
		px4_sem_t _semLock;
	} g_sem_pool;

#ifndef CONFIG_BUILD_FLAT
	static int8_t per_process_lock;
	static pid_t per_process_cb_thread;
#endif
};
} // namespace uORB
