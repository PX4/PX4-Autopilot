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

#ifndef _uORBManager_hpp_
#define _uORBManager_hpp_

#include "uORBDeviceNode.hpp"
#include "uORBCommon.hpp"
#include "uORBDeviceMaster.hpp"

#include <uORB/topics/uORBTopics.hpp> // For ORB_ID enum
#include <stdint.h>
#include <px4_platform_common/px4_config.h>

#ifdef CONFIG_ORB_COMMUNICATOR
#include "ORBSet.hpp"
#include "uORBCommunicator.hpp"
#endif /* CONFIG_ORB_COMMUNICATOR */

namespace uORB
{
class Manager;
class SubscriptionCallback;
}


/*
 * IOCTLs for manager to access device nodes using
 * a handle
 */

#define _ORBIOCDEV(_n) (_PX4_IOC(_ORBIOCDEVBASE, _n))
#define ORBIOCDEVEXISTS	_ORBIOCDEV(30)
typedef struct orbiocdevexists {
	const ORB_ID orb_id;
	const uint8_t instance;
	const bool check_advertised;
	int ret;
} orbiocdevexists_t;

#define ORBIOCDEVADVERTISE	_ORBIOCDEV(31)
typedef struct orbiocadvertise {
	const struct orb_metadata *meta;
	bool is_advertiser;
	int *instance;
	int ret;
} orbiocdevadvertise_t;

#define ORBIOCDEVUNADVERTISE	_ORBIOCDEV(32)
typedef struct orbiocunadvertise {
	void *handle;
	int ret;
} orbiocdevunadvertise_t;

#define ORBIOCDEVPUBLISH	_ORBIOCDEV(33)
typedef struct orbiocpublish {
	const struct orb_metadata *meta;
	orb_advert_t handle;
	const void *data;
	int ret;
} orbiocdevpublish_t;

#define ORBIOCDEVADDSUBSCRIBER	_ORBIOCDEV(34)
typedef struct {
	const ORB_ID orb_id;
	const uint8_t instance;
	unsigned *initial_generation;
	void *handle;
} orbiocdevaddsubscriber_t;

#define ORBIOCDEVREMSUBSCRIBER	_ORBIOCDEV(35)

#define ORBIOCDEVQUEUESIZE	_ORBIOCDEV(36)
typedef struct {
	const void *handle;
	uint8_t size;
} orbiocdevqueuesize_t;

#define ORBIOCDEVDATACOPY	_ORBIOCDEV(37)
typedef struct {
	void *handle;
	void *dst;
	unsigned generation;
	bool only_if_updated;
	bool ret;
} orbiocdevdatacopy_t;

#define ORBIOCDEVREGCALLBACK	_ORBIOCDEV(38)
typedef struct {
	void *handle;
	class uORB::SubscriptionCallback *callback_sub;
	bool registered;
} orbiocdevregcallback_t;

#define ORBIOCDEVUNREGCALLBACK	_ORBIOCDEV(39)
typedef struct {
	void *handle;
	class uORB::SubscriptionCallback *callback_sub;
} orbiocdevunregcallback_t;

#define ORBIOCDEVGETINSTANCE	_ORBIOCDEV(40)
typedef struct {
	const void *handle;
	uint8_t instance;
} orbiocdevgetinstance_t;

#define ORBIOCDEVUPDATESAVAIL	_ORBIOCDEV(41)
typedef struct {
	const void *handle;
	unsigned last_generation;
	unsigned ret;
} orbiocdevupdatesavail_t;

#define ORBIOCDEVISADVERTISED	_ORBIOCDEV(42)
typedef struct {
	const void *handle;
	bool ret;
} orbiocdevisadvertised_t;

typedef enum {
	ORB_DEVMASTER_STATUS = 0,
	ORB_DEVMASTER_TOP = 1
} orbiocdevmastercmd_t;
#define ORBIOCDEVMASTERCMD	_ORBIOCDEV(45)


/**
 * This is implemented as a singleton.  This class manages creating the
 * uORB nodes for each uORB topics and also implements the behavor of the
 * uORB Api's.
 */
class uORB::Manager
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
	 * Make sure initialize() is called first.
	 * @return uORB::Manager*
	 */
	static uORB::Manager *get_instance() { return _Instance; }

	/**
	 * Get the DeviceMaster. If it does not exist,
	 * it will be created and initialized.
	 * Note: the first call to this is not thread-safe.
	 * @return nullptr if initialization failed (and errno will be set)
	 */
	uORB::DeviceMaster *get_device_master();

#if defined(__PX4_NUTTX) && !defined(CONFIG_BUILD_FLAT) && defined(__KERNEL__)
	static int orb_ioctl(unsigned int cmd, unsigned long arg);
#endif

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
	 * @return    nullptr on error, otherwise returns an object pointer
	 *      that can be used to publish to the topic.
	 *      If the topic in question is not known (due to an
	 *      ORB_DEFINE with no corresponding ORB_DECLARE)
	 *      this function will return nullptr and set errno to ENOENT.
	 */
	orb_advert_t orb_advertise(const struct orb_metadata *meta, const void *data = nullptr)
	{
		return orb_advertise_multi(meta, data, nullptr);
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
	 * @return    nullptr on error, otherwise returns a handle
	 *      that can be used to publish to the topic.
	 *      If the topic in question is not known (due to an
	 *      ORB_DEFINE with no corresponding ORB_DECLARE)
	 *      this function will return nullptr and set errno to ENOENT.
	 */
	orb_advert_t orb_advertise_multi(const struct orb_metadata *meta, const void *data, int *instance);

	/**
	 * Unadvertise a topic.
	 *
	 * @param handle  handle returned by orb_advertise or orb_advertise_multi.
	 * @return 0 on success
	 */
	static int orb_unadvertise(orb_advert_t handle);

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
	static int  orb_publish(const struct orb_metadata *meta, orb_advert_t handle, const void *data);

	/**
	 * Subscribe to a topic.
	 *
	 * The returned value is a file descriptor that can be passed to poll()
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
	int  orb_subscribe(const struct orb_metadata *meta);

	/**
	 * Subscribe to a multi-instance of a topic.
	 *
	 * The returned value is a file descriptor that can be passed to poll()
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
	 * @return    PX4_ERROR on error, otherwise returns a handle
	 *      that can be used to read and update the topic.
	 *      If the topic in question is not known (due to an
	 *      ORB_DEFINE_OPTIONAL with no corresponding ORB_DECLARE)
	 *      this function will return -1 and set errno to ENOENT.
	 */
	int  orb_subscribe_multi(const struct orb_metadata *meta, unsigned instance);

	/**
	 * Unsubscribe from a topic.
	 *
	 * @param handle  A handle returned from orb_subscribe.
	 * @return    OK on success, PX4_ERROR otherwise with errno set accordingly.
	 */
	int  orb_unsubscribe(int handle);

	/**
	 * Fetch data from a topic.
	 *
	 * This is the only operation that will reset the internal marker that
	 * indicates that a topic has been updated for a subscriber. Once poll
	 * or check return indicating that an updaet is available, this call
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
	int  orb_copy(const struct orb_metadata *meta, int handle, void *buffer);

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
	int  orb_check(int handle, bool *updated);

	/**
	 * Check if a topic has already been created and published (advertised)
	 *
	 * @param meta    ORB topic metadata.
	 * @param instance  ORB instance
	 * @return    OK if the topic exists, PX4_ERROR otherwise.
	 */
	int  orb_exists(const struct orb_metadata *meta, int instance);

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
	int  orb_set_interval(int handle, unsigned interval);


	/**
	 * Get the minimum interval between which updates are seen for a subscription.
	 *
	 * @see orb_set_interval()
	 *
	 * @param handle  A handle returned from orb_subscribe.
	 * @param interval  The returned interval period in milliseconds.
	 * @return    OK on success, PX4_ERROR otherwise with ERRNO set accordingly.
	 */
	int	orb_get_interval(int handle, unsigned *interval);

	static bool orb_device_node_exists(ORB_ID orb_id, uint8_t instance);

	static void *orb_add_internal_subscriber(ORB_ID orb_id, uint8_t instance, unsigned *initial_generation);

	static void orb_remove_internal_subscriber(void *node_handle);

	static uint8_t orb_get_queue_size(const void *node_handle);

	static int8_t orb_get_subscriber_count(const void *node_handle);

	static bool orb_data_copy(void *node_handle, void *dst, unsigned &generation, bool only_if_updated);

	static bool register_callback(void *node_handle, SubscriptionCallback *callback_sub);

	static void unregister_callback(void *node_handle, SubscriptionCallback *callback_sub);

	static uint8_t orb_get_instance(const void *node_handle);

#if defined(CONFIG_BUILD_FLAT)
	/* These are optimized by inlining in NuttX Flat build */
	static unsigned updates_available(const void *node_handle, unsigned last_generation) { return is_advertised(node_handle) ? static_cast<const DeviceNode *>(node_handle)->updates_available(last_generation) : 0; }

	static bool is_advertised(const void *node_handle) { return static_cast<const DeviceNode *>(node_handle)->is_advertised(); }

#else
	static unsigned updates_available(const void *node_handle, unsigned last_generation);

	static bool is_advertised(const void *node_handle);
#endif

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

private: // class methods

	/**
	 * Common implementation for orb_advertise and orb_subscribe.
	 *
	 * Handles creation of the object and the initial publication for
	 * advertisers.
	 */
	int node_open(const struct orb_metadata *meta, bool advertiser, int *instance = nullptr);

private: // data members
	static Manager *_Instance;

#ifdef CONFIG_ORB_COMMUNICATOR
	// the communicator channel instance.
	uORBCommunicator::IChannel *_comm_channel{nullptr};
	static pthread_mutex_t _communicator_mutex;

	// Track the advertisements we get from the remote side
	ORBSet _remote_topics;
#endif /* CONFIG_ORB_COMMUNICATOR */

	DeviceMaster *_device_master{nullptr};

private: //class methods
	Manager();
	virtual ~Manager();

#ifdef CONFIG_ORB_COMMUNICATOR
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

};

#endif /* _uORBManager_hpp_ */
