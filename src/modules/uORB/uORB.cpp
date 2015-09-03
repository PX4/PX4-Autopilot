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

/**
 * @file uORB.cpp
 * A lightweight object broker.
 */

#include "uORB.h"
#include "uORBManager.hpp"
#include "uORBCommon.hpp"

/**
 * Advertise as the publisher of a topic.
 *
 * This performs the initial advertisement of a topic; it creates the topic
 * node in /obj if required and publishes the initial data.
 *
 * Any number of advertisers may publish to a topic; publications are atomic
 * but co-ordination between publishers is not provided by the ORB.
 *
 * @param meta    The uORB metadata (usually from the ORB_ID() macro)
 *      for the topic.
 * @param data    A pointer to the initial data to be published.
 *      For topics updated by interrupt handlers, the advertisement
 *      must be performed from non-interrupt context.
 * @return    ERROR on error, otherwise returns a handle
 *      that can be used to publish to the topic.
 *      If the topic in question is not known (due to an
 *      ORB_DEFINE with no corresponding ORB_DECLARE)
 *      this function will return -1 and set errno to ENOENT.
 */
orb_advert_t orb_advertise(const struct orb_metadata *meta, const void *data)
{
	return uORB::Manager::get_instance()->orb_advertise(meta, data);
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
 * @param meta    The uORB metadata (usually from the ORB_ID() macro)
 *      for the topic.
 * @param data    A pointer to the initial data to be published.
 *      For topics updated by interrupt handlers, the advertisement
 *      must be performed from non-interrupt context.
 * @param instance  Pointer to an integer which will yield the instance ID (0-based)
 *      of the publication.
 * @param priority  The priority of the instance. If a subscriber subscribes multiple
 *      instances, the priority allows the subscriber to prioritize the best
 *      data source as long as its available.
 * @return    ERROR on error, otherwise returns a handle
 *      that can be used to publish to the topic.
 *      If the topic in question is not known (due to an
 *      ORB_DEFINE with no corresponding ORB_DECLARE)
 *      this function will return -1 and set errno to ENOENT.
 */
orb_advert_t orb_advertise_multi(const struct orb_metadata *meta, const void *data, int *instance,
				 int priority)
{
	return uORB::Manager::get_instance()->orb_advertise_multi(meta, data, instance, priority);
}


/**
 * Publish new data to a topic.
 *
 * The data is atomically published to the topic and any waiting subscribers
 * will be notified.  Subscribers that are not waiting can check the topic
 * for updates using orb_check and/or orb_stat.
 *
 * @param meta    The uORB metadata (usually from the ORB_ID() macro)
 *      for the topic.
 * @handle    The handle returned from orb_advertise.
 * @param data    A pointer to the data to be published.
 * @return    OK on success, ERROR otherwise with errno set accordingly.
 */
int  orb_publish(const struct orb_metadata *meta, orb_advert_t handle, const void *data)
{
	return uORB::Manager::get_instance()->orb_publish(meta, handle, data);
}

/**
 * Subscribe to a topic.
 *
 * The returned value is a file descriptor that can be passed to poll()
 * in order to wait for updates to a topic, as well as topic_read,
 * orb_check and orb_stat.
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
 * @param meta    The uORB metadata (usually from the ORB_ID() macro)
 *      for the topic.
 * @return    ERROR on error, otherwise returns a handle
 *      that can be used to read and update the topic.
 *      If the topic in question is not known (due to an
 *      ORB_DEFINE_OPTIONAL with no corresponding ORB_DECLARE)
 *      this function will return -1 and set errno to ENOENT.
 */
int  orb_subscribe(const struct orb_metadata *meta)
{
	return uORB::Manager::get_instance()->orb_subscribe(meta);
}

/**
 * Subscribe to a multi-instance of a topic.
 *
 * The returned value is a file descriptor that can be passed to poll()
 * in order to wait for updates to a topic, as well as topic_read,
 * orb_check and orb_stat.
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
 * @param meta    The uORB metadata (usually from the ORB_ID() macro)
 *      for the topic.
 * @param instance  The instance of the topic. Instance 0 matches the
 *      topic of the orb_subscribe() call, higher indices
 *      are for topics created with orb_publish_multi().
 * @return    ERROR on error, otherwise returns a handle
 *      that can be used to read and update the topic.
 *      If the topic in question is not known (due to an
 *      ORB_DEFINE_OPTIONAL with no corresponding ORB_DECLARE)
 *      this function will return -1 and set errno to ENOENT.
 */
int  orb_subscribe_multi(const struct orb_metadata *meta, unsigned instance)
{
	return uORB::Manager::get_instance()->orb_subscribe_multi(meta, instance);
}

/**
 * Unsubscribe from a topic.
 *
 * @param handle  A handle returned from orb_subscribe.
 * @return    OK on success, ERROR otherwise with errno set accordingly.
 */
int  orb_unsubscribe(int handle)
{
	return uORB::Manager::get_instance()->orb_unsubscribe(handle);
}

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
 * @return    OK on success, ERROR otherwise with errno set accordingly.
 */
int  orb_copy(const struct orb_metadata *meta, int handle, void *buffer)
{
	return uORB::Manager::get_instance()->orb_copy(meta, handle, buffer);
}

/**
 * Check whether a topic has been published to since the last orb_copy.
 *
 * This check can be used to determine whether to copy the topic when
 * not using poll(), or to avoid the overhead of calling poll() when the
 * topic is likely to have updated.
 *
 * Updates are tracked on a per-handle basis; this call will continue to
 * return true until orb_copy is called using the same handle. This interface
 * should be preferred over calling orb_stat due to the race window between
 * stat and copy that can lead to missed updates.
 *
 * @param handle  A handle returned from orb_subscribe.
 * @param updated Set to true if the topic has been updated since the
 *      last time it was copied using this handle.
 * @return    OK if the check was successful, ERROR otherwise with
 *      errno set accordingly.
 */
int  orb_check(int handle, bool *updated)
{
	return uORB::Manager::get_instance()->orb_check(handle, updated);
}

/**
 * Return the last time that the topic was updated.
 *
 * @param handle  A handle returned from orb_subscribe.
 * @param time    Returns the absolute time that the topic was updated, or zero if it has
 *      never been updated. Time is measured in microseconds.
 * @return    OK on success, ERROR otherwise with errno set accordingly.
 */
int  orb_stat(int handle, uint64_t *time)
{
	return uORB::Manager::get_instance()->orb_stat(handle, time);
}

/**
 * Check if a topic has already been created.
 *
 * @param meta    ORB topic metadata.
 * @param instance  ORB instance
 * @return    OK if the topic exists, ERROR otherwise with errno set accordingly.
 */
int  orb_exists(const struct orb_metadata *meta, int instance)
{
	return uORB::Manager::get_instance()->orb_exists(meta, instance);
}

/**
 * Get the number of published instances of a topic group
 *
 * @param meta    ORB topic metadata.
 * @return    The number of published instances of this topic
 */
int  orb_group_count(const struct orb_metadata *meta)
{
	unsigned group_count = 0;
	while (!uORB::Manager::get_instance()->orb_exists(meta, group_count++)) {};

	return group_count;
}

/**
 * Return the priority of the topic
 *
 * @param handle  A handle returned from orb_subscribe.
 * @param priority  Returns the priority of this topic. This is only relevant for
 *      topics which are published by multiple publishers (e.g. mag0, mag1, etc.)
 *      and allows a subscriber to automatically pick the topic with the highest
 *      priority, independent of the startup order of the associated publishers.
 * @return    OK on success, ERROR otherwise with errno set accordingly.
 */
int  orb_priority(int handle, int32_t *priority)
{
	return uORB::Manager::get_instance()->orb_priority(handle, priority);
}

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
 * @return    OK on success, ERROR otherwise with ERRNO set accordingly.
 */
int  orb_set_interval(int handle, unsigned interval)
{
	return uORB::Manager::get_instance()->orb_set_interval(handle, interval);
}

