/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * @file px4_nodehandle.h
 *
 * PX4 Middleware Wrapper Node Handle
 */
#pragma once

/* includes for all platforms */
#include "px4_subscriber.h"
#include "px4_publisher.h"
#include "px4_middleware.h"

#if defined(__PX4_ROS)
/* includes when building for ros */
#include "ros/ros.h"
#include <list>
#include <inttypes.h>
#include <type_traits>
#else
/* includes when building for NuttX */
#include <poll.h>
#endif
#include <functional>

namespace px4
{
#if defined(__PX4_ROS)
class NodeHandle :
	private ros::NodeHandle
{
public:
	NodeHandle() :
		ros::NodeHandle(),
		_subs(),
		_pubs()
	{}

	~NodeHandle()
	{
		_subs.clear();
		_pubs.clear();
	};

	/**
	 * Subscribe with callback to function
	 * @param topic		Name of the topic
	 * @param fb		Callback, executed on receiving a new message
	 */
	template<typename T>
	Subscriber<T> *subscribe(void(*fp)(const T &), unsigned interval)
	{
		SubscriberBase *sub = new SubscriberROS<T>((ros::NodeHandle*)this, std::bind(fp, std::placeholders::_1));
		_subs.push_back(sub);
		return (Subscriber<T> *)sub;
	}

	/**
	 * Subscribe with callback to class method
	 * @param fb		Callback, executed on receiving a new message
	 * @param obj	        pointer class instance
	 */
	template<typename T, typename C>
	Subscriber<T> *subscribe(void(C::*fp)(const T &), C *obj, unsigned interval)
	{
		SubscriberBase *sub = new SubscriberROS<T>((ros::NodeHandle*)this, std::bind(fp, obj, std::placeholders::_1));
		_subs.push_back(sub);
		return (Subscriber<T> *)sub;
	}

	/**
	 * Subscribe with no callback, just the latest value is stored on updates
	 */
	template<typename T>
	Subscriber<T> *subscribe(unsigned interval)
	{
		SubscriberBase *sub = new SubscriberROS<T>((ros::NodeHandle*)this);
		_subs.push_back(sub);
		return (Subscriber<T> *)sub;
	}

	/**
	 * Advertise topic
	 */
	template<typename T>
	Publisher<T>* advertise()
	{
		PublisherROS<T> *pub =  new PublisherROS<T>((ros::NodeHandle*)this);
		_pubs.push_back((PublisherBase*)pub);
		return (Publisher<T>*)pub;
	}

	/**
	 * Calls all callback waiting to be called
	 */
	void spinOnce() { ros::spinOnce(); }

	/**
	 * Keeps calling callbacks for incomming messages, returns when module is terminated
	 */
	void spin() { ros::spin(); }


protected:
	std::list<SubscriberBase *> _subs;				/**< Subcriptions of node */
	std::list<PublisherBase *> _pubs;				/**< Publications of node */
};
#else //Building for NuttX
class __EXPORT NodeHandle
{
public:
	NodeHandle() :
		_subs(),
		_pubs(),
		_sub_min_interval(nullptr)
	{}

	~NodeHandle()
	{
		/* Empty subscriptions list */
		SubscriberNode *sub = _subs.getHead();
		int count = 0;

		while (sub != nullptr) {
			if (count++ > kMaxSubscriptions) {
				PX4_WARN("exceeded max subscriptions");
				break;
			}

			SubscriberNode *sib = sub->getSibling();
			delete sub;
			sub = sib;
		}

		/* Empty publications list */
		PublisherNode *pub = _pubs.getHead();
		count = 0;

		while (pub != nullptr) {
			if (count++ > kMaxPublications) {
				PX4_WARN("exceeded max publications");
				break;
			}

			PublisherNode *sib = pub->getSibling();
			delete pub;
			pub = sib;
		}
	};

	/**
	 * Subscribe with callback to function
	 * @param fp		Callback, executed on receiving a new message
	 * @param interval	Minimal interval between calls to callback
	 */

	template<typename T>
	Subscriber<T> *subscribe(void(*fp)(const T &),  unsigned interval)
	{
		(void)interval;
		SubscriberUORBCallback<T> *sub_px4 = new SubscriberUORBCallback<T>(interval, std::bind(fp, std::placeholders::_1));
		update_sub_min_interval(interval, sub_px4);
		_subs.add((SubscriberNode *)sub_px4);
		return (Subscriber<T> *)sub_px4;
	}

	/**
	 * Subscribe with callback to class method
	 * @param fb		Callback, executed on receiving a new message
	 * @param obj	        pointer class instance
	 */
	template<typename T, typename C>
	Subscriber<T> *subscribe(void(C::*fp)(const T &), C *obj, unsigned interval)
	{
		(void)interval;
		SubscriberUORBCallback<T> *sub_px4 = new SubscriberUORBCallback<T>(interval, std::bind(fp, obj, std::placeholders::_1));
		update_sub_min_interval(interval, sub_px4);
		_subs.add((SubscriberNode *)sub_px4);
		return (Subscriber<T> *)sub_px4;
	}

	/**
	 * Subscribe without callback to function
	 * @param interval	Minimal interval between data fetches from orb
	 */

	template<typename T>
	Subscriber<T> *subscribe(unsigned interval)
	{
		(void)interval;
		SubscriberUORB<T> *sub_px4 = new SubscriberUORB<T>(interval);
		update_sub_min_interval(interval, sub_px4);
		_subs.add((SubscriberNode *)sub_px4);
		return (Subscriber<T> *)sub_px4;
	}

	/**
	 * Advertise topic
	 */
	template<typename T>
	Publisher<T> *advertise()
	{
		PublisherUORB<T> *pub = new PublisherUORB<T>();
		_pubs.add(pub);
		return (Publisher<T>*)pub;
	}

	/**
	 * Calls all callback waiting to be called
	 */
	void spinOnce()
	{
		/* Loop through subscriptions, call callback for updated subscriptions */
		SubscriberNode *sub = _subs.getHead();
		int count = 0;

		while (sub != nullptr) {
			if (count++ > kMaxSubscriptions) {
				PX4_WARN("exceeded max subscriptions");
				break;
			}

			sub->update();
			sub = sub->getSibling();
		}
	}

	/**
	 * Keeps calling callbacks for incomming messages, returns when module is terminated
	 */
	void spin()
	{
		while (ok()) {
			const int timeout_ms = 100;

			/* Only continue in the loop if the nodehandle has subscriptions */
			if (_sub_min_interval == nullptr) {
				usleep(timeout_ms * 1000);
				continue;
			}

			/* Poll fd with smallest interval */
			struct pollfd pfd;
			pfd.fd = _sub_min_interval->getUORBHandle();
			pfd.events = POLLIN;
			poll(&pfd, 1, timeout_ms);
			spinOnce();
		}
	}
protected:
	static const uint16_t kMaxSubscriptions = 100;
	static const uint16_t kMaxPublications = 100;
	List<SubscriberNode *> _subs;		/**< Subcriptions of node */
	List<PublisherNode *> _pubs;		/**< Publications of node */
	SubscriberNode *_sub_min_interval;	/**< Points to the sub wtih the smallest interval
							  of all Subscriptions in _subs*/

	/**
	 * Check if this is the smallest interval so far and update _sub_min_interval
	 */
	template<typename T>
	void update_sub_min_interval(unsigned interval, SubscriberUORB<T> *sub)
	{
		if (_sub_min_interval == nullptr || _sub_min_interval->get_interval() > interval) {
			_sub_min_interval = sub;
		}
	}
};
#endif
}
