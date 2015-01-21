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
#if defined(__PX4_ROS)
/* includes when building for ros */
#include "ros/ros.h"
#else
/* includes when building for NuttX */
#include <uORB/Publication.hpp>
#include <containers/List.hpp>
#endif

#include <platforms/px4_message.h>

namespace px4
{

/**
 * Untemplated publisher base class
 * */
class __EXPORT PublisherBase
{
public:
	PublisherBase() {};
	~PublisherBase() {};
};

#if defined(__PX4_ROS)
class Publisher :
	public PublisherBase
{
public:
	/**
	 * Construct Publisher by providing a ros::Publisher
	 * @param ros_pub the ros publisher which will be used to perform the publications
	 */
	Publisher(ros::Publisher ros_pub) :
		PublisherBase(),
		_ros_pub(ros_pub)
	{}

	~Publisher() {};

	/** Publishes msg
	 * @param msg	    the message which is published to the topic
	 */
	template <typename T>
	int publish(T &msg) {
		_ros_pub.publish(msg.data());
		return 0;}
private:
	ros::Publisher _ros_pub;	/**< Handle to the ros publisher */
};
#else
class __EXPORT Publisher :
	public PublisherBase,
	public ListNode<Publisher *>

{
public:
	/**
	 * Construct Publisher by providing orb meta data
	 */
	Publisher(uORB::PublicationBase * uorb_pub) :
		PublisherBase(),
		_uorb_pub(uorb_pub)
	{}

	~Publisher() {};

	/** Publishes msg
	 * @param msg	    the message which is published to the topic
	 */
	template<typename T>
	int publish(const T &msg)
	{
		_uorb_pub->update((void *)&(msg.data()));
		return 0;
	}

	/**
	 * Empty callback for list traversal
	 */
	void update() {} ;
private:
	uORB::PublicationBase * _uorb_pub;	/**< Handle to the publisher */

};
#endif
}
