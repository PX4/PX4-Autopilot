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

#pragma once

#include <stdint.h>

#include <uORB/uORBCommon.hpp>
#include <uORB/topics/uORBTopics.hpp>

#include <px4_platform_common/posix.h>

namespace uORB
{
class DeviceNode;
class DeviceMaster;
class Manager;
}

#include <string.h>
#include <stdlib.h>

#include <containers/IntrusiveSortedList.hpp>
#include <px4_platform_common/atomic_bitset.h>

using px4::AtomicBitset;

/**
 * Master control device for ObjDev.
 *
 * Used primarily to create new objects via the ORBIOCCREATE
 * ioctl.
 */
class uORB::DeviceMaster
{
public:
	DeviceMaster();
	~DeviceMaster();

	/**
	 * Print statistics for each existing topic.
	 */
	void printStatistics();

	/**
	 * Continuously print statistics, like the unix top command for processes.
	 * Exited when the user presses the enter key.
	 * @param topic_filter list of topic filters: if set, each string can be a substring for topics to match.
	 *        Or it can be '-a', which means to print all topics instead of only ones currently publishing with subscribers.
	 * @param num_filters
	 */
	void showTop(char **topic_filter, int num_filters);

private:

	struct DeviceNodeStatisticsData {
		DeviceNode *node;
		unsigned int last_pub_msg_count;
		unsigned int pub_msg_delta;
		DeviceNodeStatisticsData *next = nullptr;
	};

	int addNewDeviceNodes(DeviceNodeStatisticsData **first_node, int &num_topics, size_t &max_topic_name_length,
			      char **topic_filter, int num_filters);

	friend class uORB::Manager;

	px4_sem_t	_lock; /**< lock to protect access to all class members (also for derived classes) */

	void		lock() { do {} while (px4_sem_wait(&_lock) != 0); }
	void		unlock() { px4_sem_post(&_lock); }
};
