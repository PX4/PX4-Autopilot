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

#include "uORBDeviceMaster.hpp"
#include <uORB/uORBDeviceNode.hpp>
#include <uORB/uORBManager.hpp>
#include <uORB/uORBUtils.hpp>

#include <px4_platform_common/mmap.h>
#include <px4_platform_common/sem.hpp>
#include <systemlib/px4_macros.h>

#include <math.h>

#ifndef __PX4_QURT // QuRT has no poll()
#include <poll.h>
#endif // PX4_QURT

#ifndef CONFIG_FS_SHMFS_VFS_PATH
#define CONFIG_FS_SHMFS_VFS_PATH "/dev/shm"
#endif

#include <dirent.h>

uORB::DeviceMaster::DeviceMaster()
{
	px4_sem_init(&_lock, 0, 1);
}

uORB::DeviceMaster::~DeviceMaster()
{
	px4_sem_destroy(&_lock);
}

void uORB::DeviceMaster::printStatistics()
{
	/* Add all nodes to a list while locked, and then print them in unlocked state, to avoid potential
	 * dead-locks (where printing blocks) */
	lock();
	DeviceNodeStatisticsData *first_node = nullptr;
	DeviceNodeStatisticsData *cur_node = nullptr;
	size_t max_topic_name_length = 0;
	int num_topics = 0;
	int ret = addNewDeviceNodes(&first_node, num_topics, max_topic_name_length, nullptr, 0);
	unlock();

	if (ret != 0) {
		PX4_ERR("addNewDeviceNodes failed (%i)", ret);
		return;
	}

	PX4_INFO_RAW("%-*s INST #SUB #Q SIZE PATH\n", (int)max_topic_name_length - 2, "TOPIC NAME");

	cur_node = first_node;

	while (cur_node) {
		cur_node->node->print_statistics(max_topic_name_length);

		DeviceNodeStatisticsData *prev = cur_node;
		cur_node = cur_node->next;
		px4_munmap(prev->node, sizeof(uORB::DeviceNode));
		delete prev;
	}
}

int uORB::DeviceMaster::addNewDeviceNodes(DeviceNodeStatisticsData **first_node, int &num_topics,
		size_t &max_topic_name_length, char **topic_filter, int num_filters)
{
	DeviceNodeStatisticsData *cur_node = nullptr;
	num_topics = 0;
	DeviceNodeStatisticsData *last_node = *first_node;

	if (last_node) {
		while (last_node->next) {
			last_node = last_node->next;
		}
	}

	DIR *shm_dir = opendir(CONFIG_FS_SHMFS_VFS_PATH);
	struct dirent *shm;
	const char orb_name_prefix[] = "_orb_";
	char orb_name[orb_maxpath];

	while ((shm = readdir(shm_dir)) != nullptr) {

		if (strncmp(orb_name_prefix, shm->d_name, sizeof(orb_name_prefix) - 1)) {
			continue;
		}

		// check if already added
		cur_node = *first_node;

		while (cur_node) {
			int instance = cur_node->node->get_instance();

			if (uORB::Utils::node_mkpath(orb_name, cur_node->node->get_meta(), &instance)) {
				PX4_ERR("Can't construct orb name?");
				break;
			}

			if (!strcmp(orb_name, shm->d_name)) {
				break;
			}

			cur_node = cur_node->next;
		}

		if (cur_node) {
			// already added or there was an error
			continue;
		}

		void *ptr = nullptr;
		uORB::DeviceNode *node = nullptr;

		// open and mmap the shared memory segment
		int shm_fd = shm_open(shm->d_name, O_RDWR, 0666);

		if (shm_fd >= 0) {
			ptr = px4_mmap(0, sizeof(uORB::DeviceNode), PROT_READ, MAP_SHARED, shm_fd, 0);
		}

		if (ptr != MAP_FAILED) {
			node = static_cast<uORB::DeviceNode *>(ptr);
		}

		close(shm_fd);

		if (node == nullptr) {
			PX4_ERR("Failed to MMAP an existing node\n");
			continue;
		}

		++num_topics;

		if (num_filters > 0 && topic_filter) {
			bool matched = false;

			for (int i = 0; i < num_filters; ++i) {
				if (strstr(node->get_name(), topic_filter[i])) {
					matched = true;
				}
			}

			if (!matched) {
				px4_munmap(node, sizeof(uORB::DeviceNode));
				continue;
			}
		}

		if (last_node) {
			last_node->next = new DeviceNodeStatisticsData();
			last_node = last_node->next;

		} else {
			*first_node = last_node = new DeviceNodeStatisticsData();
		}

		if (!last_node) {
			return -ENOMEM;
		}

		last_node->node = node;

		size_t name_length = strlen(last_node->node->get_meta()->o_name);

		if (name_length > max_topic_name_length) {
			max_topic_name_length = name_length;
		}

		// Pass in 0 to get the index of the latest published data
		last_node->last_pub_msg_count = last_node->node->updates_available(0);
	}

	closedir(shm_dir);
	return 0;
}

#define CLEAR_LINE "\033[K"

void uORB::DeviceMaster::showTop(char **topic_filter, int num_filters)
{
	bool print_active_only = true;
	bool only_once = false; // if true, run only once, then exit

	if (topic_filter && num_filters > 0) {
		bool show_all = false;

		for (int i = 0; i < num_filters; ++i) {
			if (!strcmp("-a", topic_filter[i])) {
				show_all = true;

			} else if (!strcmp("-1", topic_filter[i])) {
				only_once = true;
			}
		}

		print_active_only = only_once ? (num_filters == 1) : false; // print non-active if -a or some filter given

		if (show_all || print_active_only) {
			num_filters = 0;
		}
	}

	PX4_INFO_RAW("\033[2J\n"); //clear screen

	lock();

	DeviceNodeStatisticsData *first_node = nullptr;
	DeviceNodeStatisticsData *cur_node = nullptr;
	size_t max_topic_name_length = 0;
	int num_topics = 0;
	int ret = addNewDeviceNodes(&first_node, num_topics, max_topic_name_length, topic_filter, num_filters);

	/* a DeviceNode is never deleted, so it's save to unlock here and still access the DeviceNodes */
	unlock();

	if (ret != 0) {
		PX4_ERR("addNewDeviceNodes failed (%i)", ret);
	}

#ifdef __PX4_QURT // QuRT has no poll()
	only_once = true;
#else
	const int stdin_fileno = 0;

	struct pollfd fds;
	fds.fd = stdin_fileno;
	fds.events = POLLIN;
#endif
	bool quit = false;

	hrt_abstime start_time = hrt_absolute_time();

	while (!quit) {

#ifndef __PX4_QURT

		if (!only_once) {
			/* Sleep 200 ms waiting for user input five times ~ 1.4s */
			for (int k = 0; k < 7; k++) {
				char c;

				ret = ::poll(&fds, 1, 0); //just want to check if there is new data available

				if (ret > 0) {

					ret = ::read(stdin_fileno, &c, 1);

					if (ret) {
						quit = true;
						break;
					}
				}

				px4_usleep(200000);
			}

		} else {
			px4_usleep(2000000); // 2 seconds
		}

#endif

		if (!quit) {

			// update the stats
			int total_size = 0;
			int total_msgs = 0;
			hrt_abstime current_time = hrt_absolute_time();
			float dt = (current_time - start_time) / 1.e6f;
			cur_node = first_node;

			while (cur_node) {
				unsigned int num_msgs = cur_node->node->updates_available(cur_node->last_pub_msg_count);
				cur_node->pub_msg_delta = roundf(num_msgs / dt);
				cur_node->last_pub_msg_count += num_msgs;

				total_size += cur_node->pub_msg_delta * cur_node->node->get_meta()->o_size;
				total_msgs += cur_node->pub_msg_delta;

				cur_node = cur_node->next;
			}

			start_time = current_time;


			if (!only_once) {
				PX4_INFO_RAW("\033[H"); // move cursor to top left corner
			}

			PX4_INFO_RAW(CLEAR_LINE "update: 1s, topics: %i, total publications: %i, %.1f kB/s\n",
				     num_topics, total_msgs, (double)(total_size / 1000.f));
			PX4_INFO_RAW(CLEAR_LINE "%-*s INST #SUB RATE #Q SIZE\n", (int)max_topic_name_length - 2, "TOPIC NAME");
			cur_node = first_node;

			while (cur_node) {

				if (!print_active_only || (cur_node->pub_msg_delta > 0 && cur_node->node->subscriber_count() > 0)) {
					PX4_INFO_RAW(CLEAR_LINE "%-*s %2i %4i %4i %2i %4i \n", (int)max_topic_name_length,
						     cur_node->node->get_meta()->o_name, (int)cur_node->node->get_instance(),
						     (int)cur_node->node->subscriber_count(), cur_node->pub_msg_delta,
						     cur_node->node->get_queue_size(), cur_node->node->get_meta()->o_size);
				}

				cur_node = cur_node->next;
			}


			if (!only_once) {
				PX4_INFO_RAW("\033[0J"); // clear the rest of the screen
			}

			lock();
			ret = addNewDeviceNodes(&first_node, num_topics, max_topic_name_length, topic_filter, num_filters);
			unlock();

			if (ret != 0) {
				PX4_ERR("addNewDeviceNodes failed (%i)", ret);
			}

		}

		if (only_once) {
			quit = true;
		}
	}

	//cleanup
	cur_node = first_node;

	while (cur_node) {
		DeviceNodeStatisticsData *next_node = cur_node->next;
		px4_munmap(cur_node->node, sizeof(uORB::DeviceNode));
		delete (cur_node);
		cur_node = next_node;
	}
}
