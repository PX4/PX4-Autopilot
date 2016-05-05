/****************************************************************************
 *
 * Copyright (c) 2015 Mark Charlebois. All rights reserved.
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
 * @file vdev_posix.cpp
 *
 * POSIX-like API for virtual character device
 */

#include <px4_log.h>
#include <px4_posix.h>
#include <px4_time.h>
#include "device.h"
#include "vfile.h"

#include <hrt_work.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>

using namespace device;

pthread_mutex_t filemutex = PTHREAD_MUTEX_INITIALIZER;
px4_sem_t lockstep_sem;
bool sim_lockstep = false;
bool sim_delay = false;

extern "C" {

#define PX4_MAX_FD 300
	static device::file_t *filemap[PX4_MAX_FD] = {};

	int px4_errno;

	inline bool valid_fd(int fd)
	{
		pthread_mutex_lock(&filemutex);
		bool ret = (fd < PX4_MAX_FD && fd >= 0 && filemap[fd] != NULL);
		pthread_mutex_unlock(&filemutex);
		return ret;
	}

	inline VDev *get_vdev(int fd)
	{
		pthread_mutex_lock(&filemutex);
		bool valid = (fd < PX4_MAX_FD && fd >= 0 && filemap[fd] != NULL);
		VDev *dev;

		if (valid) {
			dev = (VDev *)(filemap[fd]->vdev);

		} else {
			dev = nullptr;
		}

		pthread_mutex_unlock(&filemutex);
		return dev;
	}

	int px4_open(const char *path, int flags, ...)
	{
		PX4_DEBUG("px4_open");
		VDev *dev = VDev::getDev(path);
		int ret = 0;
		int i;
		mode_t mode;

		if (!dev && (flags & (PX4_F_WRONLY | PX4_F_CREAT)) != 0 &&
		    strncmp(path, "/obj/", 5) != 0 &&
		    strncmp(path, "/dev/", 5) != 0) {
			va_list p;
			va_start(p, flags);
			mode = va_arg(p, int);
			va_end(p);

			// Create the file
			PX4_DEBUG("Creating virtual file %s", path);
			dev = VFile::createFile(path, mode);
		}

		if (dev) {

			pthread_mutex_lock(&filemutex);

			for (i = 0; i < PX4_MAX_FD; ++i) {
				if (filemap[i] == 0) {
					filemap[i] = new device::file_t(flags, dev, i);
					break;
				}
			}

			pthread_mutex_unlock(&filemutex);

			if (i < PX4_MAX_FD) {
				ret = dev->open(filemap[i]);

			} else {

				const unsigned NAMELEN = 32;
				char thread_name[NAMELEN] = {};

#ifndef __PX4_QURT
				int nret = pthread_getname_np(pthread_self(), thread_name, NAMELEN);

				if (nret || thread_name[0] == 0) {
					PX4_WARN("failed getting thread name");
				}

				PX4_BACKTRACE();
#endif

				PX4_WARN("%s: exceeded maximum number of file descriptors, accessing %s",
					 thread_name, path);
				ret = -ENOENT;
			}

		} else {
			ret = -EINVAL;
		}

		if (ret < 0) {
			px4_errno = -ret;
			return -1;
		}

		PX4_DEBUG("px4_open fd = %d", filemap[i]->fd);
		return filemap[i]->fd;
	}

	int px4_close(int fd)
	{
		int ret;

		VDev *dev = get_vdev(fd);

		if (dev) {
			pthread_mutex_lock(&filemutex);
			ret = dev->close(filemap[fd]);
			filemap[fd] = nullptr;
			pthread_mutex_unlock(&filemutex);
			PX4_DEBUG("px4_close fd = %d", fd);

		} else {
			ret = -EINVAL;
		}

		if (ret < 0) {
			px4_errno = -ret;
			ret = PX4_ERROR;
		}

		return ret;
	}

	ssize_t px4_read(int fd, void *buffer, size_t buflen)
	{
		int ret;

		VDev *dev = get_vdev(fd);

		if (dev) {
			PX4_DEBUG("px4_read fd = %d", fd);
			ret = dev->read(filemap[fd], (char *)buffer, buflen);

		} else {
			ret = -EINVAL;
		}

		if (ret < 0) {
			px4_errno = -ret;
			ret = PX4_ERROR;
		}

		return ret;
	}

	ssize_t px4_write(int fd, const void *buffer, size_t buflen)
	{
		int ret;

		VDev *dev = get_vdev(fd);

		if (dev) {
			PX4_DEBUG("px4_write fd = %d", fd);
			ret = dev->write(filemap[fd], (const char *)buffer, buflen);

		} else {
			ret = -EINVAL;
		}

		if (ret < 0) {
			px4_errno = -ret;
			ret = PX4_ERROR;
		}

		return ret;
	}

	int px4_ioctl(int fd, int cmd, unsigned long arg)
	{
		PX4_DEBUG("px4_ioctl fd = %d", fd);
		int ret = 0;

		VDev *dev = get_vdev(fd);

		if (dev) {
			ret = dev->ioctl(filemap[fd], cmd, arg);

		} else {
			ret = -EINVAL;
		}

		if (ret < 0) {
			px4_errno = -ret;
		}

		return ret;
	}

	int px4_poll(px4_pollfd_struct_t *fds, nfds_t nfds, int timeout)
	{
		if (nfds == 0) {
			PX4_WARN("px4_poll with no fds");
			return -1;
		}

		px4_sem_t sem;
		int count = 0;
		int ret = -1;
		unsigned int i;

		const unsigned NAMELEN = 32;
		char thread_name[NAMELEN] = {};

#ifndef __PX4_QURT
		int nret = pthread_getname_np(pthread_self(), thread_name, NAMELEN);

		if (nret || thread_name[0] == 0) {
			PX4_WARN("failed getting thread name");
		}

#endif

		while (sim_delay) {
			usleep(100);
		}

		PX4_DEBUG("Called px4_poll timeout = %d", timeout);
		px4_sem_init(&sem, 0, 0);

		// Go through all fds and check them for a pollable state
		bool fd_pollable = false;

		for (i = 0; i < nfds; ++i) {
			fds[i].sem     = &sem;
			fds[i].revents = 0;
			fds[i].priv    = NULL;

			VDev *dev = get_vdev(fds[i].fd);

			// If fd is valid
			if (dev) {
				PX4_DEBUG("%s: px4_poll: VDev->poll(setup) %d", thread_name, fds[i].fd);
				ret = dev->poll(filemap[fds[i].fd], &fds[i], true);

				if (ret < 0) {
					PX4_WARN("%s: px4_poll() error: %s",
						 thread_name, strerror(errno));
					break;
				}

				if (ret >= 0) {
					fd_pollable = true;
				}
			}
		}

		// If any FD can be polled, lock the semaphore and
		// check for new data
		if (fd_pollable) {
			if (timeout > 0) {

				// Get the current time
				struct timespec ts;
				// FIXME: check if QURT should probably be using CLOCK_MONOTONIC
				px4_clock_gettime(CLOCK_REALTIME, &ts);

				// Calculate an absolute time in the future
				const unsigned billion = (1000 * 1000 * 1000);
				unsigned tdiff = timeout;
				uint64_t nsecs = ts.tv_nsec + (tdiff * 1000 * 1000);
				ts.tv_sec += nsecs / billion;
				nsecs -= (nsecs / billion) * billion;
				ts.tv_nsec = nsecs;

				// Execute a blocking wait for that time in the future
				errno = 0;
				ret = px4_sem_timedwait(&sem, &ts);
#ifndef __PX4_DARWIN
				ret = errno;
#endif

				// Ensure ret is negative on failure
				if (ret > 0) {
					ret = -ret;
				}

				if (ret && ret != -ETIMEDOUT) {
					PX4_WARN("%s: px4_poll() sem error", thread_name);
				}

			} else if (timeout < 0) {
				px4_sem_wait(&sem);
			}

			// We have waited now (or not, depending on timeout),
			// go through all fds and count how many have data
			for (i = 0; i < nfds; ++i) {

				VDev *dev = get_vdev(fds[i].fd);

				// If fd is valid
				if (dev) {
					PX4_DEBUG("%s: px4_poll: VDev->poll(teardown) %d", thread_name, fds[i].fd);
					ret = dev->poll(filemap[fds[i].fd], &fds[i], false);

					if (ret < 0) {
						PX4_WARN("%s: px4_poll() 2nd poll fail", thread_name);
						break;
					}

					if (fds[i].revents) {
						count += 1;
					}
				}
			}
		}

		px4_sem_destroy(&sem);

		// Return the positive count if present,
		// return the negative error number if failed
		return (count) ? count : ret;
	}

	int px4_fsync(int fd)
	{
		return 0;
	}

	int px4_access(const char *pathname, int mode)
	{
		if (mode != F_OK) {
			errno = EINVAL;
			return -1;
		}

		VDev *dev = VDev::getDev(pathname);
		return (dev != nullptr) ? 0 : -1;
	}

	void px4_show_devices()
	{
		VDev::showDevices();
	}

	void px4_show_topics()
	{
		VDev::showTopics();
	}

	void px4_show_files()
	{
		VDev::showFiles();
	}

	void px4_enable_sim_lockstep()
	{
		px4_sem_init(&lockstep_sem, 0, 0);
		sim_lockstep = true;
		sim_delay = false;
	}

	void px4_sim_start_delay()
	{
		sim_delay = true;
	}

	void px4_sim_stop_delay()
	{
		sim_delay = false;
	}

	bool px4_sim_delay_enabled()
	{
		return sim_delay;
	}

	const char *px4_get_device_names(unsigned int *handle)
	{
		return VDev::devList(handle);
	}

	const char *px4_get_topic_names(unsigned int *handle)
	{
		return VDev::topicList(handle);
	}

}

