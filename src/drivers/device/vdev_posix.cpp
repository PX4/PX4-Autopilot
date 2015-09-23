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

extern "C" {

static void timer_cb(void *data)
{
	sem_t *p_sem = (sem_t *)data;
	sem_post(p_sem);
	PX4_DEBUG("timer_handler: Timer expired");
}

#define PX4_MAX_FD 200
static device::file_t *filemap[PX4_MAX_FD] = {};

int px4_errno;

inline bool valid_fd(int fd)
{
	return (fd < PX4_MAX_FD && fd >= 0 && filemap[fd] != NULL);
}

int px4_open(const char *path, int flags, ...)
{
	PX4_DEBUG("px4_open");
	VDev *dev = VDev::getDev(path);
	int ret = 0;
	int i;
	mode_t mode;

	if (!dev && (flags & (PX4_F_WRONLY|PX4_F_CREAT)) != 0 &&
		strncmp(path, "/obj/", 5) != 0 &&
		strncmp(path, "/dev/", 5) != 0) 
	{
		va_list p;
		va_start(p, flags);
 		mode = va_arg(p, mode_t);
		va_end(p);

		// Create the file
		PX4_DEBUG("Creating virtual file %s", path);
		dev = VFile::createFile(path, mode);
	}
	if (dev) {
		for (i=0; i<PX4_MAX_FD; ++i) {
			if (filemap[i] == 0) {
				filemap[i] = new device::file_t(flags,dev,i);
				break;
			}
		}
		if (i < PX4_MAX_FD) {
			ret = dev->open(filemap[i]);
		}
		else {
			PX4_WARN("exceeded maximum number of file descriptors!");
			ret = -ENOENT;
		}
	}
	else {
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
	if (valid_fd(fd)) {
		VDev *dev = (VDev *)(filemap[fd]->vdev);
		PX4_DEBUG("px4_close fd = %d", fd);
		ret = dev->close(filemap[fd]);
		filemap[fd] = NULL;
	}
	else { 
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
	if (valid_fd(fd)) {
		VDev *dev = (VDev *)(filemap[fd]->vdev);
		PX4_DEBUG("px4_read fd = %d", fd);
		ret = dev->read(filemap[fd], (char *)buffer, buflen);
	}
	else { 
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
        if (valid_fd(fd)) {
		VDev *dev = (VDev *)(filemap[fd]->vdev);
		PX4_DEBUG("px4_write fd = %d", fd);
		ret = dev->write(filemap[fd], (const char *)buffer, buflen);
	}
	else { 
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
        if (valid_fd(fd)) {
		VDev *dev = (VDev *)(filemap[fd]->vdev);
		ret = dev->ioctl(filemap[fd], cmd, arg);
	}
	else { 
                ret = -EINVAL;
        }
	if (ret < 0) {
		px4_errno = -ret;
	}
	
        return ret;
}

int px4_poll(px4_pollfd_struct_t *fds, nfds_t nfds, int timeout)
{
	sem_t sem;
	int count = 0;
	int ret = -1;
	unsigned int i;

	PX4_DEBUG("Called px4_poll timeout = %d", timeout);
	sem_init(&sem, 0, 0);

	// For each fd 
	for (i=0; i<nfds; ++i)
	{
		fds[i].sem     = &sem;
		fds[i].revents = 0;
		fds[i].priv    = NULL;

		// If fd is valid
		if (valid_fd(fds[i].fd))
		{
			VDev *dev = (VDev *)(filemap[fds[i].fd]->vdev);;
			PX4_DEBUG("px4_poll: VDev->poll(setup) %d", fds[i].fd);
			ret = dev->poll(filemap[fds[i].fd], &fds[i], true);

			if (ret < 0)
				break;
		}
	}

	if (ret >= 0)
	{
		if (timeout > 0)
		{
			// Use a work queue task
			work_s _hpwork;

			hrt_work_queue(&_hpwork, (worker_t)&timer_cb, (void *)&sem, 1000*timeout);
			sem_wait(&sem);

			// Make sure timer thread is killed before sem goes
			// out of scope
			hrt_work_cancel(&_hpwork);
        	}
		else if (timeout < 0) 
		{
			sem_wait(&sem);
		}

		// For each fd 
		for (i=0; i<nfds; ++i)
		{
			// If fd is valid
			if (valid_fd(fds[i].fd))
			{
				VDev *dev = (VDev *)(filemap[fds[i].fd]->vdev);;
				PX4_DEBUG("px4_poll: VDev->poll(teardown) %d", fds[i].fd);
				ret = dev->poll(filemap[fds[i].fd], &fds[i], false);
	
				if (ret < 0)
					break;

				if (fds[i].revents)
				count += 1;
			}
		}
	}

	sem_destroy(&sem);

	return count;
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

const char * px4_get_device_names(unsigned int *handle)
{
	return VDev::devList(handle);
}

const char * px4_get_topic_names(unsigned int *handle)
{
	return VDev::topicList(handle);
}

}

