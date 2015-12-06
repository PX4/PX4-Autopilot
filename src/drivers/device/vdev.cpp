/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file vcdev.cpp
 *
 * Virtual character device base class.
 */

#include "px4_posix.h"
#include "vdev.h"
#include "drivers/drv_device.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "DevMgr.hpp"

using namespace DriverFramework;

namespace device
{

int px4_errno;

struct px4_dev_t {
	char *name;
	void *cdev;

	px4_dev_t(const char *n, void *c) : cdev(c)
	{
		name = strdup(n);
	}

	~px4_dev_t() { free(name); }

private:
	px4_dev_t() {}
};

#define PX4_MAX_DEV 500
static px4_dev_t *devmap[PX4_MAX_DEV];
pthread_mutex_t devmutex = PTHREAD_MUTEX_INITIALIZER;

/*
 * The standard NuttX operation dispatch table can't call C++ member functions
 * directly, so we have to bounce them through this dispatch table.
 */

VDev::VDev(const char *name,
	   const char *devname) :
	// base class
	Device(name),
	// public
	// protected
	_pub_blocked(false),
	// private
	_devname(devname),
	_registered(false),
	_open_count(0)
{
	PX4_DEBUG("VDev::VDev");

	for (unsigned i = 0; i < _max_pollwaiters; i++) {
		_pollset[i] = nullptr;
	}
}

VDev::~VDev()
{
	PX4_DEBUG("VDev::~VDev");

	if (_registered) {
		unregister_driver(_devname);
	}
}

int
VDev::register_class_devname(const char *class_devname)
{
	PX4_DEBUG("VDev::register_class_devname %s", class_devname);

	if (class_devname == nullptr) {
		return -EINVAL;
	}

	int class_instance = 0;
	int ret = -ENOSPC;

	while (class_instance < 4) {
		char name[32];
		snprintf(name, sizeof(name), "%s%d", class_devname, class_instance);
		ret = register_driver(name, (void *)this);

		if (ret == OK) { break; }

		class_instance++;
	}

	if (class_instance == 4) {
		return ret;
	}

	return class_instance;
}

int
VDev::register_driver(const char *name, void *data)
{
	PX4_DEBUG("VDev::register_driver %s", name);
	int ret = -ENOSPC;

	if (name == NULL || data == NULL) {
		return -EINVAL;
	}

	// Make sure the device does not already exist
	// FIXME - convert this to a map for efficiency

	pthread_mutex_lock(&devmutex);

	for (int i = 0; i < PX4_MAX_DEV; ++i) {
		if (devmap[i] && (strcmp(devmap[i]->name, name) == 0)) {
			pthread_mutex_unlock(&devmutex);
			return -EEXIST;
		}
	}

	for (int i = 0; i < PX4_MAX_DEV; ++i) {
		if (devmap[i] == NULL) {
			devmap[i] = new px4_dev_t(name, (void *)data);
			PX4_DEBUG("Registered DEV %s", name);
			ret = PX4_OK;
			break;
		}
	}

	pthread_mutex_unlock(&devmutex);

	if (ret != PX4_OK) {
		PX4_ERR("No free devmap entries - increase PX4_MAX_DEV");
	}

	return ret;
}

int
VDev::unregister_driver(const char *name)
{
	PX4_DEBUG("VDev::unregister_driver %s", name);
	int ret = -EINVAL;

	if (name == NULL) {
		return -EINVAL;
	}

	pthread_mutex_lock(&devmutex);

	for (int i = 0; i < PX4_MAX_DEV; ++i) {
		if (devmap[i] && (strcmp(name, devmap[i]->name) == 0)) {
			delete devmap[i];
			devmap[i] = NULL;
			PX4_DEBUG("Unregistered DEV %s", name);
			ret = PX4_OK;
			break;
		}
	}

	pthread_mutex_unlock(&devmutex);

	return ret;
}

int
VDev::unregister_class_devname(const char *class_devname, unsigned class_instance)
{
	PX4_DEBUG("VDev::unregister_class_devname");
	char name[32];
	snprintf(name, sizeof(name), "%s%u", class_devname, class_instance);

	pthread_mutex_lock(&devmutex);

	for (int i = 0; i < PX4_MAX_DEV; ++i) {
		if (devmap[i] && strcmp(devmap[i]->name, name) == 0) {
			delete devmap[i];
			PX4_DEBUG("Unregistered class DEV %s", name);
			devmap[i] = NULL;
			pthread_mutex_unlock(&devmutex);
			return PX4_OK;
		}
	}

	pthread_mutex_unlock(&devmutex);

	return -EINVAL;
}

int
VDev::init()
{
	PX4_DEBUG("VDev::init");

	// base class init first
	int ret = Device::init();

	if (ret != PX4_OK) {
		goto out;
	}

	// now register the driver
	if (_devname != nullptr) {
		ret = register_driver(_devname, (void *)this);

		if (ret != PX4_OK) {
			goto out;
		}

		_registered = true;
	}

out:
	return ret;
}

/*
 * Default implementations of the character device interface
 */
int
VDev::open(file_t *filep)
{
	PX4_DEBUG("VDev::open");
	int ret = PX4_OK;

	lock();
	/* increment the open count */
	_open_count++;

	if (_open_count == 1) {

		/* first-open callback may decline the open */
		ret = open_first(filep);

		if (ret != PX4_OK) {
			_open_count--;
		}
	}

	unlock();

	return ret;
}

int
VDev::open_first(file_t *filep)
{
	PX4_DEBUG("VDev::open_first");
	return PX4_OK;
}

int
VDev::close(file_t *filep)
{
	PX4_DEBUG("VDev::close");
	int ret = PX4_OK;

	lock();

	if (_open_count > 0) {
		/* decrement the open count */
		_open_count--;

		/* callback cannot decline the close */
		if (_open_count == 0) {
			ret = close_last(filep);
		}

	} else {
		ret = -EBADF;
	}

	unlock();

	return ret;
}

int
VDev::close_last(file_t *filep)
{
	PX4_DEBUG("VDev::close_last");
	return PX4_OK;
}

ssize_t
VDev::read(file_t *filep, char *buffer, size_t buflen)
{
	PX4_DEBUG("VDev::read");
	return -ENOSYS;
}

ssize_t
VDev::write(file_t *filep, const char *buffer, size_t buflen)
{
	PX4_DEBUG("VDev::write");
	return -ENOSYS;
}

off_t
VDev::seek(file_t *filep, off_t offset, int whence)
{
	PX4_DEBUG("VDev::seek");
	return -ENOSYS;
}

int
VDev::ioctl(file_t *filep, int cmd, unsigned long arg)
{
	PX4_DEBUG("VDev::ioctl");
	int ret = -ENOTTY;

	switch (cmd) {

	/* fetch a pointer to the driver's private data */
	case DIOC_GETPRIV:
		*(void **)(uintptr_t)arg = (void *)this;
		ret = PX4_OK;
		break;

	case DEVIOCSPUBBLOCK:
		_pub_blocked = (arg != 0);
		ret = PX4_OK;
		break;

	case DEVIOCGPUBBLOCK:
		ret = _pub_blocked;
		break;

	case DEVIOCGDEVICEID:
		ret = (int)_device_id.devid;
		break;

	default:
		break;
	}

	return ret;
}

int
VDev::poll(file_t *filep, px4_pollfd_struct_t *fds, bool setup)
{
	PX4_DEBUG("VDev::Poll %s", setup ? "setup" : "teardown");
	int ret = PX4_OK;

	/*
	 * Lock against pollnotify() (and possibly other callers)
	 */
	lock();

	if (setup) {
		/*
		 * Save the file pointer in the pollfd for the subclass'
		 * benefit.
		 */
		fds->priv = (void *)filep;
		PX4_DEBUG("VDev::poll: fds->priv = %p", filep);

		/*
		 * Handle setup requests.
		 */
		ret = store_poll_waiter(fds);

		if (ret == PX4_OK) {

			/*
			 * Check to see whether we should send a poll notification
			 * immediately.
			 */
			fds->revents |= fds->events & poll_state(filep);

			/* yes? post the notification */
			if (fds->revents != 0) {
				px4_sem_post(fds->sem);
			}

		} else {
			PX4_WARN("Store Poll Waiter error.");
		}

	} else {
		/*
		 * Handle a teardown request.
		 */
		ret = remove_poll_waiter(fds);
	}

	unlock();

	return ret;
}

void
VDev::poll_notify(pollevent_t events)
{
	PX4_DEBUG("VDev::poll_notify events = %0x", events);

	/* lock against poll() as well as other wakeups */
	lock();

	for (unsigned i = 0; i < _max_pollwaiters; i++)
		if (nullptr != _pollset[i]) {
			poll_notify_one(_pollset[i], events);
		}

	unlock();
}

void
VDev::poll_notify_one(px4_pollfd_struct_t *fds, pollevent_t events)
{
	PX4_DEBUG("VDev::poll_notify_one");
	int value;
	px4_sem_getvalue(fds->sem, &value);

	/* update the reported event set */
	fds->revents |= fds->events & events;

	PX4_DEBUG(" Events fds=%p %0x %0x %0x %d", fds, fds->revents, fds->events, events, value);

	/* if the state is now interesting, wake the waiter if it's still asleep */
	/* XXX semcount check here is a vile hack; counting semphores should not be abused as cvars */
	if ((fds->revents != 0) && (value <= 0)) {
		px4_sem_post(fds->sem);
	}
}

pollevent_t
VDev::poll_state(file_t *filep)
{
	PX4_DEBUG("VDev::poll_notify");
	/* by default, no poll events to report */
	return 0;
}

int
VDev::store_poll_waiter(px4_pollfd_struct_t *fds)
{
	/*
	 * Look for a free slot.
	 */
	PX4_DEBUG("VDev::store_poll_waiter");

	for (unsigned i = 0; i < _max_pollwaiters; i++) {
		if (nullptr == _pollset[i]) {

			/* save the pollfd */
			_pollset[i] = fds;

			return PX4_OK;
		}
	}

	return -ENOMEM;
}

int
VDev::remove_poll_waiter(px4_pollfd_struct_t *fds)
{
	PX4_DEBUG("VDev::remove_poll_waiter");

	for (unsigned i = 0; i < _max_pollwaiters; i++) {
		if (fds == _pollset[i]) {

			_pollset[i] = nullptr;
			return PX4_OK;

		}
	}

	PX4_WARN("poll: bad fd state");
	return -EINVAL;
}

VDev *VDev::getDev(const char *path)
{
	PX4_DEBUG("VDev::getDev");
	int i = 0;

	pthread_mutex_lock(&devmutex);

	for (; i < PX4_MAX_DEV; ++i) {
		//if (devmap[i]) {
		//	printf("%s %s\n", devmap[i]->name, path);
		//}
		if (devmap[i] && (strcmp(devmap[i]->name, path) == 0)) {
			pthread_mutex_unlock(&devmutex);
			return (VDev *)(devmap[i]->cdev);
		}
	}

	pthread_mutex_unlock(&devmutex);

	return NULL;
}

void VDev::showDevices()
{
	int i = 0;
	PX4_INFO("PX4 Devices:");

	pthread_mutex_lock(&devmutex);

	for (; i < PX4_MAX_DEV; ++i) {
		if (devmap[i] && strncmp(devmap[i]->name, "/dev/", 5) == 0) {
			PX4_INFO("   %s", devmap[i]->name);
		}
	}

	pthread_mutex_unlock(&devmutex);

#ifndef __PX4_UNIT_TESTS
	PX4_INFO("DF Devices:");
	const char *dev_path;
	unsigned int index = 0;
	i = 0;

	do {
		// Each look increments index and returns -1 if end reached
		i = DevMgr::getNextDeviceName(index, &dev_path);

		if (i == 0) {
			PX4_INFO("   %s", dev_path);
		}
	} while (i == 0);

#endif
}

void VDev::showTopics()
{
	int i = 0;
	PX4_INFO("Devices:");

	pthread_mutex_lock(&devmutex);

	for (; i < PX4_MAX_DEV; ++i) {
		if (devmap[i] && strncmp(devmap[i]->name, "/obj/", 5) == 0) {
			PX4_INFO("   %s", devmap[i]->name);
		}
	}

	pthread_mutex_unlock(&devmutex);
}

void VDev::showFiles()
{
	int i = 0;
	PX4_INFO("Files:");

	pthread_mutex_lock(&devmutex);

	for (; i < PX4_MAX_DEV; ++i) {
		if (devmap[i] && strncmp(devmap[i]->name, "/obj/", 5) != 0 &&
		    strncmp(devmap[i]->name, "/dev/", 5) != 0) {
			PX4_INFO("   %s", devmap[i]->name);
		}
	}

	pthread_mutex_unlock(&devmutex);
}

const char *VDev::topicList(unsigned int *next)
{
	for (; *next < PX4_MAX_DEV; (*next)++)
		if (devmap[*next] && strncmp(devmap[(*next)]->name, "/obj/", 5) == 0) {
			return devmap[(*next)++]->name;
		}

	return NULL;
}

const char *VDev::devList(unsigned int *next)
{
	for (; *next < PX4_MAX_DEV; (*next)++)
		if (devmap[*next] && strncmp(devmap[(*next)]->name, "/dev/", 5) == 0) {
			return devmap[(*next)++]->name;
		}

	return NULL;
}

} // namespace device
