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
#include "device.h"
#include "drivers/drv_device.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

namespace device
{

int px4_errno;
 

struct px4_dev_t {
	char *name;
	void *cdev;

	px4_dev_t(const char *n, void *c) : cdev(c) {
		name = strdup(n); 
	}

	~px4_dev_t() { free(name); }

private:
	px4_dev_t() {}
};

#define PX4_MAX_DEV 100
static px4_dev_t *devmap[PX4_MAX_DEV];

/*
 * The standard NuttX operation dispatch table can't call C++ member functions
 * directly, so we have to bounce them through this dispatch table.
 */

CDev::CDev(const char *name,
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
	for (unsigned i = 0; i < _max_pollwaiters; i++)
		_pollset[i] = nullptr;
}

CDev::~CDev()
{
	if (_registered)
		unregister_driver(_devname);
}

int
CDev::register_class_devname(const char *class_devname)
{
	if (class_devname == nullptr) {
		return -EINVAL;
	}

	int class_instance = 0;
	int ret = -ENOSPC;

	while (class_instance < 4) {
		char name[32];
		snprintf(name, sizeof(name), "%s%d", class_devname, class_instance);
		ret = register_driver(name, (void *)this);
		class_instance++;
	}

	if (class_instance == 4) {
		return ret;
	}
	return class_instance;
}

int
CDev::register_driver(const char *name, void *data)
{
	int ret = -ENOSPC;

	if (name == NULL || data == NULL)
		return -EINVAL;

	// Make sure the device does not already exist
	// FIXME - convert this to a map for efficiency
	for (int i=0;i<PX4_MAX_DEV; ++i) {
		if (devmap[i] && (strcmp(devmap[i]->name,name) == 0)) {
			return -EEXIST;
		}
	}
	for (int i=0;i<PX4_MAX_DEV; ++i) {
		if (devmap[i] == NULL) {
			devmap[i] = new px4_dev_t(name, (void *)data);
			debug("Registered DEV %s", name);
			ret = PX4_OK;
			break;
		}
	}
	return ret;
}

int
CDev::unregister_driver(const char *name)
{
	int ret = -ENOSPC;

	if (name == NULL)
		return -EINVAL;

	for (int i=0;i<PX4_MAX_DEV; ++i) {
		if (devmap[i] && (strcmp(name, devmap[i]->name) == 0)) {
			delete devmap[i];
			devmap[i] = NULL;
			debug("Unregistered DEV %s", name);
			ret = PX4_OK;
			break;
		}
	}
	return ret;
}

int
CDev::unregister_class_devname(const char *class_devname, unsigned class_instance)
{
	char name[32];
	snprintf(name, sizeof(name), "%s%u", class_devname, class_instance);
	for (int i=0;i<PX4_MAX_DEV; ++i) {
		if (devmap[i] && strcmp(devmap[i]->name,name) == 0) {
			delete devmap[i];
			devmap[i] = NULL;
			return PX4_OK;
		}
	}
	return -EINVAL;
}

int
CDev::init()
{
	// base class init first
	int ret = Device::init();

	if (ret != PX4_OK)
		goto out;

	// now register the driver
	if (_devname != nullptr) {
		ret = register_driver(_devname, (void *)this);

		if (ret != PX4_OK)
			goto out;

		_registered = true;
	}

out:
	return ret;
}

/*
 * Default implementations of the character device interface
 */
int
CDev::open(px4_dev_handle_t *handlep)
{
	int ret = PX4_OK;

	debug("CDev::open");
	lock();
	/* increment the open count */
	_open_count++;

	if (_open_count == 1) {

		/* first-open callback may decline the open */
		ret = open_first(handlep);

		if (ret != PX4_OK)
			_open_count--;
	}

	unlock();

	return ret;
}

int
CDev::open_first(px4_dev_handle_t *handlep)
{
	debug("CDev::open_first");
	return PX4_OK;
}

int
CDev::close(px4_dev_handle_t *handlep)
{
	debug("CDev::close");
	int ret = PX4_OK;

	lock();

	if (_open_count > 0) {
		/* decrement the open count */
		_open_count--;

		/* callback cannot decline the close */
		if (_open_count == 0)
			ret = close_last(handlep);

	} else {
		ret = -EBADF;
	}

	unlock();

	return ret;
}

int
CDev::close_last(px4_dev_handle_t *handlep)
{
	debug("CDev::close_last");
	return PX4_OK;
}

ssize_t
CDev::read(px4_dev_handle_t *handlep, char *buffer, size_t buflen)
{
	debug("CDev::read");
	return -ENOSYS;
}

ssize_t
CDev::write(px4_dev_handle_t *handlep, const char *buffer, size_t buflen)
{
	debug("CDev::write");
	return -ENOSYS;
}

off_t
CDev::seek(px4_dev_handle_t *handlep, off_t offset, int whence)
{
	return -ENOSYS;
}

int
CDev::ioctl(px4_dev_handle_t *handlep, int cmd, unsigned long arg)
{
	debug("CDev::ioctl");
	switch (cmd) {

		/* fetch a pointer to the driver's private data */
	case PX4_DIOC_GETPRIV:
		*(void **)(uintptr_t)arg = (void *)this;
		return PX4_OK;
		break;
	case PX4_DEVIOCSPUBBLOCK:
		_pub_blocked = (arg != 0);
		return PX4_OK;
		break;
	case PX4_DEVIOCGPUBBLOCK:
		return _pub_blocked;
		break;
	}

#if 0
	/* try the superclass. The different ioctl() function form
         * means we need to copy arg */
        unsigned arg2 = arg;
	int ret = Device::ioctl(cmd, arg2);
	if (ret != -ENODEV)
		return ret;
#endif
	return -ENOTTY;
}

int
CDev::poll(px4_dev_handle_t *handlep, px4_pollfd_struct_t *fds, bool setup)
{
	int ret = PX4_OK;
	debug("CDev::Poll %s", setup ? "setup" : "teardown");

	/*
	 * Lock against pollnotify() (and possibly other callers)
	 */
	lock();

	if (setup) {
		/*
		 * Save the file pointer in the pollfd for the subclass'
		 * benefit.
		 */
		fds->priv = (void *)handlep;
		debug("CDev::poll: fds->priv = %p", handlep);

		/*
		 * Handle setup requests.
		 */
		ret = store_poll_waiter(fds);

		if (ret == PX4_OK) {

			/*
			 * Check to see whether we should send a poll notification
			 * immediately.
			 */
			fds->revents |= fds->events & poll_state(handlep);

			/* yes? post the notification */
			if (fds->revents != 0)
				sem_post(fds->sem);
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
CDev::poll_notify(pollevent_t events)
{
	debug("CDev::poll_notify events = %0x", events);

	/* lock against poll() as well as other wakeups */
	lock();

	for (unsigned i = 0; i < _max_pollwaiters; i++)
		if (nullptr != _pollset[i])
			poll_notify_one(_pollset[i], events);
	for (unsigned i = 0; i < _max_pollwaiters; i++)
		if (nullptr != _pollset[i])
			debug(" CHECK fds=%p %0x %0x",_pollset[i], _pollset[i]->revents, _pollset[i]->events);

	unlock();
}

void
CDev::poll_notify_one(px4_pollfd_struct_t *fds, pollevent_t events)
{
	debug("CDev::poll_notify_one");
	int value;
	sem_getvalue(fds->sem, &value);

	/* update the reported event set */
	fds->revents |= fds->events & events;

	debug(" Events fds=%p %0x %0x %0x %d",fds, fds->revents, fds->events, events, value);

	/* if the state is now interesting, wake the waiter if it's still asleep */
	/* XXX semcount check here is a vile hack; counting semphores should not be abused as cvars */
	if ((fds->revents != 0) && (value <= 0))
		sem_post(fds->sem);
}

pollevent_t
CDev::poll_state(px4_dev_handle_t *handlep)
{
	debug("CDev::poll_notify");
	/* by default, no poll events to report */
	return 0;
}

int
CDev::store_poll_waiter(px4_pollfd_struct_t *fds)
{
	/*
	 * Look for a free slot.
	 */
	debug("CDev::store_poll_waiter");
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
CDev::remove_poll_waiter(px4_pollfd_struct_t *fds)
{
	debug("CDev::remove_poll_waiter");
	for (unsigned i = 0; i < _max_pollwaiters; i++) {
		if (fds == _pollset[i]) {

			_pollset[i] = nullptr;
			return PX4_OK;

		}
	}

	puts("poll: bad fd state");
	return -EINVAL;
}

CDev *CDev::getDev(const char *path)
{
	int i=0;
	for (; i<PX4_MAX_DEV; ++i) {
		if (devmap[i] && (strcmp(devmap[i]->name, path) == 0)) {
			return (CDev *)(devmap[i]->cdev);
		}
	}
	return NULL;
}

} // namespace device

