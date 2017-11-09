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
 * @file CDev.cpp
 *
 * Character device base class.
 */

#include "CDev.hpp"

#include <cstring>

#include "px4_posix.h"
#include "drivers/drv_device.h"

namespace device
{

CDev::CDev(const char *name, const char *devname) :
	Device(name),
	_devname(devname)
{
	DEVICE_DEBUG("CDev::CDev");

	int ret = px4_sem_init(&_lock, 0, 1);

	if (ret != 0) {
		PX4_WARN("SEM INIT FAIL: ret %d", ret);
	}
}

CDev::~CDev()
{
	DEVICE_DEBUG("CDev::~CDev");

	if (_registered) {
		unregister_driver(_devname);
	}

	if (_pollset) {
		delete[](_pollset);
	}

	px4_sem_destroy(&_lock);
}

int
CDev::register_class_devname(const char *class_devname)
{
	DEVICE_DEBUG("CDev::register_class_devname %s", class_devname);

	if (class_devname == nullptr) {
		return -EINVAL;
	}

	int class_instance = 0;
	int ret = -ENOSPC;

	while (class_instance < 4) {
		char name[32];
		snprintf(name, sizeof(name), "%s%d", class_devname, class_instance);
		ret = register_driver(name, &fops, 0666, (void *)this);

		if (ret == OK) {
			break;
		}

		class_instance++;
	}

	if (class_instance == 4) {
		return ret;
	}

	return class_instance;
}

int
CDev::unregister_class_devname(const char *class_devname, unsigned class_instance)
{
	DEVICE_DEBUG("CDev::unregister_class_devname");
	char name[32];
	snprintf(name, sizeof(name), "%s%u", class_devname, class_instance);
	return unregister_driver(name);
}

int
CDev::init()
{
	DEVICE_DEBUG("CDev::init");

	// base class init first
	int ret = Device::init();

	if (ret != PX4_OK) {
		goto out;
	}

	// now register the driver
	if (_devname != nullptr) {
		ret = register_driver(_devname, &fops, 0666, (void *)this);

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
CDev::open(file_t *filep)
{
	DEVICE_DEBUG("CDev::open");
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
CDev::open_first(file_t *filep)
{
	DEVICE_DEBUG("CDev::open_first");
	return PX4_OK;
}

int
CDev::close(file_t *filep)
{
	DEVICE_DEBUG("CDev::close");
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
CDev::close_last(file_t *filep)
{
	DEVICE_DEBUG("CDev::close_last");
	return PX4_OK;
}

ssize_t
CDev::read(file_t *filep, char *buffer, size_t buflen)
{
	DEVICE_DEBUG("CDev::read");
	return -ENOSYS;
}

ssize_t
CDev::write(file_t *filep, const char *buffer, size_t buflen)
{
	DEVICE_DEBUG("CDev::write");
	return -ENOSYS;
}

off_t
CDev::seek(file_t *filep, off_t offset, int whence)
{
	DEVICE_DEBUG("CDev::seek");
	return -ENOSYS;
}

int
CDev::ioctl(file_t *filep, int cmd, unsigned long arg)
{
	DEVICE_DEBUG("CDev::ioctl");
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
CDev::poll(file_t *filep, px4_pollfd_struct_t *fds, bool setup)
{
	DEVICE_DEBUG("CDev::Poll %s", setup ? "setup" : "teardown");
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
		DEVICE_DEBUG("CDev::poll: fds->priv = %p", filep);

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
CDev::poll_notify(pollevent_t events)
{
	DEVICE_DEBUG("CDev::poll_notify events = %0x", events);

	/* lock against poll() as well as other wakeups */
	ATOMIC_ENTER;

	for (unsigned i = 0; i < _max_pollwaiters; i++) {
		if (nullptr != _pollset[i]) {
			poll_notify_one(_pollset[i], events);
		}
	}

	ATOMIC_LEAVE;
}

void
CDev::poll_notify_one(px4_pollfd_struct_t *fds, pollevent_t events)
{
	DEVICE_DEBUG("CDev::poll_notify_one");

#ifdef __PX4_NUTTX
	int value = fds->sem->semcount;
#else
	int value = -1;
	px4_sem_getvalue(fds->sem, &value);
#endif

	/* update the reported event set */
	fds->revents |= fds->events & events;

	DEVICE_DEBUG(" Events fds=%p %0x %0x %0x %d", fds, fds->revents, fds->events, events, value);

	/* if the state is now interesting, wake the waiter if it's still asleep */
	/* XXX semcount check here is a vile hack; counting semphores should not be abused as cvars */
	if ((fds->revents != 0) && (value <= 0)) {
		px4_sem_post(fds->sem);
	}
}

pollevent_t
CDev::poll_state(file_t *filep)
{
	DEVICE_DEBUG("CDev::poll_notify");
	/* by default, no poll events to report */
	return 0;
}

int
CDev::store_poll_waiter(px4_pollfd_struct_t *fds)
{
	/*
	 * Look for a free slot.
	 */
	DEVICE_DEBUG("CDev::store_poll_waiter");

	for (unsigned i = 0; i < _max_pollwaiters; i++) {
		if (nullptr == _pollset[i]) {

			/* save the pollfd */
			_pollset[i] = fds;

			return PX4_OK;
		}
	}

	/* No free slot found. Resize the pollset */

	if (_max_pollwaiters >= 256 / 2) { //_max_pollwaiters is uint8_t
		return -ENOMEM;
	}

	const uint8_t new_count = _max_pollwaiters > 0 ? _max_pollwaiters * 2 : 1;
	px4_pollfd_struct_t **new_pollset = new px4_pollfd_struct_t *[new_count];

	if (!new_pollset) {
		return -ENOMEM;
	}

	if (_max_pollwaiters > 0) {
		memset(new_pollset + _max_pollwaiters, 0, sizeof(px4_pollfd_struct_t *) * (new_count - _max_pollwaiters));
		memcpy(new_pollset, _pollset, sizeof(px4_pollfd_struct_t *) * _max_pollwaiters);
		delete[](_pollset);
	}

	_pollset = new_pollset;
	_pollset[_max_pollwaiters] = fds;
	_max_pollwaiters = new_count;
	return PX4_OK;
}

int
CDev::remove_poll_waiter(px4_pollfd_struct_t *fds)
{
	DEVICE_DEBUG("CDev::remove_poll_waiter");

	for (unsigned i = 0; i < _max_pollwaiters; i++) {
		if (fds == _pollset[i]) {

			_pollset[i] = nullptr;
			return PX4_OK;

		}
	}

	PX4_WARN("poll: bad fd state");
	return -EINVAL;
}

} // namespace device
