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

#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <algorithm>

#include "uORBDevices_posix.hpp"
#include "uORBUtils.hpp"
#include "uORBManager.hpp"
#include "uORBCommunicator.hpp"
#include <px4_sem.hpp>
#include <stdlib.h>


uORB::DeviceNode::SubscriberData  *uORB::DeviceNode::filp_to_sd(device::file_t *filp)
{
	uORB::DeviceNode::SubscriberData *sd;

	if (filp) {
		sd = (uORB::DeviceNode::SubscriberData *)(filp->priv);

	} else {
		sd = 0;
	}

	return sd;
}

uORB::DeviceNode::DeviceNode(const struct orb_metadata *meta, const char *name, const char *path,
			     int priority, unsigned int queue_size) :
	VDev(name, path),
	_meta(meta),
	_data(nullptr),
	_last_update(0),
	_generation(0),
	_publisher(0),
	_priority(priority),
	_published(false),
	_queue_size(queue_size),
	_subscriber_count(0)
{
	// enable debug() calls
	//_debug_enabled = true;
}

uORB::DeviceNode::~DeviceNode()
{
	if (_data != nullptr) {
		delete[] _data;
	}

}

int
uORB::DeviceNode::open(device::file_t *filp)
{
	int ret;

	/* is this a publisher? */
	if (filp->flags == PX4_F_WRONLY) {

		/* become the publisher if we can */
		lock();

		if (_publisher == 0) {
			_publisher = px4_getpid();
			ret = PX4_OK;

		} else {
			ret = -EBUSY;
		}

		unlock();

		/* now complete the open */
		if (ret == PX4_OK) {
			ret = VDev::open(filp);

			/* open failed - not the publisher anymore */
			if (ret != PX4_OK) {
				_publisher = 0;
			}
		}

		return ret;
	}

	/* is this a new subscriber? */
	if (filp->flags == PX4_F_RDONLY) {

		/* allocate subscriber data */
		SubscriberData *sd = new SubscriberData;

		if (nullptr == sd) {
			return -ENOMEM;
		}

		memset(sd, 0, sizeof(*sd));

		/* default to no pending update */
		sd->generation = _generation;

		/* set priority */
		sd->set_priority(_priority);

		filp->priv = (void *)sd;

		ret = VDev::open(filp);

		add_internal_subscriber();

		if (ret != PX4_OK) {
			warnx("ERROR: VDev::open failed\n");
			delete sd;
		}

		//warnx("uORB::DeviceNode::Open: fd = %d flags = %d, priv = %p cdev = %p\n", filp->fd, filp->flags, filp->priv, filp->cdev);
		return ret;
	}

	/* can only be pub or sub, not both */
	return -EINVAL;
}

int
uORB::DeviceNode::close(device::file_t *filp)
{
	//warnx("uORB::DeviceNode::close fd = %d", filp->fd);
	/* is this the publisher closing? */
	if (px4_getpid() == _publisher) {
		_publisher = 0;

	} else {
		SubscriberData *sd = filp_to_sd(filp);

		if (sd != nullptr) {
			if (sd->update_interval) {
				hrt_cancel(&sd->update_interval->update_call);
			}

			remove_internal_subscriber();
			delete sd;
			sd = nullptr;
		}
	}

	return VDev::close(filp);
}

ssize_t
uORB::DeviceNode::read(device::file_t *filp, char *buffer, size_t buflen)
{
	//warnx("uORB::DeviceNode::read fd = %d\n", filp->fd);
	SubscriberData *sd = (SubscriberData *)filp_to_sd(filp);

	/* if the object has not been written yet, return zero */
	if (_data == nullptr) {
		return 0;
	}

	/* if the caller's buffer is the wrong size, that's an error */
	if (buflen != _meta->o_size) {
		return -EIO;
	}

	/*
	 * Perform an atomic copy & state update
	 */
	lock();

	if (_generation > sd->generation + _queue_size) {
		/* Reader is too far behind: some messages are lost */
		_lost_messages += _generation - (sd->generation + _queue_size);
		sd->generation = _generation - _queue_size;
	}

	if (_generation == sd->generation && sd->generation > 0) {
		/* The subscriber already read the latest message, but nothing new was published yet.
		 * Return the previous message
		 */
		--sd->generation;
	}

	/* if the caller doesn't want the data, don't give it to them */
	if (nullptr != buffer) {
		memcpy(buffer, _data + (_meta->o_size * (sd->generation % _queue_size)), _meta->o_size);
	}

	if (sd->generation < _generation) {
		++sd->generation;
	}

	/* set priority */
	sd->set_priority(_priority);

	/*
	 * Clear the flag that indicates that an update has been reported, as
	 * we have just collected it.
	 */
	sd->set_update_reported(false);

	unlock();

	return _meta->o_size;
}

ssize_t
uORB::DeviceNode::write(device::file_t *filp, const char *buffer, size_t buflen)
{
	//warnx("uORB::DeviceNode::write filp = %p (null is normal)", filp);
	/*
	 * Writes are legal from interrupt context as long as the
	 * object has already been initialised from thread context.
	 *
	 * Writes outside interrupt context will allocate the object
	 * if it has not yet been allocated.
	 *
	 * Note that filp will usually be NULL.
	 */
	if (nullptr == _data) {
		lock();

		/* re-check size */
		if (nullptr == _data) {
			_data = new uint8_t[_meta->o_size * _queue_size];
		}

		unlock();

		/* failed or could not allocate */
		if (nullptr == _data) {
			return -ENOMEM;
		}
	}

	/* If write size does not match, that is an error */
	if (_meta->o_size != buflen) {
		return -EIO;
	}

	lock();
	memcpy(_data + (_meta->o_size * (_generation % _queue_size)), buffer, _meta->o_size);

	/* update the timestamp and generation count */
	_last_update = hrt_absolute_time();
	/* wrap-around happens after ~49 days, assuming a publisher rate of 1 kHz */
	_generation++;

	_published = true;

	unlock();

	/* notify any poll waiters */
	poll_notify(POLLIN);

	return _meta->o_size;
}

int
uORB::DeviceNode::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	//warnx("uORB::DeviceNode::ioctl fd = %d cmd = %d", filp->fd, cmd);
	SubscriberData *sd = filp_to_sd(filp);

	switch (cmd) {
	case ORBIOCLASTUPDATE:
		lock();
		*(hrt_abstime *)arg = _last_update;
		unlock();
		return PX4_OK;

	case ORBIOCUPDATED:
		lock();
		*(bool *)arg = appears_updated(sd);
		unlock();
		return PX4_OK;

	case ORBIOCSETINTERVAL: {
			int ret = PX4_OK;
			lock();

			if (arg == 0) {
				if (sd->update_interval) {
					delete(sd->update_interval);
					sd->update_interval = nullptr;
				}

			} else {
				if (sd->update_interval) {
					sd->update_interval->interval = arg;
					sd->update_interval->last_update = hrt_absolute_time();

				} else {
					sd->update_interval = new UpdateIntervalData();

					if (sd->update_interval) {
						memset(&sd->update_interval->update_call, 0, sizeof(hrt_call));
						sd->update_interval->interval = arg;
						sd->update_interval->last_update = hrt_absolute_time();

					} else {
						ret = -ENOMEM;
					}
				}
			}

			unlock();
			return ret;
		}

	case ORBIOCGADVERTISER:
		*(uintptr_t *)arg = (uintptr_t)this;
		return PX4_OK;

	case ORBIOCGPRIORITY:
		*(int *)arg = sd->priority();
		return PX4_OK;

	case ORBIOCSETQUEUESIZE:
		//no need for locking here, since this is used only during the advertisement call,
		//and only one advertiser is allowed to open the DeviceNode at the same time.
		return update_queue_size(arg);

	case ORBIOCGETINTERVAL:
		if (sd->update_interval) {
			*(unsigned *)arg = sd->update_interval->interval;

		} else {
			*(unsigned *)arg = 0;
		}

		return OK;

	default:
		/* give it to the superclass */
		return VDev::ioctl(filp, cmd, arg);
	}
}

ssize_t
uORB::DeviceNode::publish(const orb_metadata *meta, orb_advert_t handle, const void *data)
{
	//warnx("uORB::DeviceNode::publish meta = %p", meta);

	if (handle == nullptr) {
		warnx("uORB::DeviceNode::publish called with invalid handle");
		errno = EINVAL;
		return ERROR;
	}

	uORB::DeviceNode *devnode = (uORB::DeviceNode *)handle;
	int ret;

	/* this is a bit risky, since we are trusting the handle in order to deref it */
	if (devnode->_meta != meta) {
		errno = EINVAL;
		return ERROR;
	}

	/* call the devnode write method with no file pointer */
	ret = devnode->write(nullptr, (const char *)data, meta->o_size);

	if (ret < 0) {
		return ERROR;
	}

	if (ret != (int)meta->o_size) {
		errno = EIO;
		return ERROR;
	}

	/*
	 * if the write is successful, send the data over the Multi-ORB link
	 */
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();

	if (ch != nullptr) {
		if (ch->send_message(meta->o_name, meta->o_size, (uint8_t *)data) != 0) {
			warnx("[uORB::DeviceNode::publish(%d)]: Error Sending [%s] topic data over comm_channel",
			      __LINE__, meta->o_name);
			return ERROR;
		}
	}

	return PX4_OK;
}

int uORB::DeviceNode::unadvertise(orb_advert_t handle)
{
	if (handle == nullptr) {
		return -EINVAL;
	}

	uORB::DeviceNode *devnode = (uORB::DeviceNode *)handle;

	/*
	 * We are cheating a bit here. First, with the current implementation, we can only
	 * have multiple publishers for instance 0. In this case the caller will have
	 * instance=nullptr and _published has no effect at all. Thus no unadvertise is
	 * necessary.
	 * In case of multiple instances, we have at most 1 publisher per instance and
	 * we can signal an instance as 'free' by setting _published to false.
	 * We never really free the DeviceNode, for this we would need reference counting
	 * of subscribers and publishers. But we also do not have a leak since future
	 * publishers reuse the same DeviceNode object.
	 */
	devnode->_published = false;

	return PX4_OK;
}

pollevent_t
uORB::DeviceNode::poll_state(device::file_t *filp)
{
	//warnx("uORB::DeviceNode::poll_state fd = %d", filp->fd);
	SubscriberData *sd = filp_to_sd(filp);

	/*
	 * If the topic appears updated to the subscriber, say so.
	 */
	if (appears_updated(sd)) {
		return POLLIN;
	}

	return 0;
}

void
uORB::DeviceNode::poll_notify_one(px4_pollfd_struct_t *fds, pollevent_t events)
{
	//warnx("uORB::DeviceNode::poll_notify_one fds = %p fds->priv = %p", fds, fds->priv);
	SubscriberData *sd = filp_to_sd((device::file_t *)fds->priv);

	/*
	 * If the topic looks updated to the subscriber, go ahead and notify them.
	 */
	if (appears_updated(sd)) {
		VDev::poll_notify_one(fds, events);
	}
}

bool
uORB::DeviceNode::appears_updated(SubscriberData *sd)
{

	/* block if in simulation mode */
	while (px4_sim_delay_enabled()) {
		usleep(100);
	}

	//warnx("uORB::DeviceNode::appears_updated sd = %p", sd);
	/* assume it doesn't look updated */
	bool ret = false;

	/* check if this topic has been published yet, if not bail out */
	if (_data == nullptr) {
		return false;
	}

	/*
	 * If the subscriber's generation count matches the update generation
	 * count, there has been no update from their perspective; if they
	 * don't match then we might have a visible update.
	 */
	while (sd->generation != _generation) {

		/*
		 * Handle non-rate-limited subscribers.
		 */
		if (sd->update_interval == nullptr) {
			ret = true;
			break;
		}

		/*
		 * If we have previously told the subscriber that there is data,
		 * and they have not yet collected it, continue to tell them
		 * that there has been an update.  This mimics the non-rate-limited
		 * behaviour where checking / polling continues to report an update
		 * until the topic is read.
		 */
		if (sd->update_reported()) {
			ret = true;
			break;
		}

		// If we have not yet reached the deadline, then assume that we can ignore any
		// newly received data.
		if (sd->update_interval->last_update + sd->update_interval->interval > hrt_absolute_time()) {
			break;
		}

		/*
		 * Remember that we have told the subscriber that there is data.
		 */
		sd->set_update_reported(true);
		sd->update_interval->last_update = hrt_absolute_time();
		ret = true;

		break;
	}

	return ret;
}

void
uORB::DeviceNode::update_deferred()
{
	/*
	 * Instigate a poll notification; any subscribers whose intervals have
	 * expired will be woken.
	 */
	poll_notify(POLLIN);
}

void
uORB::DeviceNode::update_deferred_trampoline(void *arg)
{
	uORB::DeviceNode *node = (uORB::DeviceNode *)arg;

	node->update_deferred();
}

bool
uORB::DeviceNode::print_statistics(bool reset)
{
	if (!_lost_messages) {
		return false;
	}

	lock();
	//This can be wrong: if a reader never reads, _lost_messages will not be increased either
	uint32_t lost_messages = _lost_messages;

	if (reset) {
		_lost_messages = 0;
	}

	unlock();

	PX4_INFO("%s: %i", _meta->o_name, lost_messages);
	return true;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void uORB::DeviceNode::add_internal_subscriber()
{
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();

	lock();
	_subscriber_count++;

	if (ch != nullptr && _subscriber_count > 0) {
		unlock(); //make sure we cannot deadlock if add_subscription calls back into DeviceNode
		ch->add_subscription(_meta->o_name, 1);

	} else {
		unlock();
	}
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void uORB::DeviceNode::remove_internal_subscriber()
{
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();

	lock();
	_subscriber_count--;

	if (ch != nullptr && _subscriber_count == 0) {
		unlock(); //make sure we cannot deadlock if remove_subscription calls back into DeviceNode
		ch->remove_subscription(_meta->o_name);

	} else {
		unlock();
	}
}


//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool uORB::DeviceNode::is_published()
{
	return _published;
}

int uORB::DeviceNode::update_queue_size(unsigned int queue_size)
{
	if (_queue_size == queue_size) {
		return PX4_OK;
	}

	if (_data || _queue_size > queue_size) {
		return ERROR;
	}

	_queue_size = queue_size;
	return PX4_OK;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
int16_t uORB::DeviceNode::process_add_subscription(int32_t rateInHz)
{
	// if there is already data in the node, send this out to
	// the remote entity.
	// send the data to the remote entity.
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();

	if (_data != nullptr && ch != nullptr) { // _data will not be null if there is a publisher.
		ch->send_message(_meta->o_name, _meta->o_size, _data);
	}

	return PX4_OK;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
int16_t uORB::DeviceNode::process_remove_subscription()
{
	return PX4_OK;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
int16_t uORB::DeviceNode::process_received_message(int32_t length, uint8_t *data)
{
	int16_t ret = -1;

	if (length != (int32_t)(_meta->o_size)) {
		warnx("[uORB::DeviceNode::process_received_message(%d)]Error:[%s] Received DataLength[%d] != ExpectedLen[%d]",
		      __LINE__, _meta->o_name, (int)length, (int)_meta->o_size);
		return ERROR;
	}

	/* call the devnode write method with no file pointer */
	ret = write(nullptr, (const char *)data, _meta->o_size);

	if (ret < 0) {
		return ERROR;
	}

	if (ret != (int)_meta->o_size) {
		errno = EIO;
		return ERROR;
	}

	return PX4_OK;
}


uORB::DeviceMaster::DeviceMaster(Flavor f) :
	VDev((f == PUBSUB) ? "obj_master" : "param_master",
	     (f == PUBSUB) ? TOPIC_MASTER_DEVICE_PATH : PARAM_MASTER_DEVICE_PATH),
	_flavor(f)
{
	// enable debug() calls
	//_debug_enabled = true;
	_last_statistics_output = hrt_absolute_time();
}

uORB::DeviceMaster::~DeviceMaster()
{
}

int
uORB::DeviceMaster::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	int ret;

	switch (cmd) {
	case ORBIOCADVERTISE: {
			const struct orb_advertdata *adv = (const struct orb_advertdata *)arg;
			const struct orb_metadata *meta = adv->meta;
			const char *objname;
			const char *devpath;
			char nodepath[orb_maxpath];
			uORB::DeviceNode *node;

			/* construct a path to the node - this also checks the node name */
			ret = uORB::Utils::node_mkpath(nodepath, _flavor, meta, adv->instance);

			if (ret != PX4_OK) {
				return ret;
			}

			ret = ERROR;

			/* try for topic groups */
			const unsigned max_group_tries = (adv->instance != nullptr) ? ORB_MULTI_MAX_INSTANCES : 1;
			unsigned group_tries = 0;

			if (adv->instance) {
				/* for an advertiser, this will be 0, but a for subscriber that requests a certain instance,
				 * we do not want to start with 0, but with the instance the subscriber actually requests.
				 */
				group_tries = *adv->instance;

				if (group_tries >= max_group_tries) {
					return -ENOMEM;
				}
			}

			SmartLock smart_lock(_lock);

			do {
				/* if path is modifyable change try index */
				if (adv->instance != nullptr) {
					/* replace the number at the end of the string */
					nodepath[strlen(nodepath) - 1] = '0' + group_tries;
					*(adv->instance) = group_tries;
				}

				objname = meta->o_name; //no need for a copy, meta->o_name will never be freed or changed

				/* driver wants a permanent copy of the path, so make one here */
				devpath = strdup(nodepath);

				if (devpath == nullptr) {
					return -ENOMEM;
				}

				/* construct the new node */
				node = new uORB::DeviceNode(meta, objname, devpath, adv->priority);

				/* if we didn't get a device, that's bad */
				if (node == nullptr) {
					free((void *)devpath);
					return -ENOMEM;
				}

				/* initialise the node - this may fail if e.g. a node with this name already exists */
				ret = node->init();

				/* if init failed, discard the node and its name */
				if (ret != PX4_OK) {
					delete node;

					if (ret == -EEXIST) {
						/* if the node exists already, get the existing one and check if
						 * something has been published yet. */
						uORB::DeviceNode *existing_node = getDeviceNodeLocked(devpath);

						if ((existing_node != nullptr) && !(existing_node->is_published())) {
							/* nothing has been published yet, lets claim it */
							ret = PX4_OK;

						} else {
							/* otherwise: data has already been published, keep looking */
						}
					}

					/* also discard the name now */
					free((void *)devpath);

				} else {
					// add to the node map;.
					_node_map[std::string(nodepath)] = node;
				}


				group_tries++;

			} while (ret != PX4_OK && (group_tries < max_group_tries));

			if (ret != PX4_OK && group_tries >= max_group_tries) {
				ret = -ENOMEM;
			}

			return ret;
		}

	default:
		/* give it to the superclass */
		return VDev::ioctl(filp, cmd, arg);
	}
}

void uORB::DeviceMaster::printStatistics(bool reset)
{
	hrt_abstime current_time = hrt_absolute_time();
	PX4_INFO("Statistics, since last output (%i ms):",
		 (int)((current_time - _last_statistics_output) / 1000));
	_last_statistics_output = current_time;

	PX4_INFO("TOPIC, NR LOST MSGS");

	lock();
	bool had_print = false;

	for (const auto &node : _node_map) {
		if (node.second->print_statistics(reset)) {
			had_print = true;
		}
	}

	unlock();

	if (!had_print) {
		PX4_INFO("No lost messages");
	}
}

uORB::DeviceNode *uORB::DeviceMaster::getDeviceNode(const char *nodepath)
{
	lock();
	uORB::DeviceNode *node = getDeviceNodeLocked(nodepath);
	unlock();
	//We can safely return the node that can be used by any thread, because
	//a DeviceNode never gets deleted.
	return node;
}

uORB::DeviceNode *uORB::DeviceMaster::getDeviceNodeLocked(const char *nodepath)
{
	uORB::DeviceNode *rc = nullptr;
	std::string np(nodepath);

	auto iter = _node_map.find(np);

	if (iter != _node_map.end()) {
		rc = iter->second;
	}

	return rc;
}
