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

#include "uORBDeviceNode.hpp"

#include "uORBDeviceNode.hpp"
#include "uORBUtils.hpp"
#include "uORBManager.hpp"

#ifdef ORB_COMMUNICATOR
#include "uORBCommunicator.hpp"
#endif /* ORB_COMMUNICATOR */

uORB::DeviceNode::SubscriberData *uORB::DeviceNode::filp_to_sd(cdev::file_t *filp)
{
#ifndef __PX4_NUTTX

	if (!filp) {
		return nullptr;
	}

#endif
	return (SubscriberData *)(filp->f_priv);
}

uORB::DeviceNode::DeviceNode(const struct orb_metadata *meta, const uint8_t instance, const char *path,
			     uint8_t priority, uint8_t queue_size) :
	CDev(path),
	_meta(meta),
	_instance(instance),
	_priority(priority),
	_queue_size(queue_size)
{
}

uORB::DeviceNode::~DeviceNode()
{
	if (_data != nullptr) {
		delete[] _data;
	}
}

int
uORB::DeviceNode::open(cdev::file_t *filp)
{
	int ret;

	/* is this a publisher? */
	if (filp->f_oflags == PX4_F_WRONLY) {

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
			ret = CDev::open(filp);

			/* open failed - not the publisher anymore */
			if (ret != PX4_OK) {
				_publisher = 0;
			}
		}

		return ret;
	}

	/* is this a new subscriber? */
	if (filp->f_oflags == PX4_F_RDONLY) {

		/* allocate subscriber data */
		SubscriberData *sd = new SubscriberData{};

		if (nullptr == sd) {
			return -ENOMEM;
		}

		/* If there were any previous publications, allow the subscriber to read them */
		sd->generation = _generation - (_queue_size < _generation ? _queue_size : _generation);

		filp->f_priv = (void *)sd;

		ret = CDev::open(filp);

		add_internal_subscriber();

		if (ret != PX4_OK) {
			PX4_ERR("CDev::open failed");
			delete sd;
		}

		return ret;
	}

	if (filp->f_oflags == 0) {
		return CDev::open(filp);
	}

	/* can only be pub or sub, not both */
	return -EINVAL;
}

int
uORB::DeviceNode::close(cdev::file_t *filp)
{
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

	return CDev::close(filp);
}

ssize_t
uORB::DeviceNode::read(cdev::file_t *filp, char *buffer, size_t buflen)
{
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
	ATOMIC_ENTER;

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

	/*
	 * Clear the flag that indicates that an update has been reported, as
	 * we have just collected it.
	 */
	sd->set_update_reported(false);

	ATOMIC_LEAVE;

	return _meta->o_size;
}

ssize_t
uORB::DeviceNode::write(cdev::file_t *filp, const char *buffer, size_t buflen)
{
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
#ifdef __PX4_NUTTX

		if (!up_interrupt_context()) {

			lock();

			/* re-check size */
			if (nullptr == _data) {
				_data = new uint8_t[_meta->o_size * _queue_size];
			}

			unlock();
		}

#else
		lock();

		/* re-check size */
		if (nullptr == _data) {
			_data = new uint8_t[_meta->o_size * _queue_size];
		}

		unlock();
#endif

		/* failed or could not allocate */
		if (nullptr == _data) {
			return -ENOMEM;
		}
	}

	/* If write size does not match, that is an error */
	if (_meta->o_size != buflen) {
		return -EIO;
	}

	/* Perform an atomic copy. */
	ATOMIC_ENTER;
	memcpy(_data + (_meta->o_size * (_generation % _queue_size)), buffer, _meta->o_size);

	/* update the timestamp and generation count */
	_last_update = hrt_absolute_time();
	/* wrap-around happens after ~49 days, assuming a publisher rate of 1 kHz */
	_generation++;

	_published = true;

	ATOMIC_LEAVE;

	/* notify any poll waiters */
	poll_notify(POLLIN);

	return _meta->o_size;
}

int
uORB::DeviceNode::ioctl(cdev::file_t *filp, int cmd, unsigned long arg)
{
	SubscriberData *sd = filp_to_sd(filp);

	switch (cmd) {
	case ORBIOCLASTUPDATE: {
			ATOMIC_ENTER;
			*(hrt_abstime *)arg = _last_update;
			ATOMIC_LEAVE;
			return PX4_OK;
		}

	case ORBIOCUPDATED:
#ifndef __PX4_NUTTX
		lock();
#endif
		*(bool *)arg = appears_updated(sd);
#ifndef __PX4_NUTTX
		unlock();
#endif
		return PX4_OK;

	case ORBIOCSETINTERVAL: {
			int ret = PX4_OK;
			lock();

			if (arg == 0) {
				if (sd->update_interval) {
					delete (sd->update_interval);
					sd->update_interval = nullptr;
				}

			} else {
				if (sd->update_interval) {
					sd->update_interval->interval = arg;
#ifndef __PX4_NUTTX
					sd->update_interval->last_update = hrt_absolute_time();
#endif

				} else {
					sd->update_interval = new UpdateIntervalData();

					if (sd->update_interval) {
						memset(&sd->update_interval->update_call, 0, sizeof(hrt_call));
						sd->update_interval->interval = arg;
#ifndef __PX4_NUTTX
						sd->update_interval->last_update = hrt_absolute_time();
#endif

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
		*(int *)arg = get_priority();
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

	case ORBIOCISPUBLISHED:
		*(unsigned long *)arg = _published;

		return OK;

	default:
		/* give it to the superclass */
		return CDev::ioctl(filp, cmd, arg);
	}
}

ssize_t
uORB::DeviceNode::publish(const orb_metadata *meta, orb_advert_t handle, const void *data)
{
	uORB::DeviceNode *devnode = (uORB::DeviceNode *)handle;
	int ret;

	/* check if the device handle is initialized and data is valid */
	if ((devnode == nullptr) || (meta == nullptr) || (data == nullptr)) {
		errno = EFAULT;
		return PX4_ERROR;
	}

	/* check if the orb meta data matches the publication */
	if (devnode->_meta != meta) {
		errno = EINVAL;
		return PX4_ERROR;
	}

	/* call the devnode write method with no file pointer */
	ret = devnode->write(nullptr, (const char *)data, meta->o_size);

	if (ret < 0) {
		errno = -ret;
		return PX4_ERROR;
	}

	if (ret != (int)meta->o_size) {
		errno = EIO;
		return PX4_ERROR;
	}

#ifdef ORB_COMMUNICATOR
	/*
	 * if the write is successful, send the data over the Multi-ORB link
	 */
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();

	if (ch != nullptr) {
		if (ch->send_message(meta->o_name, meta->o_size, (uint8_t *)data) != 0) {
			PX4_ERR("Error Sending [%s] topic data over comm_channel", meta->o_name);
			return PX4_ERROR;
		}
	}

#endif /* ORB_COMMUNICATOR */

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

#ifdef ORB_COMMUNICATOR
int16_t uORB::DeviceNode::topic_advertised(const orb_metadata *meta, int priority)
{
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();

	if (ch != nullptr && meta != nullptr) {
		return ch->topic_advertised(meta->o_name);
	}

	return -1;
}

/*
//TODO: Check if we need this since we only unadvertise when things all shutdown and it doesn't actually remove the device
int16_t uORB::DeviceNode::topic_unadvertised(const orb_metadata *meta, int priority)
{
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();
	if (ch != nullptr && meta != nullptr) {
		return ch->topic_unadvertised(meta->o_name);
	}
	return -1;
}
*/
#endif /* ORB_COMMUNICATOR */

pollevent_t
uORB::DeviceNode::poll_state(cdev::file_t *filp)
{
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
	SubscriberData *sd = filp_to_sd((cdev::file_t *)fds->priv);

	/*
	 * If the topic looks updated to the subscriber, go ahead and notify them.
	 */
	if (appears_updated(sd)) {
		CDev::poll_notify_one(fds, events);
	}
}

#ifdef __PX4_NUTTX
bool
uORB::DeviceNode::appears_updated(SubscriberData *sd)
{
	/* assume it doesn't look updated */
	bool ret = false;

	/* avoid racing between interrupt and non-interrupt context calls */
	irqstate_t state = px4_enter_critical_section();

	/* check if this topic has been published yet, if not bail out */
	if (_data == nullptr) {
		ret = false;
		goto out;
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

		/*
		 * If the interval timer is still running, the topic should not
		 * appear updated, even though at this point we know that it has.
		 * We have previously been through here, so the subscriber
		 * must have collected the update we reported, otherwise
		 * update_reported would still be true.
		 */
		if (!hrt_called(&sd->update_interval->update_call)) {
			break;
		}

		/*
		 * Make sure that we don't consider the topic to be updated again
		 * until the interval has passed once more by restarting the interval
		 * timer and thereby re-scheduling a poll notification at that time.
		 */
		hrt_call_after(&sd->update_interval->update_call,
			       sd->update_interval->interval,
			       &uORB::DeviceNode::update_deferred_trampoline,
			       (void *)this);

		/*
		 * Remember that we have told the subscriber that there is data.
		 */
		sd->set_update_reported(true);
		ret = true;

		break;
	}

out:
	px4_leave_critical_section(state);

	/* consider it updated */
	return ret;
}

#else

bool
uORB::DeviceNode::appears_updated(SubscriberData *sd)
{

	/* block if in simulation mode */
	while (px4_sim_delay_enabled()) {
		px4_usleep(100);
	}

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
#endif /* ifdef __PX4_NUTTX */

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

void uORB::DeviceNode::add_internal_subscriber()
{
	lock();
	_subscriber_count++;

#ifdef ORB_COMMUNICATOR
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();

	if (ch != nullptr && _subscriber_count > 0) {
		unlock(); //make sure we cannot deadlock if add_subscription calls back into DeviceNode
		ch->add_subscription(_meta->o_name, 1);

	} else
#endif /* ORB_COMMUNICATOR */

	{
		unlock();
	}
}

void uORB::DeviceNode::remove_internal_subscriber()
{
	lock();
	_subscriber_count--;

#ifdef ORB_COMMUNICATOR
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();

	if (ch != nullptr && _subscriber_count == 0) {
		unlock(); //make sure we cannot deadlock if remove_subscription calls back into DeviceNode
		ch->remove_subscription(_meta->o_name);

	} else
#endif /* ORB_COMMUNICATOR */
	{
		unlock();
	}
}

#ifdef ORB_COMMUNICATOR
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

int16_t uORB::DeviceNode::process_remove_subscription()
{
	return PX4_OK;
}

int16_t uORB::DeviceNode::process_received_message(int32_t length, uint8_t *data)
{
	int16_t ret = -1;

	if (length != (int32_t)(_meta->o_size)) {
		PX4_ERR("Received '%s' with DataLength[%d] != ExpectedLen[%d]", _meta->o_name, (int)length, (int)_meta->o_size);
		return PX4_ERROR;
	}

	/* call the devnode write method with no file pointer */
	ret = write(nullptr, (const char *)data, _meta->o_size);

	if (ret < 0) {
		return PX4_ERROR;
	}

	if (ret != (int)_meta->o_size) {
		errno = EIO;
		return PX4_ERROR;
	}

	return PX4_OK;
}
#endif /* ORB_COMMUNICATOR */

int uORB::DeviceNode::update_queue_size(unsigned int queue_size)
{
	if (_queue_size == queue_size) {
		return PX4_OK;
	}

	//queue size is limited to 255 for the single reason that we use uint8 to store it
	if (_data || _queue_size > queue_size || queue_size > 255) {
		return PX4_ERROR;
	}

	_queue_size = queue_size;
	return PX4_OK;
}
