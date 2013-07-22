/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file can.cpp
 *
 * Base class and infrastructure for devices connected via CAN.
 */

#include <string.h>

#include "can.h"
#include <systemlib/uthash/utlist.h>

namespace device 
{

CAN::Bus CAN::_bus_array[CAN::_maxbus];

int
CAN::connect(can_dev_s *dev, unsigned bus, unsigned tx_queue)
{
	if (bus >= _maxbus)
		return -ERANGE;
	if (_bus_array[bus].dev != nullptr)
		return -EBUSY;
	MsgQ *txq = new MsgQ(tx_queue);
	if (txq == nullptr)
		return -ENOMEM;
	
	_bus_array[bus].dev = dev;
	_bus_array[bus].tx_queue = txq;

	return 0;
}

CAN::CAN(const char *name, 
    const char *devname,
    unsigned bus) :
	CDev(name, devname),
	_bus(bus),
	_filter_next(nullptr),
	_rx_queue(nullptr)
{
	_filter_add();
}

CAN::~CAN()
{
	_filter_remove();

	if (_rx_queue != nullptr)
		delete _rx_queue;
}

void
CAN::_filter_msg(unsigned bus, can_msg_s &msg)
{
	CAN	*drv = _filter_head;

	/* offer the message to each driver on the bus */
	while (drv != nullptr) {
		if ((bus == drv->_bus) && drv->filter(msg))
			drv->_enqueue(msg);
		drv = drv->_filter_next;
	}
}

void
CAN::_filter_add()
{
	/* push us onto the head of the filter list */
	_filter_next = _filter_head;
	_filter_head = this;
}

void
CAN::_filter_remove()
{
	CAN	**drvp = &_filter_head;

	/* scan the filter list, remove us from it */
	while (*drvp != nullptr) {
		if (*drvp == this) {
			*drvp = _filter_next;
			return;
		}
		drvp = &(*drvp)->_filter_next;
	}
}

int
CAN::can_receive(struct can_dev_s *dev, struct can_hdr_s *hdr, uint8_t *data)
{
	/* find the bus this device owns */
	for (unsigned bus = 0; bus < _maxbus; bus++) {
		if (_bus_array[bus].dev == dev) {

			/* massage the message into our buffer structure */
			can_msg_s msg;
			msg.cm_hdr = *hdr;
			memcpy(&msg.cm_data, data, CAN_MAXDATALEN);

			/* and offer it to each of the drivers */
			_filter_msg(bus, msg);
		}
	}
	return OK;
}

int
CAN::can_txdone(struct can_dev_s *dev)
{
	/* find the bus this device owns */
	for (unsigned bus = 0; bus < _maxbus; bus++) {
		if (_bus_array[bus].dev == dev) {
			MsgQ *txq = _bus_array[bus].tx_queue;
			bool sent = false;

			/* send pending messages */
			/* XXX need locking here */
			while (dev->cd_ops->co_txready(dev)) {
				can_msg_s msg;

				if (!txq->get(msg))
					break;

				/* XXX this should never fail - would have to push back otherwise */
				dev->cd_ops->co_send(dev, &msg);
				sent = true;
			}

			if (sent) {
				/* wake up blocked senders on this bus */
				CAN *drv = _filter_head;

				while (drv != nullptr) {
					if (drv->_bus == bus)
						drv->poll_notify(POLLOUT);
				}
			}
		}
	}
	return OK;
}

ssize_t
CAN::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(can_msg_s);
	unsigned index;
	can_msg_s *bufs = reinterpret_cast<can_msg_s *>(buffer);
	ssize_t ret = 0;

	for (index = 0; index < count; index++) {
		bool result = receive(bufs[count]);

		if (!result) {
			if (index == 0) {
				/* XXX handle blocking here */
			}
			break;
		}
	}
	ret = index * sizeof(can_msg_s);

	return ret;
}

ssize_t
CAN::write(struct file *filp, const char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(can_msg_s);
	const can_msg_s *bufs = reinterpret_cast<const can_msg_s *>(buffer);
	ssize_t ret = 0;

	for (unsigned index = 0; index < count; index++) {
		ret = send(bufs[count]);

		switch (ret) {
		case 0:
			/* success, go again */
			break;

		case -EWOULDBLOCK:
			/* XXX handle blocking here */
			ret = (count * sizeof(can_msg_s));

			/* FALLTHROUGH */
		default:
			goto out;
		}
	}

out:
	return ret;
}

int
CAN::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	return CDev::ioctl(filp, cmd, arg);
}

pollevent_t
CAN::poll_state(struct file *filp)
{
	pollevent_t pe = 0;

	if (!_rx_queue->empty())
		pe |= POLLIN;
	if (!_bus_array[_bus].tx_queue->full())
		pe |= POLLOUT;

	return pe;
}

int
CAN::init()
{
	return CDev::init();
}

int
CAN::probe()
{
	/* assume we can't know it's there */
	return true;
}

bool
CAN::filter(can_msg_s &msg)
{
	/* assume we aren't interested in messages */
	return false;
}

int
CAN::send(const can_msg_s &msg)
{
	int ret;

	/* XXX need locking here */
	can_dev_s *dev;
	dev = _bus_array[_bus].dev;

	/* try sending directly if nothing is queued */
	MsgQ *txq = _bus_array[_bus].tx_queue;
	if (txq->empty()) {
		ret = dev->cd_ops->co_send(dev, const_cast<can_msg_s *>(&msg));

		/* for any result other than "no room to send", return it */
		if (ret != -EBUSY)
			goto out;
	}

	/* queue the message for later transmission */
	if (!txq->put(msg)) {
		ret = -ENOSPC;
		goto out;
	}
	ret = 0;

	/* unlock here */
out:
	return ret;
}

bool
CAN::receive(can_msg_s &msg)
{
	/* pull an entry from the receive queue */
	return _rx_queue->get(msg);
}

int
CAN::set_rx_queue(unsigned size)
{
	/* allocate the new queue */
	MsgQ *mq = new MsgQ(size);
	if (mq == nullptr)
		return -ENOMEM;

	/* swap the queue in place of the old one */
	MsgQ *oq = _rx_queue;
	_rx_queue = mq;

	/* and delete the old queue */
	delete oq;

	return 0;
}

void
CAN::_enqueue(const can_msg_s &msg)
{
	/* put the message into the queue */
	_rx_queue->put(msg);

	/* and notify anyone that is looking for it */
	poll_notify(POLLIN);
}


} /* namespace */

/*
 * Thunk from the NuttX CAN generic callbacks to our implementation.
 */
 
int
can_receive(FAR struct can_dev_s *dev, FAR struct can_hdr_s *hdr, FAR uint8_t *data)
{
	return device::CAN::can_receive(dev, hdr, data);
}

int
can_txdone(FAR struct can_dev_s *dev)
{
	return device::CAN::can_txdone(dev);
}
