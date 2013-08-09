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

#include <nuttx/config.h>

#include <string.h>
#include <stdio.h>

#include <drivers/drv_can.h>

#include "can.h"

namespace device 
{

/****************************************************************************
 * CANBus - Generic bus driver
 ****************************************************************************/

CANBus *CANBus::_bus_array[CANBus::_maxbus];

CANBus::CANBus(const char *devname, unsigned bus_number, can_dev_s *dev) :
	CAN("CANBus", devname, this),
	_bus_number(bus_number),
	_dev(dev),
	_tx_queue(nullptr),
	_drivers(nullptr)
{
}

CANBus::~CANBus()
{
}

int
CANBus::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int ret;

	switch (cmd) {

	default:
		ret = CAN::ioctl(filp, cmd, arg);
	}
	return ret;
}

int
CANBus::init()
{
	if (_bus_number >= _maxbus)
		return -ERANGE;
	if (_bus_array[_bus_number] != nullptr)
		return -EBUSY;

	_tx_queue = new MsgQ(_default_txq);
	if (_tx_queue == nullptr)
		return -ENOMEM;
		
	_bus_array[_bus_number] = this;

	/* put ourself on our own bus */
	attach(this);

	return OK;
}

CANBus *
CANBus::for_bus(unsigned bus_number, can_dev_s *dev)
{
	if (bus_number >= _maxbus)
		return nullptr;

	/* if we have already allocated the bus, go ahead and return it */
	if (_bus_array[bus_number] != nullptr)
		return _bus_array[bus_number];

	/* if we don't have a device, we can't lazily construct the bus driver */
	if (dev == nullptr)
		return nullptr;

	char devname[32];
	sprintf(devname, "/dev/can%d", bus_number);

	CANBus *bus = new CANBus(devname, bus_number, dev);

	if (bus != nullptr) {
		if (bus->init() != OK) {
			delete bus;
			bus = nullptr;
		}
	}
	return bus;
}

void
CANBus::attach(CAN *driver)
{
	CRITICAL_SECTION;

	/* push the driver onto the head of the filter list */
	driver->filter_next = _drivers;
	_drivers = driver;
}

void
CANBus::detach(CAN *driver)
{
	CAN	**drvp = &_drivers;

	CRITICAL_SECTION;
	
	/* scan the filter list, remove us from it */
	while (*drvp != nullptr) {
		if (*drvp == driver) {
			*drvp = driver->filter_next;
			break;
		}
		drvp = &(*drvp)->filter_next;
	}
}

int
CANBus::send(const can_msg_s &msg)
{
	int ret;

	/* try sending directly if nothing is queued */
	if (_tx_queue->empty()) {
	ret = _dev->cd_ops->co_send(_dev, const_cast<can_msg_s *>(&msg));

		/* for any result other than "no room to send", return it */
		if (ret != -EBUSY)
			goto out;
	}

	/* queue the message for later transmission */
	if (!_tx_queue->put(msg)) {
		ret = -ENOSPC;
		goto out;
	}
	ret = OK;

out:
	return ret;
}

unsigned
CANBus::send_space()
{
	return _tx_queue->space();
}

int
CANBus::can_receive(can_dev_s *dev, can_hdr_s *hdr, uint8_t *data)
{
	/* find a bus willing to handle this device's inbound traffic */
	CANBus *cb = _bus_for_dev(dev);

	if (cb != nullptr) {

		/* massage the message into our buffer structure */
		can_msg_s msg;
		msg.cm_hdr = *hdr;
		memcpy(&msg.cm_data, data, CAN_MAXDATALEN);

		/* and offer it to each of the drivers */
		cb->_filter_msg(msg);
	}
	return OK;
}

int
CANBus::can_txdone(can_dev_s *dev)
{
	CANBus *cb = _bus_for_dev(dev);

	if (cb != nullptr)
		cb->_txdone();

	return OK;
}

int
CANBus::open_first(struct file *filp)
{
	/* allocate a small RX queue */
	set_rx_queue(4);

	return CDev::open_first(filp);
}

int
CANBus::close_last(struct file *filp)
{
	/* free the RX queue */
	set_rx_queue(0);

	return CDev::close_last(filp);
}

bool
CANBus::filter(const can_msg_s &msg)
{
	/* if we are open, we're interested in everything */
	return is_open();
}

CANBus *
CANBus::_bus_for_dev(can_dev_s *dev)
{
	/* find the bus this device owns - this is a bad impedance match to the NuttX driver */
	for (unsigned bus = 0; bus < _maxbus; bus++) {
		CANBus *cb = _bus_array[bus];

		/* is this bus handling messages from this dev? */
		if (cb->_dev == dev)
			return cb;
	}
	return nullptr;
}

void
CANBus::_filter_msg(const can_msg_s &msg)
{
	CAN	*drv = _drivers;

	/* offer the message to each driver on the bus */
	while (drv != nullptr) {
		if (drv->filter(msg))
			drv->enqueue(msg);
		drv = drv->filter_next;
	}
}

void
CANBus::_txdone()
{
	bool sent = false;

	/* send pending messages */
	while (_dev->cd_ops->co_txready(_dev)) {
		can_msg_s msg;

		if (!_tx_queue->get(msg))
			break;

		/* this should never fail - would have to push back otherwise */
		_dev->cd_ops->co_send(_dev, &msg);
		sent = true;
	}

	if (sent) {
		/* wake up blocked senders on this bus */
		CAN *drv = _drivers;

		while (drv != nullptr) {
			drv->poll_notify(POLLOUT);
			drv = drv->filter_next;
		}
	}
}

/****************************************************************************
 * CAN - abstract CAN device class
 ****************************************************************************/

CAN::CAN(const char *name, 
    const char *devname,
    CANBus *bus) :
	CDev(name, devname),
	filter_next(nullptr),
	_bus(bus),
	_rx_queue(nullptr)
{
}

CAN::~CAN()
{
	if (_bus != nullptr)
		_bus->detach(this);

	if (_rx_queue != nullptr)
		delete _rx_queue;
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
	int ret = OK;

	switch (cmd) {
	case CANIOCSETRXBUF:
		ret = set_rx_queue(arg);
		break;

	case CANIOCGETRXBUF:
		*(reinterpret_cast<unsigned *>(arg)) = _rx_queue ? _rx_queue->size() : 0;
		break;

	default:
		ret = CDev::ioctl(filp, cmd, arg);
	}
	return ret;
}

int
CAN::init()
{
	return CDev::init();
}

pollevent_t
CAN::poll_state(struct file *filp)
{
	pollevent_t pe = 0;

	CRITICAL_SECTION;

	if (!_rx_queue->empty())
		pe |= POLLIN;
	if (_bus->send_space() > 0)
		pe |= POLLOUT;

	return pe;
}

int
CAN::probe()
{
	/* assume we can't know it's there */
	return true;
}

bool
CAN::filter(const can_msg_s &msg)
{
	/* assume we aren't interested in messages */
	return false;
}

int
CAN::send(const can_msg_s &msg)
{
	return _bus->send(msg);
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
	MsgQ *oq, *mq = nullptr;
	int ret = OK;

	lock();

	if (size > 0) {
		/* allocate the new queue */
		mq = new MsgQ(size);
		if (mq == nullptr) {
			ret = -ENOMEM;
			goto out;
		}
	}

	/* swap the queue in place of the old one */
	oq = _rx_queue;
	_rx_queue = mq;

	/* and delete the old queue */
	if (oq != nullptr)
		delete oq;

out:
	unlock();
	return ret;
}

void
CAN::enqueue(const can_msg_s &msg)
{
	/* put the message into the queue */
	if (_rx_queue != nullptr) {
		_rx_queue->put(msg);

		/* and notify anyone that is looking for it */
		poll_notify(POLLIN);
	}
}


} /* namespace */

/*
 * Thunk from the NuttX CAN generic callbacks to our implementation.
 */
 
int
can_receive(FAR can_dev_s *dev, FAR can_hdr_s *hdr, FAR uint8_t *data)
{
	return device::CANBus::can_receive(dev, hdr, data);
}

int
can_txdone(FAR can_dev_s *dev)
{
	return device::CANBus::can_txdone(dev);
}
