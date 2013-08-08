/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
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
 * @file can.h
 *
 * Base class and infrastructure for devices connected via CAN.
 *
 * Note; using this class overrides the stock NuttX CAN stack with a compatible
 * version that adds support for multiple clients, poll(), etc.
 */

#pragma once

#include <nuttx/can.h>
#include "device.h"
#include "ringbuffer.h"

namespace device __EXPORT
{

class CANBus;

/**
 * Abstract class for character device connected via CAN.
 */
class __EXPORT CAN : public CDev
{
public:

	/*
	 * The CAN device class implements sensible read/write/poll semantics
	 * for devices with minimal assistance.
	 */
	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual ssize_t		write(struct file *filp, const char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	virtual int		init();

protected:

	typedef RingBuffer<can_msg_s> MsgQ;	/**< ringbuffer containing messages */

	CAN			*filter_next;	/**< next driver in list */
	CANBus			*bus;		/**< bus we are attached to

	/**
	 * Constructor
	 *
	 * @param name		Driver name
	 * @param devname	Device node name
	 * @param bus		CAN bus to which the driver will be attached
	 */
	CAN(const char *name, 
	    const char *devname,
	    unsigned bus);
	virtual ~CAN();

	virtual pollevent_t	poll_state(struct file *filp);

	/**
	 * Check for the presence of the device on the bus.
	 */
	virtual int		probe();

	/**
	 * Examine a received message and decide whether it should be queued for reception.
	 *
	 * This function is called in interrupt context when a message is received; it should
	 * return true if the message is interesting to the driver and should be queued, false 
	 * otherwise.
	 *
	 * Note that even if this function returns true, the message will be dropped if the 
	 * driver queue is full.
	 *
	 * The default implementation returns false (suitable for a send-only driver).
	 */
	virtual bool		filter(can_msg_s &msg);

	/**
	 * Queue a CAN message for transmission.
	 *
	 * @param msg		The message to be sent.
	 */
	virtual int		send(const can_msg_s &msg);

	/**
	 * Collect the next message from the internal queue.
	 *
	 * @param msg		The message to be received.
	 * @return		True if a message was received, false otherwise.
	 */
	bool			receive(can_msg_s &msg);

	/**
	 * Set the size of the receive queue.
	 *
	 * Any outstanding messages are discarded.
	 *
	 * @param size		The new size of the receive queue.
	 * @return		zero if the queue was resized, -errno otherwise.
	 *			Note that if the resize fails, the queue size remains
	 *			the same.
	 */
	int			set_rx_queue(unsigned size);

	/**
	 * Enqueue a message destined for this driver.
	 *
	 * Normally called by the bus driver.
	 *
	 * @param msg		The message to enqueue.
	 */
	void			enqueue(const can_msg_s &msg);

private:

	MsgQ			*_rx_queue;	/**< buffer of received messages */
};

/****************************************************************************
 * CAN bus device
 *
 * The bus device provides generic read/write/poll access to the CAN bus.
 *
 * If more than one client has the bus open for reading, received messages
 * will be handed out on a first-come first-served basis.
 */


class CANBus :: public CAN
{
public:
	virtual ~CANBus();

	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	virtual int		init();

	/**
	 * Lookup / lazy factory method.
	 *
	 * @param bus		Bus to look up / create.
	 */
	static CANBus		*for_bus(unsigned bus, can_dev_s dev = nullptr);

	/**
	 * Attach a driver to the bus.
	 *
	 * @param driver		Driver to attach.
	 */
	void			attach(CAN *driver);

	/**
	 * Detach a driver from the bus.
	 *
	 * @param driver		Driver to detach.
	 */
	void			detach(CAN *driver);

	/**
	 * Send a message on the bus.
	 *
	 * If the message can't be sent immediately, it may be queued for later transmission.
	 *
	 * @param msg			The message to send.
	 * @return			Zero if the message was sent or queued, -ENOSPC if the
	 *				queue is full.
	 */
	virtual int		send(can_msg_s &msg);

	/* bridges from the NuttX CAN stack replacement functions */
	static int		can_receive(struct can_dev_s *dev, struct can_hdr_s *hdr, uint8_t *data);
	static int		can_txdone(struct can_dev_s *dev);

protected:

	/**
	 * Constructor
	 *
	 * @param devname	Device node to create.
	 * @param bus		Bus number to adopt.
	 */
	CANBus(const char *devname, unsigned bus);

	virtual int		open_first(struct file *filp);
	virtual int		close_last(struct file *filp);

	virtual bool		filter(can_msg_s &msg);

private:

	static const unsigned	_maxbus = 2;	/**< max number of busses we support */
	static const unsigned	_default_txq = 8;

	static CANBus		*_bus_array[_maxbus];	/**< array of bus:device mappings */

	unsigned		_busnum;
	can_dev_s		*_drivers;
	MsgQ			*_tx_queue;
	CAN			*_devs;

	/**
	 * Loop up the CANBus handling a low-level driver
	 *
	 * @param dev		The low-level driver to look up.
	 * @return		The CANBus instance handling the bus, or nullptr if none is assigned.
	 */
	CANBus			*_bus_for_dev(can_dev_s *dev);

	/**
	 * Filter an incoming message past all drivers attached to the bus.
	 *
	 * @param bus		The bus the message was received on.
	 * @param msg		The message that was received.
	 */
	void			_filter_msg(can_msg_s &msg);

	/**
	 * Called when the low-level driver is done transmitting.
	 */
	void			_txdone();

};

} /* namespace */

/*
 * Replacements for the NuttX CAN stack entrypoints.
 */
__BEGIN_DECLS

__EXPORT int can_receive(FAR struct can_dev_s *dev, FAR struct can_hdr_s *hdr, FAR uint8_t *data);
__EXPORT int can_txdone(FAR struct can_dev_s *dev);

__END_DECLS

