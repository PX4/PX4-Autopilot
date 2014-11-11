/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
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
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <containers/List.hpp>
#include <uavcan/uavcan.hpp>
#include <drivers/device/device.h>
#include <drivers/drv_orb_dev.h>

/**
 * A sensor bridge class must implement this interface.
 */
class IUavcanSensorBridge : uavcan::Noncopyable, public ListNode<IUavcanSensorBridge*>
{
public:
	static constexpr unsigned MAX_NAME_LEN = 20;

	virtual ~IUavcanSensorBridge() { }

	/**
	 * Returns ASCII name of the bridge.
	 */
	virtual const char *get_name() const = 0;

	/**
	 * Starts the bridge.
	 * @return Non-negative value on success, negative on error.
	 */
	virtual int init() = 0;

	/**
	 * Returns number of active redundancy channels.
	 */
	virtual unsigned get_num_redundant_channels() const = 0;

	/**
	 * Prints current status in a human readable format to stdout.
	 */
	virtual void print_status() const = 0;

	/**
	 * Sensor bridge factory.
	 * Creates a bridge object by its ASCII name, e.g. "gnss", "mag".
	 * @return nullptr if such bridge can't be created.
	 */
	static void make_all(uavcan::INode &node, List<IUavcanSensorBridge*> &list);
};

/**
 * This is the base class for redundant sensors with an independent ORB topic per each redundancy channel.
 * For example, sensor_mag0, sensor_mag1, etc.
 */
class UavcanCDevSensorBridgeBase : public IUavcanSensorBridge, public device::CDev
{
	struct Channel
	{
		int node_id              = -1;
		orb_id_t orb_id          = nullptr;
		orb_advert_t orb_advert  = -1;
		int class_instance       = -1;
	};

	const unsigned _max_channels;
	const char *const _class_devname;
	orb_id_t *const _orb_topics;
	Channel *const _channels;
	bool _out_of_channels = false;

protected:
	template <unsigned MaxChannels>
	UavcanCDevSensorBridgeBase(const char *name, const char *devname, const char *class_devname,
	                           const orb_id_t (&orb_topics)[MaxChannels]) :
	device::CDev(name, devname),
	_max_channels(MaxChannels),
	_class_devname(class_devname),
	_orb_topics(new orb_id_t[MaxChannels]),
	_channels(new Channel[MaxChannels])
	{
		memcpy(_orb_topics, orb_topics, sizeof(orb_id_t) * MaxChannels);
		_device_id.devid_s.bus_type = DeviceBusType_UAVCAN;
		_device_id.devid_s.bus = 0;
	}

	/**
	 * Sends one measurement into appropriate ORB topic.
	 * New redundancy channels will be registered automatically.
	 * @param node_id Sensor's Node ID
	 * @param report  Pointer to ORB message object
	 */
	void publish(const int node_id, const void *report);

public:
	virtual ~UavcanCDevSensorBridgeBase();

	unsigned get_num_redundant_channels() const override;

	void print_status() const override;
};
