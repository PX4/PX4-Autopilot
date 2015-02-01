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

#include "sensor_bridge.hpp"
#include <cassert>

#include "gnss.hpp"
#include "mag.hpp"
#include "baro.hpp"

/*
 * IUavcanSensorBridge
 */
void IUavcanSensorBridge::make_all(uavcan::INode &node, List<IUavcanSensorBridge*> &list)
{
	list.add(new UavcanBarometerBridge(node));
	list.add(new UavcanMagnetometerBridge(node));
	list.add(new UavcanGnssBridge(node));
}

/*
 * UavcanCDevSensorBridgeBase
 */
UavcanCDevSensorBridgeBase::~UavcanCDevSensorBridgeBase()
{
	for (unsigned i = 0; i < _max_channels; i++) {
		if (_channels[i].node_id >= 0) {
			(void)unregister_class_devname(_class_devname, _channels[i].class_instance);
		}
	}
	delete [] _channels;
}

void UavcanCDevSensorBridgeBase::publish(const int node_id, const void *report)
{
	assert(report != nullptr);

	Channel *channel = nullptr;

	// Checking if such channel already exists
	for (unsigned i = 0; i < _max_channels; i++) {
		if (_channels[i].node_id == node_id) {
			channel = _channels + i;
			break;
		}
	}

	// No such channel - try to create one
	if (channel == nullptr) {
		if (_out_of_channels) {
			return;           // Give up immediately - saves some CPU time
		}

		log("adding channel %d...", node_id);

		// Search for the first free channel
		for (unsigned i = 0; i < _max_channels; i++) {
			if (_channels[i].node_id < 0) {
				channel = _channels + i;
				break;
			}
		}

		// No free channels left
		if (channel == nullptr) {
			_out_of_channels = true;
			log("out of channels");
			return;
		}

		// update device id as we now know our device node_id
		_device_id.devid_s.address = static_cast<uint8_t>(node_id);

		// Ask the CDev helper which class instance we can take
		const int class_instance = register_class_devname(_class_devname);
		if (class_instance < 0 || class_instance >= int(_max_channels)) {
			_out_of_channels = true;
			log("out of class instances");
			(void)unregister_class_devname(_class_devname, class_instance);
			return;
		}

		// Publish to the appropriate topic, abort on failure
		channel->node_id        = node_id;
		channel->class_instance = class_instance;

		channel->orb_advert = orb_advertise_multi(_orb_topic, report, &channel->orb_instance, ORB_PRIO_HIGH);
		if (channel->orb_advert < 0) {
			log("ADVERTISE FAILED");
			(void)unregister_class_devname(_class_devname, class_instance);
			*channel = Channel();
			return;
		}

		log("channel %d class instance %d ok", channel->node_id, channel->class_instance);
	}
	assert(channel != nullptr);

	(void)orb_publish(_orb_topic, channel->orb_advert, report);
}

unsigned UavcanCDevSensorBridgeBase::get_num_redundant_channels() const
{
	unsigned out = 0;
	for (unsigned i = 0; i < _max_channels; i++) {
		if (_channels[i].node_id >= 0) {
			out += 1;
		}
	}
	return out;
}

void UavcanCDevSensorBridgeBase::print_status() const
{
	printf("devname: %s\n", _class_devname);

	for (unsigned i = 0; i < _max_channels; i++) {
		if (_channels[i].node_id >= 0) {
			printf("channel %d: node id %d --> class instance %d\n",
			       i, _channels[i].node_id, _channels[i].class_instance);
		} else {
			printf("channel %d: empty\n", i);
		}
	}
}
