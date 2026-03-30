/****************************************************************************
*
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "node_info.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <drivers/drv_hrt.h>
#include <cstring>

using namespace time_literals;

NodeInfoPublisher::NodeInfoPublisher(uavcan::INode &node, uavcan::NodeInfoRetriever &node_info_retriever)
	: TimerBase(node), _node_info_retriever(node_info_retriever)
{
	_node_info_retriever.addListener(this);
}

NodeInfoPublisher::~NodeInfoPublisher()
{
	_node_info_retriever.removeListener(this);
	delete[] _device_informations;
}

void NodeInfoPublisher::handleNodeInfoRetrieved(uavcan::NodeID node_id, const uavcan::protocol::GetNodeInfo_::Response &node_info)
{
	NodeVendor vendor = NodeVendor::UNKNOWN;

	if (strstr(node_info.name.c_str(), "iq_motion") != nullptr) {
		vendor = NodeVendor::VERTIQ;
	}

	_node_vendors[node_id.get()] = vendor;

	registerNodeInfo(node_id.get(), node_info);
	startTimerIfNotRunning();
}

void NodeInfoPublisher::handleNodeInfoUnavailable(uavcan::NodeID node_id)
{
}

void NodeInfoPublisher::handleTimerEvent(const uavcan::TimerEvent &event)
{
	// Publish device information using round-robin approach
	publishDeviceInformationPeriodic();
}

void NodeInfoPublisher::startTimerIfNotRunning()
{
	if (!TimerBase::isRunning()) {
		TimerBase::startPeriodic(uavcan::MonotonicDuration::fromMSec(DEVICE_INFO_PUBLISH_INTERVAL_MS));
	}
}

void NodeInfoPublisher::registerNodeInfo(uint8_t node_id, const uavcan::protocol::GetNodeInfo_::Response &node_info)
{
	bool found = false;

	for (size_t i = 0; i < _device_informations_size; ++i) {
		if (_device_informations[i].node_id != node_id) { continue; }

		found = true;

		if (!_device_informations[i].has_node_info) {
			populateDeviceInfoFields(_device_informations[i], node_info);

			if (_device_informations[i].device_type != DEVICE_TYPE_NONE) {
				publishSingleDeviceInformation(_device_informations[i]);
			}
		}
	}

	if (!found && extendDeviceInformationsArray()) {
		_device_informations[_device_informations_size - 1] = DeviceInformation();
		_device_informations[_device_informations_size - 1].node_id = node_id;
		populateDeviceInfoFields(_device_informations[_device_informations_size - 1], node_info);
	}
}

void NodeInfoPublisher::registerCapability(uint8_t node_id, uint32_t device_id, uint8_t device_type)
{
	int multi_capability_index = -1;

	for (size_t i = 0; i < _device_informations_size; ++i) {
		if (_device_informations[i].node_id != node_id) { continue; }

		// Exact match — nothing to do
		if (_device_informations[i].device_type == device_type &&
		    _device_informations[i].device_id == device_id) {
			return;
		}

		// Different device_type on same node — remember for multi-capability copy
		if (_device_informations[i].device_type != DEVICE_TYPE_NONE &&
		    _device_informations[i].device_type != device_type) {
			multi_capability_index = i;
			continue;
		}

		// No device_type yet but has node info — fill it in and publish
		if (_device_informations[i].device_type == DEVICE_TYPE_NONE &&
		    _device_informations[i].has_node_info) {
			_device_informations[i].device_id = device_id;
			_device_informations[i].device_type = device_type;
			publishSingleDeviceInformation(_device_informations[i]);
			return;
		}
	}

	// No existing entry to update — create a new one
	if (extendDeviceInformationsArray()) {
		if (multi_capability_index >= 0) {
			_device_informations[_device_informations_size - 1] = _device_informations[multi_capability_index];

		} else {
			_device_informations[_device_informations_size - 1] = DeviceInformation();
		}

		_device_informations[_device_informations_size - 1].node_id = node_id;
		_device_informations[_device_informations_size - 1].device_id = device_id;
		_device_informations[_device_informations_size - 1].device_type = device_type;

	} else {
		PX4_DEBUG("Failed to extend device informations array for capability");
	}
}

void NodeInfoPublisher::registerDeviceCapability(uint8_t node_id, uint32_t device_id, uint8_t device_type)
{
	if (node_id < 1 || node_id > uavcan::NodeID::Max) { return; }

	registerCapability(node_id, device_id, device_type);
}

void NodeInfoPublisher::publishDeviceInformationPeriodic()
{
	// Using round-robin approach to publish one device info per timer event
	if (_device_informations_size == 0) {
		return;
	}

	size_t devices_checked = 0;

	while (devices_checked < _device_informations_size) {
		if (_next_device_to_publish >= _device_informations_size) {
			_next_device_to_publish = 0;
		}

		const auto &device_info = _device_informations[_next_device_to_publish];

		if (device_info.has_node_info && device_info.device_type != DEVICE_TYPE_NONE) {
			publishSingleDeviceInformation(device_info);
			_next_device_to_publish++;
			return;
		}

		_next_device_to_publish++;
		devices_checked++;
	}

	PX4_DEBUG("No devices ready for periodic publishing");
}

void NodeInfoPublisher::publishSingleDeviceInformation(const DeviceInformation &device_info)
{
	const uint64_t now = hrt_absolute_time();

	device_information_s msg{};
	msg.timestamp = now;
	msg.device_type = device_info.device_type;
	msg.device_id = device_info.device_id;

	// Copy name and serial directly
	static_assert(sizeof(msg.name) == sizeof(device_info.name), "Array size mismatch");
	static_assert(sizeof(msg.serial_number) == sizeof(device_info.serial_number), "Array size mismatch");

	memcpy(msg.name, device_info.name, sizeof(msg.name));
	msg.name[sizeof(msg.name) - 1] = '\0';

	memcpy(msg.serial_number, device_info.serial_number, sizeof(msg.serial_number));
	msg.serial_number[sizeof(msg.serial_number) - 1] = '\0';

	// Format version integers to strings at publish time
	snprintf(msg.firmware_version, sizeof(msg.firmware_version),
		 "%d.%d.%lu", device_info.sw_major, device_info.sw_minor,
		 static_cast<unsigned long>(device_info.sw_vcs_commit));
	snprintf(msg.hardware_version, sizeof(msg.hardware_version),
		 "%d.%d", device_info.hw_major, device_info.hw_minor);

	_device_info_pub.publish(msg);

	PX4_DEBUG("Published device info for node %d, device_id %lu, type %d",
		  device_info.node_id, static_cast<unsigned long>(device_info.device_id),
		  static_cast<int>(device_info.device_type));
}

void NodeInfoPublisher::populateDeviceInfoFields(DeviceInformation &device_info, const uavcan::protocol::GetNodeInfo_::Response &node_info)
{
	device_info.has_node_info = true;

	snprintf(device_info.name, sizeof(device_info.name), "%s", node_info.name.c_str());

	device_info.sw_major = node_info.software_version.major;
	device_info.sw_minor = node_info.software_version.minor;
	device_info.sw_vcs_commit = node_info.software_version.vcs_commit;
	device_info.hw_major = node_info.hardware_version.major;
	device_info.hw_minor = node_info.hardware_version.minor;

	const auto &uid = node_info.hardware_version.unique_id;
	snprintf(device_info.serial_number, sizeof(device_info.serial_number),
		 "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x",
		 uid[0], uid[1], uid[2], uid[3], uid[4], uid[5], uid[6], uid[7],
		 uid[8], uid[9], uid[10], uid[11], uid[12], uid[13], uid[14], uid[15]);
}

bool NodeInfoPublisher::extendDeviceInformationsArray()
{
	const size_t new_size = _device_informations_size + 1;
	DeviceInformation *new_array = new DeviceInformation[new_size];

	if (!new_array) {
		return false;
	}

	if (_device_informations_size > 0 && _device_informations != nullptr) {
		memcpy(new_array, _device_informations, _device_informations_size * sizeof(DeviceInformation));
		delete[] _device_informations;
	}

	_device_informations = new_array;
	_device_informations_size = new_size;
	return true;
}
