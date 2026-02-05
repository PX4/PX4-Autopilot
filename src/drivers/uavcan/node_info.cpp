/****************************************************************************
*
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

void NodeInfoPublisher::handleNodeInfoRetrieved(uavcan::NodeID node_id,
		const uavcan::protocol::GetNodeInfo_::Response &node_info)
{
	const NodeInfo info(node_id, node_info);
	registerDevice(info.node_id.get(), &info, UINT32_MAX, DeviceCapability::NONE);

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

void NodeInfoPublisher::registerDevice(uint8_t node_id, const NodeInfo *info, uint32_t device_id,
				       DeviceCapability capability)
{
	const bool is_registering_info = (info != nullptr);

	int multi_capability_index = -1;

	for (size_t i = 0; i < _device_informations_size; ++i) {
		if (is_registering_info) {
			// Case 1: Check if this entry already has node info - skip this specific entry
			if (_device_informations[i].node_id == node_id &&
			    _device_informations[i].has_node_info) {

				continue;  // Continue to check other entries with same node_id
			}

			// Case 2: Check if node_id already exists with capability but no info - update that entry
			if (_device_informations[i].node_id == node_id &&
			    _device_informations[i].capability != DeviceCapability::NONE &&
			    !_device_informations[i].has_node_info) {
				populateDeviceInfoFields(_device_informations[i], *info);
				publishSingleDeviceInformation(_device_informations[i]);
				continue;
			}

		} else { // registering capabilities
			// Case 1: Check if this exact capability already exists - skip
			if (_device_informations[i].node_id == node_id &&
			    _device_informations[i].device_id == device_id &&
			    _device_informations[i].capability == capability) {
				return;
			}

			// Case 1b: if this node has multiple capabilities, continue
			if (_device_informations[i].node_id == node_id &&
			    _device_informations[i].capability != DeviceCapability::NONE  &&
			    _device_informations[i].capability != capability) {
				multi_capability_index = i;
				continue;
			}

			// Case 2: Check if node_id already exists with node info but no capability - update that entry
			if (_device_informations[i].node_id == node_id &&
			    _device_informations[i].has_node_info &&
			    _device_informations[i].capability == DeviceCapability::NONE) {
				_device_informations[i].device_id = device_id;
				_device_informations[i].capability = capability;
				publishSingleDeviceInformation(_device_informations[i]);
				return;
			}
		}
	}



	// Case 3: extend array and add entry at the end
	if (extendDeviceInformationsArray()) {
		_device_informations[_device_informations_size - 1] = DeviceInformation();
		_device_informations[_device_informations_size - 1].node_id = node_id;

		if (multi_capability_index >= 0) {
			_device_informations[_device_informations_size - 1] = _device_informations[multi_capability_index];
			_device_informations[_device_informations_size - 1].node_id = node_id;
		}

		if (is_registering_info) {
			populateDeviceInfoFields(_device_informations[_device_informations_size - 1], *info);

		} else {
			_device_informations[_device_informations_size - 1].device_id = device_id;
			_device_informations[_device_informations_size - 1].capability = capability;
		}

	} else {
		PX4_DEBUG("Failed to extend device informations array for %s",
			  is_registering_info ? "node info" : "capability");
	}
}

void NodeInfoPublisher::registerDeviceCapability(uint8_t node_id, uint32_t device_id, DeviceCapability capability)
{
	registerDevice(node_id, nullptr, device_id, capability);
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

		if (device_info.has_node_info && device_info.capability != DeviceCapability::NONE) {
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
	msg.device_type = static_cast<uint8_t>(device_info.capability);
	msg.device_id = device_info.device_id;

	// Copy pre-populated fields directly from the struct
	static_assert(sizeof(msg.model_name) == sizeof(device_info.model_name), "Array size mismatch");
	static_assert(sizeof(msg.vendor_name) == sizeof(device_info.vendor_name), "Array size mismatch");
	static_assert(sizeof(msg.firmware_version) == sizeof(device_info.firmware_version), "Array size mismatch");
	static_assert(sizeof(msg.hardware_version) == sizeof(device_info.hardware_version), "Array size mismatch");
	static_assert(sizeof(msg.serial_number) == sizeof(device_info.serial_number), "Array size mismatch");
	static_assert(sizeof(msg.name) == sizeof(device_info.name), "Array size mismatch");

	memcpy(msg.model_name, device_info.model_name, sizeof(msg.model_name));
	msg.model_name[sizeof(msg.model_name) - 1] = '\0';

	memcpy(msg.vendor_name, device_info.vendor_name, sizeof(msg.vendor_name));
	msg.vendor_name[sizeof(msg.vendor_name) - 1] = '\0';

	memcpy(msg.name, device_info.name, sizeof(msg.name));
	msg.name[sizeof(msg.name) - 1] = '\0';

	memcpy(msg.firmware_version, device_info.firmware_version, sizeof(msg.firmware_version));
	msg.firmware_version[sizeof(msg.firmware_version) - 1] = '\0';

	memcpy(msg.hardware_version, device_info.hardware_version, sizeof(msg.hardware_version));
	msg.hardware_version[sizeof(msg.hardware_version) - 1] = '\0';

	memcpy(msg.serial_number, device_info.serial_number, sizeof(msg.serial_number));
	msg.serial_number[sizeof(msg.serial_number) - 1] = '\0';

	_device_info_pub.publish(msg);

	PX4_DEBUG("Published device info for node %d, device_id %lu, type %d",
		  device_info.node_id, static_cast<unsigned long>(device_info.device_id),
		  static_cast<int>(device_info.capability));
}

void NodeInfoPublisher::populateDeviceInfoFields(DeviceInformation &device_info, const NodeInfo &info)
{
	device_info.has_node_info = true;

	// Remain backward compatible - for now.
	parseNodeName(info.name, device_info);

	snprintf(device_info.name, sizeof(device_info.name), "%s", info.name);
	snprintf(device_info.firmware_version, sizeof(device_info.firmware_version),
		 "%d.%d.%lu", info.sw_major, info.sw_minor, static_cast<unsigned long>(info.vcs_commit));
	snprintf(device_info.hardware_version, sizeof(device_info.hardware_version),
		 "%d.%d", info.hw_major, info.hw_minor);
	snprintf(device_info.serial_number, sizeof(device_info.serial_number),
		 "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x",
		 info.unique_id[0], info.unique_id[1], info.unique_id[2], info.unique_id[3],
		 info.unique_id[4], info.unique_id[5], info.unique_id[6], info.unique_id[7],
		 info.unique_id[8], info.unique_id[9], info.unique_id[10], info.unique_id[11],
		 info.unique_id[12], info.unique_id[13], info.unique_id[14], info.unique_id[15]);
}

void NodeInfoPublisher::parseNodeName(const char *name, DeviceInformation &device_info)
{
	if (!name || strlen(name) == 0) {
		strlcpy(device_info.vendor_name, "", sizeof(device_info.vendor_name));
		strlcpy(device_info.model_name, "", sizeof(device_info.model_name));
		return;
	}

	// Find first dot and skip everything before it
	const char *after_first_dot = strchr(name, '.');

	if (after_first_dot == nullptr) {
		// No dot - whole string is model, vendor is -1
		strlcpy(device_info.vendor_name, "", sizeof(device_info.vendor_name));
		strlcpy(device_info.model_name, name, sizeof(device_info.model_name));
		return;
	}

	after_first_dot++;

	// Find next dot in remaining string
	const char *second_dot = strchr(after_first_dot, '.');

	if (second_dot == nullptr) {
		// Only one dot - everything after first dot is model, vendor is -1
		strlcpy(device_info.vendor_name, "", sizeof(device_info.vendor_name));
		strlcpy(device_info.model_name, after_first_dot,  sizeof(device_info.model_name));
		return;
	}

	// Copy vendor (between first and second dot)
	size_t vendor_len = second_dot - after_first_dot;
	size_t copy_len = (vendor_len < sizeof(device_info.vendor_name) - 1) ? vendor_len : sizeof(device_info.vendor_name) - 1;
	strncpy(device_info.vendor_name, after_first_dot, copy_len);
	device_info.vendor_name[copy_len] = '\0';

	// Copy model (everything after second dot)
	strlcpy(device_info.model_name, second_dot + 1, sizeof(device_info.model_name));
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
