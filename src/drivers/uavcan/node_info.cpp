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
	registerDeviceInfo(info);

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

void NodeInfoPublisher::registerDeviceInfo(const NodeInfo &info)
{
	uint8_t node_id = info.node_id.get();

	for (size_t i = 0; i < _device_informations_size; ++i) {
		// Case 1: Check if this exact node already has node info - skip
		if (_device_informations[i].node_id == node_id &&
		    _device_informations[i].has_node_info) {
			return;
		}

		// Case 2: Check if node_id already exists with capability but no info - update that entry
		if (_device_informations[i].node_id == node_id &&
		    _device_informations[i].has_capability &&
		    !_device_informations[i].has_node_info) {

			populateDeviceInfoFields(_device_informations[i], info);

			publishDeviceInformationImmediate(i);
			// Continue to check for other entries with the same node_id but different capabilities.
		}

		// Case 3: Find unused slot and use it
		if (_device_informations[i].node_id == UINT8_MAX && !_device_informations[i].has_capability
		    && !_device_informations[i].has_node_info) {

			_device_informations[i] = DeviceInformation();
			_device_informations[i].node_id = node_id;

			populateDeviceInfoFields(_device_informations[i], info);

			return;
		}
	}

	// Case 4: extend array and add entry at the end
	if (extendDeviceInformationsArray()) {
		// Use default constructor - this sets capability to DeviceCapability::NONE
		_device_informations[_device_informations_size - 1] = DeviceInformation();
		_device_informations[_device_informations_size - 1].node_id = node_id;

		// Populate device info fields directly
		populateDeviceInfoFields(_device_informations[_device_informations_size - 1], info);

	} else {
		PX4_DEBUG("Failed to extend device informations array for node info");
	}
}

void NodeInfoPublisher::registerDeviceCapability(uint8_t node_id, uint32_t device_id, DeviceCapability capability)
{
	for (size_t i = 0; i < _device_informations_size; ++i) {
		// Case 1: Check if this exact capability already exists - skip
		if (_device_informations[i].node_id == node_id &&
		    _device_informations[i].device_id == device_id &&
		    _device_informations[i].capability == capability &&
		    _device_informations[i].has_capability) {

			return;
		}

		// Case 1b: if this node has multiple capabilities, add a new entry for the new capability
		if (_device_informations[i].node_id == node_id &&
		    _device_informations[i].has_capability &&
		    _device_informations[i].capability != capability) {
			if (extendDeviceInformationsArray()) {
				_device_informations[_device_informations_size - 1] = _device_informations[i];
				_device_informations[_device_informations_size - 1].device_id = device_id;
				_device_informations[_device_informations_size - 1].capability = capability;
				_device_informations[_device_informations_size - 1].has_capability = true;

			} else {
				PX4_DEBUG("Failed to extend device informations array for node %d",
					  node_id);
			}

			return;
		}

		// Case 2: Check if node_id already exists with node info but no capability - update that entry
		if (_device_informations[i].node_id == node_id &&
		    _device_informations[i].has_node_info &&
		    !_device_informations[i].has_capability) {

			_device_informations[i].device_id = device_id;
			_device_informations[i].capability = capability;
			_device_informations[i].has_capability = true;
			publishDeviceInformationImmediate(i);
			return;
		}

		// Case 3: Find unused slot and use it
		if (_device_informations[i].node_id == UINT8_MAX && !_device_informations[i].has_capability
		    && !_device_informations[i].has_node_info) {
			_device_informations[i].node_id = node_id;
			_device_informations[i].device_id = device_id;
			_device_informations[i].capability = capability;
			_device_informations[i].has_capability = true;

			return;
		}
	}

	// Case 4: extend array and add entry at the end
	if (extendDeviceInformationsArray()) {
		_device_informations[_device_informations_size - 1].node_id = node_id;
		_device_informations[_device_informations_size - 1].device_id = device_id;
		_device_informations[_device_informations_size - 1].capability = capability;
		_device_informations[_device_informations_size - 1].has_capability = true;


	} else {
		PX4_DEBUG("Failed to extend device informations array for capability");
	}
}

bool NodeInfoPublisher::hasDeviceCapability(uint8_t node_id, uint32_t device_id, DeviceCapability capability)
{
	for (size_t i = 0; i < _device_informations_size; ++i) {
		if (_device_informations[i].node_id == node_id &&
		    _device_informations[i].device_id == device_id &&
		    _device_informations[i].capability == capability &&
		    _device_informations[i].has_capability) {
			return true;
		}
	}

	return false;
}

void NodeInfoPublisher::publishDeviceInformationImmediate(size_t device_index)
{
	if (device_index >= _device_informations_size) {
		return;
	}

	const auto &device_info = _device_informations[device_index];

	// Only publish if device has both capability and node info
	if (!device_info.has_node_info || !device_info.has_capability) {
		return;
	}

	const uint64_t now = hrt_absolute_time();

	// Rate limit immediate publishing to prevent spam
	if (now - _last_device_info_publish < DEVICE_INFO_PUBLISH_RATE_LIMIT_US) {
		PX4_ERR("Immediate publish for node %d rate limited",
			device_info.node_id);
		return;
	}

	_last_device_info_publish = now;
	publishSingleDeviceInformation(device_info);
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

		if (device_info.has_node_info && device_info.has_capability) {
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
	strlcpy(msg.model_name, device_info.model_name, sizeof(msg.model_name));
	strlcpy(msg.vendor_name, device_info.vendor_name, sizeof(msg.vendor_name));
	strlcpy(msg.firmware_version, device_info.firmware_version, sizeof(msg.firmware_version));
	strlcpy(msg.hardware_version, device_info.hardware_version, sizeof(msg.hardware_version));
	strlcpy(msg.serial_number, device_info.serial_number, sizeof(msg.serial_number));

	_device_info_pub.publish(msg);

	PX4_DEBUG("Published device info for node %d, device_id %lu, type %d",
		  device_info.node_id, static_cast<unsigned long>(device_info.device_id),
		  static_cast<int>(device_info.capability));
}

void NodeInfoPublisher::populateDeviceInfoFields(DeviceInformation &device_info, const NodeInfo &info)
{
	device_info.has_node_info = true;

	// Parse the node name to extract vendor and model information
	parseNodeName(info.name, device_info);

	snprintf(device_info.firmware_version, sizeof(device_info.firmware_version),
		 "%d.%d.%lu", info.sw_major, info.sw_minor, static_cast<unsigned long>(info.vcs_commit));
	snprintf(device_info.hardware_version, sizeof(device_info.hardware_version),
		 "%d.%d", info.hw_major, info.hw_minor);
	snprintf(device_info.serial_number, sizeof(device_info.serial_number),
		 "%02x%02x%02x%02x%02x%02x%02x%02x",
		 info.unique_id[0], info.unique_id[1], info.unique_id[2], info.unique_id[3],
		 info.unique_id[4], info.unique_id[5], info.unique_id[6], info.unique_id[7]);
}

void NodeInfoPublisher::parseNodeName(const char *name, DeviceInformation &device_info)
{
	// Node name should follow reversed internet domain structure: "com.manufacturer.project.product" as defined in 1.GetNodeInfo.uavcan

	if (!name || strlen(name) == 0) {
		strlcpy(device_info.vendor_name, "-1", sizeof(device_info.vendor_name));
		strlcpy(device_info.model_name, "-1", sizeof(device_info.model_name));
		return;
	}

	char name_copy[81];
	strlcpy(name_copy, name, sizeof(name_copy));

	char *token = strtok(name_copy, ".");
	char *parts[5];
	int part_count = 0;

	while (token != nullptr && part_count < 5) {
		parts[part_count++] = token;
		token = strtok(nullptr, ".");
	}

	if (part_count >= 2) {
		// Extract manufacturer
		strlcpy(device_info.vendor_name, parts[1], sizeof(device_info.vendor_name));

		// Extract project.product as model
		device_info.model_name[0] = '\0';

		for (int i = 2; i < part_count; i++) {
			if (i > 2) {
				strlcat(device_info.model_name, ".", sizeof(device_info.model_name));
			}

			strlcat(device_info.model_name, parts[i], sizeof(device_info.model_name));
		}

	} else {
		// Doesn't follow expected format
		strlcpy(device_info.vendor_name, "-1", sizeof(device_info.vendor_name));
		strlcpy(device_info.model_name, name, sizeof(device_info.model_name));
	}
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
