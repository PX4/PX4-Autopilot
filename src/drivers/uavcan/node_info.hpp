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

#pragma once
#include <uavcan/protocol/node_info_retriever.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/device_information.h>
#include <stdint.h>  // For UINT8_MAX, UINT32_MAX
#include <px4_platform_common/time.h>

using namespace time_literals;

constexpr int 		DEVICE_INFO_PUBLISH_INTERVAL_MS 	= 1000;
constexpr hrt_abstime 	DEVICE_INFO_PUBLISH_RATE_LIMIT_US 	= 100_ms;

class NodeInfoPublisher : private uavcan::INodeInfoListener, private uavcan::TimerBase
{
public:
	enum class DeviceCapability : uint8_t {
		NONE = UINT8_MAX,  // Invalid/unset capability value (255)
		GENERIC = device_information_s::DEVICE_TYPE_GENERIC,
		AIRSPEED = device_information_s::DEVICE_TYPE_AIRSPEED,
		ESC = device_information_s::DEVICE_TYPE_ESC,
		SERVO = device_information_s::DEVICE_TYPE_SERVO,
		GPS = device_information_s::DEVICE_TYPE_GPS,
		MAGNETOMETER = device_information_s::DEVICE_TYPE_MAGNETOMETER,
		PARACHUTE = device_information_s::DEVICE_TYPE_PARACHUTE,
		RANGEFINDER = device_information_s::DEVICE_TYPE_RANGEFINDER,
		WINCH = device_information_s::DEVICE_TYPE_WINCH,
		BAROMETER = device_information_s::DEVICE_TYPE_BAROMETER,
		OPTICAL_FLOW = device_information_s::DEVICE_TYPE_OPTICAL_FLOW,
		ACCELEROMETER = device_information_s::DEVICE_TYPE_ACCELEROMETER,
		GYROSCOPE = device_information_s::DEVICE_TYPE_GYROSCOPE,
		DIFFERENTIAL_PRESSURE = device_information_s::DEVICE_TYPE_DIFFERENTIAL_PRESSURE,
		BATTERY = device_information_s::DEVICE_TYPE_BATTERY,
		HYGROMETER = device_information_s::DEVICE_TYPE_HYGROMETER,
	};

	NodeInfoPublisher(uavcan::INode &node, uavcan::NodeInfoRetriever &node_info_retriever);
	~NodeInfoPublisher();

	// Called by sensor bridges to register device capabilities
	void registerDeviceCapability(uint8_t node_id, uint32_t device_id, DeviceCapability capability);

private:
	struct NodeInfo {
		NodeInfo(uavcan::NodeID id, const uavcan::protocol::GetNodeInfo_::Response &node_info)
			: node_id(id), sw_major(node_info.software_version.major), sw_minor(node_info.software_version.minor),
			  vcs_commit(node_info.software_version.vcs_commit), hw_major(node_info.hardware_version.major),
			  hw_minor(node_info.hardware_version.minor)
		{
			memcpy(name, node_info.name.c_str(), node_info.name.capacity());
			name[node_info.name.capacity() - 1] = '\0';
			memcpy(unique_id, &node_info.hardware_version.unique_id.front(), node_info.hardware_version.unique_id.size());
		}
		NodeInfo() = default;

		uavcan::NodeID node_id{};

		char name[uavcan::protocol::GetNodeInfo_::Response::FieldTypes::name::MaxSize];
		uint8_t unique_id[uavcan::protocol::GetNodeInfo_::Response::FieldTypes::hardware_version::FieldTypes::unique_id::MaxSize];
		uint8_t sw_major;
		uint8_t sw_minor;
		uint32_t vcs_commit;
		uint8_t hw_major;
		uint8_t hw_minor;
	};

	struct DeviceInformation {
		uint8_t node_id{UINT8_MAX};
		uint32_t device_id{UINT32_MAX};
		DeviceCapability capability{DeviceCapability::NONE};
		bool has_node_info{false};

		char name[80];
		char vendor_name[32];
		char model_name[32];
		char firmware_version[24];
		char hardware_version[24];
		char serial_number[33];

		DeviceInformation() : node_id(UINT8_MAX), device_id(UINT32_MAX), capability(DeviceCapability::NONE),
			has_node_info(false)
		{
			// Initialize string fields
			name[0] = '\0';
			vendor_name[0] = '\0';
			model_name[0] = '\0';
			firmware_version[0] = '\0';
			hardware_version[0] = '\0';
			serial_number[0] = '\0';
		}

		DeviceInformation(uint8_t nid, uint32_t did, DeviceCapability cap)
			: node_id(nid), device_id(did), capability(cap), has_node_info(false)
		{
			// Initialize string fields
			name[0] = '\0';
			vendor_name[0] = '\0';
			model_name[0] = '\0';
			firmware_version[0] = '\0';
			hardware_version[0] = '\0';
			serial_number[0] = '\0';
		}
	};

	void handleNodeInfoRetrieved(uavcan::NodeID node_id,
				     const uavcan::protocol::GetNodeInfo_::Response &node_info) override;
	void handleNodeInfoUnavailable(uavcan::NodeID node_id) override;

	void handleTimerEvent(const uavcan::TimerEvent &event) override;

	void startTimerIfNotRunning();

	// Register device info or capability, set nodeinfo to nullptr if only registering capability
	void registerDevice(uint8_t node_id, const NodeInfo *info, uint32_t device_id, DeviceCapability capability);

	// Publishing methods
	void publishDeviceInformationPeriodic();
	void publishSingleDeviceInformation(const DeviceInformation &device_info);

	// Helper functions
	void populateDeviceInfoFields(DeviceInformation &device_info, const NodeInfo &info);
	void parseNodeName(const char *name, DeviceInformation &device_info);
	bool extendDeviceInformationsArray();

	uavcan::NodeInfoRetriever &_node_info_retriever;

	// Device capability tracking
	DeviceInformation *_device_informations{nullptr};
	size_t _device_informations_size{0};
	uORB::Publication<device_information_s> _device_info_pub{ORB_ID(device_information)};
	hrt_abstime _last_device_info_publish{0};

	// Round-robin publishing
	size_t _next_device_to_publish{0};
};
