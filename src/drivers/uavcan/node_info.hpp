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

#pragma once
#include <uavcan/protocol/node_info_retriever.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/device_information.h>
#include <stdint.h>  // For UINT8_MAX, UINT32_MAX
#include <px4_platform_common/time.h>

using namespace time_literals;

constexpr int DEVICE_INFO_PUBLISH_INTERVAL_MS = 1000;
constexpr hrt_abstime DEVICE_INFO_PUBLISH_RATE_LIMIT_US = 100_ms;

class NodeInfoPublisher : private uavcan::INodeInfoListener, private uavcan::TimerBase
{
public:
	enum class NodeVendor : uint8_t {
		UNKNOWN = 0,
		VERTIQ, // formerly IQ Motion Control hence "iq_motion" vendor name
	};

	NodeInfoPublisher(uavcan::INode &node, uavcan::NodeInfoRetriever &node_info_retriever);
	~NodeInfoPublisher();

	// Called by sensor bridges to register device capabilities
	void registerDeviceCapability(uint8_t node_id, uint32_t device_id, uint8_t device_type);

	NodeVendor getNodeVendor(uint8_t node_id) const
	{
		if (node_id < 1 || node_id > uavcan::NodeID::Max) { return NodeVendor::UNKNOWN; }

		return static_cast<NodeVendor>(_node_vendors[node_id]);
	}

private:
	static constexpr uint8_t DEVICE_TYPE_NONE = UINT8_MAX;

	struct DeviceInformation {
		uint8_t node_id = UINT8_MAX;
		uint32_t device_id = UINT32_MAX;
		uint8_t device_type = DEVICE_TYPE_NONE;
		bool has_node_info = false;

		char name[80] = "";
		uint8_t sw_major = 0;
		uint8_t sw_minor = 0;
		uint32_t sw_vcs_commit = 0;
		uint8_t hw_major = 0;
		uint8_t hw_minor = 0;
		uint8_t unique_id[16] {};
	};

	void handleNodeInfoRetrieved(uavcan::NodeID node_id,
				     const uavcan::protocol::GetNodeInfo_::Response &node_info) override;
	void handleNodeInfoUnavailable(uavcan::NodeID node_id) override;

	void handleTimerEvent(const uavcan::TimerEvent &event) override;

	void startTimerIfNotRunning();

	void registerNodeInfo(uint8_t node_id, const uavcan::protocol::GetNodeInfo_::Response &node_info);
	void registerCapability(uint8_t node_id, uint32_t device_id, uint8_t device_type);
	void populateDeviceInfoFields(DeviceInformation &device_info, const uavcan::protocol::GetNodeInfo_::Response &node_info);

	// Publishing methods
	void publishDeviceInformationPeriodic();
	void publishSingleDeviceInformation(const DeviceInformation &device_info);

	bool extendDeviceInformationsArray();

	uavcan::NodeInfoRetriever &_node_info_retriever;

	NodeVendor _node_vendors[uavcan::NodeID::Max + 1] {}; // indexed by node_id

	// Device capability tracking
	DeviceInformation *_device_informations{nullptr};
	size_t _device_informations_size{0};
	size_t _device_informations_capacity{0};
	uORB::Publication<device_information_s> _device_info_pub{ORB_ID(device_information)};
	hrt_abstime _last_device_info_publish{0};

	// Round-robin publishing
	size_t _next_device_to_publish{0};
};
