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

class NodeInfoPublisher : private uavcan::INodeInfoListener, private uavcan::TimerBase
{
public:
	NodeInfoPublisher(uavcan::INode &node, uavcan::NodeInfoRetriever &node_info_retriever);
	~NodeInfoPublisher();

private:
	struct NodeInfo {
		NodeInfo(uavcan::NodeID id, const uavcan::protocol::GetNodeInfo_::Response &node_info)
			: node_id(id), sw_major(node_info.software_version.major), sw_minor(node_info.software_version.minor),
			  vcs_commit(node_info.software_version.vcs_commit)
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
		uint32_t vcs_commit; // e.g. git short commit hash. Optional.
	};

	void handleNodeInfoRetrieved(uavcan::NodeID node_id,
				     const uavcan::protocol::GetNodeInfo_::Response &node_info) override;
	void handleNodeInfoUnavailable(uavcan::NodeID node_id) override;

	void handleTimerEvent(const uavcan::TimerEvent &event) override;

	void publishInfo(const NodeInfo &info);

	void startTimerIfNotRunning();

	void addOrReplaceNodeInfo(const NodeInfo &info);

	uavcan::NodeInfoRetriever &_node_info_retriever;

	NodeInfo *_node_info_array{nullptr};
	size_t _node_info_array_size{0};
	size_t _next_to_publish{0};
};
