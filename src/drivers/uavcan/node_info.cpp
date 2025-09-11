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

NodeInfoPublisher::NodeInfoPublisher(uavcan::INode &node, uavcan::NodeInfoRetriever &node_info_retriever)
	: TimerBase(node), _node_info_retriever(node_info_retriever)
{
	_node_info_retriever.addListener(this);
}

NodeInfoPublisher::~NodeInfoPublisher()
{
	_node_info_retriever.removeListener(this);
	delete[] _node_info_array;
}

void NodeInfoPublisher::handleNodeInfoRetrieved(uavcan::NodeID node_id,
		const uavcan::protocol::GetNodeInfo_::Response &node_info)
{
	const NodeInfo info(node_id, node_info);
	addOrReplaceNodeInfo(info);

	// Publish immediately
	publishInfo(info);
	startTimerIfNotRunning();
}

void NodeInfoPublisher::handleNodeInfoUnavailable(uavcan::NodeID node_id)
{
}

void NodeInfoPublisher::handleTimerEvent(const uavcan::TimerEvent &event)
{
	// Publish next
	for (size_t i = 0; i < _node_info_array_size; ++i) {
		const size_t idx = _next_to_publish % _node_info_array_size;
		_next_to_publish = idx + 1;

		if (_node_info_array[idx].node_id.isValid()) {
			const auto node_status = _node_info_retriever.getNodeStatus(_node_info_array[idx].node_id);

			if (node_status.mode != uavcan::protocol::NodeStatus::MODE_OFFLINE) {
				publishInfo(_node_info_array[idx]);
				break;
			}
		}
	}
}

void NodeInfoPublisher::publishInfo(const NodeInfo &info)
{
	// TODO
	printf("Node info: \n");
	printf(" name: %s sw ver: %i.%i sw git commit=%lu\n", info.name, info.sw_major, info.sw_minor,
	       info.vcs_commit);
	printf(" hw unique id: %02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",
	       info.unique_id[0], info.unique_id[1], info.unique_id[2],
	       info.unique_id[3], info.unique_id[4], info.unique_id[5],
	       info.unique_id[6], info.unique_id[7], info.unique_id[8],
	       info.unique_id[9], info.unique_id[10], info.unique_id[11],
	       info.unique_id[12], info.unique_id[13], info.unique_id[14],
	       info.unique_id[15]);
}

void NodeInfoPublisher::startTimerIfNotRunning()
{
	if (!TimerBase::isRunning()) {
		const int interval_ms = 1000;
		TimerBase::startPeriodic(uavcan::MonotonicDuration::fromMSec(interval_ms));
		UAVCAN_TRACE("NodeInfoPublisher", "Timer started, interval %i ms", interval_ms);
	}
}

void NodeInfoPublisher::addOrReplaceNodeInfo(const NodeInfo &info)
{
	// Check if exists or if there is an unused index
	size_t unused_idx = _node_info_array_size;

	for (size_t i = 0; i < _node_info_array_size; ++i) {
		if (_node_info_array[i].node_id == info.node_id) {
			_node_info_array[i] = info;
			return;
		}

		if (!_node_info_array[i].node_id.isValid()) {
			unused_idx = i;
		}
	}

	// Is there an unused index?
	if (unused_idx < _node_info_array_size) {
		_node_info_array[unused_idx] = info;
		return;
	}

	// Resize array
	const size_t new_size = _node_info_array_size + 2;
	NodeInfo *new_array = new NodeInfo[new_size];

	if (!new_array) {
		PX4_ERR("Failed to resize node info array");
		return;
	}

	if (_node_info_array_size > 0) {
		memcpy(new_array, _node_info_array, _node_info_array_size * sizeof(NodeInfo));
		delete[] _node_info_array;
	}

	_node_info_array = new_array;
	_node_info_array[_node_info_array_size] = info;
	_node_info_array_size = new_size;
}
