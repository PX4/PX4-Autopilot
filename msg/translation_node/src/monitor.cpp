/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#include "monitor.h"
using namespace std::chrono_literals;

Monitor::Monitor(rclcpp::Node &node, PubSubGraph* pub_sub_graph, ServiceGraph* service_graph)
	: _node(node), _pub_sub_graph(pub_sub_graph), _service_graph(service_graph) {

	// Monitor subscriptions & publishers
	// TODO: event-based
	_node_update_timer = _node.create_wall_timer(1s, [this]() {
		updateNow();
	});
}

void Monitor::updateNow() {

	// Topics
	if (_pub_sub_graph != nullptr) {
		std::vector<PubSubGraph::TopicInfo> topic_info;
		const auto topics = _node.get_topic_names_and_types();
		for (const auto &[topic_name, topic_types]: topics) {
			auto publishers = _node.get_publishers_info_by_topic(topic_name);
			auto subscribers = _node.get_subscriptions_info_by_topic(topic_name);
			// Filter out self
			int num_publishers = 0;
			for (const auto &publisher: publishers) {
				num_publishers += publisher.node_name() != _node.get_name();
			}
			int num_subscribers = 0;
			for (const auto &subscriber: subscribers) {
				num_subscribers += subscriber.node_name() != _node.get_name();
			}

			if (num_subscribers > 0 || num_publishers > 0) {
				topic_info.emplace_back(PubSubGraph::TopicInfo{topic_name, num_subscribers, num_publishers});
			}
		}
		_pub_sub_graph->updateCurrentTopics(topic_info);
	}

	// Services
#ifndef DISABLE_SERVICES // ROS Humble does not support the count_services() call
	if (_service_graph != nullptr) {
		std::vector<ServiceGraph::ServiceInfo> service_info;
		const auto services = _node.get_service_names_and_types();
		for (const auto& [service_name, service_types] : services) {
			const int num_services = _node.get_node_graph_interface()->count_services(service_name);
			const int num_clients = _node.get_node_graph_interface()->count_clients(service_name);
			// We cannot filter out our own node, as we don't have that info.
			// We could use `get_service_names_and_types_by_node`, but then we would not get
			// services by non-ros nodes (e.g. microxrce dds bridge)
			service_info.emplace_back(ServiceGraph::ServiceInfo{service_name, num_services, num_clients});
		}
		_service_graph->updateCurrentServices(service_info);
	}
#endif
}
