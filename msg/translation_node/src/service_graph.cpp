/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "service_graph.h"

#include <utility>

using namespace std::chrono_literals;

ServiceGraph::ServiceGraph(rclcpp::Node &node, const ServiceTranslations& translations)
	: _node(node) {

	std::unordered_map<std::string, std::set<MessageVersionType>> known_versions;

	for (const auto& service : translations.nodes()) {
		const std::string full_topic_name = getFullTopicName(_node.get_effective_namespace(), service.id.topic_name);
		_known_services_warned.insert({full_topic_name, false});

		const MessageIdentifier id{full_topic_name, service.id.version};
		auto node_data = std::make_shared<NodeDataService>(service, id);
		_request_graph.addNodeIfNotExists(id, node_data, service.message_buffer_request);
		_response_graph.addNodeIfNotExists(id, node_data, service.message_buffer_response);
		known_versions[full_topic_name].insert(id.version);
	}

	auto get_full_topic_names = [this](std::vector<MessageIdentifier> ids) {
		for (auto& id : ids) {
			id.topic_name = getFullTopicName(_node.get_effective_namespace(), id.topic_name);
		}
		return ids;
	};

	for (const auto& translation : translations.requestTranslations()) {
		const std::vector<MessageIdentifier> inputs = get_full_topic_names(translation.inputs);
		const std::vector<MessageIdentifier> outputs = get_full_topic_names(translation.outputs);
		_request_graph.addTranslation(translation.cb, inputs, outputs);
	}
	for (const auto& translation : translations.responseTranslations()) {
		const std::vector<MessageIdentifier> inputs = get_full_topic_names(translation.inputs);
		const std::vector<MessageIdentifier> outputs = get_full_topic_names(translation.outputs);
		_response_graph.addTranslation(translation.cb, inputs, outputs);
	}

	printServiceInfo(known_versions);
	handleLargestTopic(known_versions);

	_cleanup_timer = _node.create_wall_timer(10s, [this]() {
		cleanupStaleRequests();
	});
}

void ServiceGraph::updateCurrentServices(const std::vector<ServiceInfo> &services) {
	_request_graph.iterateNodes([](const MessageIdentifier& type, const GraphForService::MessageNodePtr& node) {
		node->data()->has_service = false;
		node->data()->has_client = false;
		node->data()->visited = false;
	});

	for (const auto& info : services) {
		const auto [non_versioned_topic_name, version] = getNonVersionedTopicName(info.service_name);
		auto maybe_node = _request_graph.findNode({non_versioned_topic_name, version});
		if (!maybe_node) {
			auto known_topic_iter = _known_services_warned.find(non_versioned_topic_name);
			if (known_topic_iter != _known_services_warned.end() && !known_topic_iter->second) {
				RCLCPP_WARN(_node.get_logger(), "No translation available for version %i of service %s", version, non_versioned_topic_name.c_str());
				known_topic_iter->second = true;
			}
			continue;
		}
		const auto& node = maybe_node.value();

		if (info.num_services > 0) {
			node->data()->has_service = true;
		}
		if (info.num_clients > 0) {
			node->data()->has_client = true;
		}
	}

	// Iterate connected graph segments
	_request_graph.iterateNodes([this](const MessageIdentifier& type, const GraphForService::MessageNodePtr& node) {
		if (node->data()->visited) {
			return;
		}
		node->data()->visited = true;

		// Check if there's a reachable node with a service
		int num_services = 0;

		_request_graph.iterateBFS(node, [&](const GraphForService::MessageNodePtr& node) {
			if (node->data()->has_service && !node->data()->service) {
				++num_services;
			}
		});

		// We need to instantiate a service and clients if there's exactly one external service.
		if (num_services > 1 ) {
			RCLCPP_ERROR_ONCE(_node.get_logger(), "Found %i services for service '%s', skipping this service",
							  num_services, node->data()->service_name.c_str());
		} else if (num_services == 1) {
			_request_graph.iterateBFS(node, [&](const GraphForService::MessageNodePtr& node) {
				node->data()->visited = true;
				if (node->data()->has_service && !node->data()->client && !node->data()->service) {
					RCLCPP_INFO(_node.get_logger(), "Found service for '%s', version: %i, adding client", node->data()->service_name.c_str(), node->data()->version);
					auto tuple = node->data()->client_factory(_node, [this, tmp_node=node](rmw_request_id_t& request) {
						onResponse(request, tmp_node);
					});
					node->data()->client = std::get<0>(tuple);
					node->data()->client_send_cb = std::get<1>(tuple);

				} else if (!node->data()->has_service && !node->data()->service && node->data()->has_client) {
					RCLCPP_INFO(_node.get_logger(), "Found client for '%s', version: %i, adding service", node->data()->service_name.c_str(), node->data()->version);
					node->data()->service = node->data()->service_factory(_node, [this, tmp_node=node](std::shared_ptr<rmw_request_id_t> req_id) {
						onNewRequest(std::move(req_id), tmp_node);
					});
				}
			});

		} else {
			// Reset any service or client
			_request_graph.iterateBFS(node, [&](const GraphForService::MessageNodePtr& node) {
				node->data()->visited = true;
				if (node->data()->service) {
					RCLCPP_INFO(_node.get_logger(), "Removing service for '%s', version: %i",
								node->data()->service_name.c_str(), node->data()->version);
					node->data()->service.reset();
				}
				if (node->data()->client) {
					RCLCPP_INFO(_node.get_logger(), "Removing client for '%s', version: %i",
								node->data()->service_name.c_str(), node->data()->version);
					node->data()->client.reset();
				}
			});
		}
	});
}

void ServiceGraph::printServiceInfo(const std::unordered_map<std::string, std::set<MessageVersionType>>& known_versions) const {
	// Print info about known versions
	RCLCPP_INFO(_node.get_logger(), "Registered services and versions:");
	for (const auto& [topic_name, version_set] : known_versions) {
		if (version_set.empty()) {
			continue;
		}
		const std::string versions = std::accumulate(std::next(version_set.begin()), version_set.end(),
													 std::to_string(*version_set.begin()), // start with first element
													 [](std::string a, auto&& b) {
														 return std::move(a) + ", " + std::to_string(b);
													 });
		RCLCPP_INFO(_node.get_logger(), "- %s: %s", topic_name.c_str(), versions.c_str());
	}
}

void ServiceGraph::handleLargestTopic(const std::unordered_map<std::string, std::set<MessageVersionType>> &known_versions) {
	// See PubSubGraph::handleLargestTopic for an explanation why this is needed
	unsigned index = 0;
	for (const auto& [topic_name, versions] : known_versions) {
		std::array<size_t, 2> max_serialized_message_size{0, 0};
		std::array<const NamedPublicationFactoryCB*, 2> publication_factory_for_max{nullptr, nullptr};
		for (auto version : versions) {
			const auto& node = _request_graph.findNode(MessageIdentifier{topic_name, version});
			assert(node);
			const auto& node_data = node.value()->data();
			for (unsigned i = 0; i < max_serialized_message_size.size(); ++i) {
				if (node_data->max_serialized_message_size[i] > max_serialized_message_size[i]) {
					max_serialized_message_size[i] = node_data->max_serialized_message_size[i];
					publication_factory_for_max[i] = &node_data->publication_factory[i];
				}
			}
		}
		for (unsigned i = 0; i < max_serialized_message_size.size(); ++i) {
			if (publication_factory_for_max[i]) {
				const std::string tmp_topic_name = "dummy_topic" + std::to_string(index++);
				_largest_topic_publications.emplace_back((*publication_factory_for_max[i])(_node, tmp_topic_name));
			}
		}
	}
}

void ServiceGraph::onNewRequest(std::shared_ptr<rmw_request_id_t> req_id, GraphForService::MessageNodePtr node) {
	bool service_called = false;
	_request_graph.translate(node, [this, &service_called, &req_id, original_node=node](const GraphForService::MessageNodePtr& node) {
		if (node->data()->client && node->data()->client_send_cb && !service_called) {
			service_called = true;
			const int64_t client_request_id = node->data()->client_send_cb(node->buffer());
			node->data()->ongoing_requests[client_request_id] = Request{req_id, original_node->data(), _node.now()};
		}
	});
}

void ServiceGraph::onResponse(rmw_request_id_t &req_id, GraphForService::MessageNodePtr node) {
	auto iter = node->data()->ongoing_requests.find(req_id.sequence_number);
	if (iter == node->data()->ongoing_requests.end()) {
		RCLCPP_ERROR(_node.get_logger(), "Got response with unknown request %li", req_id.sequence_number);
		return;
	}
	bool service_called = false;
	auto response_node = _response_graph.findNode({node->data()->service_name, node->data()->version});
	assert(response_node);
	_response_graph.translate(response_node.value(), [this, &service_called, &iter](const GraphForService::MessageNodePtr &node) {
		if (node->data()->service && !service_called && iter->second.original_node_data == node->data()) {
			const rcl_ret_t ret = rcl_send_response(node->data()->service->get_service_handle().get(),
							  iter->second.original_request_id.get(), node->buffer().get());
			if (ret != RCL_RET_OK) {
				RCLCPP_ERROR(_node.get_logger(), "Failed to send response: %s", rcl_get_error_string().str);
			}
			service_called = true;
		}
	});

	node->data()->ongoing_requests.erase(iter);
}

void ServiceGraph::cleanupStaleRequests() {
	static const auto kRequestTimeout = 20s;
	_request_graph.iterateNodes([this](const MessageIdentifier& type, const GraphForService::MessageNodePtr& node) {
		for (auto it = node->data()->ongoing_requests.begin(); it != node->data()->ongoing_requests.end();) {
			const auto& request = it->second;
			if (_node.now() - request.timestamp_received > kRequestTimeout) {
				RCLCPP_INFO(_node.get_logger(), "Request timed out, dropping ongoing request for '%s', version: %i, request id: %li",
							node->data()->service_name.c_str(), node->data()->version, request.original_request_id->sequence_number);
				it = node->data()->ongoing_requests.erase(it);
			} else {
				++it;
			}
		}
	});
}
