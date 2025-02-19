/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <utility>
#include "translations.h"
#include "translation_util.h"
#include "graph.h"

class ServiceGraph {
public:
	struct ServiceInfo {
		std::string service_name; ///< fully qualified service name (with namespace)
		int num_services; ///< This can include a service created by the translation node
		int num_clients; ///< This can include a client created by the translation node
	};

	ServiceGraph(rclcpp::Node &node, const ServiceTranslations& translations);

	void updateCurrentServices(const std::vector<ServiceInfo>& services);

private:
	struct NodeDataService;
	using GraphForService = Graph<std::shared_ptr<NodeDataService>>;

	void printServiceInfo(const std::unordered_map<std::string, std::set<MessageVersionType>> &known_versions) const;
	void handleLargestTopic(const std::unordered_map<std::string, std::set<MessageVersionType>>& known_versions);

	void onNewRequest(std::shared_ptr<rmw_request_id_t> req_id, GraphForService::MessageNodePtr node);
	void onResponse(rmw_request_id_t& req_id, GraphForService::MessageNodePtr node);
	void cleanupStaleRequests();

	struct Request {
		std::shared_ptr<rmw_request_id_t> original_request_id;
		std::shared_ptr<NodeDataService> original_node_data{nullptr};
		rclcpp::Time timestamp_received;
	};
	struct NodeDataService {
		explicit NodeDataService(const Service& service, const MessageIdentifier& id)
				: service_factory(service.service_factory), client_factory(service.client_factory),
				  service_name(id.topic_name), version(id.version),
				  publication_factory{service.publication_factory_request, service.publication_factory_response},
				  max_serialized_message_size{service.max_serialized_message_size_request, service.max_serialized_message_size_response}
		{ }

		const ServiceFactoryCB service_factory;
		const ClientFactoryCB client_factory;
		const std::string service_name;
		const MessageVersionType version;
		const std::array<NamedPublicationFactoryCB, 2> publication_factory; // Request/Response
		const std::array<size_t, 2> max_serialized_message_size;

		// Keep track if there's currently a client/service
		bool has_service{false};
		bool has_client{false};

		rclcpp::ClientBase::SharedPtr client;
		ClientSendCB client_send_cb;
		rclcpp::ServiceBase::SharedPtr service;

		std::unordered_map<int64_t, Request> ongoing_requests; ///< Ongoing service calls for this node

		bool visited{false};
	};

	rclcpp::Node& _node;
	GraphForService _request_graph;
	GraphForService _response_graph;
	std::unordered_map<std::string, bool> _known_services_warned;
	rclcpp::TimerBase::SharedPtr _cleanup_timer;

	std::vector<rclcpp::PublisherBase::SharedPtr> _largest_topic_publications;
};
