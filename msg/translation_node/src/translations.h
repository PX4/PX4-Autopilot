/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <string>
#include <cstdint>
#include <unordered_map>
#include <utility>
#include <functional>
#include <vector>
#include <memory>
#include <tuple>

#include "util.h"
#include "graph.h"

#include <rclcpp/rclcpp.hpp>


using TranslationCB = std::function<void(const std::vector<MessageBuffer>&, std::vector<MessageBuffer>&)>;
using SubscriptionFactoryCB = std::function<rclcpp::SubscriptionBase::SharedPtr(rclcpp::Node&, const std::function<void()>& on_topic_cb)>;
using PublicationFactoryCB = std::function<rclcpp::PublisherBase::SharedPtr(rclcpp::Node&)>;
using NamedPublicationFactoryCB = std::function<rclcpp::PublisherBase::SharedPtr(rclcpp::Node&, const std::string&)>;
using ServiceFactoryCB = std::function<rclcpp::ServiceBase::SharedPtr(rclcpp::Node&, const std::function<void(std::shared_ptr<rmw_request_id_t> req_id)>& on_request_cb)>;
using ClientSendCB = std::function<int64_t(MessageBuffer)>;
using ClientFactoryCB = std::function<std::tuple<rclcpp::ClientBase::SharedPtr, ClientSendCB>(rclcpp::Node&, const std::function<void(rmw_request_id_t&)>& on_response_cb)>;

struct Topic {
	MessageIdentifier id;

	SubscriptionFactoryCB subscription_factory;
	PublicationFactoryCB publication_factory;

	std::shared_ptr<void> message_buffer;
	size_t max_serialized_message_size{};
};

struct Service {
	MessageIdentifier id;

	ServiceFactoryCB service_factory;
	ClientFactoryCB client_factory;

	NamedPublicationFactoryCB publication_factory_request;
	NamedPublicationFactoryCB publication_factory_response;

	std::shared_ptr<void> message_buffer_request;
	size_t max_serialized_message_size_request{};

	std::shared_ptr<void> message_buffer_response;
	size_t max_serialized_message_size_response{};
};

struct Translation {
	TranslationCB cb;
	std::vector<MessageIdentifier> inputs;
	std::vector<MessageIdentifier> outputs;
};

class TopicTranslations {
public:
	TopicTranslations() = default;

	void addTopic(Topic topic) { _topics.push_back(std::move(topic)); }
	void addTranslation(Translation translation) { _translations.push_back(std::move(translation)); }

	const std::vector<Topic>& topics() const { return _topics; }
	const std::vector<Translation>& translations() const { return _translations; }
private:
	std::vector<Topic> _topics;
	std::vector<Translation> _translations;
};

class ServiceTranslations {
public:
	ServiceTranslations() = default;

	void addNode(Service node) { _nodes.push_back(std::move(node)); }
	void addRequestTranslation(Translation translation) { _request_translations.push_back(std::move(translation)); }
	void addResponseTranslation(Translation translation) { _response_translations.push_back(std::move(translation)); }

	const std::vector<Service>& nodes() const { return _nodes; }
	const std::vector<Translation>& requestTranslations() const { return _request_translations; }
	const std::vector<Translation>& responseTranslations() const { return _response_translations; }
private:
	std::vector<Service> _nodes;
	std::vector<Translation> _request_translations;
	std::vector<Translation> _response_translations;
};
