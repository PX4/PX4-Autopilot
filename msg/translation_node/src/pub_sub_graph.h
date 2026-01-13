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

class PubSubGraph {
public:
	struct TopicInfo {
		std::string topic_name; ///< fully qualified topic name (with namespace)
		int num_subscribers; ///< does not include this node's subscribers
		int num_publishers; ///< does not include this node's publishers
	};

	PubSubGraph(rclcpp::Node& node, const TopicTranslations& translations);

	void updateCurrentTopics(const std::vector<TopicInfo>& topics);

private:
	struct NodeDataPubSub {
		explicit NodeDataPubSub(SubscriptionFactoryCB subscription_factory, PublicationFactoryCB publication_factory,
								const MessageIdentifier& id, size_t max_serialized_message_size)
				: subscription_factory(std::move(subscription_factory)), publication_factory(std::move(publication_factory)),
				topic_name(id.topic_name), version(id.version), max_serialized_message_size(max_serialized_message_size)
		{ }

		const SubscriptionFactoryCB subscription_factory;
		const PublicationFactoryCB publication_factory;
		const std::string topic_name;
		const MessageVersionType version;
		const size_t max_serialized_message_size;

		// Keep track if there's currently a publisher/subscriber
		bool has_external_publisher{false};
		bool has_external_subscriber{false};

		rclcpp::SubscriptionBase::SharedPtr subscription;
		rclcpp::PublisherBase::SharedPtr publication;

		bool visited{false};
	};

	void onSubscriptionUpdate(const Graph<NodeDataPubSub>::MessageNodePtr& node);
	void printTopicInfo(const std::unordered_map<std::string, std::set<MessageVersionType>>& known_versions) const;
	void handleLargestTopic(const std::unordered_map<std::string, std::set<MessageVersionType>>& known_versions);

	rclcpp::Node& _node;
	Graph<NodeDataPubSub> _pub_sub_graph;
	std::unordered_map<std::string, bool> _known_topics_warned;

	std::vector<rclcpp::PublisherBase::SharedPtr> _largest_topic_publications;
};
