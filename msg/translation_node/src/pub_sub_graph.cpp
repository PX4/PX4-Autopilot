/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#include "pub_sub_graph.h"
#include "util.h"

PubSubGraph::PubSubGraph(rclcpp::Node &node, const TopicTranslations &translations) : _node(node) {

	std::unordered_map<std::string, std::set<MessageVersionType>> known_versions;

	for (const auto& topic : translations.topics()) {
		const std::string full_topic_name = getFullTopicName(_node.get_effective_namespace(), topic.id.topic_name);
		_known_topics_warned.insert({full_topic_name, false});

		const MessageIdentifier id{full_topic_name, topic.id.version};
		NodeDataPubSub node_data{topic.subscription_factory, topic.publication_factory, id, topic.max_serialized_message_size};
		_pub_sub_graph.addNodeIfNotExists(id, std::move(node_data), topic.message_buffer);
		known_versions[full_topic_name].insert(id.version);
	}

	auto get_full_topic_names = [this](std::vector<MessageIdentifier> ids) {
		for (auto& id : ids) {
			id.topic_name = getFullTopicName(_node.get_effective_namespace(), id.topic_name);
		}
		return ids;
	};

	for (const auto& translation : translations.translations()) {
		const std::vector<MessageIdentifier> inputs = get_full_topic_names(translation.inputs);
		const std::vector<MessageIdentifier> outputs = get_full_topic_names(translation.outputs);
		_pub_sub_graph.addTranslation(translation.cb, inputs, outputs);
	}

	printTopicInfo(known_versions);
	handleLargestTopic(known_versions);
}

void PubSubGraph::updateCurrentTopics(const std::vector<TopicInfo> &topics) {

	_pub_sub_graph.iterateNodes([](const MessageIdentifier& type, const Graph<NodeDataPubSub>::MessageNodePtr& node) {
		node->data().has_external_publisher = false;
		node->data().has_external_subscriber = false;
		node->data().visited = false;
	});

	for (const auto& info : topics) {
		const auto [non_versioned_topic_name, version] = getNonVersionedTopicName(info.topic_name);
		auto maybe_node = _pub_sub_graph.findNode({non_versioned_topic_name, version});
		if (!maybe_node) {
			auto known_topic_iter = _known_topics_warned.find(non_versioned_topic_name);
			if (known_topic_iter != _known_topics_warned.end() && !known_topic_iter->second) {
				RCLCPP_WARN(_node.get_logger(), "No translation available for version %i of topic %s", version, non_versioned_topic_name.c_str());
				known_topic_iter->second = true;
			}
			continue;
		}
		const auto& node = maybe_node.value();

		if (info.num_publishers > 0) {
			node->data().has_external_publisher = true;
		}
		if (info.num_subscribers > 0) {
			node->data().has_external_subscriber = true;
		}
	}

	// Iterate connected graph segments
	_pub_sub_graph.iterateNodes([this](const MessageIdentifier& type, const Graph<NodeDataPubSub>::MessageNodePtr& node) {
		if (node->data().visited) {
			return;
		}
		node->data().visited = true;

		// Count the number of external subscribers and publishers for each connected graph
		int num_publishers = 0;
		int num_subscribers = 0;
		int num_subscribers_without_publisher = 0;

		_pub_sub_graph.iterateBFS(node, [&](const Graph<NodeDataPubSub>::MessageNodePtr& node) {
			if (node->data().has_external_publisher) {
				++num_publishers;
			}
			if (node->data().has_external_subscriber) {
				++num_subscribers;
				if (!node->data().has_external_publisher) {
					++num_subscribers_without_publisher;
				}
			}
		});

		// We need to instantiate publishers and subscribers if:
		// - there are multiple publishers and at least 1 subscriber
		// - there is 1 publisher and at least 1 subscriber on another node
		// Note that in case of splitting or merging topics, this might create more entities than actually needed
		const bool require_translation = (num_publishers >= 2 && num_subscribers >= 1)
										 || (num_publishers == 1 && num_subscribers_without_publisher >= 1);
		if (require_translation) {
			_pub_sub_graph.iterateBFS(node, [&](const Graph<NodeDataPubSub>::MessageNodePtr& node) {
				node->data().visited = true;
				// Has subscriber(s)?
				if (node->data().has_external_subscriber && !node->data().publication) {
					RCLCPP_INFO(_node.get_logger(), "Found subscriber for topic '%s', version: %i, adding publisher", node->data().topic_name.c_str(), node->data().version);
					node->data().publication = node->data().publication_factory(_node);
				} else if (!node->data().has_external_subscriber && node->data().publication) {
					RCLCPP_INFO(_node.get_logger(), "No subscribers for topic '%s', version: %i, removing publisher", node->data().topic_name.c_str(), node->data().version);
					node->data().publication.reset();
				}
				// Has publisher(s)?
				if (node->data().has_external_publisher && !node->data().subscription) {
					RCLCPP_INFO(_node.get_logger(), "Found publisher for topic '%s', version: %i, adding subscriber", node->data().topic_name.c_str(), node->data().version);
					node->data().subscription = node->data().subscription_factory(_node, [this, node_cpy=node]() {
						onSubscriptionUpdate(node_cpy);
					});
				} else if (!node->data().has_external_publisher && node->data().subscription) {
					RCLCPP_INFO(_node.get_logger(), "No publishers for topic '%s', version: %i, removing subscriber", node->data().topic_name.c_str(), node->data().version);
					node->data().subscription.reset();
				}
			});

		} else {
			// Reset any publishers or subscribers
			_pub_sub_graph.iterateBFS(node, [&](const Graph<NodeDataPubSub>::MessageNodePtr& node) {
				node->data().visited = true;
				if (node->data().publication) {
					RCLCPP_INFO(_node.get_logger(), "Removing publisher for topic '%s', version: %i",
								node->data().topic_name.c_str(), node->data().version);
					node->data().publication.reset();
				}
				if (node->data().subscription) {
					RCLCPP_INFO(_node.get_logger(), "Removing subscriber for topic '%s', version: %i",
								node->data().topic_name.c_str(), node->data().version);
					node->data().subscription.reset();
				}
			});
		}
	});
}

void PubSubGraph::onSubscriptionUpdate(const Graph<NodeDataPubSub>::MessageNodePtr& node) {
	_pub_sub_graph.translate(
			node,
			[this](const Graph<NodeDataPubSub>::MessageNodePtr& node) {
				if (node->data().publication != nullptr) {
					const auto ret = rcl_publish(node->data().publication->get_publisher_handle().get(),
								node->buffer().get(), nullptr);
					if (ret != RCL_RET_OK) {
						RCLCPP_WARN_ONCE(_node.get_logger(), "Failed to publish on topic '%s', version: %i",
										node->data().topic_name.c_str(), node->data().version);
					}
				}
			});

}

void PubSubGraph::printTopicInfo(const std::unordered_map<std::string, std::set<MessageVersionType>>& known_versions) const {
	// Print info about known versions
	RCLCPP_INFO(_node.get_logger(), "Registered pub/sub topics and versions:");
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


void PubSubGraph::handleLargestTopic(const std::unordered_map<std::string, std::set<MessageVersionType>> &known_versions) {
	// FastDDS caches some type information per DDS participant when first creating a publisher or subscriber for a given
	// type. The information that is relevant for us is the maximum serialized message size.
	// Since different versions can have different sizes, we need to ensure the first publication or subscription
	// happens with the version of the largest size. Otherwise, an out-of-memory exception can be triggered.
	// And the type must continue to be in use (so we cannot delete it)
	for (const auto& [topic_name, versions] : known_versions) {
		size_t max_serialized_message_size = 0;
		const PublicationFactoryCB* publication_factory_for_max = nullptr;
		for (auto version : versions) {
			const auto& node = _pub_sub_graph.findNode(MessageIdentifier{topic_name, version});
			assert(node);
			const auto& node_data = node.value()->data();
			if (node_data.max_serialized_message_size > max_serialized_message_size) {
				max_serialized_message_size = node_data.max_serialized_message_size;
				publication_factory_for_max = &node_data.publication_factory;
			}
		}
		if (publication_factory_for_max) {
			_largest_topic_publications.emplace_back((*publication_factory_for_max)(_node));
		}
	}
}
