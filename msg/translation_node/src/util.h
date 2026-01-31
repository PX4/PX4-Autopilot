/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <string>
#include <cstdint>

using MessageVersionType = uint32_t;

static inline std::string getVersionedTopicName(const std::string& topic_name, MessageVersionType version) {
	// version == 0 can be used to transition from non-versioned topics to versioned ones
	if (version == 0) {
		return topic_name;
	}
	return topic_name + "_v" + std::to_string(version);
}

static inline std::pair<std::string, MessageVersionType> getNonVersionedTopicName(const std::string& topic_name) {
	// topic name has the form <name>_v<version>, or just <name> (with version=0)
	auto pos = topic_name.find_last_of("_v");
	// Ensure there's at least one more char after the found string
	if (pos == std::string::npos || pos + 2 > topic_name.length()) {
		return std::make_pair(topic_name, 0);
	}
	std::string non_versioned_topic_name = topic_name.substr(0, pos - 1);
	std::string version = topic_name.substr(pos + 1);
	// Ensure only digits are in the version string
	for (char c : version) {
		if (!std::isdigit(c)) {
			return std::make_pair(topic_name, 0);
		}
	}
	return std::make_pair(non_versioned_topic_name, std::stol(version));
}

/**
 * Get the full topic name, including namespace from a topic name.
 * namespace_name should be set to Node::get_effective_namespace()
 */
static inline std::string getFullTopicName(const std::string& namespace_name, const std::string& topic_name) {
	std::string full_topic_name = topic_name;
	if (!full_topic_name.empty() && full_topic_name[0] != '/') {
		if (namespace_name.empty() || namespace_name.back() != '/') {
			full_topic_name = '/' + full_topic_name;
		}
		full_topic_name = namespace_name + full_topic_name;
	}
	return full_topic_name;
}
