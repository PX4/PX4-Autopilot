/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <rclcpp/rclcpp.hpp>
#include "pub_sub_graph.h"
#include "service_graph.h"
#include <functional>

class Monitor {
public:
	explicit Monitor(rclcpp::Node &node, PubSubGraph* pub_sub_graph, ServiceGraph* service_graph);

	void updateNow();

private:
	rclcpp::Node &_node;
	PubSubGraph* _pub_sub_graph{nullptr};
	ServiceGraph* _service_graph{nullptr};
	rclcpp::TimerBase::SharedPtr _node_update_timer;
};
