
#pragma once

#include "rclcpp/rclcpp.hpp"

namespace px4
{

class Node : public rclcpp::Node
{
public:
	explicit Node(const char *node_name) :
		rclcpp::Node(node_name)
	{

	}


	~Node() override = default;

private:

};

};


// int main(int argc, char *argv[])
// {
// 	rclcpp::init(argc, argv);
// 	rclcpp::spin(std::make_shared<MulticopterRateControl>());
// 	rclcpp::shutdown();
// 	return 0;
// }


// #include "rclcpp_components/register_node_macro.hpp"

// // Register the component with class_loader.
// // This acts as a sort of entry point, allowing the component to be discoverable when its library
// // is being loaded into a running process.
// RCLCPP_COMPONENTS_REGISTER_NODE(composition::Client)
