#pragma once

#include <gz/msgs/image.pb.h>

int calculate_flow(const gz::msgs::Image &image_msg, uint64_t sim_time, int &integration_time, float &flow_x,
		   float &flow_y);
