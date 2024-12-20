#include "gz_camera.hpp"

#include <gz/msgs.hh>
#include <gz/math.hh>
#include <gz/transport.hh>

// #include <opencv2/opencv.hpp>
#include "flow_opencv.hpp"
#include <iostream>

OpticalFlowOpenCV *_optical_flow = nullptr;
int _dt_us = 0;

int calculate_flow(const gz::msgs::Image &image_msg, uint64_t sim_time, int &integration_time, float &flow_x,
		   float &flow_y)
{
	if (!_optical_flow) {
		float hfov = 1.74;
		int output_rate = 30;
		int image_width = image_msg.width();
		int image_height = image_msg.height();
		float focal_length = (image_width / 2.0f) / tan(hfov / 2.0f);

		_optical_flow = new OpticalFlowOpenCV(focal_length, focal_length, output_rate, image_width, image_height);
	}

	cv::Mat image = cv::Mat(image_msg.height(), image_msg.width(), CV_8UC3, (void *)image_msg.data().c_str());

	cv::Mat gray_image;
	cv::cvtColor(image, gray_image, cv::COLOR_RGB2GRAY);

	int quality = _optical_flow->calcFlow(gray_image.data, sim_time, integration_time, flow_x, flow_y);

	return quality;
}
