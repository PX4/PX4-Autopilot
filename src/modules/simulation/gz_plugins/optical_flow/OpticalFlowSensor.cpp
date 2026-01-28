/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <gz/common/Console.hh>
#include <gz/msgs/Utility.hh>
#include <gz/sensors/Util.hh>

#include "OpticalFlowSensor.hpp"
#include "opticalflow.pb.h"

using namespace custom;

bool OpticalFlowSensor::Load(const sdf::Sensor &_sdf)
{
	auto type = gz::sensors::customType(_sdf);

	if ("optical_flow" != type) {
		gzerr << "Trying to load [optical_flow] sensor, but got type [" << type << "] instead." << std::endl;
		return false;
	}

	gz::sensors::Sensor::Load(_sdf);

	std::string output_topic = this->Topic();
	_publisher = _node.Advertise<px4::msgs::OpticalFlow>(output_topic);
	gzdbg << "Advertising optical flow data on: " << output_topic << std::endl;

	std::string camera_topic = output_topic;
	size_t last_segment = camera_topic.rfind("/optical_flow/optical_flow");

	if (last_segment != std::string::npos) {
		camera_topic = camera_topic.substr(0, last_segment) + "/flow_camera/image";
	}

	int image_width = 0;
	int image_height = 0;
	int update_rate = 0;
	float hfov = 0;

	auto sensorElem = _sdf.Element()->GetParent()->GetElement("sensor");

	while (sensorElem) {
		if (sensorElem->Get<std::string>("name") == "flow_camera") {
			auto cameraElem = sensorElem->GetElement("camera");
			update_rate = sensorElem->GetElement("update_rate")->Get<int>();
			hfov = cameraElem->GetElement("horizontal_fov")->Get<double>();

			auto imageElem = cameraElem->GetElement("image");
			image_width = imageElem->GetElement("width")->Get<int>();
			image_height = imageElem->GetElement("height")->Get<int>();
			break;
		}

		sensorElem = sensorElem->GetNextElement("sensor");
	}

	gzdbg << "Camera parameters:" << std::endl
	      << "  image_width: " << image_width << std::endl
	      << "  image_height: " << image_height << std::endl
	      << "  update_rate: " << update_rate << std::endl
	      << "  hfov: " << hfov << std::endl;

	gzdbg << "Subscribing to camera topic for flow: " << camera_topic << std::endl;

	if (!_node.Subscribe(camera_topic, &OpticalFlowSensor::OnImage, this)) {
		gzerr << "Failed to subscribe to camera topic: " << camera_topic << std::endl;
		return false;
	}

	// Assume pinhole camera and 1:1 aspect ratio
	float focal_length = (image_width / 2.0f) / tan(hfov / 2.0f);
	_optical_flow = std::make_shared<OpticalFlowOpenCV>(focal_length, focal_length,
			update_rate, image_width, image_height);

	return true;
}

void OpticalFlowSensor::OnImage(const gz::msgs::Image &image_msg)
{
	if (image_msg.width() == 0 || image_msg.height() == 0) {
		gzerr << "Invalid image dimensions" << std::endl;
		return;
	}

	if (image_msg.pixel_format_type() == gz::msgs::PixelFormatType::RGB_INT8) {
		cv::Mat temp(image_msg.height(), image_msg.width(), CV_8UC3);
		std::memcpy(temp.data, image_msg.data().c_str(), image_msg.data().size());
		cv::cvtColor(temp, _last_image_gray, cv::COLOR_RGB2GRAY);

	} else if (image_msg.pixel_format_type() == gz::msgs::PixelFormatType::L_INT8) {
		std::memcpy(_last_image_gray.data, image_msg.data().c_str(), image_msg.data().size());

	} else {
		gzerr << "Unsupported image format" << std::endl;
		return;
	}

	uint32_t current_timestamp = (image_msg.header().stamp().sec() * 1000000ULL +
				      image_msg.header().stamp().nsec() / 1000ULL) & 0xFFFFFFFF;

	if (_last_image_timestamp != 0) {
		_integration_time_us = (current_timestamp - _last_image_timestamp) & 0xFFFFFFFF;
	}

	_last_image_timestamp = current_timestamp;
	_new_image_available = true;
}

bool OpticalFlowSensor::Update(const std::chrono::steady_clock::duration &_now)
{
	if (!_new_image_available) {
		return true;
	}

	px4::msgs::OpticalFlow msg;
	msg.set_time_usec(_last_image_timestamp);

	float flow_x = 0.f;
	float flow_y = 0.f;

	int quality = _optical_flow->calcFlow(_last_image_gray.data, _last_image_timestamp,
					      _integration_time_us, flow_x, flow_y);

	msg.set_integrated_x(flow_x);
	msg.set_integrated_y(flow_y);
	msg.set_integration_time_us(_integration_time_us);
	msg.set_quality(quality);

	if (!_publisher.Publish(msg)) {
		gzwarn << "Failed to publish optical flow message" << std::endl;
	}

	_new_image_available = false;
	return true;
}
