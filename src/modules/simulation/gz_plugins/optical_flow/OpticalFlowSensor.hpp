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

#pragma once

#include <gz/sensors/Sensor.hh>
#include <gz/sensors/CameraSensor.hh>
#include <gz/sensors/SensorTypes.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>
#include <opencv2/opencv.hpp>
#include <numeric>
#include <vector>
#include <memory>

#include "flow_opencv.hpp"

namespace custom
{
class OpticalFlowSensor : public gz::sensors::Sensor
{
public:
	virtual bool Load(const sdf::Sensor &_sdf) override;
	virtual bool Update(const std::chrono::steady_clock::duration &_now) override;

private:
	void OnImage(const gz::msgs::Image &_msg);

	gz::transport::Node _node;
	gz::transport::Node::Publisher _publisher;

	// Flow
	std::shared_ptr<OpticalFlowOpenCV> _optical_flow {nullptr};
	int _integration_time_us;

	// Camera
	double _horizontal_fov {0.0};
	double _vertical_fov {0.0};

	cv::Mat _last_image_gray;
	uint32_t _last_image_timestamp {0};
	bool _new_image_available {false};
};

} // end namespace custom
