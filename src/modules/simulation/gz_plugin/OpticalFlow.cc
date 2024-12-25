// OpticalFlow.cc
#include <gz/common/Console.hh>
#include <gz/msgs/Utility.hh>
#include <gz/sensors/Util.hh>

#include "OpticalFlow.hh"
#include "optical_flow.pb.h"

using namespace custom;

bool OpticalFlow::Load(const sdf::Sensor &_sdf)
{
	auto type = gz::sensors::customType(_sdf);

	if ("optical_flow" != type) {
		gzerr << "Trying to load [optical_flow] sensor, but got type [" << type << "] instead." << std::endl;
		return false;
	}

	gzdbg << "Loading optical flow sensor..." << std::endl;
	gz::sensors::Sensor::Load(_sdf);

	gzdbg << "Adverising optical flow sensor on: " << this->Topic() << std::endl;
	this->pub = this->node.Advertise<sensor_msgs::msgs::OpticalFlow>(this->Topic());

	auto elem = _sdf.Element();
	auto opticalFlowElem = elem->GetElement("gz:optical_flow");
	auto cameraTopic = opticalFlowElem->Get<std::string>("camera_topic");

	gzdbg << "Subscribing to camera topic: " << cameraTopic << std::endl;

	if (!this->node.Subscribe(cameraTopic, &OpticalFlow::OnImage, this)) {
		gzerr << "Failed to subscribe to camera topic: " << cameraTopic << std::endl;
		return false;
	}

	this->lastUpdateTime = std::chrono::steady_clock::now();

	return true;
}

void OpticalFlow::OnImage(const gz::msgs::Image &_msg)
{
	cv::Mat frame;

	if (_msg.pixel_format_type() == gz::msgs::PixelFormatType::RGB_INT8) {
		frame = cv::Mat(_msg.height(), _msg.width(), CV_8UC3, (void *)_msg.data().c_str());
		cv::cvtColor(frame, frame, cv::COLOR_RGB2GRAY);

	} else if (_msg.pixel_format_type() == gz::msgs::PixelFormatType::L_INT8) {
		frame = cv::Mat(_msg.height(), _msg.width(), CV_8UC1, (void *)_msg.data().c_str());

	} else {
		gzerr << "Unsupported image format" << std::endl;
		return;
	}

	if (prevFrame.empty()) {
		frame.copyTo(prevFrame);
		return;
	}

	cv::Mat flow;
	cv::calcOpticalFlowFarneback(prevFrame, frame, flow,
				     0.5, // Pyramid scale
				     3,   // Pyramid levels
				     15,  // Window size
				     3,   // Iteratpions
				     5,   // Poly_n
				     1.2, // Poly_sigma
				     0);  // Flags

	// Calculate average flow and quality
	cv::Scalar meanFlow = cv::mean(flow);

	// Update integrated flow (in radians)
	// TODO: FOV from SDF
	// Assuming 60° horizontal field of view and 45° vertical field of view
	const double rad_per_pixel_x = (M_PI / 3.0) / frame.cols;  // 60° in radians / width
	const double rad_per_pixel_y = (M_PI / 4.0) / frame.rows;  // 45° in radians / height

	this->integrated_x = meanFlow[0] * rad_per_pixel_x;
	this->integrated_y = meanFlow[1] * rad_per_pixel_y;

	// Calculate quality (0-255)
	cv::Mat magnitude, angle;
	cv::cartToPolar(flow.col(0), flow.col(1), magnitude, angle);
	this->quality = cv::mean(magnitude)[0] * 255.0;

	if (this->quality > 255.0f) {
		this->quality = 255.0f;
	}

	frame.copyTo(prevFrame);
}

bool OpticalFlow::Update(const std::chrono::steady_clock::duration &_now)
{
	auto currentTime = std::chrono::steady_clock::now();
	auto deltaTime = std::chrono::duration_cast<std::chrono::microseconds>(currentTime - this->lastUpdateTime);

	sensor_msgs::msgs::OpticalFlow msg;
	msg.set_time_usec(std::chrono::duration_cast<std::chrono::microseconds>(_now).count());
	msg.set_integration_time_us(deltaTime.count());
	msg.set_integrated_x(this->integrated_x);
	msg.set_integrated_y(this->integrated_y);
	msg.set_quality(this->quality);
	msg.set_time_delta_distance_us(deltaTime.count());

	this->pub.Publish(msg);
	this->lastUpdateTime = currentTime;

	return true;
}
