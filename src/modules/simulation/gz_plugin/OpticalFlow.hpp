#pragma once

#include <gz/sensors/Sensor.hh>
#include <gz/sensors/CameraSensor.hh>
#include <gz/sensors/SensorTypes.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>
#include <opencv2/opencv.hpp>

namespace custom
{
class OpticalFlow : public gz::sensors::Sensor
{
public:
	virtual bool Load(const sdf::Sensor &_sdf) override;
	virtual bool Update(const std::chrono::steady_clock::duration &_now) override;

private:
	void OnImage(const gz::msgs::Image &_msg);

	cv::Mat prevFrame;
	gz::transport::Node node;
	gz::transport::Node::Publisher pub;

	float integrated_x{0.0};
	float integrated_y{0.0};
	float quality{0.0};
	std::chrono::steady_clock::time_point lastUpdateTime;
};
} // end namespace custom
