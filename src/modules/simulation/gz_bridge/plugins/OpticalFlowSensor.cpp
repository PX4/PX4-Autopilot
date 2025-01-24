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

    _publisher = _node.Advertise<px4::msgs::OpticalFlow>(this->Topic());
    gzdbg << "Advertising optical flow data on: " << this->Topic() << std::endl;

    // Get camera topic from our sensor config
    auto elem = _sdf.Element();
    auto opticalFlowElem = elem->GetElement("gz:optical_flow");
    auto camera_topic = opticalFlowElem->Get<std::string>("camera_topic");

    std::string topic;
    int image_width = 0;
    int image_height = 0;
    int update_rate = 0;
    float hfov = 0;

    // Get FOV from the actual camera sensor's config
    auto sensorElem = elem->GetParent()->GetElement("sensor");
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

    gzdbg << "image_width: " << image_width << std::endl;
    gzdbg << "image_height: " << image_height << std::endl;
    gzdbg << "update_rate: " << update_rate << std::endl;
    gzdbg << "hfov: " << hfov << std::endl;

    // Subscribe to camera
    gzdbg << "Subscribing to camera topic: " << camera_topic << std::endl;
    if (!_node.Subscribe(camera_topic, &OpticalFlowSensor::OnImage, this)) {
        gzerr << "Failed to subscribe to camera topic: " << camera_topic << std::endl;
        return false;
    }

    // TODO: get from sdf
    float focal_length = (image_width / 2.0f) / tan(hfov / 2.0f);

    // Create OpticalFlow
    _optical_flow = std::make_shared<OpticalFlowOpenCV>(focal_length, focal_length, update_rate, image_width, image_height);

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

    // Store current timestamp for integration time calculation
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

    int quality = _optical_flow->calcFlow(_last_image_gray.data, _last_image_timestamp, _integration_time_us, _flow_x, _flow_y);

    msg.set_integrated_x(_flow_x);
    msg.set_integrated_y(_flow_y);
    msg.set_integration_time_us(_integration_time_us);
    msg.set_quality(quality);

    if (!_publisher.Publish(msg)) {
        gzwarn << "Failed to publish optical flow message" << std::endl;
    }

    _new_image_available = false;
    return true;
}
