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
    float _flow_x {0.0f};
    float _flow_y {0.0f};
    int _integration_time_us;

    // Camera
    double _horizontal_fov {0.0};
    double _vertical_fov {0.0};

    cv::Mat _last_image_gray;
    uint32_t _last_image_timestamp {0};
    bool _new_image_available {false};
};

} // end namespace custom
