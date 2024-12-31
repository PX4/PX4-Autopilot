#pragma once

#include <gz/sensors/Sensor.hh>
#include <gz/sensors/CameraSensor.hh>
#include <gz/sensors/SensorTypes.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>
#include <opencv2/opencv.hpp>
#include <numeric>
#include <vector>

namespace custom
{
class OpticalFlow : public gz::sensors::Sensor
{
public:
    virtual bool Load(const sdf::Sensor &_sdf) override;
    virtual bool Update(const std::chrono::steady_clock::duration &_now) override;

private:
    void OnImage(const gz::msgs::Image &_msg);
    void ProcessFlow(const cv::Mat &current_frame);

    cv::Mat prevFrame;
    gz::transport::Node node;
    gz::transport::Node::Publisher pub;

    // Camera parameters
    double horizontal_fov{0.79};  // Default FOV in radians
    double vertical_fov{0.6};    // Default FOV in radians

    // Flow computation parameters
    const int max_corners{100};
    const double quality_level{0.3};
    const double min_distance{7.0};

    // Flow state
    double integrated_x{0.0};
    double integrated_y{0.0};
    double quality{0.0};
    std::chrono::steady_clock::time_point lastUpdateTime;

    // Image processing parameters
    const cv::Size blur_size{5, 5};
    const double blur_sigma{1.5};
    const float scale_factor{0.5};  // Scale image down for performance

    bool flow_updated{false};

    // Previous points for optical flow
    std::vector<cv::Point2f> prev_points;
    bool flow_initialized{false};
};
} // end namespace custom
