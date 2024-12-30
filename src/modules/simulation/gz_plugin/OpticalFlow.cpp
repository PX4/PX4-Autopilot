#include <gz/common/Console.hh>
#include <gz/msgs/Utility.hh>
#include <gz/sensors/Util.hh>

#include "OpticalFlow.hpp"
#include "optical_flow.pb.h"

using namespace custom;

bool OpticalFlow::Load(const sdf::Sensor &_sdf)
{
    auto type = gz::sensors::customType(_sdf);
    if ("optical_flow" != type) {
        gzerr << "Trying to load [optical_flow] sensor, but got type [" << type << "] instead." << std::endl;
        return false;
    }

    gz::sensors::Sensor::Load(_sdf);

    this->pub = this->node.Advertise<sensor_msgs::msgs::OpticalFlow>(this->Topic());
    gzdbg << "Advertising optical flow data on: " << this->Topic() << std::endl;

    // Get camera topic from our sensor config
    auto elem = _sdf.Element();
    auto opticalFlowElem = elem->GetElement("gz:optical_flow");
    auto cameraTopic = opticalFlowElem->Get<std::string>("camera_topic");

    // Get FOV from the actual camera sensor's config
    auto cameraElem = elem->GetParent()->GetElement("sensor");
    while (cameraElem) {
        if (cameraElem->Get<std::string>("name") == "flow_camera") {
            auto camera = cameraElem->GetElement("camera");
            this->horizontal_fov = camera->GetElement("horizontal_fov")->Get<double>();
            this->vertical_fov = this->horizontal_fov * 0.75;  // Assume 4:3 aspect ratio
            break;
        }
        cameraElem = cameraElem->GetNextElement("sensor");
    }

    gzdbg << "Using camera FOV - horizontal: " << this->horizontal_fov
          << " vertical: " << this->vertical_fov << std::endl;

    // Subscribe to camera
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
    if (_msg.width() == 0 || _msg.height() == 0) {
        gzerr << "Invalid image dimensions" << std::endl;
        return;
    }

    cv::Mat frame;

    // Convert image to grayscale
    if (_msg.pixel_format_type() == gz::msgs::PixelFormatType::RGB_INT8) {
        frame = cv::Mat(_msg.height(), _msg.width(), CV_8UC3, (void *)_msg.data().c_str());
        cv::cvtColor(frame, frame, cv::COLOR_RGB2GRAY);
    } else if (_msg.pixel_format_type() == gz::msgs::PixelFormatType::L_INT8) {
        frame = cv::Mat(_msg.height(), _msg.width(), CV_8UC1, (void *)_msg.data().c_str());
    } else {
        gzerr << "Unsupported image format" << std::endl;
        return;
    }

    // Preprocess image
    cv::GaussianBlur(frame, frame, this->blur_size, this->blur_sigma);

    // Scale down for performance
    cv::Mat scaled_frame;
    cv::resize(frame, scaled_frame, cv::Size(), this->scale_factor, this->scale_factor);

    ProcessFlow(scaled_frame);
}

void OpticalFlow::ProcessFlow(const cv::Mat &current_frame)
{
    if (!flow_initialized) {
        current_frame.copyTo(prevFrame);
        flow_initialized = true;
        return;
    }

    // Detect features in previous frame
    std::vector<cv::Point2f> current_points;
    std::vector<uchar> status;
    std::vector<float> err;

    if (prev_points.empty()) {
        cv::goodFeaturesToTrack(prevFrame, prev_points, max_corners, quality_level, min_distance);
    }

    if (prev_points.empty()) {
        gzwarn << "No features detected in previous frame" << std::endl;
        current_frame.copyTo(prevFrame);
        return;
    }

    // Calculate optical flow
    cv::calcOpticalFlowPyrLK(prevFrame, current_frame, prev_points, current_points, status, err);

    // Filter valid points and calculate flow
    std::vector<cv::Point2f> good_old, good_new;
    for (size_t i = 0; i < status.size(); i++) {
        if (status[i]) {
            good_old.push_back(prev_points[i]);
            good_new.push_back(current_points[i]);
        }
    }

    if (good_new.empty() || good_old.empty()) {
        gzwarn << "No valid flow vectors" << std::endl;
        quality = 0;
        current_frame.copyTo(prevFrame);
        prev_points.clear();
        return;
    }

    // Calculate average flow
    cv::Point2f mean_flow(0, 0);
    for (size_t i = 0; i < good_new.size(); i++) {
        mean_flow += good_new[i] - good_old[i];
    }
    mean_flow = mean_flow * (1.0f / good_new.size());

    // Convert to radians using FOV and resolution
    double rad_per_pixel_x = horizontal_fov / (current_frame.cols / double(scale_factor));
    double rad_per_pixel_y = vertical_fov / (current_frame.rows / double(scale_factor));

    integrated_x = (double)mean_flow.x * rad_per_pixel_x;
    integrated_y = (double)mean_flow.y * rad_per_pixel_y;

    // Calculate quality metric
    std::vector<float> flow_magnitudes;
    for (size_t i = 0; i < good_new.size(); i++) {
        cv::Point2f flow = good_new[i] - good_old[i];
        flow_magnitudes.push_back(cv::norm(flow));
    }

    float avg_magnitude = 0;
    if (!flow_magnitudes.empty()) {
        avg_magnitude = std::accumulate(flow_magnitudes.begin(), flow_magnitudes.end(), 0.0f)
                       / flow_magnitudes.size();
    }

    // Compute quality based on flow consistency and magnitude
    float std_dev = 0;
    for (float mag : flow_magnitudes) {
        std_dev += (mag - avg_magnitude) * (mag - avg_magnitude);
    }
    std_dev = std_dev > 0 ? sqrt(std_dev / flow_magnitudes.size()) : 0;

    // Higher quality when flow is consistent (low std_dev) and has reasonable magnitude
    quality = std::min(255.0f, (avg_magnitude * 100.0f) / (std_dev + 1.0f));

    // Check for excessive motion
    if (std::abs(integrated_x) > M_PI_2 || std::abs(integrated_y) > M_PI_2) {
        gzwarn << "Excessive motion detected" << std::endl;
        quality = 0;
    }

    // Update state for next iteration
    current_frame.copyTo(prevFrame);
    prev_points = good_new;  // Use current good points for next iteration
}

bool OpticalFlow::Update(const std::chrono::steady_clock::duration &_now)
{
    auto currentTime = std::chrono::steady_clock::now();
    auto deltaTime = std::chrono::duration_cast<std::chrono::microseconds>(
        currentTime - this->lastUpdateTime);

    sensor_msgs::msgs::OpticalFlow msg;
    msg.set_time_usec(std::chrono::duration_cast<std::chrono::microseconds>(_now).count());
    msg.set_integration_time_us(deltaTime.count());
    msg.set_integrated_x(this->integrated_x);
    msg.set_integrated_y(this->integrated_y);
    msg.set_quality(this->quality);

    if (!this->pub.Publish(msg)) {
        gzwarn << "Failed to publish optical flow message" << std::endl;
    }

    this->lastUpdateTime = currentTime;
    return true;
}
