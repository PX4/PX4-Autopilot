#ifndef SPEEDCONTROLLER_HPP
#define SPEEDCONTROLLER_HPP

#include <uORB/topics/sensor_combined.h>
#include <px4_platform_common/px4_config.h>
#include <uORB/uORB.h>
#include <cmath>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/log.h>

class SpeedController {
private:
    int Sens_sub;
    struct sensor_combined_s Sens_st;
    px4_pollfd_struct_t comb_fds[1];

    float* velocities = new float[3];

    float* positions = new float[3]; // Initial position

    hrt_abstime prev_timestamp = 0;

public:
    // Constructor
    SpeedController();
    ~SpeedController();
    bool update();
    float* getPostion();
    float* getVelocities();


};

#endif // SPEEDCONTROLLER_HPP
