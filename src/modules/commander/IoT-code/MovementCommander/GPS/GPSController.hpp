#ifndef GPS_CONTROLLER_HPP
#define GPS_CONTROLLER_HPP

#include <cmath>
#include <px4_platform_common/px4_config.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/uORB.h>

class GPSController {
private:
    int gps_sub;
    struct sensor_gps_s gps_s;
    px4_pollfd_struct_t gps_fds[1];

    double* start_latitude;
    double* start_longitude;
    double* start_altitude;

    double* waypoint_latitude;
    double* waypoint_longitude;
    double* waypoint_altitude;

    double* getDistancesFromStartOrWaypoint(bool Start);

public:
    GPSController(int CBinterval_ms);
    ~GPSController();

    bool getposition(double *latitude, double *longitude, double *altitude, int poll_T_ms);
    double* getDistancesFromStart();
    double* getDistancesFromWaypoint();
    double meters_to_longitude(double meters);
};

#endif /* GPS_CONTROLLER_HPP */
