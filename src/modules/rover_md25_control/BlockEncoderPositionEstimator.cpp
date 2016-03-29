#include "BlockEncoderPositionEstimator.hpp"
#include "geo/geo.h"
#include "systemlib/err.h"

BlockEncoderPositionEstimator::BlockEncoderPositionEstimator() :
    SuperBlock(NULL, "ENCP"),
    // subscriptions
    _att(ORB_ID(vehicle_attitude), 20, 0, &getSubscriptions()), // 50 Hz
    _param_update(ORB_ID(parameter_update), 1000, 0, &getSubscriptions()), // limit to 1 Hz
    _encoders(ORB_ID(encoders), 100, 0, &getSubscriptions()), // limit to  10 Hz
    // publications
    _lpos(ORB_ID(vehicle_local_position), ORB_PRIO_DEFAULT, &getPublications()),
    _pos(ORB_ID(vehicle_global_position), ORB_PRIO_DEFAULT, &getPublications()),
    // data
    _rWheel(this, "RWHEEL"),
    _pulsesPerRev(this, "PPR"),
    _poll(),
    _timeStamp(0),
    _pos_ref()
{
    _poll.fd = _att.getHandle();
    _poll.events = POLLIN;
}

void BlockEncoderPositionEstimator::update()
{

    // wait for a sensor update, check for exit condition every 100 ms
    if (poll(&_poll, 1, 100) < 0) { return; } // poll error

    uint64_t newTimeStamp = hrt_absolute_time();
    float dt = (newTimeStamp - _timeStamp) / 1.0e6f;
    _timeStamp = newTimeStamp;

    // check for sane values of dt
    // to prevent large control responses
    if (dt > 1.0f || dt < 0) { return; }

    // set dt for all child blocks
    setDt(dt);

    // check for new updates
    if (_param_update.updated()) { updateParams(); }

    // get new information from subscriptions
    updateSubscriptions();

    // roughtly estimate position
    float countsTometers = 2 * M_PI_F * _rWheel.get() / _pulsesPerRev.get();
    float x = countsTometers * (_encoders.get().counts[0] + _encoders.get().counts[1]) / 2.0f;
    float xDot = countsTometers * (_encoders.get().velocity[0] + _encoders.get().velocity[1]) / 2.0f;
    struct vehicle_local_position_s & lpos = _lpos.get();
    lpos.timestamp = _timeStamp;
    lpos.xy_valid = true;
    lpos.z_valid = true;
    lpos.v_xy_valid = true;
    lpos.v_z_valid = true;
    lpos.x = x;
    lpos.y = 0;
    lpos.z = -10; // must be greater than 5 to start mission
    lpos.vx = xDot;
    lpos.vy = 0;
    lpos.vz = 0;
    lpos.yaw = _att.get().yaw;
    lpos.xy_global = true;
    lpos.z_global = true;
    lpos.ref_timestamp = _timeStamp;
    lpos.ref_lat = 0;
    lpos.ref_lon = 0;
    lpos.ref_alt = 0;
    lpos.dist_bottom = 0;
    lpos.dist_bottom_rate = 0;
    lpos.surface_bottom_timestamp = _timeStamp;
    lpos.dist_bottom_valid = true;
    lpos.eph = 1;
    lpos.epv = 1;
    _lpos.update();

    // assume we are always at lat0, lon0 for now
    double latDeg = 0;
    double lonDeg = 0;
    map_projection_reproject(&_pos_ref, x, 0, &latDeg, &lonDeg);

    struct vehicle_global_position_s & pos = _pos.get();
    pos.timestamp = _timeStamp;
    pos.time_utc_usec = _timeStamp;
    pos.lat = latDeg;
    pos.lon = lonDeg;
    pos.alt = 0;
    pos.vel_n = xDot;
    pos.vel_e = 0;
    pos.vel_d = 0;
    pos.yaw = _att.get().yaw;
    pos.eph = 1;
    pos.epv = 1;
    pos.terrain_alt = 0;
    pos.terrain_alt_valid = false;
    pos.dead_reckoning = true;
    _pos.update();
}
