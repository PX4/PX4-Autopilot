#include "BlockEncoderPositionEstimator.hpp"
#include "geo/geo.h"
#include "systemlib/err.h"

BlockEncoderPositionEstimator::BlockEncoderPositionEstimator() :
	SuperBlock(NULL, "ENCP"),
	_att(&getSubscriptions(), ORB_ID(vehicle_attitude), 20), // 50 Hz
	_param_update(&getSubscriptions(),
		      ORB_ID(parameter_update), 1000), // limit to 1 Hz
	_localPos(&getPublications(), ORB_ID(vehicle_local_position)),
	_pos(&getPublications(), ORB_ID(vehicle_global_position)),
	_encoders(&getSubscriptions(),
		  ORB_ID(encoders), 100), // limit to  10 Hz
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
	float x = countsTometers * (_encoders.counts[0] + _encoders.counts[1]) / 2.0f;
	float xDot = countsTometers * (_encoders.velocity[0] + _encoders.velocity[1]) / 2.0f;
	_localPos.timestamp = _timeStamp;
	_localPos.xy_valid = true;
	_localPos.z_valid = true;
	_localPos.v_xy_valid = true;
	_localPos.v_z_valid = true;
	_localPos.x = x;
	_localPos.y = 0;
	_localPos.z = -10; // must be greater than 5 to start mission
	_localPos.vx = xDot;
	_localPos.vy = 0;
	_localPos.vz = 0;
	_localPos.yaw = _att.yaw;
	_localPos.xy_global = true;
	_localPos.z_global = true;
	_localPos.ref_timestamp = _timeStamp;
	_localPos.ref_lat = 0;
	_localPos.ref_lon = 0;
	_localPos.ref_alt = 0;
	_localPos.landed = false;
	_localPos.dist_bottom = 0;
	_localPos.dist_bottom_rate = 0;
	_localPos.surface_bottom_timestamp = _timeStamp;
	_localPos.dist_bottom_valid = true;
	_localPos.eph = 1;
	_localPos.epv = 1;
	_localPos.update();

	// assume we are always at lat0, lon0 for now
	double latDeg = 0;
	double lonDeg = 0;
	map_projection_reproject(&_pos_ref, x, 0, &latDeg, &lonDeg);

	_pos.timestamp = _timeStamp;
	_pos.time_gps_usec = _timeStamp;
	_pos.lat = 1e7 * latDeg;
	_pos.lon = 1e7 * lonDeg;
	_pos.alt = 0;
	_pos.vel_n = xDot;
	_pos.vel_e = 0;
	_pos.vel_d = 0;
	_pos.yaw = _att.yaw;
	_pos.eph = 1;
	_pos.epv = 1;
	_pos.terrain_alt = 0;
	_pos.terrain_alt_valid = true;
	_pos.update();
}

