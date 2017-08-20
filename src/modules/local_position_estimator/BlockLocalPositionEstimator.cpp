#include "BlockLocalPositionEstimator.hpp"
#include <drivers/drv_hrt.h>
#include <systemlib/mavlink_log.h>
#include <fcntl.h>
#include <systemlib/err.h>
#include <matrix/math.hpp>
#include <cstdlib>

orb_advert_t mavlink_log_pub = nullptr;

// required standard deviation of estimate for estimator to publish data
static const uint32_t 		EST_STDDEV_XY_VALID = 2.0; // 2.0 m
static const uint32_t 		EST_STDDEV_Z_VALID = 2.0; // 2.0 m
static const uint32_t 		EST_STDDEV_TZ_VALID = 2.0; // 2.0 m

static const float P_MAX = 1.0e6f; // max allowed value in state covariance
static const float LAND_RATE = 10.0f; // rate of land detector correction

static const char *msg_label = "[lpe] ";  // rate of land detector correction

BlockLocalPositionEstimator::BlockLocalPositionEstimator() :
	// this block has no parent, and has name LPE
	SuperBlock(nullptr, "LPE"),
	// subscriptions, set rate, add to list
	_sub_armed(ORB_ID(actuator_armed), 1000 / 2, 0, &getSubscriptions()),
	_sub_land(ORB_ID(vehicle_land_detected), 1000 / 2, 0, &getSubscriptions()),
	_sub_att(ORB_ID(vehicle_attitude), 1000 / 100, 0, &getSubscriptions()),
	// set flow max update rate higher than expected to we don't lose packets
	_sub_flow(ORB_ID(optical_flow), 1000 / 100, 0, &getSubscriptions()),
	// main prediction loop, 100 hz
	_sub_sensor(ORB_ID(sensor_combined), 1000 / 100, 0, &getSubscriptions()),
	// status updates 2 hz
	_sub_param_update(ORB_ID(parameter_update), 1000 / 2, 0, &getSubscriptions()),
	_sub_manual(ORB_ID(manual_control_setpoint), 1000 / 2, 0, &getSubscriptions()),
	// gps 10 hz
	_sub_gps(ORB_ID(vehicle_gps_position), 1000 / 10, 0, &getSubscriptions()),
	// vision 50 hz
	_sub_vision_pos(ORB_ID(vehicle_vision_position), 1000 / 50, 0, &getSubscriptions()),
	// mocap 50 hz
	_sub_mocap(ORB_ID(att_pos_mocap), 1000 / 50, 0, &getSubscriptions()),
	// all distance sensors, 10 hz
	_sub_dist0(ORB_ID(distance_sensor), 1000 / 10, 0, &getSubscriptions()),
	_sub_dist1(ORB_ID(distance_sensor), 1000 / 10, 1, &getSubscriptions()),
	_sub_dist2(ORB_ID(distance_sensor), 1000 / 10, 2, &getSubscriptions()),
	_sub_dist3(ORB_ID(distance_sensor), 1000 / 10, 3, &getSubscriptions()),
	_dist_subs(),
	_sub_lidar(nullptr),
	_sub_sonar(nullptr),

	// publications
	_pub_lpos(ORB_ID(vehicle_local_position), -1, &getPublications()),
	_pub_gpos(ORB_ID(vehicle_global_position), -1, &getPublications()),
	_pub_est_status(ORB_ID(estimator_status), -1, &getPublications()),
	_pub_innov(ORB_ID(ekf2_innovations), -1, &getPublications()),

	// map projection
	_map_ref(),

	// block parameters
	_fusion(this, "FUSION"),
	_vxy_pub_thresh(this, "VXY_PUB"),
	_z_pub_thresh(this, "Z_PUB"),
	_sonar_z_stddev(this, "SNR_Z"),
	_sonar_z_offset(this, "SNR_OFF_Z"),
	_lidar_z_stddev(this, "LDR_Z"),
	_lidar_z_offset(this, "LDR_OFF_Z"),
	_accel_xy_stddev(this, "ACC_XY"),
	_accel_z_stddev(this, "ACC_Z"),
	_baro_stddev(this, "BAR_Z"),
	_gps_delay(this, "GPS_DELAY"),
	_gps_xy_stddev(this, "GPS_XY"),
	_gps_z_stddev(this, "GPS_Z"),
	_gps_vxy_stddev(this, "GPS_VXY"),
	_gps_vz_stddev(this, "GPS_VZ"),
	_gps_eph_max(this, "EPH_MAX"),
	_gps_epv_max(this, "EPV_MAX"),
	_vision_xy_stddev(this, "VIS_XY"),
	_vision_z_stddev(this, "VIS_Z"),
	_vision_delay(this, "VIS_DELAY"),
	_mocap_p_stddev(this, "VIC_P"),
	_flow_z_offset(this, "FLW_OFF_Z"),
	_flow_scale(this, "FLW_SCALE"),
	//_flow_board_x_offs(NULL, "SENS_FLW_XOFF"),
	//_flow_board_y_offs(NULL, "SENS_FLW_YOFF"),
	_flow_min_q(this, "FLW_QMIN"),
	_flow_r(this, "FLW_R"),
	_flow_rr(this, "FLW_RR"),
	_land_z_stddev(this, "LAND_Z"),
	_land_vxy_stddev(this, "LAND_VXY"),
	_pn_p_noise_density(this, "PN_P"),
	_pn_v_noise_density(this, "PN_V"),
	_pn_b_noise_density(this, "PN_B"),
	_pn_t_noise_density(this, "PN_T"),
	_t_max_grade(this, "T_MAX_GRADE"),

	// init origin
	_fake_origin(this, "FAKE_ORIGIN"),
	_init_origin_lat(this, "LAT"),
	_init_origin_lon(this, "LON"),

	// flow gyro
	_flow_gyro_x_high_pass(this, "FGYRO_HP"),
	_flow_gyro_y_high_pass(this, "FGYRO_HP"),

	// stats
	_baroStats(this, ""),
	_sonarStats(this, ""),
	_lidarStats(this, ""),
	_flowQStats(this, ""),
	_visionStats(this, ""),
	_mocapStats(this, ""),
	_gpsStats(this, ""),

	// low pass
	_xLowPass(this, "X_LP"),
	// use same lp constant for agl
	_aglLowPass(this, "X_LP"),

	// delay
	_xDelay(this, ""),
	_tDelay(this, ""),

	// misc
	_polls(),
	_timeStamp(hrt_absolute_time()),
	_time_origin(0),
	_timeStampLastBaro(hrt_absolute_time()),
	_time_last_hist(0),
	_time_last_flow(0),
	_time_last_baro(0),
	_time_last_gps(0),
	_time_last_lidar(0),
	_time_last_sonar(0),
	_time_init_sonar(0),
	_time_last_vision_p(0),
	_time_last_mocap(0),
	_time_last_land(0),

	// reference altitudes
	_altOrigin(0),
	_altOriginInitialized(false),
	_baroAltOrigin(0),
	_gpsAltOrigin(0),

	// status
	_receivedGps(false),
	_lastArmedState(false),

	// masks
	_sensorTimeout(255),
	_sensorFault(0),
	_estimatorInitialized(0)
{
	// assign distance subs to array
	_dist_subs[0] = &_sub_dist0;
	_dist_subs[1] = &_sub_dist1;
	_dist_subs[2] = &_sub_dist2;
	_dist_subs[3] = &_sub_dist3;

	// setup event triggering based on new flow messages to integrate
	_polls[POLL_FLOW].fd = _sub_flow.getHandle();
	_polls[POLL_FLOW].events = POLLIN;

	_polls[POLL_PARAM].fd = _sub_param_update.getHandle();
	_polls[POLL_PARAM].events = POLLIN;

	_polls[POLL_SENSORS].fd = _sub_sensor.getHandle();
	_polls[POLL_SENSORS].events = POLLIN;

	// initialize A, B,  P, x, u
	_x.setZero();
	_u.setZero();
	initSS();

	// map
	_map_ref.init_done = false;

	// intialize parameter dependent matrices
	updateParams();

	// print fusion settings to console
	printf("[lpe] fuse gps: %d, flow: %d, vis_pos: %d, "
	       "vis_yaw: %d, land: %d, pub_agl_z: %d, flow_gyro: %d\n",
	       (_fusion.get() & FUSE_GPS) != 0,
	       (_fusion.get() & FUSE_FLOW) != 0,
	       (_fusion.get() & FUSE_VIS_POS) != 0,
	       (_fusion.get() & FUSE_VIS_YAW) != 0,
	       (_fusion.get() & FUSE_LAND) != 0,
	       (_fusion.get() & FUSE_PUB_AGL_Z) != 0,
	       (_fusion.get() & FUSE_FLOW_GYRO_COMP) != 0);
}

BlockLocalPositionEstimator::~BlockLocalPositionEstimator()
{
}

Vector<float, BlockLocalPositionEstimator::n_x> BlockLocalPositionEstimator::dynamics(
	float t,
	const Vector<float, BlockLocalPositionEstimator::n_x> &x,
	const Vector<float, BlockLocalPositionEstimator::n_u> &u)
{
	return _A * x + _B * u;
}

void BlockLocalPositionEstimator::update()
{

	// wait for a sensor update, check for exit condition every 100 ms
	int ret = px4_poll(_polls, 3, 100);

	if (ret < 0) {
		return;
	}

	uint64_t newTimeStamp = hrt_absolute_time();
	float dt = (newTimeStamp - _timeStamp) / 1.0e6f;
	_timeStamp = newTimeStamp;

	// set dt for all child blocks
	setDt(dt);

	// auto-detect connected rangefinders while not armed
	bool armedState = _sub_armed.get().armed;

	if (!armedState && (_sub_lidar == nullptr || _sub_sonar == nullptr)) {

		// detect distance sensors
		for (int i = 0; i < N_DIST_SUBS; i++) {
			uORB::Subscription<distance_sensor_s> *s = _dist_subs[i];

			if (s == _sub_lidar || s == _sub_sonar) { continue; }

			if (s->updated()) {
				s->update();

				if (s->get().timestamp == 0) { continue; }

				if (s->get().type == \
				    distance_sensor_s::MAV_DISTANCE_SENSOR_LASER &&
				    _sub_lidar == nullptr) {
					_sub_lidar = s;
					mavlink_and_console_log_info(&mavlink_log_pub, "%sLidar detected with ID %i", msg_label, i);

				} else if (s->get().type == \
					   distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND &&
					   _sub_sonar == nullptr) {
					_sub_sonar = s;
					mavlink_and_console_log_info(&mavlink_log_pub, "%sSonar detected with ID %i", msg_label, i);
				}
			}
		}
	}

	// reset pos, vel, and terrain on arming

	// XXX this will be re-enabled for indoor use cases using a
	// selection param, but is really not helping outdoors
	// right now.

	// if (!_lastArmedState && armedState) {

	// 	// we just armed, we are at origin on the ground
	// 	_x(X_x) = 0;
	// 	_x(X_y) = 0;
	// 	// reset Z or not? _x(X_z) = 0;

	// 	// we aren't moving, all velocities are zero
	// 	_x(X_vx) = 0;
	// 	_x(X_vy) = 0;
	// 	_x(X_vz) = 0;

	// 	// assume we are on the ground, so terrain alt is local alt
	// 	_x(X_tz) = _x(X_z);

	// 	// reset lowpass filter as well
	// 	_xLowPass.setState(_x);
	// 	_aglLowPass.setState(0);
	// }

	_lastArmedState = armedState;

	// see which updates are available
	bool paramsUpdated = _sub_param_update.updated();
	bool baroUpdated = false;

	if ((_fusion.get() & FUSE_BARO) && _sub_sensor.updated()) {
		int32_t baro_timestamp_relative = _sub_sensor.get().baro_timestamp_relative;

		if (baro_timestamp_relative != _sub_sensor.get().RELATIVE_TIMESTAMP_INVALID) {
			uint64_t baro_timestamp = _sub_sensor.get().timestamp + \
						  _sub_sensor.get().baro_timestamp_relative;

			if (baro_timestamp != _timeStampLastBaro) {
				baroUpdated = true;
				_timeStampLastBaro = baro_timestamp;
			}
		}
	}

	bool flowUpdated = (_fusion.get() & FUSE_FLOW) && _sub_flow.updated();
	bool gpsUpdated = (_fusion.get() & FUSE_GPS) && _sub_gps.updated();
	bool visionUpdated = (_fusion.get() & FUSE_VIS_POS) && _sub_vision_pos.updated();
	bool mocapUpdated = _sub_mocap.updated();
	bool lidarUpdated = (_sub_lidar != nullptr) && _sub_lidar->updated();
	bool sonarUpdated = (_sub_sonar != nullptr) && _sub_sonar->updated();
	bool landUpdated = landed()
			   && ((_timeStamp - _time_last_land) > 1.0e6f / LAND_RATE); // throttle rate

	// get new data
	updateSubscriptions();

	// update parameters
	if (paramsUpdated) {
		updateParams();
		updateSSParams();
	}

	// is xy valid?
	bool vxy_stddev_ok = false;

	if (math::max(_P(X_vx, X_vx), _P(X_vy, X_vy)) < _vxy_pub_thresh.get()*_vxy_pub_thresh.get()) {
		vxy_stddev_ok = true;
	}

	if (_estimatorInitialized & EST_XY) {
		// if valid and gps has timed out, set to not valid
		if (!vxy_stddev_ok && (_sensorTimeout & SENSOR_GPS)) {
			_estimatorInitialized &= ~EST_XY;
		}

	} else {
		if (vxy_stddev_ok) {
			if (!(_sensorTimeout & SENSOR_GPS)
			    || !(_sensorTimeout & SENSOR_FLOW)
			    || !(_sensorTimeout & SENSOR_VISION)
			    || !(_sensorTimeout & SENSOR_MOCAP)
			    || !(_sensorTimeout & SENSOR_LAND)
			   ) {
				_estimatorInitialized |= EST_XY;
			}
		}
	}

	// is z valid?
	bool z_stddev_ok = sqrtf(_P(X_z, X_z)) < _z_pub_thresh.get();

	if (_estimatorInitialized & EST_Z) {
		// if valid and baro has timed out, set to not valid
		if (!z_stddev_ok && (_sensorTimeout & SENSOR_BARO)) {
			_estimatorInitialized &= ~EST_Z;
		}

	} else {
		if (z_stddev_ok) {
			_estimatorInitialized |= EST_Z;
		}
	}

	// is terrain valid?
	bool tz_stddev_ok = sqrtf(_P(X_tz, X_tz)) < _z_pub_thresh.get();

	if (_estimatorInitialized & EST_TZ) {
		if (!tz_stddev_ok) {
			_estimatorInitialized &= ~EST_TZ;
		}

	} else {
		if (tz_stddev_ok) {
			_estimatorInitialized |= EST_TZ;
		}
	}

	// check timeouts
	checkTimeouts();

	// if we have no lat, lon initialize projection to LPE_LAT, LPE_LON parameters
	if (!_map_ref.init_done && (_estimatorInitialized & EST_XY) && _fake_origin.get()) {
		map_projection_init(&_map_ref,
				    _init_origin_lat.get(),
				    _init_origin_lon.get());

		// set timestamp when origin was set to current time
		_time_origin = _timeStamp;

		mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] global origin init (parameter) : lat %6.2f lon %6.2f alt %5.1f m",
					     double(_init_origin_lat.get()), double(_init_origin_lon.get()), double(_altOrigin));

	}

	// reinitialize x if necessary
	bool reinit_x = false;

	for (int i = 0; i < n_x; i++) {
		// should we do a reinit
		// of sensors here?
		// don't want it to take too long
		if (!PX4_ISFINITE(_x(i))) {
			reinit_x = true;
			mavlink_and_console_log_info(&mavlink_log_pub, "%sreinit x, x(%d) not finite", msg_label, i);
			break;
		}
	}

	if (reinit_x) {
		for (int i = 0; i < n_x; i++) {
			_x(i) = 0;
		}

		mavlink_and_console_log_info(&mavlink_log_pub, "%sreinit x", msg_label);
	}

	// force P symmetry and reinitialize P if necessary
	bool reinit_P = false;

	for (int i = 0; i < n_x; i++) {
		for (int j = 0; j <= i; j++) {
			if (!PX4_ISFINITE(_P(i, j))) {
				mavlink_and_console_log_info(&mavlink_log_pub,
							     "%sreinit P (%d, %d) not finite", msg_label, i, j);
				reinit_P = true;
			}

			if (i == j) {
				// make sure diagonal elements are positive
				if (_P(i, i) <= 0) {
					mavlink_and_console_log_info(&mavlink_log_pub,
								     "%sreinit P (%d, %d) negative", msg_label, i, j);
					reinit_P = true;
				}

			} else {
				// copy elememnt from upper triangle to force
				// symmetry
				_P(j, i) = _P(i, j);
			}

			if (reinit_P) { break; }
		}

		if (reinit_P) { break; }
	}

	if (reinit_P) {
		initP();
	}

	// do prediction
	predict();

	// sensor corrections/ initializations
	if (gpsUpdated) {
		if (_sensorTimeout & SENSOR_GPS) {
			gpsInit();

		} else {
			gpsCorrect();
		}
	}

	if (baroUpdated) {
		if (_sensorTimeout & SENSOR_BARO) {
			baroInit();

		} else {
			baroCorrect();
		}
	}

	if (lidarUpdated) {
		if (_sensorTimeout & SENSOR_LIDAR) {
			lidarInit();

		} else {
			lidarCorrect();
		}
	}

	if (sonarUpdated) {
		if (_sensorTimeout & SENSOR_SONAR) {
			sonarInit();

		} else {
			sonarCorrect();
		}
	}

	if (flowUpdated) {
		if (_sensorTimeout & SENSOR_FLOW) {
			flowInit();

		} else {
			flowCorrect();
		}
	}

	if (visionUpdated) {
		if (_sensorTimeout & SENSOR_VISION) {
			visionInit();

		} else {
			visionCorrect();
		}
	}

	if (mocapUpdated) {
		if (_sensorTimeout & SENSOR_MOCAP) {
			mocapInit();

		} else {
			mocapCorrect();
		}
	}

	if (landUpdated) {
		if (_sensorTimeout & SENSOR_LAND) {
			landInit();

		} else {
			landCorrect();
		}
	}

	if (_altOriginInitialized) {
		// update all publications if possible
		publishLocalPos();
		publishEstimatorStatus();
		_pub_innov.update();

		if ((_estimatorInitialized & EST_XY) && (_map_ref.init_done || _fake_origin.get())) {
			publishGlobalPos();
		}
	}

	// propagate delayed state, no matter what
	// if state is frozen, delayed state still
	// needs to be propagated with frozen state
	float dt_hist = 1.0e-6f * (_timeStamp - _time_last_hist);

	if (_time_last_hist == 0 ||
	    (dt_hist > HIST_STEP)) {
		_tDelay.update(Scalar<uint64_t>(_timeStamp));
		_xDelay.update(_x);
		_time_last_hist = _timeStamp;
	}
}

void BlockLocalPositionEstimator::checkTimeouts()
{
	baroCheckTimeout();
	gpsCheckTimeout();
	lidarCheckTimeout();
	flowCheckTimeout();
	sonarCheckTimeout();
	visionCheckTimeout();
	mocapCheckTimeout();
	landCheckTimeout();
}

bool BlockLocalPositionEstimator::landed()
{
	if (!(_fusion.get() & FUSE_LAND)) {
		return false;
	}

	bool disarmed_not_falling = (!_sub_armed.get().armed) && (!_sub_land.get().freefall);

	return _sub_land.get().landed || disarmed_not_falling;
}

void BlockLocalPositionEstimator::publishLocalPos()
{
	const Vector<float, n_x> &xLP = _xLowPass.getState();

	// lie about eph/epv to allow visual odometry only navigation when velocity est. good
	float vxy_stddev = sqrtf(_P(X_vx, X_vx) + _P(X_vy, X_vy));
	float epv = sqrtf(_P(X_z, X_z));
	float eph = sqrtf(_P(X_x, X_x) + _P(X_y, X_y));
	float eph_thresh = 3.0f;
	float epv_thresh = 3.0f;

	if (vxy_stddev < _vxy_pub_thresh.get()) {
		if (eph > eph_thresh) {
			eph = eph_thresh;
		}

		if (epv > epv_thresh) {
			epv = epv_thresh;
		}
	}

	// publish local position
	if (PX4_ISFINITE(_x(X_x)) && PX4_ISFINITE(_x(X_y)) && PX4_ISFINITE(_x(X_z)) &&
	    PX4_ISFINITE(_x(X_vx)) && PX4_ISFINITE(_x(X_vy))
	    && PX4_ISFINITE(_x(X_vz))) {
		_pub_lpos.get().timestamp = _timeStamp;
		_pub_lpos.get().xy_valid = _estimatorInitialized & EST_XY;
		_pub_lpos.get().z_valid = _estimatorInitialized & EST_Z;
		_pub_lpos.get().v_xy_valid = _estimatorInitialized & EST_XY;
		_pub_lpos.get().v_z_valid = _estimatorInitialized & EST_Z;
		_pub_lpos.get().x = xLP(X_x); 	// north
		_pub_lpos.get().y = xLP(X_y);  	// east

		if (_fusion.get() & FUSE_PUB_AGL_Z) {
			_pub_lpos.get().z = -_aglLowPass.getState(); // agl

		} else {
			_pub_lpos.get().z = xLP(X_z); 	// down
		}

		_pub_lpos.get().vx = xLP(X_vx); // north
		_pub_lpos.get().vy = xLP(X_vy); // east
		_pub_lpos.get().vz = xLP(X_vz); // down

		// this estimator does not provide a separate vertical position time derivative estimate, so use the vertical velocity
		_pub_lpos.get().z_deriv = xLP(X_vz);

		_pub_lpos.get().yaw = _eul(2);
		_pub_lpos.get().xy_global = _estimatorInitialized & EST_XY;
		_pub_lpos.get().z_global = !(_sensorTimeout & SENSOR_BARO);
		_pub_lpos.get().ref_timestamp = _time_origin;
		_pub_lpos.get().ref_lat = _map_ref.lat_rad * 180 / M_PI;
		_pub_lpos.get().ref_lon = _map_ref.lon_rad * 180 / M_PI;
		_pub_lpos.get().ref_alt = _altOrigin;
		_pub_lpos.get().dist_bottom = _aglLowPass.getState();
		_pub_lpos.get().dist_bottom_rate = - xLP(X_vz);
		// we estimate agl even when we don't have terrain info
		// if you are in terrain following mode this is important
		// so that if terrain estimation fails there isn't a
		// sudden altitude jump
		_pub_lpos.get().dist_bottom_valid = _estimatorInitialized & EST_Z;
		_pub_lpos.get().eph = eph;
		_pub_lpos.get().epv = epv;
		_pub_lpos.update();
		//TODO provide calculated values for these
		_pub_lpos.get().evh = 0.0f;
		_pub_lpos.get().evv = 0.0f;
	}
}

void BlockLocalPositionEstimator::publishEstimatorStatus()
{
	_pub_est_status.get().timestamp = _timeStamp;

	for (int i = 0; i < n_x; i++) {
		_pub_est_status.get().states[i] = _x(i);
		_pub_est_status.get().covariances[i] = _P(i, i);
	}

	_pub_est_status.get().n_states = n_x;
	_pub_est_status.get().nan_flags = 0;
	_pub_est_status.get().health_flags = _sensorFault;
	_pub_est_status.get().timeout_flags = _sensorTimeout;
	_pub_est_status.get().pos_horiz_accuracy = _pub_gpos.get().eph;
	_pub_est_status.get().pos_vert_accuracy = _pub_gpos.get().epv;

	_pub_est_status.update();
}

void BlockLocalPositionEstimator::publishGlobalPos()
{
	// publish global position
	double lat = 0;
	double lon = 0;
	const Vector<float, n_x> &xLP = _xLowPass.getState();
	map_projection_reproject(&_map_ref, xLP(X_x), xLP(X_y), &lat, &lon);
	float alt = -xLP(X_z) + _altOrigin;

	// lie about eph/epv to allow visual odometry only navigation when velocity est. good
	float vxy_stddev = sqrtf(_P(X_vx, X_vx) + _P(X_vy, X_vy));
	float epv = sqrtf(_P(X_z, X_z));
	float eph = sqrtf(_P(X_x, X_x) + _P(X_y, X_y));
	float eph_thresh = 3.0f;
	float epv_thresh = 3.0f;

	if (vxy_stddev < _vxy_pub_thresh.get()) {
		if (eph > eph_thresh) {
			eph = eph_thresh;
		}

		if (epv > epv_thresh) {
			epv = epv_thresh;
		}
	}

	if (PX4_ISFINITE(lat) && PX4_ISFINITE(lon) && PX4_ISFINITE(alt) &&
	    PX4_ISFINITE(xLP(X_vx)) && PX4_ISFINITE(xLP(X_vy)) &&
	    PX4_ISFINITE(xLP(X_vz))) {
		_pub_gpos.get().timestamp = _timeStamp;
		_pub_gpos.get().lat = lat;
		_pub_gpos.get().lon = lon;
		_pub_gpos.get().alt = alt;
		_pub_gpos.get().vel_n = xLP(X_vx);
		_pub_gpos.get().vel_e = xLP(X_vy);
		_pub_gpos.get().vel_d = xLP(X_vz);

		// this estimator does not provide a separate vertical position time derivative estimate, so use the vertical velocity
		_pub_gpos.get().pos_d_deriv = xLP(X_vz);

		_pub_gpos.get().yaw = _eul(2);
		_pub_gpos.get().eph = eph;
		_pub_gpos.get().epv = epv;
		_pub_gpos.get().pressure_alt = _sub_sensor.get().baro_alt_meter;
		_pub_gpos.get().terrain_alt = _altOrigin - xLP(X_tz);
		_pub_gpos.get().terrain_alt_valid = _estimatorInitialized & EST_TZ;
		_pub_gpos.get().dead_reckoning = !(_estimatorInitialized & EST_XY);
		_pub_gpos.update();
		// TODO provide calculated values for these
		_pub_gpos.get().evh = 0.0f;
		_pub_gpos.get().evv = 0.0f;
	}
}

void BlockLocalPositionEstimator::initP()
{
	_P.setZero();
	// initialize to twice valid condition
	_P(X_x, X_x) = 2 * EST_STDDEV_XY_VALID * EST_STDDEV_XY_VALID;
	_P(X_y, X_y) = 2 * EST_STDDEV_XY_VALID * EST_STDDEV_XY_VALID;
	_P(X_z, X_z) = 2 * EST_STDDEV_Z_VALID * EST_STDDEV_Z_VALID;
	_P(X_vx, X_vx) = 2 * _vxy_pub_thresh.get() * _vxy_pub_thresh.get();
	_P(X_vy, X_vy) = 2 * _vxy_pub_thresh.get() * _vxy_pub_thresh.get();
	// use vxy thresh for vz init as well
	_P(X_vz, X_vz) = 2 * _vxy_pub_thresh.get() * _vxy_pub_thresh.get();
	// initialize bias uncertainty to small values to keep them stable
	_P(X_bx, X_bx) = 1e-6;
	_P(X_by, X_by) = 1e-6;
	_P(X_bz, X_bz) = 1e-6;
	_P(X_tz, X_tz) = 2 * EST_STDDEV_TZ_VALID * EST_STDDEV_TZ_VALID;
}

void BlockLocalPositionEstimator::initSS()
{
	initP();

	// dynamics matrix
	_A.setZero();
	// derivative of position is velocity
	_A(X_x, X_vx) = 1;
	_A(X_y, X_vy) = 1;
	_A(X_z, X_vz) = 1;

	// input matrix
	_B.setZero();
	_B(X_vx, U_ax) = 1;
	_B(X_vy, U_ay) = 1;
	_B(X_vz, U_az) = 1;

	// update components that depend on current state
	updateSSStates();
	updateSSParams();
}

void BlockLocalPositionEstimator::updateSSStates()
{
	// derivative of velocity is accelerometer acceleration
	// (in input matrix) - bias (in body frame)
	_A(X_vx, X_bx) = -_R_att(0, 0);
	_A(X_vx, X_by) = -_R_att(0, 1);
	_A(X_vx, X_bz) = -_R_att(0, 2);

	_A(X_vy, X_bx) = -_R_att(1, 0);
	_A(X_vy, X_by) = -_R_att(1, 1);
	_A(X_vy, X_bz) = -_R_att(1, 2);

	_A(X_vz, X_bx) = -_R_att(2, 0);
	_A(X_vz, X_by) = -_R_att(2, 1);
	_A(X_vz, X_bz) = -_R_att(2, 2);
}

void BlockLocalPositionEstimator::updateSSParams()
{
	// input noise covariance matrix
	_R.setZero();
	_R(U_ax, U_ax) = _accel_xy_stddev.get() * _accel_xy_stddev.get();
	_R(U_ay, U_ay) = _accel_xy_stddev.get() * _accel_xy_stddev.get();
	_R(U_az, U_az) = _accel_z_stddev.get() * _accel_z_stddev.get();

	// process noise power matrix
	_Q.setZero();
	float pn_p_sq = _pn_p_noise_density.get() * _pn_p_noise_density.get();
	float pn_v_sq = _pn_v_noise_density.get() * _pn_v_noise_density.get();
	_Q(X_x, X_x) = pn_p_sq;
	_Q(X_y, X_y) = pn_p_sq;
	_Q(X_z, X_z) = pn_p_sq;
	_Q(X_vx, X_vx) = pn_v_sq;
	_Q(X_vy, X_vy) = pn_v_sq;
	_Q(X_vz, X_vz) = pn_v_sq;

	// technically, the noise is in the body frame,
	// but the components are all the same, so
	// ignoring for now
	float pn_b_sq = _pn_b_noise_density.get() * _pn_b_noise_density.get();
	_Q(X_bx, X_bx) = pn_b_sq;
	_Q(X_by, X_by) = pn_b_sq;
	_Q(X_bz, X_bz) = pn_b_sq;

	// terrain random walk noise ((m/s)/sqrt(hz)), scales with velocity
	float pn_t_noise_density =
		_pn_t_noise_density.get() +
		(_t_max_grade.get() / 100.0f) * sqrtf(_x(X_vx) * _x(X_vx) + _x(X_vy) * _x(X_vy));
	_Q(X_tz, X_tz) = pn_t_noise_density * pn_t_noise_density;

}

void BlockLocalPositionEstimator::predict()
{
	// get acceleration
	matrix::Quaternion<float> q(&_sub_att.get().q[0]);
	_eul = matrix::Euler<float>(q);
	_R_att = matrix::Dcm<float>(q);
	Vector3f a(_sub_sensor.get().accelerometer_m_s2);
	// note, bias is removed in dynamics function
	_u = _R_att * a;
	_u(U_az) += 9.81f; // add g

	// update state space based on new states
	updateSSStates();

	// continuous time kalman filter prediction
	// integrate runge kutta 4th order
	// TODO move rk4 algorithm to matrixlib
	// https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
	float h = getDt();
	Vector<float, n_x> k1, k2, k3, k4;
	k1 = dynamics(0, _x, _u);
	k2 = dynamics(h / 2, _x + k1 * h / 2, _u);
	k3 = dynamics(h / 2, _x + k2 * h / 2, _u);
	k4 = dynamics(h, _x + k3 * h, _u);
	Vector<float, n_x> dx = (k1 + k2 * 2 + k3 * 2 + k4) * (h / 6);

	// don't integrate position if no valid xy data
	if (!(_estimatorInitialized & EST_XY))  {
		dx(X_x) = 0;
		dx(X_vx) = 0;
		dx(X_y) = 0;
		dx(X_vy) = 0;
	}

	// don't integrate z if no valid z data
	if (!(_estimatorInitialized & EST_Z))  {
		dx(X_z) = 0;
	}

	// don't integrate tz if no valid tz data
	if (!(_estimatorInitialized & EST_TZ))  {
		dx(X_tz) = 0;
	}

	// saturate bias
	float bx = dx(X_bx) + _x(X_bx);
	float by = dx(X_by) + _x(X_by);
	float bz = dx(X_bz) + _x(X_bz);

	if (std::abs(bx) > BIAS_MAX) {
		bx = BIAS_MAX * bx / std::abs(bx);
		dx(X_bx) = bx - _x(X_bx);
	}

	if (std::abs(by) > BIAS_MAX) {
		by = BIAS_MAX * by / std::abs(by);
		dx(X_by) = by - _x(X_by);
	}

	if (std::abs(bz) > BIAS_MAX) {
		bz = BIAS_MAX * bz / std::abs(bz);
		dx(X_bz) = bz - _x(X_bz);
	}

	// propagate
	_x += dx;
	Matrix<float, n_x, n_x> dP = (_A * _P + _P * _A.transpose() +
				      _B * _R * _B.transpose() + _Q) * getDt();

	// covariance propagation logic
	for (int i = 0; i < n_x; i++) {
		if (_P(i, i) > P_MAX) {
			// if diagonal element greater than max, stop propagating
			dP(i, i) = 0;

			for (int j = 0; j < n_x; j++) {
				dP(i, j) = 0;
				dP(j, i) = 0;
			}
		}
	}

	_P += dP;
	_xLowPass.update(_x);
	_aglLowPass.update(agl());
}

int BlockLocalPositionEstimator::getDelayPeriods(float delay, uint8_t *periods)
{
	float t_delay = 0;
	uint8_t i_hist = 0;

	for (i_hist = 1; i_hist < HIST_LEN; i_hist++) {
		t_delay = 1.0e-6f * (_timeStamp - _tDelay.get(i_hist)(0, 0));

		if (t_delay > delay) {
			break;
		}
	}

	*periods = i_hist;

	if (t_delay > DELAY_MAX) {
		mavlink_and_console_log_info(&mavlink_log_pub, "%sdelayed data old: %8.4f", msg_label, double(t_delay));
		return -1;
	}

	return OK;
}
