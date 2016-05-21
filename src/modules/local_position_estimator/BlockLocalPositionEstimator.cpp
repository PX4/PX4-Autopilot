#include "BlockLocalPositionEstimator.hpp"
#include <systemlib/mavlink_log.h>
#include <fcntl.h>
#include <systemlib/err.h>
#include <matrix/math.hpp>

orb_advert_t mavlink_log_pub = nullptr;

// timeouts for sensors in microseconds
static const uint32_t 		EST_SRC_TIMEOUT = 10000; // 0.01 s

// minimum flow altitude
static const float flow_min_agl = 0.3;

BlockLocalPositionEstimator::BlockLocalPositionEstimator() :
	// this block has no parent, and has name LPE
	SuperBlock(NULL, "LPE"),

	// subscriptions, set rate, add to list
	// TODO topic speed limiting?
	_sub_status(ORB_ID(vehicle_status), 0, 0, &getSubscriptions()),
	_sub_armed(ORB_ID(actuator_armed), 0, 0, &getSubscriptions()),
	_sub_control_mode(ORB_ID(vehicle_control_mode),
			  0, 0, &getSubscriptions()),
	_sub_att(ORB_ID(vehicle_attitude), 0, 0, &getSubscriptions()),
	_sub_att_sp(ORB_ID(vehicle_attitude_setpoint),
		    0, 0, &getSubscriptions()),
	_sub_flow(ORB_ID(optical_flow), 0, 0, &getSubscriptions()),
	_sub_sensor(ORB_ID(sensor_combined), 0, 0, &getSubscriptions()),
	_sub_param_update(ORB_ID(parameter_update), 0, 0, &getSubscriptions()),
	_sub_manual(ORB_ID(manual_control_setpoint), 0, 0, &getSubscriptions()),
	_sub_home(ORB_ID(home_position), 0, 0, &getSubscriptions()),
	_sub_gps(ORB_ID(vehicle_gps_position), 0, 0, &getSubscriptions()),
	_sub_vision_pos(ORB_ID(vision_position_estimate), 0, 0, &getSubscriptions()),
	_sub_mocap(ORB_ID(att_pos_mocap), 0, 0, &getSubscriptions()),
	_sub_dist0(ORB_ID(distance_sensor), 0, 0, &getSubscriptions()),
	_sub_dist1(ORB_ID(distance_sensor), 0, 1, &getSubscriptions()),
	_sub_dist2(ORB_ID(distance_sensor), 0, 2, &getSubscriptions()),
	_sub_dist3(ORB_ID(distance_sensor), 0, 3, &getSubscriptions()),
	_dist_subs(),
	_sub_lidar(NULL),
	_sub_sonar(NULL),

	// publications
	_pub_lpos(ORB_ID(vehicle_local_position), -1, &getPublications()),
	_pub_gpos(ORB_ID(vehicle_global_position), -1, &getPublications()),
	_pub_est_status(ORB_ID(estimator_status), -1, &getPublications()),

	// map projection
	_map_ref(),

	// block parameters
	_integrate(this, "INTEGRATE"),
	_sonar_z_stddev(this, "SNR_Z"),
	_sonar_z_offset(this, "SNR_OFF_Z"),
	_lidar_z_stddev(this, "LDR_Z"),
	_lidar_z_offset(this, "LDR_OFF_Z"),
	_accel_xy_stddev(this, "ACC_XY"),
	_accel_z_stddev(this, "ACC_Z"),
	_baro_stddev(this, "BAR_Z"),
	_gps_on(this, "GPS_ON"),
	_gps_delay(this, "GPS_DELAY"),
	_gps_xy_stddev(this, "GPS_XY"),
	_gps_z_stddev(this, "GPS_Z"),
	_gps_vxy_stddev(this, "GPS_VXY"),
	_gps_vz_stddev(this, "GPS_VZ"),
	_gps_eph_max(this, "EPH_MAX"),
	_vision_xy_stddev(this, "VIS_XY"),
	_vision_z_stddev(this, "VIS_Z"),
	_vision_on(this, "VIS_ON"),
	_mocap_p_stddev(this, "VIC_P"),
	_flow_z_offset(this, "FLW_OFF_Z"),
	_flow_xy_stddev(this, "FLW_XY"),
	//_flow_board_x_offs(NULL, "SENS_FLW_XOFF"),
	//_flow_board_y_offs(NULL, "SENS_FLW_YOFF"),
	_flow_min_q(this, "FLW_QMIN"),
	_pn_p_noise_density(this, "PN_P"),
	_pn_v_noise_density(this, "PN_V"),
	_pn_b_noise_density(this, "PN_B"),
	_pn_t_noise_density(this, "PN_T"),

	// init home
	_init_home_lat(this, "LAT"),
	_init_home_lon(this, "LON"),

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

	// stats
	_xDelay(this, ""),
	_tDelay(this, ""),

	// misc
	_polls(),
	_timeStamp(hrt_absolute_time()),
	_time_last_hist(0),
	_time_last_xy(0),
	_time_last_z(0),
	_time_last_tz(0),
	_time_last_flow(0),
	_time_last_baro(0),
	_time_last_gps(0),
	_time_last_lidar(0),
	_time_last_sonar(0),
	_time_last_vision_p(0),
	_time_last_mocap(0),

	// initialization flags
	_receivedGps(false),
	_baroInitialized(false),
	_gpsInitialized(false),
	_lidarInitialized(false),
	_sonarInitialized(false),
	_flowInitialized(false),
	_visionInitialized(false),
	_mocapInitialized(false),

	// reference altitudes
	_altHome(0),
	_altHomeInitialized(false),
	_baroAltHome(0),
	_gpsAltHome(0),
	_visionHome(),
	_mocapHome(),

	// flow integration
	_flowX(0),
	_flowY(0),
	_flowMeanQual(0),

	// status
	_canEstimateXY(false),
	_canEstimateZ(false),
	_canEstimateT(false),
	_xyTimeout(true),
	_zTimeout(true),
	_tzTimeout(true),
	_lastArmedState(false),

	// faults
	_baroFault(FAULT_NONE),
	_gpsFault(FAULT_NONE),
	_lidarFault(FAULT_NONE),
	_flowFault(FAULT_NONE),
	_sonarFault(FAULT_NONE),
	_visionFault(FAULT_NONE),
	_mocapFault(FAULT_NONE),

	// loop performance
	_loop_perf(),
	_interval_perf(),
	_err_perf(),

	// kf matrices
	_x(), _u(), _P()
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

	// initialize P, x, u
	initP();
	_x.setZero();
	_u.setZero();
	_flowX = 0;
	_flowY = 0;

	// perf counters
	_loop_perf = perf_alloc(PC_ELAPSED,
				"local_position_estimator_runtime");
	//_interval_perf = perf_alloc(PC_INTERVAL,
	//"local_position_estimator_interval");
	_err_perf = perf_alloc(PC_COUNT, "local_position_estimator_err");

	// map
	_map_ref.init_done = false;

	// intialize parameter dependent matrices
	updateParams();
}

BlockLocalPositionEstimator::~BlockLocalPositionEstimator()
{
}

void BlockLocalPositionEstimator::update()
{

	// wait for a sensor update, check for exit condition every 100 ms
	int ret = px4_poll(_polls, 3, 100);

	if (ret < 0) {
		/* poll error, count it in perf */
		perf_count(_err_perf);
		return;
	}

	uint64_t newTimeStamp = hrt_absolute_time();
	float dt = (newTimeStamp - _timeStamp) / 1.0e6f;
	_timeStamp = newTimeStamp;

	// set dt for all child blocks
	setDt(dt);

	// auto-detect connected rangefinders while not armed
	bool armedState = _sub_armed.get().armed;

	if (!armedState && (_sub_lidar == NULL || _sub_sonar == NULL)) {
		detectDistanceSensors();
	}

	// reset pos, vel, and terrain on arming
	if (!_lastArmedState && armedState) {

		// we just armed, we are at home position on the ground
		_x(X_x) = 0;
		_x(X_y) = 0;

		// the pressure altitude of home may have drifted, so we don't
		// reset z to zero

		// reset flow integral
		_flowX = 0;
		_flowY = 0;

		// we aren't moving, all velocities are zero
		_x(X_vx) = 0;
		_x(X_vy) = 0;
		_x(X_vz) = 0;

		// assume we are on the ground, so terrain alt is local alt
		_x(X_tz) = _x(X_z);
	}

	_lastArmedState = armedState;

	// see which updates are available
	bool flowUpdated = _sub_flow.updated();
	bool paramsUpdated = _sub_param_update.updated();
	bool baroUpdated = _sub_sensor.updated();
	bool gpsUpdated = _gps_on.get() && _sub_gps.updated();
	bool homeUpdated = _sub_home.updated();
	bool visionUpdated = _vision_on.get() && _sub_vision_pos.updated();
	bool mocapUpdated = _sub_mocap.updated();
	bool lidarUpdated = (_sub_lidar != NULL) && _sub_lidar->updated();
	bool sonarUpdated = (_sub_sonar != NULL) && _sub_sonar->updated();

	// get new data
	updateSubscriptions();

	// update parameters
	if (paramsUpdated) {
		updateParams();
	}

	// update home position projection
	if (homeUpdated) {
		updateHome();
	}

	// determine if we should start estimating
	_canEstimateZ =
		(_baroInitialized && _baroFault < fault_lvl_disable);
	_canEstimateXY =
		(_gpsInitialized && _gpsFault < fault_lvl_disable) ||
		(_flowInitialized && _flowFault < fault_lvl_disable) ||
		(_visionInitialized && _visionFault < fault_lvl_disable) ||
		(_mocapInitialized && _mocapFault < fault_lvl_disable);
	_canEstimateT =
		(_lidarInitialized && _lidarFault < fault_lvl_disable) ||
		(_sonarInitialized && _sonarFault < fault_lvl_disable);

	if (_canEstimateXY) {
		_time_last_xy = _timeStamp;
	}

	if (_canEstimateZ) {
		_time_last_z = _timeStamp;
	}

	if (_canEstimateT) {
		_time_last_tz = _timeStamp;
	}

	// check timeouts
	checkTimeouts();

	// if we have no lat, lon initialize projection at 0,0
	if (_canEstimateXY && !_map_ref.init_done) {
		map_projection_init(&_map_ref,
				    _init_home_lat.get(),
				    _init_home_lon.get());
	}

	// reinitialize x if necessary
	bool reinit_x = false;

	for (int i = 0; i < n_x; i++) {
		// should we do a reinit
		// of sensors here?
		// don't want it to take too long
		if (!PX4_ISFINITE(_x(i))) {
			reinit_x = true;
			break;
		}
	}

	if (reinit_x) {
		for (int i = 0; i < n_x; i++) {
			_x(i) = 0;
		}

		mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] reinit x");
	}

	// reinitialize P if necessary
	bool reinit_P = false;

	for (int i = 0; i < n_x; i++) {
		for (int j = 0; j < n_x; j++) {
			if (!PX4_ISFINITE(_P(i, j))) {
				reinit_P = true;
				break;
			}
		}

		if (reinit_P) { break; }
	}

	if (reinit_P) {
		mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] reinit P");
		initP();
	}

	// do prediction
	predict();

	// sensor corrections/ initializations
	if (gpsUpdated) {
		if (!_gpsInitialized) {
			gpsInit();

		} else {
			gpsCorrect();
		}
	}

	if (baroUpdated) {
		if (!_baroInitialized) {
			baroInit();

		} else {
			baroCorrect();
		}
	}

	if (lidarUpdated) {
		if (!_lidarInitialized) {
			lidarInit();

		} else {
			lidarCorrect();
		}
	}

	if (sonarUpdated) {
		if (!_sonarInitialized) {
			sonarInit();

		} else {
			sonarCorrect();
		}
	}

	if (flowUpdated) {
		if (!_flowInitialized) {
			flowInit();

		} else {
			perf_begin(_loop_perf);// TODO
			flowCorrect();
			//perf_count(_interval_perf);
			perf_end(_loop_perf);
		}
	}

	if (visionUpdated) {
		if (!_visionInitialized) {
			visionInit();

		} else {
			visionCorrect();
		}
	}

	if (mocapUpdated) {
		if (!_mocapInitialized) {
			mocapInit();

		} else {
			mocapCorrect();
		}
	}

	if (_altHomeInitialized) {
		// update all publications if possible
		publishLocalPos();
		publishEstimatorStatus();

		if (_canEstimateXY) {
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
	if (_timeStamp - _time_last_xy > EST_SRC_TIMEOUT) {
		if (!_xyTimeout) {
			_xyTimeout = true;
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] xy timeout ");
		}

	} else if (_xyTimeout) {
		mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] xy resume ");
		_xyTimeout = false;
	}

	if (_timeStamp - _time_last_z > EST_SRC_TIMEOUT) {
		if (!_zTimeout) {
			_zTimeout = true;
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] z timeout ");
		}

	} else if (_zTimeout) {
		mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] z resume ");
		_zTimeout = false;
	}

	if (_timeStamp - _time_last_tz > EST_SRC_TIMEOUT) {
		if (!_tzTimeout) {
			_tzTimeout = true;
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] tz timeout ");
		}

	} else if (_tzTimeout) {
		mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] tz resume ");
		_tzTimeout = false;
	}

	lidarCheckTimeout();
	sonarCheckTimeout();
	baroCheckTimeout();
	gpsCheckTimeout();
	flowCheckTimeout();
	visionCheckTimeout();
	mocapCheckTimeout();
}

float BlockLocalPositionEstimator::agl()
{
	return _x(X_tz) - _x(X_z);
}

void BlockLocalPositionEstimator::detectDistanceSensors()
{
	for (int i = 0; i < N_DIST_SUBS; i++) {
		uORB::Subscription<distance_sensor_s> *s = _dist_subs[i];

		if (s == _sub_lidar || s == _sub_sonar) { continue; }

		if (s->updated()) {
			s->update();

			if (s->get().timestamp == 0) { continue; }

			if (s->get().type == \
			    distance_sensor_s::MAV_DISTANCE_SENSOR_LASER &&
			    _sub_lidar == NULL) {
				_sub_lidar = s;
				mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] Lidar detected with ID %i", i);

			} else if (s->get().type == \
				   distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND &&
				   _sub_sonar == NULL) {
				_sub_sonar = s;
				mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] Sonar detected with ID %i", i);
			}
		}
	}
}

void BlockLocalPositionEstimator::updateHome()
{
	double lat = _sub_home.get().lat;
	double lon = _sub_home.get().lon;
	float alt = _sub_home.get().alt;

	// updating home causes absolute measurements
	// like gps and baro to be off, need to allow it
	// to reset by resetting covariance
	initP();
	mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] home "
				     "lat %6.2f lon %6.2f alt %5.1f m",
				     lat, lon, double(alt));
	map_projection_init(&_map_ref, lat, lon);
	float delta_alt = alt - _altHome;
	_altHomeInitialized = true;
	_altHome = alt;
	_gpsAltHome += delta_alt;
	_baroAltHome +=  delta_alt;
	_visionHome(2) += delta_alt;
	_mocapHome(2) += delta_alt;
}

void BlockLocalPositionEstimator::publishLocalPos()
{
	// publish local position
	if (PX4_ISFINITE(_x(X_x)) && PX4_ISFINITE(_x(X_y)) && PX4_ISFINITE(_x(X_z)) &&
	    PX4_ISFINITE(_x(X_vx)) && PX4_ISFINITE(_x(X_vy))
	    && PX4_ISFINITE(_x(X_vz))) {
		_pub_lpos.get().timestamp = _timeStamp;
		_pub_lpos.get().xy_valid = _canEstimateXY;
		_pub_lpos.get().z_valid = _canEstimateZ;
		_pub_lpos.get().v_xy_valid = _canEstimateXY;
		_pub_lpos.get().v_z_valid = _canEstimateZ;
		_pub_lpos.get().x = _x(X_x); 	// north
		_pub_lpos.get().y = _x(X_y);  	// east
		_pub_lpos.get().z = _x(X_z); 	// down
		_pub_lpos.get().vx = _x(X_vx);  // north
		_pub_lpos.get().vy = _x(X_vy);  // east
		_pub_lpos.get().vz = _x(X_vz); 	// down
		_pub_lpos.get().yaw = _sub_att.get().yaw;
		_pub_lpos.get().xy_global = _sub_home.get().timestamp != 0; // need home for reference
		_pub_lpos.get().z_global = _baroInitialized;
		_pub_lpos.get().ref_timestamp = _sub_home.get().timestamp;
		_pub_lpos.get().ref_lat = _map_ref.lat_rad * 180 / M_PI;
		_pub_lpos.get().ref_lon = _map_ref.lon_rad * 180 / M_PI;
		_pub_lpos.get().ref_alt = _sub_home.get().alt;
		_pub_lpos.get().dist_bottom = agl();
		_pub_lpos.get().dist_bottom_rate = -_x(X_vz);
		_pub_lpos.get().surface_bottom_timestamp = _timeStamp;
		_pub_lpos.get().dist_bottom_valid = _canEstimateZ;
		_pub_lpos.get().eph = sqrtf(_P(X_x, X_x) + _P(X_y, X_y));
		_pub_lpos.get().epv = sqrtf(_P(X_z, X_z));
		_pub_lpos.update();
	}
}

void BlockLocalPositionEstimator::publishEstimatorStatus()
{
	if (PX4_ISFINITE(_x(X_x)) &&
	    PX4_ISFINITE(_x(X_y)) && PX4_ISFINITE(_x(X_z)) &&
	    PX4_ISFINITE(_x(X_vx)) && PX4_ISFINITE(_x(X_vy))
	    && PX4_ISFINITE(_x(X_vz))) {
		_pub_est_status.get().timestamp = _timeStamp;

		for (int i = 0; i < n_x; i++) {
			_pub_est_status.get().states[i] = _x(i);
			_pub_est_status.get().covariances[i] = _P(i, i);
		}

		_pub_est_status.get().n_states = n_x;
		_pub_est_status.get().nan_flags = 0;
		_pub_est_status.get().health_flags =
			((_baroFault > fault_lvl_disable) << SENSOR_BARO)
			+ ((_gpsFault > fault_lvl_disable) << SENSOR_GPS)
			+ ((_lidarFault > fault_lvl_disable) << SENSOR_LIDAR)
			+ ((_flowFault > fault_lvl_disable) << SENSOR_FLOW)
			+ ((_sonarFault > fault_lvl_disable) << SENSOR_SONAR)
			+ ((_visionFault > fault_lvl_disable) << SENSOR_VISION)
			+ ((_mocapFault > fault_lvl_disable) << SENSOR_MOCAP);
		_pub_est_status.get().timeout_flags =
			(_baroInitialized << SENSOR_BARO)
			+ (_gpsInitialized << SENSOR_GPS)
			+ (_flowInitialized << SENSOR_FLOW)
			+ (_lidarInitialized << SENSOR_LIDAR)
			+ (_sonarInitialized << SENSOR_SONAR)
			+ (_visionInitialized << SENSOR_VISION)
			+ (_mocapInitialized << SENSOR_MOCAP);
		_pub_est_status.update();
	}
}

void BlockLocalPositionEstimator::publishGlobalPos()
{
	// publish global position
	double lat = 0;
	double lon = 0;
	map_projection_reproject(&_map_ref, _x(X_x), _x(X_y), &lat, &lon);
	float alt = -_x(X_z) + _altHome;

	if (PX4_ISFINITE(lat) && PX4_ISFINITE(lon) && PX4_ISFINITE(alt) &&
	    PX4_ISFINITE(_x(X_vx)) && PX4_ISFINITE(_x(X_vy)) &&
	    PX4_ISFINITE(_x(X_vz))) {
		_pub_gpos.get().timestamp = _timeStamp;
		_pub_gpos.get().time_utc_usec = _sub_gps.get().time_utc_usec;
		_pub_gpos.get().lat = lat;
		_pub_gpos.get().lon = lon;
		_pub_gpos.get().alt = alt;
		_pub_gpos.get().vel_n = _x(X_vx);
		_pub_gpos.get().vel_e = _x(X_vy);
		_pub_gpos.get().vel_d = _x(X_vz);
		_pub_gpos.get().yaw = _sub_att.get().yaw;
		_pub_gpos.get().eph = sqrtf(_P(X_x, X_x) + _P(X_y, X_y));
		_pub_gpos.get().epv = sqrtf(_P(X_z, X_z));
		_pub_gpos.get().terrain_alt = _altHome - _x(X_tz);
		_pub_gpos.get().terrain_alt_valid = _canEstimateT;
		_pub_gpos.get().dead_reckoning = !_canEstimateXY && !_xyTimeout;
		_pub_gpos.get().pressure_alt = _sub_sensor.get().baro_alt_meter[0];
		_pub_gpos.update();
	}
}

void BlockLocalPositionEstimator::initP()
{
	_P.setZero();
	_P(X_x, X_x) = 1;
	_P(X_y, X_y) = 1;
	_P(X_z, X_z) = 1;
	_P(X_vx, X_vx) = 1;
	_P(X_vy, X_vy) = 1;
	_P(X_vz, X_vz) = 1;
	_P(X_bx, X_bx) = 1e-6;
	_P(X_by, X_by) = 1e-6;
	_P(X_bz, X_bz) = 1e-6;
	_P(X_tz, X_tz) = 1;
}

void BlockLocalPositionEstimator::predict()
{
	// if can't update anything, don't propagate
	// state or covariance
	if (!_canEstimateXY && !_canEstimateZ) { return; }

	if (_integrate.get() && _sub_att.get().R_valid) {
		Matrix3f R_att(_sub_att.get().R);
		Vector3f a(_sub_sensor.get().accelerometer_m_s2);
		_u = R_att * a;
		_u(U_az) += 9.81f; // add g

	} else {
		_u = Vector3f(0, 0, 0);
	}

	// dynamics matrix
	Matrix<float, n_x, n_x>  A; // state dynamics matrix
	A.setZero();
	// derivative of position is velocity
	A(X_x, X_vx) = 1;
	A(X_y, X_vy) = 1;
	A(X_z, X_vz) = 1;

	// derivative of velocity is accelerometer acceleration
	// (in input matrix) - bias (in body frame)
	Matrix3f R_att(_sub_att.get().R);
	A(X_vx, X_bx) = -R_att(0, 0);
	A(X_vx, X_by) = -R_att(0, 1);
	A(X_vx, X_bz) = -R_att(0, 2);

	A(X_vy, X_bx) = -R_att(1, 0);
	A(X_vy, X_by) = -R_att(1, 1);
	A(X_vy, X_bz) = -R_att(1, 2);

	A(X_vz, X_bx) = -R_att(2, 0);
	A(X_vz, X_by) = -R_att(2, 1);
	A(X_vz, X_bz) = -R_att(2, 2);

	// input matrix
	Matrix<float, n_x, n_u>  B; // input matrix
	B.setZero();
	B(X_vx, U_ax) = 1;
	B(X_vy, U_ay) = 1;
	B(X_vz, U_az) = 1;

	// input noise covariance matrix
	Matrix<float, n_u, n_u> R;
	R.setZero();
	R(U_ax, U_ax) = _accel_xy_stddev.get() * _accel_xy_stddev.get();
	R(U_ay, U_ay) = _accel_xy_stddev.get() * _accel_xy_stddev.get();
	R(U_az, U_az) = _accel_z_stddev.get() * _accel_z_stddev.get();

	// process noise power matrix
	Matrix<float, n_x, n_x>  Q;
	Q.setZero();
	float pn_p_sq = _pn_p_noise_density.get() * _pn_p_noise_density.get();
	float pn_v_sq = _pn_v_noise_density.get() * _pn_v_noise_density.get();
	Q(X_x, X_x) = pn_p_sq;
	Q(X_y, X_y) = pn_p_sq;
	Q(X_z, X_z) = pn_p_sq;
	Q(X_vx, X_vx) = pn_v_sq;
	Q(X_vy, X_vy) = pn_v_sq;
	Q(X_vz, X_vz) = pn_v_sq;

	// technically, the noise is in the body frame,
	// but the components are all the same, so
	// ignoring for now
	float pn_b_sq = _pn_b_noise_density.get() * _pn_b_noise_density.get();
	Q(X_bx, X_bx) = pn_b_sq;
	Q(X_by, X_by) = pn_b_sq;
	Q(X_bz, X_bz) = pn_b_sq;

	// terrain random walk noise
	float pn_t_sq = _pn_t_noise_density.get() * _pn_t_noise_density.get();
	Q(X_tz, X_tz) = pn_t_sq;

	// continuous time kalman filter prediction
	Vector<float, n_x> dx = (A * _x + B * _u) * getDt();

	// only predict for components we have
	// valid measurements for
	if (!_canEstimateXY) {
		dx(X_x) = 0;
		dx(X_y) = 0;
		dx(X_vx) = 0;
		dx(X_vy) = 0;
	}

	if (!_canEstimateZ) {
		dx(X_z) = 0;
		dx(X_vz) = 0;
	}

	// propagate
	_x += dx;
	_P += (A * _P + _P * A.transpose() +
	       B * R * B.transpose() + Q) * getDt();
}
