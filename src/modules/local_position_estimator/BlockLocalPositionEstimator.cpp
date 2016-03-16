#include "BlockLocalPositionEstimator.hpp"
#include <mavlink/mavlink_log.h>
#include <fcntl.h>
#include <systemlib/err.h>
#include <matrix/math.hpp>

static const int 		REQ_BARO_INIT_COUNT = 100;
static const int 		REQ_FLOW_INIT_COUNT = 20;
static const int 		REQ_GPS_INIT_COUNT = 10;
static const int 		REQ_LIDAR_INIT_COUNT = 20;
static const int 		REQ_SONAR_INIT_COUNT = 20;
static const int 		REQ_VISION_INIT_COUNT = 20;
static const int 		REQ_MOCAP_INIT_COUNT = 20;

static const uint32_t 		BARO_TIMEOUT = 1000000;		// 1.0 s
static const uint32_t 		FLOW_TIMEOUT = 500000;		// 0.5 s
static const uint32_t 		GPS_TIMEOUT = 1000000; 		// 1.0 s
static const uint32_t 		RANGER_TIMEOUT = 500000; 	// 0.5 s
static const uint32_t 		VISION_TIMEOUT = 500000;	// 0.5 s
static const uint32_t 		MOCAP_TIMEOUT = 200000;		// 0.2 s
static const uint32_t 		EST_SRC_TIMEOUT = 500000; // 0.5 s

// for fault detection
// chi squared distribution, false alarm probability 0.005
// http://sites.stat.psu.edu/~mga/401/tables/Chi-square-table.pdf
static const float BETA_TABLE[7] = {0, 7.879, 10.597,
				    12.838, 14.860, 16.750, 18.548
				   };

using namespace std;

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
	_distance_subs(),
	_sub_lidar(NULL),
	_sub_sonar(NULL),

	// publications
	_pub_lpos(ORB_ID(vehicle_local_position), -1, &getPublications()),
	_pub_gpos(ORB_ID(vehicle_global_position), -1, &getPublications()),
	//_pub_filtered_flow(ORB_ID(filtered_bottom_flow), -1, &getPublications()),
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
	_gps_xy_stddev(this, "GPS_XY"),
	_gps_z_stddev(this, "GPS_Z"),
	_gps_vxy_stddev(this, "GPS_VXY"),
	_gps_vz_stddev(this, "GPS_VZ"),
	_gps_eph_max(this, "EPH_MAX"),
	_vision_xy_stddev(this, "VIS_XY"),
	_vision_z_stddev(this, "VIS_Z"),
	_no_vision(this, "NO_VISION"),
	_mocap_p_stddev(this, "VIC_P"),
	_flow_z_offset(this, "FLW_OFF_Z"),
	_flow_xy_stddev(this, "FLW_XY"),
	//_flow_board_x_offs(NULL, "SENS_FLW_XOFF"),
	//_flow_board_y_offs(NULL, "SENS_FLW_YOFF"),
	_flow_min_q(this, "FLW_QMIN"),
	_pn_p_noise_power(this, "PN_P"),
	_pn_v_noise_power(this, "PN_V"),
	_pn_b_noise_power(this, "PN_B"),
	_pn_t_noise_power(this, "PN_T"),

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
	//_xDelay(this, ""),
	//_PDelay(this, ""),
	//_tDelay(this, ""),

	// misc
	_polls(),
	_timeStamp(hrt_absolute_time()),
	//_time_last_hist(0),
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

	// mavlink log
	_mavlink_fd(open(MAVLINK_LOG_DEVICE, 0)),

	// initialization flags
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

	// reference lat/lon
	_gpsLatHome(0),
	_gpsLonHome(0),

	// status
	_canEstimateXY(false),
	_canEstimateZ(false),
	_canEstimateT(false),
	_xyTimeout(true),
	_zTimeout(true),
	_tzTimeout(true),

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
	// setup event triggering based on new flow messages to integrate
	_polls[POLL_FLOW].fd = _sub_flow.getHandle();
	_polls[POLL_FLOW].events = POLLIN;

	_polls[POLL_PARAM].fd = _sub_param_update.getHandle();
	_polls[POLL_PARAM].events = POLLIN;

	_polls[POLL_SENSORS].fd = _sub_sensor.getHandle();
	_polls[POLL_SENSORS].events = POLLIN;

	//subscribe to all distance sensors
	for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
		_distance_subs[i] = new uORB::Subscription<distance_sensor_s>(
			ORB_ID(distance_sensor), 0, i, &getSubscriptions());
	}

	// initialize P, x, u
	initP();
	_x.setZero();
	_u.setZero();

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
	if (!_sub_armed.get().armed && (_sub_lidar == NULL || _sub_sonar == NULL)) {
		for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
			if (_distance_subs[i]->get().timestamp == 0) {
				continue; // ignore sensors with no data coming in
			}

			if (_distance_subs[i]->get().type == distance_sensor_s::MAV_DISTANCE_SENSOR_LASER &&
			    _sub_lidar == NULL) {
				_sub_lidar = _distance_subs[i];
				warnx("[lpe] Lidar detected with ID %i", i);

			} else if (_distance_subs[i]->get().type == distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND &&
				   _sub_sonar == NULL) {
				_sub_sonar = _distance_subs[i];
				warnx("[lpe] Sonar detected with ID %i", i);
			}
		}
	}

	// see which updates are available
	bool flowUpdated = _sub_flow.updated();
	bool paramsUpdated = _sub_param_update.updated();
	bool baroUpdated = _sub_sensor.updated();
	bool gpsUpdated = _sub_gps.updated();
	bool homeUpdated = _sub_home.updated();
	bool visionUpdated = _sub_vision_pos.updated();
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
		(_baroInitialized && _baroFault < FAULT_SEVERE);
	_canEstimateXY =
		(_gpsInitialized && _gpsFault < FAULT_SEVERE) ||
		(_flowInitialized && _flowFault < FAULT_SEVERE) ||
		(_visionInitialized && _visionFault < FAULT_SEVERE) ||
		(_mocapInitialized && _mocapFault < FAULT_SEVERE);
	_canEstimateT =
		(_lidarInitialized && _lidarFault < FAULT_SEVERE) ||
		(_sonarInitialized && _sonarFault < FAULT_SEVERE);

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
		map_projection_init(&_map_ref, 0, 0);
	}

	// reinitialize x if necessary
	bool reinit_x = false;

	for (int i = 0; i < n_x; i++) {
		// should we do a reinit
		// of sensors here?
		// don't want it to take too long
		if (!isfinite(_x(i))) {
			reinit_x = true;
			break;
		}
	}

	if (reinit_x) {
		for (int i = 0; i < n_x; i++) {
			_x(i) = 0;
		}

		mavlink_log_info(_mavlink_fd, "[lpe] reinit x");
		warnx("[lpe] reinit x");
	}

	// reinitialize P if necessary
	bool reinit_P = false;

	for (int i = 0; i < n_x; i++) {
		for (int j = 0; j < n_x; j++) {
			if (!isfinite(_P(i, j))) {
				reinit_P = true;
				break;
			}
		}

		if (reinit_P) { break; }
	}

	if (reinit_P) {
		mavlink_log_info(_mavlink_fd, "[lpe] reinit P");
		warnx("[lpe] reinit P");
		initP();
	}

	// do prediction
	predict();

	// sensor corrections/ initializations
	if (gpsUpdated) {
		if (!_gpsInitialized) {
			initGps();

		} else {
			correctGps();
		}
	}

	if (baroUpdated) {
		if (!_baroInitialized) {
			initBaro();

		} else {
			correctBaro();
		}
	}

	if (lidarUpdated) {
		if (!_lidarInitialized) {
			initLidar();

		} else {
			correctLidar();
		}
	}

	if (sonarUpdated) {
		if (!_sonarInitialized) {
			initSonar();

		} else {
			correctSonar();
		}
	}

	if (flowUpdated) {
		if (!_flowInitialized) {
			initFlow();

		} else {
			perf_begin(_loop_perf);// TODO
			correctFlow();
			//perf_count(_interval_perf);
			perf_end(_loop_perf);
		}
	}

	if (_no_vision.get() != CBRK_NO_VISION_KEY) { // check if no vision circuit breaker is set
		if (visionUpdated) {
			if (!_visionInitialized) {
				initVision();

			} else {
				correctVision();
			}
		}
	}

	if (mocapUpdated) {
		if (!_mocapInitialized) {
			initMocap();

		} else {
			correctMocap();
		}
	}

	if (!_xyTimeout && _altHomeInitialized) {
		// update all publications if possible
		publishLocalPos();
		publishEstimatorStatus();
		publishGlobalPos();

	} else if (!_zTimeout && _altHomeInitialized) {
		// publish only Z estimate
		publishLocalPos();
		publishEstimatorStatus();
	}

}

void BlockLocalPositionEstimator::checkTimeouts()
{
	if (_timeStamp - _time_last_xy > EST_SRC_TIMEOUT) {
		if (!_xyTimeout) {
			_xyTimeout = true;
			mavlink_log_info(_mavlink_fd, "[lpe] xy timeout ");
			warnx("[lpe] xy timeout ");
		}

	} else {
		_xyTimeout = false;
	}

	if (_timeStamp - _time_last_z > EST_SRC_TIMEOUT) {
		if (!_zTimeout) {
			_zTimeout = true;
			mavlink_log_info(_mavlink_fd, "[lpe] z timeout ");
			warnx("[lpe] z timeout ");
		}

	} else {
		_zTimeout = false;
	}

	if (_timeStamp - _time_last_tz > EST_SRC_TIMEOUT) {
		if (!_tzTimeout) {
			_tzTimeout = true;
			mavlink_log_info(_mavlink_fd, "[lpe] tz timeout ");
			warnx("[lpe] tz timeout ");
		}

	} else {
		_tzTimeout = false;
	}

	if (_timeStamp - _time_last_baro > BARO_TIMEOUT) {
		if (_baroInitialized) {
			_baroInitialized = false;
			mavlink_log_info(_mavlink_fd, "[lpe] baro timeout ");
			warnx("[lpe] baro timeout ");
		}
	}

	if (_timeStamp - _time_last_gps > GPS_TIMEOUT) {
		if (_gpsInitialized) {
			_gpsInitialized = false;
			mavlink_log_info(_mavlink_fd, "[lpe] GPS timeout ");
			warnx("[lpe] GPS timeout ");
		}
	}

	if (_timeStamp - _time_last_flow > FLOW_TIMEOUT) {
		if (_flowInitialized) {
			_flowInitialized = false;
			mavlink_log_info(_mavlink_fd, "[lpe] flow timeout ");
			warnx("[lpe] flow timeout ");
		}
	}

	if (_timeStamp - _time_last_sonar > RANGER_TIMEOUT) {
		if (_sonarInitialized) {
			_sonarInitialized = false;
			mavlink_log_info(_mavlink_fd, "[lpe] sonar timeout ");
			warnx("[lpe] sonar timeout ");
		}
	}

	if (_timeStamp - _time_last_lidar > RANGER_TIMEOUT) {
		if (_lidarInitialized) {
			_lidarInitialized = false;
			mavlink_log_info(_mavlink_fd, "[lpe] lidar timeout ");
			warnx("[lpe] lidar timeout ");
		}
	}

	if (_timeStamp - _time_last_vision_p > VISION_TIMEOUT) {
		if (_visionInitialized) {
			_visionInitialized = false;
			mavlink_log_info(_mavlink_fd, "[lpe] vision position timeout ");
			warnx("[lpe] vision position timeout ");
		}
	}

	if (_timeStamp - _time_last_mocap > MOCAP_TIMEOUT) {
		if (_mocapInitialized) {
			_mocapInitialized = false;
			mavlink_log_info(_mavlink_fd, "[lpe] mocap timeout ");
			warnx("[lpe] mocap timeout ");
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

	mavlink_log_info(_mavlink_fd, "[lpe] home: lat %5.0f, lon %5.0f, alt %5.0f", lat, lon, double(alt));
	warnx("[lpe] home: lat %5.0f, lon %5.0f, alt %5.0f", lat, lon, double(alt));
	map_projection_init(&_map_ref, lat, lon);
	float delta_alt = alt - _altHome;
	_altHomeInitialized = true;
	_altHome = alt;
	_gpsAltHome += delta_alt;
	_baroAltHome +=  delta_alt;
	_visionHome(2) += delta_alt;
	_mocapHome(2) += delta_alt;
}

void BlockLocalPositionEstimator::initBaro()
{
	// collect baro data
	_baroStats.update(Scalarf(_sub_sensor.get().baro_alt_meter[0]));
	_time_last_baro = _timeStamp;

	if (_baroStats.getCount() > REQ_BARO_INIT_COUNT) {
		_baroAltHome = _baroStats.getMean()(0);
		mavlink_log_info(_mavlink_fd,
				 "[lpe] baro offs: %d m stddev %d cm",
				 (int)_baroStats.getMean()(0),
				 (int)(100 * _baroStats.getStdDev()(0)));
		warnx("[lpe] baro offs: %d m stddev %d cm",
		      (int)_baroStats.getMean()(0),
		      (int)(100 * _baroStats.getStdDev()(0)));
		_baroInitialized = true;
		_baroStats.reset();

		if (!_altHomeInitialized) {
			_altHomeInitialized = true;
			_altHome = _baroAltHome;
		}
	}
}


void BlockLocalPositionEstimator::initGps()
{
	// check for good gps signal
	uint8_t nSat = _sub_gps.get().satellites_used;
	float eph = _sub_gps.get().eph;

	if (nSat < 6 || eph > _gps_eph_max.get()) {
		_gpsStats.reset();
		return;
	}

	// collect gps data
	Vector3<double> p(
		_sub_gps.get().lat * 1e-7,
		_sub_gps.get().lon * 1e-7,
		_sub_gps.get().alt * 1e-3);

	// increament sums for mean
	_gpsStats.update(p);
	_time_last_gps = _timeStamp;

	if (_gpsStats.getCount() > REQ_GPS_INIT_COUNT) {
		_gpsLatHome = _gpsStats.getMean()(0);
		_gpsLonHome = _gpsStats.getMean()(1);
		_gpsAltHome = _gpsStats.getMean()(2);
		map_projection_init(&_map_ref,
				    _gpsLatHome, _gpsLonHome);
		mavlink_log_info(_mavlink_fd, "[lpe] gps init: "
				 "lat %d, lon %d, alt %d m",
				 int(_gpsLatHome),
				 int(_gpsLonHome),
				 int(_gpsAltHome));
		warnx("[lpe] gps init: lat %d, lon %d, alt %d m",
		      int(_gpsLatHome),
		      int(_gpsLonHome),
		      int(_gpsAltHome));
		_gpsInitialized = true;
		_gpsStats.reset();

		if (!_altHomeInitialized) {
			_altHomeInitialized = true;
			_altHome = _gpsAltHome;
		}
	}
}

void BlockLocalPositionEstimator::initLidar()
{
	// measure
	float d = _sub_lidar->get().current_distance + _lidar_z_offset.get();
	float eps = 0.01f;
	float min_dist = _sub_lidar->get().min_distance + eps;
	float max_dist = _sub_lidar->get().max_distance - eps;

	// check for bad data
	if (d > max_dist || d < min_dist) {
		_lidarStats.reset();
		return;
	}

	// update stats
	_lidarStats.update(Scalarf(d));
	_time_last_lidar = _timeStamp;

	// if finished
	if (_lidarStats.getCount() > REQ_LIDAR_INIT_COUNT) {
		// if stddev too high, retry
		if (_lidarStats.getStdDev()(0) > 0.1f) {
			_lidarStats.reset();
			return;
		}

		// not, might want to hard code this to zero
		mavlink_log_info(_mavlink_fd, "[lpe] lidar init: "
				 "mean %d cm, stddev %d cm",
				 int(100 * _lidarStats.getMean()(0)),
				 int(100 * _lidarStats.getStdDev()(0)));
		warnx("[lpe] lidar init: "
		      "mean %d cm, stddev %d cm",
		      int(100 * _lidarStats.getMean()(0)),
		      int(100 * _lidarStats.getStdDev()(0)));
		_lidarInitialized = true;
		_lidarStats.reset();
	}
}

void BlockLocalPositionEstimator::initSonar()
{
	// measure
	float d = _sub_sonar->get().current_distance + _sonar_z_offset.get();
	float eps = 0.01f;
	float min_dist = _sub_sonar->get().min_distance + eps;
	float max_dist = _sub_sonar->get().max_distance - eps;

	// check for bad data
	if (d < min_dist || d > max_dist) {
		_sonarStats.reset();
		return;
	}

	// update stats
	_sonarStats.update(Scalarf(d));
	_time_last_sonar = _timeStamp;

	// if finished
	if (_sonarStats.getCount() > REQ_SONAR_INIT_COUNT) {
		// if stddev too high, retry
		if (_sonarStats.getStdDev()(0) > 0.1f) {
			_sonarStats.reset();
			return;
		}

		// not, might want to hard code this to zero
		mavlink_log_info(_mavlink_fd, "[lpe] sonar init: "
				 "mean %d cm, stddev %d cm",
				 int(100 * _sonarStats.getMean()(0)),
				 int(100 * _sonarStats.getStdDev()(0)));
		warnx("[lpe] sonar init: "
		      "mean %d cm, stddev %d cm",
		      int(100 * _sonarStats.getMean()(0)),
		      int(100 * _sonarStats.getStdDev()(0)));
		_sonarInitialized = true;
	}
}

void BlockLocalPositionEstimator::initFlow()
{
	// increament sums for mean
	float qual = _sub_flow.get().quality;

	// check for bad data
	if (qual < _flow_min_q.get()) {
		_flowQStats.reset();
		return;
	}

	_flowQStats.update(Scalarf(_sub_flow.get().quality));
	_time_last_flow = _timeStamp;

	if (_flowQStats.getCount() > REQ_FLOW_INIT_COUNT) {
		mavlink_log_info(_mavlink_fd, "[lpe] flow init: "
				 "quality %d stddev %d",
				 int(_flowQStats.getMean()(0)),
				 int(_flowQStats.getStdDev()(0)));
		warnx("[lpe] flow init: "
		      "quality %d stddev %d",
		      int(_flowQStats.getMean()(0)),
		      int(_flowQStats.getStdDev()(0)));
		_flowInitialized = true;
		_flowQStats.reset();
	}
}

void BlockLocalPositionEstimator::initVision()
{
	// collect vision position data
	Vector3f pos;
	pos(0) = _sub_vision_pos.get().x;
	pos(1) = _sub_vision_pos.get().y;
	pos(2) = _sub_vision_pos.get().z;

	// increament sums for mean
	_visionStats.update(pos);
	_time_last_vision_p = _timeStamp;

	if (_visionStats.getCount() > REQ_VISION_INIT_COUNT) {
		_visionHome = _visionStats.getMean();
		mavlink_log_info(_mavlink_fd, "[lpe] vision position init: "
				 "%f, %f, %f m std dev. %f, %f, %f m",
				 double(_visionStats.getMean()(0)),
				 double(_visionStats.getMean()(1)),
				 double(_visionStats.getMean()(2)),
				 double(_visionStats.getStdDev()(0)),
				 double(_visionStats.getStdDev()(1)),
				 double(_visionStats.getStdDev()(2)));
		warnx("[lpe] vision position init: "
		      "%f, %f, %f m std dev. %f, %f, %f m",
		      double(_visionStats.getMean()(0)),
		      double(_visionStats.getMean()(1)),
		      double(_visionStats.getMean()(2)),
		      double(_visionStats.getStdDev()(0)),
		      double(_visionStats.getStdDev()(1)),
		      double(_visionStats.getStdDev()(2)));
		_visionInitialized = true;
		_visionStats.reset();

		if (!_altHomeInitialized) {
			_altHomeInitialized = true;
			_altHome = _visionHome(2);
		}
	}
}

void BlockLocalPositionEstimator::initMocap()
{
	// collect mocap data
	Vector3f pos;
	pos(0) = _sub_mocap.get().x;
	pos(1) = _sub_mocap.get().y;
	pos(2) = _sub_mocap.get().z;

	// increament sums for mean
	_mocapStats.update(pos);
	_time_last_mocap = _timeStamp;

	if (_mocapStats.getCount() > REQ_MOCAP_INIT_COUNT) {
		_mocapHome = _mocapStats.getMean();
		mavlink_log_info(_mavlink_fd, "[lpe] mocap position init: "
				 "%f, %f, %f m std dev. %f, %f, %f m",
				 double(_mocapStats.getMean()(0)),
				 double(_mocapStats.getMean()(1)),
				 double(_mocapStats.getMean()(2)),
				 double(_mocapStats.getStdDev()(0)),
				 double(_mocapStats.getStdDev()(1)),
				 double(_mocapStats.getStdDev()(2)));
		warnx("[lpe] mocap position init: "
		      "%f, %f, %f m std dev. %f, %f, %f m",
		      double(_mocapStats.getMean()(0)),
		      double(_mocapStats.getMean()(1)),
		      double(_mocapStats.getMean()(2)),
		      double(_mocapStats.getStdDev()(0)),
		      double(_mocapStats.getStdDev()(1)),
		      double(_mocapStats.getStdDev()(2)));
		_mocapInitialized = true;
		_mocapStats.reset();

		if (!_altHomeInitialized) {
			_altHomeInitialized = true;
			_altHome = _mocapHome(2);
		}
	}
}

void BlockLocalPositionEstimator::publishLocalPos()
{
	// publish local position
	if (isfinite(_x(X_x)) && isfinite(_x(X_y)) && isfinite(_x(X_z)) &&
	    isfinite(_x(X_vx)) && isfinite(_x(X_vy))
	    && isfinite(_x(X_vz))) {
		_pub_lpos.get().timestamp = _timeStamp;
		_pub_lpos.get().xy_valid = _canEstimateXY;
		_pub_lpos.get().z_valid = _canEstimateZ;
		_pub_lpos.get().v_xy_valid = _canEstimateXY;
		_pub_lpos.get().v_z_valid = _canEstimateZ;
		_pub_lpos.get().x = _x(X_x); 	// north
		_pub_lpos.get().y = _x(X_y);  	// east
		_pub_lpos.get().z = _x(X_z) - _x(X_tz); 	// down, AGL
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
		_pub_lpos.get().dist_bottom = -_x(X_tz);
		_pub_lpos.get().dist_bottom_rate = -_x(X_vz);
		_pub_lpos.get().surface_bottom_timestamp = _timeStamp;
		_pub_lpos.get().dist_bottom_valid = _sonarInitialized || _lidarInitialized;
		_pub_lpos.get().eph = sqrtf(_P(X_x, X_x) + _P(X_y, X_y));
		_pub_lpos.get().epv = sqrtf(_P(X_z, X_z));
		_pub_lpos.update();
	}
}

void BlockLocalPositionEstimator::publishEstimatorStatus()
{
	if (isfinite(_x(X_x)) && isfinite(_x(X_y)) && isfinite(_x(X_z)) &&
	    isfinite(_x(X_vx)) && isfinite(_x(X_vy))
	    && isfinite(_x(X_vz))) {
		_pub_est_status.get().timestamp = _timeStamp;

		for (int i = 0; i < n_x; i++) {
			_pub_est_status.get().states[i] = _x(i);
			_pub_est_status.get().covariances[i] = _P(i, i);
		}

		_pub_est_status.get().n_states = n_x;
		_pub_est_status.get().nan_flags = 0;
		_pub_est_status.get().health_flags =
			((_baroFault > 0) << SENSOR_BARO)
			+ ((_gpsFault > 0) << SENSOR_GPS)
			+ ((_lidarFault > 0) << SENSOR_LIDAR)
			+ ((_flowFault > 0) << SENSOR_FLOW)
			+ ((_sonarFault > 0) << SENSOR_SONAR)
			+ ((_visionFault > 0) << SENSOR_VISION)
			+ ((_mocapFault > 0) << SENSOR_MOCAP);
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

	if (isfinite(lat) && isfinite(lon) && isfinite(alt) &&
	    isfinite(_x(X_vx)) && isfinite(_x(X_vy)) &&
	    isfinite(_x(X_vz))) {
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
		_pub_gpos.get().terrain_alt = alt - _x(X_tz); // TODO assuming this is ASL?
		_pub_gpos.get().terrain_alt_valid = _lidarInitialized || _sonarInitialized;
		_pub_gpos.get().dead_reckoning = !_canEstimateXY && !_xyTimeout;
		_pub_gpos.get().pressure_alt = _sub_sensor.get().baro_alt_meter[0];
		_pub_gpos.update();
	}
}

//void BlockLocalPositionEstimator::publishFilteredFlow()
//{
//// publish filtered flow
//if (isfinite(_pub_filtered_flow.get().sumx) &&
//isfinite(_pub_filtered_flow.get().sumy) &&
//isfinite(_pub_filtered_flow.get().vx) &&
//isfinite(_pub_filtered_flow.get().vy)) {
//_pub_filtered_flow.update();
//}
//}

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
	Q(X_x, X_x) = _pn_p_noise_power.get();
	Q(X_y, X_y) = _pn_p_noise_power.get();
	Q(X_z, X_z) = _pn_p_noise_power.get();
	Q(X_vx, X_vx) = _pn_v_noise_power.get();
	Q(X_vy, X_vy) = _pn_v_noise_power.get();
	Q(X_vz, X_vz) = _pn_v_noise_power.get();

	// technically, the noise is in the body frame,
	// but the components are all the same, so
	// ignoring for now
	Q(X_bx, X_bx) = _pn_b_noise_power.get();
	Q(X_by, X_by) = _pn_b_noise_power.get();
	Q(X_bz, X_bz) = _pn_b_noise_power.get();

	// terrain random walk noise
	Q(X_tz, X_tz) = _pn_t_noise_power.get();

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

	// propagate delayed state
	//if (_time_last_hist == 0 || _time_last_hist - _timeStamp > 1000) {
	//_xDelay.update(_x);
	//_PDelay.update(_P);
	//_tDelay.update(Scalar<uint64_t>(_timeStamp));
	//}
}

void BlockLocalPositionEstimator::correctFlow()
{
	// check quality
	float qual = _sub_flow.get().quality;

	if (qual < _flow_min_q.get()) {
		if (_flowFault < FAULT_SEVERE) {
			mavlink_log_info(_mavlink_fd, "[lpe] low flow quality %d", int(qual));
			warnx("[lpe] low flow quality %d", int(qual));
			_flowFault = FAULT_SEVERE;
		}

		return;
	}

	// imporant to timestamp flow even if distance is bad
	_time_last_flow = _sub_flow.get().timestamp;

	// calculate range to center of image for flow
	float d = 0;

	if (_lidarInitialized && _lidarFault < FAULT_SEVERE) {
		d = _sub_lidar->get().current_distance
		    + (_lidar_z_offset.get() - _flow_z_offset.get());

	} else if (_sonarInitialized && _sonarFault < FAULT_SEVERE) {
		d = _sub_sonar->get().current_distance
		    + (_sonar_z_offset.get() - _flow_z_offset.get());

	} else {
		// no valid distance sensor, so return
		if (_flowFault < FAULT_SEVERE) {
			mavlink_log_info(_mavlink_fd, "[lpe] no distance for flow");
			warnx("[lpe] no distance for flow");
			_flowFault = FAULT_SEVERE;
		}

		return;
	}

	// flow measurement matrix and noise matrix
	Matrix<float, n_y_flow, n_x> C;
	C.setZero();
	C(Y_flow_x, X_x) = 1;
	C(Y_flow_y, X_y) = 1;

	Matrix<float, n_y_flow, n_y_flow> R;
	R.setZero();
	R(Y_flow_x, Y_flow_x) =
		_flow_xy_stddev.get() * _flow_xy_stddev.get();
	R(Y_flow_y, Y_flow_y) =
		_flow_xy_stddev.get() * _flow_xy_stddev.get();

	// calc dt between flow timestamps
	// ignore first flow msg
	if (_time_last_flow == 0) {
		_time_last_flow = _sub_flow.get().timestamp;
		return;
	}

	// optical flow in x, y axis
	float flow_x_rad = _sub_flow.get().pixel_flow_x_integral;
	float flow_y_rad = _sub_flow.get().pixel_flow_y_integral;

	// angular rotation in x, y axis
	float gyro_x_rad = _flow_gyro_x_high_pass.update(
				   _sub_flow.get().gyro_x_rate_integral);
	float gyro_y_rad = _flow_gyro_y_high_pass.update(
				   _sub_flow.get().gyro_y_rate_integral);

	// compute velocities in camera frame using ground distance
	// assume camera frame is body frame
	// TODO account for frame where flow is mounted
	Vector3f delta_b(
		-(flow_x_rad - gyro_x_rad)*d,
		-(flow_y_rad - gyro_y_rad)*d,
		0);

	// rotation of flow from body to nav frame
	Matrix3f R_nb(_sub_att.get().R);
	Vector3f delta_n = R_nb * delta_b;

	// flow integration
	_flowX += delta_n(0);
	_flowY += delta_n(1);

	/* update filtered flow */
	//float dt_flow = _sub_flow.get().integration_timespan / 1.0e6;
	//_pub_filtered_flow.get().sumx = delta_n(0);
	//_pub_filtered_flow.get().sumy = delta_n(1);
	//_pub_filtered_flow.get().vx = delta_n(0) / dt_flow;
	//_pub_filtered_flow.get().vy = delta_n(1) / dt_flow;

	// measurement
	Vector<float, 2> y;
	y(0) = _flowX;
	y(1) = _flowY;

	// residual
	Vector<float, 2> r = y - C * _x;

	// residual covariance, (inverse)
	Matrix<float, n_y_flow, n_y_flow> S_I =
		inv<float, n_y_flow>(C * _P * C.transpose() + R);

	// fault detection
	float beta = (r.transpose() * (S_I * r))(0, 0);

	if (beta > BETA_TABLE[n_y_flow]) {
		if (_flowFault < FAULT_MINOR) {
			mavlink_log_info(_mavlink_fd, "[lpe] flow fault,  beta %5.2f", double(beta));
			warnx("[lpe] flow fault,  beta %5.2f", double(beta));
			_flowFault = FAULT_MINOR;
		}

	} else if (_flowFault) {
		_flowFault = FAULT_NONE;
		mavlink_log_info(_mavlink_fd, "[lpe] flow OK");
		warnx("[lpe] flow OK");
	}

	// kalman filter correction if no fault
	if (_flowFault < FAULT_SEVERE) {
		Matrix<float, n_x, n_y_flow> K =
			_P * C.transpose() * S_I;
		_x += K * r;
		_P -= K * C * _P;

	} else {
		// reset flow integral to current estimate of position
		// if a fault occurred
		_flowX = _x(X_x);
		_flowY = _x(X_y);
	}

}

void BlockLocalPositionEstimator::correctSonar()
{
	// measure
	float d = _sub_sonar->get().current_distance + _sonar_z_offset.get();
	float eps = 0.01f;
	float min_dist = _sub_sonar->get().min_distance + eps;
	float max_dist = _sub_sonar->get().max_distance - eps;

	if (d < min_dist) {
		//mavlink_log_info(_mavlink_fd, "[lpe] sonar min dist");
		warnx("[lpe] sonar min dist");
		// can't correct, so return
		return;

	} else if (d > max_dist) {
		if (_sonarFault < FAULT_SEVERE) {
			mavlink_log_info(_mavlink_fd, "[lpe] sonar max distance");
			warnx("[lpe] sonar max distance");
			_sonarFault = FAULT_SEVERE;
		}

		// can't correct, so return
		return;
	}

	_time_last_sonar = _timeStamp;

	// do not use sonar if lidar is active
	if (_lidarInitialized && _lidarFault < FAULT_SEVERE) { return; }

	// calculate covariance
	float cov = _sub_sonar->get().covariance;

	if (cov < 1.0e-3f) {
		// use sensor value if reasoanble
		cov = _sonar_z_stddev.get() * _sonar_z_stddev.get();
	}

	// sonar measurement matrix and noise matrix
	Matrix<float, n_y_sonar, n_x> C;
	C.setZero();
	// y = -(z - tz)
	// TODO could add trig to make this an EKF correction
	C(Y_sonar_z, X_z) = -1; // measured altitude, negative down dir.
	C(Y_sonar_z, X_tz) = 1; // measured altitude, negative down dir.

	// covariance matrix
	Matrix<float, n_y_sonar, n_y_sonar> R;
	R.setZero();
	R(0, 0) = cov;

	// measurement
	Vector<float, n_y_sonar> y;
	y(0) = d *
	       cosf(_sub_att.get().roll) *
	       cosf(_sub_att.get().pitch);

	// residual
	Vector<float, n_y_sonar> r = y - C * _x;

	// residual covariance, (inverse)
	Matrix<float, n_y_sonar, n_y_sonar> S_I =
		inv<float, n_y_sonar>(C * _P * C.transpose() + R);

	// fault detection
	float beta = (r.transpose()  * (S_I * r))(0, 0);

	if (beta > BETA_TABLE[n_y_sonar]) {
		if (_sonarFault < FAULT_MINOR) {
			mavlink_log_info(_mavlink_fd, "[lpe] sonar fault,  beta %5.2f", double(beta));
			warnx("[lpe] sonar fault,  beta %5.2f", double(beta));
			_sonarFault = FAULT_MINOR;
		}

	} else if (_sonarFault) {
		_sonarFault = FAULT_NONE;
		mavlink_log_info(_mavlink_fd, "[lpe] sonar OK");
		warnx("[lpe] sonar OK");
	}

	// kalman filter correction if no fault
	if (_sonarFault < FAULT_SEVERE) {
		Matrix<float, n_x, n_y_sonar> K =
			_P * C.transpose() * S_I;
		Vector<float, n_x> dx = K * r;

		if (!_canEstimateXY) {
			dx(X_x) = 0;
			dx(X_y) = 0;
			dx(X_vx) = 0;
			dx(X_vy) = 0;
		}

		_x += dx;
		_P -= K * C * _P;
	}

}

void BlockLocalPositionEstimator::correctBaro()
{
	// measure
	Vector<float, n_y_baro> y;
	y(0) = _sub_sensor.get().baro_alt_meter[0] - _baroAltHome;
	_time_last_baro = _timeStamp;

	// baro measurement matrix
	Matrix<float, n_y_baro, n_x> C;
	C.setZero();
	C(Y_baro_z, X_z) = -1; // measured altitude, negative down dir.

	Matrix<float, n_y_baro, n_y_baro> R;
	R.setZero();
	R(0, 0) = _baro_stddev.get() * _baro_stddev.get();

	// residual
	Matrix<float, n_y_baro, n_y_baro> S_I =
		inv<float, n_y_baro>((C * _P * C.transpose()) + R);
	Vector<float, n_y_baro> r = y - (C * _x);

	// fault detection
	float beta = (r.transpose() * (S_I * r))(0, 0);

	if (beta > BETA_TABLE[n_y_baro]) {
		if (_baroFault < FAULT_MINOR) {
			mavlink_log_info(_mavlink_fd, "[lpe] baro fault, beta %5.2f", double(beta));
			warnx("[lpe] baro fault, beta %5.2f", double(beta));
			_baroFault = FAULT_MINOR;
		}

		// lower baro trust
		S_I = inv<float, n_y_baro>((C * _P * C.transpose()) + R * 10);

	} else if (_baroFault) {
		_baroFault = FAULT_NONE;
		mavlink_log_info(_mavlink_fd, "[lpe] baro OK");
		warnx("[lpe] baro OK");
	}

	// kalman filter correction if no fault
	if (_baroFault < FAULT_SEVERE) {
		Matrix<float, n_x, n_y_baro> K = _P * C.transpose() * S_I;
		Vector<float, n_x> dx = K * r;

		if (!_canEstimateXY) {
			dx(X_x) = 0;
			dx(X_y) = 0;
			dx(X_vx) = 0;
			dx(X_vy) = 0;
		}

		_x += dx;
		_P -= K * C * _P;
	}
}

void BlockLocalPositionEstimator::correctLidar()
{
	// measure
	float d = _sub_lidar->get().current_distance + _lidar_z_offset.get();
	float eps = 0.01f;
	float min_dist = _sub_lidar->get().min_distance + eps;
	float max_dist = _sub_lidar->get().max_distance - eps;

	// if out of range, this is an error
	if (d < min_dist || d > max_dist) {
		if (_lidarFault < FAULT_SEVERE) {
			mavlink_log_info(_mavlink_fd, "[lpe] lidar out of range");
			warnx("[lpe] lidar out of range");
			_lidarFault = FAULT_SEVERE;
		}

		return;
	}

	_time_last_lidar = _timeStamp;

	Matrix<float, n_y_lidar, n_x> C;
	C.setZero();
	// y = -(z - tz)
	// TODO could add trig to make this an EKF correction
	C(Y_lidar_z, X_z) = -1; // measured altitude, negative down dir.
	C(Y_lidar_z, X_tz) = 1; // measured altitude, negative down dir.

	// use parameter covariance unless sensor provides reasonable value
	Matrix<float, n_y_lidar, n_y_lidar> R;
	R.setZero();
	float cov = _sub_lidar->get().covariance;

	if (cov < 1.0e-3f) {
		R(0, 0) = _lidar_z_stddev.get() * _lidar_z_stddev.get();

	} else {
		R(0, 0) = cov;
	}

	Vector<float, n_y_lidar> y;
	y.setZero();
	y(0) = d *
	       cosf(_sub_att.get().roll) *
	       cosf(_sub_att.get().pitch);

	// residual
	Matrix<float, n_y_lidar, n_y_lidar> S_I = inv<float, n_y_lidar>((C * _P * C.transpose()) + R);
	Vector<float, n_y_lidar> r = y - C * _x;

	// fault detection
	float beta = (r.transpose() * (S_I * r))(0, 0);

	if (beta > BETA_TABLE[n_y_lidar]) {
		if (_lidarFault < FAULT_MINOR) {
			mavlink_log_info(_mavlink_fd, "[lpe] lidar fault, beta %5.2f", double(beta));
			warnx("[lpe] lidar fault, beta %5.2f", double(beta));
			_lidarFault = FAULT_MINOR;
		}

	} else if (_lidarFault) { // disable fault if ok
		_lidarFault = FAULT_NONE;
		mavlink_log_info(_mavlink_fd, "[lpe] lidar OK");
		warnx("[lpe] lidar OK");
	}

	// kalman filter correction if no fault
	if (_lidarFault < FAULT_SEVERE) {
		Matrix<float, n_x, n_y_lidar> K = _P * C.transpose() * S_I;
		Vector<float, n_x> dx = K * r;

		if (!_canEstimateXY) {
			dx(X_x) = 0;
			dx(X_y) = 0;
			dx(X_vx) = 0;
			dx(X_vy) = 0;
		}

		_x += dx;
		_P -= K * C * _P;
	}
}

void BlockLocalPositionEstimator::correctGps()
{
	// check for good gps signal
	uint8_t nSat = _sub_gps.get().satellites_used;
	float eph = _sub_gps.get().eph;

	if (nSat < 6 || eph > _gps_eph_max.get()) {
		if (!_gpsFault) {
			mavlink_log_info(_mavlink_fd, "[lpe] gps fault nSat: %d eph: %5.2f", nSat, double(eph));
			warnx("[lpe] gps fault nSat: %d eph: %5.2f", nSat, double(eph));
			_gpsFault = FAULT_SEVERE;
		}

		return;
	}

	// gps measurement in local frame
	_time_last_gps = _timeStamp;
	double  lat = _sub_gps.get().lat * 1.0e-7;
	double  lon = _sub_gps.get().lon * 1.0e-7;
	float  alt = _sub_gps.get().alt * 1.0e-3;
	float px = 0;
	float py = 0;
	float pz = -(alt - _gpsAltHome);
	map_projection_project(&_map_ref, lat, lon, &px, &py);
	Vector<float, 6> y;
	y.setZero();
	y(0) = px;
	y(1) = py;
	y(2) = pz;
	y(3) = _sub_gps.get().vel_n_m_s;
	y(4) = _sub_gps.get().vel_e_m_s;
	y(5) = _sub_gps.get().vel_d_m_s;

	// gps measurement matrix, measures position and velocity
	Matrix<float, n_y_gps, n_x> C;
	C.setZero();
	C(Y_gps_x, X_x) = 1;
	C(Y_gps_y, X_y) = 1;
	C(Y_gps_z, X_z) = 1;
	C(Y_gps_vx, X_vx) = 1;
	C(Y_gps_vy, X_vy) = 1;
	C(Y_gps_vz, X_vz) = 1;

	// gps covariance matrix
	Matrix<float, n_y_gps, n_y_gps> R;
	R.setZero();

	// default to parameter, use gps cov if provided
	float var_xy = _gps_xy_stddev.get() * _gps_xy_stddev.get();
	float var_z = _gps_z_stddev.get() * _gps_z_stddev.get();
	float var_vxy = _gps_vxy_stddev.get() * _gps_vxy_stddev.get();
	float var_vz = _gps_vz_stddev.get() * _gps_vz_stddev.get();

	// if field is not zero, set it to the value provided
	if (_sub_gps.get().eph > 1e-3f) {
		var_xy = _sub_gps.get().eph * _sub_gps.get().eph;
	}

	if (_sub_gps.get().epv > 1e-3f) {
		var_z = _sub_gps.get().epv * _sub_gps.get().epv;
	}

	R(0, 0) = var_xy;
	R(1, 1) = var_xy;
	R(2, 2) = var_z;
	R(3, 3) = var_vxy;
	R(4, 4) = var_vxy;
	R(5, 5) = var_vz;

	// get delayed x and P
	//uint64_t t_delay = 0;
	//int i = 0;

	//for (i = 0; i < HIST_LEN; i++) {
	//t_delay += _tDelay.get(i)(0, 0);

	//if (t_delay > 2000) {
	//break;
	//}
	//}

	//Vector<float, n_x> x0 = _xDelay.get(i);
	//Matrix<float, n_x, n_x> P0 = _PDelay.get(i);

	// residual
	Vector<float, n_y_gps> r = y - C * _x;
	Matrix<float, n_y_gps, n_y_gps> S_I = inv<float, 6>(C * _P * C.transpose() + R);

	// fault detection
	float beta = (r.transpose() * (S_I * r))(0, 0);

	if (beta > BETA_TABLE[n_y_gps]) {
		if (!_gpsFault) {
			mavlink_log_info(_mavlink_fd, "[lpe] gps fault, beta: %5.2f", double(beta));
			warnx("[lpe] gps fault, beta: %5.2f", double(beta));
			mavlink_log_info(_mavlink_fd, "[lpe] r: %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f",
					 double(r(0)),  double(r(1)), double(r(2)),
					 double(r(3)), double(r(4)), double(r(5)));
			mavlink_log_info(_mavlink_fd, "[lpe] S_I: %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f",
					 double(S_I(0, 0)),  double(S_I(1, 1)), double(S_I(2, 2)),
					 double(S_I(3, 3)),  double(S_I(4, 4)), double(S_I(5, 5)));
			warnx("[lpe] r: %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f",
			      double(r(0)),  double(r(1)), double(r(2)),
			      double(r(3)), double(r(4)), double(r(5)));
			warnx("[lpe] S_I: %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f",
			      double(S_I(0, 0)),  double(S_I(1, 1)), double(S_I(2, 2)),
			      double(S_I(3, 3)),  double(S_I(4, 4)), double(S_I(5, 5)));
			_gpsFault = FAULT_MINOR;
		}

	} else if (_gpsFault) {
		_gpsFault = FAULT_NONE;
		mavlink_log_info(_mavlink_fd, "[lpe] GPS OK");
		warnx("[lpe] GPS OK");
	}

	// kalman filter correction if no hard fault
	if (_gpsFault < FAULT_SEVERE) {
		Matrix<float, n_x, n_y_gps> K = _P * C.transpose() * S_I;
		_x += K * r;
		_P -= K * C * _P;
	}
}

void BlockLocalPositionEstimator::correctVision()
{

	Vector<float, 3> y;
	y.setZero();
	y(0) = _sub_vision_pos.get().x - _visionHome(0);
	y(1) = _sub_vision_pos.get().y - _visionHome(1);
	y(2) = _sub_vision_pos.get().z - _visionHome(2);
	_time_last_vision_p = _sub_vision_pos.get().timestamp_boot;

	// vision measurement matrix, measures position
	Matrix<float, n_y_vision, n_x> C;
	C.setZero();
	C(Y_vision_x, X_x) = 1;
	C(Y_vision_y, X_y) = 1;
	C(Y_vision_z, X_z) = 1;

	// noise matrix
	Matrix<float, n_y_vision, n_y_vision> R;
	R.setZero();
	R(Y_vision_x, Y_vision_x) = _vision_xy_stddev.get() * _vision_xy_stddev.get();
	R(Y_vision_y, Y_vision_y) = _vision_xy_stddev.get() * _vision_xy_stddev.get();
	R(Y_vision_z, Y_vision_z) = _vision_z_stddev.get() * _vision_z_stddev.get();

	// residual
	Matrix<float, n_y_vision, n_y_vision> S_I = inv<float, n_y_vision>((C * _P * C.transpose()) + R);
	Matrix<float, n_y_vision, 1> r = y - C * _x;

	// fault detection
	float beta = (r.transpose() * (S_I * r))(0, 0);

	if (beta > BETA_TABLE[n_y_vision]) {
		if (!_visionFault) {
			mavlink_log_info(_mavlink_fd, "[lpe] vision position fault, beta %5.2f", double(beta));
			warnx("[lpe] vision position fault, beta %5.2f", double(beta));
			_visionFault = FAULT_MINOR;
		}

		// trust less
		S_I = inv<float, n_y_vision>((C * _P * C.transpose()) + R * 10);

	} else if (_visionFault) {
		_visionFault = FAULT_NONE;
		mavlink_log_info(_mavlink_fd, "[lpe] vision position OK");
		warnx("[lpe] vision position OK");
	}

	// kalman filter correction if no fault
	if (_visionFault <  FAULT_SEVERE) {
		Matrix<float, n_x, n_y_vision> K = _P * C.transpose() * S_I;
		_x += K * r;
		_P -= K * C * _P;
	}
}

void BlockLocalPositionEstimator::correctMocap()
{
	// measure
	Vector<float, n_y_mocap> y;
	y.setZero();
	y(Y_mocap_x) = _sub_mocap.get().x - _mocapHome(0);
	y(Y_mocap_y) = _sub_mocap.get().y - _mocapHome(1);
	y(Y_mocap_z) = _sub_mocap.get().z - _mocapHome(2);
	_time_last_mocap = _sub_mocap.get().timestamp_boot;

	// mocap measurement matrix, measures position
	Matrix<float, n_y_mocap, n_x> C;
	C.setZero();
	C(Y_mocap_x, X_x) = 1;
	C(Y_mocap_y, X_y) = 1;
	C(Y_mocap_z, X_z) = 1;

	// noise matrix
	Matrix<float, n_y_mocap, n_y_mocap> R;
	R.setZero();
	float mocap_p_var = _mocap_p_stddev.get()* \
			    _mocap_p_stddev.get();
	R(Y_mocap_x, Y_mocap_x) = mocap_p_var;
	R(Y_mocap_y, Y_mocap_y) = mocap_p_var;
	R(Y_mocap_z, Y_mocap_z) = mocap_p_var;

	// residual
	Matrix<float, n_y_mocap, n_y_mocap> S_I = inv<float, n_y_mocap>((C * _P * C.transpose()) + R);
	Matrix<float, n_y_mocap, 1> r = y - C * _x;

	// fault detection
	float beta = (r.transpose() * (S_I * r))(0, 0);

	if (beta > BETA_TABLE[n_y_mocap]) {
		if (!_mocapFault) {
			mavlink_log_info(_mavlink_fd, "[lpe] mocap fault, beta %5.2f", double(beta));
			warnx("[lpe] mocap fault, beta %5.2f", double(beta));
			_mocapFault = FAULT_MINOR;
		}

		// trust less
		S_I = inv<float, n_y_mocap>((C * _P * C.transpose()) + R * 10);

	} else if (_mocapFault) {
		_mocapFault = FAULT_NONE;
		mavlink_log_info(_mavlink_fd, "[lpe] mocap OK");
		warnx("[lpe] mocap OK");
	}

	// kalman filter correction if no fault
	if (_mocapFault <  FAULT_SEVERE) {
		Matrix<float, n_x, n_y_mocap> K = _P * C.transpose() * S_I;
		_x += K * r;
		_P -= K * C * _P;
	}
}
