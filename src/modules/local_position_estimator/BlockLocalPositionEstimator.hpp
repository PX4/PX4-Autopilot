#pragma once

#include <px4_posix.h>
#include <controllib/blocks.hpp>
#include <mathlib/mathlib.h>
#include <systemlib/perf_counter.h>
#include <lib/geo/geo.h>

#ifdef USE_MATRIX_LIB
#include "matrix/Matrix.hpp"
using namespace matrix;
#else
#include <Eigen/Eigen>
using namespace Eigen;
#endif

// uORB Subscriptions
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vision_position_estimate.h>
#include <uORB/topics/att_pos_mocap.h>

// uORB Publications
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/estimator_status.h>

#define CBRK_NO_VISION_KEY	328754

using namespace control;

static const float GPS_DELAY_MAX = 0.5; // seconds
static const float HIST_STEP = 0.05; // 20 hz
static const size_t HIST_LEN = (GPS_DELAY_MAX / HIST_STEP);

enum fault_t {
	FAULT_NONE = 0,
	FAULT_MINOR,
	FAULT_SEVERE
};

enum sensor_t {
	SENSOR_BARO = 0,
	SENSOR_GPS,
	SENSOR_LIDAR,
	SENSOR_FLOW,
	SENSOR_SONAR,
	SENSOR_VISION,
	SENSOR_MOCAP
};

class BlockLocalPositionEstimator : public control::SuperBlock
{
//
// The purpose of this estimator is to provide a robust solution for
// indoor flight.
//
// dynamics:
//
//	x(+) = A * x(-) + B * u(+)
//	y_i = C_i*x
//
// kalman filter
//
//	E[xx'] = P
//	E[uu'] = W
//	E[y_iy_i'] = R_i
//
//	prediction
//		x(+|-) = A*x(-|-) + B*u(+)
//		P(+|-) = A*P(-|-)*A' + B*W*B'
//
//	correction
//		x(+|+) =  x(+|-) + K_i * (y_i - H_i * x(+|-) )
//
//
// input:
// 	ax, ay, az (acceleration NED)
//
// states:
// 	px, py, pz , ( position NED)
// 	vx, vy, vz ( vel NED),
// 	bx, by, bz ( TODO accelerometer bias)
// 	tz (TODO terrain altitude)
//
// measurements:
//
// 	sonar: pz (measured d*cos(phi)*cos(theta))
//
// 	baro: pz
//
// 	flow: vx, vy (flow is in body x, y frame)
//
// 	gps: px, py, pz, vx, vy, vz (flow is in body x, y frame)
//
// 	lidar: px (actual measured d*cos(phi)*cos(theta))
//
// 	vision: px, py, pz, vx, vy, vz
//
// 	mocap: px, py, pz
//
public:

	// constants
	enum {X_x = 0, X_y, X_z, X_vx, X_vy, X_vz, X_bx, X_by, X_bz, X_tz, n_x};
	enum {U_ax = 0, U_ay, U_az, n_u};
	enum {Y_baro_z = 0, n_y_baro};
	enum {Y_lidar_z = 0, n_y_lidar};
	enum {Y_flow_x = 0, Y_flow_y, n_y_flow};
	enum {Y_sonar_z = 0, n_y_sonar};
	enum {Y_gps_x = 0, Y_gps_y, Y_gps_z, Y_gps_vx, Y_gps_vy, Y_gps_vz, n_y_gps};
	enum {Y_vision_x = 0, Y_vision_y, Y_vision_z, n_y_vision};
	enum {Y_mocap_x = 0, Y_mocap_y, Y_mocap_z, n_y_mocap};
	enum {POLL_FLOW, POLL_SENSORS, POLL_PARAM, n_poll};

	BlockLocalPositionEstimator();
	void update();
	virtual ~BlockLocalPositionEstimator();

private:
	// prevent copy and assignment
	BlockLocalPositionEstimator(const BlockLocalPositionEstimator &);
	BlockLocalPositionEstimator operator=(const BlockLocalPositionEstimator &);

	// methods
	// ----------------------------
	void initP();

	// predict the next state
	void predict();

	// correct the state prediction with a measurement
	void correctBaro();
	void correctGps();
	void correctLidar();
	void correctFlow();
	void correctSonar();
	void correctVision();
	void correctMocap();

	// sensor timeout checks
	void checkTimeouts();

	// sensor initialization
	void updateHome();
	void initBaro();
	void initGps();
	void initLidar();
	void initSonar();
	void initFlow();
	void initVision();
	void initMocap();

	// publications
	void publishLocalPos();
	void publishGlobalPos();
	void publishEstimatorStatus();

	// attributes
	// ----------------------------

	// subscriptions
	uORB::Subscription<vehicle_status_s> _sub_status;
	uORB::Subscription<actuator_armed_s> _sub_armed;
	uORB::Subscription<vehicle_control_mode_s> _sub_control_mode;
	uORB::Subscription<vehicle_attitude_s> _sub_att;
	uORB::Subscription<vehicle_attitude_setpoint_s> _sub_att_sp;
	uORB::Subscription<optical_flow_s> _sub_flow;
	uORB::Subscription<sensor_combined_s> _sub_sensor;
	uORB::Subscription<parameter_update_s> _sub_param_update;
	uORB::Subscription<manual_control_setpoint_s> _sub_manual;
	uORB::Subscription<home_position_s> _sub_home;
	uORB::Subscription<vehicle_gps_position_s> _sub_gps;
	uORB::Subscription<vision_position_estimate_s> _sub_vision_pos;
	uORB::Subscription<att_pos_mocap_s> _sub_mocap;
	uORB::Subscription<distance_sensor_s> *_distance_subs[ORB_MULTI_MAX_INSTANCES];
	uORB::Subscription<distance_sensor_s> *_sub_lidar;
	uORB::Subscription<distance_sensor_s> *_sub_sonar;

	// publications
	uORB::Publication<vehicle_local_position_s> _pub_lpos;
	uORB::Publication<vehicle_global_position_s> _pub_gpos;
	uORB::Publication<estimator_status_s> _pub_est_status;

	// map projection
	struct map_projection_reference_s _map_ref;

	// general parameters
	BlockParamInt  _integrate;

	// sonar parameters
	BlockParamFloat  _sonar_z_stddev;
	BlockParamFloat  _sonar_z_offset;

	// lidar parameters
	BlockParamFloat  _lidar_z_stddev;
	BlockParamFloat  _lidar_z_offset;

	// accel parameters
	BlockParamFloat  _accel_xy_stddev;
	BlockParamFloat  _accel_z_stddev;

	// baro parameters
	BlockParamFloat  _baro_stddev;

	// gps parameters
	BlockParamFloat  _gps_delay;
	BlockParamFloat  _gps_xy_stddev;
	BlockParamFloat  _gps_z_stddev;
	BlockParamFloat  _gps_vxy_stddev;
	BlockParamFloat  _gps_vz_stddev;
	BlockParamFloat  _gps_eph_max;

	// vision parameters
	BlockParamFloat  _vision_xy_stddev;
	BlockParamFloat  _vision_z_stddev;
	BlockParamInt    _no_vision;

	// mocap parameters
	BlockParamFloat  _mocap_p_stddev;

	// flow parameters
	BlockParamFloat  _flow_z_offset;
	BlockParamFloat  _flow_xy_stddev;
	//BlockParamFloat  _flow_board_x_offs;
	//BlockParamFloat  _flow_board_y_offs;
	BlockParamInt    _flow_min_q;

	// process noise
	BlockParamFloat  _pn_p_noise_density;
	BlockParamFloat  _pn_v_noise_density;
	BlockParamFloat  _pn_b_noise_density;
	BlockParamFloat  _pn_t_noise_density;

	// flow gyro filter
	BlockHighPass _flow_gyro_x_high_pass;
	BlockHighPass _flow_gyro_y_high_pass;

	// stats
	BlockStats<float, 1> _baroStats;
	BlockStats<float, 1> _sonarStats;
	BlockStats<float, 1> _lidarStats;
	BlockStats<float, 1> _flowQStats;
	BlockStats<float, 3> _visionStats;
	BlockStats<float, 3> _mocapStats;
	BlockStats<double, 3> _gpsStats;

	// delay blocks
	BlockDelay<float, n_x, 1, HIST_LEN> _xDelay;
	BlockDelay<uint64_t, 1, 1, HIST_LEN> _tDelay;

	// misc
	px4_pollfd_struct_t _polls[3];
	uint64_t _timeStamp;
	uint64_t _time_last_hist;
	uint64_t _time_last_xy;
	uint64_t _time_last_z;
	uint64_t _time_last_tz;
	uint64_t _time_last_flow;
	uint64_t _time_last_baro;
	uint64_t _time_last_gps;
	uint64_t _time_last_lidar;
	uint64_t _time_last_sonar;
	uint64_t _time_last_vision_p;
	uint64_t _time_last_mocap;
	orb_advert_t _mavlink_log_pub;

	// initialization flags
	bool _baroInitialized;
	bool _gpsInitialized;
	bool _lidarInitialized;
	bool _sonarInitialized;
	bool _flowInitialized;
	bool _visionInitialized;
	bool _mocapInitialized;

	// reference altitudes
	float _altHome;
	bool _altHomeInitialized;
	float _baroAltHome;
	float _gpsAltHome;
	Vector3f _visionHome;
	Vector3f _mocapHome;

	// flow integration
	float _flowX;
	float _flowY;
	float _flowMeanQual;

	// referene lat/lon
	double _gpsLatHome;
	double _gpsLonHome;

	// status
	bool _canEstimateXY;
	bool _canEstimateZ;
	bool _canEstimateT;
	bool _canEstimateGlobal;
	bool _xyTimeout;
	bool _zTimeout;
	bool _tzTimeout;

	// sensor faults
	fault_t _baroFault;
	fault_t _gpsFault;
	fault_t _lidarFault;
	fault_t _flowFault;
	fault_t _sonarFault;
	fault_t _visionFault;
	fault_t _mocapFault;

	// performance counters
	perf_counter_t _loop_perf;
	perf_counter_t _interval_perf;
	perf_counter_t _err_perf;

	// state space
	Vector<float, n_x>  _x; // state vector
	Vector<float, n_u>  _u; // input vector
	Matrix<float, n_x, n_x>  _P; // state covariance matrix
};
