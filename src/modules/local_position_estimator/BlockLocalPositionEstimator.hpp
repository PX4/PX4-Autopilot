#pragma once

#include <controllib/uorb/blocks.hpp>
#include <mathlib/mathlib.h>
#include <systemlib/perf_counter.h>
#include <lib/geo/geo.h>

// uORB Subscriptions
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/sensor_combined.h>
#include <drivers/drv_range_finder.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_gps_position.h>

// uORB Publications
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/filtered_bottom_flow.h>

using namespace control;

class BlockLocalPositionEstimator : public control::SuperBlock {
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
// 	x, y, z , ( position NED)
// 	vx, vy, vz ( vel NED),
// 	bx, by, bz ( TODO accelerometer bias)
//
// measurements:
//
// 	sonar: pz (measured d*cos(phi)*cos(theta))
//
// 	baro: pz
//
// 	flow: vx, vy (flow is in body x, y frame)
//
// 	gps: x, y, z, vx, vy, vz (flow is in body x, y frame)
//
// 	lidar: px (measured d*cos(phi)*cos(theta))
//
public:
	BlockLocalPositionEstimator();
	void update();
	virtual ~BlockLocalPositionEstimator();

private:
	// prevent copy and assignment
	BlockLocalPositionEstimator(const BlockLocalPositionEstimator &);
	BlockLocalPositionEstimator operator=(const BlockLocalPositionEstimator&);

	// constants
	static const uint8_t n_x = 6;
	static const uint8_t n_u = 3; // 3 accelerations
	static const uint8_t n_y_flow = 3;
	static const uint8_t n_y_baro = 1;
	static const uint8_t n_y_lidar = 1;
	static const uint8_t n_y_gps = 6;
	enum {X_x=0, X_y, X_z, X_vx, X_vy, X_vz}; //, X_bx, X_by, X_bz};
	enum {U_ax=0, U_ay, U_az};
	enum {Y_baro_z=0};
	enum {Y_lidar_z=0};
	enum {Y_flow_vx=0, Y_flow_vy, Y_flow_z};
	enum {Y_gps_x=0, Y_gps_y, Y_gps_z, Y_gps_vx, Y_gps_vy, Y_gps_vz};
	enum {POLL_FLOW, POLL_SENSORS, POLL_PARAM};
	enum {CH_LEFT, CH_RIGHT};

	// methods
	// ----------------------------
	
	// predict the next state
	void predict();

	// correct the state prediction wtih a measurement
	void correctFlow();
	void correctBaro();
	void correctLidar();
	void correctGps();

	// sensor initialization
	void updateHome();
	void initBaro();
	void initGps();
	void initLidar();
	void initFlow();

	// publications
	void publishLocalPos();
	void publishGlobalPos();
	void publishFilteredFlow();
	void updateParams();
	
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
	uORB::Subscription<range_finder_report> _sub_range_finder;
	uORB::Subscription<parameter_update_s> _sub_param_update;
	uORB::Subscription<manual_control_setpoint_s> _sub_manual;
	uORB::Subscription<home_position_s> _sub_home;
	uORB::Subscription<vehicle_gps_position_s> _sub_gps;

	// publications
	uORB::Publication<vehicle_local_position_s> _pub_lpos;
	uORB::Publication<vehicle_global_position_s> _pub_gpos;
	uORB::Publication<filtered_bottom_flow_s> _pub_filtered_flow;

	// map projection
	struct map_projection_reference_s _map_ref;

	// parameters
	BlockParamFloat  _flow_v_stddev;
	BlockParamFloat  _flow_z_stddev;

	BlockParamFloat  _lidar_z_stddev;

	BlockParamFloat  _accel_xy_stddev;
	BlockParamFloat  _accel_z_stddev;

	BlockParamFloat  _baro_stddev;

	BlockParamFloat  _gps_xy_stddev;
	BlockParamFloat  _gps_z_stddev;

	BlockParamFloat  _gps_vxy_stddev;
	BlockParamFloat  _gps_vz_stddev;

	// process noise
	BlockParamFloat  _pn_p_stddev;
	BlockParamFloat  _pn_v_stddev;

	struct pollfd _polls[3];
	uint64_t _timeStamp;
	uint64_t _time_last_flow;
	uint64_t _time_last_baro;
	uint64_t _time_last_gps;
	uint64_t _time_last_lidar;
	float _altHomeLast;

	int _mavlink_fd;

	// initialization flags
	bool _baroInitialized;
	bool _gpsInitialized;
	bool _lidarInitialized;
	bool _flowInitialized;

	// init counts
	int _baroInitCount;
	int _gpsInitCount;
	int _lidarInitCount;
	int _flowInitCount;

	// reference altitudes
	float _baroAltHome;
	float _gpsAltHome;
	float _lidarAltHome;
	float _flowAltHome;

	// referene lat/lon
	double _gpsLatHome;
	double _gpsLonHome;

	// sensor faults
	int _baroFault;
	int _gpsFault;
	int _lidarFault;
	int _sonarFault;

	perf_counter_t _loop_perf;
	perf_counter_t _interval_perf;
	perf_counter_t _err_perf;

	math::Matrix<n_x, n_x>  _A; // state dynamics matrix

	// accelerometer measurement and noise matrix
	math::Matrix<n_x, n_u>  _B; // input matrix
	math::Matrix<n_y_flow, n_y_flow> _R_accel;

	math::Matrix<n_x, n_x>  _Q; // process noise

	// flow measurement matrix and noise matrix
	math::Matrix<n_y_flow, n_x> _C_flow;
	math::Matrix<n_y_flow, n_y_flow> _R_flow;

	// baro measurement matrix and noise matrix
	math::Matrix<n_y_baro, n_x> _C_baro;
	math::Matrix<n_y_baro, n_y_baro> _R_baro;

	// lidar measurement matrix and noise matrix
	math::Matrix<n_y_lidar, n_x> _C_lidar;
	math::Matrix<n_y_lidar, n_y_lidar> _R_lidar;

	// gps measurement matrix and noise matrix
	math::Matrix<n_y_gps, n_x> _C_gps;
	math::Matrix<n_y_gps, n_y_gps> _R_gps;

	math::Vector<n_x>  _x; // state vecotr
	math::Vector<n_u>  _u; // input vector
	math::Matrix<n_x, n_x>  _P; // state covariance matrix
};
