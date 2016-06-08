#include "../BlockLocalPositionEstimator.hpp"
#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>

// mavlink pub
extern orb_advert_t mavlink_log_pub;

// required number of samples for sensor
// to initialize
static const uint32_t 		REQ_FLOW_INIT_COUNT = 10;
static const uint32_t 		FLOW_TIMEOUT =   	1000000;	// 1 s

// minimum flow altitude
static const float flow_min_agl = 0.3;

void BlockLocalPositionEstimator::flowInit()
{
	// measure
	Vector<float, n_y_flow> y;

	if (flowMeasure(y) != OK) {
		_flowQStats.reset();
		return;
	}

	// if finished
	if (_flowQStats.getCount() > REQ_FLOW_INIT_COUNT) {
		mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] flow init: "
					     "quality %d std %d",
					     int(_flowQStats.getMean()(0)),
					     int(_flowQStats.getStdDev()(0)));
		_flowInitialized = true;
		_flowFault = FAULT_NONE;
	}
}

int BlockLocalPositionEstimator::flowMeasure(Vector<float, n_y_flow> &y)
{
	// check for agl
	if (agl() < flow_min_agl) {
		return -1;
	}

	// check quality
	float qual = _sub_flow.get().quality;

	if (qual < _flow_min_q.get()) {
		return -1;
	}

	// calculate range to center of image for flow
	float d = 0;

	if (_lidarInitialized && (_lidarFault < fault_lvl_disable)) {
		d = _sub_lidar->get().current_distance
		    + (_lidar_z_offset.get() - _flow_z_offset.get());

	} else if (_sonarInitialized && (_sonarFault < fault_lvl_disable)) {
		d = _sub_sonar->get().current_distance
		    + (_sonar_z_offset.get() - _flow_z_offset.get());

	} else {
		// no valid distance data
		return -1;
	}

	// check for global accuracy
	if (_gpsInitialized) {
		double  lat = _sub_gps.get().lat * 1.0e-7;
		double  lon = _sub_gps.get().lon * 1.0e-7;
		float px = 0;
		float py = 0;
		map_projection_project(&_map_ref, lat, lon, &px, &py);
		Vector2f delta(px - _flowX, py - _flowY);

		if (delta.norm() > 3) {
			mavlink_and_console_log_info(&mavlink_log_pub,
						     "[lpe] flow too far from GPS, disabled");
			_flowInitialized = false;
			return -1;
		}
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

	// measurement
	y(Y_flow_x) = _flowX;
	y(Y_flow_y) = _flowY;

	_flowQStats.update(Scalarf(_sub_flow.get().quality));

	// imporant to timestamp flow even if distance is bad
	_time_last_flow = _timeStamp;

	return OK;
}

void BlockLocalPositionEstimator::flowCorrect()
{
	// measure flow
	Vector<float, n_y_flow> y;

	if (flowMeasure(y) != OK) { return; }

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

	// residual
	Vector<float, 2> r = y - C * _x;

	// residual covariance, (inverse)
	Matrix<float, n_y_flow, n_y_flow> S_I =
		inv<float, n_y_flow>(C * _P * C.transpose() + R);

	// fault detection
	float beta = (r.transpose() * (S_I * r))(0, 0);

	if (beta > BETA_TABLE[n_y_flow]) {
		if (_flowFault < FAULT_MINOR) {
			//mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] flow fault,  beta %5.2f", double(beta));
			_flowFault = FAULT_MINOR;
		}

	} else if (_flowFault) {
		_flowFault = FAULT_NONE;
		//mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] flow OK");
	}

	if (_flowFault < fault_lvl_disable) {
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

void BlockLocalPositionEstimator::flowCheckTimeout()
{
	if (_timeStamp - _time_last_flow > FLOW_TIMEOUT) {
		if (_flowInitialized) {
			_flowInitialized = false;
			_flowQStats.reset();
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] flow timeout ");
		}
	}
}
