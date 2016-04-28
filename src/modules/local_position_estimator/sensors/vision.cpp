#include "../BlockLocalPositionEstimator.hpp"
#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>

extern orb_advert_t mavlink_log_pub;

// required number of samples for sensor
// to initialize
static const uint32_t 		REQ_VISION_INIT_COUNT = 20;
static const uint32_t 		VISION_TIMEOUT =    500000;	// 0.5 s

void BlockLocalPositionEstimator::visionInit()
{
	// measure
	Vector<float, n_y_vision> y;

	if (visionMeasure(y) != OK) {
		_visionStats.reset();
		return;
	}

	// increament sums for mean
	if (_visionStats.getCount() > REQ_VISION_INIT_COUNT) {
		_visionHome = _visionStats.getMean();
		mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] vision position init: "
					     "%5.2f %5.2f %5.2f m std %5.2f %5.2f %5.2f m",
					     double(_visionStats.getMean()(0)),
					     double(_visionStats.getMean()(1)),
					     double(_visionStats.getMean()(2)),
					     double(_visionStats.getStdDev()(0)),
					     double(_visionStats.getStdDev()(1)),
					     double(_visionStats.getStdDev()(2)));
		_visionInitialized = true;
		_visionFault = FAULT_NONE;

		if (!_altHomeInitialized) {
			_altHomeInitialized = true;
			_altHome = _visionHome(2);
		}
	}
}

int BlockLocalPositionEstimator::visionMeasure(Vector<float, n_y_vision> &y)
{
	y.setZero();
	y(Y_vision_x) = _sub_vision_pos.get().x;
	y(Y_vision_y) = _sub_vision_pos.get().y;
	y(Y_vision_z) = _sub_vision_pos.get().z;
	_visionStats.update(y);
	_time_last_vision_p = _sub_vision_pos.get().timestamp_boot;
	return OK;
}

void BlockLocalPositionEstimator::visionCorrect()
{
	// measure
	Vector<float, n_y_vision> y;

	if (visionMeasure(y) != OK) { return; }

	// make measurement relative to home
	y -= _visionHome;

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
		if (_visionFault < FAULT_MINOR) {
			//mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] vision position fault, beta %5.2f", double(beta));
			_visionFault = FAULT_MINOR;
		}

	} else if (_visionFault) {
		_visionFault = FAULT_NONE;
		//mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] vision position OK");
	}

	// kalman filter correction if no fault
	if (_visionFault <  fault_lvl_disable) {
		Matrix<float, n_x, n_y_vision> K = _P * C.transpose() * S_I;
		_x += K * r;
		_P -= K * C * _P;
	}
}

void BlockLocalPositionEstimator::visionCheckTimeout()
{
	if (_timeStamp - _time_last_vision_p > VISION_TIMEOUT) {
		if (_visionInitialized) {
			_visionInitialized = false;
			_visionStats.reset();
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] vision position timeout ");
		}
	}
}
