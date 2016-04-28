#include "../BlockLocalPositionEstimator.hpp"
#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>

extern orb_advert_t mavlink_log_pub;

// required number of samples for sensor
// to initialize
static const uint32_t 		REQ_BARO_INIT_COUNT = 100;
static const uint32_t 		BARO_TIMEOUT =   	100000;	// 0.1 s

void BlockLocalPositionEstimator::baroInit()
{
	// measure
	Vector<float, n_y_baro> y;

	if (baroMeasure(y) != OK) {
		_baroStats.reset();
		return;
	}

	// if finished
	if (_baroStats.getCount() > REQ_BARO_INIT_COUNT) {
		_baroAltHome = _baroStats.getMean()(0);
		mavlink_and_console_log_info(&mavlink_log_pub,
					     "[lpe] baro init %d m std %d cm",
					     (int)_baroStats.getMean()(0),
					     (int)(100 * _baroStats.getStdDev()(0)));
		_baroInitialized = true;
		_baroFault = FAULT_NONE;

		if (!_altHomeInitialized) {
			_altHomeInitialized = true;
			_altHome = _baroAltHome;
		}
	}
}

int BlockLocalPositionEstimator::baroMeasure(Vector<float, n_y_baro> &y)
{
	//measure
	y.setZero();
	y(0) = _sub_sensor.get().baro_alt_meter[0];
	_baroStats.update(y);
	_time_last_baro = _timeStamp;
	return OK;
}

void BlockLocalPositionEstimator::baroCorrect()
{
	// measure
	Vector<float, n_y_baro> y;

	if (baroMeasure(y) != OK) { return; }

	// subtract baro home alt
	y -= _baroAltHome;

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
			//mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] baro fault, r %5.2f m, beta %5.2f",
			//double(r(0)), double(beta));
			_baroFault = FAULT_MINOR;
		}

	} else if (_baroFault) {
		_baroFault = FAULT_NONE;
		//mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] baro OK");
	}

	// kalman filter correction if no fault
	if (_baroFault < fault_lvl_disable) {
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

void BlockLocalPositionEstimator::baroCheckTimeout()
{
	if (_timeStamp - _time_last_baro > BARO_TIMEOUT) {
		if (_baroInitialized) {
			_baroInitialized = false;
			_baroStats.reset();
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] baro timeout ");
		}
	}
}
