#include "../BlockLocalPositionEstimator.hpp"
#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>

extern orb_advert_t mavlink_log_pub;

// required number of samples for sensor
// to initialize
//
static const uint32_t 		REQ_LAND_INIT_COUNT = 1;
static const uint32_t 		LAND_TIMEOUT =   1000000; // 1.0 s

void BlockLocalPositionEstimator::landInit()
{
	// measure
	Vector<float, n_y_land> y;

	if (landMeasure(y) != OK) {
		_landCount = 0;
	}

	// if finished
	if (_landCount > REQ_LAND_INIT_COUNT) {
		mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] land init");
		_landInitialized = true;
		_landFault = FAULT_NONE;
	}
}

int BlockLocalPositionEstimator::landMeasure(Vector<float, n_y_land> &y)
{
	_time_last_land = _timeStamp;
	y.setZero();
	_landCount += 1;
	return OK;
}

void BlockLocalPositionEstimator::landCorrect()
{
	// measure land
	Vector<float, n_y_land> y;

	if (landMeasure(y) != OK) { return; }

	// measurement matrix
	Matrix<float, n_y_land, n_x> C;
	C.setZero();
	// y = -(z - tz)
	C(Y_land_z, X_z) = -1; // measured altitude, negative down dir.
	C(Y_land_z, X_tz) = 1; // measured altitude, negative down dir.

	// use parameter covariance
	SquareMatrix<float, n_y_land> R;
	R.setZero();
	R(0, 0) = _land_z_stddev.get() * _land_z_stddev.get();

	// residual
	Matrix<float, n_y_land, n_y_land> S_I = inv<float, n_y_land>((C * _P * C.transpose()) + R);
	Vector<float, n_y_land> r = y - C * _x;
	_pub_innov.get().hagl_innov = r(0);
	_pub_innov.get().hagl_innov_var = R(0, 0);

	// fault detection
	float beta = (r.transpose() * (S_I * r))(0, 0);

	if (beta > BETA_TABLE[n_y_land]) {
		if (_landFault < FAULT_MINOR) {
			_landFault = FAULT_MINOR;
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] land fault,  beta %5.2f", double(beta));
		}

		// abort correction
		return;

	} else if (_landFault) { // disable fault if ok
		_landFault = FAULT_NONE;
		//mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] land OK");
	}

	// kalman filter correction if no fault
	if (_landFault < fault_lvl_disable) {
		Matrix<float, n_x, n_y_land> K = _P * C.transpose() * S_I;
		Vector<float, n_x> dx = K * r;
		correctionLogic(dx);
		_x += dx;
		_P -= K * C * _P;
	}
}

void BlockLocalPositionEstimator::landCheckTimeout()
{
	if (_timeStamp - _time_last_land > LAND_TIMEOUT) {
		if (_landInitialized) {
			_landInitialized = false;
			_landCount = 0;
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] land timeout ");
		}
	}
}
