#include "../BlockLocalPositionEstimator.hpp"
#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>

extern orb_advert_t mavlink_log_pub;

static const uint64_t 	BEACON_TIMEOUT =   2000000; // [us]

void BlockLocalPositionEstimator::beaconInit()
{
	if (_beacon_mode.get() == Beacon_Moving) {
		// beacon is in moving mode, do not initialize
		return;
	}

	Vector<float, n_y_beacon> y;
	if (beaconMeasure(y) == OK)
	{
		mavlink_and_console_log_info(&mavlink_log_pub, "Beacon init");
		_beaconInitialized = true;
	}
}

int BlockLocalPositionEstimator::beaconMeasure(Vector<float, n_y_beacon> &y)
{
	if (_beacon_mode.get() == Beacon_Stationary) {
		if (_sub_beacon_position.get().rel_vel_valid) {
			y(0) = _sub_beacon_position.get().vx_rel;
			y(1) = _sub_beacon_position.get().vy_rel;
			_time_last_beacon = _timeStamp;
		} else {
			return -1;
		}

		return OK;

	}

	return -1;
}

void BlockLocalPositionEstimator::beaconCorrect()
{
	if (_beacon_mode.get() == Beacon_Moving) {
		// nothing to do in this mode
		return;
	}

	// measure
	Vector<float, n_y_beacon> y;

	if (beaconMeasure(y) != OK) { return; }

	// calculate covariance
	float cov_vx = _sub_beacon_position.get().cov_vx_rel;
	float cov_vy = _sub_beacon_position.get().cov_vy_rel;

	// use sensor value only if reasoanble
	if (cov_vx < _beacon_min_cov.get() || cov_vy < _beacon_min_cov.get()) {
		cov_vx = _beacon_min_cov.get();
		cov_vy = _beacon_min_cov.get();
	}

	// beacon measurement matrix and noise matrix
	Matrix<float, n_y_beacon, n_x> C;
	C.setZero();
	// residual = (y + vehicle velocity)
	// sign change because beacon velocitiy is -vehicle velocity
	C(Y_beacon_x, X_vx) = -1;
	C(Y_beacon_y, X_vy) = -1;

	// covariance matrix
	SquareMatrix<float, n_y_beacon> R;
	R.setZero();
	R(0, 0) = cov_vx;
	R(1, 1) = cov_vy;

	// residual
	Vector<float, n_y_beacon> r = y - C * _x;

	Matrix<float, n_y_beacon, n_y_beacon> S = C * _P * C.transpose() + R;

	// residual covariance, (inverse)
	Matrix<float, n_y_beacon, n_y_beacon> S_I =
		inv<float, n_y_beacon>(C * _P * C.transpose() + R);

	// XXX abuse flow innov for debugging
	_pub_innov.get().flow_innov[0] = r(0);
	_pub_innov.get().flow_innov[1] = r(1);
	_pub_innov.get().flow_innov_var[0] = S(0, 0);
	_pub_innov.get().flow_innov_var[1] = S(1, 1);

	// fault detection
	float beta = (r.transpose()  * (S_I * r))(0, 0);

	if (beta > BETA_TABLE[n_y_beacon]) {
		if (!_beaconFault) {
			_beaconFault = true;
			mavlink_and_console_log_info(&mavlink_log_pub, "Beacon fault,  beta %5.2f", double(beta));
		}

		// abort correction
		return;

	} else if (_beaconFault) {
		_beaconFault = false;
		mavlink_and_console_log_info(&mavlink_log_pub, "Beacon OK");
	}

	// kalman filter correction if no fault
	if (!_beaconFault) {
		Matrix<float, n_x, n_y_beacon> K =
			_P * C.transpose() * S_I;
		Vector<float, n_x> dx = K * r;
		_x += dx;
		_P -= K * C * _P;
	}

}

void BlockLocalPositionEstimator::beaconCheckTimeout()
{
	if (_beaconInitialized && _timeStamp - _time_last_beacon > BEACON_TIMEOUT) {
		_beaconInitialized = false;
		mavlink_and_console_log_info(&mavlink_log_pub, "Beacon timeout ");
	}
}
