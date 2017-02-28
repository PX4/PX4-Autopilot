#include "../BlockLocalPositionEstimator.hpp"
#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>

extern orb_advert_t mavlink_log_pub;

// required number of samples for sensor
// to initialize
static const int 		REQ_BEACON_INIT_COUNT = 10;
static const uint32_t 	BEACON_TIMEOUT =   5000000; // 2.0 s
static const float  	BEACON_MAX_INIT_STD =   0.3f; // meters

void BlockLocalPositionEstimator::beaconInit()
{
	// this is only called once the beacon position estimator has seen the beacon, so we can just say initialized
	mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] beacon init");
	_beaconInitialized = true;
}

int BlockLocalPositionEstimator::beaconMeasure(Vector<float, n_y_beacon> &y)
{
	// TODO check valid flags
	// get data depending on mode
	if (_bestParams.mode == BeaconMode::Moving) {
		// don't have to do anything in this mode
		return -OK; // TODO what should be returned?

	} else if (_bestParams.mode == BeaconMode::Stationary) {
		if (_sub_beacon_position.get().rel_vel_valid) {
			y(0) = _sub_beacon_position.get().vx_rel;
			y(1) = _sub_beacon_position.get().vy_rel;

		} else {
			return -OK;
		}

		return OK;

	} else if (_bestParams.mode == BeaconMode::KnownLocation) {
		// TODO
		return OK;
	}

	return OK;
}

void BlockLocalPositionEstimator::beaconCorrect()
{
	// get data depending on mode
	if (_bestParams.mode == BeaconMode::Moving) {
		// don't have to do anything in this mode
		return;
	}

	// measure
	Vector<float, n_y_beacon> y;

	if (beaconMeasure(y) != OK) { return; }

	// Perform correction depeding on what we know about the beacon
	if (_bestParams.mode == BeaconMode::Stationary) {
		// we know the beacon is stationary, use measured beacon velocity to improve vehicle velocity
		// calculate covariance
		float cov_vx = _sub_beacon_position.get().cov_vx_rel;
		float cov_vy = _sub_beacon_position.get().cov_vy_rel;

		if (cov_vx < 1.0e-3f || cov_vy < 1.0e-3f) {
			PX4_WARN("fallback cov");
			// use sensor value only if reasoanble
			cov_vx = 0.1; // TODO make parameter
			cov_vy = 0.1; // TODO make parameter
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
		// _pub_innov.get().hagl_innov = r(0);
		// _pub_innov.get().hagl_innov_var = R(0, 0);

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
			if (_beaconFault < FAULT_MINOR) {
				_beaconFault = FAULT_MINOR;
				mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] beacon fault,  beta %5.2f", double(beta));
				PX4_WARN("beacon fault,  beta %5.2f", double(beta));
			}

			// abort correction
			return;

		} else if (_beaconFault) {
			_beaconFault = FAULT_NONE;
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] beacon OK");
			PX4_WARN("[lpe] beacon OK");
		}

		// kalman filter correction if no fault
		if (_beaconFault < fault_lvl_disable) {
			Matrix<float, n_x, n_y_beacon> K =
				_P * C.transpose() * S_I;
			Vector<float, n_x> dx = K * r;
			correctionLogic(dx);
			// PX4_WARN("correcting with %f %f", dx(0), dx(1));
			_x += dx;
			_P -= K * C * _P;
		}

	} else if (_bestParams.mode == BeaconMode::KnownLocation) {
		// float cov_x = _sub_beacon_position.get().cov_x_rel;
		// float cov_y = _sub_beacon_position.get().cov_y_rel;
	}

}

void BlockLocalPositionEstimator::beaconCheckTimeout()
{
	if (_timeStamp - _time_last_beacon > BEACON_TIMEOUT) {
		if (_beaconInitialized) {
			_beaconInitialized = false;
			// _beaconStats.reset();
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] beacon timeout ");
		}
	}
}
