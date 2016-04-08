#include "../BlockLocalPositionEstimator.hpp"
#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>

extern orb_advert_t mavlink_log_pub;

// required number of samples for sensor
// to initialize
static const int 		REQ_SONAR_INIT_COUNT = 10;
static const uint32_t 	SONAR_TIMEOUT =   1000000; // 1.0 s

void BlockLocalPositionEstimator::sonarInit()
{
	// measure
	Vector<float, n_y_sonar> y;

	if (sonarMeasure(y) != OK) {
		_sonarStats.reset();
		return;
	}

	// if finished
	if (_sonarStats.getCount() > REQ_SONAR_INIT_COUNT) {
		mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] sonar init "
					     "mean %d cm std %d cm",
					     int(100 * _sonarStats.getMean()(0)),
					     int(100 * _sonarStats.getStdDev()(0)));
		_sonarInitialized = true;
		_sonarFault = FAULT_NONE;
	}
}

int BlockLocalPositionEstimator::sonarMeasure(Vector<float, n_y_sonar> &y)
{
	// measure
	float d = _sub_sonar->get().current_distance + _sonar_z_offset.get();
	float eps = 0.01f;
	float min_dist = _sub_sonar->get().min_distance + eps;
	float max_dist = _sub_sonar->get().max_distance - eps;

	// check for bad data
	if (d > max_dist || d < min_dist) {
		return -1;
	}

	// update stats
	_sonarStats.update(Scalarf(d));
	_time_last_sonar = _timeStamp;
	y.setZero();
	y(0) = d *
	       cosf(_sub_att.get().roll) *
	       cosf(_sub_att.get().pitch);
	return OK;
}

void BlockLocalPositionEstimator::sonarCorrect()
{
	// measure
	Vector<float, n_y_sonar> y;

	if (sonarMeasure(y) != OK) { return; }

	// do not use sonar if lidar is active
	//if (_lidarInitialized && (_lidarFault < fault_lvl_disable)) { return; }

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

	// residual
	Vector<float, n_y_sonar> r = y - C * _x;

	// residual covariance, (inverse)
	Matrix<float, n_y_sonar, n_y_sonar> S_I =
		inv<float, n_y_sonar>(C * _P * C.transpose() + R);

	// fault detection
	float beta = (r.transpose()  * (S_I * r))(0, 0);

	if (beta > BETA_TABLE[n_y_sonar]) {
		if (_sonarFault < FAULT_MINOR) {
			_sonarFault = FAULT_MINOR;
			//mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] sonar fault,  beta %5.2f", double(beta));
		}

		// abort correction
		return;

	} else if (_sonarFault) {
		_sonarFault = FAULT_NONE;
		//mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] sonar OK");
	}

	// kalman filter correction if no fault
	if (_sonarFault < fault_lvl_disable) {
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

void BlockLocalPositionEstimator::sonarCheckTimeout()
{
	if (_timeStamp - _time_last_sonar > SONAR_TIMEOUT) {
		if (_sonarInitialized) {
			_sonarInitialized = false;
			_sonarStats.reset();
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] sonar timeout ");
		}
	}
}


