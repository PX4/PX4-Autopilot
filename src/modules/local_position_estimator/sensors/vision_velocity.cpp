#include "../BlockLocalPositionEstimator.hpp"
#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>

extern orb_advert_t mavlink_log_pub;

static const uint32_t 		REQ_VISION_VEL_INIT_COUNT = 10;
static const uint32_t 		VISION_VELOCITY_TIMEOUT =    500000;	// 0.5 s

void BlockLocalPositionEstimator::visionVelocityInit()
{
	// measure
	Vector<float, n_y_vision_vel> y;

	if (visionVelocityMeasure(y) != OK) {
		_visionVelocityStats.reset();
		return;
	}

	// increament sums for mean
	if (_visionVelocityStats.getCount() > REQ_VISION_VEL_INIT_COUNT) {
		mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] vision velocity init: "
					     "%5.2f %5.2f %5.2f m std %5.2f %5.2f %5.2f m",
					     double(_visionVelocityStats.getMean()(0)),
					     double(_visionVelocityStats.getMean()(1)),
					     double(_visionVelocityStats.getMean()(2)),
					     double(_visionVelocityStats.getStdDev()(0)),
					     double(_visionVelocityStats.getStdDev()(1)),
					     double(_visionVelocityStats.getStdDev()(2)));
		_visionVelocityInitialized = true;
		_visionVelocityFault = FAULT_NONE;

		if (!_altOriginInitialized) {
			_altOriginInitialized = true;
			_altOrigin = 0;
		}
	}
}

int BlockLocalPositionEstimator::visionVelocityMeasure(Vector<float, n_y_vision> &y)
{
	y.setZero();
	y(Y_vision_vel_vx) = _sub_vision_speed.get().x;
	y(Y_vision_vel_vy) = _sub_vision_speed.get().y;
	y(Y_vision_vel_vz) = _sub_vision_speed.get().z;
	_visionVelocityStats.update(y);
	_time_last_vision_vel = _sub_vision_speed.get().timestamp;
	return OK;
}

void BlockLocalPositionEstimator::visionVelocityCorrect()
{
	// measure
	Vector<float, n_y_vision_vel> y;

	if (visionVelocityMeasure(y) != OK) { return; }

	// vision velocity measurement matrix, measures position
	Matrix<float, n_y_vision_vel, n_x> C;
	C.setZero();
	C(Y_vision_vel_vx, X_vx) = 1;
	C(Y_vision_vel_vy, X_vy) = 1;
	C(Y_vision_vel_vz, X_vz) = 1;

	// noise matrix
	Matrix<float, n_y_vision_vel, n_y_vision_vel> R;
	R.setZero();
	R(Y_vision_vel_vx, Y_vision_vel_vx) = _vision_vxy_stddev.get() * _vision_vxy_stddev.get();
	R(Y_vision_vel_vy, Y_vision_vel_vy) = _vision_vxy_stddev.get() * _vision_vxy_stddev.get();
	R(Y_vision_vel_vz, Y_vision_vel_vz) = _vision_vz_stddev.get() * _vision_vz_stddev.get();

	// residual
	Matrix<float, n_y_vision_vel, n_y_vision_vel> S_I = inv<float, n_y_vision>((C * _P * C.transpose()) + R);
	Matrix<float, n_y_vision_vel, 1> r = y - C * _x;

	// fault detection
	float beta = (r.transpose() * (S_I * r))(0, 0);

	if (beta > BETA_TABLE[n_y_vision_vel]) {
		if (_visionVelocityFault < FAULT_MINOR) {
			//mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] vision velocity fault, beta %5.2f", double(beta));
			_visionVelocityFault = FAULT_MINOR;
		}

	} else if (_visionVelocityFault) {
		_visionVelocityFault = FAULT_NONE;
		//mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] vision velocity OK");
	}

	// kalman filter correction if no fault
	if (_visionVelocityFault <  fault_lvl_disable) {
		Matrix<float, n_x, n_y_vision_vel> K = _P * C.transpose() * S_I;
		Vector<float, n_x> dx = K * r;
		correctionLogic(dx);
		_x += dx;
		_P -= K * C * _P;
	}
}

void BlockLocalPositionEstimator::visionVelocityCheckTimeout()
{
	if (_timeStamp - _time_last_vision_vel > VISION_VELOCITY_TIMEOUT) {
		if (_visionVelocityInitialized) {
			_visionVelocityInitialized = false;
			_visionVelocityStats.reset();
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] vision velocity timeout ");
		}
	}
}
