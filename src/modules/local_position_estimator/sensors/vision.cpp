#include "../BlockLocalPositionEstimator.hpp"
#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>

extern orb_advert_t mavlink_log_pub;

// required number of samples for sensor
// to initialize

// this is a vision based position measurement so we assume as soon as we get one
// measurement it is initialized, we also don't want to deinitialize it because
// this will throw away a correction before it starts using the data so we
// set the timeout to 10 seconds
static const uint32_t 		REQ_VISION_INIT_COUNT = 1;
static const uint32_t 		VISION_TIMEOUT =    10000000;	// 10 s

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
		mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] vision position init: "
					     "%5.2f %5.2f %5.2f m std %5.2f %5.2f %5.2f m",
					     double(_visionStats.getMean()(0)),
					     double(_visionStats.getMean()(1)),
					     double(_visionStats.getMean()(2)),
					     double(_visionStats.getStdDev()(0)),
					     double(_visionStats.getStdDev()(1)),
					     double(_visionStats.getStdDev()(2)));
		_sensorTimeout &= ~SENSOR_VISION;
		_sensorFault &= ~SENSOR_VISION;

		if (!_altOriginInitialized) {
			_altOriginInitialized = true;
			_altOrigin = 0;
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
	_time_last_vision_p = _sub_vision_pos.get().timestamp;
	return OK;
}

void BlockLocalPositionEstimator::visionCorrect()
{
	// measure
	Vector<float, n_y_vision> y;

	if (visionMeasure(y) != OK) { return; }

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

	// vision delayed x
	uint8_t i_hist = 0;

	if (getDelayPeriods(_vision_delay.get(), &i_hist)  < 0) { return; }

	Vector<float, n_x> x0 = _xDelay.get(i_hist);

	// residual
	Matrix<float, n_y_vision, n_y_vision> S_I = inv<float, n_y_vision>((C * _P * C.transpose()) + R);
	Matrix<float, n_y_vision, 1> r = y - C * x0;

	// fault detection
	float beta = (r.transpose() * (S_I * r))(0, 0);

	if (beta > BETA_TABLE[n_y_vision]) {
		if (!(_sensorFault & SENSOR_VISION)) {
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] vision position fault, beta %5.2f", double(beta));
			_sensorFault |= SENSOR_VISION;
		}

	} else if (_sensorFault & SENSOR_VISION) {
		_sensorFault &= ~SENSOR_VISION;
		mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] vision position OK");
	}

	// kalman filter correction if no fault
	if (!(_sensorFault & SENSOR_VISION)) {
		Matrix<float, n_x, n_y_vision> K = _P * C.transpose() * S_I;
		Vector<float, n_x> dx = K * r;
		_x += dx;
		_P -= K * C * _P;
	}
}

void BlockLocalPositionEstimator::visionCheckTimeout()
{
	if (_timeStamp - _time_last_vision_p > VISION_TIMEOUT) {
		if (!(_sensorTimeout & SENSOR_VISION)) {
			_sensorTimeout |= SENSOR_VISION;
			_visionStats.reset();
			mavlink_log_critical(&mavlink_log_pub, "[lpe] vision position timeout ");
		}
	}
}
