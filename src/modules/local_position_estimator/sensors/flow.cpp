#include "../BlockLocalPositionEstimator.hpp"
#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>

// mavlink pub
extern orb_advert_t mavlink_log_pub;

// required number of samples for sensor
// to initialize
static const uint32_t 		REQ_FLOW_INIT_COUNT = 10;
static const uint32_t 		FLOW_TIMEOUT = 1000000;	// 1 s

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

void BlockLocalPositionEstimator::flowDeinit()
{
	_flowInitialized = false;
	_flowQStats.reset();
}

int BlockLocalPositionEstimator::flowMeasure(Vector<float, n_y_flow> &y)
{
	// check for sane pitch/roll
	if (_eul(0) > 0.5f || _eul(1) > 0.5f) {
		return -1;
	}

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
	if (!_validTZ) {
		return -1;
	}

	matrix::Eulerf euler = matrix::Quatf(_sub_att.get().q);

	float d = agl() * cosf(euler.phi()) * cosf(euler.theta());

	// optical flow in x, y axis
	// TODO consider making flow scale a states of the kalman filter
	float flow_x_rad = _sub_flow.get().pixel_flow_x_integral * _flow_scale.get();
	float flow_y_rad = _sub_flow.get().pixel_flow_y_integral * _flow_scale.get();
	float dt_flow = _sub_flow.get().integration_timespan / 1.0e6f;

	if (dt_flow > 0.5f || dt_flow < 1.0e-6f) {
		return -1;
	}

	// angular rotation in x, y axis
	float gyro_x_rad = 0;
	float gyro_y_rad = 0;

	if (_flow_gyro_comp.get()) {
		gyro_x_rad = _flow_gyro_x_high_pass.update(
				     _sub_flow.get().gyro_x_rate_integral);
		gyro_y_rad = _flow_gyro_y_high_pass.update(
				     _sub_flow.get().gyro_y_rate_integral);
	}

	//warnx("flow x: %10.4f y: %10.4f gyro_x: %10.4f gyro_y: %10.4f d: %10.4f",
	//double(flow_x_rad), double(flow_y_rad), double(gyro_x_rad), double(gyro_y_rad), double(d));

	// compute velocities in camera frame using ground distance
	// assume camera frame is body frame
	Vector3f delta_b(
		-(flow_x_rad - gyro_x_rad)*d,
		-(flow_y_rad - gyro_y_rad)*d,
		0);

	// rotation of flow from body to nav frame
	Vector3f delta_n = _R_att * delta_b;

	// imporant to timestamp flow even if distance is bad
	_time_last_flow = _timeStamp;

	// measurement
	y(Y_flow_vx) = delta_n(0) / dt_flow;
	y(Y_flow_vy) = delta_n(1) / dt_flow;

	_flowQStats.update(Scalarf(_sub_flow.get().quality));

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
	C(Y_flow_vx, X_vx) = 1;
	C(Y_flow_vy, X_vy) = 1;

	SquareMatrix<float, n_y_flow> R;
	R.setZero();

	// polynomial noise model, found using least squares fit
	// h, h**2, v, v*h, v*h**2
	const float p[5] = {0.04005232f, -0.00656446f, -0.26265873f,  0.13686658f, -0.00397357f};

	// prevent extrapolation past end of polynomial fit by bounding independent variables
	float h = agl();
	float v = y.norm();
	const float h_min = 2.0f;
	const float h_max = 8.0f;
	const float v_min = 0.5f;
	const float v_max = 1.0f;

	if (h > h_max) {
		h = h_max;
	}

	if (h < h_min) {
		h = h_min;
	}

	if (v > v_max) {
		v = v_max;
	}

	if (v < v_min) {
		v = v_min;
	}

	// compute polynomial value
	float flow_vxy_stddev = p[0] * h + p[1] * h * h + p[2] * v + p[3] * v * h + p[4] * v * h * h;

	R(Y_flow_vx, Y_flow_vx) = flow_vxy_stddev * flow_vxy_stddev;
	R(Y_flow_vy, Y_flow_vy) = R(Y_flow_vx, Y_flow_vx);

	// residual
	Vector<float, 2> r = y - C * _x;
	_pub_innov.get().flow_innov[0] = r(0);
	_pub_innov.get().flow_innov[1] = r(1);
	_pub_innov.get().flow_innov_var[0] = R(0, 0);
	_pub_innov.get().flow_innov_var[1] = R(1, 1);

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
		Vector<float, n_x> dx = K * r;
		correctionLogic(dx);
		_x += dx;
		_P -= K * C * _P;

	}

}

void BlockLocalPositionEstimator::flowCheckTimeout()
{
	if (_timeStamp - _time_last_flow > FLOW_TIMEOUT) {
		if (_flowInitialized) {
			flowDeinit();
			mavlink_log_critical(&mavlink_log_pub, "[lpe] flow timeout ");
		}
	}
}
