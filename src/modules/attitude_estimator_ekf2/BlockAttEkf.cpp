#include "BlockAttEkf.hpp"

#include <drivers/drv_hrt.h>

BlockAttEkf::BlockAttEkf() :
	SuperBlock(NULL, "AE2"),
	// publications
	_pub_control(ORB_ID(control_state), -1, &getPublications()),
	_pub_att(ORB_ID(vehicle_attitude), -1, &getPublications()),
	// subscriptions
	_sub_sensor(ORB_ID(sensor_combined), 0, 0, &getSubscriptions()),
	_sub_param_update(ORB_ID(parameter_update), 0, 0, &getSubscriptions()),
	_sub_pos(ORB_ID(vehicle_global_position), 0, 0, &getSubscriptions()),
	// params
	_omega_stddev(this, "SD_GYRO"),
	_bias_dot_stddev(this, "SD_DBIAS"),
	_mag_stddev(this, "SD_MAG"),
	_accel_stddev(this, "SD_ACCEL"),
	// derivatives
	_omega_dot_x(this, "ALPHA"),
	_omega_dot_y(this, "ALPHA"),
	_omega_dot_z(this, "ALPHA"),
	_a_x(this, "ACCEL_LP"),
	_a_y(this, "ACCEL_LP"),
	_a_z(this, "ACCEL_LP"),
	// states
	_q_nr(1, 0, 0, 0),
	_b_gyro(0, 0, 0),
	_omega_nr(0, 0, 0),
	_P(),
	// misc
	_timestamp(),
	_gyro_timestamp(),
	_mag_timestamp(),
	_accel_timestamp(),
	_polls()
{
	_P.setIdentity();
	_P *= 1e-10;
	_polls[0].fd = _sub_sensor.getHandle();
	_polls[0].events = POLLIN;
	_a_x.update(0);
	_a_y.update(0);
	_a_z.update(0);
}

void BlockAttEkf::update()
{
	// wait for new sensor combined data
	poll(_polls, 1, 100);

	// computed time since last run
	uint64_t now = hrt_absolute_time();
	float dt = (now - _timestamp) / 1.0e6f;

	if (dt < 1e-3f) {
		return;

	} else if (dt > 1e-1f) {
		_timestamp = now;
		return;
	}

	_timestamp = now;
	setDt(dt);

	// check topic status
	bool pos_update = _sub_pos.updated();
	bool param_update = _sub_param_update.updated();

	// get new data
	updateSubscriptions();

	// check for param updates
	if (param_update) {
		updateParams();
	}

	// update body acceleration estimate
	if (pos_update) {
		Dcmf C_nr(_q_nr);
		Vector3f v_n(_sub_pos.get().vel_n,
			_sub_pos.get().vel_e,
			_sub_pos.get().vel_d);
		Vector3f v_r = C_nr.T() * v_n;
		_a_x.update(v_r(0));
		_a_y.update(v_r(1));
		_a_z.update(v_r(2));
	}

	// prediction step
	uint64_t gyro_now = _sub_sensor.get().gyro_timestamp[0];

	if (gyro_now != _gyro_timestamp) {
		_gyro_timestamp = gyro_now;
		predict();
	}

	// correct mag
	uint64_t mag_now = _sub_sensor.get().magnetometer_timestamp[0];

	if (mag_now != _mag_timestamp) {
		_mag_timestamp = mag_now;
		correct_mag();
	}

	// correct accel
	uint64_t accel_now = _sub_sensor.get().accelerometer_timestamp[0];

	if (accel_now != _accel_timestamp) {
		_accel_timestamp = accel_now;
		correct_accel();
	}

	// normalize quaternion after corrections
	_q_nr.normalize();

	// publish attitude
	publish_attitude();
}

void BlockAttEkf::predict()
{
	// calculate mean of gyro over interval
	//Vector<3> y_gyro(&(_sub_sensor.get().gyro_integral_rad[0]));
	//float dt_gyro = _sub_sensor.get().gyro_integral_dt[0]/1.0e6f;
	//y_gyro /= dt_gyro;

	Vector3f y_gyro(&(_sub_sensor.get().gyro_rad_s[0]));

	// angular velocity of reference frame wrt nav frame
	_omega_nr = y_gyro - _b_gyro;
	float omega_x = _omega_nr(0);
	float omega_y = _omega_nr(1);
	float omega_z = _omega_nr(2);

	// update derivates
	_omega_dot_x.update(omega_x);
	_omega_dot_y.update(omega_y);
	_omega_dot_z.update(omega_z);

	SquareMatrix<float, 6> A;
	A.setZero();
	A(0, 1) = omega_z;
	A(0, 2) = -omega_y;
	A(0, 3) = 1;
	A(1, 0) = -omega_z;
	A(1, 2) = omega_x;
	A(1, 4) = 1;
	A(2, 0) = omega_y;
	A(2, 1) = -omega_x;
	A(2, 5) = 1;

	float bias_dot_stddev = _bias_dot_stddev.get();
	float omega_stddev = _omega_stddev.get();

	// process noise
	SquareMatrix<float, 6> Q;
	Q.setZero();
	Q(0, 0) = omega_stddev * omega_stddev;
	Q(1, 1) = omega_stddev * omega_stddev;;
	Q(2, 2) = omega_stddev * omega_stddev;
	Q(3, 3) = bias_dot_stddev * bias_dot_stddev;
	Q(4, 4) = bias_dot_stddev * bias_dot_stddev;
	Q(5, 5) = bias_dot_stddev * bias_dot_stddev;

	// propagate state
	// TODO, improve integration method, just using euler method
	_q_nr += _q_nr.derivative(_omega_nr) * getDt();

	// propagate covariance matrix
	_P += (A * _P + _P * A.T() + Q) * getDt();
}

void BlockAttEkf::correct_mag()
{
	Vector3f y(&(_sub_sensor.get().magnetometer_ga[0]));
	y.normalize();

	// quaternion
	float q_0 = _q_nr(0);
	float q_1 = _q_nr(1);
	float q_2 = _q_nr(2);
	float q_3 = _q_nr(3);

	// TODO set based on mag inc, dec
	float m_N = 0.375;
	float m_E = 0.071;
	float m_D = 0.928;

	// measurement matrix
	Matrix<float, 3, 6> C;
	C.setZero();
	C(0, 1) = -m_D * (q_0 * q_0 - q_1 * q_1 - q_2 * q_2 + q_3 * q_3) - m_E * (-2 * q_0 * q_1 + 2 * q_2 * q_3) - m_N *
		  (2 * q_0 * q_2 + 2 * q_1 * q_3);
	C(0, 2) = m_D * (2 * q_0 * q_1 + 2 * q_2 * q_3) + m_E * (q_0 * q_0 - q_1 * q_1 + q_2 * q_2 - q_3 * q_3) + m_N *
		  (-2 * q_0 * q_3 + 2 * q_1 * q_2);
	C(1, 0) = m_D * (q_0 * q_0 - q_1 * q_1 - q_2 * q_2 + q_3 * q_3) + m_E * (-2 * q_0 * q_1 + 2 * q_2 * q_3) + m_N *
		  (2 * q_0 * q_2 + 2 * q_1 * q_3);
	C(1, 2) = m_D * (2 * q_0 * q_2 - 2 * q_1 * q_3) + m_E * (-2 * q_0 * q_3 - 2 * q_1 * q_2) + m_N *
		  (-q_0 * q_0 - q_1 * q_1 + q_2 * q_2 + q_3 * q_3);
	C(2, 0) = -m_D * (2 * q_0 * q_1 + 2 * q_2 * q_3) - m_E * (q_0 * q_0 - q_1 * q_1 + q_2 * q_2 - q_3 * q_3) - m_N *
		  (-2 * q_0 * q_3 + 2 * q_1 * q_2);
	C(2, 1) = m_D * (-2 * q_0 * q_2 + 2 * q_1 * q_3) + m_E * (2 * q_0 * q_3 + 2 * q_1 * q_2) + m_N *
		  (q_0 * q_0 + q_1 * q_1 - q_2 * q_2 - q_3 * q_3);

	// measurement noise
	float mag_stddev = _mag_stddev.get();
	SquareMatrix<float, 3> R;
	R.setZero();
	R(0, 0) = mag_stddev * mag_stddev;
	R(1, 1) = mag_stddev * mag_stddev;
	R(2, 2) = mag_stddev * mag_stddev;

	// predicted measurement
	Vector3f y_h;
	y_h(0) = m_D * (-2 * q_0 * q_2 + 2 * q_1 * q_3) + m_E * (2 * q_0 * q_3 + 2 * q_1 * q_2) + m_N *
		 (q_0 * q_0 + q_1 * q_1 - q_2 * q_2 - q_3 * q_3);
	y_h(1) = m_D * (2 * q_0 * q_1 + 2 * q_2 * q_3) + m_E * (q_0 * q_0 - q_1 * q_1 + q_2 * q_2 - q_3 * q_3) + m_N *
		 (-2 * q_0 * q_3 + 2 * q_1 * q_2);
	y_h(2) = m_D * (q_0 * q_0 - q_1 * q_1 - q_2 * q_2 + q_3 * q_3) + m_E * (-2 * q_0 * q_1 + 2 * q_2 * q_3) + m_N *
		 (2 * q_0 * q_2 + 2 * q_1 * q_3);

	// kalman filter correction
	kalman_correction<6, 3>(C, R, y, y_h);
}

void BlockAttEkf::correct_accel()
{
	Vector3f y(&(_sub_sensor.get().accelerometer_m_s2[0]));

	// quaternion
	float q_0 = _q_nr(0);
	float q_1 = _q_nr(1);
	float q_2 = _q_nr(2);
	float q_3 = _q_nr(3);

	// TODO make sure matches g used in pos filter
	float g = 9.8;

	// measurement matrix
	Matrix<float, 3, 6> C;
	C.setZero();
	C(0, 1) = g * (q_0 * q_0 - q_1 * q_1 - q_2 * q_2 + q_3 * q_3);
	C(0, 2) = -g * (2 * q_0 * q_1 + 2 * q_2 * q_3);
	C(1, 0) = -g * (q_0 * q_0 - q_1 * q_1 - q_2 * q_2 + q_3 * q_3);
	C(1, 2) = -g * (2 * q_0 * q_2 - 2 * q_1 * q_3);
	C(2, 0) = g * (2 * q_0 * q_1 + 2 * q_2 * q_3);
	C(2, 1) = -g * (-2 * q_0 * q_2 + 2 * q_1 * q_3);

	// acceleration
	// TODO debug why a_x.. etc aren't helpful
	float a_x = 0; //_a_x.getState();
	float a_y = 0; //_a_y.getState();
	float a_z = 0; //_a_z.getState();

	// predicted measurement
	Vector3f y_h;
	y_h(0) = a_x - g * (-2 * q_0 * q_2 + 2 * q_1 * q_3);
	y_h(1) = a_y - g * (2 * q_0 * q_1 + 2 * q_2 * q_3);
	y_h(2) = a_z - g * (q_0 * q_0 - q_1 * q_1 - q_2 * q_2 + q_3 * q_3);

	// measurement noise
	float accel_stddev = _accel_stddev.get();
	Matrix<float, 3, 3> R;
	R.setZero();
	R(0, 0) = accel_stddev * accel_stddev;
	R(1, 1) = accel_stddev * accel_stddev;
	R(2, 2) = accel_stddev * accel_stddev;

	// kalman filter correction
	kalman_correction<6, 3>(C, R, y, y_h);
}

void BlockAttEkf::publish_attitude()
{
	Dcmf dcm(_q_nr);
	Eulerf euler(dcm);

	control_state_s &pub = _pub_control.get();
	pub.timestamp = hrt_absolute_time();
	for (int i=0; i<4; i++) {
		pub.q[i] = _q_nr(i);
	}
	pub.roll_rate = _omega_nr(0);
	pub.pitch_rate = _omega_nr(1);
	pub.yaw_rate = _omega_nr(2);
	_pub_control.update();

	vehicle_attitude_s &pub2 = _pub_att.get();
	pub2.timestamp = hrt_absolute_time();
	pub2.roll = euler(0);
	pub2.pitch = euler(1);
	pub2.yaw = euler(2);
	pub2.rollspeed = _omega_nr(0);
	pub2.pitchspeed = _omega_nr(1);
	pub2.yawspeed = _omega_nr(2);
	Vector3f y_accel(&(_sub_sensor.get().accelerometer_m_s2[0]));
	Vector3f g_comp = y_accel -
		Vector3f(_a_x.getState(), _a_y.getState(), _a_z.getState());
	for (int i=0; i<3; i++) {
		pub2.g_comp[i] = g_comp(i);
	}
	memcpy(&pub2.R[0], dcm.data(), sizeof(pub2.R));
	pub2.R_valid = true;
	pub2.rate_vibration = 0;
	pub2.accel_vibration = 0;
	pub2.mag_vibration = 0;
	_pub_att.update();
}

template<int n_x, int n_y>
void BlockAttEkf::kalman_correction(const Matrix<float, n_y, n_x> &C, const SquareMatrix<float, n_y> &R, const Vector<float, n_y> &y,
				    const Vector<float, n_y> &y_h)
{
	// kalman filter correction
	SquareMatrix<float, n_y> S = C * _P * C.T() + R;
	Vector<float, n_y> r = y - y_h;
	Matrix<float, n_x, n_y> K = _P * C.T() * S.I();
	Vector<float, n_x> d_x = K * r;

	// quaternion correction, multiplicative
	Quatf d_alpha(Eulerf(d_x(0), d_x(1), d_x(2)));
	_q_nr = _q_nr * d_alpha;

	// bias correction, additive
	Vector3f d_b(d_x(3), d_x(4), d_x(5));
	_b_gyro += d_b;

	// update covariance matrix
	SquareMatrix<float, n_x> I;
	I.setIdentity();
	_P = (I - K * C) * _P;
}
