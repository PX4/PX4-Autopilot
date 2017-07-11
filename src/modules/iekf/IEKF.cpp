/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "IEKF.hpp"
#include "constants.hpp"
#include <px4_posix.h>
#include <drivers/drv_hrt.h>

float condMaxDefault = 100;
float betaMaxDefault = 10;

IEKF::IEKF() :
	SuperBlock(nullptr, "IEKF"),
	// blocks
	//_aglLP(this, "POS_LP"),
	//_baroLP(this, "BARO_LP"),
	//_accelLP(this, "ACCEL_LP"),
	//_magLP(this, "MAG_LP"),
	// sensors, rates set in updateParam, default to 0
	_sensorAccel("accel", betaMaxDefault, condMaxDefault, 0),
	_sensorMag("mag", betaMaxDefault, condMaxDefault, 0),
	_sensorBaro("baro", betaMaxDefault, condMaxDefault, 0),
	_sensorGps("gps", betaMaxDefault, condMaxDefault, 0),
	_sensorAirspeed("airspeed", betaMaxDefault, condMaxDefault, 0),
	_sensorFlow("flow", betaMaxDefault, condMaxDefault, 0),
	_sensorSonar("sonar", betaMaxDefault, condMaxDefault, 0),
	_sensorLidar("lidar", betaMaxDefault, condMaxDefault, 0),
	_sensorVision("vision", betaMaxDefault, condMaxDefault, 0),
	_sensorMocap("mocap", betaMaxDefault, condMaxDefault, 0),
	_sensorLand("land_detected", betaMaxDefault, condMaxDefault, 0),
	// subscriptions
	_subImu(ORB_ID(sensor_combined), 1000 / 1000, 0),
	_subGps(ORB_ID(vehicle_gps_position), 1000 / 10),
	_subAirspeed(ORB_ID(airspeed), 1000 / 10, 0),
	_subFlow(ORB_ID(optical_flow), 1000 / 50, 0),
	_subDistance(ORB_ID(distance_sensor), 1000 / 20, 0),
	_subVision(ORB_ID(vehicle_vision_position), 1000 / 10, 0),
	_subMocap(ORB_ID(att_pos_mocap), 1000 / 10, 0),
	_subLand(ORB_ID(vehicle_land_detected), 1000 / 10, 0),
	_subParamUpdate(ORB_ID(parameter_update), 1000 / 10, 0),
	// publications
	_pubAttitude(ORB_ID(vehicle_attitude), -1),
	_pubLocalPosition(ORB_ID(vehicle_local_position), -1),
	_pubGlobalPosition(ORB_ID(vehicle_global_position), -1),
	_pubControlState(ORB_ID(control_state), -1),
	_pubEstimatorStatus(ORB_ID(estimator_status), -1),

	// data
	_x0(),
	_xMin(),
	_xMax(),
	_P0Diag(),
	_x(),
	_P(),
	_u(),
	_g_n(0, 0, -g),
	_origin(),
	_baroAsl(0),
	_baroOffset(0),
	_landed(true),
	_freefall(false),
	_gpsUSec(0),
	_attitudeInitialized(false),
	_stateTimestamp(),
	_covarianceTimestamp(),
	_imuLowRateIndex(0),
	_accelSaturated(false),
	_gyroSaturated(false),
	_A(), _Q(), _dxe(), _dP(),
	_innov(),
	_innovStd(),
	_overruns(),
	// params
	_gyro_nd(this, "GYRO_ND"),
	_gyro_rw_nd(this, "GYRO_RW_ND"),
	_gyro_rw_ct(this, "GYRO_RW_CT"),
	_accel_nd(this, "ACCEL_ND"),
	_accel_rw_nd(this, "ACCEL_RW_ND"),
	_accel_rw_ct(this, "ACCEL_RW_CT"),
	_baro_nd(this, "BARO_ND"),
	_baro_rw_nd(this, "BARO_RW_ND"),
	_baro_rw_ct(this, "BARO_RW_CT"),
	_mag_nd(this, "MAG_ND"),
	_mag_rw_nd(this, "MAG_RW_ND"),
	_mag_rw_ct(this, "MAG_RW_CT"),
	_mag_decl_deg(this, "MAG_DECL"),
	_gps_xy_nd(this, "GPS_XY_ND"),
	_gps_z_nd(this, "GPS_Z_ND"),
	_gps_vxy_nd(this, "GPS_VXY_ND"),
	_gps_vz_nd(this, "GPS_VZ_ND"),
	_vision_xy_nd(this, "VIS_XY_ND"),
	_vision_z_nd(this, "VIS_Z_ND"),
	_vision_vxy_nd(this, "VIS_VXY_ND"),
	_vision_vz_nd(this, "VIS_VZ_ND"),
	_flow_nd(this, "FLOW_ND"),
	_lidar_nd(this, "LIDAR_ND"),
	_sonar_nd(this, "SONAR_ND"),
	_land_vxy_nd(this, "LAND_VXY_ND"),
	_land_vz_nd(this, "LAND_VZ_ND"),
	_land_agl_nd(this, "LAND_AGL_ND"),
	_pn_xy_nd(this, "PN_XY_ND"),
	_pn_vxy_nd(this, "PN_VXY_ND"),
	_pn_z_nd(this, "PN_Z_ND"),
	_pn_vz_nd(this, "PN_VZ_ND"),
	_pn_rot_nd(this, "PN_ROT_ND"),
	_pn_t_asl_nd(this, "PN_T_ND"),
	_pn_t_asl_s_nd(this, "PN_TS_ND"),
	_rate_accel(this, "RATE_ACCEL"),
	_rate_mag(this, "RATE_MAG"),
	_rate_baro(this, "RATE_BARO"),
	_rate_gps(this, "RATE_GPS"),
	_rate_airspeed(this, "RATE_AIRSPD"),
	_rate_flow(this, "RATE_FLOW"),
	_rate_sonar(this, "RATE_SONAR"),
	_rate_lidar(this, "RATE_LIDAR"),
	_rate_vision(this, "RATE_VISION"),
	_rate_mocap(this, "RATE_MOCAP"),
	_rate_land(this, "RATE_LAND")
{
	// for quaterinons we bound at 2
	// so it has a chance to
	// do a renormalization first
	_xMin(X::q_nb_0) = -1.1;
	_xMin(X::q_nb_1) = -1.1;
	_xMin(X::q_nb_2) = -1.1;
	_xMin(X::q_nb_3) = -1.1;
	_xMin(X::vel_N) = -100;
	_xMin(X::vel_E) = -100;
	_xMin(X::vel_D) = -100;
	_xMin(X::gyro_bias_bX) = -1;
	_xMin(X::gyro_bias_bY) = -1;
	_xMin(X::gyro_bias_bZ) = -1;
	_xMin(X::accel_bias_bX) = -10;
	_xMin(X::accel_bias_bY) = -10;
	_xMin(X::accel_bias_bZ) = -10;
	_xMin(X::pos_N) = -1e30;
	_xMin(X::pos_E) = -1e30;
	_xMin(X::asl) = -1e30;
	_xMin(X::terrain_asl) = -1e30;
	_xMin(X::baro_bias) = -1e4;
	//_xMin(X::wind_N) = -100;
	//_xMin(X::wind_E) = -100;
	//_xMin(X::wind_D) = -100;

	_xMax(X::q_nb_0) = 1.1;
	_xMax(X::q_nb_1) = 1.1;
	_xMax(X::q_nb_2) = 1.1;
	_xMax(X::q_nb_3) = 1.1;
	_xMax(X::vel_N) = 100;
	_xMax(X::vel_E) = 100;
	_xMax(X::vel_D) = 100;
	_xMax(X::gyro_bias_bX) = 1;
	_xMax(X::gyro_bias_bY) = 1;
	_xMax(X::gyro_bias_bZ) = 1;
	_xMax(X::accel_bias_bX) = 10;
	_xMax(X::accel_bias_bY) = 10;
	_xMax(X::accel_bias_bZ) = 10;
	_xMax(X::pos_N) = 1e30;
	_xMax(X::pos_E) = 1e30;
	_xMax(X::asl) = 1e30;
	_xMax(X::terrain_asl) = 1e30;
	_xMax(X::baro_bias) = 1e4;
	//_xMax(X::wind_N) = 100;
	//_xMax(X::wind_E) = 100;
	//_xMax(X::wind_D) = 100;

	// initialize state
	_x0(X::q_nb_0) = 1;
	_x0(X::q_nb_1) = 0;
	_x0(X::q_nb_2) = 0;
	_x0(X::q_nb_3) = 0;
	setX(_x0);

	// initialize covariance
	_P0Diag(Xe::rot_N) = 1e-3;
	_P0Diag(Xe::rot_E) = 1e-3;
	_P0Diag(Xe::rot_D) = 1e-3;
	_P0Diag(Xe::vel_N) = 1;
	_P0Diag(Xe::vel_E) = 1;
	_P0Diag(Xe::vel_D) = 1;
	_P0Diag(Xe::gyro_bias_N) = 1e-2;
	_P0Diag(Xe::gyro_bias_E) = 1e-2;
	_P0Diag(Xe::gyro_bias_D) = 1e-2;
	_P0Diag(Xe::accel_bias_N) = 1e-2;
	_P0Diag(Xe::accel_bias_E) = 1e-2;
	_P0Diag(Xe::accel_bias_D) = 1e-2;
	_P0Diag(Xe::pos_N) = 1;
	_P0Diag(Xe::pos_E) = 1;
	_P0Diag(Xe::asl) = 1;
	_P0Diag(Xe::terrain_asl) = 1;
	_P0Diag(Xe::baro_bias) = 1;
	//_P0Diag(Xe::wind_N) = 0;
	//_P0Diag(Xe::wind_E) = 0;
	//_P0Diag(Xe::wind_D) = 0;
	setP(diag(_P0Diag));

	updateParams();
}

void IEKF::update()
{
	// polls
	px4_pollfd_struct_t polls[1];

	// wait for a sensor update, check for exit condition every 100 ms
	polls[0].fd = _subImu.getHandle();
	polls[0].events = POLLIN;
	int ret = px4_poll(polls, 1, 100);

	if (ret < 0) {
		return;
	}

	// callbacks
	// TODO: make callbacks a uORB class arg
	if (_subImu.updated()) {
		sensor_combined_s msg;
		_subImu.update(&msg);
		callbackImu(&msg);
	}

	if (_subGps.updated()) {
		vehicle_gps_position_s msg;
		_subGps.update(&msg);
		correctGps(&msg);
	}

	if (_subAirspeed.updated()) {
		airspeed_s msg;
		_subAirspeed.update(&msg);
		correctAirspeed(&msg);
	}

	if (_subFlow.updated()) {
		optical_flow_s msg;
		_subFlow.update(&msg);
		correctFlow(&msg);
	}

	if (_subDistance.updated()) {
		distance_sensor_s msg;
		_subDistance.update(&msg);
		correctDistance(&msg);
	}

	if (_subVision.updated()) {
		vehicle_local_position_s msg;
		_subVision.update(&msg);
		correctVision(&msg);
	}

	if (_subMocap.updated()) {
		att_pos_mocap_s msg;
		_subMocap.update(&msg);
		correctMocap(&msg);
	}

	if (_subLand.updated()) {
		vehicle_land_detected_s msg;
		_subLand.update(&msg);
		callbackLand(&msg);
	}

	if (_subParamUpdate.updated()) {
		updateParams();
	}

	publish();
}

Vector<float, X::n> IEKF::dynamics(float t, const Vector<float, X::n> &x, const Vector<float, U::n> &u) const
{
	Vector<float, X::n> dx;
	Quatf q_nb(x(X::q_nb_0), x(X::q_nb_1), x(X::q_nb_2), x(X::q_nb_3));
	Vector3f a_b(u(U::accel_bX), u(U::accel_bY), u(U::accel_bZ));
	Vector3f a_bias_b(x(X::accel_bias_bX), x(X::accel_bias_bY), x(X::accel_bias_bZ));
	Vector3f a_b_corrected = a_b - a_bias_b;
	Vector3f a_n = q_nb.conjugate(a_b_corrected);
	Vector3f as_n = a_n - _g_n;
	Vector3f gyro_bias_b(x(X::gyro_bias_bX), x(X::gyro_bias_bY), x(X::gyro_bias_bZ));
	Vector3f omega_nb_b(u(U::omega_nb_bX), u(U::omega_nb_bY), u(U::omega_nb_bZ));
	Vector3f omega_nb_b_corrected = omega_nb_b - gyro_bias_b;
	Quatf dq_nb = q_nb * Quatf(0, omega_nb_b_corrected(0),
				   omega_nb_b_corrected(1), omega_nb_b_corrected(2)) * 0.5f;

	dx(X::q_nb_0) = dq_nb(0);
	dx(X::q_nb_1) = dq_nb(1);
	dx(X::q_nb_2) = dq_nb(2);
	dx(X::q_nb_3) = dq_nb(3);

	//PX4_INFO("as_n: %10.4f %10.4f %10.4f",
	//double(as_n(0)),
	//double(as_n(1)),
	//double(as_n(2)));

	//PX4_INFO("V_n: %10.4f %10.4f %10.4f",
	//double(_x(X::vel_N)),
	//double(_x(X::vel_E)),
	//double(_x(X::vel_D)));

	//PX4_INFO("wind: %10.4f %10.4f %10.4f",
	//double(_x(X::wind_N)),
	//double(_x(X::wind_E)),
	//double(_x(X::wind_D)));

	// params
	dx(X::gyro_bias_bX) = -_x(X::gyro_bias_bX) / _gyro_rw_ct.get();
	dx(X::gyro_bias_bY) = -_x(X::gyro_bias_bY) / _gyro_rw_ct.get();
	dx(X::gyro_bias_bZ) = -_x(X::gyro_bias_bZ) / _gyro_rw_ct.get();

	dx(X::accel_bias_bX) = -_x(X::accel_bias_bX) / _accel_rw_ct.get();
	dx(X::accel_bias_bY) = -_x(X::accel_bias_bY) / _accel_rw_ct.get();
	dx(X::accel_bias_bZ) = -_x(X::accel_bias_bZ) / _accel_rw_ct.get();

	dx(X::vel_N) = as_n(0);
	dx(X::vel_E) = as_n(1);
	dx(X::vel_D) = as_n(2);

	dx(X::pos_N) = x(X::vel_N);
	dx(X::pos_E) = x(X::vel_E);
	dx(X::asl) = -x(X::vel_D);

	// want terrain dynamics to be static, so when out of range it keeps
	// last estimate and doesn't decay
	dx(X::terrain_asl) = 0;
	dx(X::baro_bias) = -x(X::baro_bias) / _baro_rw_ct.get();
	//dx(X::wind_N) = -_x(X::wind_N) / _wind_rw_ct.get();
	//dx(X::wind_E) = -_x(X::wind_E) / _wind_rw_ct.get();
	//dx(X::wind_D) = -_x(X::wind_D) / _wind_rw_ct.get();
	return dx;
}

void IEKF::callbackImu(const sensor_combined_s *msg)
{
	//PX4_INFO("gyro rate: %f Hz", double(1.0f / dt));
	Vector3f gyro_b(
		msg->gyro_rad[0],
		msg->gyro_rad[1],
		msg->gyro_rad[2]);

	Vector3f accel_b(
		msg->accelerometer_m_s2[0],
		msg->accelerometer_m_s2[1],
		msg->accelerometer_m_s2[2]);

	Vector3f mag_b(
		msg->magnetometer_ga[0],
		msg->magnetometer_ga[1],
		msg->magnetometer_ga[2]);

	//PX4_INFO("imu callback");
	_u(U::omega_nb_bX) = gyro_b(0);
	_u(U::omega_nb_bY) = gyro_b(1);
	_u(U::omega_nb_bZ) = gyro_b(2);
	_u(U::accel_bX) = accel_b(0);
	_u(U::accel_bY) = accel_b(1);
	_u(U::accel_bZ) = accel_b(2);

	// update gyro saturation
	if (gyro_b.norm() > gyro_saturation_thresh) {
		_gyroSaturated = true;

	} else {
		_gyroSaturated = false;
	}

	// update accel saturation
	if (accel_b.norm() > accel_saturation_thresh) {
		_accelSaturated = true;

	} else {
		_accelSaturated = false;
	}

	if (_attitudeInitialized) {

		predictState(msg);

		// set correciton deadline to 250 hz

		// max update rate of innert loops, 125 hz
		int lowRateCount = 5;

		// check if sensors are ready using row late cycle
		if (_imuLowRateIndex % lowRateCount == 0) {
			predictCovariance(msg);

		} else if (_imuLowRateIndex % lowRateCount == 1) {
			stateSpaceStateUpdate();

		} else if (_imuLowRateIndex % lowRateCount == 2) {
			correctAccel(msg);

		} else if (_imuLowRateIndex % lowRateCount == 3) {
			correctMag(msg);

		} else if (_imuLowRateIndex % lowRateCount == 4) {
			correctBaro(msg);
			correctLand(msg->timestamp);
		}

		float elapsedMillis = (hrt_absolute_time() - _stateTimestamp) / 1.0e3f;

		if (elapsedMillis > 10) {
			_overruns += 1;

			if (_overruns % 100 == 0) {
				PX4_WARN("late %10.4f msec, overruns: %d",
					 double(elapsedMillis), _overruns);
			}
		}

		_imuLowRateIndex++;

	} else {
		initializeAttitude(msg);
	}
}

void IEKF::updateParams()
{
	// update all block params
	SuperBlock::updateParams();
	_sensorMag.setRateMax(_rate_mag.get());
	_sensorFlow.setRateMax(_rate_flow.get());
	_sensorAccel.setRateMax(_rate_accel.get());
	_sensorBaro.setRateMax(_rate_baro.get());
	_sensorGps.setRateMax(_rate_gps.get());
	_sensorAirspeed.setRateMax(_rate_airspeed.get());
	_sensorLidar.setRateMax(_rate_lidar.get());
	_sensorVision.setRateMax(_rate_vision.get());
	_sensorMocap.setRateMax(_rate_mocap.get());
	_sensorLand.setRateMax(_rate_land.get());
	stateSpaceParamUpdate();
}

void IEKF::initializeAttitude(const sensor_combined_s *msg)
{
	PX4_INFO("initialize attitude");
	// return if no new mag data
	float dt = 0;
	uint64_t timestamp = msg->timestamp + msg->magnetometer_timestamp_relative;

	if (!_sensorMag.ready(timestamp, dt)) {
		PX4_INFO("mag not ready: %10.6g", double(dt));
		return;
	}

	// TODO handle freefall initialization (where accel norm is small and can't be used)
	Vector3f accel = Vector3f(msg->accelerometer_m_s2[0],
				  msg->accelerometer_m_s2[1],
				  msg->accelerometer_m_s2[2]);

	Vector3f mag = Vector3f(
			       msg->magnetometer_ga[0],
			       msg->magnetometer_ga[1],
			       msg->magnetometer_ga[2]).unit();

	Vector3f bz = -accel.unit(); // then only acceleration is likely opposing gravity
	Vector3f bx = (mag - mag.dot(bz) * bz).unit(); // project onto NE plane
	Vector3f by = bz.cross(bx).unit();

	Dcmf C_nb;
	C_nb.setRow(0, bx);
	C_nb.setRow(1, by);
	C_nb.setRow(2, bz);

	// account for magnetic declination
	Quatf q_nb = Dcmf(Dcmf(AxisAnglef(Vector3f(0, 0, 1), deg2radf * _mag_decl_deg.get())) * C_nb);

	_x(X::q_nb_0) = q_nb(0);
	_x(X::q_nb_1) = q_nb(1);
	_x(X::q_nb_2) = q_nb(2);
	_x(X::q_nb_3) = q_nb(3);

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			_P(Xe::gyro_bias_N + i, Xe::gyro_bias_N + j) = 0;

			if (i == j) {
				_P(Xe::rot_N + i, Xe::rot_N + j) = 0.1f;

			} else {
				_P(Xe::rot_N + i, Xe::rot_N + j) = 0;
			}
		}
	}

	_attitudeInitialized = true;

	Eulerf euler = q_nb;
	PX4_INFO("initial euler angles (deg) roll: %10.4f pitch: %10.4f yaw: %10.4f",
		 double(rad2degf * euler(0)),
		 double(rad2degf * euler(1)),
		 double(rad2degf * euler(2)));
}

void IEKF::predictState(const sensor_combined_s *msg)
{
	// calculate dt
	if (_stateTimestamp == 0) {
		_stateTimestamp = msg->timestamp;
		return;
	}

	float dt = (msg->timestamp - _stateTimestamp) / 1e6f;
	_stateTimestamp = msg->timestamp;

	// make sure dt is reasonable
	if (dt < 0 || dt > 0.1f) {
		return;
	}

	//PX4_INFO("predict state");

	// normalize quaternions if needed
	float qNorm = getQuaternionNB().norm();

	if (fabsf(qNorm - 1.0f) > 1e-3f) {
		PX4_INFO("normalizing quaternion, norm was %6.4f\n",
			 double(qNorm));
		normalizeQuaternion();
	}

	//PX4_INFO("prediction rate: %10.4g Hz", double(1/dt));

	// continuous time kalman filter prediction
	// integrate runge kutta 4th order
	// TODO move rk4 algorithm to matrixlib
	// https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
	float h = dt;
	Vector<float, X::n> k1, k2, k3, k4;
	k1 = dynamics(0, _x, _u);
	k2 = dynamics(h / 2, _x + k1 * h / 2, _u);
	k3 = dynamics(h / 2, _x + k2 * h / 2, _u);
	k4 = dynamics(h, _x + k3 * h, _u);
	Vector<float, X::n> dx = (k1 + k2 * 2 + k3 * 2 + k4) * (h / 6);

	//PX4_INFO("dx predict \n");
	//dx.print();

	//PX4_INFO("x");
	//_x.print();

	// euler integration
	//Vector<float, X::n> dx = dynamics(0, _x, _u) * dt;

	incrementX(dx);
}

void IEKF::predictCovariance(const sensor_combined_s *msg)
{
	// calculate dt
	if (_covarianceTimestamp == 0) {
		_covarianceTimestamp = msg->timestamp;
		return;
	}

	float dt = (msg->timestamp - _covarianceTimestamp) / 1e6f;
	_covarianceTimestamp = msg->timestamp;

	if (dt < 0 || dt > 0.5f) {
		return;
	}

	//PX4_INFO("predict covariance");

	// propgate covariance using euler integration
	// save stack space and cpu by doing this incrementally
	// P+ = P- + (A*P- + P-*A^T + Q)*dt;
	SquareMatrix<float, Xe::n> tmp = _A * _P;
	tmp += tmp.T();
	tmp += _Q;
	incrementP(tmp * dt);
}

Vector<float, X::n> IEKF::computeErrorCorrection(const Vector<float, Xe::n> &d_xe) const
{
	Vector<float, X::n> dx;

	Quatf q_nb = getQuaternionNB();
	Quatf d_q_nb = Quatf(0.0f,
			     d_xe(Xe::rot_N), d_xe(Xe::rot_E), d_xe(Xe::rot_D)) * q_nb;

	//PX4_INFO("d_q_nb");
	//d_q_nb.print();
	Vector3f d_gyro_bias_b = q_nb.conjugate_inversed(
					 Vector3f(d_xe(Xe::gyro_bias_N),
							 d_xe(Xe::gyro_bias_E),
							 d_xe(Xe::gyro_bias_D)));

	Vector3f d_accel_bias_b = q_nb.conjugate_inversed(
					  Vector3f(d_xe(Xe::accel_bias_N),
							  d_xe(Xe::accel_bias_E),
							  d_xe(Xe::accel_bias_D)));


	// linear term correction is the same
	// as the error correction
	dx(X::q_nb_0) += d_q_nb(0);
	dx(X::q_nb_1) += d_q_nb(1);
	dx(X::q_nb_2) += d_q_nb(2);
	dx(X::q_nb_3) += d_q_nb(3);
	dx(X::vel_N) += d_xe(Xe::vel_N);
	dx(X::vel_E) += d_xe(Xe::vel_E);
	dx(X::vel_D) += d_xe(Xe::vel_D);
	dx(X::gyro_bias_bX) += d_gyro_bias_b(0);
	dx(X::gyro_bias_bY) += d_gyro_bias_b(1);
	dx(X::gyro_bias_bZ) +=  d_gyro_bias_b(2);

	dx(X::accel_bias_bX) += d_accel_bias_b(0);
	dx(X::accel_bias_bY) += d_accel_bias_b(1);
	dx(X::accel_bias_bZ) += d_accel_bias_b(2);

	dx(X::pos_N) += d_xe(Xe::pos_N);
	dx(X::pos_E) += d_xe(Xe::pos_E);
	dx(X::asl) += d_xe(Xe::asl);
	dx(X::terrain_asl) += d_xe(Xe::terrain_asl);
	dx(X::baro_bias) += d_xe(Xe::baro_bias);
	//dx(X::wind_N) += d_xe(Xe::wind_N);
	//dx(X::wind_E) += d_xe(Xe::wind_E);
	//dx(X::wind_D) += d_xe(Xe::wind_D);

	correctionLogic(dx);
	return dx;
}


void IEKF::correctionLogic(Vector<float, X::n> &dx) const
{

	if (getLanded()) {
		//PX4_INFO("not updating position, landed, agl: %10.4f, landed: %d",
		//double(getAgl()), _landed);
		dx(X::pos_N) = 0;
		dx(X::pos_E) = 0;
	}

	if (!getPositionXYValid()) {
		//PX4_INFO("not updating position, xy not valid");
		dx(X::pos_N) = 0;
		dx(X::pos_E) = 0;
	}

	if (!getAltitudeValid()) {
		//PX4_INFO("not updating position, altitude not valid");
		dx(X::asl) = 0;
	}

	if (!getVelocityXYValid()) {
		//PX4_INFO("not updating velocity xy, not valid");
		dx(X::vel_N) = 0;
		dx(X::vel_E) = 0;
	}

	if (!getVelocityZValid()) {
		//PX4_INFO("not updating velocity z, not valid");
		dx(X::vel_D) = 0;
	}

	if (!getTerrainValid()) {
		//PX4_INFO("not updating terrain asl, not valid");
		dx(X::terrain_asl) = 0;
	}

	bool rotating = Vector3f(
				_u(U::omega_nb_bX),
				_u(U::omega_nb_bY),
				_u(U::omega_nb_bZ)).norm() > 30 * deg2radf;

	bool accelerating = Vector3f(_u(U::accel_bX),
				     _u(U::accel_bY),
				     _u(U::accel_bZ)).norm() > 1.2f * g;

	if (rotating || accelerating) {
		//PX4_INFO("not updating bias, rotating or accelerating");
		dx(X::gyro_bias_bX) = 0;
		dx(X::gyro_bias_bY) = 0;
		dx(X::gyro_bias_bZ) = 0;
		dx(X::accel_bias_bX) = 0;
		dx(X::accel_bias_bY) = 0;
		dx(X::accel_bias_bZ) = 0;
	}
}

void IEKF::boundP()
{
	for (int i = 0; i < Xe::n; i++) {
		// only operate on upper triangle, then copy to lower

		// don't allow NaN or large numbers
		for (int j = 0; j <= i; j++) {
			if (!PX4_ISFINITE(_P(i, j))) {
				PX4_WARN("P(%d, %d) NaN, resetting", i, j);

				if (i == j) {
					_P(i, j) = _P0Diag(i);

				} else {
					_P(i, j) = 0;
				}
			}

			if (_P(i, j) > 1e3f) {
				// upper bound
				_P(i, j) = 1e3f;

			} else if (_P(i, j) < -1e3f) {
				// lower bound
				_P(i, j) = -1e3f;
			}
		}

		// force non-negative diagonal
		if (_P(i, i) < 0) {

			//PX4_WARN("P(%d, %d) < 0, setting to 0", i, i, double(0));
			//_P(i, i) = 0;
			//for (int k = 0; k < Xe::n; k++) {
			//_P(i, k) = 0;
			//_P(k, i) = 0;
			//}


			// force PSD using cholesky decomposition
			// This decomposes the matrix into L * L.T
			// then multiplies it back to zero.
			// The matrix cholesky decomposition nulls
			// any negative components in the decomposition.
			// This isn't strickly the closest PSD matrix
			// to P, but it does the job of keeping the small
			// terms from going negative.
			PX4_WARN("P(%d, %d) < 0, restructuring", i, i, double(0));
			_P = cholesky(_P);
			_P *= _P.T();
		}

		// force symmetry, copy uppper triangle to lower
		for (int j = 0; j < i; j++) {
			_P(j, i) = _P(i, j);
		}
	}
}

void IEKF::boundX()
{
	// normalize quaternion
	Quatf q_nb = getQuaternionNB();

	if (fabsf(q_nb.norm() - 1.0f) > 1e-3f) {
		PX4_INFO("normalizing quaternion, norm was %6.4f\n", double(q_nb.norm()));
		normalizeQuaternion();
	}

	// saturate
	for (int i = 0; i < X::n; i++) {
		if (!PX4_ISFINITE(_x(i))) {
			PX4_WARN("x(%d) NaN, setting to %10.4f", i, double(_x0(i)));
			_x(i) = _x0(i);
		}

		if (_x(i) < _xMin(i)) {
			PX4_WARN("x(%d) < lower bound, saturating", i);
			_x(i) = _xMin(i);

		} else if (_x(i) > _xMax(i)) {
			PX4_WARN("x(%d) > upper bound, saturating", i);
			_x(i) = _xMax(i);
		}
	}
}

void IEKF::publish()
{
	if (!_attitudeInitialized) {
		return;
	}

	//PX4_INFO("x:");
	//_x.print();

	//PX4_INFO("P:");
	//_P.diag().print();

	float eph = sqrt(_P(Xe::pos_N, Xe::pos_N) + _P(Xe::pos_E, Xe::pos_E));
	float epv = _P(Xe::asl, Xe::asl);
	Quatf q_nb(
		_x(X::q_nb_0), _x(X::q_nb_1),
		_x(X::q_nb_2), _x(X::q_nb_3));
	// normalize published quaternion
	q_nb.normalize();
	Euler<float> euler_nb = q_nb;
	Vector3f a_b(_u(U::accel_bX), _u(U::accel_bY), _u(U::accel_bZ));
	Vector3f a_bias_b(_x(X::accel_bias_bX), _x(X::accel_bias_bY), _x(X::accel_bias_bZ));
	Vector3f a_b_corrected = a_b - a_bias_b;
	Vector3f a_n = q_nb.conjugate(a_b_corrected);
	uint64_t now = hrt_absolute_time();

	// predicted airspeed
	//Vector3f wind_n(_x(X::wind_N), _x(X::wind_E), _x(X::wind_D));
	Vector3f wind_n(0, 0, 0);
	Vector3f vel_n(_x(X::vel_N), _x(X::vel_E), _x(X::vel_D));
	Vector3f wind_rel_b = q_nb.conjugate_inversed(wind_n - vel_n);
	float airspeed = -wind_rel_b(0); // body -x component aligned with pitot tube

	// publish attitude
	{
		vehicle_attitude_s msg = {};
		msg.timestamp = now;

		msg.q[0] = q_nb(0);
		msg.q[1] = q_nb(1);
		msg.q[2] = q_nb(2);
		msg.q[3] = q_nb(3);
		msg.rollspeed = _u(U::omega_nb_bX) - _x(X::gyro_bias_bX);
		msg.pitchspeed = _u(U::omega_nb_bY) - _x(X::gyro_bias_bY);
		msg.yawspeed = _u(U::omega_nb_bZ) - _x(X::gyro_bias_bZ);
		_pubAttitude.update(&msg);
	}

	// publish local position
	{
		vehicle_local_position_s msg = {};
		msg.timestamp = now;
		msg.xy_valid = getPositionXYValid();
		msg.z_valid = getAltitudeValid();
		msg.v_xy_valid = getVelocityXYValid();
		msg.v_z_valid = getVelocityZValid();
		msg.x = _x(X::pos_N);
		msg.y = _x(X::pos_E);
		msg.z = -getAltAboveOrigin();
		msg.delta_xy[0] = 0;
		msg.delta_xy[1] = 0;
		msg.delta_z = 0;
		msg.vx = _x(X::vel_N);
		msg.vy = _x(X::vel_E);
		msg.vz = _x(X::vel_D);
		msg.delta_vxy[0] = 0;
		msg.delta_vxy[1] = 0;
		msg.delta_vz = 0;
		msg.xy_reset_counter = 0;
		msg.z_reset_counter = 0;
		msg.vxy_reset_counter = 0;
		msg.vz_reset_counter = 0;
		msg.yaw = euler_nb(2);
		msg.xy_global = _origin.xyInitialized();
		msg.z_global = _origin.altInitialized();
		msg.ref_timestamp = _origin.getXYTimestamp();
		msg.ref_lat = _origin.getLatDeg();
		msg.ref_lon = _origin.getLonDeg();
		msg.ref_alt = _x(X::terrain_asl);
		msg.dist_bottom = getAgl();
		msg.dist_bottom_rate = -_x(X::vel_D);
		msg.surface_bottom_timestamp = now;
		msg.dist_bottom_valid = getAglValid();
		msg.eph = eph;
		msg.epv = epv;
		_pubLocalPosition.update(&msg);
	}

	// publish global position
	if (_origin.xyInitialized()
	    && _origin.altInitialized()
	    && getVelocityXYValid()
	    && getVelocityZValid()) {
		double lat_deg = 0;
		double lon_deg = 0;
		_origin.northEastToLatLon(_x(X::pos_N), _x(X::pos_E), lat_deg, lon_deg);
		//PX4_INFO("alt %10.4f m", double(alt_m));
		vehicle_global_position_s msg = {};
		msg.timestamp = now;
		msg.time_utc_usec = _gpsUSec;
		msg.lat = lat_deg;
		msg.lon = lon_deg;
		msg.alt = _x(X::asl);
		msg.delta_lat_lon[0] = 0;
		msg.delta_lat_lon[1] = 0;
		msg.delta_alt = 0;
		msg.lat_lon_reset_counter = 0;
		msg.alt_reset_counter = 0;
		msg.vel_n = _x(X::vel_N);
		msg.vel_e = _x(X::vel_E);
		msg.vel_d = _x(X::vel_D);
		msg.yaw = euler_nb(2);
		msg.eph = eph;
		msg.epv = epv;
		msg.terrain_alt = _x(X::terrain_asl);
		msg.terrain_alt_valid = getTerrainValid();
		msg.dead_reckoning = false;
		msg.pressure_alt = _baroAsl;
		_pubGlobalPosition.update(&msg);
	}

	// publish control state
	{
		// specific acceleration
		control_state_s msg = {};
		msg.timestamp = now;
		msg.x_acc = a_n(0);
		msg.y_acc = a_n(1);
		msg.z_acc = a_n(2);
		msg.x_vel = _x(X::vel_N);
		msg.y_vel = _x(X::vel_E);
		msg.z_vel = _x(X::vel_D);
		msg.x_pos = _x(X::pos_N);
		msg.y_pos = _x(X::pos_E);
		msg.z_pos = -getAltAboveOrigin();
		msg.airspeed = airspeed;
		msg.airspeed_valid = true;
		msg.vel_variance[0] = _P(Xe::vel_N, Xe::vel_N);
		msg.vel_variance[1] = _P(Xe::vel_E, Xe::vel_E);
		msg.vel_variance[2] = _P(Xe::vel_D, Xe::vel_D);
		msg.pos_variance[0] = _P(Xe::pos_N, Xe::pos_N);
		msg.pos_variance[1] = _P(Xe::pos_E, Xe::pos_E);
		msg.pos_variance[2] = _P(Xe::asl, Xe::asl);
		msg.q[0] = _x(X::q_nb_0);
		msg.q[1] = _x(X::q_nb_1);
		msg.q[2] = _x(X::q_nb_2);
		msg.q[3] = _x(X::q_nb_3);
		msg.delta_q_reset[0] = 0;
		msg.delta_q_reset[1] = 0;
		msg.delta_q_reset[2] = 0;
		msg.delta_q_reset[3] = 0;
		msg.quat_reset_counter = 0;
		msg.roll_rate = _u(U::omega_nb_bX) - _x(X::gyro_bias_bX);
		msg.pitch_rate = _u(U::omega_nb_bY) - _x(X::gyro_bias_bY);
		msg.yaw_rate = _u(U::omega_nb_bZ) - _x(X::gyro_bias_bZ);
		msg.horz_acc_mag = 0;
		_pubControlState.update(&msg);
	}

	// estimator status
	{
		estimator_status_s msg = {};
		msg.timestamp = now;
		msg.vibe[0] = 0; // TODO
		msg.vibe[1] = 0; // TODO
		msg.vibe[2] = 0; // TODO
		msg.n_states = X::n;

		for (int i = 0; i < X::n; i++) {
			msg.states[i] = _x(i);
		}

		for (int i = 0; i < Xe::n; i++) {
			msg.covariances[i] = _P(i, i);
		}

		// XXX
		// this isn't really general and is
		// tailored to EKF2 so just dumping
		// data in the best field names available
		msg.gps_check_fail_flags = 0; // TODO
		msg.control_mode_flags = 0; // TODO
		msg.filter_fault_flags = 0; // TODO
		msg.pos_horiz_accuracy = eph;
		msg.pos_vert_accuracy = epv;
		msg.innovation_check_flags = 0; // TODO
		msg.mag_test_ratio = _sensorMag.getBeta();
		msg.vel_test_ratio = _sensorFlow.getBeta();
		msg.pos_test_ratio = _sensorGps.getBeta();
		msg.hgt_test_ratio = _sensorAccel.getBeta();
		msg.tas_test_ratio = _sensorAirspeed.getBeta();
		msg.hagl_test_ratio = _sensorLidar.getBeta();
		msg.solution_status_flags = 0; // TODO
		_pubEstimatorStatus.update(&msg);
	}

}

void IEKF::normalizeQuaternion()
{
	Quatf q = getQuaternionNB();
	q.normalize();
	_x(X::q_nb_0) = q(0);
	_x(X::q_nb_1) = q(1);
	_x(X::q_nb_2) = q(2);
	_x(X::q_nb_3) = q(3);
}

void IEKF::stateSpaceParamUpdate()
{
	//update A
	{
		// derivative of rotation error is -0.5 * gyro bias
		_A(Xe::rot_N, Xe::Xe::gyro_bias_N) = -0.5;
		_A(Xe::rot_E, Xe::Xe::gyro_bias_E) = -0.5;
		_A(Xe::rot_D, Xe::Xe::gyro_bias_D) = -0.5;

		// derivative of position is velocity
		_A(Xe::pos_N, Xe::vel_N) = 1;
		_A(Xe::pos_E, Xe::vel_E) = 1;
		_A(Xe::asl, Xe::vel_D) = -1;

		// derivative of terrain alt is zero

		// derivative of baro bias
		_A(Xe::baro_bias, Xe::baro_bias) = -1 / _baro_rw_ct.get();

		// derivative of gyro bias
		_A(Xe::gyro_bias_N, Xe::gyro_bias_N) = -1 / _gyro_rw_ct.get();
		_A(Xe::gyro_bias_E, Xe::gyro_bias_E) = -1 / _gyro_rw_ct.get();
		_A(Xe::gyro_bias_D, Xe::gyro_bias_D) = -1 / _gyro_rw_ct.get();

		// derivative of accel bias
		_A(Xe::accel_bias_N, Xe::accel_bias_N) = -1 / _accel_rw_ct.get();
		_A(Xe::accel_bias_E, Xe::accel_bias_E) = -1 / _accel_rw_ct.get();
		_A(Xe::accel_bias_D, Xe::accel_bias_D) = -1 / _accel_rw_ct.get();
	}

	// update Q
	{
		// variances
		float accel_var_rw = _accel_rw_nd.get() * _accel_rw_nd.get();
		float gyro_var_rw = _gyro_rw_nd.get() * _gyro_rw_nd.get();
		float baro_var_rw = _baro_rw_nd.get() * _baro_rw_nd.get();
		float pos_var_xy = _pn_xy_nd.get() * _pn_xy_nd.get();
		float pos_var_z = _pn_z_nd.get() * _pn_z_nd.get();
		float rot_var = _gyro_nd.get() * _gyro_nd.get() +  \
				_pn_rot_nd.get() * _pn_rot_nd.get();
		float vel_var_xy = _accel_nd.get() * _accel_nd.get() + \
				   _pn_vxy_nd.get() * _pn_vxy_nd.get();
		float vel_var_z = _accel_nd.get() * _accel_nd.get() + \
				  _pn_vz_nd.get() * _pn_vz_nd.get();
		float groundSpeedSq = getGroundVelocity().dot(getGroundVelocity());
		float terrain_var_asl = _pn_t_asl_s_nd.get() * _pn_t_asl_s_nd.get() * groundSpeedSq
					+ _pn_t_asl_nd.get() * _pn_t_asl_nd.get() ;

		_Q(Xe::rot_N, Xe::rot_N) = rot_var;
		_Q(Xe::rot_E, Xe::rot_E) = rot_var;
		_Q(Xe::rot_D, Xe::rot_D) = rot_var;
		_Q(Xe::vel_N, Xe::vel_N) = vel_var_xy;
		_Q(Xe::vel_E, Xe::vel_E) = vel_var_xy;
		_Q(Xe::vel_D, Xe::vel_D) = vel_var_z;
		_Q(Xe::gyro_bias_N, Xe::gyro_bias_N) = gyro_var_rw;
		_Q(Xe::gyro_bias_E, Xe::gyro_bias_E) = gyro_var_rw;
		_Q(Xe::gyro_bias_D, Xe::gyro_bias_D) = gyro_var_rw;
		_Q(Xe::accel_bias_N, Xe::accel_bias_N) = accel_var_rw;
		_Q(Xe::accel_bias_E, Xe::accel_bias_E) = accel_var_rw;
		_Q(Xe::accel_bias_D, Xe::accel_bias_D) = accel_var_rw;
		_Q(Xe::pos_N, Xe::pos_N) = pos_var_xy;
		_Q(Xe::pos_E, Xe::pos_E) = pos_var_xy;
		_Q(Xe::asl, Xe::asl) = pos_var_z;
		_Q(Xe::terrain_asl, Xe::terrain_asl) = terrain_var_asl;
		_Q(Xe::baro_bias, Xe::baro_bias) = baro_var_rw;
		//_Q(Xe::wind_N, Xe::wind_N) = 1e-2f;
		//_Q(Xe::wind_E, Xe::wind_E) = 1e-2f;
		//_Q(Xe::wind_D, Xe::wind_D) = 1e-2f;
	}
}

void IEKF::stateSpaceStateUpdate()
{
	//PX4_INFO("ccelcovariance prediction rate period: %10.4g", double(dt));
	Quatf q_nb = getQuaternionNB();

	// rotation rate
	Vector3f omega_nb_b_corrected = getAngularVelocityNBFrameB();


	// derivative of velocity
	Vector3f a_b_corrected = getAccelerationFrameB();
	Vector3f J_a_n = q_nb.conjugate(a_b_corrected);
	Matrix<float, 3, 3> a_tmp = -J_a_n.hat() * 2;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			_A(Xe::vel_N + i, Xe::rot_N + j) = a_tmp(i, j);
		}

		_A(Xe::vel_N + i, Xe::accel_bias_N + i) = -1.0f;
	}

	// derivative of gyro bias
	Vector3f J_omega_n = q_nb.conjugate(omega_nb_b_corrected);
	Matrix<float, 3, 3> g_tmp = J_omega_n.hat();

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			// this term isn't helpful, lets bias drift too much
			_A(Xe::gyro_bias_N + i, Xe::gyro_bias_N + j) = 0 * g_tmp(i, j);
		}
	}
}
