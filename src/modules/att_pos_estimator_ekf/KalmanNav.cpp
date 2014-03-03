/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
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

/**
 * @file KalmanNav.cpp
 *
 * Kalman filter navigation code
 */

#include <poll.h>

#include "KalmanNav.hpp"
#include <systemlib/err.h>
#include <geo/geo.h>

// constants
// Titterton pg. 52
static const float omega = 7.2921150e-5f; // earth rotation rate, rad/s
static const float R0 = 6378137.0f; // earth radius, m
static const float g0 = 9.806f; // standard gravitational accel. m/s^2
static const int8_t ret_ok = 0; 		// no error in function
static const int8_t ret_error = -1; 	// error occurred

KalmanNav::KalmanNav(SuperBlock *parent, const char *name) :
	SuperBlock(parent, name),
	// ekf matrices
	F(9, 9),
	G(9, 6),
	P(9, 9),
	P0(9, 9),
	V(6, 6),
	// attitude measurement ekf matrices
	HAtt(4, 9),
	RAtt(4, 4),
	// position measurement ekf matrices
	HPos(6, 9),
	RPos(6, 6),
	// attitude representations
	C_nb(),
	q(),
	// subscriptions
	_sensors(&getSubscriptions(), ORB_ID(sensor_combined), 5), // limit to 200 Hz
	_gps(&getSubscriptions(), ORB_ID(vehicle_gps_position), 100), // limit to 10 Hz
	_param_update(&getSubscriptions(), ORB_ID(parameter_update), 1000), // limit to 1 Hz
	// publications
	_pos(&getPublications(), ORB_ID(vehicle_global_position)),
	_localPos(&getPublications(), ORB_ID(vehicle_local_position)),
	_att(&getPublications(), ORB_ID(vehicle_attitude)),
	// timestamps
	_pubTimeStamp(hrt_absolute_time()),
	_predictTimeStamp(hrt_absolute_time()),
	_attTimeStamp(hrt_absolute_time()),
	_outTimeStamp(hrt_absolute_time()),
	// frame count
	_navFrames(0),
	// miss counts
	_miss(0),
	// accelerations
	fN(0), fE(0), fD(0),
	// state
	phi(0), theta(0), psi(0),
	vN(0), vE(0), vD(0),
	lat(0), lon(0), alt(0),
	lat0(0), lon0(0), alt0(0),
	// parameters for ground station
	_vGyro(this, "V_GYRO"),
	_vAccel(this, "V_ACCEL"),
	_rMag(this, "R_MAG"),
	_rGpsVel(this, "R_GPS_VEL"),
	_rGpsPos(this, "R_GPS_POS"),
	_rGpsAlt(this, "R_GPS_ALT"),
	_rPressAlt(this, "R_PRESS_ALT"),
	_rAccel(this, "R_ACCEL"),
	_magDip(this, "ENV_MAG_DIP"),
	_magDec(this, "ENV_MAG_DEC"),
	_g(this, "ENV_G"),
	_faultPos(this, "FAULT_POS"),
	_faultAtt(this, "FAULT_ATT"),
	_attitudeInitialized(false),
	_positionInitialized(false),
	_attitudeInitCounter(0)
{
	using namespace math;

	// initial state covariance matrix
	P0 = Matrix::identity(9) * 0.01f;
	P = P0;

	// initial state
	phi = 0.0f;
	theta = 0.0f;
	psi = 0.0f;
	vN = 0.0f;
	vE = 0.0f;
	vD = 0.0f;
	lat = 0.0f;
	lon = 0.0f;
	alt = 0.0f;

	// initialize rotation quaternion with a single raw sensor measurement
	_sensors.update();
	q = init(
		_sensors.accelerometer_m_s2[0],
		_sensors.accelerometer_m_s2[1],
		_sensors.accelerometer_m_s2[2],
		_sensors.magnetometer_ga[0],
		_sensors.magnetometer_ga[1],
		_sensors.magnetometer_ga[2]);

	// initialize dcm
	C_nb = Dcm(q);

	// HPos is constant
	HPos(0, 3) = 1.0f;
	HPos(1, 4) = 1.0f;
	HPos(2, 6) = 1.0e7f * M_RAD_TO_DEG_F;
	HPos(3, 7) = 1.0e7f * M_RAD_TO_DEG_F;
	HPos(4, 8) = 1.0f;
	HPos(5, 8) = 1.0f;

	// initialize all parameters
	updateParams();
}

math::Quaternion KalmanNav::init(float ax, float ay, float az, float mx, float my, float mz)
{
    float initialRoll, initialPitch;
    float cosRoll, sinRoll, cosPitch, sinPitch;
    float magX, magY;
    float initialHdg, cosHeading, sinHeading;

    initialRoll = atan2(-ay, -az);
    initialPitch = atan2(ax, -az);

    cosRoll = cosf(initialRoll);
    sinRoll = sinf(initialRoll);
    cosPitch = cosf(initialPitch);
    sinPitch = sinf(initialPitch);

    magX = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;

    magY = my * cosRoll - mz * sinRoll;

    initialHdg = atan2f(-magY, magX);

    cosRoll = cosf(initialRoll * 0.5f);
    sinRoll = sinf(initialRoll * 0.5f);

    cosPitch = cosf(initialPitch * 0.5f);
    sinPitch = sinf(initialPitch * 0.5f);

    cosHeading = cosf(initialHdg * 0.5f);
    sinHeading = sinf(initialHdg * 0.5f);

    float q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
    float q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
    float q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
    float q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;

    return math::Quaternion(q0, q1, q2, q3);

}

void KalmanNav::update()
{
	using namespace math;

	struct pollfd fds[1];
	fds[0].fd = _sensors.getHandle();
	fds[0].events = POLLIN;

	// poll for new data
	int ret = poll(fds, 1, 1000);

	if (ret < 0) {
		// XXX this is seriously bad - should be an emergency
		return;

	} else if (ret == 0) { // timeout
		return;
	}

	// get new timestamp
	uint64_t newTimeStamp = hrt_absolute_time();

	// check updated subscriptions
	if (_param_update.updated()) updateParams();

	bool gpsUpdate = _gps.updated();
	bool sensorsUpdate = _sensors.updated();

	// get new information from subscriptions
	// this clears update flag
	updateSubscriptions();

	// initialize attitude when sensors online
	if (!_attitudeInitialized && sensorsUpdate) {
		if (correctAtt() == ret_ok) _attitudeInitCounter++;

		if (_attitudeInitCounter > 100) {
			warnx("initialized EKF attitude\n");
			warnx("phi: %8.4f, theta: %8.4f, psi: %8.4f\n",
			       double(phi), double(theta), double(psi));
			_attitudeInitialized = true;
		}
	}

	// initialize position when gps received
	if (!_positionInitialized &&
	    _attitudeInitialized && // wait for attitude first
	    gpsUpdate &&
	    _gps.fix_type > 2
	    //&& _gps.counter_pos_valid > 10
	   ) {
		vN = _gps.vel_n_m_s;
		vE = _gps.vel_e_m_s;
		vD = _gps.vel_d_m_s;
		setLatDegE7(_gps.lat);
		setLonDegE7(_gps.lon);
		setAltE3(_gps.alt);
		// set reference position for
		// local position
		lat0 = lat;
		lon0 = lon;
		alt0 = alt;
		// XXX map_projection has internal global
		// states that multiple things could change, 
		// should make map_projection take reference
		// lat/lon and not have init
		map_projection_init(lat0, lon0);
		_positionInitialized = true;
		warnx("initialized EKF state with GPS\n");
		warnx("vN: %8.4f, vE: %8.4f, vD: %8.4f, lat: %8.4f, lon: %8.4f, alt: %8.4f\n",
		       double(vN), double(vE), double(vD),
		       lat, lon, double(alt));
	}

	// prediction step
	// using sensors timestamp so we can account for packet lag
	float dt = (_sensors.timestamp - _predictTimeStamp) / 1.0e6f;
	//printf("dt: %15.10f\n", double(dt));
	_predictTimeStamp = _sensors.timestamp;

	// don't predict if time greater than a second
	if (dt < 1.0f) {
		predictState(dt);
		predictStateCovariance(dt);
		// count fast frames
		_navFrames += 1;
	}

	// count times 100 Hz rate isn't met
	if (dt > 0.01f) _miss++;

	// gps correction step
	if (_positionInitialized && gpsUpdate) {
		correctPos();
	}

	// attitude correction step
	if (_attitudeInitialized 								// initialized
	    && sensorsUpdate 								// new data
	    && _sensors.timestamp - _attTimeStamp > 1e6 / 50 	// 50 Hz
	   ) {
		_attTimeStamp = _sensors.timestamp;
		correctAtt();
	}

	// publication
	if (newTimeStamp - _pubTimeStamp > 1e6 / 50) { // 50 Hz
		_pubTimeStamp = newTimeStamp;

		updatePublications();
	}

	// output
	if (newTimeStamp - _outTimeStamp > 10e6) { // 0.1 Hz
		_outTimeStamp = newTimeStamp;
		//printf("nav: %4d Hz, miss #: %4d\n",
		//       _navFrames / 10, _miss / 10);
		_navFrames = 0;
		_miss = 0;
	}
}

void KalmanNav::updatePublications()
{
	using namespace math;

	// global position publication
	_pos.timestamp = _pubTimeStamp;
	_pos.time_gps_usec = _gps.timestamp_position;
	_pos.valid = true;
	_pos.lat = getLatDegE7();
	_pos.lon = getLonDegE7();
	_pos.alt = float(alt);
	_pos.relative_alt = float(alt); // TODO, make relative
	_pos.vx = vN;
	_pos.vy = vE;
	_pos.vz = vD;
	_pos.yaw = psi;

	// local position publication
	float x;
	float y;
	bool landed = alt < (alt0 + 0.1); // XXX improve?
	map_projection_project(lat, lon, &x, &y); 
	_localPos.timestamp = _pubTimeStamp;
	_localPos.xy_valid = true;
	_localPos.z_valid = true;
	_localPos.v_xy_valid = true;
	_localPos.v_z_valid = true;
	_localPos.x = x;
	_localPos.y = y;
	_localPos.z = alt0 - alt;
	_localPos.vx = vN;
	_localPos.vy = vE;
	_localPos.vz = vD;
	_localPos.yaw = psi;
	_localPos.xy_global = true;
	_localPos.z_global = true;
	_localPos.ref_timestamp = _pubTimeStamp;
	_localPos.ref_lat = getLatDegE7();
	_localPos.ref_lon = getLonDegE7();
	_localPos.ref_alt = 0;
	_localPos.landed = landed;

	// attitude publication
	_att.timestamp = _pubTimeStamp;
	_att.roll = phi;
	_att.pitch = theta;
	_att.yaw = psi;
	_att.rollspeed = _sensors.gyro_rad_s[0];
	_att.pitchspeed = _sensors.gyro_rad_s[1];
	_att.yawspeed = _sensors.gyro_rad_s[2];
	// TODO, add gyro offsets to filter
	_att.rate_offsets[0] = 0.0f;
	_att.rate_offsets[1] = 0.0f;
	_att.rate_offsets[2] = 0.0f;

	for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++)
			_att.R[i][j] = C_nb(i, j);

	for (int i = 0; i < 4; i++) _att.q[i] = q(i);

	_att.R_valid = true;
	_att.q_valid = true;

	// selectively update publications,
	// do NOT call superblock do-all method
	if (_positionInitialized) {
		_pos.update();
		_localPos.update();
	}

	if (_attitudeInitialized)
		_att.update();
}

int KalmanNav::predictState(float dt)
{
	using namespace math;

	// trig
	float sinL = sinf(lat);
	float cosL = cosf(lat);
	float cosLSing = cosf(lat);

	// prevent singularity
	if (fabsf(cosLSing) < 0.01f) {
		if (cosLSing > 0) cosLSing = 0.01;
		else cosLSing = -0.01;
	}

	// attitude prediction
	if (_attitudeInitialized) {
		Vector3 w(_sensors.gyro_rad_s);

		// attitude
		q = q + q.derivative(w) * dt;

		// renormalize quaternion if needed
		if (fabsf(q.norm() - 1.0f) > 1e-4f) {
			q = q.unit();
		}

		// C_nb update
		C_nb = Dcm(q);

		// euler update
		EulerAngles euler(C_nb);
		phi = euler.getPhi();
		theta = euler.getTheta();
		psi = euler.getPsi();

		// specific acceleration in nav frame
		Vector3 accelB(_sensors.accelerometer_m_s2);
		Vector3 accelN = C_nb * accelB;
		fN = accelN(0);
		fE = accelN(1);
		fD = accelN(2);
	}

	// position prediction
	if (_positionInitialized) {
		// neglects angular deflections in local gravity
		// see Titerton pg. 70
		float R = R0 + float(alt);
		float LDot = vN / R;
		float lDot = vE / (cosLSing * R);
		float rotRate = 2 * omega + lDot;

		// XXX position prediction using speed
		float vNDot = fN - vE * rotRate * sinL +
			      vD * LDot;
		float vDDot = fD - vE * rotRate * cosL -
			      vN * LDot + _g.get();
		float vEDot = fE + vN * rotRate * sinL +
			      vDDot * rotRate * cosL;

		// rectangular integration
		vN += vNDot * dt;
		vE += vEDot * dt;
		vD += vDDot * dt;
		lat += double(LDot * dt);
		lon += double(lDot * dt);
		alt += double(-vD * dt);
	}

	return ret_ok;
}

int KalmanNav::predictStateCovariance(float dt)
{
	using namespace math;

	// trig
	float sinL = sinf(lat);
	float cosL = cosf(lat);
	float cosLSq = cosL * cosL;
	float tanL = tanf(lat);

	// prepare for matrix
	float R = R0 + float(alt);
	float RSq = R * R;

	// F Matrix
	// Titterton pg. 291

	F(0, 1) = -(omega * sinL + vE * tanL / R);
	F(0, 2) = vN / R;
	F(0, 4) = 1.0f / R;
	F(0, 6) = -omega * sinL;
	F(0, 8) = -vE / RSq;

	F(1, 0) = omega * sinL + vE * tanL / R;
	F(1, 2) = omega * cosL + vE / R;
	F(1, 3) = -1.0f / R;
	F(1, 8) = vN / RSq;

	F(2, 0) = -vN / R;
	F(2, 1) = -omega * cosL - vE / R;
	F(2, 4) = -tanL / R;
	F(2, 6) = -omega * cosL - vE / (R * cosLSq);
	F(2, 8) = vE * tanL / RSq;

	F(3, 1) = -fD;
	F(3, 2) = fE;
	F(3, 3) = vD / R;
	F(3, 4) = -2 * (omega * sinL + vE * tanL / R);
	F(3, 5) = vN / R;
	F(3, 6) = -vE * (2 * omega * cosL + vE / (R * cosLSq));
	F(3, 8) = (vE * vE * tanL - vN * vD) / RSq;

	F(4, 0) = fD;
	F(4, 2) = -fN;
	F(4, 3) = 2 * omega * sinL + vE * tanL / R;
	F(4, 4) = (vN * tanL + vD) / R;
	F(4, 5) = 2 * omega * cosL + vE / R;
	F(4, 6) = 2 * omega * (vN * cosL - vD * sinL) +
		  vN * vE / (R * cosLSq);
	F(4, 8) = -vE * (vN * tanL + vD) / RSq;

	F(5, 0) = -fE;
	F(5, 1) = fN;
	F(5, 3) = -2 * vN / R;
	F(5, 4) = -2 * (omega * cosL + vE / R);
	F(5, 6) = 2 * omega * vE * sinL;
	F(5, 8) = (vN * vN + vE * vE) / RSq;

	F(6, 3) = 1 / R;
	F(6, 8) = -vN / RSq;

	F(7, 4) = 1 / (R * cosL);
	F(7, 6) = vE * tanL / (R * cosL);
	F(7, 8) = -vE / (cosL * RSq);

	F(8, 5) = -1;

	// G Matrix
	// Titterton pg. 291
	G(0, 0) = -C_nb(0, 0);
	G(0, 1) = -C_nb(0, 1);
	G(0, 2) = -C_nb(0, 2);
	G(1, 0) = -C_nb(1, 0);
	G(1, 1) = -C_nb(1, 1);
	G(1, 2) = -C_nb(1, 2);
	G(2, 0) = -C_nb(2, 0);
	G(2, 1) = -C_nb(2, 1);
	G(2, 2) = -C_nb(2, 2);

	G(3, 3) = C_nb(0, 0);
	G(3, 4) = C_nb(0, 1);
	G(3, 5) = C_nb(0, 2);
	G(4, 3) = C_nb(1, 0);
	G(4, 4) = C_nb(1, 1);
	G(4, 5) = C_nb(1, 2);
	G(5, 3) = C_nb(2, 0);
	G(5, 4) = C_nb(2, 1);
	G(5, 5) = C_nb(2, 2);

	// continuous predictioon equations
	// for discrte time EKF
	// http://en.wikipedia.org/wiki/Extended_Kalman_filter
	P = P + (F * P + P * F.transpose() + G * V * G.transpose()) * dt;

	return ret_ok;
}

int KalmanNav::correctAtt()
{
	using namespace math;

	// trig
	float cosPhi = cosf(phi);
	float cosTheta = cosf(theta);
	// float cosPsi = cosf(psi);
	float sinPhi = sinf(phi);
	float sinTheta = sinf(theta);
	// float sinPsi = sinf(psi);

	// mag predicted measurement
	// choosing some typical magnetic field properties,
	//  TODO dip/dec depend on lat/ lon/ time
	//float dip = _magDip.get() / M_RAD_TO_DEG_F; // dip, inclination with level
	float dec = _magDec.get() / M_RAD_TO_DEG_F; // declination, clockwise rotation from north

	// compensate roll and pitch, but not yaw
	// XXX take the vectors out of the C_nb matrix to avoid singularities
	math::Dcm C_rp(math::EulerAngles(phi, theta, 0.0f));//C_nb.transpose();

	// mag measurement
	Vector3 magBody(_sensors.magnetometer_ga);

	// transform to earth frame
	Vector3 magNav = C_rp * magBody;

	// calculate error between estimate and measurement
	// apply declination correction for true heading as well.
	float yMag = -atan2f(magNav(1),magNav(0)) - psi - dec;
	if (yMag > M_PI_F) yMag -= 2*M_PI_F;
	if (yMag < -M_PI_F) yMag += 2*M_PI_F;

	// accel measurement
	Vector3 zAccel(_sensors.accelerometer_m_s2);
	float accelMag = zAccel.norm();
	zAccel = zAccel.unit();

	// ignore accel correction when accel mag not close to g
	Matrix RAttAdjust = RAtt;

	bool ignoreAccel = fabsf(accelMag - _g.get()) > 1.1f;

	if (ignoreAccel) {
		RAttAdjust(1, 1) = 1.0e10;
		RAttAdjust(2, 2) = 1.0e10;
		RAttAdjust(3, 3) = 1.0e10;

	} else {
		//printf("correcting attitude with accel\n");
	}

	// accel predicted measurement
	Vector3 zAccelHat = (C_nb.transpose() * Vector3(0, 0, -_g.get())).unit();

	// calculate residual
	Vector y(4);
	y(0) = yMag;
	y(1) = zAccel(0) - zAccelHat(0);
	y(2) = zAccel(1) - zAccelHat(1);
	y(3) = zAccel(2) - zAccelHat(2);

	// HMag
	HAtt(0, 2) = 1;

	// HAccel
	HAtt(1, 1) = cosTheta;
	HAtt(2, 0) = -cosPhi * cosTheta;
	HAtt(2, 1) = sinPhi * sinTheta;
	HAtt(3, 0) = sinPhi * cosTheta;
	HAtt(3, 1) = cosPhi * sinTheta;

	// compute correction
	// http://en.wikipedia.org/wiki/Extended_Kalman_filter
	Matrix S = HAtt * P * HAtt.transpose() + RAttAdjust; // residual covariance
	Matrix K = P * HAtt.transpose() * S.inverse();
	Vector xCorrect = K * y;

	// check correciton is sane
	for (size_t i = 0; i < xCorrect.getRows(); i++) {
		float val = xCorrect(i);

		if (isnan(val) || isinf(val)) {
			// abort correction and return
			warnx("numerical failure in att correction\n");
			// reset P matrix to P0
			P = P0;
			return ret_error;
		}
	}

	// correct state
	if (!ignoreAccel) {
		phi += xCorrect(PHI);
		theta += xCorrect(THETA);
	}

	psi += xCorrect(PSI);

	// attitude also affects nav velocities
	if (_positionInitialized) {
		vN += xCorrect(VN);
		vE += xCorrect(VE);
		vD += xCorrect(VD);
	}

	// update state covariance
	// http://en.wikipedia.org/wiki/Extended_Kalman_filter
	P = P - K * HAtt * P;

	// fault detection
	float beta = y.dot(S.inverse() * y);

	if (beta > _faultAtt.get()) {
		warnx("fault in attitude: beta = %8.4f", (double)beta);
		warnx("y:"); y.print();
	}

	// update quaternions from euler
	// angle correction
	q = Quaternion(EulerAngles(phi, theta, psi));

	return ret_ok;
}

int KalmanNav::correctPos()
{
	using namespace math;

	// residual
	Vector y(6);
	y(0) = _gps.vel_n_m_s - vN;
	y(1) = _gps.vel_e_m_s - vE;
	y(2) = double(_gps.lat) - double(lat) * 1.0e7 * M_RAD_TO_DEG;
	y(3) = double(_gps.lon) - double(lon) * 1.0e7 * M_RAD_TO_DEG;
	y(4) = _gps.alt / 1.0e3f - alt;
	y(5) = _sensors.baro_alt_meter - alt;

	// compute correction
	// http://en.wikipedia.org/wiki/Extended_Kalman_filter
	Matrix S = HPos * P * HPos.transpose() + RPos; // residual covariance
	Matrix K = P * HPos.transpose() * S.inverse();
	Vector xCorrect = K * y;

	// check correction is sane
	for (size_t i = 0; i < xCorrect.getRows(); i++) {
		float val = xCorrect(i);

		if (!isfinite(val)) {
			// abort correction and return
			warnx("numerical failure in gps correction\n");
			// fallback to GPS
			vN = _gps.vel_n_m_s;
			vE = _gps.vel_e_m_s;
			vD = _gps.vel_d_m_s;
			setLatDegE7(_gps.lat);
			setLonDegE7(_gps.lon);
			setAltE3(_gps.alt);
			// reset P matrix to P0
			P = P0;
			return ret_error;
		}
	}

	// correct state
	vN += xCorrect(VN);
	vE += xCorrect(VE);
	vD += xCorrect(VD);
	lat += double(xCorrect(LAT));
	lon += double(xCorrect(LON));
	alt += xCorrect(ALT);

	// update state covariance
	// http://en.wikipedia.org/wiki/Extended_Kalman_filter
	P = P - K * HPos * P;

	// fault detetcion
	float beta = y.dot(S.inverse() * y);

	static int counter = 0;
	if (beta > _faultPos.get() && (counter % 10 == 0)) {
		warnx("fault in gps: beta = %8.4f", (double)beta);
		warnx("Y/N: vN: %8.4f, vE: %8.4f, lat: %8.4f, lon: %8.4f, alt: %8.4f, baro: %8.4f",
		       double(y(0) / sqrtf(RPos(0, 0))),
		       double(y(1) / sqrtf(RPos(1, 1))),
		       double(y(2) / sqrtf(RPos(2, 2))),
		       double(y(3) / sqrtf(RPos(3, 3))),
		       double(y(4) / sqrtf(RPos(4, 4))),
		       double(y(5) / sqrtf(RPos(5, 5))));
	}
	counter++;

	return ret_ok;
}

void KalmanNav::updateParams()
{
	using namespace math;
	using namespace control;
	SuperBlock::updateParams();

	// gyro noise
	V(0, 0) = _vGyro.get();   // gyro x, rad/s
	V(1, 1) = _vGyro.get();   // gyro y
	V(2, 2) = _vGyro.get();   // gyro z

	// accel noise
	V(3, 3) = _vAccel.get();   // accel x, m/s^2
	V(4, 4) = _vAccel.get();   // accel y
	V(5, 5) = _vAccel.get();   // accel z

	// magnetometer noise
	float noiseMin = 1e-6f;
	float noiseMagSq = _rMag.get() * _rMag.get();

	if (noiseMagSq < noiseMin) noiseMagSq = noiseMin;

	RAtt(0, 0) = noiseMagSq; // normalized direction

	// accelerometer noise
	float noiseAccelSq = _rAccel.get() * _rAccel.get();

	// bound noise to prevent singularities
	if (noiseAccelSq < noiseMin) noiseAccelSq = noiseMin;

	RAtt(1, 1) = noiseAccelSq; // normalized direction
	RAtt(2, 2) = noiseAccelSq;
	RAtt(3, 3) = noiseAccelSq;

	// gps noise
	float R = R0 + float(alt);
	float cosLSing = cosf(lat);

	// prevent singularity
	if (fabsf(cosLSing) < 0.01f) {
		if (cosLSing > 0) cosLSing = 0.01;
		else cosLSing = -0.01;
	}

	float noiseVel = _rGpsVel.get();
	float noiseLatDegE7 = 1.0e7f * M_RAD_TO_DEG_F * _rGpsPos.get() / R;
	float noiseLonDegE7 = noiseLatDegE7 / cosLSing;
	float noiseGpsAlt = _rGpsAlt.get();
	float noisePressAlt = _rPressAlt.get();

	// bound noise to prevent singularities
	if (noiseVel < noiseMin) noiseVel = noiseMin;

	if (noiseLatDegE7 < noiseMin) noiseLatDegE7 = noiseMin;

	if (noiseLonDegE7 < noiseMin) noiseLonDegE7 = noiseMin;

	if (noiseGpsAlt < noiseMin) noiseGpsAlt = noiseMin;

	if (noisePressAlt < noiseMin) noisePressAlt = noiseMin;

	RPos(0, 0) = noiseVel * noiseVel; // vn
	RPos(1, 1) = noiseVel * noiseVel; // ve
	RPos(2, 2) = noiseLatDegE7 * noiseLatDegE7; // lat
	RPos(3, 3) = noiseLonDegE7 * noiseLonDegE7; // lon
	RPos(4, 4) = noiseGpsAlt * noiseGpsAlt; // h
	RPos(5, 5) = noisePressAlt * noisePressAlt; // h
	// XXX, note that RPos depends on lat, so updateParams should
	// be called if lat changes significantly
}
