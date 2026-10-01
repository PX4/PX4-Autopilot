#include "../BlockLocalPositionEstimator.hpp"
#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>

extern orb_advert_t mavlink_log_pub;

// required number of samples for sensor
// to initialize
static const uint32_t		REQ_GNSS_INIT_COUNT = 10;
static const uint32_t		GNSS_TIMEOUT = 1000000;	// 1.0 s

void BlockLocalPositionEstimator::gnssInit()
{
	// check for good gnss signal
	uint8_t nSat = _sub_gnss.get().receiver.satellites_used;
	float eph = _sub_gnss.get().receiver.eph;
	float epv = _sub_gnss.get().receiver.epv;
	uint8_t fix_type = _sub_gnss.get().receiver.fix_type;

	if (
		nSat < 6 ||
		eph > _param_lpe_eph_max.get() ||
		epv > _param_lpe_epv_max.get() ||
		fix_type < 3
	) {
		_gnssStats.reset();
		return;
	}

	// measure
	Vector<double, n_y_gnss> y;

	if (gnssMeasure(y) != OK) {
		_gnssStats.reset();
		return;
	}

	// if finished
	if (_gnssStats.getCount() > REQ_GNSS_INIT_COUNT) {
		// get mean gnss values
		double gnssLat = _gnssStats.getMean()(0);
		double gnssLon = _gnssStats.getMean()(1);
		float gnssAlt = _gnssStats.getMean()(2);

		_sensorTimeout &= ~SENSOR_GNSS;
		_sensorFault &= ~SENSOR_GNSS;
		_gnssStats.reset();

		if (!_receivedGnss) {
			// this is the first time we have received gnss
			_receivedGnss = true;

			// note we subtract X_z which is in down directon so it is
			// an addition
			_gnssAltOrigin = gnssAlt + _x(X_z);

			// find lat, lon of current origin by subtracting x and y
			// if not using vision position since vision will
			// have it's own origin, not necessarily where vehicle starts
			if (!_map_ref.isInitialized()) {
				double gnssLatOrigin = 0;
				double gnssLonOrigin = 0;
				// reproject at current coordinates
				_map_ref.initReference(gnssLat, gnssLon);
				// find origin
				_map_ref.reproject(-_x(X_x), -_x(X_y), gnssLatOrigin, gnssLonOrigin);
				// reinit origin
				_map_ref.initReference(gnssLatOrigin, gnssLonOrigin);
				// set timestamp when origin was set to current time
				_time_origin = _timeStamp;

				// always override alt origin on first GNSS to fix
				// possible baro offset in global altitude at init
				_altOrigin = _gnssAltOrigin;
				_altOriginInitialized = true;
				_altOriginGlobal = true;

				mavlink_log_info(&mavlink_log_pub, "[lpe] global origin init (gps) : lat %6.2f lon %6.2f alt %5.1f m",
						 gnssLatOrigin, gnssLonOrigin, double(_gnssAltOrigin));
			}

			PX4_INFO("[lpe] gps init "
				 "lat %6.2f lon %6.2f alt %5.1f m",
				 gnssLat,
				 gnssLon,
				 double(gnssAlt));
		}
	}
}

int BlockLocalPositionEstimator::gnssMeasure(Vector<double, n_y_gnss> &y)
{
	// gnss measurement
	y.setZero();
	y(0) = _sub_gnss.get().receiver.latitude;
	y(1) = _sub_gnss.get().receiver.longitude;
	y(2) = _sub_gnss.get().receiver.altitude_msl;
	y(3) = (double)_sub_gnss.get().receiver.vel_north;
	y(4) = (double)_sub_gnss.get().receiver.vel_east;
	y(5) = (double)_sub_gnss.get().receiver.vel_down;

	// increament sums for mean
	_gnssStats.update(y);
	_time_last_gnss = _timeStamp;
	return OK;
}

void BlockLocalPositionEstimator::gnssCorrect()
{
	// measure
	Vector<double, n_y_gnss> y_global;

	if (gnssMeasure(y_global) != OK) { return; }

	// gnss measurement in local frame
	double lat = y_global(Y_gnss_x);
	double lon = y_global(Y_gnss_y);
	float alt = y_global(Y_gnss_z);
	float px = 0;
	float py = 0;
	float pz = -(alt - _gnssAltOrigin);
	_map_ref.project(lat, lon, px, py);
	Vector<float, n_y_gnss> y;
	y.setZero();
	y(Y_gnss_x) = px;
	y(Y_gnss_y) = py;
	y(Y_gnss_z) = pz;
	y(Y_gnss_vx) = y_global(Y_gnss_vx);
	y(Y_gnss_vy) = y_global(Y_gnss_vy);
	y(Y_gnss_vz) = y_global(Y_gnss_vz);

	// gnss measurement matrix, measures position and velocity
	Matrix<float, n_y_gnss, n_x> C;
	C.setZero();
	C(Y_gnss_x, X_x) = 1;
	C(Y_gnss_y, X_y) = 1;
	C(Y_gnss_z, X_z) = 1;
	C(Y_gnss_vx, X_vx) = 1;
	C(Y_gnss_vy, X_vy) = 1;
	C(Y_gnss_vz, X_vz) = 1;

	// gnss covariance matrix
	SquareMatrix<float, n_y_gnss> R;
	R.setZero();

	// default to parameter, use gnss cov if provided
	float var_xy = _param_lpe_gps_xy.get() * _param_lpe_gps_xy.get();
	float var_z = _param_lpe_gps_z.get() * _param_lpe_gps_z.get();
	float var_vxy = _param_lpe_gps_vxy.get() * _param_lpe_gps_vxy.get();
	float var_vz = _param_lpe_gps_vz.get() * _param_lpe_gps_vz.get();

	// if field is not below minimum, set it to the value provided
	if (_sub_gnss.get().receiver.eph > _param_lpe_gps_xy.get()) {
		var_xy = _sub_gnss.get().receiver.eph * _sub_gnss.get().receiver.eph;
	}

	if (_sub_gnss.get().receiver.epv > _param_lpe_gps_z.get()) {
		var_z = _sub_gnss.get().receiver.epv * _sub_gnss.get().receiver.epv;
	}

	float gnss_s_stddev =  _sub_gnss.get().receiver.speed_accuracy;

	if (gnss_s_stddev > _param_lpe_gps_vxy.get()) {
		var_vxy = gnss_s_stddev * gnss_s_stddev;
	}

	if (gnss_s_stddev > _param_lpe_gps_vz.get()) {
		var_vz = gnss_s_stddev * gnss_s_stddev;
	}

	R(0, 0) = var_xy;
	R(1, 1) = var_xy;
	R(2, 2) = var_z;
	R(3, 3) = var_vxy;
	R(4, 4) = var_vxy;
	R(5, 5) = var_vz;

	// get delayed x
	uint8_t i_hist = 0;

	if (getDelayPeriods(_param_lpe_gps_delay.get(), &i_hist)  < 0) { return; }

	Vector<float, n_x> x0 = _xDelay.get(i_hist);

	// residual
	Vector<float, n_y_gnss> r = y - C * x0;

	// residual covariance
	Matrix<float, n_y_gnss, n_y_gnss> S = C * m_P * C.transpose() + R;

	// publish innovations
	_pub_innov.get().gps_hpos[0] = r(0);
	_pub_innov.get().gps_hpos[1] = r(1);
	_pub_innov.get().gps_vpos    = r(2);
	_pub_innov.get().gps_hvel[0] = r(3);
	_pub_innov.get().gps_hvel[1] = r(4);
	_pub_innov.get().gps_vvel    = r(5);

	// publish innovation variances
	_pub_innov_var.get().gps_hpos[0] = S(0, 0);
	_pub_innov_var.get().gps_hpos[1] = S(1, 1);
	_pub_innov_var.get().gps_vpos    = S(2, 2);
	_pub_innov_var.get().gps_hvel[0] = S(3, 3);
	_pub_innov_var.get().gps_hvel[1] = S(4, 4);
	_pub_innov_var.get().gps_vvel    = S(5, 5);

	// residual covariance, (inverse)
	Matrix<float, n_y_gnss, n_y_gnss> S_I = inv<float, n_y_gnss>(S);

	// fault detection
	float beta = (r.transpose() * (S_I * r))(0, 0);

	// artificially increase beta threshhold to prevent fault during landing
	float beta_thresh = 1e2f;

	if (beta / BETA_TABLE[n_y_gnss] > beta_thresh) {
		if (!(_sensorFault & SENSOR_GNSS)) {
			mavlink_log_critical(&mavlink_log_pub, "[lpe] gps fault %3g %3g %3g %3g %3g %3g",
					     double(r(0) * r(0) / S_I(0, 0)),  double(r(1) * r(1) / S_I(1, 1)), double(r(2) * r(2) / S_I(2, 2)),
					     double(r(3) * r(3) / S_I(3, 3)),  double(r(4) * r(4) / S_I(4, 4)), double(r(5) * r(5) / S_I(5, 5)));
			_sensorFault |= SENSOR_GNSS;
		}

	} else if (_sensorFault & SENSOR_GNSS) {
		_sensorFault &= ~SENSOR_GNSS;
		mavlink_log_info(&mavlink_log_pub, "[lpe] GNSS OK");
	}

	// kalman filter correction always for GNSS
	Matrix<float, n_x, n_y_gnss> K = m_P * C.transpose() * S_I;
	Vector<float, n_x> dx = K * r;
	_x += dx;
	m_P -= K * C * m_P;
}

void BlockLocalPositionEstimator::gnssCheckTimeout()
{
	if (_timeStamp - _time_last_gnss > GNSS_TIMEOUT) {
		if (!(_sensorTimeout & SENSOR_GNSS)) {
			_sensorTimeout |= SENSOR_GNSS;
			_gnssStats.reset();
			mavlink_log_critical(&mavlink_log_pub, "[lpe] GNSS timeout ");
		}
	}
}
