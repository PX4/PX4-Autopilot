#include "EKFGSF_yaw.h"
#include <cstdlib>

EKFGSF_yaw::EKFGSF_yaw()
{
	// this flag must be false when we start
	_ahrs_ekf_gsf_tilt_aligned = false;

	// these objects are initialised in initialise() before being used internally, but can be reported for logging before then
	memset(&_ahrs_ekf_gsf, 0, sizeof(_ahrs_ekf_gsf));
	memset(&_ekf_gsf, 0, sizeof(_ekf_gsf));
	_gsf_yaw = 0.0f;
	_ahrs_accel.zero();
}

void EKFGSF_yaw::update(const imuSample& imu_sample,
                	bool run_EKF,           // set to true when flying or movement is suitable for yaw estimation
                	float airspeed)   	// true airspeed used for centripetal accel compensation - set to 0 when not required.
{
	// copy to class variables
	_delta_ang = imu_sample.delta_ang;
	_delta_vel = imu_sample.delta_vel;
	_delta_ang_dt = imu_sample.delta_ang_dt;
	_delta_vel_dt = imu_sample.delta_vel_dt;
	_run_ekf_gsf = run_EKF;
	_true_airspeed = airspeed;

	// to reduce effect of vibration, filter using an LPF whose time constant is 1/10 of the AHRS tilt correction time constant
	const float filter_coef = fminf(10.0f * _delta_vel_dt * _tilt_gain, 1.0f);
	const Vector3f accel = _delta_vel / fmaxf(_delta_vel_dt, 0.001f);
	_ahrs_accel = _ahrs_accel * (1.0f - filter_coef) + accel * filter_coef;

	// Initialise states first time
	if (!_ahrs_ekf_gsf_tilt_aligned) {
		// check for excessive acceleration to reduce likelihood of large initial roll/pitch errors
		// due to vehicle movement
		const float accel_norm_sq = accel.norm_squared();
		const float upper_accel_limit = CONSTANTS_ONE_G * 1.1f;
		const float lower_accel_limit = CONSTANTS_ONE_G * 0.9f;
		const bool ok_to_align = (accel_norm_sq > sq(lower_accel_limit)) && (accel_norm_sq < sq(upper_accel_limit));
		if (ok_to_align) {
			initialiseEKFGSF();
			ahrsAlignTilt();
			_ahrs_ekf_gsf_tilt_aligned = true;
		}
		return;
	}

	// calculate common values used by the AHRS complementary filter models
	_ahrs_accel_norm = _ahrs_accel.norm();

	// AHRS prediction cycle for each model - this always runs
	_ahrs_accel_fusion_gain = ahrsCalcAccelGain();
	for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) {
		predictEKF(model_index);
	}

	// The 3-state EKF models only run when flying to avoid corrupted estimates due to operator handling and GPS interference
	if (_run_ekf_gsf && _vel_data_updated) {
		if (!_ekf_gsf_vel_fuse_started) {
			initialiseEKFGSF();
			ahrsAlignYaw();
			_ekf_gsf_vel_fuse_started = true;
		} else {
			bool bad_update = false;
			for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) {
				// subsequent measurements are fused as direct state observations
				if (!updateEKF(model_index)) {
					bad_update = true;
				}
			}

			if (!bad_update) {
				float total_weight = 0.0f;
				// calculate weighting for each model assuming a normal distribution
				for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) {
					_model_weights(model_index) = fmaxf(gaussianDensity(model_index) * _model_weights(model_index), 0.0f);
					total_weight += _model_weights(model_index);
				}

				// normalise the weighting function
				if (total_weight > 1e-15f) {
					_model_weights /= total_weight;
				}

				// Enforce a minimum weighting value. This was added during initial development but has not been needed
				// subsequently, so this block of code and the corresponding _weight_min can be removed if we get
				// through testing without any weighting function issues.
				if (_weight_min > FLT_EPSILON) {
					float correction_sum = 0.0f; // amount the sum of weights has been increased by application of the limit
					bool change_mask[N_MODELS_EKFGSF] = {}; // true when the weighting for that model has been increased
					float unmodified_weights_sum = 0.0f; // sum of unmodified weights
					for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) {
						if (_model_weights(model_index) < _weight_min) {
							correction_sum += _weight_min - _model_weights(model_index);
							_model_weights(model_index) = _weight_min;
							change_mask[model_index] = true;
						} else {
							unmodified_weights_sum += _model_weights(model_index);
						}
					}

					// rescale the unmodified weights to make the total sum unity
					const float scale_factor = (unmodified_weights_sum - correction_sum - _weight_min) / (unmodified_weights_sum - _weight_min);
					for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) {
						if (!change_mask[model_index]) {
							_model_weights(model_index) = _weight_min + scale_factor * (_model_weights(model_index) - _weight_min);
						}
					}
				}
			}
		}
	} else if (_ekf_gsf_vel_fuse_started && !_run_ekf_gsf) {
		// wait to fly again
		_ekf_gsf_vel_fuse_started = false;
	}

	// Calculate a composite yaw vector as a weighted average of the states for each model.
	// To avoid issues with angle wrapping, the yaw state is converted to a vector with length
	// equal to the weighting value before it is summed.
	Vector2f yaw_vector;
	for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) {
		yaw_vector(0) += _model_weights(model_index) * cosf(_ekf_gsf[model_index].X(2));
		yaw_vector(1) += _model_weights(model_index) * sinf(_ekf_gsf[model_index].X(2));
	}
	_gsf_yaw = atan2f(yaw_vector(1),yaw_vector(0));

	// calculate a composite variance for the yaw state from a weighted average of the variance for each model
	// models with larger innovations are weighted less
	_gsf_yaw_variance = 0.0f;
	for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) {
		const float yaw_delta = wrap_pi(_ekf_gsf[model_index].X(2) - _gsf_yaw);
		_gsf_yaw_variance += _model_weights(model_index) * (_ekf_gsf[model_index].P(2,2) + yaw_delta * yaw_delta);
	}

	// prevent the same velocity data being used more than once
	_vel_data_updated = false;
}

void EKFGSF_yaw::ahrsPredict(const uint8_t model_index)
{
	// generate attitude solution using simple complementary filter for the selected model

	const Vector3f ang_rate = _delta_ang / fmaxf(_delta_ang_dt, 0.001f) - _ahrs_ekf_gsf[model_index].gyro_bias;

	const Dcmf R_to_body = _ahrs_ekf_gsf[model_index].R.transpose();
	const Vector3f gravity_direction_bf = R_to_body.col(2);

	// Perform angular rate correction using accel data and reduce correction as accel magnitude moves away from 1 g (reduces drift when vehicle picked up and moved).
	// During fixed wing flight, compensate for centripetal acceleration assuming coordinated turns and X axis forward
	Vector3f tilt_correction;
	if (_ahrs_accel_fusion_gain > 0.0f) {

		Vector3f accel = _ahrs_accel;

		if (_true_airspeed > FLT_EPSILON) {
			// Calculate body frame centripetal acceleration with assumption X axis is aligned with the airspeed vector
			// Use cross product of body rate and body frame airspeed vector
			const Vector3f centripetal_accel_bf = Vector3f(0.0f, _true_airspeed * ang_rate(2), - _true_airspeed * ang_rate(1));

			// correct measured accel for centripetal acceleration
			accel -= centripetal_accel_bf;
		}

		tilt_correction = (gravity_direction_bf % accel) * _ahrs_accel_fusion_gain / _ahrs_accel_norm;

	}

	// Gyro bias estimation
	constexpr float gyro_bias_limit = 0.05f;
	const float spinRate = ang_rate.length();
	if (spinRate < 0.175f) {
		_ahrs_ekf_gsf[model_index].gyro_bias -= tilt_correction * (_gyro_bias_gain * _delta_ang_dt);
		_ahrs_ekf_gsf[model_index].gyro_bias = matrix::constrain(_ahrs_ekf_gsf[model_index].gyro_bias, -gyro_bias_limit, gyro_bias_limit);
	}

	// delta angle from previous to current frame
	const Vector3f delta_angle_corrected = _delta_ang + (tilt_correction - _ahrs_ekf_gsf[model_index].gyro_bias) * _delta_ang_dt;

	// Apply delta angle to rotation matrix
	_ahrs_ekf_gsf[model_index].R = ahrsPredictRotMat(_ahrs_ekf_gsf[model_index].R, delta_angle_corrected);

}

void EKFGSF_yaw::ahrsAlignTilt()
{
	// Rotation matrix is constructed directly from acceleration measurement and will be the same for
	// all models so only need to calculate it once. Assumptions are:
	// 1) Yaw angle is zero - yaw is aligned later for each model when velocity fusion commences.
	// 2) The vehicle is not accelerating so all of the measured acceleration is due to gravity.

	// Calculate earth frame Down axis unit vector rotated into body frame
	const Vector3f down_in_bf = -_delta_vel.normalized();

	// Calculate earth frame North axis unit vector rotated into body frame, orthogonal to 'down_in_bf'
	const Vector3f i_vec_bf(1.0f,0.0f,0.0f);
	Vector3f north_in_bf = i_vec_bf - down_in_bf * (i_vec_bf.dot(down_in_bf));
	north_in_bf.normalize();

	// Calculate earth frame East axis unit vector rotated into body frame, orthogonal to 'down_in_bf' and 'north_in_bf'
	const Vector3f east_in_bf = down_in_bf % north_in_bf;

	// Each column in a rotation matrix from earth frame to body frame represents the projection of the
	// corresponding earth frame unit vector rotated into the body frame, eg 'north_in_bf' would be the first column.
	// We need the rotation matrix from body frame to earth frame so the earth frame unit vectors rotated into body
	// frame are copied into corresponding rows instead.
	Dcmf R;
	R.setRow(0, north_in_bf);
	R.setRow(1, east_in_bf);
	R.setRow(2, down_in_bf);

	for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index++) {
		_ahrs_ekf_gsf[model_index].R = R;
	}
}

void EKFGSF_yaw::ahrsAlignYaw()
{
	// Align yaw angle for each model
	for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index++) {
		if (fabsf(_ahrs_ekf_gsf[model_index].R(2, 0)) < fabsf(_ahrs_ekf_gsf[model_index].R(2, 1))) {
			// get the roll, pitch, yaw estimates from the rotation matrix using a  321 Tait-Bryan rotation sequence
			Eulerf euler_init(_ahrs_ekf_gsf[model_index].R);

			// set the yaw angle
			euler_init(2) = wrap_pi(_ekf_gsf[model_index].X(2));

			// update the rotation matrix
			_ahrs_ekf_gsf[model_index].R = Dcmf(euler_init);

		} else {
			// Calculate the 312 Tait-Bryan rotation sequence that rotates from earth to body frame
			Vector3f rot312;
			rot312(0) = wrap_pi(_ekf_gsf[model_index].X(2)); // first rotation (yaw) taken from EKF model state
			rot312(1) = asinf(_ahrs_ekf_gsf[model_index].R(2, 1)); // second rotation (roll)
			rot312(2) = atan2f(-_ahrs_ekf_gsf[model_index].R(2, 0), _ahrs_ekf_gsf[model_index].R(2, 2));  // third rotation (pitch)

			// Calculate the body to earth frame rotation matrix
			_ahrs_ekf_gsf[model_index].R = taitBryan312ToRotMat(rot312);

		}
		_ahrs_ekf_gsf[model_index].aligned = true;
	}
}

void EKFGSF_yaw::predictEKF(const uint8_t model_index)
{
	// generate an attitude reference using IMU data
	ahrsPredict(model_index);

	// we don't start running the EKF part of the algorithm until there are regular velocity observations
	if (!_ekf_gsf_vel_fuse_started) {
		return;
	}

	// Calculate the yaw state using a projection onto the horizontal that avoids gimbal lock
	if (fabsf(_ahrs_ekf_gsf[model_index].R(2, 0)) < fabsf(_ahrs_ekf_gsf[model_index].R(2, 1))) {
		// use 321 Tait-Bryan rotation to define yaw state
		_ekf_gsf[model_index].X(2) = atan2f(_ahrs_ekf_gsf[model_index].R(1, 0), _ahrs_ekf_gsf[model_index].R(0, 0));
	} else {
		// use 312 Tait-Bryan rotation to define yaw state
		_ekf_gsf[model_index].X(2) = atan2f(-_ahrs_ekf_gsf[model_index].R(0, 1), _ahrs_ekf_gsf[model_index].R(1, 1)); // first rotation (yaw)
	}

	// calculate delta velocity in a horizontal front-right frame
	const Vector3f del_vel_NED = _ahrs_ekf_gsf[model_index].R * _delta_vel;
	const float dvx =   del_vel_NED(0) * cosf(_ekf_gsf[model_index].X(2)) + del_vel_NED(1) * sinf(_ekf_gsf[model_index].X(2));
	const float dvy = - del_vel_NED(0) * sinf(_ekf_gsf[model_index].X(2)) + del_vel_NED(1) * cosf(_ekf_gsf[model_index].X(2));

	// sum delta velocities in earth frame:
	_ekf_gsf[model_index].X(0) += del_vel_NED(0);
	_ekf_gsf[model_index].X(1) += del_vel_NED(1);

	// predict covariance - autocode from https://github.com/priseborough/3_state_filter/blob/flightLogReplay-wip/calcPupdate.txt

	// Local short variable name copies required for readability
	// Compiler might be smart enough to optimise these out
	const float &P00 = _ekf_gsf[model_index].P(0,0);
	const float &P01 = _ekf_gsf[model_index].P(0,1);
	const float &P02 = _ekf_gsf[model_index].P(0,2);
	const float &P10 = _ekf_gsf[model_index].P(1,0);
	const float &P11 = _ekf_gsf[model_index].P(1,1);
	const float &P12 = _ekf_gsf[model_index].P(1,2);
	const float &P20 = _ekf_gsf[model_index].P(2,0);
	const float &P21 = _ekf_gsf[model_index].P(2,1);
	const float &P22 = _ekf_gsf[model_index].P(2,2);

	// Use fixed values for delta velocity and delta angle process noise variances
	const float dvxVar = sq(_accel_noise * _delta_vel_dt); // variance of forward delta velocity - (m/s)^2
	const float dvyVar = dvxVar; // variance of right delta velocity - (m/s)^2
	const float dazVar = sq(_gyro_noise * _delta_ang_dt); // variance of yaw delta angle - rad^2

	const float t2 = sinf(_ekf_gsf[model_index].X(2));
	const float t3 = cosf(_ekf_gsf[model_index].X(2));
	const float t4 = dvy*t3;
	const float t5 = dvx*t2;
	const float t6 = t4+t5;
	const float t8 = P22*t6;
	const float t7 = P02-t8;
	const float t9 = dvx*t3;
	const float t11 = dvy*t2;
	const float t10 = t9-t11;
	const float t12 = dvxVar*t2*t3;
	const float t13 = t2*t2;
	const float t14 = t3*t3;
	const float t15 = P22*t10;
	const float t16 = P12+t15;

	const float min_var = 1e-6f;
	_ekf_gsf[model_index].P(0,0) = fmaxf(P00-P20*t6+dvxVar*t14+dvyVar*t13-t6*t7 , min_var);
	_ekf_gsf[model_index].P(0,1) = P01+t12-P21*t6+t7*t10-dvyVar*t2*t3;
	_ekf_gsf[model_index].P(0,2) = t7;
	_ekf_gsf[model_index].P(1,0) = P10+t12+P20*t10-t6*t16-dvyVar*t2*t3;
	_ekf_gsf[model_index].P(1,1) = fmaxf(P11+P21*t10+dvxVar*t13+dvyVar*t14+t10*t16 , min_var);
	_ekf_gsf[model_index].P(1,2) = t16;
	_ekf_gsf[model_index].P(2,0) = P20-t8;
	_ekf_gsf[model_index].P(2,1) = P21+t15;
	_ekf_gsf[model_index].P(2,2) = fmaxf(P22+dazVar , min_var);

	// force symmetry
	_ekf_gsf[model_index].P.makeBlockSymmetric<3>(0);
}

// Update EKF states and covariance for specified model index using velocity measurement
bool EKFGSF_yaw::updateEKF(const uint8_t model_index)
{
	// set observation variance from accuracy estimate supplied by GPS and apply a sanity check minimum
	const float velObsVar = sq(fmaxf(_vel_accuracy, 0.5f));

	// calculate velocity observation innovations
	_ekf_gsf[model_index].innov(0) = _ekf_gsf[model_index].X(0) - _vel_NE(0);
	_ekf_gsf[model_index].innov(1) = _ekf_gsf[model_index].X(1) - _vel_NE(1);

	// Use temporary variables for covariance elements to reduce verbosity of auto-code expressions
	const float &P00 = _ekf_gsf[model_index].P(0,0);
	const float &P01 = _ekf_gsf[model_index].P(0,1);
	const float &P02 = _ekf_gsf[model_index].P(0,2);
	const float &P10 = _ekf_gsf[model_index].P(1,0);
	const float &P11 = _ekf_gsf[model_index].P(1,1);
	const float &P12 = _ekf_gsf[model_index].P(1,2);
	const float &P20 = _ekf_gsf[model_index].P(2,0);
	const float &P21 = _ekf_gsf[model_index].P(2,1);
	const float &P22 = _ekf_gsf[model_index].P(2,2);

	// calculate innovation variance
	matrix::SquareMatrix<float, 2> S = _ekf_gsf[model_index].P.slice<2, 2>(0, 0);
	S(0, 0) += velObsVar;
	S(1, 1) += velObsVar;

	// Update the inverse of the innovation covariance matrix S_inverse
	updateInnovCovMatInv(model_index, S);

	// Perform a chi-square innovation consistency test and calculate a compression scale factor that limits the magnitude of innovations to 5-sigma
	float innov_comp_scale_factor = 1.0f;

	// test ratio = transpose(innovation) * inverse(innovation variance) * innovation = [1x2] * [2,2] * [2,1] = [1,1]
	const float test_ratio = _ekf_gsf[model_index].innov * (_ekf_gsf[model_index].S_inverse * _ekf_gsf[model_index].innov);

	// If the test ratio is greater than 25 (5 Sigma) then reduce the length of the innovation vector to clip it at 5-Sigma
	// This protects from large measurement spikes
	if (test_ratio > 25.0f) {
		innov_comp_scale_factor = sqrtf(25.0f / test_ratio);
	}

	// calculate Kalman gain K  nd covariance matrix P
	// autocode from https://github.com/priseborough/3_state_filter/blob/flightLogReplay-wip/calcK.txt
	// and https://github.com/priseborough/3_state_filter/blob/flightLogReplay-wip/calcPmat.txt
	const float t2 = P00*velObsVar;
 	const float t3 = P11*velObsVar;
	const float t4 = velObsVar*velObsVar;
	const float t5 = P00*P11;
	const float t9 = P01*P10;
	const float t6 = t2+t3+t4+t5-t9;
	float t7;
	if (fabsf(t6) > 1e-6f) {
		t7 = 1.0f/t6;
	} else {
		// skip this fusion step
		return false;
	}
	const float t8 = P11+velObsVar;
	const float t10 = P00+velObsVar;

	matrix::Matrix<float, 3, 2> K;
 	K(0,0) = -P01*P10*t7+P00*t7*t8;
	K(0,1) = -P00*P01*t7+P01*t7*t10;
	K(1,0) = -P10*P11*t7+P10*t7*t8;
	K(1,1) = -P01*P10*t7+P11*t7*t10;
	K(2,0) = -P10*P21*t7+P20*t7*t8;
	K(2,1) = -P01*P20*t7+P21*t7*t10;

	const float t11 = P00*P01*t7;
	const float t15 = P01*t7*t10;
	const float t12 = t11-t15;
	const float t13 = P01*P10*t7;
	const float t16 = P00*t7*t8;
	const float t14 = t13-t16;
	const float t17 = t8*t12;
	const float t18 = P01*t14;
	const float t19 = t17+t18;
	const float t20 = t10*t14;
	const float t21 = P10*t12;
	const float t22 = t20+t21;
	const float t27 = P11*t7*t10;
	const float t23 = t13-t27;
	const float t24 = P10*P11*t7;
	const float t26 = P10*t7*t8;
	const float t25 = t24-t26;
	const float t28 = t8*t23;
	const float t29 = P01*t25;
	const float t30 = t28+t29;
	const float t31 = t10*t25;
	const float t32 = P10*t23;
	const float t33 = t31+t32;
	const float t34 = P01*P20*t7;
	const float t38 = P21*t7*t10;
	const float t35 = t34-t38;
	const float t36 = P10*P21*t7;
	const float t39 = P20*t7*t8;
	const float t37 = t36-t39;
	const float t40 = t8*t35;
	const float t41 = P01*t37;
	const float t42 = t40+t41;
	const float t43 = t10*t37;
	const float t44 = P10*t35;
	const float t45 = t43+t44;

	const float min_var = 1e-6f;
	_ekf_gsf[model_index].P(0,0) = fmaxf(P00-t12*t19-t14*t22 , min_var);
	_ekf_gsf[model_index].P(0,1) = P01-t19*t23-t22*t25;
	_ekf_gsf[model_index].P(0,2) = P02-t19*t35-t22*t37;
	_ekf_gsf[model_index].P(1,0) = P10-t12*t30-t14*t33;
	_ekf_gsf[model_index].P(1,1) = fmaxf(P11-t23*t30-t25*t33 , min_var);
	_ekf_gsf[model_index].P(1,2) = P12-t30*t35-t33*t37;
	_ekf_gsf[model_index].P(2,0) = P20-t12*t42-t14*t45;
	_ekf_gsf[model_index].P(2,1) = P21-t23*t42-t25*t45;
	_ekf_gsf[model_index].P(2,2) = fmaxf(P22-t35*t42-t37*t45 , min_var);

	// force symmetry
	_ekf_gsf[model_index].P.makeBlockSymmetric<3>(0);

	// Correct the state vector and capture the change in yaw angle
	const float oldYaw = _ekf_gsf[model_index].X(2);

	_ekf_gsf[model_index].X -= (K * _ekf_gsf[model_index].innov) * innov_comp_scale_factor;

	const float yawDelta = _ekf_gsf[model_index].X(2) - oldYaw;

	// apply the change in yaw angle to the AHRS
	// take advantage of sparseness in the yaw rotation matrix
	const float cosYaw = cosf(yawDelta);
	const float sinYaw = sinf(yawDelta);
	const float R_prev00 = _ahrs_ekf_gsf[model_index].R(0, 0);
	const float R_prev01 = _ahrs_ekf_gsf[model_index].R(0, 1);
	const float R_prev02 = _ahrs_ekf_gsf[model_index].R(0, 2);

	_ahrs_ekf_gsf[model_index].R(0, 0) = R_prev00 * cosYaw - _ahrs_ekf_gsf[model_index].R(1, 0) * sinYaw;
	_ahrs_ekf_gsf[model_index].R(0, 1) = R_prev01 * cosYaw - _ahrs_ekf_gsf[model_index].R(1, 1) * sinYaw;
	_ahrs_ekf_gsf[model_index].R(0, 2) = R_prev02 * cosYaw - _ahrs_ekf_gsf[model_index].R(1, 2) * sinYaw;
	_ahrs_ekf_gsf[model_index].R(1, 0) = R_prev00 * sinYaw + _ahrs_ekf_gsf[model_index].R(1, 0) * cosYaw;
	_ahrs_ekf_gsf[model_index].R(1, 1) = R_prev01 * sinYaw + _ahrs_ekf_gsf[model_index].R(1, 1) * cosYaw;
	_ahrs_ekf_gsf[model_index].R(1, 2) = R_prev02 * sinYaw + _ahrs_ekf_gsf[model_index].R(1, 2) * cosYaw;

	return true;
}

void EKFGSF_yaw::initialiseEKFGSF()
{
	_gsf_yaw = 0.0f;
	_ekf_gsf_vel_fuse_started = false;
	_gsf_yaw_variance = _m_pi2 * _m_pi2;
	_model_weights.setAll(1.0f / (float)N_MODELS_EKFGSF);  // All filter models start with the same weight

	memset(&_ekf_gsf, 0, sizeof(_ekf_gsf));
	const float yaw_increment = 2.0f * _m_pi / (float)N_MODELS_EKFGSF;
	for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index++) {
		// evenly space initial yaw estimates in the region between +-Pi
		_ekf_gsf[model_index].X(2) = -_m_pi + (0.5f * yaw_increment) + ((float)model_index * yaw_increment);

		// take velocity states and corresponding variance from last measurement
		_ekf_gsf[model_index].X(0) = _vel_NE(0);
		_ekf_gsf[model_index].X(1) = _vel_NE(1);
		_ekf_gsf[model_index].P(0,0) = sq(_vel_accuracy);
		_ekf_gsf[model_index].P(1,1) = _ekf_gsf[model_index].P(0,0);

		// use half yaw interval for yaw uncertainty
		_ekf_gsf[model_index].P(2,2) = sq(0.5f * yaw_increment);
	}
}

float EKFGSF_yaw::gaussianDensity(const uint8_t model_index) const
{
	// calculate transpose(innovation) * inv(S) * innovation
	const float normDist = _ekf_gsf[model_index].innov.dot(_ekf_gsf[model_index].S_inverse * _ekf_gsf[model_index].innov);

	return _m_2pi_inv * sqrtf(_ekf_gsf[model_index].S_det_inverse) * expf(-0.5f * normDist);
}

void EKFGSF_yaw::updateInnovCovMatInv(const uint8_t model_index, const matrix::SquareMatrix<float, 2> &S)
{
	// calculate determinant for innovation covariance matrix
	const float t2 = S(0,0) * S(1,1);
	const float t5 = S(0,1) * S(1,0);
	const float t3 = t2 - t5;

	// calculate determinant inverse and protect against badly conditioned matrix
	_ekf_gsf[model_index].S_det_inverse = 1.0f / fmaxf(t3 , 1e-12f);

	// calculate inv(S)
	_ekf_gsf[model_index].S_inverse(0,0) =   _ekf_gsf[model_index].S_det_inverse * S(1,1);
	_ekf_gsf[model_index].S_inverse(1,1) =   _ekf_gsf[model_index].S_det_inverse * S(0,0);
	_ekf_gsf[model_index].S_inverse(0,1) = - _ekf_gsf[model_index].S_det_inverse * S(0,1);
	_ekf_gsf[model_index].S_inverse(1,0) = - _ekf_gsf[model_index].S_det_inverse * S(1,0);

}

bool EKFGSF_yaw::getLogData(float *yaw_composite, float *yaw_variance, float yaw[N_MODELS_EKFGSF], float innov_VN[N_MODELS_EKFGSF], float innov_VE[N_MODELS_EKFGSF], float weight[N_MODELS_EKFGSF])
{
	if (_ekf_gsf_vel_fuse_started) {
		*yaw_composite = _gsf_yaw;
		*yaw_variance = _gsf_yaw_variance;
		for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index++) {
			yaw[model_index] = _ekf_gsf[model_index].X(2);
			innov_VN[model_index] = _ekf_gsf[model_index].innov(0);
			innov_VE[model_index] = _ekf_gsf[model_index].innov(1);
			weight[model_index] = _model_weights(model_index);
		}
		return true;
	}
	return false;
}

Dcmf EKFGSF_yaw::taitBryan312ToRotMat(const Vector3f &rot312)
{
	// Calculate the frame2 to frame 1 rotation matrix from a 312 rotation sequence
	const float c2 = cosf(rot312(2));
	const float s2 = sinf(rot312(2));
	const float s1 = sinf(rot312(1));
	const float c1 = cosf(rot312(1));
	const float s0 = sinf(rot312(0));
	const float c0 = cosf(rot312(0));

	Dcmf R;
	R(0, 0) = c0 * c2 - s0 * s1 * s2;
	R(1, 1) = c0 * c1;
	R(2, 2) = c2 * c1;
	R(0, 1) = -c1 * s0;
	R(0, 2) = s2 * c0 + c2 * s1 * s0;
	R(1, 0) = c2 * s0 + s2 * s1 * c0;
	R(1, 2) = s0 * s2 - s1 * c0 * c2;
	R(2, 0) = -s2 * c1;
	R(2, 1) = s1;

	return R;
}

float EKFGSF_yaw::ahrsCalcAccelGain() const
{
	// Calculate the acceleration fusion gain using a continuous function that is unity at 1g and zero
	// at the min and max g value. Allow for more acceleration when flying as a fixed wing vehicle using centripetal
	// acceleration correction as higher and more sustained g will be experienced.
	// Use a quadratic instead of linear function to prevent vibration around 1g reducing the tilt correction effectiveness.
	// see https://www.desmos.com/calculator/dbqbxvnwfg

	float attenuation = 2.f;
	const bool centripetal_accel_compensation_enabled = (_true_airspeed > FLT_EPSILON);

	if (centripetal_accel_compensation_enabled
	    && _ahrs_accel_norm > CONSTANTS_ONE_G) {
		attenuation = 1.f;
	}

	const float delta_accel_g = (_ahrs_accel_norm - CONSTANTS_ONE_G) / CONSTANTS_ONE_G;
	return _tilt_gain * sq(1.f - math::min(attenuation * fabsf(delta_accel_g), 1.f));
}

Matrix3f EKFGSF_yaw::ahrsPredictRotMat(const Matrix3f &R, const Vector3f &g)
{
	Matrix3f ret = R;
	ret(0,0) += R(0,1) * g(2) - R(0,2) * g(1);
	ret(0,1) += R(0,2) * g(0) - R(0,0) * g(2);
	ret(0,2) += R(0,0) * g(1) - R(0,1) * g(0);
	ret(1,0) += R(1,1) * g(2) - R(1,2) * g(1);
	ret(1,1) += R(1,2) * g(0) - R(1,0) * g(2);
	ret(1,2) += R(1,0) * g(1) - R(1,1) * g(0);
	ret(2,0) += R(2,1) * g(2) - R(2,2) * g(1);
	ret(2,1) += R(2,2) * g(0) - R(2,0) * g(2);
	ret(2,2) += R(2,0) * g(1) - R(2,1) * g(0);

	// Renormalise rows
	float rowLengthSq;
	for (uint8_t r = 0; r < 3; r++) {
		rowLengthSq = ret.row(r).norm_squared();
		if (rowLengthSq > FLT_EPSILON) {
			// Use linear approximation for inverse sqrt taking advantage of the row length being close to 1.0
			const float rowLengthInv = 1.5f - 0.5f * rowLengthSq;
			ret(r,0) *= rowLengthInv;
			ret(r,1) *= rowLengthInv;
			ret(r,2) *= rowLengthInv;
		}
        }

	return ret;
}

bool EKFGSF_yaw::getYawData(float *yaw, float *yaw_variance)
{
	if(_ekf_gsf_vel_fuse_started) {
		*yaw = _gsf_yaw;
		*yaw_variance = _gsf_yaw_variance;
		return true;
	}
	return false;
}

void EKFGSF_yaw::setVelocity(const Vector2f &velocity, float accuracy)
{
	_vel_NE = velocity;
	_vel_accuracy = accuracy;
	_vel_data_updated = true;
}
