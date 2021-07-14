#ifndef EKF_EKFGSF_YAW_H
#define EKF_EKFGSF_YAW_H

#include <lib/geo/geo.h>
#include <matrix/math.hpp>
#include <mathlib/mathlib.h>

#include "common.h"
#include "utils.hpp"

using matrix::AxisAnglef;
using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Matrix3f;
using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector3f;
using matrix::wrap_pi;

static constexpr uint8_t N_MODELS_EKFGSF = 5;

// Required math constants
static constexpr float _m_2pi_inv = 0.159154943f;
static constexpr float _m_pi = 3.14159265f;
static constexpr float _m_pi2 = 1.57079632f;

using namespace estimator;

class EKFGSF_yaw
{
public:
    	EKFGSF_yaw();

    	// Update Filter States - this should be called whenever new IMU data is available
	void update(const imuSample &imu_sample,
			bool run_EKF,  			// set to true when flying or movement is suitable for yaw estimation
			float airspeed,			// true airspeed used for centripetal accel compensation - set to 0 when not required.
			const Vector3f &imu_gyro_bias); // estimated rate gyro bias (rad/sec)

	void setVelocity(const Vector2f &velocity, // NE velocity measurement (m/s)
			float accuracy);	   // 1-sigma accuracy of velocity measurement (m/s)

	// get solution data for logging
	bool getLogData(float *yaw_composite,
			float *yaw_composite_variance,
			float yaw[N_MODELS_EKFGSF],
			float innov_VN[N_MODELS_EKFGSF],
			float innov_VE[N_MODELS_EKFGSF],
			float weight[N_MODELS_EKFGSF]) const;

    	// get yaw estimate and the corresponding variance
    	// return false if no yaw estimate available
    	bool getYawData(float *yaw, float *yaw_variance) const;

private:

	// Parameters - these could be made tuneable
	const float _gyro_noise{1.0e-1f}; 	// yaw rate noise used for covariance prediction (rad/sec)
	const float _accel_noise{2.0f};		// horizontal accel noise used for covariance prediction (m/sec**2)
	const float _tilt_gain{0.2f};		// gain from tilt error to gyro correction for complementary filter (1/sec)
	const float _gyro_bias_gain{0.04f};	// gain applied to integral of gyro correction for complementary filter (1/sec)

	// Declarations used by the bank of N_MODELS_EKFGSF AHRS complementary filters

	Vector3f _delta_ang{};	// IMU delta angle (rad)
	Vector3f _delta_vel{};	// IMU delta velocity (m/s)
	float _delta_ang_dt{};	// _delta_ang integration time interval (sec)
	float _delta_vel_dt{};	// _delta_vel integration time interval (sec)
	float _true_airspeed{};	// true airspeed used for centripetal accel compensation (m/s)

	struct _ahrs_ekf_gsf_struct{
		Dcmf R;			// matrix that rotates a vector from body to earth frame
		Vector3f gyro_bias;	// gyro bias learned and used by the quaternion calculation
		bool aligned;		// true when AHRS has been aligned
		float vel_NE[2];	// NE velocity vector from last GPS measurement (m/s)
		bool fuse_gps;		// true when GPS should be fused on that frame
		float accel_dt;		// time step used when generating _simple_accel_FR data (sec)
	} _ahrs_ekf_gsf[N_MODELS_EKFGSF]{};

	bool _ahrs_ekf_gsf_tilt_aligned{};	// true the initial tilt alignment has been calculated
	float _ahrs_accel_fusion_gain{};	// gain from accel vector tilt error to rate gyro correction used by AHRS calculation
	Vector3f _ahrs_accel{};			// low pass filtered body frame specific force vector used by AHRS calculation (m/s/s)
	float _ahrs_accel_norm{};		// length of _ahrs_accel specific force vector used by AHRS calculation (m/s/s)

	// calculate the gain from gravity vector misalingment to tilt correction to be used by all AHRS filters
	float ahrsCalcAccelGain() const;

	// update specified AHRS rotation matrix using IMU and optionally true airspeed data
	void ahrsPredict(const uint8_t model_index);

	// align all AHRS roll and pitch orientations using IMU delta velocity vector
	void ahrsAlignTilt();

	// align all AHRS yaw orientations to initial values
	void ahrsAlignYaw();

	// Efficient propagation of a delta angle in body frame applied to the body to earth frame rotation matrix
	Matrix3f ahrsPredictRotMat(const Matrix3f &R, const Vector3f &g);

	// Declarations used by a bank of N_MODELS_EKFGSF EKFs

	struct _ekf_gsf_struct{
		matrix::Vector3f X; 				// Vel North (m/s),  Vel East (m/s), yaw (rad)s
		matrix::SquareMatrix<float, 3> P; 		// covariance matrix
		matrix::SquareMatrix<float, 2> S_inverse;	// inverse of the innovation covariance matrix
		float S_det_inverse; 				// inverse of the innovation covariance matrix determinant
		matrix::Vector2f innov; 			// Velocity N,E innovation (m/s)
	} _ekf_gsf[N_MODELS_EKFGSF]{};

	bool _vel_data_updated{};	// true when velocity data has been updated
	bool _run_ekf_gsf{};		// true when operating condition is suitable for to run the GSF and EKF models and fuse velocity data
	Vector2f _vel_NE{};        // NE velocity observations (m/s)
	float _vel_accuracy{};     // 1-sigma accuracy of velocity observations (m/s)
	bool _ekf_gsf_vel_fuse_started{}; // true when the EKF's have started fusing velocity data and the prediction and update processing is active

	// initialise states and covariance data for the GSF and EKF filters
	void initialiseEKFGSF();

	// predict state and covariance for the specified EKF using inertial data
	void predictEKF(const uint8_t model_index);

	// update state and covariance for the specified EKF using a NE velocity measurement
	// return false if update failed
	bool updateEKF(const uint8_t model_index);

	inline float sq(float x) const { return x * x; };

	// Declarations used by the Gaussian Sum Filter (GSF) that combines the individual EKF yaw estimates

	matrix::Vector<float, N_MODELS_EKFGSF> _model_weights{};
	float _gsf_yaw{}; 		// yaw estimate (rad)
	float _gsf_yaw_variance{}; 	// variance of yaw estimate (rad^2)

	// return the probability of the state estimate for the specified EKF assuming a gaussian error distribution
	float gaussianDensity(const uint8_t model_index) const;
};
#endif // !EKF_EKFGSF_YAW_H
