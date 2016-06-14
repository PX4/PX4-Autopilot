/**
 * @file av_estimator.h
 * Filter parent class header file
 *
 * @author frits.kuipers <f.p.kuipers@student.utwente.nl>
 */

#include <px4_eigen.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_velocity_est_inertial.h>
#include <uORB/topics/vehicle_velocity_meas_inertial.h>
#include <uORB/topics/vehicle_velocity_meas_est_body.h>
#include <systemlib/err.h>

#ifdef __cplusplus
extern "C" {
#endif
#include "av_estimator_params.h"
#ifdef __cplusplus
}
#endif

#ifndef AV_ESTIMATOR_H
#define AV_ESTIMATOR_H

using namespace Eigen;

class av_estimator
{
protected:
	const float g 		= 9.80665f;
	const Vector3f e3 	= {0.0f,0.0f,1.0f};

	/* Gyroscope and Accelerometer bias */
	Vector3f beta_w_dot 	= Vector3f::Zero();
	Vector3f beta_a_dot 	= Vector3f::Zero();
	Vector3f beta_w_prev 	= Vector3f::Zero();
	Vector3f beta_a_prev 	= Vector3f::Zero();

	/* Velocity variables */
	Vector3f verror 	= Vector3f::Zero();
	Vector3f vhat_prev 	= Vector3f::Zero();
	Vector3f vhat_dot 	= Vector3f::Zero();

	/* Rotation and scaled latent rotation */
	Matrix3f Rhat_dot 	= Matrix3f::Identity();
	Matrix3f Rhat_prev 	= Matrix3f::Identity();
	Matrix3f X 		= Matrix3f::Identity();

	/* Initial magnetic field */
	Vector3f mu_init 	= Vector3f::Zero();

	/* Scaling factor */
	float u 	= 1.0f;
	float u_prev 	= 1.0f;
	float udot 	= 1.0f;

public:
	/* States */
	Vector3f beta_w 	= Vector3f::Zero();
	Vector3f beta_a 	= Vector3f::Zero();
	Matrix3f Rhat 		= Matrix3f::Identity();
	Vector3f vhat 		= Vector3f::Zero();

	/**
	 * @brief Initialiazes the filter
	 * @details Initializes the filter by calculating the initial Rotation matrix.
	 * 
	 * @param a initial acceleration measurement from IMU.
	 * @param mu initial magnetometer measurement.
	 */
	void init(Vector3f &a, Vector3f &mu);

	/**
	 * @brief Orthanormalizes matrix.
	 * 
	 * @param mat matrix to orthanormalize.
	 */
	void orthonormalize(Matrix3f & mat);

	/**
	 * @brief Creates a skew symetric matrix from a vector.
	 * 
	 * @param a vector.
	 * @return skew-symetric matrix.
	 */
	Matrix3f skew(Vector3f a);
};

#endif










