/**
 * @file av_estimator_a.h
 * Filter class header file
 *
 * @author frits.kuipers <f.p.kuipers@student.utwente.nl>
 */

#include <px4_eigen.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_vicon_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_velocity_meas_est_body.h>
#include <systemlib/err.h>
#include "av_estimator.h"

#ifdef __cplusplus
extern "C" {
#endif
#include "av_estimator_params.h"
#ifdef __cplusplus
}
#endif

using namespace Eigen;

class av_estimator_a : public av_estimator
{
private:
	/* dt for the velocity measurements, either VICON or GPS */
	float dt2 = 0.0f;
	uint64_t prev_gps_timestamp = 0;
	uint64_t prev_vel_timestamp = 0;

	/* Measured Vbar */
	Vector3f vbar_a = Vector3f::Zero();

	/* Extra integration variables, because of integration with differtent timesteps */
	Vector3f vhat_dot_2 = Vector3f::Zero();
	Matrix3f Rhat_dot_2 = Matrix3f::Zero();

	/* Variables for the wind */	
	Vector3f what_dot	= Vector3f::Zero();
	Vector3f what_prev	= Vector3f::Zero();

	/* Variable for filtering velocity of vicon */
	Vector3f vbar_a_prev	= Vector3f::Zero();

	/* Previous VICON position measurements for differentiating */
	float vicon_x_prev = 0;
	float vicon_y_prev = 0;
	float vicon_z_prev = 0;

	/* Gains */
	float k2v, k2w, k2ba, k2r, k2u, k2vc, k2rc, k2bac;

public:

	/**/
	bool valid = false;
	Vector3f what 		= Vector3f::Zero();

	/**
	 * @brief Update the filter with new measurements;
	 * 
	 * @param a accelerometer measurement (m/(s^2).
	 * @param w gyroscope measurement (rad/s).
	 * @param Rhat Rhat from filter b
	 * @param rawVicon raw VICON data.
	 * @param rawGPS raw GPS data.
	 * @param attitude_params attititude params containing the gains of the filter.
	 * @param dt timestep
	 */
	void update(Vector3f &a, Vector3f &w, Matrix3f &Rhat_b, vehicle_velocity_meas_inertial_s &rawVelocity, vehicle_gps_position_s &rawGPS, av_estimator_params  attitude_params, Vector3f &vhat_body, Vector3f &beta_a_filterb, float &dt);
	

	//updateConverge();

};