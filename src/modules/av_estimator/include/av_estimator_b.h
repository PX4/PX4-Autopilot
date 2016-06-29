/**
 * @file av_estimator_b.h
 * Filter class header file
 *
 * @author frits.kuipers <f.p.kuipers@student.utwente.nl>
 */

#include <px4_eigen.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_velocity_est_inertial.h>
#include <uORB/topics/vehicle_velocity_meas_inertial.h>
#include <uORB/topics/vehicle_velocity_meas_est_body.h>
#include <uORB/topics/vehicle_vicon_position.h>
#include <uORB/uORB.h>
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

class av_estimator_b : public av_estimator
{
private:
	/* Rotation rate */
	Vector3f omega 		= Vector3f::Zero();

	/* e1 */
	const Vector3f e1 = {1.0f, 0.0f, 0.0f};

	/* Gains */
	float k1, k3, k5, k6, k7, k8, k1vicon, k1vc;

	
	vehicle_velocity_est_inertial_s vel;

	orb_advert_t att_pub;
	orb_advert_t vel_pub = nullptr;

	uint64_t viconTimestampPrev = 0;

public:
	vehicle_attitude_s att;
	Vector3f strapdown_est_vhat  = Vector3f::Zero();
	Vector3f beta_a_filterb = Vector3f::Zero();
	
	float scaling_u = 0.0f;
	av_estimator_b();

	/**
	 * @brief update the filter with new measurements
	 * 
	 * @param a accelerometer measurement (m/(s^2).
	 * @param w gyroscope measurement (rad/s).
	 * @param vbar speed measurement (m/s).
	 * @param mu magnetometer measurement (mgauss).
	 * @param attitude_params attititude params containing the gains of the filter.
	 * @param dt timestep
	 */
	void update(Vector3f &a, Vector3f &w, Vector3f &vbar, Vector3f &mu, vehicle_vicon_position_s &vicon, av_estimator_params  attitude_params, Vector3f &vhat_in, Vector3f &what, bool valid, float &dt);
	

	//updateConverge();

	/**
	 * @brief Publish the results of the filter in the appropriate topics.
	 * 
	 * @param timestamp timestamp of the published data
	 * @param att attitude container containing all the attitudes.
	 * @param att_pub uORB publish structure.
	 * @param valid indicates filter a is valid
	 */
	void publish(float timestamp, Vector3f vhat_a, Vector3f wind_vel, bool valid);
};

























