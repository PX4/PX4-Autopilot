/**
 * Implementation of a sigmoidal shear EKF for estimating the shear parameters of the wind.
 * The estimate is then used by the dynmic soaring controller in "fw_dyn_soar_control".
 *
 * @author Marvin Harms <marv@teleport.ch>
 */

// use inclusion guards
#ifndef FIXEDWINGSHEARESTIMATOR_HPP_
#define FIXEDWINGSHEARESTIMATOR_HPP_

#include <float.h>
#include <math.h>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/soaring_controller_wind.h>
#include <uORB/topics/soaring_estimator_shear.h>
#include <uORB/uORB.h>

using namespace time_literals;

using matrix::Dcmf;
using matrix::Quatf;
using matrix::Vector;
using matrix::Matrix;
using matrix::Matrix3f;
using matrix::Vector3f;


class FixedwingShearEstimator final : public ModuleBase<FixedwingShearEstimator>, public ModuleParams,
	public px4::WorkItem
{
public:
	FixedwingShearEstimator();
	~FixedwingShearEstimator() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	orb_advert_t	_mavlink_log_pub{nullptr};

	// make the main task run, whenever a new body rate becomes available
	uORB::SubscriptionCallbackWorkItem _soaring_controller_wind_sub{this, ORB_ID(soaring_controller_wind)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

    // Subscriptions
    //uORB::Subscription _soaring_controller_wind_sub{ORB_ID(soaring_controller_wind)};
	
    // Publishers
	uORB::Publication<soaring_estimator_shear_s> _soaring_estimator_shear_pub{ORB_ID(soaring_estimator_shear)};

    // Message structs
	soaring_estimator_shear_s	_soaring_estimator_shear{};	///< soaring controller pos 
	soaring_controller_wind_s	_soaring_controller_wind{};	///< soaring controller wind

	// parameter struct
	DEFINE_PARAMETERS(
		// aircraft params
		(ParamFloat<px4::params::DS_SIGMA_Q_V>) _param_sigma_q_vel,
        (ParamFloat<px4::params::DS_SIGMA_Q_H>) _param_sigma_q_h,
        (ParamFloat<px4::params::DS_SIGMA_Q_A>) _param_sigma_q_a,
        (ParamFloat<px4::params::DS_SIGMA_R_V>) _param_sigma_r_vel,
        (ParamFloat<px4::params::DS_INIT_H>) _param_init_h
	)


	perf_counter_t	_loop_perf;				///< loop performance counter
	
	// Update our local parameter cache.
	int		parameters_update();
    void    reset_filter();
    void    perform_prior_update();
    void    perform_posterior_update(float height, Vector3f wind);
    bool    check_plausibility();
    void    publish_estimate();
    void	status_publish();

    // control variables
    hrt_abstime _last_run{0};
    uint _reset_counter = {};
    const static size_t _dim_vertical = 2;  // order of vertical approximation function for vertical wind

	Vector<float, 6> _X_prior_horizontal= {};
    Matrix<float, 6, 6> _P_prior_horizontal = {};
    Vector<float, 6> _X_posterior_horizontal= {};	
    Matrix<float, 6, 6> _P_posterior_horizontal = {};
    Matrix<float, 6, 6> _Q_horizontal = {};
    Matrix<float, 2, 2> _R_horizontal = {};
    Matrix<float, 2, 6> _H_horizontal = {};
    Matrix<float, 6, 6> _A_horizontal = {};
    Matrix<float, 6, 2> _K_horizontal = {};


    Vector<float, _dim_vertical> _X_prior_vertical= {};
    Matrix<float, _dim_vertical, _dim_vertical> _P_prior_vertical = {};
    Vector<float, _dim_vertical> _X_posterior_vertical= {};
    Matrix<float, _dim_vertical, _dim_vertical> _P_posterior_vertical = {};
    Vector<float, _dim_vertical> _X_vertical = {};			// params of vertical wind
    Matrix<float, _dim_vertical, _dim_vertical> _Q_vertical = {};
    Matrix<float, 1, 1>  _R_vertical = {};
    Matrix<float, 1, _dim_vertical> _H_vertical = {};
    Matrix<float, _dim_vertical, _dim_vertical> _A_vertical = {};
    Matrix<float, _dim_vertical, 1> _K_vertical = {};

    // measurement variables
    Vector3f _current_wind = {};
    float _current_height = {};

    // helper variables
    float _init_height = {};

};


#endif // FIXEDWINGSHEARESTIMATOR_HPP_