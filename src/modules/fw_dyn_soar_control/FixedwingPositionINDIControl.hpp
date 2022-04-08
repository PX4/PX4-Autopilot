/**
 * Implementation of a generic incremental position controller 
 * using incremental nonlinear dynamic inversion and differential flatness
 * for a fixed wing aircraft during dynamic soaring cycles.
 * The controller directly outputs actuator deflections for ailerons, elevator and rudder.
 *
 * @author Marvin Harms <marv@teleport.ch>
 */

// use inclusion guards
#ifndef FIXEDWINGPOSITIONINDICONTROL_HPP_
#define FIXEDWINGPOSITIONINDICONTROL_HPP_

#include <float.h>

#include <vector>
#include <array>
#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <lib/l1/ECL_L1_Pos_Controller.hpp>
#include <lib/npfg/npfg.hpp>
#include <lib/tecs/TECS.hpp>
#include <lib/landing_slope/Landingslope.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/airflow_aoa.h>
#include <uORB/topics/airflow_slip.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_angular_acceleration.h>
#include <uORB/topics/vehicle_angular_acceleration_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/wind.h>
#include <uORB/uORB.h>


using namespace time_literals;

using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector;
using matrix::Matrix3f;
using matrix::Vector3f;


class FixedwingPositionINDIControl final : public ModuleBase<FixedwingPositionINDIControl>, public ModuleParams,
	public px4::WorkItem
{
public:
	FixedwingPositionINDIControl();
	~FixedwingPositionINDIControl() override;

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

	uORB::SubscriptionCallbackWorkItem _local_pos_sub{this, ORB_ID(vehicle_local_position)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

    // Subscriptions
	uORB::Subscription _airspeed_validated_sub{ORB_ID(airspeed_validated)};             // airspeed 
    uORB::Subscription _airflow_aoa_sub{ORB_ID(airflow_aoa)};                           // angle of attack
    uORB::Subscription _airflow_slip_sub{ORB_ID(airflow_slip)};                         // angle of sideslip
    uORB::Subscription _vehicle_global_position_sub{ORB_ID(vehicle_global_position)};   // global position
    uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};     // local NED position
    uORB::Subscription _vehicle_odometry_sub{ORB_ID(vehicle_odometry)};                 // vehicle velocity
    uORB::Subscription _vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};         // vehicle acceleration
    uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};                 // vehicle attitude
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)}; // vehicle body rates
    uORB::Subscription _vehicle_angular_acceleration_sub{ORB_ID(vehicle_angular_acceleration)}; // vehicle body accel
	

    // Publishers
	uORB::Publication<vehicle_angular_acceleration_setpoint_s>		_alpha_sp_pub;
	
    // Message structs
	vehicle_angular_acceleration_setpoint_s _angular_accel_sp {};
	manual_control_setpoint_s	_manual_control_setpoint {};			///< r/c channel data
	vehicle_control_mode_s		_control_mode {};		///< control mode
	vehicle_local_position_s	_local_pos {};			///< vehicle local position
	vehicle_status_s		    _vehicle_status {};		///< vehicle status

	// parameter struct
	DEFINE_PARAMETERS(
		// aircraft params
		(ParamFloat<px4::params::FW_MASS>) _param_fw_mass,
		(ParamFloat<px4::params::FW_WING_AREA>) _param_fw_wing_area,
		(ParamFloat<px4::params::RHO>) _param_rho,
		// aerodynamic params

		(ParamFloat<px4::params::C_L0>) _param_fw_c_l0,
		(ParamFloat<px4::params::C_L1>) _param_fw_c_l1,
		(ParamFloat<px4::params::C_D0>) _param_fw_c_d0,
		(ParamFloat<px4::params::C_D1>) _param_fw_c_d1,
		(ParamFloat<px4::params::C_D2>) _param_fw_c_d2,
		// filter params
		(ParamFloat<px4::params::FILTER_A1>) _param_filter_a1,
		(ParamFloat<px4::params::FILTER_A2>) _param_filter_a2,
		(ParamFloat<px4::params::FILTER_B1>) _param_filter_b1,
		(ParamFloat<px4::params::FILTER_B2>) _param_filter_b2,
		(ParamFloat<px4::params::FILTER_B3>) _param_filter_b3,
		// controller params
		(ParamFloat<px4::params::K_X_ROLL>) _param_k_x_roll,
		(ParamFloat<px4::params::K_X_PITCH>) _param_k_x_pitch,
		(ParamFloat<px4::params::K_X_YAW>) _param_k_x_yaw,
		(ParamFloat<px4::params::K_V_ROLL>) _param_k_v_roll,
		(ParamFloat<px4::params::K_V_PITCH>) _param_k_v_pitch,
		(ParamFloat<px4::params::K_V_YAW>) _param_k_v_yaw,
		(ParamFloat<px4::params::K_A_ROLL>) _param_k_a_roll,
		(ParamFloat<px4::params::K_A_PITCH>) _param_k_a_pitch,
		(ParamFloat<px4::params::K_A_YAW>) _param_k_a_yaw,
		(ParamFloat<px4::params::K_Q_ROLL>) _param_k_q_roll,
		(ParamFloat<px4::params::K_Q_PITCH>) _param_k_q_pitch,
		(ParamFloat<px4::params::K_Q_YAW>) _param_k_q_yaw,
		(ParamFloat<px4::params::K_W_ROLL>) _param_k_w_roll,
		(ParamFloat<px4::params::K_W_PITCH>) _param_k_w_pitch,
		(ParamFloat<px4::params::K_W_YAW>) _param_k_w_yaw

	)


	perf_counter_t	_loop_perf;				///< loop performance counter

	// estimator reset counters
	uint8_t _pos_reset_counter{0};				///< captures the number of times the estimator has reset the horizontal position
	uint8_t _alt_reset_counter{0};				///< captures the number of times the estimator has reset the altitude state

	
	// Update our local parameter cache.
	int		parameters_update();

	// Update subscriptions
	void		airspeed_poll();
	void		control_update();
	void 		manual_control_setpoint_poll();
	void		vehicle_attitude_poll();
	void		vehicle_command_poll();
	void		vehicle_control_mode_poll();
	void		vehicle_status_poll();
	void        wind_poll();

	//
	void		status_publish();

	const int _num_points = 30;				// number of points on the precomputed trajectory
	const static size_t _num_basis_funs = 15;			// number of basis functions used for the trajectory approximation

	// controller methods
	void _set_wind_estimate(Vector3f wind);
	float _get_closest_t(Vector3f pos);				// get the normalized time, at which the reference path is closest to the current position
	Vector<float, _num_basis_funs> _get_basis_funs(float t=0);			// compute the vector of basis functions at normalized time t in [0,1]
	Vector<float, _num_basis_funs> _get_d_dt_basis_funs(float t=0);	// compute the vector of basis function gradients at normalized time t in [0,1]
	Vector<float, _num_basis_funs> _get_d2_dt2_basis_funs(float t=0);	// compute the vector of basis function curvatures at normalized time t in [0,1]
	void _load_basis_coefficients();		// load the coefficients of the current path approximation
	Vector3f _get_position_ref(float t=0);	// get the reference position on the current path, at normalized time t in [0,1]
	Vector3f _get_velocity_ref(float t=0, float T=1);	// get the reference velocity on the current path, at normalized time t in [0,1], with an intended cycle time of T
	Vector3f _get_acceleration_ref(float t=0, float T=1);	// get the reference acceleration on the current path, at normalized time t in [0,1], with an intended cycle time of T
	Quatf _get_attitude_ref(float t=0, float T=1);	// get the reference attitude on the current path, at normalized time t in [0,1], with an intended cycle time of T
	Vector3f _get_angular_velocity_ref(float t=0, float T=1);	// get the reference angular velocity on the current path, at normalized time t in [0,1], with an intended cycle time of T
	Vector3f _get_angular_acceleration_ref(float t=0, float T=1);	// get the reference angular acceleration on the current path, at normalized time t in [0,1], with an intended cycle time of T
	Quatf _get_attitude(Vector3f vel, Vector3f f);	// get the attitude to produce force f while flying with velocity vel
	void _compute_NDI_control_input(Vector3f pos, Vector3f vel, Vector3f acc, Quatf att, Vector3f omega, Vector3f alpha);
	void _compute_INDI_control_input(Vector3f pos, Vector3f vel, Vector3f acc, Quatf att, Vector3f omega, Vector3f alpha);

	// control variables
	Vector<float, _num_basis_funs> _basis_coeffs_x;				// coefficients of the current path
	Vector<float, _num_basis_funs> _basis_coeffs_y;				// coefficients of the current path
	Vector<float, _num_basis_funs> _basis_coeffs_z;				// coefficients of the current path
	/*
	Vector3f _pos;
	Vector3f _pos_sp;
	Vector3f _vel;
	Vector3f _vel_sp;
	Vector3f _acc;
	Vector3f _acc_sp;
	Quatf _att;
	Quatf _att_sp;
	Vector3f _omega;
	Vector3f _omega_sp;
	Vector3f _alpha;
	*/
	Vector3f _alpha_sp;
	Vector3f _wind_estimate;
	Matrix3f _K_x;
	Matrix3f _K_v;
	Matrix3f _K_a;
	Matrix3f _K_q;
	Matrix3f _K_w;

	// filter variables
	std::array<Vector3f, 3> _f_list;
	std::array<Vector3f, 3> _a_list;
	std::array<Vector3f, 3> _f_lpf_list;
	std::array<Vector3f, 3> _a_lpf_list;

};



#endif // FIXEDWINGPOSITIONINDICONTROL_HPP_