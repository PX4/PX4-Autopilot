/****************************************************************************
 *
 *   @author Rohan Inamdar <rninamdar@wpi.edu>
 *
 ****************************************************************************/

#pragma once

#include <matrix/matrix/math.hpp>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/esc_status.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_controls.h>
#include <lib/mathlib/math/filter/LowPass2p.hpp>  // PX4’s 2nd-order Butterworth class

using namespace time_literals;

class INDIController : public ModuleBase<INDIController>, public ModuleParams, public px4::WorkItem
{
public:
	INDIController(bool vtol = false);
	~INDIController() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	// ------------------------------------------------------------------------
	// 1) Subscriptions (uORB)
	// ------------------------------------------------------------------------
	uORB::Subscription	_vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription	_vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription	_gyro_sub{ORB_ID(sensor_gyro)};
	uORB::Subscription	_esc_sub{ORB_ID(esc_status)};

	sensor_gyro_s gyro{};
	esc_status_s esc{};
	actuator_controls_s actuator_sp{};

	// ------------------------------------------------------------------------
	// 2) Publication
	// ------------------------------------------------------------------------
	uORB::Publication<actuator_controls_s>	_actuator_pub{ORB_ID(actuator_controls_0)};

	// ------------------------------------------------------------------------
	// 3) Internal state: attitude, desired rates, filtered gyro, etc.
	// ------------------------------------------------------------------------
	matrix::Quatf		_q;          // current attitude from vehicle_attitude
	matrix::Vector3f	_Omega_f;    // filtered body-rates (p, q, r)
	matrix::Vector3f	_Omega_dot_f;// filtered body-acc (d(p,q,r)/dt)
	matrix::Vector3f	_Omega_prev; // last step’s filtered gyro
	matrix::Vector3f	_Omega_dot_prev;

	matrix::Vector4f	_omega_f;    // filtered motor RPM (if you have esc_status)
	matrix::Vector4f	_omega_dot_f; // filtered derivative of motor RPM
	matrix::Vector4f	_omega_prev;
	matrix::Vector4f	_omega_dot_prev;

	matrix::Vector4f	_omega_c;    // commanded motor RPM (output)

	// Desired body-rate setpoint:
	matrix::Quatf _q_sp;



	// ------------------------------------------------------------------------
	// 4) Filters
	// ------------------------------------------------------------------------
	// PX4’s LowPass2p filter: second-order Butterworth with given cutoff/wn, damping
	math::LowPass2p _gyro_filter[3];  // axis-by-axis for raw gyro
	math::LowPass2p _rpm_filter[4];   // axis-by-axis for esc_status.rpm

	// ------------------------------------------------------------------------
	// 5) Adaptive INDI internals (LMS)
	// ------------------------------------------------------------------------
	// Combined “control‐effectiveness matrix” G (3×8 = [G1·diag(ω) ,  G2 ]). Initialize to a small guess.
	matrix::Matrix<float,3,8>	_G;

	matrix::Matrix<float,3,4>  _G1_base;       // G1 constants computed at init
	matrix::Matrix<float,3,4>  _G1_diag;       // reused each cycle

	//NOTE: Values gotten from /Tools/simulation/gz/models/x500/x500_base/model.sdf
	// Inertial matrix calculated using parallel axis theorem and python script to combine base w/ rotors
	const float m = 2 + 4 * 0.016076923076923075;          // quad mass [kg]
	const float ct = 8.54858e-6;          // thrust coefficient (from your param)
	const float cq = 0.016f;       // torque coeff
	const float l  = 0.174f;         // "half" arm length, length in axis components [m]
	const float b  = 0.174f;         // "half" arm length, length in axis components [m]
	const float Ip = 2.649868234714004e-5;        // rotor inertia [kg·m²] (also Ir_zz, rotors inertia around spinn axis)
	const float Ivx = 0.0238466927,
	 	    Ivy = 0.0239496175,
		    Ivz = 0.0439999537; // vehicle inertia
	float _max_rpm = 10000;

	// LMS adaptation gains: (for later implementation)
	float	_mu2_roll{1e-4f};
	float	_mu2_pitch{1e-4f};
	float	_mu2_yaw{1e-4f};

	// 8 “mu1” gains: one per column of G
	float	_mu1[8] = {1e-5f,1e-5f,1e-5f,1e-5f,  1e-5f,1e-5f,1e-5f,1e-5f};

	bool	_first_gyro_sample{true};
	bool	_first_rpm_sample{true};

	// time of last gyro sample, for dt
	hrt_abstime	_last_gyro_time{0};

	// ------------------------------------------------------------------------
	// 6) Parameter definitions (PX4 param framework)
	// ------------------------------------------------------------------------
	DEFINE_PARAMETERS(
		// Can tweak these at runtime with `param set INDI_MU2_ROLL 0.00005`
		(ParamFloat<px4::params::INDI_MU2_ROLL>)	_param_indi_mu2_roll,
		(ParamFloat<px4::params::INDI_MU2_PITCH>)	_param_indi_mu2_pitch,
		(ParamFloat<px4::params::INDI_MU2_YAW>)	_param_indi_mu2_yaw,

		// mu1 values (eight of them).
		(ParamFloat<px4::params::INDI_MU1_0>)		_param_indi_mu1_0,
		(ParamFloat<px4::params::INDI_MU1_1>)		_param_indi_mu1_1,
		(ParamFloat<px4::params::INDI_MU1_2>)		_param_indi_mu1_2,
		(ParamFloat<px4::params::INDI_MU1_3>)		_param_indi_mu1_3,
		(ParamFloat<px4::params::INDI_MU1_4>)		_param_indi_mu1_4,
		(ParamFloat<px4::params::INDI_MU1_5>)		_param_indi_mu1_5,
		(ParamFloat<px4::params::INDI_MU1_6>)		_param_indi_mu1_6,
		(ParamFloat<px4::params::INDI_MU1_7>)		_param_indi_mu1_7,

		// Filter cutoff (rad/s) and damping, so you can tune per‐axis if needed
		(ParamFloat<px4::params::INDI_GYRO_WN>)	_param_indi_gyro_wn,
		(ParamFloat<px4::params::INDI_GYRO_Z>)	_param_indi_gyro_z,

		(ParamFloat<px4::params::INDI_RPM_WN>)	_param_indi_rpm_wn,
		(ParamFloat<px4::params::INDI_RPM_Z>)	_param_indi_rpm_z,

		(ParamFloat<px4::params::INDI_INNER_LOOP_FREQUENCY>)	param_indi_inner_loop_f,

		// Gains for PD tilt prioritized control
		(ParamFloat<px4::params::INDI_KP_ROLL>)		_param_indi_kp_roll,
		(ParamFloat<px4::params::INDI_KP_PITCH>)	_param_indi_kp_pitch,
		(ParamFloat<px4::params::INDI_KP_YAW>)		_param_indi_kp_yaw,

		// MAX Motor RPM
		(ParamFloat<px4::params::INDI_MAX_MOTOR_RPM>)	_param_indi_max_rpm
	)

	// ------------------------------------------------------------------------
	// 7) Private helper functions
	// ------------------------------------------------------------------------
	// Called once at module startup to set up filters and initial G matrix
	void parameters_updated();

	// Called each 2.5ms (400 Hz). All the “INDI math” happens here.
	void update();

	// Build “nu” = desired body‐angular‐acceleration (3×1) from (q, Omega, and (p_des,q_des,r_des))
	matrix::Vector3f computeNU();

	// Solve for delta‐RPM = pseudoinv( G_eff ) * (nu – Omega_dot_f)
	void computeINIDelta(const matrix::Vector3f &nu);

	// LMS update to adapt G
	void updateAdaptiveLMS();

};
