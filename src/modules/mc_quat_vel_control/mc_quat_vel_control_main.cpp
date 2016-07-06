/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file mc_quat_vel_control_main.cpp
 * Multicopter velocity and attitude controller.
 *
 * @author Moses Bangura <moses.bangura@anu.edu.au, dnovichman@hotmail.com>
 * 
 *		
 *			TODO describe this controller and modify theory and get it working
 * This controller uses quaternions and is based on controller theory presented in the paper
 * Bangura, Lim, Kim and Mahony, "An Open-Source Implementation of a Unit Quaternion based Attitude and Trajectory Tracking for 	  	Quadrotors", In ACRA, 2014.
 * 			TODO Change text below
 * The controller operates in two modes: manual and autonomous. In autonomous mode, the desired quaternion,  attitude rate and attitude 
 * acceleration are determined which are controlled by this controller. In manual mode, the controller regulates the desired quaternion
 * and desired feedforward terms (angular velocity and acceleration) which are zero. The controller is a PD controller on 
 * attitude/attitude ratte with the momentum of inertia (Kd = I) used as a feedforward only for autonomous mode. Though quaternions are
 * non-intuitive, the manual control handles this.
 *
 * Change header: this is typically controller in ACRA2015
 * Modes: Manual
 *		Default: quaternion control
 *		Velocity in x, y control of body velocities are available
 *	  Auto  Only when vehicle is in auto mode and velocities are valid
*/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/circuit_breaker.h>
#include <lib/mathlib/mathlib.h>
#include <lib/geo/geo.h>

#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/home_position.h>

#include <uORB/topics/vehicle_velocity_est_inertial.h>
#include <uORB/topics/vehicle_velocity_meas_inertial.h>
#include <uORB/topics/vehicle_velocity_meas_est_body.h>  
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_vicon_position.h>

#include <uORB/topics/rc_channels.h>

#include <px4_eigen.h>

 #define acc_grav 9.81f

using namespace Eigen;

/**
 * Quadroter quaternion based attitude and velocity controller app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mc_quat_vel_control_main(int argc, char *argv[]);


class MulticopterQuaternionVelControl
{
public:
	/**
	 * Constructor
	 */
	MulticopterQuaternionVelControl();

	/**
	 * Destructor, also kills the quaternion attitude task.
	 */
	~MulticopterQuaternionVelControl();

	/**
	 * Start the quaternion control task.
	 *
	 * @return		OK on success.
	 */
	int		start();

private:

	bool	_task_should_exit;			/**< if true, task should exit */
	int		_control_task;				/**< task handle for quaternion attitude task */

	int		_v_att_sub;					/**< vehicle attitude subscription */
	int		_v_att_sp_sub;				/**< vehicle attitude setpoint subscription */
	int		_v_rates_sp_sub;			/**< vehicle rates setpoint subscription */
	int		_v_control_mode_sub;		/**< vehicle control mode subscription */
	int		_params_sub;				/**< parameter updates subscription */
	int		_manual_control_sp_sub;		/**< manual control setpoint subscription */
	int		_armed_sub;					/**< arming status subscription */
	
	int  		_v_vel_sub;				/**< Subscribe to vehicle velocity measurements */
	int  		_v_lpos_sp_sub;			/**< Subscribe to vehicle local pos setpoint */
	int         _v_local_pos_sub;		/**< Subscribe to vehicle velocity */
	int         _v_vel_est_sub;			/**< Subscribe to vehicle est velo */
	int         _v_vel_body_est_sub;	/**< Subscribe to vehicle body frame est velocity */
	int 		_v_veh_vicon_pos_sub;	/**< Subscribe to vehicle vicon pos */

	int 		_v_rc_sub; //add description

	orb_advert_t	_att_sp_pub;			/**< attitude setpoint publication */
	orb_advert_t	_v_rates_sp_pub;		/**< rate setpoint publication */
	orb_advert_t	_actuators_0_pub;		/**< attitude actuator controls publication */

	orb_advert_t	_lpos_sp_pub;			/**< vehicle lpos sp publication */


	bool		_actuators_0_circuit_breaker_enabled;			/**< circuit breaker to suppress output */

	struct vehicle_attitude_s			_v_att;					/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s	_v_att_sp;				/**< vehicle attitude setpoint */
	struct vehicle_rates_setpoint_s		_v_rates_sp;			/**< vehicle rates setpoint */
	struct manual_control_setpoint_s	_manual_control_sp;		/**< manual control setpoint */
	struct vehicle_control_mode_s		_v_control_mode;		/**< vehicle control mode */
	struct actuator_controls_s			_actuators;				/**< actuator controls */
	struct actuator_armed_s				_armed;					/**< actuator arming status */

	struct vehicle_velocity_meas_inertial_s		_v_vel;				/**< vehicle velocity */
	struct vehicle_velocity_est_inertial_s		_v_vel_est;			/**< vehicle velocity estimated */
	struct vehicle_velocity_meas_est_body_s		_v_vel_body_est;	/**< vehicle velocity estimated */
	struct vehicle_local_position_setpoint_s _lpos_sp;			/**< Vehicle local position setpoint */
	struct vehicle_local_position_s 		_v_local_position;	/**< Vehicle local position */
	struct vehicle_vicon_position_s 		_v_vicon_position;	/**< Subscribe to vicon for landing in auto mode */

	struct rc_channels_s					_v_rc_channels;

	perf_counter_t	_loop_perf;				/**< loop performance counter */

	math::Vector<3>		_rates_prev;		/**< angular rates on previous step */
	math::Vector<3>		_rates_sp;			/**< angular rates setpoint */
	math::Vector<3>		_rates_int;			/**< angular rates integral error */
	float				_thrust_sp;			/**< thrust setpoint */
	math::Vector<3>		_att_control;		/**< attitude control vector */
	math::Vector<3>		_vel_setpoint;		/**< velocity setpoint */

	uint64_t old_vel_timestamp;
	
	float int_vx = 0.0f, int_vy = 0.0f, int_vz = 0.0f, dtt = 0.0f;
	float int_yawrate = 0.0f;
	bool vel_cont = false; //flag for yaw rate so that we use the correct dtt
	float q_d[4] = {1.0f, 0.0f, 0.0f, 0.0f};
	float q_d_inv[4] = {1.0f, 0.0f, 0.0f, 0.0f};
	float q_e[4] = {1.0f, 0.0f, 0.0f, 0.0f};
	float vx_err, vy_err, vz_err, z_omegaVd;
	bool takeoff = false;

	float Kp_velx = 0.0f, Ki_velx = 0.0f, Kp_vely = 0.0f, Ki_vely = 0.0f, Kp_velz = 0.0f, Ki_velz = 0.0f;
	float yr_max = 30.0f;
	bool change_control_gain = false;


	struct {
		param_t vx_p;
		param_t vx_i;
		param_t vy_p;
		param_t vy_i;
		param_t vz_p;
		param_t vz_i;
		param_t ixy_max;
		param_t iz_max;
		param_t drag_cbar;
		param_t vz_mg;
		param_t thr_max;
		param_t thr_min;
		param_t roll_p;
		param_t roll_rate_p;
		param_t roll_rate_d;
		param_t pitch_p;
		param_t pitch_rate_p;
		param_t pitch_rate_d;
		param_t yaw_p;
		param_t yaw_rate_p;
		param_t yaw_rate_d;
		param_t yaw_rate_i;

		param_t yawr_imax;

		param_t man_roll_max;
		param_t man_pitch_max;
		param_t man_yaw_max;
		param_t acro_roll_max;
		param_t acro_pitch_max;
		param_t acro_yaw_max;

		param_t man_vx_max;
		param_t man_vy_max;
	}		_params_handles;				/**< handles for interesting parameters */

	struct {
		math::Vector<3> att_p;				/**< P gain for attitude error */
		math::Vector<3> rate_p;				/**< P gain for attitude rate error */
		math::Vector<3> rate_d;				/**< D gain for attitude rate error */
		math::Vector<3> rate_i;				/**< I gain for attitude rate error */
		float yaw_rate_max;					/**< max yaw rate */

		math::Vector<3> vel_p;				/**< P gain for velocity error */
		math::Vector<3> vel_i;				/**< I gain for velocity error */
		float mg, thr_max, thr_min;					/**< mg of vehicle */

		float ixy_max, iz_max, cbar;		/** Integral saturation limits */

		float man_vx_max;					/**< Maximum vx for manual control */
		float man_vy_max;					/**< Maximum vy for manual control */
		float man_roll_max;					/**< Maximum roll for manual control */
		float man_pitch_max;				/**< Maximum pitch for manual control */
		float man_yaw_max;
		float yawr_imax;
		math::Vector<3> acro_rate_max;		/**< max attitude rates in acro mode */
	}		_params;

	/**
	 * Update our local parameter cache.
	 */
	int			parameters_update();

	/**
	 * Check for parameter update and handle it.
	 */
	void		parameter_update_poll();

	/**
	 * Check for changes in vehicle control mode.
	 */
	void		vehicle_control_mode_poll();

	/**
	 * Check for changes in RC.
	 */
	void		vehicle_rc_poll();	

	/**
	 * Check for changes in vehicle inertial velocity
	*/
	void 		vehicle_velocity_meas_inertial_poll();

	/**
	 * Check for changes in vehicle estimated velocity
	*/
	void 		vehicle_velocity_est_inertial_poll();

	/**
	 * Check for changes in vehicle body frame estimated velocity
	*/
	void 		vehicle_velocity_measured_body_poll();
	
	/**
	* Check for changes in vehicle local position setpoint
	*/
	void 		vehicle_lpos_setpoint_poll();

	/**
	* Check for changes in vehicle local position
	*/
	void 		vehicle_local_postion_poll();	

	/**
	* Check for changes in vehicle vicon position setpoint
	*/
	void 		vehicle_vicon_postion_poll();

	/**
	 * Check for changes in manual inputs.
	 */
	void		vehicle_manual_poll();

	/**
	 * Check for attitude setpoint updates.
	 */
	void		vehicle_attitude_setpoint_poll();

	/**
	 * Check for rates setpoint updates.
	 */
	void		vehicle_rates_setpoint_poll();

	/**
	 * Poll every uORB topic required.
	 */
	void		poll_subscriptions();

	/**
	  *	Reset gains so that when we are using body est, we can still fly
	*/
	void 		set_control_gains(bool gain_body_est);

	/**
	 * Check for arming status updates.
	 */
	void		arming_status_poll();

	/**
	 * Quaternion attitude controller.
	 */
	void		control_attitude(float dt);

	/**
	 * vehicle velocity controller.
	 */
	void		control_velocity(math::Vector<3> vel);

	/**
	*  Determine desired quaternion from x_con, y_con 
	*/
	void 		determine_desired_quat(float x_con, float y_con, float yaw_con, float *quat);

	/**
	*  Determine desired quaternion from x_con, y_con, yaw_des 
	*/
	void 		determine_desired_quat_Rd(float x_con, float y_con, float cos_sin, float yaw_con, float *quat);

	/**
	 * Attitude rates controller.
	 */
	void		control_attitude_rates(float dt);

	/**
	 * Quaternion compilation files.
	*/
	float 		invSqrt(float number);

	void 		getQuatFromRPY123(const float roll, const float pitch, const float yaw, float* qd);

	void 		quat_mult(const float *a,const float *b,float *prod);

	void 		quatConj(float *q, float *qconj);

	math::Vector<3>  do_rotation(math::Vector<3> vec, math::Vector<3> angle);
	math::Vector<3>  quat_rotation(math::Vector<3> vec, float *quatr);
	void 		quat_quat_rotation(float *quat_a, float *quat_b, float *quatr);
	void 		quat_norm(float *q);
	void  		control_vz(void);

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main quaternion control task.
	 */
	void		task_main();
};

namespace mc_quat_vel_control
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

MulticopterQuaternionVelControl	*g_control;
}

MulticopterQuaternionVelControl::MulticopterQuaternionVelControl() :

	_task_should_exit(false),
	_control_task(-1),

/* subscriptions */
	_v_att_sub(-1),
	_v_att_sp_sub(-1),
	_v_control_mode_sub(-1),
	_params_sub(-1),
	_manual_control_sp_sub(-1),
	_armed_sub(-1),
	_v_vel_sub(-1),
	_v_lpos_sp_sub(-1),
	_v_local_pos_sub(-1),
	_v_vel_est_sub(-1),
	_v_vel_body_est_sub(-1),
	_v_veh_vicon_pos_sub(-1),

/* publications */
	_att_sp_pub(nullptr),
	_v_rates_sp_pub(nullptr),
	_actuators_0_pub(nullptr),
	_lpos_sp_pub(nullptr),

	_actuators_0_circuit_breaker_enabled(false),

/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "mc_quat_vel_control"))

{
	memset(&_v_att, 0, sizeof(_v_att));
	memset(&_v_att_sp, 0, sizeof(_v_att_sp));
	memset(&_v_rates_sp, 0, sizeof(_v_rates_sp));
	memset(&_manual_control_sp, 0, sizeof(_manual_control_sp));
	memset(&_v_control_mode, 0, sizeof(_v_control_mode));
	memset(&_actuators, 0, sizeof(_actuators));
	memset(&_armed, 0, sizeof(_armed));


	memset(&_v_vel, 0, sizeof(_v_vel));
	memset(&_v_vel_est, 0, sizeof(_v_vel_est));
	memset(&_v_vel_body_est, 0, sizeof(_v_vel_body_est));
	memset(&_lpos_sp, 0, sizeof(_lpos_sp));
	memset(&_v_local_position, 0, sizeof(_v_local_position));
	memset(&_v_vicon_position, 0, sizeof(_v_vicon_position));

	_params.att_p.zero();
	_params.rate_p.zero();
	_params.rate_d.zero();
	_params.yaw_rate_max = 0.0f;
	_params.man_roll_max = 0.0f;
	_params.man_pitch_max = 0.0f;
	_params.man_yaw_max = 0.0f;
	_params.acro_rate_max.zero();

	_params.vel_p.zero();
	_params.vel_p.zero();
	_params.man_vx_max = 0.0f;
	_params.man_vy_max = 0.0f;

	_rates_prev.zero();
	_rates_sp.zero();
	_rates_int.zero();
	_thrust_sp = 0.0f;
	_att_control.zero();

	_params_handles.roll_p			= 	param_find("MC_QUAT_ROLL_P");
	_params_handles.roll_rate_p		= 	param_find("MC_QUAT_ROLLR_P");
	_params_handles.roll_rate_d		= 	param_find("MC_QUAT_ROLLR_D");
	_params_handles.pitch_p			= 	param_find("MC_QUAT_PITCH_P");
	_params_handles.pitch_rate_p	= 	param_find("MC_QUAT_PITCHR_P");
	_params_handles.pitch_rate_d	= 	param_find("MC_QUAT_PITCHR_D");
	_params_handles.yaw_p			=	param_find("MC_QUAT_YAW_P");
	_params_handles.yaw_rate_p		= 	param_find("MC_QUAT_YAWR_P");
	_params_handles.yaw_rate_d		= 	param_find("MC_QUAT_YAWR_D");
	_params_handles.yaw_rate_i		= 	param_find("MC_QUAT_YAWR_I");
	_params_handles.yawr_imax		= 	param_find("MC_QUAT_YR_IMAX");
	_params_handles.man_roll_max	= 	param_find("MC_MAN_R_MAX");
	_params_handles.man_pitch_max	= 	param_find("MC_MAN_P_MAX");
	_params_handles.man_yaw_max		= 	param_find("MC_MAN_Y_MAX");
	_params_handles.acro_roll_max	= 	param_find("MC_ACRO_R_MAX");
	_params_handles.acro_pitch_max	= 	param_find("MC_ACRO_P_MAX");
	_params_handles.acro_yaw_max	= 	param_find("MC_ACRO_Y_MAX");
	_params_handles.vx_p			= 	param_find("MC_VEL_VX_P");
	_params_handles.vx_i			= 	param_find("MC_VEL_VX_I");
	_params_handles.vy_p			= 	param_find("MC_VEL_VY_P");
	_params_handles.vy_i			= 	param_find("MC_VEL_VY_I");
	_params_handles.ixy_max 		=	param_find("MC_VEL_IXY_MAX");
	_params_handles.iz_max			=	param_find("MC_VEL_IZ_MAX");
	_params_handles.drag_cbar		=	param_find("MC_VEL_CBAR");
	_params_handles.man_vx_max		= 	param_find("MC_VEL_VX_MAX");
	_params_handles.man_vy_max		= 	param_find("MC_VEL_VY_MAX");
	_params_handles.vz_p			= 	param_find("MC_VEL_VZ_P");
	_params_handles.vz_i			= 	param_find("MC_VEL_VZ_I");
	_params_handles.vz_mg			= 	param_find("MC_VEL_VZ_MG");
	_params_handles.thr_max			= 	param_find("MC_THR_MAX");
	_params_handles.thr_min			= 	param_find("MC_THR_MIN");

	/* fetch initial parameter values */
	parameters_update();
}

/*******************************************************************************/
/* Quaternion specific files */

/**
  * Fast inverse square-root see: http://en.wikipedia.org/wiki/Fast_inverse_square_root 
*/
float MulticopterQuaternionVelControl::invSqrt(float number) {
    volatile long i;
    volatile float x, y;
    volatile const float f = 1.5F;

    x = number * 0.5F;
    y = number;
    i = * (( long * ) &y);
    i = 0x5f375a86 - ( i >> 1 );
    y = * (( float * ) &i);
    y = y * ( f - ( x * y * y ) );
    return y;
}
/**
  *	Extract quaternion from Euler angles
*/
void MulticopterQuaternionVelControl::getQuatFromRPY123(const float roll, const float pitch, const float yaw, float* qd)
{
	float c_phi, c_theta, c_psi;
	float s_phi, s_theta, s_psi;
	float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

	/* Precompute frequently used variables. */
	c_phi 	= cosf(0.5f*roll);
	c_theta = cosf(0.5f*pitch);
	c_psi 	= cosf(0.5f*yaw);

	s_phi 	= sinf(0.5f*roll);
	s_theta = sinf(0.5f*pitch);
	s_psi 	= sinf(0.5f*yaw);

	/* Convert RPY to Quaternion by 1-2-3 representation. See equation (297) of James Diebel paper. should actually be 3-2-1*/	
	q[0] = c_phi*c_theta*c_psi + s_phi*s_theta*s_psi;
	q[1] = -c_phi*s_theta*s_psi + c_theta*c_psi*s_phi;
	q[2] = c_phi*c_psi*s_theta + s_phi*c_theta*s_psi;
	q[3] = c_phi*c_theta*s_psi - s_phi*c_psi*s_theta;

	float recipNorm = invSqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);

	/* Normalization */
	qd[0] = q[0]*recipNorm;
	qd[1] = q[1]*recipNorm;
	qd[2] = q[2]*recipNorm;
	qd[3] = q[3]*recipNorm;
}

/**
  * Quaternion normalisation 
*/
void MulticopterQuaternionVelControl::quat_norm(float *q)
{
	float recipNorm = 1/(sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]));
	//invSqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
	q[0] = q[0]*recipNorm;
	q[1] = q[1]*recipNorm;
	q[2] = q[2]*recipNorm;
	q[3] = q[3]*recipNorm;
}


/**
  * Quaternion multiplication 
*/
void MulticopterQuaternionVelControl::quat_mult(const float *a, const float *b, float *prod)
{
	prod[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    prod[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    prod[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    prod[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

/**
  * Quaternion conjugation 
*/
void MulticopterQuaternionVelControl::quatConj(float *q, float *qconj)
{
	float recipNorm = invSqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);

	qconj[0] = q[0]*recipNorm;
	qconj[1] = -q[1]*recipNorm;
	qconj[2] = -q[2]*recipNorm;
	qconj[3] = -q[3]*recipNorm;
}

/**
  * Rotation of a vector by a quaternion 
*/
math::Vector<3> MulticopterQuaternionVelControl::quat_rotation(math::Vector<3> vec, float *quatr)
{
	float vecq[4], qvc[4], qconj[4], output[4];
	math::Vector<3> outp;

	vecq[0] = 0;
	vecq[1] = vec(0);
	vecq[2] = vec(1);
	vecq[3] = vec(2);

	quatConj(quatr,qconj);

	quat_mult(quatr,vecq,qvc);
	quat_mult(qvc,qconj,output);

	//Need to swap to match FIXME CHECK THIS
	outp(0) = output[1];
	outp(1) = output[2];
	outp(2) = output[3];
	return outp;
}

/**
  *	Rotate a quaternion by another quaternion
*/
void MulticopterQuaternionVelControl::quat_quat_rotation(float *quat_a, float *quat_b, float *quatr)
{
	float qvc[4], qconj[4];

	quatConj(quat_a,qconj);

	quat_mult(quatr,quat_b,qvc);
	quat_mult(qvc,qconj,quatr);
}

/**
  *	Rotation of a vector by an angle in rotation matrices
*/
math::Vector<3> MulticopterQuaternionVelControl::do_rotation(math::Vector<3> vec, math::Vector<3> angle)
{
	float cphi, ctheta, cpsi, sphi, stheta, spsi, vx_tmp, vy_tmp, vz_tmp;
	float roll, pitch, yaw;
	math::Matrix<3,3> Rot;
	math::Vector<3> output;

	roll = angle(0);
	pitch = angle(1);
	yaw = angle(2);
	vx_tmp = vec(1);
	vy_tmp = vec(0);
	vz_tmp = vec(2);

	cphi   = cos(roll);
	ctheta = cos(pitch);
	cpsi   = cos(yaw);

	sphi   = sin(roll);
	stheta = sin(pitch);
	spsi   = sin(yaw); 

	Rot(0,0) = ctheta*cpsi; 
	Rot(0,1) = ctheta*spsi; 
	Rot(0,2) = -stheta;     

	Rot(1,0) = sphi*stheta*cpsi - cphi*spsi; 
	Rot(1,1) = sphi*stheta*spsi + cphi*cpsi; 
	Rot(1,2) = ctheta*sphi;		    

	Rot(2,0) = cphi*stheta*cpsi + sphi*spsi; 
	Rot(2,1) = cphi*stheta*spsi - sphi*cpsi; 
	Rot(2,2) = ctheta*cphi; 		    
	Rot = Rot.transposed();

	output(1) = vx_tmp*Rot(0,0) + vy_tmp*Rot(0,1) + vz_tmp*Rot(0,2); //Need to swap to match
	output(0) = vx_tmp*Rot(1,0) + vy_tmp*Rot(1,1) + vz_tmp*Rot(1,2); 
	output(2) = vx_tmp*Rot(2,0) + vy_tmp*Rot(2,1) + vz_tmp*Rot(2,2); 

	return output;
}

/* Quaternion to Euler */
/* End Quaternion specific files */
/*******************************************************************************/

/**
   * x-y vehicle velocity controller in body fixed frame 
*/
void MulticopterQuaternionVelControl::control_velocity(math::Vector<3> vel)
{
	vy_err = _vel_setpoint(0) - vel(1);
	vx_err = _vel_setpoint(1) - vel(0);
	
	int_vy += vy_err*dtt;
	int_vx += vx_err*dtt;

	if (int_vy > _params.ixy_max) 
		int_vy = _params.ixy_max;
	if (int_vy < -_params.ixy_max)
		int_vy = -_params.ixy_max;

	if (int_vx > _params.ixy_max)
		int_vx = _params.ixy_max;
	if (int_vx < -_params.ixy_max)
		int_vy = -_params.ixy_max;

	if (_manual_control_sp.z < 0.2f )
	{
		int_vx = 0.0f;
		int_vy = 0.0f;
	}

	/* coriolis effect */
	float pitch_cor = -_v_att.yawspeed*_vel_setpoint(0) + (-_v_att.pitchspeed)*_vel_setpoint(2); //changed
	float roll_cor  =  _v_att.yawspeed*_vel_setpoint(1) - _v_att.rollspeed*_vel_setpoint(2);  //pitch
	z_omegaVd		=  -(-_v_att.pitchspeed)*_vel_setpoint(1) + (_v_att.rollspeed)*_vel_setpoint(0); //swapped

	//rollspeed is omega1 or omega-1

	/* These are more like x_con, y_con, z_con */
	_v_att_sp.roll_body 	= vy_err*Kp_vely + int_vy*Ki_vely - roll_cor/acc_grav - _params.cbar*_vel_setpoint(0);
	_v_att_sp.pitch_body 	= -vx_err*Kp_velx - int_vx*Ki_velx + pitch_cor/acc_grav + _params.cbar*_vel_setpoint(1);

	/* Capture NaN and INF */
	if ((isfinite(_v_att_sp.roll_body) == false) || !(isnan(_v_att_sp.roll_body) == false))
		_v_att_sp.roll_body = 0.0f;

	if ((isfinite(_v_att_sp.pitch_body) == false) || !(isnan(_v_att_sp.pitch_body) == false))
		_v_att_sp.pitch_body = 0.0f;	
}

/**
  * Control Vz when in auto mode
*/
void  
MulticopterQuaternionVelControl::control_vz(void)
{
	/* Vz controller */
	if (!_v_control_mode.flag_control_manual_enabled || _v_rc_channels.channels[4] > 0.0f)
	{
		_thrust_sp = _params.mg + Kp_velz * vz_err + int_vz * Ki_velz - z_omegaVd*0.0f; //fix the angle we need here		

		/* We we are moving into ground, need to avoid crashing */
		/*if ((_lpos_sp.vz > 0.0f) & (fabs(_v_local_position.z) < 0.25) & takeoff)
		{
			_thrust_sp = 0.1f;
			takeoff = false;
		}*/

		

		/* Capture inf and NaN in thrust this should capture all the others*/
		if ((isfinite(_thrust_sp) == false) || !(isnan(_thrust_sp) == false))
			_thrust_sp = _params.mg;

		if (_thrust_sp > _params.thr_max)
			_thrust_sp =_params.thr_max; 
		if (_thrust_sp < _params.thr_min)
			_thrust_sp = _params.thr_min;
	}
}
/**
  * @function determine_desired att in R
  * Function receives x_con, y_con and yaw_con to produce quat
*/
void 
MulticopterQuaternionVelControl::determine_desired_quat_Rd(float x_con, float y_con, float cos_sin, float yaw_con, float *quat)
{	
	//math::Vector<3> 
	Vector3f yaw_rot = Vector3f::Zero();
	Vector3f Rd_trans = Vector3f::Zero();
	Vector3f nu = Vector3f::Zero();
	Vector3f xi = Vector3f::Zero();
	Matrix3f Rdtrans = Matrix3f::Zero();

	yaw_rot(0) = cos(yaw_con);
	yaw_rot(1) = -sin(yaw_con);
	yaw_rot(2) = 0; //Mod Dec

	Rd_trans(0) = x_con;
	Rd_trans(1) = y_con;
	Rd_trans(2) = cos_sin; //Mod Dec

	nu = Rd_trans.cross(yaw_rot);

	nu.normalize();

	xi = nu.cross(Rd_trans);

	Rdtrans.row(0) = xi;
	Rdtrans.row(1) = nu;
	Rdtrans.row(2) = Rd_trans;

	//Quaternion<float> q;

	Quaternionf q(Rdtrans);
	q.normalize();

	quat[0] = q.w();
	quat[1] = q.x();
	quat[2] = -q.y();
	quat[3] = q.z();

	/*float r, p, y;
	r 	= atan2f(Rdtrans(2,1), Rdtrans(2,2));	
	p 	= -asinf(Rdtrans(2,0));	
	y 	= atan2f(Rdtrans(1,0), Rdtrans(0,0));	

	printf("rypcalc %3.3f %3.3f %3.3f\n", double(r),double(p),(double)y);
	*/
}

/**
  * Function accepts x_con, y_con and determines desired quaternion 
*/
void MulticopterQuaternionVelControl::determine_desired_quat(float x_con, float y_con, float yaw_con, float *quat)
{	
	float q0, q1, q2;//, psi_a, quat_b[4];

	/* Let us assume that q0 > 0 */
	q0 = sqrt(1 + (sqrt(1 - (x_con*x_con + y_con*y_con)))/2);
	q2 = x_con/(2*q0); //No need for minus as it has already been dealt with
	q1 = y_con/(2*q0);

	float recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2);

	quat[0] = q0*recipNorm;
	quat[1] = q1*recipNorm;
	quat[2] = q2*recipNorm;
	quat[3] = 0.0f;


	/* Do rotation by yaw */
	float roll = atan2(2*(quat[0]*quat[1] + quat[2]*quat[3]),(1 - 2*(quat[1]*quat[1] + quat[2]*quat[2])));
	float theta = asin(2*(quat[0]*quat[2] - quat[3]*quat[1]));


	theta = asin(x_con);
	float mid = y_con/((float) cos(theta));
	roll = asin(mid);
	getQuatFromRPY123(roll, theta, yaw_con, quat);
}

/**
  * Quaternion velocity controller main file
*/
MulticopterQuaternionVelControl::~MulticopterQuaternionVelControl()
{
	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	mc_quat_vel_control::g_control = nullptr;
}

/**
  * Extract and update parameters
*/
int MulticopterQuaternionVelControl::parameters_update()
{
	float v;

	/* roll gains */
	param_get(_params_handles.roll_p, &v);
	_params.att_p(0) = v;
	param_get(_params_handles.roll_rate_p, &v);
	_params.rate_p(0) = v;
	param_get(_params_handles.vy_p, &v);
	_params.vel_p(0) = v;
	param_get(_params_handles.vy_i, &v);
	_params.vel_i(0) = v;

	/* pitch gains */
	param_get(_params_handles.pitch_p, &v);
	_params.att_p(1) = v;
	param_get(_params_handles.pitch_rate_p, &v);
	_params.rate_p(1) = v;
	param_get(_params_handles.vx_p, &v);
	_params.vel_p(1) = v;
	param_get(_params_handles.vx_i, &v);
	_params.vel_i(1) = v;

	/* yaw gains */
	param_get(_params_handles.yaw_p, &v);
	_params.att_p(2) = v;
	param_get(_params_handles.yaw_rate_p, &v);
	_params.rate_p(2) = v;
	param_get(_params_handles.yaw_rate_i, &v);
	_params.rate_i(2) = v;
	//_params.yaw_rate_i = v;

	/* vz control */
	param_get(_params_handles.vz_p, &v);
	_params.vel_p(2) = v;
	param_get(_params_handles.vz_i, &v);
	_params.vel_i(2) = v;
	param_get(_params_handles.vz_mg, &v);
	_params.mg = v;
	param_get(_params_handles.thr_max, &v);
	_params.thr_max = v;
	param_get(_params_handles.thr_min, &v);
	_params.thr_min = v;

	/* integral saturations */
	param_get(_params_handles.ixy_max, &v);
	_params.ixy_max = v;
	param_get(_params_handles.iz_max, &v);
	_params.iz_max = v;
	param_get(_params_handles.drag_cbar, &v);
	_params.cbar = v;


	/* manual control scale */
	param_get(_params_handles.man_roll_max, &_params.man_roll_max);
	param_get(_params_handles.man_pitch_max, &_params.man_pitch_max);
	param_get(_params_handles.man_yaw_max, &_params.man_yaw_max);
	_params.man_roll_max = math::radians(_params.man_roll_max);
	_params.man_pitch_max = math::radians(_params.man_pitch_max);
	_params.man_yaw_max = math::radians(_params.man_yaw_max);

	/* Manual control velocity mode */
	param_get(_params_handles.man_vx_max, &_params.man_vx_max);
	param_get(_params_handles.man_vy_max, &_params.man_vy_max);

	/* acro control scale */
	param_get(_params_handles.acro_roll_max, &v);
	_params.acro_rate_max(0) = math::radians(v);
	param_get(_params_handles.acro_pitch_max, &v);
	_params.acro_rate_max(1) = math::radians(v);
	param_get(_params_handles.acro_yaw_max, &v);
	_params.acro_rate_max(2) = math::radians(v);

	param_get(_params_handles.yawr_imax, &v);
	_params.yawr_imax = v;

	_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled("CBRK_RATE_CTRL", CBRK_RATE_CTRL_KEY);

	return OK;
}

void
MulticopterQuaternionVelControl::parameter_update_poll()
{
	bool updated;

	/* Check HIL state if vehicle status has changed */
	orb_check(_params_sub, &updated);

	if (updated) {
		struct parameter_update_s param_update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
		parameters_update();
	}
}

void
MulticopterQuaternionVelControl::vehicle_rc_poll()
{
	bool updated;

	/* Check HIL state if vehicle status has changed */
	orb_check(_v_rc_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(rc_channels), _v_rc_sub, &_v_rc_channels);
	}
}

void
MulticopterQuaternionVelControl::vehicle_control_mode_poll()
{
	bool updated;

	/* Check HIL state if vehicle status has changed */
	orb_check(_v_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _v_control_mode_sub, &_v_control_mode);
	}
}

void
MulticopterQuaternionVelControl::vehicle_manual_poll()
{
	bool updated;

	/* get pilots inputs */
	orb_check(_manual_control_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, &_manual_control_sp);
	}
}

void
MulticopterQuaternionVelControl::vehicle_attitude_setpoint_poll()
{
	/* check if there is a new attitude setpoint */
	bool updated;
	orb_check(_v_att_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_sub, &_v_att_sp);
	}
}

void
MulticopterQuaternionVelControl::vehicle_lpos_setpoint_poll()
{
	/* check if there is a new local position setpoint */
	bool updated;
	orb_check(_v_lpos_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_local_position_setpoint), _v_lpos_sp_sub, &_lpos_sp);
	}
	//printf("Body vel time %lu %u\n",_lpos_sp.timestamp,_lpos_sp.timestamp);
}

void
MulticopterQuaternionVelControl::vehicle_local_postion_poll()
{
	/* check if there is a new local position for vz */
	bool updated;
	orb_check(_v_local_pos_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_local_position), _v_local_pos_sub, &_v_local_position);
	}

}

void
MulticopterQuaternionVelControl::vehicle_vicon_postion_poll()
{
	/* check if there is a new vicon position for vz */
	bool updated;
	orb_check(_v_veh_vicon_pos_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_vicon_position), _v_veh_vicon_pos_sub, &_v_vicon_position);
	}

}

void
MulticopterQuaternionVelControl::vehicle_velocity_meas_inertial_poll()
{
	/* check if there is a new velocity measurement */
	bool updated;
	orb_check(_v_vel_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_velocity_meas_inertial), _v_vel_sub, &_v_vel);
	}
}

void
MulticopterQuaternionVelControl::vehicle_velocity_est_inertial_poll()
{
	/* check if there is a new velocity estimated */
	bool updated;
	orb_check(_v_vel_est_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_velocity_est_inertial), _v_vel_est_sub, &_v_vel_est);
	}
}

void
MulticopterQuaternionVelControl::vehicle_velocity_measured_body_poll()
{
	/* check if there is a new velocity estimated */
	bool updated;
	orb_check(_v_vel_body_est_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_velocity_meas_est_body), _v_vel_body_est_sub, &_v_vel_body_est);
	}
}

void
MulticopterQuaternionVelControl::vehicle_rates_setpoint_poll()
{
	/* check if there is a new rates setpoint for acro mode */
	bool updated;
	orb_check(_v_rates_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);
	}
}

void
MulticopterQuaternionVelControl::poll_subscriptions()
{
	/* Poll vehicle RC need for forces automode when INS is unavailable*/
	vehicle_rc_poll();

	/* Update parameters */
	parameter_update_poll();

	/* Poll vehicle control mode */
	vehicle_control_mode_poll();

	/* Poll vehicle arming status */
	arming_status_poll();

	/* Poll Manual mode values */
	vehicle_manual_poll();

	/* Poll measured INS velocity */
	vehicle_velocity_meas_inertial_poll();

	/* Poll estimated INS velocity */
	vehicle_velocity_est_inertial_poll();

	/* Poll body frame meas and est velocity */
	vehicle_velocity_measured_body_poll();
}

void
MulticopterQuaternionVelControl::set_control_gains(bool gain_body_est)
{
	if (!gain_body_est)
	{
		Kp_velx = _params.vel_p(1);
		Ki_velx = _params.vel_i(1);

		Kp_vely = _params.vel_p(0);
		Ki_vely = _params.vel_i(0);

		Kp_velz = _params.vel_p(2);
		Ki_velz = _params.vel_i(2);
	} else
	{
		Kp_velx = _params.vel_p(1)*0.5f;
		Ki_velx = _params.vel_i(1)*0.5f;

		Kp_vely = _params.vel_p(0)*0.65f;
		Ki_vely = _params.vel_i(0)*0.65f;

		Kp_velz = _params.vel_p(2)*0.75f;
		Ki_velz = _params.vel_i(2)*0.75f;
	}
}

void
MulticopterQuaternionVelControl::arming_status_poll()
{
	/* check if there is a new arming state */
	bool updated;
	orb_check(_armed_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
	}
}

/*
 * Quaternion Attitude controller.
 * Input: 'manual_control_setpoint' and 'vehicle_attitude_setpoint' and 'local_position_stp' topics (depending on mode)
 * Output: '_rates_sp' vector, '_thrust_sp', 'vehicle_attitude_setpoint' topic (for manual modes)
 */
void MulticopterQuaternionVelControl::control_attitude(float dt)
{
	Matrix3f Rhot;
	float ctheta, stheta, cphi, sphi, cpsi, spsi;

	float cs_angle = 1.0f;
	bool publish_att_sp = false;
	math::Vector<3> veh_vel;

	/* Flag for changing gains when we lose inertial sensors. A more efficient method is required */
	change_control_gain = false;
	set_control_gains(change_control_gain);

	/* Determine the actual attitude of the vehicle in quaternion since we don't know which observer is running */
	if (_v_att.q_valid) {	

		if (_v_control_mode.flag_control_manual_enabled  && _v_rc_channels.channels[4] < 0.0f) {	
			getQuatFromRPY123(_v_att.roll, _v_att.pitch, 0, _v_att.q);
		} else { //Jan16
			getQuatFromRPY123(_v_att.roll, _v_att.pitch, _v_att.yaw, _v_att.q);
		}//end Jan16
	} else 	{
		if (_v_control_mode.flag_control_manual_enabled  && _v_rc_channels.channels[4] < 0.0f) {
			getQuatFromRPY123(_v_att.roll, _v_att.pitch, 0, _v_att.q);
		} else {
			getQuatFromRPY123(_v_att.roll, _v_att.pitch, _v_att.yaw, _v_att.q);
		}
	}

	/*************** Now lets calculate the desired quaternion **************************/
	if (_v_control_mode.flag_control_manual_enabled && _v_rc_channels.channels[4] < 0.0f) 
	{

		//takeoff = false;

		if (_v_local_position.z < -0.25f)
		{
			takeoff = true;
		}

		/* manual input, set or modify velocity of velocity is available or attitude setpoint */
		_vel_setpoint(0) = _manual_control_sp.y * _params.man_vy_max;
		_vel_setpoint(1) = _manual_control_sp.x * _params.man_vx_max;
		_vel_setpoint(2) = 0.0f;

		math::Vector<3> ang;
		ang(0) = _v_att.roll;
		ang(1) = _v_att.pitch;
		ang(2) = 0.0f;
		
		ctheta = cos(-_v_att.pitch);
		stheta = sin(-_v_att.pitch);
		cphi = cos(-_v_att.roll);
		sphi = sin(-_v_att.roll);
		cpsi = cos(0);
		spsi = sin(0);
		Rhot(0,0) = ctheta*cpsi; 
		Rhot(0,1) = sphi*stheta*cpsi-cphi*spsi; 
		Rhot(0,2) = cphi*stheta*cpsi+sphi*spsi;     

		Rhot(1,0) = ctheta*spsi; 
		Rhot(1,1) = sphi*stheta*spsi+cphi*cpsi; 
		Rhot(1,2) = cphi*stheta*spsi-sphi*cpsi;		    

		Rhot(2,0) = -stheta; 
		Rhot(2,1) = ctheta*sphi; 
		Rhot(2,2) = ctheta*cphi;

		//_vel_setpoint = quat_rotation(_vel_setpoint,_v_att.q);
		Vector3f vel_stpMos;
		vel_stpMos(0) = _vel_setpoint(0);
		vel_stpMos(1) = _vel_setpoint(1);
		vel_stpMos(2) = _vel_setpoint(2);
		vel_stpMos = Rhot.transpose()*vel_stpMos;
		_vel_setpoint(0) = vel_stpMos(0);
		_vel_setpoint(1) = vel_stpMos(1);
		_vel_setpoint(2) = vel_stpMos(2);
		
		/* If velocity measurements are available, use them TODO timeout for _velocity.timestamp */
		if (_v_vel.body_valid )
		{
			vel_cont = true;

			/* If meas velocity is >35Hz there is no advantage in using v_est
			   For now lets use vicon = lpos since that is what we have at 39Hz */
			/*if (_v_vicon_position.valid) //These comments Jan15
			{ 
				if  (_v_vel.timestamp != old_vel_timestamp)
					{
						veh_vel(0) = _v_vel.body_vx;
						veh_vel(1) = _v_vel.body_vy;
						veh_vel(2) = _v_vel.body_vz;

						dtt = (_v_vel.timestamp - old_vel_timestamp)*0.000001f;

						control_velocity(veh_vel);
						old_vel_timestamp = _v_vel.timestamp;
					}
			} *///else
			//{
					if (_v_vel_est.inertial_valid)
					{
						/* Only run controller when new data is available */
						if  (_v_vel_est.timestamp != old_vel_timestamp)
						{
							veh_vel(0) = _v_vel_est.inertial_bvx;
							veh_vel(1) = _v_vel_est.inertial_bvy;
							veh_vel(2) = _v_vel_est.inertial_bvz;

							dtt = (_v_vel_est.timestamp - old_vel_timestamp)*0.000001f;
							control_velocity(veh_vel);
							old_vel_timestamp = _v_vel_est.timestamp;
						}
				} else 
				{    /* We know that there is no v_est data so lets use v_meas */
					/* Only run controller when new data is available */
					if  (_v_vel.timestamp != old_vel_timestamp)
					{
						veh_vel(0) = _v_vel.body_vx;
						veh_vel(1) = _v_vel.body_vy;
						veh_vel(2) = _v_vel.body_vz;

						dtt = (_v_vel.timestamp - old_vel_timestamp)*0.000001f;

						control_velocity(veh_vel);
						old_vel_timestamp = _v_vel.timestamp;
					}
				}

			//} Jan15
					
			_lpos_sp.vy = _vel_setpoint(0);
			_lpos_sp.vx = _vel_setpoint(1);
			_lpos_sp.vz = _vel_setpoint(2);

			/* Limit desired angular setpoints */
			if (_v_att_sp.roll_body > _params.man_roll_max)
				_v_att_sp.roll_body = _params.man_roll_max;
			if (_v_att_sp.roll_body < -_params.man_roll_max)
				_v_att_sp.roll_body = -_params.man_roll_max;
			if (_v_att_sp.pitch_body > _params.man_pitch_max)
				_v_att_sp.pitch_body = _params.man_roll_max;
			if (_v_att_sp.pitch_body < -_params.man_pitch_max)
				_v_att_sp.pitch_body = -_params.man_pitch_max;

			/* Now we need to determine the third angle */
			float thet = -asin(_v_att_sp.pitch_body);
			float ph   = asin(_v_att_sp.roll_body/cosf(thet));
			cs_angle = cos(thet)*cos(ph);

		} else /* Go straight to normal attitude control mode */
		{		
			vel_cont = false;	
			float ph 	= _manual_control_sp.y * _params.man_roll_max; //_v_att_sp.roll_body
			float thet 	= -_manual_control_sp.x * _params.man_pitch_max; //_v_att_sp.pitch_body

			/* So now lets calculate the actual desired angle in Rot */
			_v_att_sp.pitch_body 	= sin(thet);
			_v_att_sp.roll_body 	= cos(thet) * sin(ph);
			cs_angle 				= cos(thet) * cos(ph);
		}
		
		_v_att_sp.R_valid = false;
		_v_att_sp.q_d_valid = false;
		_thrust_sp = _manual_control_sp.z; 
		_v_att_sp.thrust = _thrust_sp;
		publish_att_sp = true;
		

		if (_lpos_sp_pub != nullptr) {
			orb_publish(ORB_ID(vehicle_local_position_setpoint), _lpos_sp_pub, &_lpos_sp);

		} else {
			_lpos_sp_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &_lpos_sp);
		}

		/* Compute desired quaternion q_d:*/	
		//getQuatFromRPY123(_v_att_sp.roll_body, _v_att_sp.pitch_body, 0, q_d);
		//determine_desired_quat(_v_att_sp.pitch_body,_v_att_sp.roll_body, 0, q_d);

		determine_desired_quat_Rd(_v_att_sp.pitch_body,_v_att_sp.roll_body, cs_angle, 0.0f, q_d);

		/* In here, use the above to determine qd given that the current _v_att_sp is xcon, ycon */ 
		
		/* Update the attitude setpoint */
		_v_att_sp.q_d[0] = q_d[0];
		_v_att_sp.q_d[1] = q_d[1];
		_v_att_sp.q_d[2] = q_d[2];
		_v_att_sp.q_d[3] = q_d[3];
		if (_att_sp_pub != nullptr) {
			orb_publish(ORB_ID(vehicle_attitude_setpoint), _att_sp_pub, &_v_att_sp);

		} else {
			_att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_v_att_sp);
		}
		_thrust_sp = _v_att_sp.thrust;
		int_vz = 0.0f;

	} 
	else {
		/* Autonomous mode use Use the vehicle local position setpoint */
		vehicle_lpos_setpoint_poll();
		/* We need to poll the position of the vehicle for takeoff and landing */
		vehicle_local_postion_poll(); 

		_vel_setpoint(0) = _lpos_sp.vy;
		_vel_setpoint(1) = _lpos_sp.vx;
		_vel_setpoint(2) = _lpos_sp.vz;

		math::Vector<3> ang;
		ang(0) = 0.0f;
		ang(1) = 0.0f;
		ang(2) = _lpos_sp.yaw;

		/* Added Jan16 */
		ctheta = cos(-_v_att.pitch);
		stheta = sin(-_v_att.pitch);
		cphi = cos(-_v_att.roll);
		sphi = sin(-_v_att.roll);
		cpsi = cos(0);
		spsi = sin(0);
		Rhot(0,0) = ctheta*cpsi; 
		Rhot(0,1) = sphi*stheta*cpsi-cphi*spsi; 
		Rhot(0,2) = cphi*stheta*cpsi+sphi*spsi;     

		Rhot(1,0) = ctheta*spsi; 
		Rhot(1,1) = sphi*stheta*spsi+cphi*cpsi; 
		Rhot(1,2) = cphi*stheta*spsi-sphi*cpsi;		    

		Rhot(2,0) = -stheta; 
		Rhot(2,1) = ctheta*sphi; 
		Rhot(2,2) = ctheta*cphi;

		//_vel_setpoint = quat_rotation(_vel_setpoint,_v_att.q);
		Vector3f vel_stpMos;
		vel_stpMos(0) = _vel_setpoint(0);
		vel_stpMos(1) = _vel_setpoint(1);
		vel_stpMos(2) = _vel_setpoint(2);
		vel_stpMos = Rhot.transpose()*vel_stpMos;
		_vel_setpoint(0) = vel_stpMos(0);
		_vel_setpoint(1) = vel_stpMos(1);
		_vel_setpoint(2) = vel_stpMos(2);
		/* End addition Jan16 */

		/* The lpos_set is in inertial frame so we need to convert to body fixed frame */
		//_vel_setpoint = quat_rotation(_vel_setpoint,_v_att.q); //Changed on Jan2016


		if (_v_vel.body_valid )
		{
			/* If meas velocity is >35Hz there is no advantage in using v_est
			   For now lets use vicon = lpos since that is what we have at 39Hz */
			/*if (_v_vicon_position.valid)
			{
				if  (_v_vel.timestamp != old_vel_timestamp)
					{
						veh_vel(0) = _v_vel.body_vx;
						veh_vel(1) = _v_vel.body_vy;
						veh_vel(2) = _v_vel.body_vz;

						dtt = (_v_vel.timestamp - old_vel_timestamp)*0.000001f;

						control_velocity(veh_vel);
						old_vel_timestamp = _v_vel.timestamp;
					}
			} else
			{*/
				if (_v_vel_est.inertial_valid)
				{
					//change_control_gain = true;
					//set_control_gains(change_control_gain);

					/* Only run controller when new data is available */
					if  (_v_vel_est.timestamp != old_vel_timestamp)
					{
						veh_vel(0) = _v_vel_est.inertial_bvx;
						veh_vel(1) = _v_vel_est.inertial_bvy;
						veh_vel(2) = _v_vel_est.inertial_bvz;

						dtt = (_v_vel_est.timestamp - old_vel_timestamp)*0.000001f;
						control_velocity(veh_vel);
						old_vel_timestamp = _v_vel_est.timestamp;
					} 
				} else
				{ // This is there just in case we are not running our av_estimator
					if  (_v_vel.timestamp != old_vel_timestamp)
					{
						veh_vel(0) = _v_vel.body_vx;
						veh_vel(1) = _v_vel.body_vy;
						veh_vel(2) = _v_vel.body_vz;

						dtt = (_v_vel.timestamp - old_vel_timestamp)*0.000001f;

						control_velocity(veh_vel);
						old_vel_timestamp = _v_vel.timestamp;
					}
				}
			//} 
		} else
		{
			if  (_v_vel_body_est.timestamp != old_vel_timestamp)
				{
					change_control_gain = true;
					set_control_gains(change_control_gain);
					veh_vel(0) = _v_vel_body_est.est_vx;
					veh_vel(1) = _v_vel_body_est.est_vy;
					veh_vel(2) = _v_vel_body_est.est_vz;

					dtt = (_v_vel_body_est.timestamp - old_vel_timestamp)*0.000001f;

					control_velocity(veh_vel);
					old_vel_timestamp = _v_vel_body_est.timestamp;
				}
		}
		
			//Moses: I should end the if loop here and use v_est_body
			vz_err = veh_vel(2) - _vel_setpoint(2); //create new local variable for vz
			vz_err =_v_vel.body_vz - _vel_setpoint(2);
			int_vz += vz_err*dtt;	
			if (int_vz > _params.iz_max) 
				int_vz = _params.iz_max;
			if (int_vz < -_params.iz_max)
				int_vz = -_params.iz_max;	

			/* If we haven't taken off yet FIXME somewhat not necessary*/
			if (_v_vicon_position.valid){
					if ((int(_lpos_sp.vz) == 0) & (fabs(_v_local_position.z) < 0.25))
					{
						int_vz = 0.0f;
						int_vx = 0.0f;
						int_vy = 0.0f;
					} 
			}
			

			/* vz control */
			control_vz();

			/* Limit desired angular setpoints */
			if (_v_att_sp.roll_body > _params.man_roll_max*0.3f)
				_v_att_sp.roll_body = _params.man_roll_max*0.3f;
			if (_v_att_sp.roll_body < -_params.man_roll_max*0.3f)
				_v_att_sp.roll_body = -_params.man_roll_max*0.3f;
			if (_v_att_sp.pitch_body > _params.man_pitch_max*0.3f)
				_v_att_sp.pitch_body = _params.man_pitch_max*0.3f;
			if (_v_att_sp.pitch_body < -_params.man_pitch_max*0.3f)
				_v_att_sp.pitch_body = -_params.man_pitch_max*0.3f;


			/* Now we need to determine the third angle */
			float thet 	= -asin(_v_att_sp.pitch_body);
			float ph   	= asin(_v_att_sp.roll_body / cosf(thet));
			cs_angle 	= cos(thet) * cos(ph);

			_v_att_sp.thrust = _manual_control_sp.z;

			/* Determine desired quaternion */
			//determine_desired_quat_Rd(_v_att_sp.pitch_body,_v_att_sp.roll_body,cs_angle,_lpos_sp.yaw,q_d);
			getQuatFromRPY123(_v_att_sp.roll_body, _v_att_sp.pitch_body, _lpos_sp.yaw, q_d);
			//printf("rypd %3.3f %3.3f %3.3f\n", double(ph),double(thet),(double)_lpos_sp.yaw);
		
			/* Update the attitude setpoint */
			_v_att_sp.q_d[0] = q_d[0];
			_v_att_sp.q_d[1] = q_d[1];
			_v_att_sp.q_d[2] = q_d[2];
			_v_att_sp.q_d[3] = q_d[3];
			_thrust_sp = _manual_control_sp.z; //uncomment
			_v_att_sp.thrust = _thrust_sp;
			publish_att_sp = true;
		//}	else { // We know that the body fixed frame estimated velocities are always available

		//}				
	}
	/********************* End calculation of the desired quaternion ************************/

	/* publish the attitude setpoint if needed */
	if (publish_att_sp) {
		_v_att_sp.timestamp = hrt_absolute_time();

		if (_att_sp_pub != nullptr) {
			orb_publish(ORB_ID(vehicle_attitude_setpoint), _att_sp_pub, &_v_att_sp);

		} else {
			_att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_v_att_sp);
		}
	}

	/* Compute or obtain error quaternion q_e */	
	if (_v_control_mode.flag_control_manual_enabled  && _v_rc_channels.channels[4] < 0.0f) 
	{
		quatConj(q_d,q_d_inv);
		quat_mult(q_d_inv,_v_att.q,q_e);

		_v_att_sp.q_e[0] = q_e[0];
		_v_att_sp.q_e[1] = q_e[1];
		_v_att_sp.q_e[2] = q_e[2];
		_v_att_sp.q_e[3] = q_e[3];
		_v_att_sp.q_e_valid = true;

		_v_att_sp.q_d[0] = q_d[0];
		_v_att_sp.q_d[1] = q_d[1];
		_v_att_sp.q_d[2] = q_d[2];
		_v_att_sp.q_d[3] = q_d[3];
		_v_att_sp.q_d_valid = true;
	} else 
	{ //TODO handle other apps producing not only qd but also qe
		quatConj(q_d,q_d_inv);
		quat_mult(q_d_inv,_v_att.q,q_e);

		_v_att_sp.q_e[0] = q_e[0];
		_v_att_sp.q_e[1] = q_e[1];
		_v_att_sp.q_e[2] = q_e[2];
		_v_att_sp.q_e[3] = q_e[3];
		_v_att_sp.q_e_valid = true;

		_v_att_sp.q_d[0] = q_d[0];
		_v_att_sp.q_d[1] = q_d[1];
		_v_att_sp.q_d[2] = q_d[2];
		_v_att_sp.q_d[3] = q_d[3];
		_v_att_sp.q_d_valid = true;
	}

	/* Calculate the desired Omega and publish */
	_rates_sp(0) = -_params.att_p(0)*q_e[1]; 
	_rates_sp(1) = -_params.att_p(1)*q_e[2]; 

	if (_v_control_mode.flag_control_manual_enabled  && _v_rc_channels.channels[4] < 0.0f) 
	{
		_rates_sp(2) = _manual_control_sp.r * _params.acro_rate_max(2); 
	} else 
		_rates_sp(2) = -_params.att_p(2)*q_e[3]; 
}

/*
 * Attitude rates controller tau = -K_d*\tilde{Omega} + I Omega_d +....
 * Input: '_rates_sp' _'rate_rates_sp', '_thrust_sp'
 * Output: '_att_control' 
 */
void
MulticopterQuaternionVelControl::control_attitude_rates(float dt)
{
	yr_max = _params.yawr_imax;

	/* current body angular rates */
	math::Vector<3> rates, vehicles_Omega_rates_d;
	rates(0) = _v_att.rollspeed;
	rates(1) = _v_att.pitchspeed;
	rates(2) = _v_att.yawspeed;

	/* angular rates error */
	//here we fix rates = rates + Omega_d
	//tau_d = k_d*omega_rate_d + S(omega_d)KdOmega - 0. The zero is for Ga which is hard to determine
	math::Vector<3> rates_err = rates - _rates_sp;

	/* Feedforward for angular velocity */
	math::Vector<3> vehicles_Omega_rates = (rates - _rates_prev)/dt;

	/* To keep up with PX4 Notation, Ixx = _params.rate_d(0), Iyy = _params.rate_d(1), Izz = _params.rate_d(2). For manual flight, 		these are set to zero */
	_params.rate_d(0) = 0.0f;
	_params.rate_d(1) = 0.0f;
	_params.rate_d(2) = 0.0f;

	/* There should be a topic for these and they should come from high-level trajectory controller */
	vehicles_Omega_rates_d(0) = 0.0f;
	vehicles_Omega_rates_d(1) = 0.0f;
	vehicles_Omega_rates_d(2) = 0.0f;


	_att_control(0) = -_params.rate_p(0)*rates_err(0) - _params.rate_d(0)*(vehicles_Omega_rates(0) - vehicles_Omega_rates_d(0));
	_att_control(1) = -_params.rate_p(1)*rates_err(1) - _params.rate_d(1)*(vehicles_Omega_rates(1) - vehicles_Omega_rates_d(1));
	_att_control(2) = -_params.rate_p(2)*rates_err(2) - _params.rate_d(2)*(vehicles_Omega_rates(2) - vehicles_Omega_rates_d(2));

	if (_v_control_mode.flag_control_manual_enabled && _v_rc_channels.channels[4] < 0.0f) 
	{
		if (vel_cont)
		{
			int_yawrate += rates_err(2)*dtt;
		} else
		{ //rescale the integral gain though this shouldn't matter
			int_yawrate += rates_err(2)*dt;
		}

		if (_manual_control_sp.z < 0.2f )
		{
			int_yawrate = 0.0f;
		}
		if (int_yawrate > yr_max)
			int_yawrate = yr_max;
		if (int_yawrate < -yr_max)
			int_yawrate = -yr_max;

		_att_control(2) -= int_yawrate*_params.rate_i(2);
	} else
	{
		int_yawrate += rates_err(2)*dtt;
		if (_thrust_sp < 0.2f )
		{
			int_yawrate = 0.0f;
			_att_control(2) = 0.0f;
		}
		if (int_yawrate > yr_max)
			int_yawrate = yr_max;
		if (int_yawrate < -yr_max)
			int_yawrate = -yr_max;

		_att_control(2) -=int_yawrate*_params.rate_i(2)*0;
	}		
	_rates_prev = rates;

	printf("yaw con %3.3f\n",double(_att_control(2)));
}

void
MulticopterQuaternionVelControl::task_main_trampoline(int argc, char *argv[])
{
	mc_quat_vel_control::g_control->task_main();
}

void
MulticopterQuaternionVelControl::task_main()
{
	warnx("started");
	fflush(stdout);

	/*
	 * do subscriptions
	 */
	_v_att_sp_sub 			= orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_v_rates_sp_sub 		= orb_subscribe(ORB_ID(vehicle_rates_setpoint));
	_v_att_sub 				= orb_subscribe(ORB_ID(vehicle_attitude));
	_v_control_mode_sub 	= orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub 			= orb_subscribe(ORB_ID(parameter_update));
	_manual_control_sp_sub 	= orb_subscribe(ORB_ID(manual_control_setpoint));
	_armed_sub 				= orb_subscribe(ORB_ID(actuator_armed));
	_v_lpos_sp_sub 			= orb_subscribe(ORB_ID(vehicle_local_position_setpoint));

	_v_vel_sub 				= orb_subscribe(ORB_ID(vehicle_velocity_meas_inertial));
	_v_local_pos_sub 		= orb_subscribe(ORB_ID(vehicle_local_position));
	_v_vel_est_sub 			= orb_subscribe(ORB_ID(vehicle_velocity_est_inertial));
	_v_vel_body_est_sub	  	= orb_subscribe(ORB_ID(vehicle_velocity_meas_est_body));
	_v_veh_vicon_pos_sub  	= orb_subscribe(ORB_ID(vehicle_vicon_position));
	_v_rc_sub 				= orb_subscribe(ORB_ID(rc_channels));

	/* initialize parameters cache */
	parameters_update();

	/* wakeup source: vehicle attitude */
	struct pollfd fds[1];

	fds[0].fd = _v_att_sub;
	fds[0].events = POLLIN;

	/* Do initial polling */
	poll_subscriptions();


	while (!_task_should_exit) {

		/* wait for up to 100ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit */
		if (pret == 0)
			continue;

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			usleep(100000);
			continue;
		}

		perf_begin(_loop_perf);

		/* run controller on attitude changes */
		if (fds[0].revents & POLLIN) {
			static uint64_t last_run = 0;
			float dt = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too small (< 2ms) and too large (> 20ms) dt's */
			if (dt < 0.002f) {
				dt = 0.002f;

			} else if (dt > 0.02f) {
				dt = 0.02f;
			}

			/* copy attitude topic */
			orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);

			/* check for updates in other topics */
			poll_subscriptions();			
			

			if (_v_control_mode.flag_control_attitude_enabled) {
				control_attitude(dt);

				/* publish attitude rates setpoint */
				_v_rates_sp.roll = _rates_sp(0);
				_v_rates_sp.pitch = _rates_sp(1);
				_v_rates_sp.yaw = _rates_sp(2);
				_v_rates_sp.thrust = _thrust_sp;
				_v_rates_sp.timestamp = hrt_absolute_time();

				if (_v_rates_sp_pub != nullptr) {
					orb_publish(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_pub, &_v_rates_sp);

				} else {
					_v_rates_sp_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &_v_rates_sp);
				}

			} else {
				/* quaternion controller disabled, poll rates setpoint topic for acro mode. Not tested*/
				if (_v_control_mode.flag_control_manual_enabled) 
				{
					/* manual rates control - ACRO mode */
					_rates_sp = math::Vector<3>(_manual_control_sp.y, -_manual_control_sp.x, _manual_control_sp.r).emult(_params.acro_rate_max);
					_thrust_sp = _manual_control_sp.z;

					/* publish attitude rates setpoint */
					_v_rates_sp.roll = _rates_sp(0);
					_v_rates_sp.pitch = _rates_sp(1);
					_v_rates_sp.yaw = _rates_sp(2);
					_v_rates_sp.thrust = _thrust_sp;
					_v_rates_sp.timestamp = hrt_absolute_time();

					if (_v_rates_sp_pub != nullptr) {
						orb_publish(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_pub, &_v_rates_sp);

					} else {
						_v_rates_sp_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &_v_rates_sp);
					}

				} else 
				{
					/* quaternion controller disabled, poll rates setpoint topic */
					vehicle_rates_setpoint_poll();
					_rates_sp(0) = _v_rates_sp.roll;
					_rates_sp(1) = _v_rates_sp.pitch;
					_rates_sp(2) = _v_rates_sp.yaw;
					_thrust_sp = _v_rates_sp.thrust;
				}
			}

			if (_v_control_mode.flag_control_rates_enabled) {
				control_attitude_rates(dt);

				/* publish actuator controls */
				_actuators.control[0] = (isfinite(_att_control(0))) ? _att_control(0) : 0.0f;
				_actuators.control[1] = (isfinite(_att_control(1))) ? _att_control(1) : 0.0f;
				_actuators.control[2] = (isfinite(_att_control(2))) ? _att_control(2) : 0.0f;
				_actuators.control[3] = (isfinite(_thrust_sp)) ? _thrust_sp : 0.0f;
				_actuators.timestamp = hrt_absolute_time();

				if (!_actuators_0_circuit_breaker_enabled) {
					if (_actuators_0_pub != nullptr) {
						orb_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, &_actuators);

					} else {
						_actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators);
					}
				}
			}
		}

		perf_end(_loop_perf);
	}

	warnx("exit");

	_control_task = -1;
	_exit(0);
}

int
MulticopterQuaternionVelControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("mc_quat_vel_control",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_MAX - 5,
				       2000,
				       (main_t)&MulticopterQuaternionVelControl::task_main_trampoline,
				       nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int mc_quat_vel_control_main(int argc, char *argv[])
{
	if (argc < 1)
		errx(1, "usage: mc_quat_vel_control {start|stop|status}");

	if (!strcmp(argv[1], "start")) {

		if (mc_quat_vel_control::g_control != nullptr)
			errx(1, "already running");

		mc_quat_vel_control::g_control = new MulticopterQuaternionVelControl;

		if (mc_quat_vel_control::g_control == nullptr)
			errx(1, "alloc failed");

		if (OK != mc_quat_vel_control::g_control->start()) {
			delete mc_quat_vel_control::g_control;
			mc_quat_vel_control::g_control = nullptr;
			err(1, "start failed");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (mc_quat_vel_control::g_control == nullptr)
			errx(1, "not running");

		delete mc_quat_vel_control::g_control;
		mc_quat_vel_control::g_control = nullptr;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (mc_quat_vel_control::g_control) {
			errx(0, "running");

		} else {
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	return 1;
}
