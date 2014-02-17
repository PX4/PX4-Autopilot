#ifndef JETDRIVE_CONTROL_H
#define JETDRIVE_CONTROL_H


#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
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
#include <systemlib/pid/pid.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>


/**
 * Multicopter attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int jetdrive_control_main(int argc, char *argv[]);

#define MIN_TAKEOFF_THROTTLE	0.3f
#define YAW_DEADZONE	0.05f
#define RATES_I_LIMIT	0.5f

class JetdriveControl
{
public:
	/**
	 * Constructor
	 */
	JetdriveControl();

	/**
	 * Destructor, also kills the sensors task.
	 */
	~JetdriveControl();

	/**
	 * Start the sensors task.
	 *
	 * @return		OK on success.
	 */
	int		start();

private:

	bool	_task_should_exit;		/**< if true, sensor task should exit */
	int		_control_task;			/**< task handle for sensor task */

	int		_v_att_sub;				/**< vehicle attitude subscription */
	int		_v_att_sp_sub;			/**< vehicle attitude setpoint subscription */
	int		_v_rates_sp_sub;		/**< vehicle rates setpoint subscription */
	int		_v_control_mode_sub;	/**< vehicle control mode subscription */
	int		_params_sub;			/**< parameter updates subscription */
	int		_manual_control_sp_sub;	/**< manual control setpoint subscription */
	int		_armed_sub;				/**< arming status subscription */

	orb_advert_t	_att_sp_pub;			/**< attitude setpoint publication */
	orb_advert_t	_v_rates_sp_pub;		/**< rate setpoint publication */
	orb_advert_t	_actuators_0_pub;		/**< attitude actuator controls publication */

	struct vehicle_attitude_s			_v_att;				/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s	_v_att_sp;			/**< vehicle attitude setpoint */
	struct vehicle_rates_setpoint_s		_v_rates_sp;		/**< vehicle rates setpoint */
	struct manual_control_setpoint_s	_manual_control_sp;	/**< manual control setpoint */
	struct vehicle_control_mode_s		_v_control_mode;	/**< vehicle control mode */
	struct actuator_controls_s			_actuators;			/**< actuator controls */
	struct actuator_armed_s				_armed;				/**< actuator arming status */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	math::Matrix<3, 3>	_R_sp;			/**< attitude setpoint rotation matrix */
	math::Matrix<3, 3>	_R;				/**< rotation matrix for current state */
	math::Vector<3>		_rates_prev;	/**< angular rates on previous step */
	math::Vector<3>		_rates_sp;		/**< angular rates setpoint */
	math::Vector<3>		_rates_int;		/**< angular rates integral error */
	float				_thrust_sp;		/**< thrust setpoint */
	math::Vector<3>		_att_control;	/**< attitude control vector */

	math::Matrix<3, 3>	I;				/**< identity matrix */

	bool	_reset_yaw_sp;			/**< reset yaw setpoint flag */

	struct {
		param_t roll_p;
		param_t roll_rate_p;
		param_t roll_rate_i;
		param_t roll_rate_d;
		param_t pitch_p;
		param_t pitch_rate_p;
		param_t pitch_rate_i;
		param_t pitch_rate_d;
		param_t yaw_p;
		param_t yaw_rate_p;
		param_t yaw_rate_i;
		param_t yaw_rate_d;
		param_t yaw_ff;

		param_t rc_scale_yaw;
	}		_params_handles;		/**< handles for interesting parameters */

	struct {
		math::Vector<3> att_p;					/**< P gain for angular error */
		math::Vector<3> rate_p;				/**< P gain for angular rate error */
		math::Vector<3> rate_i;				/**< I gain for angular rate error */
		math::Vector<3> rate_d;				/**< D gain for angular rate error */
		float yaw_ff;						/**< yaw control feed-forward */

		float rc_scale_yaw;
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
	 * Check for arming status updates.
	 */
	void		arming_status_poll();

	/**
	 * Attitude controller.
	 */
	void		control_attitude(float dt);

	/**
	 * Attitude rates controller.
	 */
	void		control_attitude_rates(float dt);

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main sensor collection task.
	 */
	void		task_main() __attribute__((noreturn));
};


#ifndef jetdrive_control
#define jetdrive_control

namespace jetdrive_control
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

JetdriveControl	*g_control;
}

#endif // jetdrive_control

#endif // JETDRIVE_CONTROL_H
