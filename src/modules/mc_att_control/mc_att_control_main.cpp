
/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file mc_att_control_main.cpp
 * Multicopter attitude controller. ¶àĞıÒí×ËÌ¬¿ØÖÆÆ÷
 *
 * Publication for the desired attitude tracking:
 * 
 * Daniel Mellinger and Vijay Kumar. Minimum Snap Trajectory Generation and Control for Quadrotors.
 * Int. Conf. on Robotics and Automation, Shanghai, China, May 2011.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * The controller has two loops: P loop for angular error and PD loop for angular rate error.
 * ¿ØÖÆÆ÷ÓĞÁ½¸öÑ­»·: ½Ç¶ÈÎó²îµÄP»ØÂ·Óë½ÇËÙ¶ÈÎó²îµÄPD»ØÂ·
 * Desired rotation calculated keeping in mind that yaw response is normally slower than roll/pitch.
 * Æ«º½ÏìÓ¦±Èºá¹ö/¸©ÑöÂı
 * For small deviations controller rotates copter to have shortest path of thrust vector and independently rotates around yaw,
 * ¶ÔÓÚĞ¡µÄÆ«²î£¬¿ØÖÆÆ÷Ğı×ª·ÉĞĞÆ÷Ê¹ÍÆÁ¦ÏòÁ¿ÒÔ×î¶ÌµÄÂ·¾¶¶ÀÁ¢µÄÈÆºá¹öÖá×ª¶¯
 * so actual rotation axis is not constant. For large deviations controller rotates copter around fixed axis.
 * Êµ¼Ê×ª¶¯Öá²»ÊÇ³£Á¿¡£¶ÔÓÚ´óµÄÆ«²î¿ØÖÆÆ÷ÈÆ¹Ì¶¨Öá×ª¶¯·ÉĞĞÆ÷
 * These two approaches fused seamlessly with weight depending on angular error.
 * ÕâÁ½¸ö·½·¨¸ù¾İ½Ç¶ÈÎó²îÎŞ·ìÈÚºÏ
 * When thrust vector directed near-horizontally (e.g. roll ~= PI/2) yaw setpoint ignored because of singularity.
 * µ±ÍÆÁ¦ÏòÁ¿Ö¸ÏòË®Æ½£¬Æ«º½Éèµã¿ÉÒÔºöÂÔ£¬ÒòÎªÆæµãÎÊÌâ   (ÍòÏòËø)
 * Controller doesn't use Euler angles for work, they generated only for more human-friendly control and logging.
 * ¿ØÖÆÆ÷²»ÓÃÅ·À­½Ç¹¤×÷£¬Å·À­½ÇÖ»ÊÇÓÃÀ´¸ü¼ÓÈËĞÔ»¯µÄ¿ØÖÆÓë¼ÇÂ¼£¬ÒòÎªÅ·À­½ÇºÜÖ±¹Û£¬ÓëÉú»îÏà½ü
 * If rotation matrix setpoint is invalid it will be generated from Euler angles for compatibility with old position controllers.
 * Èç¹ûĞı×ª¾ØÕóÉèµãÎŞĞ§£¬½«ÓÃÅ·À­½ÇÉú³É¾ØÕó£¬ÒÔ¼æÈİ¾ÉµÄÎ»ÖÃ¿ØÖÆÆ÷
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
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
#include <uORB/topics/actuator_controls_virtual_fw.h>
#include <uORB/topics/actuator_controls_virtual_mc.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/fw_virtual_rates_setpoint.h>
#include <uORB/topics/mc_virtual_rates_setpoint.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/mc_att_ctrl_status.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/circuit_breaker.h>
#include <lib/mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <lib/tailsitter_recovery/tailsitter_recovery.h>

/**
 * Multicopter attitude control app start / stop handling function
 * ¶àĞıÒí×ËÌ¬¿ØÖÆÓ¦ÓÃ ¿ªÊ¼/½áÊø ´¦Àíº¯Êı
 * @ingroup apps
 */
 
/* PX4µÄ×ËÌ¬¿ØÖÆ²¿·ÖÊ¹ÓÃµÄÊÇroll-pitchºÍyaw·Ö¿ª¿ØÖÆµÄ£¨ÊÇÎªÁË½âñî¿ØÖÆĞĞÎª) 
 * ¼´ tilt(ÇãĞ±)ºÍtorsion(×ªÍä)Á½¸ö»·½Ú
 */
 
/*
 * ¿ØÖÆÁ÷³Ì£º 
 * 1£©Ô¤´¦Àí£º¸÷²ÎÊıµÄ³õÊ¼»¯¡£ 
 * 2£©ÎÈ¶¨roll-pitchµÄ½ÇËÙ¶È¡£
 * 3£©ÎÈ¶¨roll-pitchµÄ½Ç¶È¡£ 
 * 4£©ÎÈ¶¨yawµÄ½ÇËÙ¶È¡£ 
 * 5£©ÎÈ¶¨yawµÄ½Ç¶È¡£ ÆäÖĞÓĞÒ»¸öyawµÄÇ°À¡¿ØÖÆ£¨MC_YAW_FF£©
 */
extern "C" __EXPORT int mc_att_control_main(int argc, char *argv[]);

#define YAW_DEADZONE	0.05f
#define MIN_TAKEOFF_THRUST    0.2f
#define RATES_I_LIMIT	0.3f
#define MANUAL_THROTTLE_MAX_MULTICOPTER	0.9f
#define ATTITUDE_TC_DEFAULT 0.2f

class MulticopterAttitudeControl
{
public:
	/**
	 * Constructor
	 */
	MulticopterAttitudeControl();

	/**
	 * Destructor, also kills the main task
	 */
	~MulticopterAttitudeControl();

	/**
	 * Start the multicopter attitude control task.
	 *
	 * @return		OK on success.
	 */
	int		start();

private:

	bool	_task_should_exit;		/**< if true, task_main() should exit  */
	  // Èç¹ûÎªÕæ£¬task_main()ÍË³ö
	int		_control_task;			/**< task handle ÈÎÎñ¾ä±ú */

	int		_ctrl_state_sub;		/**< control state subscription  ¶©ÔÄ¿ØÖÆ×´Ì¬*/
	int		_v_att_sp_sub;			/**< vehicle attitude setpoint subscription  ¶©ÔÄ·ÉĞĞÆ÷×ËÌ¬Éè¶¨Öµ*/
	int		_v_rates_sp_sub;		/**< vehicle rates setpoint subscription ¶©ÔÄ·ÉĞĞÆ÷µÄËÙ¶ÈÉè¶¨Öµ*/
	int		_v_control_mode_sub;	/**< vehicle control mode subscription  ¶©ÔÄ·É»úµÄ¿ØÖÆÄ£Ê½*/
	int		_params_sub;			/**< parameter updates subscription  ¶©ÔÄ²ÎÊı¸üĞÂ */
	int		_manual_control_sp_sub;	/**< manual control setpoint subscription ¶©ÔÄÊÖ¶¯¿ØÖÆÉè¶¨Öµ*/
	int		_armed_sub;				/**< arming status subscription */
	int		_vehicle_status_sub;	    /**< vehicle status subscription */
	int 	_motor_limits_sub;		/**< motor limits subscription */

	orb_advert_t	_v_rates_sp_pub;		/**< rate setpoint publication */
	orb_advert_t	_actuators_0_pub;		/**< attitude actuator controls publication */
	orb_advert_t	_controller_status_pub;	/**< controller status publication */

	orb_id_t _rates_sp_id;	   /**< pointer to correct rates setpoint uORB metadata structure */
							   // uORBÔªÊı¾İ½á¹¹Ìå½ÃÕıËÙ¶ÈÉè¶¨ÖµµÄÖ¸Õë
							   
	orb_id_t _actuators_id;	/**< pointer to correct actuator controls0 uORB metadata structure */

	bool		_actuators_0_circuit_breaker_enabled;	/**< circuit breaker to suppress output */
															// ÒÖÖÆÊä³öµÄ¶ÏÂ·Æ÷

	struct control_state_s				_ctrl_state;		/**< control state ¿ØÖÆ×´Ì¬*/
	struct vehicle_attitude_setpoint_s	_v_att_sp;			/**< vehicle attitude setpoint ·ÉĞĞÆ÷×ËÌ¬Éè¶¨Öµ */
	struct vehicle_rates_setpoint_s		_v_rates_sp;		/**< vehicle rates setpoint ·ÉĞĞÆ÷ËÙ¶ÈÉè¶¨Öµ*/
	struct manual_control_setpoint_s	_manual_control_sp;	/**< manual control setpoint ÊÖ¶¯¿ØÖÆÉè¶¨Öµ*/
	struct vehicle_control_mode_s		_v_control_mode;	    /**< vehicle control mode */
	struct actuator_controls_s			_actuators;			/**< actuator controls */
	struct actuator_armed_s				_armed;				/**< actuator arming status */
	struct vehicle_status_s				_vehicle_status;	/**< vehicle status */
	struct multirotor_motor_limits_s	_motor_limits;		/**< motor limits */
	struct mc_att_ctrl_status_s 		_controller_status; /**< controller status ¿ØÖÆÆ÷×´Ì¬*/

	perf_counter_t	_loop_perf;			/**< loop performance counter */
	perf_counter_t	_controller_latency_perf;

	math::Vector<3>		_rates_prev;		/**< angular rates on previous step Ç°Ò»²½µÄ½ÇËÙ¶È */
	math::Vector<3>		_rates_sp_prev;   /**< previous rates setpoint Ö®Ç°µÄ½ÇËÙ¶ÈÉè¶¨Öµ*/
	math::Vector<3>		_rates_sp;			/**< angular rates setpoint ½ÇËÙ¶ÈÉè¶¨Öµ*/
	math::Vector<3>		_rates_int;		/**< angular rates integral error ½ÇËÙ¶È»ı·ÖÎó²î*/
	float					_thrust_sp;		/**< thrust setpoint ÍÆÁ¦Éè¶¨Öµ*/
	math::Vector<3>		_att_control;		/**< attitude control vector ×ËÌ¬¿ØÖÆÏòÁ¿-Å·À­½Ç*/

	math::Matrix<3, 3>  _I;					/**< identity matrix µ¥Î»¾ØÕó*/

	struct {
		param_t roll_p;
		param_t roll_rate_p;
		param_t roll_rate_i;
		param_t roll_rate_d;
		param_t roll_rate_ff;
		param_t pitch_p;
		param_t pitch_rate_p;
		param_t pitch_rate_i;
		param_t pitch_rate_d;
		param_t pitch_rate_ff;
		param_t yaw_p;
		param_t yaw_rate_p;
		param_t yaw_rate_i;
		param_t yaw_rate_d;
		param_t yaw_rate_ff;
		param_t yaw_ff;
		param_t roll_rate_max;
		param_t pitch_rate_max;
		param_t yaw_rate_max;

		param_t acro_roll_max;
		param_t acro_pitch_max;
		param_t acro_yaw_max;
		param_t rattitude_thres;

		param_t vtol_type;
		param_t roll_tc;
		param_t pitch_tc;

	}		_params_handles;		/**< handles for interesting parameters  */
															   // ¸ĞĞËÈ¤µÄ²ÎÊıµÄ¾ä±ú
	struct {
		math::Vector<3> att_p;					/**< P gain for angular error  ½Ç¶ÈÎó²îµÄPÔöÒæ*/
		math::Vector<3> rate_p;				/**< P gain for angular rate error ½ÇËÙ¶ÈÎó²îµÄPÔöÒæ*/
		math::Vector<3> rate_i;				/**< I gain for angular rate error ½ÇËÙ¶ÈÎó²îµÄIÔöÒæ*/
		math::Vector<3> rate_d;				/**< D gain for angular rate error ½ÇËÙ¶ÈÎó²îµÄDÔöÒæ*/
		math::Vector<3>	rate_ff;			/**< Feedforward gain for desired rates ÆÚÍûËÙ¶ÈµÄÇ°À¡ÔöÒæ*/
		float yaw_ff;						/**< yaw control feed-forward Æ«º½¿ØÖÆµÄÇ°À¡*/

		float roll_rate_max;
		float pitch_rate_max;
		float yaw_rate_max;
		math::Vector<3> mc_rate_max;		/**< attitude rate limits in stabilized modes */
											// ×ÔÎÈÄ£Ê½ÏÂµÄ½ÇËÙ¶È

		math::Vector<3> acro_rate_max;		/**< max attitude rates in acro mode  */
											// ÌØ¼¼Ä£Ê½ÏÂµÄ×î´óËÙ¶È
		float rattitude_thres;
		int vtol_type;						/**< 0 = Tailsitter, 1 = Tiltrotor, 2 = Standard airframe */
	}		_params;

	TailsitterRecovery *_ts_opt_recovery;	/**< Computes optimal rates for tailsitter recovery */
											//¼ÆËãtailsitterµÄ×îÓÅËÙ¶È

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
	 * Attitude controller. ×ËÌ¬¿ØÖÆÆ÷
	 */
	void		control_attitude(float dt);

	/**
	 * Attitude rates controller.
	 */
	void		control_attitude_rates(float dt);

	/**
	 * Check for vehicle status updates.
	 */
	void		vehicle_status_poll();

	/**
	 * Check for vehicle motor limits status.
	 */
	void		vehicle_motor_limits_poll();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main attitude control task.
	 */
	void		task_main();
};

namespace mc_att_control
{

MulticopterAttitudeControl	*g_control;
}

MulticopterAttitudeControl::MulticopterAttitudeControl() :

	_task_should_exit(false),
	_control_task(-1),

	/* subscriptions */
	_ctrl_state_sub(-1),
	_v_att_sp_sub(-1),
	_v_control_mode_sub(-1),
	_params_sub(-1),
	_manual_control_sp_sub(-1),
	_armed_sub(-1),
	_vehicle_status_sub(-1),

	/* publications */
	_v_rates_sp_pub(nullptr),
	_actuators_0_pub(nullptr),
	_controller_status_pub(nullptr),
	_rates_sp_id(0),
	_actuators_id(0),

	_actuators_0_circuit_breaker_enabled(false),

	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "mc_att_control")),
	_controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency")),
	_ts_opt_recovery(nullptr)

{
	memset(&_ctrl_state, 0, sizeof(_ctrl_state));
	memset(&_v_att_sp, 0, sizeof(_v_att_sp));
	memset(&_v_rates_sp, 0, sizeof(_v_rates_sp));
	memset(&_manual_control_sp, 0, sizeof(_manual_control_sp));
	memset(&_v_control_mode, 0, sizeof(_v_control_mode));
	memset(&_actuators, 0, sizeof(_actuators));
	memset(&_armed, 0, sizeof(_armed));
	memset(&_vehicle_status, 0, sizeof(_vehicle_status));
	memset(&_motor_limits, 0, sizeof(_motor_limits));
	memset(&_controller_status, 0, sizeof(_controller_status));
	_vehicle_status.is_rotary_wing = true;

	_params.att_p.zero();
	_params.rate_p.zero();
	_params.rate_i.zero();
	_params.rate_d.zero();
	_params.rate_ff.zero();
	_params.yaw_ff = 0.0f;
	_params.roll_rate_max = 0.0f;
	_params.pitch_rate_max = 0.0f;
	_params.yaw_rate_max = 0.0f;
	_params.mc_rate_max.zero();
	_params.acro_rate_max.zero();
	_params.rattitude_thres = 1.0f;

	_rates_prev.zero();
	_rates_sp.zero();
	_rates_sp_prev.zero();
	_rates_int.zero();
	_thrust_sp = 0.0f;
	_att_control.zero();

	_I.identity();

	_params_handles.roll_p			= 	param_find("MC_ROLL_P");
	_params_handles.roll_rate_p		= 	param_find("MC_ROLLRATE_P");
	_params_handles.roll_rate_i		= 	param_find("MC_ROLLRATE_I");
	_params_handles.roll_rate_d		= 	param_find("MC_ROLLRATE_D");
	_params_handles.roll_rate_ff	= 	param_find("MC_ROLLRATE_FF");
	_params_handles.pitch_p			= 	param_find("MC_PITCH_P");
	_params_handles.pitch_rate_p	= 	param_find("MC_PITCHRATE_P");
	_params_handles.pitch_rate_i	= 	param_find("MC_PITCHRATE_I");
	_params_handles.pitch_rate_d	= 	param_find("MC_PITCHRATE_D");
	_params_handles.pitch_rate_ff 	= 	param_find("MC_PITCHRATE_FF");
	_params_handles.yaw_p			=	param_find("MC_YAW_P");
	_params_handles.yaw_rate_p		= 	param_find("MC_YAWRATE_P");
	_params_handles.yaw_rate_i		= 	param_find("MC_YAWRATE_I");
	_params_handles.yaw_rate_d		= 	param_find("MC_YAWRATE_D");
	_params_handles.yaw_rate_ff	 	= 	param_find("MC_YAWRATE_FF");
	_params_handles.yaw_ff			= 	param_find("MC_YAW_FF");
	_params_handles.roll_rate_max	= 	param_find("MC_ROLLRATE_MAX");
	_params_handles.pitch_rate_max	= 	param_find("MC_PITCHRATE_MAX");
	_params_handles.yaw_rate_max	= 	param_find("MC_YAWRATE_MAX");
	_params_handles.acro_roll_max	= 	param_find("MC_ACRO_R_MAX");
	_params_handles.acro_pitch_max	= 	param_find("MC_ACRO_P_MAX");
	_params_handles.acro_yaw_max	= 	param_find("MC_ACRO_Y_MAX");
	_params_handles.rattitude_thres = 	param_find("MC_RATT_TH");
	_params_handles.vtol_type 		= 	param_find("VT_TYPE");
	_params_handles.roll_tc			= 	param_find("MC_ROLL_TC");
	_params_handles.pitch_tc		= 	param_find("MC_PITCH_TC");

	/* fetch initial parameter values */
	parameters_update();

	if (_params.vtol_type == 0) {
		// the vehicle is a tailsitter, use optimal recovery control strategy
		_ts_opt_recovery = new TailsitterRecovery();
	}


}

MulticopterAttitudeControl::~MulticopterAttitudeControl()
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
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	delete _ts_opt_recovery;
	mc_att_control::g_control = nullptr;
}

int
MulticopterAttitudeControl::parameters_update()
{
	float v;

	float roll_tc, pitch_tc;

	param_get(_params_handles.roll_tc, &roll_tc);
	param_get(_params_handles.pitch_tc, &pitch_tc);

	/* roll gains ºá¹öÔöÒæ */
	param_get(_params_handles.roll_p, &v);
	_params.att_p(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc);
	param_get(_params_handles.roll_rate_p, &v);
	_params.rate_p(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc);
	param_get(_params_handles.roll_rate_i, &v);
	_params.rate_i(0) = v;
	param_get(_params_handles.roll_rate_d, &v);
	_params.rate_d(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc);
	param_get(_params_handles.roll_rate_ff, &v);
	_params.rate_ff(0) = v;

	/* pitch gains */
	param_get(_params_handles.pitch_p, &v);
	_params.att_p(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
	param_get(_params_handles.pitch_rate_p, &v);
	_params.rate_p(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
	param_get(_params_handles.pitch_rate_i, &v);
	_params.rate_i(1) = v;
	param_get(_params_handles.pitch_rate_d, &v);
	_params.rate_d(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
	param_get(_params_handles.pitch_rate_ff, &v);
	_params.rate_ff(1) = v;

	/* yaw gains */
	param_get(_params_handles.yaw_p, &v);
	_params.att_p(2) = v;
	param_get(_params_handles.yaw_rate_p, &v);
	_params.rate_p(2) = v;
	param_get(_params_handles.yaw_rate_i, &v);
	_params.rate_i(2) = v;
	param_get(_params_handles.yaw_rate_d, &v);
	_params.rate_d(2) = v;
	param_get(_params_handles.yaw_rate_ff, &v);
	_params.rate_ff(2) = v;

	param_get(_params_handles.yaw_ff, &_params.yaw_ff);

	/* angular rate limits */
	param_get(_params_handles.roll_rate_max, &_params.roll_rate_max);
	_params.mc_rate_max(0) = math::radians(_params.roll_rate_max);
	param_get(_params_handles.pitch_rate_max, &_params.pitch_rate_max);
	_params.mc_rate_max(1) = math::radians(_params.pitch_rate_max);
	param_get(_params_handles.yaw_rate_max, &_params.yaw_rate_max);
	_params.mc_rate_max(2) = math::radians(_params.yaw_rate_max);

	/* manual rate control scale(±ÈÂÊ¿ØÖÆÄ£Ê½-ACROÌØ¼¼Ä£Ê½) and auto mode roll/pitch rate limits */
	param_get(_params_handles.acro_roll_max, &v);
	_params.acro_rate_max(0) = math::radians(v);
	param_get(_params_handles.acro_pitch_max, &v);
	_params.acro_rate_max(1) = math::radians(v);
	param_get(_params_handles.acro_yaw_max, &v);
	_params.acro_rate_max(2) = math::radians(v);

	/* stick deflection needed in rattitude mode to control rates not angles */
	// rAttitudeÄ£Ê½ÏÂ¿ØÖÆËÙ¶È¶ø·Ç½Ç¶ÈµÄ ¹Ì¶¨Ğı×ª
	param_get(_params_handles.rattitude_thres, &_params.rattitude_thres);

	param_get(_params_handles.vtol_type, &_params.vtol_type);

	_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled("CBRK_RATE_CTRL", CBRK_RATE_CTRL_KEY);

	return OK;
}

void
MulticopterAttitudeControl::parameter_update_poll()
{
	bool updated;

	/* Check if parameters have changed */
	orb_check(_params_sub, &updated);

	if (updated) {
		struct parameter_update_s param_update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
		parameters_update();
	}
}

void
MulticopterAttitudeControl::vehicle_control_mode_poll()
{
	bool updated;

	/* Check if vehicle control mode has changed */
	orb_check(_v_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _v_control_mode_sub, &_v_control_mode);
	}
}

void
MulticopterAttitudeControl::vehicle_manual_poll()
{
	bool updated;

	/* get pilots inputs */
	orb_check(_manual_control_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, &_manual_control_sp);
	}
}

void
MulticopterAttitudeControl::vehicle_attitude_setpoint_poll()
{
	/* check if there is a new setpoint */
	// ¼ì²éÊÇ·ñÓĞĞÂµÄÉè¶¨Öµ
	bool updated;
	orb_check(_v_att_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_sub, &_v_att_sp);
	}
}

void
MulticopterAttitudeControl::vehicle_rates_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_rates_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);
	}
}

void
MulticopterAttitudeControl::arming_status_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_armed_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
	}
}

void
MulticopterAttitudeControl::vehicle_status_poll()
{
	/* check if there is new status information */
	bool vehicle_status_updated;
	orb_check(_vehicle_status_sub, &vehicle_status_updated);

	if (vehicle_status_updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);

		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (!_rates_sp_id) {
			if (_vehicle_status.is_vtol) {
				_rates_sp_id = ORB_ID(mc_virtual_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_virtual_mc);

			} else {
				_rates_sp_id = ORB_ID(vehicle_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_0);
			}
		}
	}
}

void
MulticopterAttitudeControl::vehicle_motor_limits_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_motor_limits_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(multirotor_motor_limits), _motor_limits_sub, &_motor_limits);
	}
}

/**
 * Attitude controller.
 * Input: 'vehicle_attitude_setpoint' topics (depending on mode)
 * Output: '_rates_sp' vector, '_thrust_sp'
 */
 
 /*
 * ×ËÌ¬¿ØÖÆÆ÷ ¼´ Å·À­½Ç¿ØÖÆ
 *ÊäÈë: '·ÉĞĞÆ÷µÄ×ËÌ¬Éè¶¨Öµ vehicle_attitude_setpoint'»°Ìâ(È¡¾öÓÚÄ£Ê½)
 *Êä³ö:'½ÇËÙ¶ÈÉè¶¨Öµrate_sp'ÏòÁ¿£¬ 'ÓÍÃÅÉè¶¨Öµthrust_sp'
 */

// x,yÖáÓëzÖá·Ö¿ª¿ØÖÆ£¬Ä¿µÄÊÇÎªÁË½âñî¿ØÖÆĞĞÎª
// ·Ö±ğÖ´ĞĞ½Ï¿ìÏàÓ¦µÄ¶¯×÷ºÍ½ÏÂıÏìÓ¦µÄ¶¯×÷
// Îó²î¾ØÕó = ×ª¶¯¾ØÕó * ÇãĞ±¾ØÕó   Re = Rtorsion * Rtilt  
// ÏÈRtiltÊ¹µ±Ç°×ËÌ¬µÄZÖáºÍÄ¿±ê×ËÌ¬µÄZÖá¶ÔÆë£¬È»ºóÔÙ½øĞĞRtorsionĞı×ª¶ÔÆëXYÖá
void
MulticopterAttitudeControl::control_attitude(float dt)
{
	vehicle_attitude_setpoint_poll(); // Ê×ÏÈ¾ÍÊÇÍ¨¹ıuORBÄ£ĞÍ¼ì²â×ËÌ¬Êı¾İÊÇ·ñÒÑ¾­¸üĞÂ¡£
	                                  // ¼ì²âµ½¸üĞÂÊı¾İÒÔºó£¬°ÑÊı¾İ¿½±´µ½µ±Ç°

	_thrust_sp = _v_att_sp.thrust; //°ÑÓÍÃÅ¿ØÖÆÁ¿¸³Öµ¸ø¿ØÖÆ±äÁ¿¡£

	/* construct attitude setpoint rotation matrix */
	// ¹¹Ôì×ËÌ¬Éè¶¨ÖµĞı×ª¾ØÕó £¨Ä¿±ê×´Ì¬£¬ËùÎ½µÄTargetRotation£©¡£
	math::Matrix<3, 3> R_sp;
	R_sp.set(_v_att_sp.R_body);  //body-»úÌå  ¸Ã¾ØÕóµÄ¸÷¸öÔªËØÊÇ»úÌåµÄÅ·À­½Ç

	/* get current rotation matrix from control state quaternions */
	// ´Ó¿ØÖÆ×´Ì¬ËÄÔªÊıÖĞÈ¡µÃµ±Ç°Ğı×ª¾ØÕó
	math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
	math::Matrix<3, 3> R = q_att.to_dcm(); //½«¿ØÖÆ×´Ì¬ËÄÔªÊı×ª»»ÎªDCM

	/* all input data is ready, run controller itself */
	// ËùÓĞµÄÊäÈëÊı¾İÒÑ¾­×¼±¸ºÃÁË£¬ÔËĞĞ¿ØÖÆÆ÷

	/* try to move thrust vector shortest way, because yaw response is slower than roll/pitch */
	// ³¢ÊÔÒÔ×î¶ÌµÄÂ·¾¶ÒÆ¶¯ÍÆÁ¦ÏòÁ¿£¬ÒòÎªÆ«º½ÏìÓ¦±Èºá¹ö/¸©ÑöÂı
	// È¡³öÁ½¸ö¾ØÕóµÄZÖáÏòÁ¿
	math::Vector<3> R_z(R(0, 2), R(1, 2), R(2, 2));
	math::Vector<3> R_sp_z(R_sp(0, 2), R_sp(1, 2), R_sp(2, 2));

	/* axis and sin(angle) of desired rotation */
	/* ÖáÒÔ¼°ÓëÆÚÍûĞı×ªµÄ½Ç¶ÈÕıÏÒsin(angle)
	 * ¸ù¾İÕâÁ½¸öZÖá¼ÆËã³öÎó²îÏòÁ¿(²Î¿¼×ø±êÏµ)£¬²¢×ª»»µ½»úÌå×ø±êÏµ
	 *
	 * µ±Ç°×ËÌ¬µÄzÖáºÍÄ¿±ê×ËÌ¬µÄzÖáµÄÎó²î´óĞ¡£¨¼´ĞèÒªĞı×ªµÄ½Ç¶È£©²¢Ğı×ªµ½
	 * bÏµ£¨¼´ÏÈ¶ÔÆëZÖá£©¡£
	 */
	math::Vector<3> e_R = R.transposed() * (R_z % R_sp_z);
    // ¾ÍÊÇÇóÈ¡Îó²îµÄ£¬±¾À´Ó¦¸ÃzÖáÏà»¥ÖØºÏµÄ£¬Èç¹û²»ÊÇ0¾Í×÷ÎªÎó²îÏî¡£
    // È»ºóÔÙ×ó³ËĞı×ª¾ØÕóĞı×ªµ½bÏµ¡£


	/* calculate angle error */
	// ¼ÆËã×ËÌ¬½Ç¶ÈÎó²î
	/*
	*ÓÉ¹«Ê½a¡Áb=|a||b|sin(theta)£¬a¡¤b=|a||b|cos(theta)¡£
	*ÕâÀïR_zºÍR_sp_z¶¼ÊÇµ¥Î»ÏòÁ¿£¬Ä£Îª1
	*Îó²îÏòÁ¿e_RµÄÄ£¾ÍÊÇsin(theta)£¬µã»ı¾ÍÊÇcos(theta)
	*/
	float e_R_z_sin = e_R.length(); // length()¼ÆËãÏòÁ¿µÄÄ£ length-->Æ½·½¸ù
	float e_R_z_cos = R_z * R_sp_z; // ¼Ğ½ÇÓàÏÒ

	/* calculate weight for yaw control */
	// ¼ÆËãÓÃÓÚÆ«º½¿ØÖÆµÄÈ¨ÖØ	
	float yaw_w = R_sp(2, 2) * R_sp(2, 2);

	/*ÒòÎª¶àÖáµÄyawÏìÓ¦Ò»°ã±Èroll/pitchÂı£¬Òò´Ë½«Á½Õß½âñî£¬ÏÈ²¹³¥roll/pitchµÄ±ä»¯£¬¼ÆËãR_rp*/
	/* calculate rotation matrix after roll/pitch only rotation */
	// ¼ÆËãÔÚ½ö½øĞĞºá¹ö/¸©ÑöĞı×ªÖ®ºóĞı×ª¾ØÕó
	math::Matrix<3, 3> R_rp;

	if (e_R_z_sin > 0.0f) {  //ÅĞ¶ÏÁ½¸öZÖáÊÇ·ñ´æÔÚÎó²î
		/* get axis-angle representation */
		// È¡µÃÖá-½ÇÖ®¼äµÄ¹ØÏµ   r=(u,theta)
		float e_R_z_angle = atan2f(e_R_z_sin, e_R_z_cos);  // Ğı×ª½Ç¶È theta
		math::Vector<3> e_R_z_axis = e_R / e_R_z_sin;      // ÇãĞ±µÄĞı×ªÖáu
									// ¹éÒ»»¯£¬ÒòÎªe_R_z_axisÊÇ¸÷ÔªËØµÄÆ½·½ºÍ

		e_R = e_R_z_axis * e_R_z_angle; //½«e_R×ª³Éµ¥Î»ÏòÁ¿(Ö÷ÒªÊÇÎªÁËÓÃ½Ç¶ÈÁ¿±íÊ¾Îó²îÏòÁ¿)

		/* cross product matrix for e_R_axis */
		// ÒÔe_R_axisÎªĞı×ªÖáµÄÏòÁ¿²æ»ı¾ØÕó Rcp(CrossProduct) 
		// µÃµ½Ò»¸ö·´¶Ô³Æ¾ØÕó£¬¿ÉÒÔ±íÊ¾Ğı×ª¡£Ò»¸öÏòÁ¿ÓĞÈı¸ö×ÔÓÉ¶È£¬·´¶Ô³Æ¾ØÕóÒ²Ö»ÓĞÈı¸ö±äÁ¿
		math::Matrix<3, 3> e_R_cp;
		e_R_cp.zero();
		e_R_cp(0, 1) = -e_R_z_axis(2);
		e_R_cp(0, 2) = e_R_z_axis(1);
		e_R_cp(1, 0) = e_R_z_axis(2);
		e_R_cp(1, 2) = -e_R_z_axis(0);
		e_R_cp(2, 0) = -e_R_z_axis(1);
		e_R_cp(2, 1) = e_R_z_axis(0);

		/* rotation matrix for roll/pitch only rotation */
		// ½öÓÃÓÚºá¹ö/¸©ÑöĞı×ªµÄĞı×ª¾ØÕó
		
		R_rp = R * (_I + e_R_cp * e_R_z_sin + e_R_cp * e_R_cp * (1.0f - e_R_z_cos)); // ÂŞµÂÀï¸ñĞı×ª¹«Ê½£ºRodrigues rotation formula
		// _I´ú±íµ¥Î»Õó

	} else {
		/* zero roll/pitch rotation */
		// 0ºá¹ö/¸©ÑöĞı×ª
		R_rp = R;
	}
    /*
	 * ÉÏÊö´¦Àí¹ı³ÌÖĞµÄ DCM Á¿¶¼ÊÇÍ¨¹ıÅ·À­½ÇÀ´±íÊ¾µÄ£¬Õâ¸öÖ÷Òª¾Í
     * ÊÇ¿¼ÂÇÔÚ¿ØÖÆÊ±ĞèÒªÃ÷È·¾ßÌåµÄÅ·À­½ÇµÄ´óĞ¡£¬»¹ÓĞ¾ÍÊÇËã·¨µÄ½âËã¹ı³ÌÊÇÍ¨¹ı¾ØÕóÎ¢·Ö
	 * ·½³ÌÍÆµ¼µÃµ½µÄ
	 */
	 
	/*ÏÖÔÚZÖáÒÑ¾­ÖØºÏÁË£¬Ö»ĞèÒªÇóyawµÄÎó²î½Ç¶È*/

	/* R_rp and R_sp has the same Z axis, calculate yaw error */
	// R_spÊÇ ×ËÌ¬Éè¶¨ÖµĞı×ª¾ØÕó sp=setpoint
	// R_rpºÍR_sp¾ßÓĞÏàÍ¬µÄZÖá¡£  ¼ÆËãÆ«º½Îó²î
	// È¡³öÁ½¸ö¾ØÕóµÄXÖá(ÏÖÔÚÖ»ÓĞxÖá,yÖá´æÔÚÎó²î)
	math::Vector<3> R_sp_x(R_sp(0, 0), R_sp(1, 0), R_sp(2, 0));
	math::Vector<3> R_rp_x(R_rp(0, 0), R_rp(1, 0), R_rp(2, 0));
	//Í¬Ñù¸ù¾İÏòÁ¿µÄ²æ»ıºÍµã»ıÇó³öÎó²î½Ç¶ÈµÄÕıÏÒºÍÓàÏÒ£¬ÔÙ·´ÕıÇĞÇó³ö½Ç¶È£»
	e_R(2) = atan2f((R_rp_x % R_sp_x) * R_sp_z, R_rp_x * R_sp_x) * yaw_w;

   /*
	* ÒÔÉÏĞı×ª·½·¨£¬ÊÊÓÃÓÚĞ¡½Ç¶ÈµÄÎó²î£¬µ±×ª¶¯µÄ½Ç¶ÈÆ«´óÊ±£¬»¹ĞèÁíÒ»ÖÖ·½·¨£»
	* Ö±½Ó¼ÆËã²Î¿¼Ïµµ½»úÌåÏµµÄĞı×ª¾ØÕó£¬²¢×ª»»³ÉËÄÔªÊıĞÎÊ½
	*/
	if (e_R_z_cos < 0.0f) {
		 /* for large thrust vector rotations use another rotation method:
		  * calculate angle and axis for R -> R_sp rotation directly 
		  * ¶ÔÓÚ´óµÄÍÆÁ¦ÏòÁ¿Ê¹ÓÃÆäËûĞı×ª·½·¨:¼ÆËã½Ç¶ÈÒÔ¼°ÖáÖ±½Ó×÷ÓÃÓÚR->R_spµÄĞı×ª
		  */
		 /*
     	  * ÓÉDCM»ñÈ¡ËÄÔªÊı£»È»ºó°ÑËÄÔªÊıµÄĞé²¿È¡³ö¸³Öµ¸øe_R_d(e_R_d = q.imag());
    	  * È»ºó¶ÔÆä½øĞĞ¹éÒ»»¯´¦Àí£»×îºó2ĞĞÊÇÏÈÇó³ö»¥²¹ÏµÊı£¬ÔÙÍ¨¹ı»¥²¹·½Ê½ÇóÈ¡e_R¡£
		  */
		math::Quaternion q;
		q.from_dcm(R.transposed() * R_sp);  // ÓÉDCM»ñÈ¡ËÄÔªÊı
		math::Vector<3> e_R_d = q.imag(); //È¡³öĞé²¿
		e_R_d.normalize(); // ¹éÒ»»¯
		e_R_d *= 2.0f * atan2f(e_R_d.length(), q(0)); //µÃµ½Ò»¸öÎó²î½Ç¶ÈÏòÁ¿

		/* use fusion of Z axis based rotation and direct rotation */
		// Ê¹ÓÃZÖáÈÚºÏ»ù±¾Ğı×ªÒÔ¼°Ö±½ÓĞı×ª
		float direct_w = e_R_z_cos * e_R_z_cos * yaw_w;  //Çó³ö»¥²¹ÏµÊı
		// ¸üĞÂe_R,°üº¬Á½ÖÖĞı×ª·½·¨£¬»¥²¹£»
		e_R = e_R * (1.0f - direct_w) + e_R_d * direct_w;  
	}        

	/* calculate angular rates setpoint */
	// ¼ÆËã½ÇËÙ¶ÈÉè¶¨Öµ
	// Çó³ö½ÇËÙ¶ÈµÄÆÚÍûÖµ£¬¹©ÄÚ»·½ÇËÙ¶È¿ØÖÆÊ¹ÓÃ
	_rates_sp = _params.att_p.emult(e_R);
	//emult: Á½ÏòÁ¿ÔªËØ¶ÔÓ¦Ïà³Ëdata[i] = data[i] * v.data[i]

	/* limit rates ½ÇËÙ¶ÈÏŞÖÆ*/
	for (int i = 0; i < 3; i++) {
		_rates_sp(i) = math::constrain(_rates_sp(i), -_params.mc_rate_max(i), _params.mc_rate_max(i));
	}

	/* feed forward yaw setpoint rate */
	// Æ«º½ËÙ¶ÈÉè¶¨ÖµµÄÇ°À¡
	// yawÏìÓ¦½ÏÂı£¬Òò´ËÔÙ¼ÓÈëÒ»¸öÇ°À¡¿ØÖÆ
	_rates_sp(2) += _v_att_sp.yaw_sp_move_rate * yaw_w * _params.yaw_ff;
}
   /*
    *ÉÏÃæÕâ²¿·Ö´úÂë¾Í¾­¹ıÒ»ÏµÁĞµÄËã·¨´¦Àí¹ıÒÔºó»ñÈ¡µÃµ½Ä¿±êÄÚ»·
	* ½ÇËÙ¶ÈÖµroll-pitch-yaw¡¢ÓÍÃÅÁ¿ºÍÊ±¼ä´Á¡£
	* ²¢Í¨¹ı uORB Ä£ĞÍ·¢²¼³öÈ¥   publish attitude rates setpoint
    */

/*
 * Attitude rates controller.
 * Input: '_rates_sp' vector, '_thrust_sp'
 * Output: '_att_control' vector
 */
/*
 * ½ÇËÙ¶È¿ØÖÆÆ÷
 * ÊäÈë:'½ÇËÙ¶ÈÉè¶¨Öµ rate_sp'ÏòÁ¿,'ÍÆÁ¦Éè¶¨Öµthrust_sp'
 * Êä³ö:'×ËÌ¬¿ØÖÆÏòÁ¿ att_control'ÏòÁ¿---Å·À­½Ç
 */
void
MulticopterAttitudeControl::control_attitude_rates(float dt)
{
	/* reset integral if disarmed */
	// ÈôÎ´½âËøÔò¸´Î»»ı·Ö
	if (!_armed.armed || !_vehicle_status.is_rotary_wing) {
		_rates_int.zero();
	}

	/* current body angular rates */
	// µ±Ç°»úÉí½ÇËÙ¶È
	math::Vector<3> rates;
	rates(0) = _ctrl_state.roll_rate;
	rates(1) = _ctrl_state.pitch_rate;
	rates(2) = _ctrl_state.yaw_rate;

	/* angular rates error */
	// ½ÇËÙ¶ÈÎó²î 
	math::Vector<3> rates_err = _rates_sp - rates;
	_att_control = _params.rate_p.emult(rates_err) + _params.rate_d.emult(_rates_prev - rates) / dt + _rates_int +
		       _params.rate_ff.emult(_rates_sp - _rates_sp_prev) / dt;  // Î¢·Ö
	_rates_sp_prev = _rates_sp;
	_rates_prev = rates;

	/* update integral only if not saturated on low limit and if motor commands are not saturated */
	// ½öµ±µÍÏŞÖÆ²»±¥ºÍÒÔ¼°µç»úÃüÁî²»±¥ºÍÊ±¸üĞÂ»ı·Ö???
	if (_thrust_sp > MIN_TAKEOFF_THRUST && !_motor_limits.lower_limit && !_motor_limits.upper_limit) {
		for (int i = 0; i < 3; i++) {
			if (fabsf(_att_control(i)) < _thrust_sp) {
				float rate_i = _rates_int(i) + _params.rate_i(i) * rates_err(i) * dt;

				if (PX4_ISFINITE(rate_i) && rate_i > -RATES_I_LIMIT && rate_i < RATES_I_LIMIT &&
				    _att_control(i) > -RATES_I_LIMIT && _att_control(i) < RATES_I_LIMIT) {
					_rates_int(i) = rate_i;
				}
			}
		}
	}
}
   /*
    * attitude_control ÊäÈëÊÇÌåÖá¾ØÕó R ºÍÆÚÍûµÄÌåÖá¾ØÕó Rsp£¬½Ç¶È»·Ö»ÊÇÒ»¸ö P ¿Ø
	* ÖÆ£¬Ëã³öÀ´Ö®ºóÊä³öµÄÊÇÆÚÍûµÄ½ÇËÙ¶ÈÖµ rate_sp£¨ÕâÒ»¶ÎÒÑ¾­Íê³ÉÁËËùĞèÒªµÄ½Ç¶È±ä
	* »¯£¬²¢½«½Ç¶ÈµÄ±ä»¯Öµ×ª»»µ½ÁËĞèÒªµÄ½ÇËÙ¶ÈÖµ£©¡£²¢ÇÒ°Ñ¼ÓËÙ¶ÈÖµÖ±½ÓÊä³ö¸ø 
	* attitude rate control£¬ÔÙ¾­¹ı½ÇËÙ¶È»·µÄ pid ¿ØÖÆ£¬Êä³öÖµÖ±½Ó¾Í¸ø mixer£¬È»ºó¿ØÖÆµç»úÊä³öÁË
	*
	* ÆäÊµattitude control Êä³öÊÇĞèÒª´ïµ½Õâ¸öÎó²î½Ç¶ÈÊ±ËùĞèÒªµÄ½ÇËÙ¶ÈÖµ£¬ÓÃÕâ¸öÖµÓëµ±Ç°µÄ½Ç
	* ËÙ¶ÈÖµ×ö²î£¬Çó³öÏÖÔÚĞèÒªµÄ½ÇËÙ¶ÈÖµ¶øÒÑ¡£Õâ¸ö¾ÍÊÇÎªÊ²Ã´¿ØÖÆ½ÇËÙ¶ÈµÄÔ­Òò£¬½ø¶ø´ï
	* µ½¿ØÖÆ½Ç¶ÈµÄĞ§¹û¡£
    */

void
MulticopterAttitudeControl::task_main_trampoline(int argc, char *argv[])
{
	mc_att_control::g_control->task_main();
}

void
MulticopterAttitudeControl::task_main()
{

	/*
	 * do subscriptions
	 * ¶©ÔÄ
	 * 
	 * ×¢Òâ¸ÃËã·¨´¦Àí¹ı³ÌÖĞµÄÓĞĞ§Êı¾İµÄÓÃÍ¾ÎÊÌâ£¬×îºó´¦Àí¹ıµÄÊı¾İ×îºóÓÖ±»¸Ä½ø³Ì×Ô¼º¶©ÔÄÁË£¬ 
	 * È»ºóÔÙ´¦Àí£¬ÔÙ¶©ÔÄ£¬Ò»Ö±´¦ÓÚÑ­»·×´Ì¬£¬Õâ¾ÍÊÇËùÎ½µÄPID·´À¡¿ØÖÆÆ÷°É£¡£
	 * ×îÖÕ´ïµ½ËùĞèÇóµÄ¿ØÖÆĞ§¹û£¬´ïµ½¿ØÖÆĞ§¹ûÒÔºó¾Í°ÑÒ»ÏµÁĞµÄ¿ØÖÆÁ¿ÖÃ0£¨ÀàËÆÓÚidle£©£¬
	 * ¸ÃÈÎÎñÒ»Ö±ÔÚÔËĞĞ£¬ËæÆô¶¯½Å±¾Æô¶¯µÄ¡£
	 */
	_v_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_v_rates_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
	_ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
	_v_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_motor_limits_sub = orb_subscribe(ORB_ID(multirotor_motor_limits));

	/* initialize parameters cache ³õÊ¼»¯²ÎÊı»º´æ */
	parameters_update(); // parametersÖ÷Òª¾ÍÊÇÎÒÃÇÇ°ÆÚ¶¨ÒåµÄ¸ĞĞËÈ¤µÄÊı¾İ
						 //ÔÚ×ËÌ¬¿ØÖÆÖĞµÄÕâĞ©Êı¾İ¶¼ÊÇË½ÓĞÊı¾İ£¨private£©

    /* 
     * ¾­¹ıÉÏÊö·ÖÎö£¬¸Ãparameters_update()º¯ÊıÖ÷Òª¾ÍÊÇ»ñÈ¡roll¡¢pitch¡¢yawµÄPID²ÎÊıµÄ¡£ 
     * ²¢¶ÔÈıÖÖ·ÉĞĞÄ£Ê½£¨stablize¡¢auto¡¢acro£©ÏÂµÄ×î´ó×ËÌ¬ËÙ¶È×öÁËÏŞÖÆ¡£
     */

    // NuttXÈÎÎñÊ¹ÄÜ						 
	/* wakeup source: vehicle attitude »½ĞÑÔ´:·ÉĞĞÆ÷×ËÌ¬*/
	px4_pollfd_struct_t fds[1];

	fds[0].fd = _ctrl_state_sub;
	fds[0].events = POLLIN;
	/*
	 * ×¢ÒâÉÏÃæµÄfdµÄ¸³Öµ¡£
	 * * * *Ëæºó½øÈëÈÎÎñµÄÑ­»·º¯Êı£º while (!_task_should_exit){ }¡£ * * * *
	 * ¶¼ÊÇÒ»ÑùµÄÄ£Ê½£¬ÔÚ×ËÌ¬½âËãÊ±Ò²ÊÇÊ¹ÓÃµÄ¸ÃÖÖ·½Ê½¡£
	 */


	while (!_task_should_exit) {

		/* wait for up to 100ms for data ×î¶àµÈ100ms´ıÊı¾İ*/
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit 
		³¬Ê±--ÖÜÆÚĞÔ¼ì²â_task_should_exitÊÇ·ñÍË³ö */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status
		Õâ²»ÊÇÆÚÍûµÄ½á¹ûµ«ÊÇÎÒÃÇÒ²ÎŞÄÜÎªÁ¦: ¿ÉÄÜÒª±ê¼Ç²»ºÃµÄÊı¾İ */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			usleep(100000);
			continue;
		}

		perf_begin(_loop_perf); // ´ø perf¿ªÍ·µÄ¶¼ÊÇ¿Õº¯Êı
								 // ËüµÄ×÷ÓÃÖ÷ÒªÊÇ ¡°Empty function calls for ros compatibility ¡±
		
		/* run controller on attitude changes  ÔÚ×ËÌ¬¸Ä±äÉÏÔËĞĞ¿ØÖÆÆ÷*/
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

			/* copy attitude and control state topics */
			// »ñÈ¡µ±Ç°×ËÌ¬Êı¾İ
			orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);

			/* check for updates in other topics 
				¼ì²âÊı¾İÊÇ·ñÒÑ¾­¸üĞÂ*/
			parameter_update_poll();
			vehicle_control_mode_poll(); //×¢ÒâÕâ¸ö£¬ºóÃæ»áÓÃµ½ÄÚ²¿µÄÊı¾İ´¦Àí½á¹û£¬¼´·¢²¼ºÍ¶©ÔÄµÄIDÎÊÌâ¡£
			arming_status_poll();
			vehicle_manual_poll();
			vehicle_status_poll();
			vehicle_motor_limits_poll();

			/*¹ÙÍøRATTITUDE×ËÌ¬½éÉÜ*/

			/* Check if we are in rattitude mode and the pilot is above the threshold on pitch
			 * or roll (yaw can rotate 360 in normal att control).  If both are true don't
			 * even bother running the attitude controllers 
			 * ¼ì²éÎÒÃÇÊÇ·ñ´¦ÓÚrAttitudeÄ£Ê½ÒÔ¼°·ÉĞĞÔ±(¸©Ñö»òÕßÆ«º½(Æ«º½ÔÚÒ»°ã×ËÌ¬¿ØÖÆÏÂ¿ÉÒÔ360¶È
			 * Ğı×ª))ÊÇ·ñÔÚÓÍÃÅãĞÖµÖ®ÉÏ¡£
			 * Èç¹ûÁ½Õß¶¼ÎªÕæ£¬ÄÇÃ´²»ÓÃÔËĞĞ×ËÌ¬¿ØÖÆÆ÷*/
			if (_vehicle_status.main_state == vehicle_status_s::MAIN_STATE_RATTITUDE) {
				if (fabsf(_manual_control_sp.y) > _params.rattitude_thres ||
				    fabsf(_manual_control_sp.x) > _params.rattitude_thres) {
					_v_control_mode.flag_control_attitude_enabled = false;
				}  //x¡¢yãĞÖµµÄ¼ì²â ¼ì²â¹ö×ª ¸©Ñö
			}

			if (_v_control_mode.flag_control_attitude_enabled) {

				if (_ts_opt_recovery == nullptr) {
					// the  tailsitter recovery instance has not been created, thus, the vehicle
					// is not a tailsitter, do normal attitude control
					// Èô´¹Ö±Æğ½µ»Ö¸´ÊµÀı»¹Ã»ÓĞ±»´´½¨£¬ÄÇÃ´·ÉĞĞÆ÷¾Í²»ÊÇtailsitter
					//½øĞĞÒ»°ã×ËÌ¬¿ØÖÆ

				   /*
					*Çë
					*×¢
					*Òâ
					*¿´
					*Õâ
					*Àï
					*/
					control_attitude(dt);  // ½Ç¶È¿ØÖÆËã·¨			
				} else {
					vehicle_attitude_setpoint_poll();
					_thrust_sp = _v_att_sp.thrust;
					math::Quaternion q(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
					math::Quaternion q_sp(&_v_att_sp.q_d[0]);
					_ts_opt_recovery->setAttGains(_params.att_p, _params.yaw_ff);
					_ts_opt_recovery->calcOptimalRates(q, q_sp, _v_att_sp.yaw_sp_move_rate, _rates_sp);

					/* limit rates */
					for (int i = 0; i < 3; i++) {
						_rates_sp(i) = math::constrain(_rates_sp(i), -_params.mc_rate_max(i), _params.mc_rate_max(i));
					}
				}

				/* publish attitude rates setpoint
				   ·¢²¼control_attitude(dt)½Ç¶È¿ØÖÆËã·¨µÃµ½µÄ½ÇËÙ¶ÈÉè¶¨Öµ*/
				_v_rates_sp.roll = _rates_sp(0);
				_v_rates_sp.pitch = _rates_sp(1);
				_v_rates_sp.yaw = _rates_sp(2);
				_v_rates_sp.thrust = _thrust_sp;
				_v_rates_sp.timestamp = hrt_absolute_time();

				if (_v_rates_sp_pub != nullptr) {
					orb_publish(_rates_sp_id, _v_rates_sp_pub, &_v_rates_sp);

				} else if (_rates_sp_id) {
					_v_rates_sp_pub = orb_advertise(_rates_sp_id, &_v_rates_sp);
				}

				//}

			} else {
				/* attitude controller disabled, poll rates setpoint topic */
				// ½ûÓÃ×ËÌ¬¿ØÖÆÆ÷£¬¼à²âËÙ¶ÈÉè¶¨Öµ»°Ìâ
				if (_v_control_mode.flag_control_manual_enabled) {
					/* manual rates control - ACRO mode */
					// ÊÖ¶¯ËÙ¶È¿ØÖÆ-ÌØ¼¼Ä£Ê½
					_rates_sp = math::Vector<3>(_manual_control_sp.y, -_manual_control_sp.x,
								    _manual_control_sp.r).emult(_params.acro_rate_max);
					_thrust_sp = math::min(_manual_control_sp.z, MANUAL_THROTTLE_MAX_MULTICOPTER);

					/* publish attitude rates setpoint */
					_v_rates_sp.roll = _rates_sp(0);
					_v_rates_sp.pitch = _rates_sp(1);
					_v_rates_sp.yaw = _rates_sp(2);
					_v_rates_sp.thrust = _thrust_sp;
					_v_rates_sp.timestamp = hrt_absolute_time();

					if (_v_rates_sp_pub != nullptr) {
						orb_publish(_rates_sp_id, _v_rates_sp_pub, &_v_rates_sp);

					} else if (_rates_sp_id) {
						_v_rates_sp_pub = orb_advertise(_rates_sp_id, &_v_rates_sp);
					}

				} else {
					/* attitude controller disabled, poll rates setpoint topic */
					// ½ûÓÃ×ËÌ¬¿ØÖÆÆ÷£¬¼à²âËÙ¶ÈÉè¶¨Öµ»°Ìâ
					vehicle_rates_setpoint_poll();
					_rates_sp(0) = _v_rates_sp.roll;
					_rates_sp(1) = _v_rates_sp.pitch;
					_rates_sp(2) = _v_rates_sp.yaw;
					_thrust_sp = _v_rates_sp.thrust;
				}
			}

			if (_v_control_mode.flag_control_rates_enabled) {
					/*
					*Çë
					*×¢
					*Òâ
					*¿´
					*Õâ
					*Àï
					*/
				control_attitude_rates(dt); //½ÇËÙ¶È¿ØÖÆËã·¨

				/* publish actuator controls */
				_actuators.control[0] = (PX4_ISFINITE(_att_control(0))) ? _att_control(0) : 0.0f;
				_actuators.control[1] = (PX4_ISFINITE(_att_control(1))) ? _att_control(1) : 0.0f;
				_actuators.control[2] = (PX4_ISFINITE(_att_control(2))) ? _att_control(2) : 0.0f;
				_actuators.control[3] = (PX4_ISFINITE(_thrust_sp)) ? _thrust_sp : 0.0f;
				_actuators.timestamp = hrt_absolute_time();
				_actuators.timestamp_sample = _ctrl_state.timestamp;

				_controller_status.roll_rate_integ = _rates_int(0);
				_controller_status.pitch_rate_integ = _rates_int(1);
				_controller_status.yaw_rate_integ = _rates_int(2);
				_controller_status.timestamp = hrt_absolute_time();

				if (!_actuators_0_circuit_breaker_enabled) {
					if (_actuators_0_pub != nullptr) {
						orb_publish(_actuators_id, _actuators_0_pub, &_actuators);
						perf_end(_controller_latency_perf);

					} else if (_actuators_id) {
						_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
					}

				}

				/* publish controller status */
				// ¹«²¼¿ØÖÆÆ÷×´Ì¬
				if (_controller_status_pub != nullptr) {
					orb_publish(ORB_ID(mc_att_ctrl_status), _controller_status_pub, &_controller_status);

				} else {
					_controller_status_pub = orb_advertise(ORB_ID(mc_att_ctrl_status), &_controller_status);
				}
			}
		}

		perf_end(_loop_perf);
	}

	_control_task = -1;
	return;
}

int
MulticopterAttitudeControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("mc_att_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1500,
					   (px4_main_t)&MulticopterAttitudeControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int mc_att_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: mc_att_control {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (mc_att_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		mc_att_control::g_control = new MulticopterAttitudeControl;

		if (mc_att_control::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != mc_att_control::g_control->start()) {
			delete mc_att_control::g_control;
			mc_att_control::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (mc_att_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete mc_att_control::g_control;
		mc_att_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (mc_att_control::g_control) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}
