/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * 
 * This module is a modification of the fixed wing module and it is designed for ground rovers.
 * It has been developed starting from the fw module, simplified and improved with dedicated items.
 * 
 * All the ackowledgments and credits for the fw wing app are reported in those files.  
 * 
 * @author Marco Zorzi <mzorzi@student.ethz.ch>
 */


#include <px4_config.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <px4_tasks.h>

#include <cfloat>


#include <arch/board/board.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_hrt.h>
#include <ecl/l1/ecl_l1_pos_controller.h>
#include <external_lgpl/tecs/tecs.h>
#include <geo/geo.h>
#include <launchdetection/LaunchDetector.h>
#include <mathlib/mathlib.h>
#include <systemlib/perf_counter.h>
 #include <systemlib/pid/pid.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/fw_pos_ctrl_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/tecs_status.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/uORB.h>
#include <vtol_att_control/vtol_type.h>

static int	_control_task = -1;			/**< task handle for sensor task */

using matrix::Eulerf;
using matrix::Quatf;

/**
 * L1 control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int gnd_pos_control_main(int argc, char *argv[]);

using namespace launchdetection;

class GroundRoverPositionControl
{
public:
	/**
	 * Constructor
	 */
	GroundRoverPositionControl();

	/**
	 * Destructor, also kills the sensors task.
	 */
	~GroundRoverPositionControl();

	// prevent copying
	GroundRoverPositionControl(const GroundRoverPositionControl &) = delete;
	GroundRoverPositionControl operator=(const GroundRoverPositionControl &other) = delete;

	/**
	 * Start the sensors task.
	 *
	 * @return	OK on success.
	 */
	static int	start();

	/**
	 * Task status
	 *
	 * @return	true if the mainloop is running
	 */
	bool		task_running() { return _task_running; }

private:
	orb_advert_t	_mavlink_log_pub;

	bool		_task_should_exit;		/**< if true, sensor task should exit */
	bool		_task_running;			/**< if true, task is running in its mainloop */

	int		_global_pos_sub;
	int		_pos_sp_triplet_sub;
	int		_ctrl_state_sub;			/**< control state subscription */
	int		_control_mode_sub;		/**< control mode subscription */
	int		_vehicle_command_sub;		/**< vehicle command subscription */
	int		_vehicle_status_sub;		/**< vehicle status subscription */
	int		_vehicle_land_detected_sub;	/**< vehicle land detected subscription */
	int		_params_sub;			/**< notification of parameter updates */
	int		_manual_control_sub;		/**< notification of manual control updates */

	orb_advert_t	_attitude_sp_pub;		/**< attitude setpoint */
	orb_advert_t	_tecs_status_pub;		/**< TECS status publication */
	orb_advert_t	_gnd_pos_ctrl_status_pub;		/**< navigation capabilities publication */

	orb_id_t _attitude_setpoint_id;

	struct control_state_s				_ctrl_state;			/**< control state */
	struct vehicle_attitude_setpoint_s		_att_sp;			/**< vehicle attitude setpoint */
	struct fw_pos_ctrl_status_s			_gnd_pos_ctrl_status;		/**< navigation capabilities */
	struct manual_control_setpoint_s		_manual;			/**< r/c channel data */
	struct vehicle_control_mode_s			_control_mode;			/**< control mode */
	struct vehicle_command_s			_vehicle_command;		/**< vehicle commands */
	struct vehicle_status_s				_vehicle_status;		/**< vehicle status */
	struct vehicle_land_detected_s			_vehicle_land_detected;		/**< vehicle land detected */
	struct vehicle_global_position_s		_global_pos;			/**< global vehicle position */
	struct position_setpoint_triplet_s		_pos_sp_triplet;		/**< triplet of mission items */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	float	_hold_alt;				/**< hold altitude for altitude mode */
	float	_hdg_hold_yaw;				/**< hold heading for velocity mode */
	bool	_hdg_hold_enabled;			/**< heading hold enabled */
	bool	_yaw_lock_engaged;			/**< yaw is locked for heading hold */
	float	_althold_epv;				/**< the position estimate accuracy when engaging alt hold */
	struct position_setpoint_s _hdg_hold_prev_wp;	/**< position where heading hold started */
	struct position_setpoint_s _hdg_hold_curr_wp;	/**< position to which heading hold flies */
	hrt_abstime _control_position_last_called; 	/**<last call of control_position  */

	/* Takeoff launch detection and runway */
	LaunchDetectionResult _launch_detection_state;

	/* Pid controller for the speed. Here we assume we can control airspeed but the control variable is actually on
	 the throttle. For now just assuming a proportional scaler between controlled airspeed and throttle output.*/
	PID_t _speed_ctrl;

	/* throttle and airspeed states */
	float _speed_error;				///< airspeed error to setpoint in m/s
	bool _airspeed_valid;				///< flag if a valid airspeed estimate exists
	uint64_t _airspeed_last_received;		///< last time airspeed was received. Used to detect timeouts.
		
	bool _global_pos_valid;				///< global position is valid
	math::Matrix<3, 3> _R_nb;			///< current attitude
	float _roll;
	float _pitch;
	float _yaw;
	bool _reinitialize_tecs;			///< indicates if the TECS states should be reinitialized (used for VTOL)
	bool _is_tecs_running;
	hrt_abstime _last_tecs_update;
	float _asp_after_transition;
	bool _was_in_transition;

	// estimator reset counters
	uint8_t _pos_reset_counter;		// captures the number of times the estimator has reset the horizontal position
	uint8_t _alt_reset_counter;		// captures the number of times the estimator has reset the altitude state

	ECL_L1_Pos_Controller				_gnd_control;
	TECS						_tecs;
	enum UGV_POSCTRL_MODE {
		UGV_POSCTRL_MODE_AUTO,
		UGV_POSCTRL_MODE_POSITION,
		UGV_POSCTRL_MODE_ALTITUDE,
		UGV_POSCTRL_MODE_OTHER
	} _control_mode_current;			///< used to check the mode in the last control loop iteration. Use to check if the last iteration was in the same mode.

	struct {
		float l1_period;
		float l1_damping;
		float l1_distance;

		float time_const;
		float time_const_throt;
		float min_sink_rate;
		float max_sink_rate;
		float max_climb_rate;
		float heightrate_p;
		float heightrate_ff;
		float speedrate_p;
		float throttle_damp;
		float integrator_gain;
		float vertical_accel_limit;
		float height_comp_filter_omega;
		float speed_comp_filter_omega;
		float roll_throttle_compensation;
		float speed_weight;
		float pitch_damping;

		float airspeed_min;
		float airspeed_trim;
		float airspeed_max;
		int airspeed_mode;
		int speed_control_mode;
		float speed_p;
		float speed_i;
		float speed_d;
		float speed_imax;
		float speed_throttle_airspeed_scaler;

		float pitch_limit_min;
		float pitch_limit_max;
		float roll_limit;
		float throttle_min;
		float throttle_max;
		float throttle_cruise;
		float throttle_slew_max;
		float man_roll_max_rad;
		float man_pitch_max_rad;


	} _parameters;			/**< local copies of interesting parameters */

	struct {

		param_t l1_period;
		param_t l1_damping;
		param_t l1_distance;

		param_t time_const;
		param_t time_const_throt;
		param_t min_sink_rate;
		param_t max_sink_rate;
		param_t max_climb_rate;
		param_t heightrate_p;
		param_t heightrate_ff;
		param_t speedrate_p;
		param_t throttle_damp;
		param_t integrator_gain;
		param_t vertical_accel_limit;
		param_t height_comp_filter_omega;
		param_t speed_comp_filter_omega;
		param_t roll_throttle_compensation;
		param_t speed_weight;
		param_t pitch_damping;

		param_t airspeed_min;
		param_t airspeed_trim;
		param_t airspeed_max;
		param_t airspeed_mode;

		param_t speed_control_mode;
		param_t speed_p;
		param_t speed_i;
		param_t speed_d;
		param_t speed_imax;
		param_t speed_throttle_airspeed_scaler;

		param_t pitch_limit_min;
		param_t pitch_limit_max;
		param_t roll_limit;
		param_t throttle_min;
		param_t throttle_max;
		param_t throttle_cruise;
		param_t throttle_slew_max;
		param_t man_roll_max_deg;
		param_t man_pitch_max_deg;


	} _parameter_handles;		/**< handles for interesting parameters */


	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	/**
	 * Update control outputs
	 *
	 */
	void		control_update();

	/**
	 * Check for changes in control mode
	 */
	void		vehicle_control_mode_poll();

	/**
	 * Check for new in vehicle commands
	 */
	void		vehicle_command_poll();

	/**
	 * Check for changes in vehicle status.
	 */
	void		vehicle_status_poll();

	/**
	 * Check for changes in vehicle land detected.
	 */
	void		vehicle_land_detected_poll();

	/**
	 * Check for manual setpoint updates.
	 */
	bool		vehicle_manual_control_setpoint_poll();

	/**
	 * Check for changes in control state.
	 */
	void		control_state_poll();

	/**
	 * Check for set triplet updates.
	 */
	void		vehicle_setpoint_poll();

	/**
	 * Publish navigation capabilities
	 */
	void		gnd_pos_ctrl_status_publish();

	/**
	 * Control position.
	 */
	bool		control_position(const math::Vector<2> &global_pos, const math::Vector<3> &ground_speed,
					 const struct position_setpoint_triplet_s &_pos_sp_triplet);

	float		get_tecs_pitch();
	float		get_tecs_thrust();

	float		calculate_target_speed(float airspeed_demand);

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main sensor collection task.
	 */
	void		task_main();

	/*
	 * Call TECS : a wrapper function to call the TECS implementation
	 */
	void tecs_update_throttle(float alt_sp, float v_sp, float eas2tas,
					float pitch_min_rad, float pitch_max_rad,
					float throttle_min, float throttle_max, float throttle_cruise,
					float climbout_pitch_min_rad,
					float altitude,
					const math::Vector<3> &ground_speed,
					unsigned mode = tecs_status_s::TECS_MODE_NORMAL);



};

namespace gnd_control
{

GroundRoverPositionControl	*g_control = nullptr;
}

GroundRoverPositionControl::GroundRoverPositionControl() :

	_mavlink_log_pub(nullptr),
	_task_should_exit(false),
	_task_running(false),

	/* subscriptions */
	_global_pos_sub(-1),
	_pos_sp_triplet_sub(-1),
	_ctrl_state_sub(-1),
	_control_mode_sub(-1),
	_vehicle_command_sub(-1),
	_vehicle_status_sub(-1),
	_vehicle_land_detected_sub(-1),
	_params_sub(-1),
	_manual_control_sub(-1),

	/* publications */
	_attitude_sp_pub(nullptr),
	_tecs_status_pub(nullptr),
	_gnd_pos_ctrl_status_pub(nullptr),

	/* publication ID */
	_attitude_setpoint_id(nullptr),

	/* states */
	_ctrl_state(),
	_att_sp(),
	_gnd_pos_ctrl_status(),
	_manual(),
	_control_mode(),
	_vehicle_command(),
	_vehicle_status(),
	_vehicle_land_detected(),
	_global_pos(),
	_pos_sp_triplet(),

	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "fw l1 control")),

	_hold_alt(0.0f),
	_hdg_hold_yaw(0.0f),
	_hdg_hold_enabled(false),
	_yaw_lock_engaged(false),
	_althold_epv(0.0f),
	_hdg_hold_prev_wp{},
	_hdg_hold_curr_wp{},
	_control_position_last_called(0),

	_launch_detection_state(LAUNCHDETECTION_RES_NONE),
	_speed_ctrl(),
	_speed_error(0.0f),
	_airspeed_valid(false),
	_airspeed_last_received(0),

	_global_pos_valid(false),
	_R_nb(),
	_roll(0.0f),
	_pitch(0.0f),
	_yaw(0.0f),
	_reinitialize_tecs(true),
	_is_tecs_running(false),
	_last_tecs_update(0.0f),
	_asp_after_transition(0.0f),
	_was_in_transition(false),
	_pos_reset_counter(0),
	_alt_reset_counter(0),
	_gnd_control(),
	_tecs(),
	_control_mode_current(UGV_POSCTRL_MODE_OTHER),
	_parameters(),
	_parameter_handles()
{
	_gnd_pos_ctrl_status = {};

	_parameter_handles.l1_period = param_find("GND_L1_PERIOD");
	_parameter_handles.l1_damping = param_find("GND_L1_DAMPING");
	_parameter_handles.l1_distance = param_find("GND_L1_DIST");


	_parameter_handles.airspeed_min = param_find("FW_AIRSPD_MIN");
	_parameter_handles.airspeed_trim = param_find("FW_AIRSPD_TRIM");
	_parameter_handles.airspeed_max = param_find("FW_AIRSPD_MAX");
	_parameter_handles.airspeed_mode = param_find("FW_ARSP_MODE");

	_parameter_handles.speed_control_mode = param_find("GND_SP_CTRL_MODE");
	_parameter_handles.speed_p = param_find("GND_SPEED_P");
	_parameter_handles.speed_i = param_find("GND_SPEED_I");
	_parameter_handles.speed_d = param_find("GND_SPEED_D");
	_parameter_handles.speed_imax = param_find("GND_SPEED_IMAX");
	_parameter_handles.speed_throttle_airspeed_scaler = param_find("GND_AIRSP_THR_SC");

	_parameter_handles.pitch_limit_min = param_find("GND_P_LIM_MIN");
	_parameter_handles.pitch_limit_max = param_find("GND_P_LIM_MAX");
	_parameter_handles.roll_limit = param_find("GND_R_LIM");
	_parameter_handles.throttle_min = param_find("GND_THR_MIN");
	_parameter_handles.throttle_max = param_find("GND_THR_MAX");
	_parameter_handles.throttle_slew_max = param_find("GND_THR_SLEW_MAX");
	_parameter_handles.throttle_cruise = param_find("GND_THR_CRUISE");
	_parameter_handles.man_roll_max_deg = param_find("FW_MAN_R_MAX");
	_parameter_handles.man_pitch_max_deg = param_find("FW_MAN_P_MAX");

	_parameter_handles.time_const = 			param_find("FW_T_TIME_CONST");
	_parameter_handles.time_const_throt = 			param_find("FW_T_THRO_CONST");
	_parameter_handles.min_sink_rate = 			param_find("FW_T_SINK_MIN");
	_parameter_handles.max_sink_rate =			param_find("FW_T_SINK_MAX");
	_parameter_handles.max_climb_rate =			param_find("FW_T_CLMB_MAX");
	_parameter_handles.throttle_damp = 			param_find("FW_T_THR_DAMP");
	_parameter_handles.integrator_gain =			param_find("FW_T_INTEG_GAIN");
	_parameter_handles.vertical_accel_limit =		param_find("FW_T_VERT_ACC");
	_parameter_handles.height_comp_filter_omega =		param_find("FW_T_HGT_OMEGA");
	_parameter_handles.speed_comp_filter_omega =		param_find("FW_T_SPD_OMEGA");
	_parameter_handles.roll_throttle_compensation = 	param_find("FW_T_RLL2THR");
	_parameter_handles.speed_weight = 			param_find("FW_T_SPDWEIGHT");
	_parameter_handles.pitch_damping = 			param_find("FW_T_PTCH_DAMP");
	_parameter_handles.heightrate_p =			param_find("FW_T_HRATE_P");
	_parameter_handles.heightrate_ff =			param_find("FW_T_HRATE_FF");
	_parameter_handles.speedrate_p =			param_find("FW_T_SRATE_P");


	/* fetch initial parameter values */
	parameters_update();
}

GroundRoverPositionControl::~GroundRoverPositionControl()
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

	gnd_control::g_control = nullptr;
}

int
GroundRoverPositionControl::parameters_update()
{

	/* L1 control parameters */
	param_get(_parameter_handles.l1_damping, &(_parameters.l1_damping));
	param_get(_parameter_handles.l1_period, &(_parameters.l1_period));
	param_get(_parameter_handles.l1_distance, &(_parameters.l1_distance));

	param_get(_parameter_handles.airspeed_min, &(_parameters.airspeed_min));
	param_get(_parameter_handles.airspeed_trim, &(_parameters.airspeed_trim));
	param_get(_parameter_handles.airspeed_max, &(_parameters.airspeed_max));
	param_get(_parameter_handles.airspeed_mode, &(_parameters.airspeed_mode));

	param_get(_parameter_handles.speed_control_mode, &(_parameters.speed_control_mode));
	param_get(_parameter_handles.speed_p, &(_parameters.speed_p));
	param_get(_parameter_handles.speed_i, &(_parameters.speed_i));
	param_get(_parameter_handles.speed_d, &(_parameters.speed_d));
	param_get(_parameter_handles.speed_imax, &(_parameters.speed_imax));
	param_get(_parameter_handles.speed_throttle_airspeed_scaler, &(_parameters.speed_throttle_airspeed_scaler));


	param_get(_parameter_handles.pitch_limit_min, &(_parameters.pitch_limit_min));
	param_get(_parameter_handles.pitch_limit_max, &(_parameters.pitch_limit_max));
	param_get(_parameter_handles.roll_limit, &(_parameters.roll_limit));
	param_get(_parameter_handles.throttle_min, &(_parameters.throttle_min));
	param_get(_parameter_handles.throttle_max, &(_parameters.throttle_max));
	param_get(_parameter_handles.throttle_cruise, &(_parameters.throttle_cruise));
	param_get(_parameter_handles.throttle_slew_max, &(_parameters.throttle_slew_max));

	param_get(_parameter_handles.man_roll_max_deg, &_parameters.man_roll_max_rad);
	_parameters.man_roll_max_rad = math::radians(_parameters.man_roll_max_rad);
	param_get(_parameter_handles.man_pitch_max_deg, &_parameters.man_pitch_max_rad);
	_parameters.man_pitch_max_rad = math::radians(_parameters.man_pitch_max_rad);

	param_get(_parameter_handles.time_const, &(_parameters.time_const));
	param_get(_parameter_handles.time_const_throt, &(_parameters.time_const_throt));
	param_get(_parameter_handles.min_sink_rate, &(_parameters.min_sink_rate));
	param_get(_parameter_handles.max_sink_rate, &(_parameters.max_sink_rate));
	param_get(_parameter_handles.throttle_damp, &(_parameters.throttle_damp));
	param_get(_parameter_handles.integrator_gain, &(_parameters.integrator_gain));
	param_get(_parameter_handles.vertical_accel_limit, &(_parameters.vertical_accel_limit));
	param_get(_parameter_handles.height_comp_filter_omega, &(_parameters.height_comp_filter_omega));
	param_get(_parameter_handles.speed_comp_filter_omega, &(_parameters.speed_comp_filter_omega));
	param_get(_parameter_handles.roll_throttle_compensation, &(_parameters.roll_throttle_compensation));
	param_get(_parameter_handles.speed_weight, &(_parameters.speed_weight));
	param_get(_parameter_handles.pitch_damping, &(_parameters.pitch_damping));
	param_get(_parameter_handles.max_climb_rate, &(_parameters.max_climb_rate));
	param_get(_parameter_handles.heightrate_p, &(_parameters.heightrate_p));
	param_get(_parameter_handles.heightrate_ff, &(_parameters.heightrate_ff));
	param_get(_parameter_handles.speedrate_p, &(_parameters.speedrate_p));

	_gnd_control.set_l1_damping(_parameters.l1_damping);
	_gnd_control.set_l1_period(_parameters.l1_period);
	_gnd_control.set_l1_roll_limit(math::radians(_parameters.roll_limit));

	_tecs.set_time_const(_parameters.time_const);
	_tecs.set_time_const_throt(_parameters.time_const_throt);
	_tecs.set_min_sink_rate(_parameters.min_sink_rate);
	_tecs.set_max_sink_rate(_parameters.max_sink_rate);
	_tecs.set_throttle_damp(_parameters.throttle_damp);
	_tecs.set_throttle_slewrate(_parameters.throttle_slew_max);
	_tecs.set_integrator_gain(_parameters.integrator_gain);
	_tecs.set_vertical_accel_limit(_parameters.vertical_accel_limit);
	_tecs.set_height_comp_filter_omega(_parameters.height_comp_filter_omega);
	_tecs.set_speed_comp_filter_omega(_parameters.speed_comp_filter_omega);
	_tecs.set_roll_throttle_compensation(_parameters.roll_throttle_compensation);
	_tecs.set_speed_weight(_parameters.speed_weight);
	_tecs.set_pitch_damping(_parameters.pitch_damping);
	_tecs.set_indicated_airspeed_min(_parameters.airspeed_min);
	_tecs.set_indicated_airspeed_max(_parameters.airspeed_max);
	_tecs.set_max_climb_rate(_parameters.max_climb_rate);
	_tecs.set_heightrate_p(_parameters.heightrate_p);
	_tecs.set_heightrate_ff(_parameters.heightrate_ff);
	_tecs.set_speedrate_p(_parameters.speedrate_p);

	pid_init(&_speed_ctrl, PID_MODE_DERIVATIV_CALC, 0.01f);
	pid_set_parameters(&_speed_ctrl, 
				_parameters.speed_p,
				_parameters.speed_d,
				_parameters.speed_i,
				_parameters.speed_imax,
				_parameters.airspeed_max); 

	/* sanity check parameters  */
	if (_parameters.airspeed_max < _parameters.airspeed_min ||
	    _parameters.airspeed_min > 100.0f ||
	    _parameters.airspeed_trim < _parameters.airspeed_min ||
	    _parameters.airspeed_trim > _parameters.airspeed_max) {
		warnx("error: airspeed parameters invalid");
		return 1;
	}
	

	/* Update and publish the navigation capabilities */
	_gnd_pos_ctrl_status.landing_slope_angle_rad = 0;
	_gnd_pos_ctrl_status.landing_horizontal_slope_displacement = 0;
	_gnd_pos_ctrl_status.landing_flare_length = 0;
	gnd_pos_ctrl_status_publish();

	return OK;
}

void
GroundRoverPositionControl::vehicle_control_mode_poll()
{
	bool updated;

	orb_check(_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
	}
}

void
GroundRoverPositionControl::vehicle_command_poll()
{
	bool updated;

	orb_check(_vehicle_command_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_command), _vehicle_command_sub, &_vehicle_command);
	}
}

void
GroundRoverPositionControl::vehicle_status_poll()
{
	bool updated;

	orb_check(_vehicle_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);

		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (!_attitude_setpoint_id) {
			if (_vehicle_status.is_vtol) {
				_attitude_setpoint_id = ORB_ID(fw_virtual_attitude_setpoint);

			} else {
				_attitude_setpoint_id = ORB_ID(vehicle_attitude_setpoint);
			}
		}
	}
}

void
GroundRoverPositionControl::vehicle_land_detected_poll()
{
	bool updated;

	orb_check(_vehicle_land_detected_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_vehicle_land_detected);
	}
}

bool
GroundRoverPositionControl::vehicle_manual_control_setpoint_poll()
{
	bool manual_updated;

	/* Check if manual setpoint has changed */
	orb_check(_manual_control_sub, &manual_updated);

	if (manual_updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sub, &_manual);
	}

	return manual_updated;
}

void
GroundRoverPositionControl::control_state_poll()
{
	/* check if there is a new position */
	bool ctrl_state_updated;
	orb_check(_ctrl_state_sub, &ctrl_state_updated);

	if (ctrl_state_updated) {
		orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);
		_airspeed_valid = _ctrl_state.airspeed_valid;
		_airspeed_last_received = hrt_absolute_time();

	} else {

		/* no airspeed updates for one second */
		if (_airspeed_valid && (hrt_absolute_time() - _airspeed_last_received) > 1e6) {
			_airspeed_valid = false;
		}
	}

	/* set rotation matrix and euler angles */
	math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
	_R_nb = q_att.to_dcm();

	math::Vector<3> euler_angles;
	euler_angles = _R_nb.to_euler();
	_roll    = euler_angles(0);
	_pitch   = euler_angles(1);
	_yaw     = euler_angles(2);

	/* update TECS state */
	_tecs.enable_airspeed(_airspeed_valid);
}

void
GroundRoverPositionControl::vehicle_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool pos_sp_triplet_updated;
	orb_check(_pos_sp_triplet_sub, &pos_sp_triplet_updated);

	if (pos_sp_triplet_updated) {
		orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);
	}
}

float
GroundRoverPositionControl::calculate_target_speed(float speed_demand)
{
	float ground_speed;

	if (_airspeed_valid) {
		ground_speed = _ctrl_state.airspeed;
	} else {
		ground_speed = _parameters.airspeed_min + (_parameters.airspeed_max - _parameters.airspeed_min) / 2.0f;
	}

	/* cruise ground_speed for all modes unless modified below */
	float target_speed = speed_demand;

	/* sanity check: limit to range */
	target_speed = math::constrain(target_speed, _parameters.airspeed_min, _parameters.airspeed_max);

	/* plain ground_speed error */
	_speed_error = target_speed - ground_speed;

	return target_speed;
}


void GroundRoverPositionControl::gnd_pos_ctrl_status_publish()
{
	_gnd_pos_ctrl_status.timestamp = hrt_absolute_time();

	if (_gnd_pos_ctrl_status_pub != nullptr) {
		orb_publish(ORB_ID(fw_pos_ctrl_status), _gnd_pos_ctrl_status_pub, &_gnd_pos_ctrl_status);

	} else {
		_gnd_pos_ctrl_status_pub = orb_advertise(ORB_ID(fw_pos_ctrl_status), &_gnd_pos_ctrl_status);
	}
}


void GroundRoverPositionControl::tecs_update_throttle(float alt_sp, float v_sp, float eas2tas,
		float pitch_min_rad, float pitch_max_rad,
		float throttle_min, float throttle_max, float throttle_cruise,
		float climbout_pitch_min_rad,
		float altitude,
		const math::Vector<3> &ground_speed,
		unsigned mode)
{

	_last_tecs_update = hrt_absolute_time();

	// do not run TECS if we are not in air
	bool run_tecs = !_vehicle_land_detected.landed;

	//If the wrong vehicle type is selected return because we want to use it for rovers only.
	if (_vehicle_status.is_vtol || _vehicle_status.is_rotary_wing) {
		return;
	}

	_is_tecs_running = run_tecs;

	if (!run_tecs) {
		// next time we run TECS we should reinitialize states
		_reinitialize_tecs = true;
		return;
	}

	if (_reinitialize_tecs) {
		_tecs.reset_state();
		_reinitialize_tecs = false;
	}

	_tecs.update_pitch_throttle(_R_nb, 0.0f , altitude, alt_sp, v_sp,
				    _ctrl_state.airspeed, eas2tas,
				    false /*climbout_mode*/, climbout_pitch_min_rad,
				    throttle_min, throttle_max, throttle_cruise,
				    pitch_min_rad, pitch_max_rad);

	struct TECS::tecs_state s;
	_tecs.get_tecs_state(s);

	struct tecs_status_s t = {};

	t.timestamp = s.timestamp;

	switch (s.mode) {
	case TECS::ECL_TECS_MODE_NORMAL:
		t.mode = tecs_status_s::TECS_MODE_NORMAL;
		break;
	default:
		t.mode = tecs_status_s::TECS_MODE_BAD_DESCENT;
		break;
	}

	t.altitudeSp 		= s.altitude_sp;
	t.altitude_filtered = s.altitude_filtered;
	t.airspeedSp 		= s.airspeed_sp;
	t.airspeed_filtered = s.airspeed_filtered;

	t.flightPathAngleSp 		= s.altitude_rate_sp;
	t.flightPathAngle 			= s.altitude_rate;
	t.flightPathAngleFiltered 	= s.altitude_rate;

	t.airspeedDerivativeSp 	= s.airspeed_rate_sp;
	t.airspeedDerivative 	= s.airspeed_rate;

	t.totalEnergyError 				= s.total_energy_error;
	t.totalEnergyRateError 			= s.total_energy_rate_error;
	t.energyDistributionError 		= s.energy_distribution_error;
	t.energyDistributionRateError 	= s.energy_distribution_rate_error;

	t.throttle_integ 	= s.throttle_integ;
	t.pitch_integ 		= s.pitch_integ;

	if (_tecs_status_pub != nullptr) {
		orb_publish(ORB_ID(tecs_status), _tecs_status_pub, &t);

	} else {
		_tecs_status_pub = orb_advertise(ORB_ID(tecs_status), &t);
	}
}

float
GroundRoverPositionControl::get_tecs_pitch()
{
	if (_is_tecs_running) {
		return _tecs.get_pitch_demand();

	} else {
		// return 0 to prevent stale tecs state when it's not running
		return 0.0f;
	}
}

float
GroundRoverPositionControl::get_tecs_thrust()
{
	if (_is_tecs_running) {
		return _tecs.get_throttle_demand();

	} else {
		// return 0 to prevent stale tecs state when it's not running
		return 0.0f;
	}
}

bool
GroundRoverPositionControl::control_position(const math::Vector<2> &current_position, const math::Vector<3> &ground_speed,
		const struct position_setpoint_triplet_s &pos_sp_triplet)
{
	 float dt = 0.01; // Using non zero value to a avoid division by zero
	if (_control_position_last_called > 0) {
	 	dt = (float)hrt_elapsed_time(&_control_position_last_called) * 1e-6f;
	 }

	_control_position_last_called = hrt_absolute_time();

	/* only run position controller if we are in fixed wing configuration */
	//TODO: add a vehicle status for ground based robots, less safety required.
	if (_vehicle_status.is_rotary_wing || _vehicle_status.in_transition_mode || _vehicle_status.is_vtol) {
		_control_mode_current = UGV_POSCTRL_MODE_OTHER;
		return false;
	}

	bool setpoint = true;

	_att_sp.fw_control_yaw = true;		// We want to control yaw to turn the car
	float eas2tas = 1.0f; // XXX calculate actual number based on current measurements

	/* filter speed and altitude for controller */
	math::Vector<3> accel_body(_ctrl_state.x_acc, _ctrl_state.y_acc, _ctrl_state.z_acc);
	math::Vector<3> accel_earth{_R_nb * accel_body};

	/* tell TECS to update its state and update TECS filters */
	_tecs.update_state(_global_pos.alt, _ctrl_state.airspeed, _R_nb,
			   accel_body, accel_earth, (_global_pos.timestamp > 0), true);

	math::Vector<2> ground_speed_2d = {ground_speed(0), ground_speed(1)};

	// compute 2D groundspeed from airspeed-heading projection
	math::Vector<2> air_speed_2d = {_ctrl_state.airspeed * cosf(_yaw), _ctrl_state.airspeed * sinf(_yaw)};
	math::Vector<2> nav_speed_2d = {0, 0};
	
	nav_speed_2d = ground_speed_2d;
	float nav_speed = 0.0f;

	/* no throttle limit as default */
	float throttle_max = 1.0f;

	if (_control_mode.flag_control_auto_enabled && pos_sp_triplet.current.valid) {
		/* AUTONOMOUS FLIGHT */

		/* Reset integrators if switching to this mode from a other mode in which posctl was not active */
		if (_control_mode_current == UGV_POSCTRL_MODE_OTHER) {
			/* reset integrators */
			_tecs.reset_state();
		}

		_control_mode_current = UGV_POSCTRL_MODE_AUTO;

		/* reset hold altitude */
		_hold_alt = _global_pos.alt;
		/* reset hold yaw */
		_hdg_hold_yaw = _yaw;

		/* get circle mode */
		bool was_circle_mode = _gnd_control.circle_mode();

		/* restore speed weight, in case changed intermittently (e.g. in landing handling) */
		_tecs.set_speed_weight(_parameters.speed_weight);

		/* current waypoint (the one currently heading for) */
		math::Vector<2> next_wp((float)pos_sp_triplet.current.lat, (float)pos_sp_triplet.current.lon);

		/* current waypoint (the one currently heading for) */
		math::Vector<2> curr_wp((float)pos_sp_triplet.current.lat, (float)pos_sp_triplet.current.lon);

		/* Initialize attitude controller integrator reset flags to 0 */
		_att_sp.roll_reset_integral = false;
		_att_sp.pitch_reset_integral = false;
		_att_sp.yaw_reset_integral = false;

		/* previous waypoint */
		math::Vector<2> prev_wp;

		if (pos_sp_triplet.previous.valid) {
			prev_wp(0) = (float)pos_sp_triplet.previous.lat;
			prev_wp(1) = (float)pos_sp_triplet.previous.lon;

		} else {
			/*
			 * No valid previous waypoint, go for the current wp.
			 * This is automatically handled by the L1 library.
			 */
			prev_wp(0) = (float)pos_sp_triplet.current.lat;
			prev_wp(1) = (float)pos_sp_triplet.current.lon;

		}

		float mission_target_speed = _parameters.airspeed_trim;
		float mission_throttle = _parameters.throttle_cruise;

			/* Just control the throttle */
		if ( _parameters.speed_control_mode > 0 ) {
			/* control the speed in closed loop */
			if (PX4_ISFINITE(_pos_sp_triplet.current.cruising_speed) &&
		    _pos_sp_triplet.current.cruising_speed > 0.1f) {
				mission_target_speed = _pos_sp_triplet.current.cruising_speed;
			} 

			/* sanity check for target airspeed before using it for computations*/
			if (mission_target_speed < _parameters.airspeed_min ||
				mission_target_speed > _parameters.airspeed_max) {

				/* If something is wrong revert back to the safe trim but warn the user*/ 
				mission_target_speed = _parameters.airspeed_trim;
				warnx("Something went wrong with target mission target speed, reverting to trim for safety");
			}

			nav_speed = sqrtf(powf(nav_speed_2d(0),2) + powf(nav_speed_2d(1),2));
			
			//Compute airspeed control out and just scale it as a constant
			mission_throttle =  _parameters.speed_throttle_airspeed_scaler * pid_calculate(&_speed_ctrl, mission_target_speed, nav_speed, 0.01f, dt);
			//mission_throttle = 0.5f;

			// Constrain throttle between min and max
			mission_throttle = math::constrain(mission_throttle, _parameters.throttle_min, _parameters.throttle_max);
			// warnx("mission_throttle %.4f", (double)mission_throttle);

		} else {
			/* Just control throttle in open loop */
			if (PX4_ISFINITE(_pos_sp_triplet.current.cruising_throttle) &&
			    _pos_sp_triplet.current.cruising_throttle > 0.01f) {

				mission_throttle = _pos_sp_triplet.current.cruising_throttle;
			}
		}

		// at this point we have a target throttle no matter what

		if (pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_IDLE) {
			_att_sp.thrust = 0.0f;
			_att_sp.roll_body = 0.0f;
			_att_sp.pitch_body = 0.0f;

		} else if ((pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_POSITION)
				|| (pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF)) {
			
			 if (_launch_detection_state != LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS) {
			 	_launch_detection_state = LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS;
			 }

			/* waypoint is a plain navigation waypoint or the takeoff waypoint, does not matter */
			_gnd_control.navigate_waypoints(prev_wp, curr_wp, current_position, nav_speed_2d);
			_att_sp.roll_body = _gnd_control.nav_roll();
			_att_sp.yaw_body = _gnd_control.nav_bearing();

			/* Apply control output */
			tecs_update_throttle(pos_sp_triplet.current.alt, calculate_target_speed(mission_target_speed), eas2tas,
						   math::radians(_parameters.pitch_limit_min), math::radians(_parameters.pitch_limit_max),
						   _parameters.throttle_min, _parameters.throttle_max, mission_throttle,
						   math::radians(_parameters.pitch_limit_min), _global_pos.alt, ground_speed);

		} else if (pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LOITER) {

			/* waypoint is a loiter waypoint so we want to stop*/
			_gnd_control.navigate_loiter(curr_wp, current_position, pos_sp_triplet.current.loiter_radius,
						    pos_sp_triplet.current.loiter_direction, nav_speed_2d);
			_att_sp.roll_body = _gnd_control.nav_roll();
			_att_sp.yaw_body = _gnd_control.nav_bearing();

			float alt_sp = pos_sp_triplet.current.alt;

			tecs_update_throttle(alt_sp,
						   calculate_target_speed(mission_target_speed),
						   eas2tas,
						   math::radians(_parameters.pitch_limit_min),
						   math::radians(_parameters.pitch_limit_max),
						   0, // _parameters.throttle_min,
						   0, // _parameters.throttle_max,
						   0, // _parameters.throttle_cruise,
						   math::radians(_parameters.pitch_limit_min),
						   _global_pos.alt,
						   ground_speed);
		}

		if (was_circle_mode && !_gnd_control.circle_mode()) {
			/* just kicked out of loiter, reset integrals */
			_att_sp.yaw_reset_integral = true;
		}

	}else {
		_control_mode_current = UGV_POSCTRL_MODE_OTHER;

		/* do not publish the setpoint */
		setpoint = false;

		// reset hold altitude
		_hold_alt = _global_pos.alt;
	}

	/* Copy thrust output for publication */
	if (_vehicle_status.engine_failure ||
		_vehicle_status.engine_failure_cmd ||
		   ( _control_mode_current == UGV_POSCTRL_MODE_AUTO &&
		   	 pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_IDLE ) ||
		   _control_mode_current == UGV_POSCTRL_MODE_OTHER ) {
		/* Set thrust to 0 to minimize damage */
		_att_sp.thrust = 0.0f;
	} else {
		_att_sp.thrust = math::min(get_tecs_thrust(), throttle_max);
	}

	return setpoint;
}


void
GroundRoverPositionControl::task_main()
{

	/*
	 * do subscriptions
	 */
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	_ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_vehicle_command_sub = orb_subscribe(ORB_ID(vehicle_command));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_control_sub = orb_subscribe(ORB_ID(manual_control_setpoint));

	/* rate limit control mode updates to 5Hz */
	orb_set_interval(_control_mode_sub, 200);
	/* rate limit vehicle status updates to 5Hz */
	orb_set_interval(_vehicle_status_sub, 200);
	/* rate limit vehicle land detected updates to 5Hz */
	orb_set_interval(_vehicle_land_detected_sub, 200);
	/* rate limit position updates to 50 Hz */
	orb_set_interval(_global_pos_sub, 20);

	/* abort on a nonzero return value from the parameter init */
	if (parameters_update()) {
		/* parameter setup went wrong, abort */
		warnx("aborting startup due to errors.");
		_task_should_exit = true;
	}

	/* wakeup source(s) */
	px4_pollfd_struct_t fds[2];

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _global_pos_sub;
	fds[1].events = POLLIN;

	_task_running = true;

	while (!_task_should_exit) {

		//warnx("Looping....");

		/* wait for up to 500ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit, etc. */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		/* check vehicle control mode for changes to publication state */
		vehicle_control_mode_poll();

		/* check for new vehicle commands */
		vehicle_command_poll();

		/* check vehicle status for changes to publication state */
		vehicle_status_poll();

		/* check vehicle land detected for changes to publication state */
		vehicle_land_detected_poll();

		/* only update parameters if they changed */
		if (fds[0].revents & POLLIN) {
			/* read from param to clear updated flag */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			/* update parameters from storage */
			parameters_update();
		}

		/* only run controller if position changed */
		if (fds[1].revents & POLLIN) {
			perf_begin(_loop_perf);

			/* load local copies */
			orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);

			// handle estimator reset events. we only adjust setpoins for manual modes
			if (_control_mode.flag_control_manual_enabled) {
				if (_control_mode.flag_control_altitude_enabled && _global_pos.alt_reset_counter != _alt_reset_counter) {
					_hold_alt += _global_pos.delta_alt;
					// make TECS accept step in altitude and demanded altitude
					_tecs.handle_alt_step(_global_pos.delta_alt, _global_pos.alt);
				}

				// adjust navigation waypoints in position control mode
				if (_control_mode.flag_control_altitude_enabled && _control_mode.flag_control_velocity_enabled
				    && _global_pos.lat_lon_reset_counter != _pos_reset_counter) {

					// reset heading hold flag, which will re-initialise position control
					_hdg_hold_enabled = false;
				}
			}

			// update the reset counters in any case
			_alt_reset_counter = _global_pos.alt_reset_counter;
			_pos_reset_counter = _global_pos.lat_lon_reset_counter;

			// XXX add timestamp check
			_global_pos_valid = true;

			control_state_poll();
			vehicle_setpoint_poll();
			vehicle_manual_control_setpoint_poll();

			math::Vector<3> ground_speed(_global_pos.vel_n, _global_pos.vel_e,  _global_pos.vel_d);
			math::Vector<2> current_position((float)_global_pos.lat, (float)_global_pos.lon);

			/*
			 * Attempt to control position, on success (= sensors present and not in manual mode),
			 * publish setpoint.
			 */
			if (control_position(current_position, ground_speed, _pos_sp_triplet)) {
				_att_sp.timestamp = hrt_absolute_time();

				if (_control_mode.flag_control_manual_enabled) {
					_att_sp.roll_body = math::constrain(_att_sp.roll_body, -_parameters.man_roll_max_rad, _parameters.man_roll_max_rad);
					_att_sp.pitch_body = math::constrain(_att_sp.pitch_body, -_parameters.man_pitch_max_rad, _parameters.man_pitch_max_rad);
				}

				Quatf q(Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body));
				q.copyTo(_att_sp.q_d);
				_att_sp.q_d_valid = true;

				if (!_control_mode.flag_control_offboard_enabled ||
				    _control_mode.flag_control_position_enabled ||
				    _control_mode.flag_control_velocity_enabled ||
				    _control_mode.flag_control_acceleration_enabled) {

					/* lazily publish the setpoint only once available */
					if (_attitude_sp_pub != nullptr) {
						/* publish the attitude setpoint */
						orb_publish(_attitude_setpoint_id, _attitude_sp_pub, &_att_sp);

					} else if (_attitude_setpoint_id) {
						/* advertise and publish */
						_attitude_sp_pub = orb_advertise(_attitude_setpoint_id, &_att_sp);
					}
				}

				/* XXX check if radius makes sense here */
				float turn_distance = _parameters.l1_distance; //_gnd_control.switch_distance(100.0f);

				/* lazily publish navigation capabilities */
				if ((hrt_elapsed_time(&_gnd_pos_ctrl_status.timestamp) > 1000000)
				    || (fabsf(turn_distance - _gnd_pos_ctrl_status.turn_distance) > FLT_EPSILON
					&& turn_distance > 0)) {

					/* set new turn distance */
					_gnd_pos_ctrl_status.turn_distance = turn_distance;

					_gnd_pos_ctrl_status.nav_roll = _gnd_control.nav_roll();
					_gnd_pos_ctrl_status.nav_pitch = get_tecs_pitch();
					_gnd_pos_ctrl_status.nav_bearing = _gnd_control.nav_bearing();

					_gnd_pos_ctrl_status.target_bearing = _gnd_control.target_bearing();
					_gnd_pos_ctrl_status.xtrack_error = _gnd_control.crosstrack_error();

					math::Vector<2> curr_wp((float)_pos_sp_triplet.current.lat, (float)_pos_sp_triplet.current.lon);
					_gnd_pos_ctrl_status.wp_dist = get_distance_to_next_waypoint(current_position(0), current_position(1), curr_wp(0),
								      curr_wp(1));

					gnd_pos_ctrl_status_publish();
				}

			}

			perf_end(_loop_perf);
		}

	}

	_task_running = false;

	warnx("exiting.\n");

	_control_task = -1;
}

void
GroundRoverPositionControl::task_main_trampoline(int argc, char *argv[])
{
	gnd_control::g_control = new GroundRoverPositionControl();

	if (gnd_control::g_control == nullptr) {
		warnx("OUT OF MEM");
		return;
	}

	/* only returns on exit */
	gnd_control::g_control->task_main();
	delete gnd_control::g_control;
	gnd_control::g_control = nullptr;
}


int
GroundRoverPositionControl::start()
{
	ASSERT(_control_task == -1);
	warn("Starting by marco");

	/* start the task */
	_control_task = px4_task_spawn_cmd("gnd_pos_ctrl",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1700,
					   (px4_main_t)&GroundRoverPositionControl::task_main_trampoline,
					   nullptr);
	warn("done");

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int gnd_pos_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: gnd_pos_control {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (gnd_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		if (OK != GroundRoverPositionControl::start()) {
			warn("start failed");
			return 1;
		}

		/* avoid memory fragmentation by not exiting start handler until the task has fully started */
		while (gnd_control::g_control == nullptr || !gnd_control::g_control->task_running()) {
			usleep(50000);
			printf(".");
			fflush(stdout);
		}

		printf("\n");

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (gnd_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete gnd_control::g_control;
		gnd_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (gnd_control::g_control) {
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
