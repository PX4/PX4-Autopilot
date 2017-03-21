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
 * @file gnd_pos_control_l1_main.c
 * Implementation of a generic position controller based on the L1 norm. Outputs a bank / roll
 * angle, equivalent to a lateral motion (for copters and rovers).
 *
 * Original publication for horizontal control class:
 *    S. Park, J. Deyst, and J. P. How, "A New Nonlinear Guidance Logic for Trajectory Tracking,"
 *    Proceedings of the AIAA Guidance, Navigation and Control
 *    Conference, Aug 2004. AIAA-2004-4900.
 *
 * Original implementation for total energy control class:
 *    Paul Riseborough and Andrew Tridgell, 2013 (code in lib/external_lgpl)
 *
 * More details and acknowledgements in the referenced library headers.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 * @author Andreas Antener <andreas@uaventure.com>
 * @author Marco Zorzi <mzorzi@student.ethz.ch>
 */

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <float.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>

//#include "landingslope.h"

#include <arch/board/board.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_hrt.h>
#include <ecl/l1/ecl_l1_pos_controller.h>
#include <external_lgpl/tecs/tecs.h>
#include <geo/geo.h>
#include <launchdetection/LaunchDetector.h>
#include <mathlib/mathlib.h>
#include <platforms/px4_defines.h>
#include <runway_takeoff/RunwayTakeoff.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>
#include <systemlib/pid/pid.h>
#include <systemlib/systemlib.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/fw_pos_ctrl_status.h>
#include <uORB/topics/fw_virtual_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/tecs_status.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/uORB.h>

static int	_control_task = -1;			/**< task handle for sensor task */

#define HDG_HOLD_DIST_NEXT 			3000.0f 	// initial distance of waypoint in front of plane in heading hold mode
#define HDG_HOLD_REACHED_DIST 		1000.0f 	// distance (plane to waypoint in front) at which waypoints are reset in heading hold mode
#define HDG_HOLD_SET_BACK_DIST 		100.0f 		// distance by which previous waypoint is set behind the plane
#define HDG_HOLD_YAWRATE_THRESH 	0.15f 		// max yawrate at which plane locks yaw for heading hold mode
#define HDG_HOLD_MAN_INPUT_THRESH 	0.01f 		// max manual roll/yaw input from user which does not change the locked heading
#define T_ALT_TIMEOUT 				1 			// time after which we abort landing if terrain estimate is not valid
#define THROTTLE_THRESH 0.05f 	///< max throttle from user which will not lead to motors spinning up in altitude controlled modes
#define MANUAL_THROTTLE_CLIMBOUT_THRESH 0.85f	///< a throttle / pitch input above this value leads to the system switching to climbout mode
#define ALTHOLD_EPV_RESET_THRESH 5.0f

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
	int		_sensor_combined_sub;		/**< for body frame accelerations */

	orb_advert_t	_attitude_sp_pub;		/**< attitude setpoint */
	orb_advert_t	_tecs_status_pub;		/**< TECS status publication */
	orb_advert_t	_fw_pos_ctrl_status_pub;		/**< navigation capabilities publication */

	orb_id_t _attitude_setpoint_id;

	struct control_state_s				_ctrl_state;			/**< control state */
	struct vehicle_attitude_setpoint_s		_att_sp;			/**< vehicle attitude setpoint */
	struct fw_pos_ctrl_status_s			_fw_pos_ctrl_status;		/**< navigation capabilities */
	struct manual_control_setpoint_s		_manual;			/**< r/c channel data */
	struct vehicle_control_mode_s			_control_mode;			/**< control mode */
	struct vehicle_command_s			_vehicle_command;		/**< vehicle commands */
	struct vehicle_status_s				_vehicle_status;		/**< vehicle status */
	struct vehicle_land_detected_s			_vehicle_land_detected;		/**< vehicle land detected */
	struct vehicle_global_position_s		_global_pos;			/**< global vehicle position */
	struct position_setpoint_triplet_s		_pos_sp_triplet;		/**< triplet of mission items */
	struct sensor_combined_s			_sensor_combined;		/**< for body frame accelerations */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	float	_hold_alt;				/**< hold altitude for altitude mode */
	float	_takeoff_ground_alt;			/**< ground altitude at which plane was launched */
	float	_hdg_hold_yaw;				/**< hold heading for velocity mode */
	bool	_hdg_hold_enabled;			/**< heading hold enabled */
	bool	_yaw_lock_engaged;			/**< yaw is locked for heading hold */
	float	_althold_epv;				/**< the position estimate accuracy when engaging alt hold */
	bool	_was_in_deadband;			/**< wether the last stick input was in althold deadband */
	struct position_setpoint_s _hdg_hold_prev_wp;	/**< position where heading hold started */
	struct position_setpoint_s _hdg_hold_curr_wp;	/**< position to which heading hold flies */
	hrt_abstime _control_position_last_called; 	/**<last call of control_position  */

	/* Landing */
	bool _land_noreturn_horizontal;
	bool _land_noreturn_vertical;
	bool _land_stayonground;
	bool _land_motor_lim;
	bool _land_onslope;
	bool _land_useterrain;

	//Landingslope _landingslope;

	hrt_abstime _time_started_landing;	//*< time at which landing started */

	float _t_alt_prev_valid;		//**< last terrain estimate which was valid */
	hrt_abstime _time_last_t_alt; 		//*< time at which we had last valid terrain alt */

	float _flare_height;				//*< estimated height to ground at which flare started */
	float _flare_curve_alt_rel_last;
	float _target_bearing;				//*< estimated height to ground at which flare started */

	bool _was_in_air;	/**< indicated wether the plane was in the air in the previous interation*/
	hrt_abstime _time_went_in_air;	/**< time at which the plane went in the air */

	/* Takeoff launch detection and runway */
	launchdetection::LaunchDetector _launchDetector;
	LaunchDetectionResult _launch_detection_state;

	runwaytakeoff::RunwayTakeoff _runway_takeoff;

	/* throttle and airspeed states */
	float _airspeed_error;				///< airspeed error to setpoint in m/s
	bool _airspeed_valid;				///< flag if a valid airspeed estimate exists
	uint64_t _airspeed_last_received;		///< last time airspeed was received. Used to detect timeouts.
	
	float _gpsspeed_error;				///< gpsspeed error to setpoint in m/s
	bool _gpsspeed_valid;				///< flag if a valid gpsspeed estimate exists
	uint64_t _gpsspeed_last_received;		///< last time gpsspeed was received. Used to detect timeouts.
	
	float _groundspeed_undershoot;			///< ground speed error to min. speed in m/s
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
	enum FW_POSCTRL_MODE {
		FW_POSCTRL_MODE_AUTO,
		FW_POSCTRL_MODE_POSITION,
		FW_POSCTRL_MODE_ALTITUDE,
		FW_POSCTRL_MODE_OTHER
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
		float climbout_diff;
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
		float airspeed_trans;
		int airspeed_mode;

		float gpsspeed_min;
		float gpsspeed_trim;
		float gpsspeed_max;
		float gpsspeed_trans;
		int gpsspeed_mode;

		float pitch_limit_min;
		float pitch_limit_max;
		float roll_limit;
		float throttle_min;
		float throttle_max;
		float throttle_idle;
		float throttle_cruise;
		float throttle_slew_max;
		float man_roll_max_rad;
		float man_pitch_max_rad;
		float rollsp_offset_rad;
		float pitchsp_offset_rad;


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
		param_t climbout_diff;
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
		param_t airspeed_trans;
		param_t airspeed_mode;

		param_t gpsspeed_min;
		param_t gpsspeed_trim;
		param_t gpsspeed_max;
		param_t gpsspeed_trans;
		param_t gpsspeed_mode;


		param_t pitch_limit_min;
		param_t pitch_limit_max;
		param_t roll_limit;
		param_t throttle_min;
		param_t throttle_max;
		param_t throttle_idle;
		param_t throttle_cruise;
		param_t throttle_slew_max;
		param_t man_roll_max_deg;
		param_t man_pitch_max_deg;
		param_t rollsp_offset_deg;
		param_t pitchsp_offset_deg;


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
	 * Check for accel updates.
	 */
	void		vehicle_sensor_combined_poll();

	/**
	 * Check for set triplet updates.
	 */
	void		vehicle_setpoint_poll();

	/**
	 * Publish navigation capabilities
	 */
	void		fw_pos_ctrl_status_publish();

	/**
	 * Get a new waypoint based on heading and distance from current position
	 *
	 * @param heading the heading to fly to
	 * @param distance the distance of the generated waypoint
	 * @param waypoint_prev the waypoint at the current position
	 * @param waypoint_next the waypoint in the heading direction
	 */
	void		get_waypoint_heading_distance(float heading, float distance,
			struct position_setpoint_s &waypoint_prev, struct position_setpoint_s &waypoint_next, bool flag_init);

	
	/**
	 * Update desired altitude base on user pitch stick input
	 *
	 * @param dt Time step
	 * @return true if climbout mode was requested by user (climb with max rate and min airspeed)
	 */
	// bool		update_desired_altitude(float dt);

	/**
	 * Control position.
	 */
	bool		control_position(const math::Vector<2> &global_pos, const math::Vector<3> &ground_speed,
					 const struct position_setpoint_triplet_s &_pos_sp_triplet);

	float		get_tecs_pitch();
	float		get_tecs_thrust();

	float		get_demanded_airspeed();
	float		calculate_target_airspeed(float airspeed_demand);
	void		calculate_gndspeed_undershoot(const math::Vector<2> &current_position, const math::Vector<2> &ground_speed_2d,
			const struct position_setpoint_triplet_s &pos_sp_triplet);

	/**
	 * Handle incoming vehicle commands
	 */
	void		handle_command();

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
	void tecs_update_pitch_throttle(float alt_sp, float v_sp, float eas2tas,
					float pitch_min_rad, float pitch_max_rad,
					float throttle_min, float throttle_max, float throttle_cruise,
					bool climbout_mode, float climbout_pitch_min_rad,
					float altitude,
					const math::Vector<3> &ground_speed,
					unsigned mode = tecs_status_s::TECS_MODE_NORMAL);

};

namespace gnd_control
{

GroundRoverPositionControl	*g_control = nullptr;
}

/*********************************************************************************************************/
/******************************************* CONSTRUCTOR DESTRUCTOR **************************************/
/*********************************************************************************************************/

/**
 * @brief      Constructs the object.
 */
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
	_sensor_combined_sub(-1),

	/* publications */
	_attitude_sp_pub(nullptr),
	_tecs_status_pub(nullptr),
	_fw_pos_ctrl_status_pub(nullptr),

	/* publication ID */
	_attitude_setpoint_id(nullptr),

	/* states */
	_ctrl_state(),
	_att_sp(),
	_fw_pos_ctrl_status(),
	_manual(),
	_control_mode(),
	_vehicle_command(),
	_vehicle_status(),
	_vehicle_land_detected(),
	_global_pos(),
	_pos_sp_triplet(),
	_sensor_combined(),

	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "fw l1 control")),

	_hold_alt(0.0f),
	_takeoff_ground_alt(0.0f),
	_hdg_hold_yaw(0.0f),
	_hdg_hold_enabled(false),
	_yaw_lock_engaged(false),
	_althold_epv(0.0f),
	_was_in_deadband(false),
	_hdg_hold_prev_wp{},
	_hdg_hold_curr_wp{},
	_control_position_last_called(0),
	_land_noreturn_horizontal(false),
	_land_noreturn_vertical(false),
	_land_stayonground(false),
	_land_motor_lim(false),
	_land_onslope(false),
	_land_useterrain(false),
	//_landingslope(),
	_time_started_landing(0),
	_t_alt_prev_valid(0),
	_time_last_t_alt(0),
	_flare_height(0.0f),
	_flare_curve_alt_rel_last(0.0f),
	_target_bearing(0.0f),
	_was_in_air(false),
	_time_went_in_air(0),
	_launchDetector(),
	_launch_detection_state(LAUNCHDETECTION_RES_NONE),
	_runway_takeoff(),
	_airspeed_error(0.0f),
	_airspeed_valid(false),
	_airspeed_last_received(0),
	_gpsspeed_error(0.0f),
	_gpsspeed_valid(false),
	_gpsspeed_last_received(0),

	_groundspeed_undershoot(0.0f),
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
	_control_mode_current(FW_POSCTRL_MODE_OTHER),
	_parameters(),
	_parameter_handles()
{
	_fw_pos_ctrl_status = {};

	_parameter_handles.l1_period = param_find("GND_L1_PERIOD");
	_parameter_handles.l1_damping = param_find("GND_L1_DAMPING");
	_parameter_handles.l1_distance = param_find("GND_L1_DIST");


	_parameter_handles.airspeed_min = param_find("FW_AIRSPD_MIN");
	_parameter_handles.airspeed_trim = param_find("FW_AIRSPD_TRIM");
	_parameter_handles.airspeed_max = param_find("FW_AIRSPD_MAX");
	_parameter_handles.airspeed_trans = param_find("VT_ARSP_TRANS");
	_parameter_handles.airspeed_mode = param_find("FW_ARSP_MODE");

	_parameter_handles.pitch_limit_min = param_find("GND_P_LIM_MIN");
	_parameter_handles.pitch_limit_max = param_find("GND_P_LIM_MAX");
	_parameter_handles.roll_limit = param_find("GND_R_LIM");
	_parameter_handles.throttle_min = param_find("GND_THR_MIN");
	_parameter_handles.throttle_max = param_find("GND_THR_MAX");
	_parameter_handles.throttle_idle = param_find("GND_THR_IDLE");
	_parameter_handles.throttle_slew_max = param_find("GND_THR_SLEW_MAX");
	_parameter_handles.throttle_cruise = param_find("GND_THR_CRUISE");
	_parameter_handles.man_roll_max_deg = param_find("FW_MAN_R_MAX");
	_parameter_handles.man_pitch_max_deg = param_find("FW_MAN_P_MAX");
	_parameter_handles.rollsp_offset_deg = param_find("FW_RSP_OFF");
	_parameter_handles.pitchsp_offset_deg = param_find("FW_PSP_OFF");

	_parameter_handles.time_const = 			param_find("FW_T_TIME_CONST");
	_parameter_handles.time_const_throt = 			param_find("FW_T_THRO_CONST");
	_parameter_handles.min_sink_rate = 			param_find("FW_T_SINK_MIN");
	_parameter_handles.max_sink_rate =			param_find("FW_T_SINK_MAX");
	_parameter_handles.max_climb_rate =			param_find("FW_T_CLMB_MAX");
	_parameter_handles.climbout_diff =			param_find("FW_CLMBOUT_DIFF");
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

/*********************************************************************************************************/
/********************************************** PARAMETERS UPDATE ****************************************/
/*********************************************************************************************************/
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
	param_get(_parameter_handles.airspeed_trans, &(_parameters.airspeed_trans));
	param_get(_parameter_handles.airspeed_mode, &(_parameters.airspeed_mode));

	param_get(_parameter_handles.pitch_limit_min, &(_parameters.pitch_limit_min));
	param_get(_parameter_handles.pitch_limit_max, &(_parameters.pitch_limit_max));
	param_get(_parameter_handles.roll_limit, &(_parameters.roll_limit));
	param_get(_parameter_handles.throttle_min, &(_parameters.throttle_min));
	param_get(_parameter_handles.throttle_max, &(_parameters.throttle_max));
	param_get(_parameter_handles.throttle_idle, &(_parameters.throttle_idle));
	param_get(_parameter_handles.throttle_cruise, &(_parameters.throttle_cruise));
	param_get(_parameter_handles.throttle_slew_max, &(_parameters.throttle_slew_max));

	param_get(_parameter_handles.man_roll_max_deg, &_parameters.man_roll_max_rad);
	_parameters.man_roll_max_rad = math::radians(_parameters.man_roll_max_rad);
	param_get(_parameter_handles.man_pitch_max_deg, &_parameters.man_pitch_max_rad);
	_parameters.man_pitch_max_rad = math::radians(_parameters.man_pitch_max_rad);
	param_get(_parameter_handles.rollsp_offset_deg, &_parameters.rollsp_offset_rad);
	_parameters.rollsp_offset_rad = math::radians(_parameters.rollsp_offset_rad);
	param_get(_parameter_handles.pitchsp_offset_deg, &_parameters.pitchsp_offset_rad);
	_parameters.pitchsp_offset_rad = math::radians(_parameters.pitchsp_offset_rad);


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
	param_get(_parameter_handles.climbout_diff, &(_parameters.climbout_diff));

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

	/* sanity check parameters 
	if (_parameters.airspeed_max < _parameters.airspeed_min ||
	    _parameters.airspeed_max < 5.0f ||
	    _parameters.airspeed_min > 100.0f ||
	    _parameters.airspeed_trim < _parameters.airspeed_min ||
	    _parameters.airspeed_trim > _parameters.airspeed_max) {
		warnx("error: airspeed parameters invalid");
		return 1;
	}
	*/

	/* Update and publish the navigation capabilities */
	_fw_pos_ctrl_status.landing_slope_angle_rad = 0;//_landingslope.landing_slope_angle_rad();
	_fw_pos_ctrl_status.landing_horizontal_slope_displacement = 0;// _landingslope.horizontal_slope_displacement();
	_fw_pos_ctrl_status.landing_flare_length = 0;// _landingslope.flare_length();
	fw_pos_ctrl_status_publish();

	/* Update Launch Detector Parameters */
	_launchDetector.updateParams();

	_runway_takeoff.updateParams();

	return OK;
}

/*********************************************************************************************************/
/************************************************* POLLINGS **********************************************/
/*********************************************************************************************************/
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
		handle_command();
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
GroundRoverPositionControl::vehicle_sensor_combined_poll()
{
	/* check if there is a new position */
	bool sensors_updated;
	orb_check(_sensor_combined_sub, &sensors_updated);

	if (sensors_updated) {
		orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub, &_sensor_combined);
	}
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
GroundRoverPositionControl::get_demanded_airspeed()
{
	float altctrl_airspeed = 0;

	// neutral throttle corresponds to trim airspeed
	if (_manual.z < 0.5f) {
		// lower half of throttle is min to trim airspeed
		altctrl_airspeed = _parameters.airspeed_min +
				   (_parameters.airspeed_trim - _parameters.airspeed_min) *
				   _manual.z * 2;

	} else {
		// upper half of throttle is trim to max airspeed
		altctrl_airspeed = _parameters.airspeed_trim +
				   (_parameters.airspeed_max - _parameters.airspeed_trim) *
				   (_manual.z * 2 - 1);
	}

	return altctrl_airspeed;
}

float
GroundRoverPositionControl::calculate_target_airspeed(float airspeed_demand)
{
	float airspeed;

	if (_airspeed_valid) {
		airspeed = _ctrl_state.airspeed;

	} else {
		airspeed = _parameters.airspeed_min + (_parameters.airspeed_max - _parameters.airspeed_min) / 2.0f;
	}

	/* cruise airspeed for all modes unless modified below */
	float target_airspeed = airspeed_demand;

	/* add minimum ground speed undershoot (only non-zero in presence of sufficient wind) */
	target_airspeed += _groundspeed_undershoot;

	if (0/* throttle nudging enabled */) {
		//target_airspeed += nudge term.
	}

	/* sanity check: limit to range */
	target_airspeed = math::constrain(target_airspeed, _parameters.airspeed_min, _parameters.airspeed_max);

	/* plain airspeed error */
	_airspeed_error = target_airspeed - airspeed;

	return target_airspeed;
}

void
GroundRoverPositionControl::calculate_gndspeed_undershoot(const math::Vector<2> &current_position,
		const math::Vector<2> &ground_speed_2d, const struct position_setpoint_triplet_s &pos_sp_triplet)
{

	if (pos_sp_triplet.current.valid && !_gnd_control.circle_mode()) {

		/* rotate ground speed vector with current attitude */
		math::Vector<2> yaw_vector(_R_nb(0, 0), _R_nb(1, 0));
		yaw_vector.normalize();
		float ground_speed_body = yaw_vector * ground_speed_2d;

		/* The minimum desired ground speed is the minimum airspeed projected on to the ground using the altitude and horizontal difference between the waypoints if available*/
		float distance = 0.0f;
		float delta_altitude = 0.0f;

		if (pos_sp_triplet.previous.valid) {
			distance = get_distance_to_next_waypoint(pos_sp_triplet.previous.lat, pos_sp_triplet.previous.lon,
					pos_sp_triplet.current.lat, pos_sp_triplet.current.lon);
			delta_altitude = pos_sp_triplet.current.alt - pos_sp_triplet.previous.alt;

		} else {
			distance = get_distance_to_next_waypoint(current_position(0), current_position(1), pos_sp_triplet.current.lat,
					pos_sp_triplet.current.lon);
			delta_altitude = pos_sp_triplet.current.alt -  _global_pos.alt;
		}

		float ground_speed_desired = _parameters.airspeed_min * cosf(atan2f(delta_altitude, distance));


		/*
		 * Ground speed undershoot is the amount of ground velocity not reached
		 * by the plane. Consequently it is zero if airspeed is >= min ground speed
		 * and positive if airspeed < min ground speed.
		 *
		 * This error value ensures that a plane (as long as its throttle capability is
		 * not exceeded) travels towards a waypoint (and is not pushed more and more away
		 * by wind). Not countering this would lead to a fly-away.
		 */
		_groundspeed_undershoot = math::max(ground_speed_desired - ground_speed_body, 0.0f);

	} else {
		_groundspeed_undershoot = 0;
	}
}


void GroundRoverPositionControl::fw_pos_ctrl_status_publish()
{
	_fw_pos_ctrl_status.timestamp = hrt_absolute_time();

	if (_fw_pos_ctrl_status_pub != nullptr) {
		orb_publish(ORB_ID(fw_pos_ctrl_status), _fw_pos_ctrl_status_pub, &_fw_pos_ctrl_status);

	} else {
		_fw_pos_ctrl_status_pub = orb_advertise(ORB_ID(fw_pos_ctrl_status), &_fw_pos_ctrl_status);
	}
}

void GroundRoverPositionControl::get_waypoint_heading_distance(float heading, float distance,
		struct position_setpoint_s &waypoint_prev, struct position_setpoint_s &waypoint_next, bool flag_init)
{
	waypoint_prev.valid = true;
	waypoint_prev.alt = _hold_alt;
	position_setpoint_s temp_next {};
	position_setpoint_s temp_prev {};

	if (flag_init) {
		// on init set previous waypoint HDG_HOLD_SET_BACK_DIST meters behind us
		waypoint_from_heading_and_distance(_global_pos.lat, _global_pos.lon, heading + 180.0f * M_DEG_TO_RAD_F,
						   HDG_HOLD_SET_BACK_DIST,
						   &temp_prev.lat, &temp_prev.lon);

		// set next waypoint HDG_HOLD_DIST_NEXT meters in front of us
		waypoint_from_heading_and_distance(_global_pos.lat, _global_pos.lon, heading, HDG_HOLD_DIST_NEXT,
						   &temp_next.lat, &temp_next.lon);
		waypoint_prev = temp_prev;
		waypoint_next = temp_next;
		waypoint_next.valid = true;
		waypoint_next.alt = _hold_alt;

		return;


	} else {
		// for previous waypoint use the one still in front of us but shift it such that it is
		// located on the desired flight path but HDG_HOLD_SET_BACK_DIST behind us
		create_waypoint_from_line_and_dist(waypoint_next.lat, waypoint_next.lon, waypoint_prev.lat, waypoint_prev.lon,
						   HDG_HOLD_REACHED_DIST + HDG_HOLD_SET_BACK_DIST,
						   &temp_prev.lat, &temp_prev.lon);
	}

	waypoint_next.valid = true;

	create_waypoint_from_line_and_dist(waypoint_next.lat, waypoint_next.lon, waypoint_prev.lat, waypoint_prev.lon,
					   -(HDG_HOLD_DIST_NEXT + HDG_HOLD_REACHED_DIST),
					   &temp_next.lat, &temp_next.lon);
	waypoint_prev = temp_prev;
	waypoint_next = temp_next;
	waypoint_next.alt = _hold_alt;
}





void
GroundRoverPositionControl::handle_command()
{
	if (_vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_GO_AROUND) {
		// only abort landing before point of no return (horizontal and vertical)
		if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {

			if (_land_noreturn_vertical) {
				mavlink_log_info(&_mavlink_log_pub, "#Landing, can't abort after flare");

			} else {
				_fw_pos_ctrl_status.abort_landing = true;
				mavlink_log_info(&_mavlink_log_pub, "#Landing, aborted");
			}
		}
	}
}



/*********************************************************************************************************/
/*********************************************** TECS FUNCTIONS ******************************************/
/*********************************************************************************************************/

void GroundRoverPositionControl::tecs_update_pitch_throttle(float alt_sp, float v_sp, float eas2tas,
		float pitch_min_rad, float pitch_max_rad,
		float throttle_min, float throttle_max, float throttle_cruise,
		bool climbout_mode, float climbout_pitch_min_rad,
		float altitude,
		const math::Vector<3> &ground_speed,
		unsigned mode)
{
	float dt = 0.01f; // prevent division with 0

	if (_last_tecs_update > 0) {
		dt = hrt_elapsed_time(&_last_tecs_update) * 1e-6;
	}

	_last_tecs_update = hrt_absolute_time();

	// do not run TECS if we are not in air
	bool run_tecs = !_vehicle_land_detected.landed;

	// do not run TECS if vehicle is a VTOL and we are in rotary wing mode or in transition
	// (it should also not run during VTOL blending because airspeed is too low still)
	if (_vehicle_status.is_vtol) {
		run_tecs &= !_vehicle_status.is_rotary_wing && !_vehicle_status.in_transition_mode;
	}

	// we're in transition
	if (_vehicle_status.is_vtol && _vehicle_status.in_transition_mode) {
		_was_in_transition = true;

		// set this to transition airspeed to init tecs correctly
		if (_parameters.airspeed_mode == control_state_s::AIRSPD_MODE_DISABLED) {
			// some vtols fly without airspeed sensor
			_asp_after_transition = _parameters.airspeed_trans;

		} else {
			_asp_after_transition = _ctrl_state.airspeed;
		}

		_asp_after_transition = math::constrain(_asp_after_transition, _parameters.airspeed_min, _parameters.airspeed_max);

	} else if (_was_in_transition) {
		// after transition we ramp up desired airspeed from the speed we had coming out of the transition
		_asp_after_transition += dt * 2; // increase 2m/s

		if (_asp_after_transition < v_sp && _ctrl_state.airspeed < v_sp) {
			v_sp = fmaxf(_asp_after_transition, _ctrl_state.airspeed);

		} else {
			_was_in_transition = false;
			_asp_after_transition = 0;
		}
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

	if (_vehicle_status.engine_failure || _vehicle_status.engine_failure_cmd) {
		/* Force the slow downwards spiral */
		pitch_min_rad = M_DEG_TO_RAD_F * -1.0f;
		pitch_max_rad = M_DEG_TO_RAD_F * 5.0f;
	}

	/* No underspeed protection in landing mode */
	_tecs.set_detect_underspeed_enabled(!(mode == tecs_status_s::TECS_MODE_LAND
					      || mode == tecs_status_s::TECS_MODE_LAND_THROTTLELIM));

	/* Using tecs library */
	float pitch_for_tecs = _pitch - _parameters.pitchsp_offset_rad;

	_tecs.update_pitch_throttle(_R_nb, pitch_for_tecs, altitude, alt_sp, v_sp,
				    _ctrl_state.airspeed, eas2tas,
				    climbout_mode, climbout_pitch_min_rad,
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

	case TECS::ECL_TECS_MODE_UNDERSPEED:
		t.mode = tecs_status_s::TECS_MODE_UNDERSPEED;
		break;

	case TECS::ECL_TECS_MODE_BAD_DESCENT:
		t.mode = tecs_status_s::TECS_MODE_BAD_DESCENT;
		break;

	case TECS::ECL_TECS_MODE_CLIMBOUT:
		t.mode = tecs_status_s::TECS_MODE_CLIMBOUT;
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

/*********************************************************************************************************/
/****************************************** MAIN CONTROL LOOP ********************************************/
/*********************************************************************************************************/
bool
GroundRoverPositionControl::control_position(const math::Vector<2> &current_position, const math::Vector<3> &ground_speed,
		const struct position_setpoint_triplet_s &pos_sp_triplet)
{
	// float dt = 0.01; // Using non zero value to a avoid division by zero
	// if (_control_position_last_called > 0) {
	// 	dt = (float)hrt_elapsed_time(&_control_position_last_called) * 1e-6f;
	// }

	_control_position_last_called = hrt_absolute_time();

	/* only run position controller in fixed-wing mode and during transitions for VTOL */
	if (_vehicle_status.is_rotary_wing && !_vehicle_status.in_transition_mode) {
		_control_mode_current = FW_POSCTRL_MODE_OTHER;
		return false;
	}

	bool setpoint = true;

	_att_sp.fw_control_yaw = true;		// by default we don't want yaw to be contoller directly with rudder
	float eas2tas = 1.0f; // XXX calculate actual number based on current measurements

	/* filter speed and altitude for controller */
	math::Vector<3> accel_body(_sensor_combined.accelerometer_m_s2);

	math::Vector<3> accel_earth = _R_nb * accel_body;

	/* tell TECS to update its state, but let it know when it cannot actually control the plane */
	bool in_air_alt_control = (!_vehicle_land_detected.landed &&
				   (_control_mode.flag_control_auto_enabled ||
				    _control_mode.flag_control_velocity_enabled ||
				    _control_mode.flag_control_altitude_enabled));

	/* update TECS filters */
	_tecs.update_state(_global_pos.alt, _ctrl_state.airspeed, _R_nb,
			   accel_body, accel_earth, (_global_pos.timestamp > 0), in_air_alt_control);

	math::Vector<2> ground_speed_2d = {ground_speed(0), ground_speed(1)};
	calculate_gndspeed_undershoot(current_position, ground_speed_2d, pos_sp_triplet);

	// l1 navigation logic breaks down when wind speed exceeds max airspeed
	// compute 2D groundspeed from airspeed-heading projection
	math::Vector<2> air_speed_2d = {_ctrl_state.airspeed * cosf(_yaw), _ctrl_state.airspeed * sinf(_yaw)};
	math::Vector<2> nav_speed_2d = {0, 0};


	/***** Comment this out to remove air speed and only use gps speed ******/
	// angle between air_speed_2d and ground_speed_2d
	//float air_gnd_angle = acosf((air_speed_2d * ground_speed_2d) / (air_speed_2d.length() * ground_speed_2d.length()));

	// // if angle > 90 degrees or groundspeed is less than threshold, replace groundspeed with airspeed projection
	// if ((fabsf(air_gnd_angle) > (float)M_PI) || (ground_speed_2d.length() < 3.0f)) {
	// 	nav_speed_2d = air_speed_2d;

	// } else {
	// 	nav_speed_2d = ground_speed_2d;
	// }
	/***** Commented this out to remove air speed and only use gps speed ******/
	
	nav_speed_2d = ground_speed_2d;


	/* define altitude error */
	// float altitude_error = pos_sp_triplet.current.alt - _global_pos.alt;

	/* no throttle limit as default */
	float throttle_max = 1.0f;

	/* save time when airplane is in air */
	if (!_was_in_air && !_vehicle_land_detected.landed) {
		_was_in_air = true;
		_time_went_in_air = hrt_absolute_time();
		_takeoff_ground_alt = _global_pos.alt;
	}

	/* reset flag when airplane landed */
	if (_vehicle_land_detected.landed) {
		_was_in_air = false;
	}

	if (_control_mode.flag_control_auto_enabled && pos_sp_triplet.current.valid) {
		/* AUTONOMOUS FLIGHT */

		/* Reset integrators if switching to this mode from a other mode in which posctl was not active */
		if (_control_mode_current == FW_POSCTRL_MODE_OTHER) {
			/* reset integrators */
			_tecs.reset_state();
		}

		_control_mode_current = FW_POSCTRL_MODE_AUTO;

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

		float mission_airspeed = _parameters.airspeed_trim;

		if (PX4_ISFINITE(_pos_sp_triplet.current.cruising_speed) &&
		    _pos_sp_triplet.current.cruising_speed > 0.1f) {
			mission_airspeed = _pos_sp_triplet.current.cruising_speed;
		}

		float mission_throttle = _parameters.throttle_cruise;

		if (PX4_ISFINITE(_pos_sp_triplet.current.cruising_throttle) &&
		    _pos_sp_triplet.current.cruising_throttle > 0.01f) {

			mission_throttle = _pos_sp_triplet.current.cruising_throttle;
		}

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

			tecs_update_pitch_throttle(pos_sp_triplet.current.alt, calculate_target_airspeed(mission_airspeed), eas2tas,
						   math::radians(_parameters.pitch_limit_min), math::radians(_parameters.pitch_limit_max),
						   _parameters.throttle_min, _parameters.throttle_max, mission_throttle,
						   false, math::radians(_parameters.pitch_limit_min), _global_pos.alt, ground_speed);

		} else if (pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LOITER) {

			/* waypoint is a loiter waypoint */
			_gnd_control.navigate_loiter(curr_wp, current_position, pos_sp_triplet.current.loiter_radius,
						    pos_sp_triplet.current.loiter_direction, nav_speed_2d);
			_att_sp.roll_body = _gnd_control.nav_roll();
			_att_sp.yaw_body = _gnd_control.nav_bearing();

			float alt_sp = pos_sp_triplet.current.alt;

			tecs_update_pitch_throttle(alt_sp,
						   calculate_target_airspeed(mission_airspeed),
						   eas2tas,
						   math::radians(_parameters.pitch_limit_min),
						   math::radians(_parameters.pitch_limit_max),
						   0, // _parameters.throttle_min,
						   0, // _parameters.throttle_max,
						   0, // _parameters.throttle_cruise,
						   false,
						   math::radians(_parameters.pitch_limit_min),
						   _global_pos.alt,
						   ground_speed);
		}

	

		if (was_circle_mode && !_gnd_control.circle_mode()) {
			/* just kicked out of loiter, reset roll integrals */
			_att_sp.roll_reset_integral = true;
		}

	}else {
		_control_mode_current = FW_POSCTRL_MODE_OTHER;

		/* do not publish the setpoint */
		setpoint = false;

		// reset hold altitude
		_hold_alt = _global_pos.alt;

	}

	/* Copy thrust output for publication */
	if (_vehicle_status.engine_failure ||
		_vehicle_status.engine_failure_cmd ||
		   ( _control_mode_current == FW_POSCTRL_MODE_AUTO &&
		   	 pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_IDLE ) ||
		   _control_mode_current == FW_POSCTRL_MODE_OTHER
		) {
		/* Set thrust to 0 to minimize damage */
		_att_sp.thrust = 0.0f;
	} else {
		_att_sp.thrust = math::min(get_tecs_thrust(), throttle_max);
	}

	// decide when to use pitch setpoint from TECS because in some cases pitch
	// setpoint is generated by other means
	bool use_tecs_pitch = true;

	// auto runway takeoff
	use_tecs_pitch &= !(_control_mode_current ==  FW_POSCTRL_MODE_AUTO &&
			    pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF &&
			    (_launch_detection_state == LAUNCHDETECTION_RES_NONE || _runway_takeoff.runwayTakeoffEnabled()));


	// flaring during landing
	use_tecs_pitch &= !(pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND &&
			    _land_noreturn_vertical);

	// manual attitude control
	use_tecs_pitch &= !(_control_mode_current == FW_POSCTRL_MODE_OTHER);

	if (use_tecs_pitch) {
		_att_sp.pitch_body = get_tecs_pitch();
	}

	return setpoint;
}


/*********************************************************************************************************/
/*********************************************** MAIN LOOP ***********************************************/
/*********************************************************************************************************/
/**
 * @brief      Main task which starts the control loop
 */
void
GroundRoverPositionControl::task_main()
{

	/*
	 * do subscriptions
	 */
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	_ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
	_sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
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
			vehicle_sensor_combined_poll();
			vehicle_manual_control_setpoint_poll();

			/******************************************************************************
			* MARCO HERE: here the gps speed is parsed in the NED frame
			* With the rover we don't need air speed and all the control here.
			******************************************************************************/
			math::Vector<3> ground_speed(_global_pos.vel_n, _global_pos.vel_e,  _global_pos.vel_d);
			math::Vector<2> current_position((float)_global_pos.lat, (float)_global_pos.lon);

			/*
			 * Attempt to control position, on success (= sensors present and not in manual mode),
			 * publish setpoint.
			 */
			if (control_position(current_position, ground_speed, _pos_sp_triplet)) {
				_att_sp.timestamp = hrt_absolute_time();

				// add attitude setpoint offsets
				_att_sp.roll_body += _parameters.rollsp_offset_rad;
				_att_sp.pitch_body += _parameters.pitchsp_offset_rad;

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
				if ((hrt_elapsed_time(&_fw_pos_ctrl_status.timestamp) > 1000000)
				    || (fabsf(turn_distance - _fw_pos_ctrl_status.turn_distance) > FLT_EPSILON
					&& turn_distance > 0)) {

					/* set new turn distance */
					_fw_pos_ctrl_status.turn_distance = turn_distance;

					_fw_pos_ctrl_status.nav_roll = _gnd_control.nav_roll();
					_fw_pos_ctrl_status.nav_pitch = get_tecs_pitch();
					_fw_pos_ctrl_status.nav_bearing = _gnd_control.nav_bearing();

					_fw_pos_ctrl_status.target_bearing = _gnd_control.target_bearing();
					_fw_pos_ctrl_status.xtrack_error = _gnd_control.crosstrack_error();

					math::Vector<2> curr_wp((float)_pos_sp_triplet.current.lat, (float)_pos_sp_triplet.current.lon);
					_fw_pos_ctrl_status.wp_dist = get_distance_to_next_waypoint(current_position(0), current_position(1), curr_wp(0),
								      curr_wp(1));

					fw_pos_ctrl_status_publish();
				}

			}

			perf_end(_loop_perf);
		}

	}

	_task_running = false;

	warnx("exiting.\n");

	_control_task = -1;
}


/*********************************************************************************************************/
/********************************************** APP STARTING *********************************************/
/*********************************************************************************************************/
/**
 * @brief      this creates a new object and starts the main task which loops
 *
 * @param[in]  argc  number of starting strings, not used
 * @param      argv  starting characters, not used
 */
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


/**
 * @brief      Spawns a new task for the PX4
 *
 * @return     OK or error
 */
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

/**
 * @brief      Lander function that starts the application
 *
 * @param[in]  argc  number of appended strings
 * @param      argv  The string commands attached
 *
 * @return     success or not
 */
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
