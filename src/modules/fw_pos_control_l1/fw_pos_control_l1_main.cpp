/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: 	Lorenz Meier
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
 * @file fw_pos_control_l1_main.c
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
 */

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
#include <drivers/drv_accel.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_global_position_set_triplet.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/navigation_capabilities.h>
#include <uORB/topics/parameter_update.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/pid/pid.h>
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>

#include <ecl/l1/ecl_l1_pos_controller.h>
#include <external_lgpl/tecs/tecs.h>

/**
 * L1 control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int fw_pos_control_l1_main(int argc, char *argv[]);

class FixedwingPositionControl
{
public:
	/**
	 * Constructor
	 */
	FixedwingPositionControl();

	/**
	 * Destructor, also kills the sensors task.
	 */
	~FixedwingPositionControl();

	/**
	 * Start the sensors task.
	 *
	 * @return		OK on success.
	 */
	int		start();

private:

	bool		_task_should_exit;		/**< if true, sensor task should exit */
	int		_control_task;			/**< task handle for sensor task */

	int		_global_pos_sub;
	int		_global_set_triplet_sub;
	int		_att_sub;			/**< vehicle attitude subscription */
	int		_attitude_sub;			/**< raw rc channels data subscription */
	int		_airspeed_sub;			/**< airspeed subscription */
	int		_control_mode_sub;			/**< vehicle status subscription */
	int 		_params_sub;			/**< notification of parameter updates */
	int 		_manual_control_sub;		/**< notification of manual control updates */
	int		_accel_sub;			/**< body frame accelerations */

	orb_advert_t	_attitude_sp_pub;		/**< attitude setpoint */
	orb_advert_t	_nav_capabilities_pub;		/**< navigation capabilities publication */

	struct vehicle_attitude_s			_att;			/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s		_att_sp;		/**< vehicle attitude setpoint */
	struct navigation_capabilities_s		_nav_capabilities;	/**< navigation capabilities */
	struct manual_control_setpoint_s		_manual;		/**< r/c channel data */
	struct airspeed_s				_airspeed;		/**< airspeed */
	struct vehicle_control_mode_s				_control_mode;		/**< vehicle status */
	struct vehicle_global_position_s		_global_pos;		/**< global vehicle position */
	struct vehicle_global_position_set_triplet_s	_global_triplet;	/**< triplet of global setpoints */
	struct accel_report				_accel;			/**< body frame accelerations */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	bool		_setpoint_valid;		/**< flag if the position control setpoint is valid */

	/** manual control states */
	float		_seatbelt_hold_heading;		/**< heading the system should hold in seatbelt mode */
	float		_loiter_hold_lat;
	float		_loiter_hold_lon;
	float		_loiter_hold_alt;
	bool		_loiter_hold;

	float		_launch_lat;
	float		_launch_lon;
	float		_launch_alt;
	bool		_launch_valid;

	/* land states */
	/* not in non-abort mode for landing yet */
	bool land_noreturn;
	/* heading hold */
	float target_bearing;

	/* throttle and airspeed states */
	float _airspeed_error;				///< airspeed error to setpoint in m/s
	bool _airspeed_valid;				///< flag if a valid airspeed estimate exists
	uint64_t _airspeed_last_valid;			///< last time airspeed was valid. Used to detect sensor failures
	float _groundspeed_undershoot;			///< ground speed error to min. speed in m/s
	bool _global_pos_valid;				///< global position is valid
	math::Dcm _R_nb;				///< current attitude

	ECL_L1_Pos_Controller				_l1_control;
	TECS						_tecs;

	struct {
		float l1_period;
		float l1_damping;

		float time_const;
		float min_sink_rate;
		float max_sink_rate;
		float max_climb_rate;
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

		float pitch_limit_min;
		float pitch_limit_max;
		float roll_limit;
		float throttle_min;
		float throttle_max;
		float throttle_cruise;

		float throttle_land_max;

		float loiter_hold_radius;
	}		_parameters;			/**< local copies of interesting parameters */

	struct {

		param_t l1_period;
		param_t l1_damping;

		param_t time_const;
		param_t min_sink_rate;
		param_t max_sink_rate;
		param_t max_climb_rate;
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

		param_t pitch_limit_min;
		param_t pitch_limit_max;
		param_t roll_limit;
		param_t throttle_min;
		param_t throttle_max;
		param_t throttle_cruise;

		param_t throttle_land_max;

		param_t loiter_hold_radius;
	}		_parameter_handles;		/**< handles for interesting parameters */


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
	 * Check for changes in vehicle status.
	 */
	void		vehicle_control_mode_poll();

	/**
	 * Check for airspeed updates.
	 */
	bool		vehicle_airspeed_poll();

	/**
	 * Check for position updates.
	 */
	void		vehicle_attitude_poll();

	/**
	 * Check for accel updates.
	 */
	void		vehicle_accel_poll();

	/**
	 * Check for set triplet updates.
	 */
	void		vehicle_setpoint_poll();

	/**
	 * Control position.
	 */
	bool		control_position(const math::Vector2f &global_pos, const math::Vector2f &ground_speed,
					 const struct vehicle_global_position_set_triplet_s &global_triplet);

	float calculate_target_airspeed(float airspeed_demand);
	void calculate_gndspeed_undershoot();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main sensor collection task.
	 */
	void		task_main() __attribute__((noreturn));
};

namespace l1_control
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

FixedwingPositionControl	*g_control;
}

FixedwingPositionControl::FixedwingPositionControl() :

	_task_should_exit(false),
	_control_task(-1),

/* subscriptions */
	_global_pos_sub(-1),
	_global_set_triplet_sub(-1),
	_att_sub(-1),
	_airspeed_sub(-1),
	_control_mode_sub(-1),
	_params_sub(-1),
	_manual_control_sub(-1),

/* publications */
	_attitude_sp_pub(-1),
	_nav_capabilities_pub(-1),

/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "fw l1 control")),
/* states */
	_setpoint_valid(false),
	_loiter_hold(false),
	_airspeed_error(0.0f),
	_airspeed_valid(false),
	_groundspeed_undershoot(0.0f),
	_global_pos_valid(false),
	land_noreturn(false)
{
	_nav_capabilities.turn_distance = 0.0f;

	_parameter_handles.l1_period = param_find("FW_L1_PERIOD");
	_parameter_handles.l1_damping = param_find("FW_L1_DAMPING");
	_parameter_handles.loiter_hold_radius = param_find("FW_LOITER_R");

	_parameter_handles.airspeed_min = param_find("FW_AIRSPD_MIN");
	_parameter_handles.airspeed_trim = param_find("FW_AIRSPD_TRIM");
	_parameter_handles.airspeed_max = param_find("FW_AIRSPD_MAX");

	_parameter_handles.pitch_limit_min = param_find("FW_P_LIM_MIN");
	_parameter_handles.pitch_limit_max = param_find("FW_P_LIM_MAX");
	_parameter_handles.roll_limit = param_find("FW_R_LIM");
	_parameter_handles.throttle_min = param_find("FW_THR_MIN");
	_parameter_handles.throttle_max = param_find("FW_THR_MAX");
	_parameter_handles.throttle_cruise = param_find("FW_THR_CRUISE");
	_parameter_handles.throttle_land_max = param_find("FW_THR_LND_MAX");

	_parameter_handles.time_const = 			param_find("FW_T_TIME_CONST");
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

	/* fetch initial parameter values */
	parameters_update();
}

FixedwingPositionControl::~FixedwingPositionControl()
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

	l1_control::g_control = nullptr;
}

int
FixedwingPositionControl::parameters_update()
{

	/* L1 control parameters */
	param_get(_parameter_handles.l1_damping, &(_parameters.l1_damping));
	param_get(_parameter_handles.l1_period, &(_parameters.l1_period));
	param_get(_parameter_handles.loiter_hold_radius, &(_parameters.loiter_hold_radius));

	param_get(_parameter_handles.airspeed_min, &(_parameters.airspeed_min));
	param_get(_parameter_handles.airspeed_trim, &(_parameters.airspeed_trim));
	param_get(_parameter_handles.airspeed_max, &(_parameters.airspeed_max));

	param_get(_parameter_handles.pitch_limit_min, &(_parameters.pitch_limit_min));
	param_get(_parameter_handles.pitch_limit_max, &(_parameters.pitch_limit_max));
	param_get(_parameter_handles.roll_limit, &(_parameters.roll_limit));
	param_get(_parameter_handles.throttle_min, &(_parameters.throttle_min));
	param_get(_parameter_handles.throttle_max, &(_parameters.throttle_max));
	param_get(_parameter_handles.throttle_cruise, &(_parameters.throttle_cruise));

	param_get(_parameter_handles.throttle_land_max, &(_parameters.throttle_land_max));

	param_get(_parameter_handles.time_const, &(_parameters.time_const));
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

	_l1_control.set_l1_damping(_parameters.l1_damping);
	_l1_control.set_l1_period(_parameters.l1_period);
	_l1_control.set_l1_roll_limit(math::radians(_parameters.roll_limit));

	_tecs.set_time_const(_parameters.time_const);
	_tecs.set_min_sink_rate(_parameters.min_sink_rate);
	_tecs.set_max_sink_rate(_parameters.max_sink_rate);
	_tecs.set_throttle_damp(_parameters.throttle_damp);
	_tecs.set_integrator_gain(_parameters.integrator_gain);
	_tecs.set_vertical_accel_limit(_parameters.vertical_accel_limit);
	_tecs.set_height_comp_filter_omega(_parameters.height_comp_filter_omega);
	_tecs.set_speed_comp_filter_omega(_parameters.speed_comp_filter_omega);
	_tecs.set_roll_throttle_compensation(math::radians(_parameters.roll_throttle_compensation));
	_tecs.set_speed_weight(_parameters.speed_weight);
	_tecs.set_pitch_damping(_parameters.pitch_damping);
	_tecs.set_indicated_airspeed_min(_parameters.airspeed_min);
	_tecs.set_indicated_airspeed_max(_parameters.airspeed_max);
	_tecs.set_max_climb_rate(_parameters.max_climb_rate);

	/* sanity check parameters */
	if (_parameters.airspeed_max < _parameters.airspeed_min ||
	    _parameters.airspeed_max < 5.0f ||
	    _parameters.airspeed_min > 100.0f ||
	    _parameters.airspeed_trim < _parameters.airspeed_min ||
	    _parameters.airspeed_trim > _parameters.airspeed_max) {
		warnx("error: airspeed parameters invalid");
		return 1;
	}

	return OK;
}

void
FixedwingPositionControl::vehicle_control_mode_poll()
{
	bool vstatus_updated;

	/* Check HIL state if vehicle status has changed */
	orb_check(_control_mode_sub, &vstatus_updated);

	if (vstatus_updated) {

		bool was_armed = _control_mode.flag_armed;

		orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);

		if (!was_armed && _control_mode.flag_armed) {
			_launch_lat = _global_pos.lat / 1e7f;
			_launch_lon = _global_pos.lon / 1e7f;
			_launch_alt = _global_pos.alt;
			_launch_valid = true;
		}
	}
}

bool
FixedwingPositionControl::vehicle_airspeed_poll()
{
	/* check if there is an airspeed update or if it timed out */
	bool airspeed_updated;
	orb_check(_airspeed_sub, &airspeed_updated);

	if (airspeed_updated) {
		orb_copy(ORB_ID(airspeed), _airspeed_sub, &_airspeed);
		_airspeed_valid = true;
		_airspeed_last_valid = hrt_absolute_time();

	} else {

		/* no airspeed updates for one second */
		if (_airspeed_valid && (hrt_absolute_time() - _airspeed_last_valid) > 1e6) {
			_airspeed_valid = false;
		}
	}

	/* update TECS state */
	_tecs.enable_airspeed(_airspeed_valid);

	return airspeed_updated;
}

void
FixedwingPositionControl::vehicle_attitude_poll()
{
	/* check if there is a new position */
	bool att_updated;
	orb_check(_att_sub, &att_updated);

	if (att_updated) {
		orb_copy(ORB_ID(vehicle_attitude), _att_sub, &_att);

		/* set rotation matrix */
		for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++)
				_R_nb(i, j) = _att.R[i][j];
	}
}

void
FixedwingPositionControl::vehicle_accel_poll()
{
	/* check if there is a new position */
	bool accel_updated;
	orb_check(_accel_sub, &accel_updated);

	if (accel_updated) {
		orb_copy(ORB_ID(sensor_accel), _accel_sub, &_accel);
	}
}

void
FixedwingPositionControl::vehicle_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool global_sp_updated;
	orb_check(_global_set_triplet_sub, &global_sp_updated);

	if (global_sp_updated) {
		orb_copy(ORB_ID(vehicle_global_position_set_triplet), _global_set_triplet_sub, &_global_triplet);
		_setpoint_valid = true;
	}
}

void
FixedwingPositionControl::task_main_trampoline(int argc, char *argv[])
{
	l1_control::g_control->task_main();
}

float
FixedwingPositionControl::calculate_target_airspeed(float airspeed_demand)
{
	float airspeed;

	if (_airspeed_valid) {
		airspeed = _airspeed.true_airspeed_m_s;

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
FixedwingPositionControl::calculate_gndspeed_undershoot()
{

	if (_global_pos_valid) {
		/* get ground speed vector */
		math::Vector2f ground_speed_vector(_global_pos.vx, _global_pos.vy);

		/* rotate with current attitude */
		math::Vector2f yaw_vector(_R_nb(0, 0), _R_nb(1, 0));
		yaw_vector.normalize();
		float ground_speed_body = yaw_vector * ground_speed_vector;

		/*
		 * Ground speed undershoot is the amount of ground velocity not reached
		 * by the plane. Consequently it is zero if airspeed is >= min ground speed
		 * and positive if airspeed < min ground speed.
		 *
		 * This error value ensures that a plane (as long as its throttle capability is
		 * not exceeded) travels towards a waypoint (and is not pushed more and more away
		 * by wind). Not countering this would lead to a fly-away.
		 */
		_groundspeed_undershoot = math::max(_parameters.airspeed_min - ground_speed_body, 0.0f);

	} else {
		_groundspeed_undershoot = 0;
	}
}

bool
FixedwingPositionControl::control_position(const math::Vector2f &current_position, const math::Vector2f &ground_speed,
		const struct vehicle_global_position_set_triplet_s &global_triplet)
{
	bool setpoint = true;

	calculate_gndspeed_undershoot();

	float eas2tas = 1.0f; // XXX calculate actual number based on current measurements

	// XXX re-visit
	float baro_altitude = _global_pos.alt;

	/* filter speed and altitude for controller */
	math::Vector3 accel_body(_accel.x, _accel.y, _accel.z);
	math::Vector3 accel_earth = _R_nb.transpose() * accel_body;

	_tecs.update_50hz(baro_altitude, _airspeed.indicated_airspeed_m_s, _R_nb, accel_body, accel_earth);
	float altitude_error = _global_triplet.current.altitude - _global_pos.alt;

	/* no throttle limit as default */
	float throttle_max = 1.0f;

	/* AUTONOMOUS FLIGHT */

	// XXX this should only execute if auto AND safety off (actuators active),
	// else integrators should be constantly reset.
	if (_control_mode.flag_control_position_enabled) {

		/* get circle mode */
		bool was_circle_mode = _l1_control.circle_mode();

		/* restore speed weight, in case changed intermittently (e.g. in landing handling) */
		_tecs.set_speed_weight(_parameters.speed_weight);

		/* execute navigation once we have a setpoint */
		if (_setpoint_valid) {

			/* current waypoint (the one currently heading for) */
			math::Vector2f next_wp(global_triplet.current.lat / 1e7f, global_triplet.current.lon / 1e7f);

			/* previous waypoint */
			math::Vector2f prev_wp;

			if (global_triplet.previous_valid) {
				prev_wp.setX(global_triplet.previous.lat / 1e7f);
				prev_wp.setY(global_triplet.previous.lon / 1e7f);

			} else {
				/*
				 * No valid previous waypoint, go for the current wp.
				 * This is automatically handled by the L1 library.
				 */
				prev_wp.setX(global_triplet.current.lat / 1e7f);
				prev_wp.setY(global_triplet.current.lon / 1e7f);

			}

			// XXX add RTL switch
			if (global_triplet.current.nav_cmd == NAV_CMD_RETURN_TO_LAUNCH && _launch_valid) {

				math::Vector2f rtl_pos(_launch_lat, _launch_lon);

				_l1_control.navigate_waypoints(rtl_pos, rtl_pos, current_position, ground_speed);
				_att_sp.roll_body = _l1_control.nav_roll();
				_att_sp.yaw_body = _l1_control.nav_bearing();

				_tecs.update_pitch_throttle(_R_nb, _att.pitch, _global_pos.alt, _launch_alt, calculate_target_airspeed(_parameters.airspeed_trim),
							    _airspeed.indicated_airspeed_m_s, eas2tas,
							    false, math::radians(_parameters.pitch_limit_min),
							    _parameters.throttle_min, _parameters.throttle_max, _parameters.throttle_cruise,
							    math::radians(_parameters.pitch_limit_min), math::radians(_parameters.pitch_limit_max));

				// XXX handle case when having arrived at home (loiter)

			} else if (global_triplet.current.nav_cmd == NAV_CMD_WAYPOINT) {
				/* waypoint is a plain navigation waypoint */
				_l1_control.navigate_waypoints(prev_wp, next_wp, current_position, ground_speed);
				_att_sp.roll_body = _l1_control.nav_roll();
				_att_sp.yaw_body = _l1_control.nav_bearing();

				_tecs.update_pitch_throttle(_R_nb, _att.pitch, _global_pos.alt, _global_triplet.current.altitude, calculate_target_airspeed(_parameters.airspeed_trim),
							    _airspeed.indicated_airspeed_m_s, eas2tas,
							    false, math::radians(_parameters.pitch_limit_min),
							    _parameters.throttle_min, _parameters.throttle_max, _parameters.throttle_cruise,
							    math::radians(_parameters.pitch_limit_min), math::radians(_parameters.pitch_limit_max));

			} else if (global_triplet.current.nav_cmd == NAV_CMD_LOITER_TURN_COUNT ||
				   global_triplet.current.nav_cmd == NAV_CMD_LOITER_TIME_LIMIT ||
				   global_triplet.current.nav_cmd == NAV_CMD_LOITER_UNLIMITED) {

				/* waypoint is a loiter waypoint */
				_l1_control.navigate_loiter(next_wp, current_position, global_triplet.current.loiter_radius,
							  global_triplet.current.loiter_direction, ground_speed);
				_att_sp.roll_body = _l1_control.nav_roll();
				_att_sp.yaw_body = _l1_control.nav_bearing();

				_tecs.update_pitch_throttle(_R_nb, _att.pitch, _global_pos.alt, _global_triplet.current.altitude, calculate_target_airspeed(_parameters.airspeed_trim),
							    _airspeed.indicated_airspeed_m_s, eas2tas,
							    false, math::radians(_parameters.pitch_limit_min),
							    _parameters.throttle_min, _parameters.throttle_max, _parameters.throttle_cruise,
							    math::radians(_parameters.pitch_limit_min), math::radians(_parameters.pitch_limit_max));

			} else if (global_triplet.current.nav_cmd == NAV_CMD_LAND) {

				/* switch to heading hold for the last meters, continue heading hold after */

				float wp_distance = get_distance_to_next_waypoint(prev_wp.getX(), prev_wp.getY(), current_position.getX(), current_position.getY());
				//warnx("wp dist: %d, alt err: %d, noret: %s", (int)wp_distance, (int)altitude_error, (land_noreturn) ? "YES" : "NO");
				if (wp_distance < 15.0f || land_noreturn) {

					/* heading hold, along the line connecting this and the last waypoint */
					

					// if (global_triplet.previous_valid) {
					// 	target_bearing = get_bearing_to_next_waypoint(prev_wp.getX(), prev_wp.getY(), next_wp.getX(), next_wp.getY());
					// } else {

					if (!land_noreturn)
						target_bearing = _att.yaw;
					//}

					warnx("NORET: %d, target_bearing: %d, yaw: %d", (int)land_noreturn, (int)math::degrees(target_bearing), (int)math::degrees(_att.yaw));

					_l1_control.navigate_heading(target_bearing, _att.yaw, ground_speed);

					if (altitude_error > -5.0f)
						land_noreturn = true;

				} else {

					/* normal navigation */
					_l1_control.navigate_waypoints(prev_wp, next_wp, current_position, ground_speed);
				}

				/* do not go down too early */
				if (wp_distance > 50.0f) {
					altitude_error = (_global_triplet.current.altitude + 25.0f) - _global_pos.alt;
				}


				_att_sp.roll_body = _l1_control.nav_roll();
				_att_sp.yaw_body = _l1_control.nav_bearing();

				/* apply minimum pitch (flare) and limit roll if close to touch down, altitude error is negative (going down) */
				// XXX this could make a great param

				float flare_angle_rad = math::radians(10.0f);//math::radians(global_triplet.current.param1)
				float land_pitch_min = math::radians(5.0f);
				float throttle_land = _parameters.throttle_min + (_parameters.throttle_max - _parameters.throttle_min) * 0.1f;
				float airspeed_land = _parameters.airspeed_min;
				float airspeed_approach = (_parameters.airspeed_min + _parameters.airspeed_trim) / 2.0f;

				if (altitude_error > -4.0f) {

					/* land with minimal speed */

					/* force TECS to only control speed with pitch, altitude is only implicitely controlled now */
					_tecs.set_speed_weight(2.0f);

					_tecs.update_pitch_throttle(_R_nb, _att.pitch, _global_pos.alt, _global_triplet.current.altitude, calculate_target_airspeed(airspeed_land),
								    _airspeed.indicated_airspeed_m_s, eas2tas,
								    false, flare_angle_rad,
								    0.0f, _parameters.throttle_max, throttle_land,
								    math::radians(-10.0f), math::radians(15.0f));

					/* kill the throttle if param requests it */
					throttle_max = math::min(throttle_max, _parameters.throttle_land_max);

					/* limit roll motion to prevent wings from touching the ground first */
					_att_sp.roll_body = math::constrain(_att_sp.roll_body, math::radians(-10.0f), math::radians(10.0f));

				} else if (wp_distance < 60.0f && altitude_error > -20.0f) {

					/* minimize speed to approach speed */

					_tecs.update_pitch_throttle(_R_nb, _att.pitch, _global_pos.alt, _global_triplet.current.altitude, calculate_target_airspeed(airspeed_approach),
								    _airspeed.indicated_airspeed_m_s, eas2tas,
								    false, flare_angle_rad,
								    _parameters.throttle_min, _parameters.throttle_max, _parameters.throttle_cruise,
								    math::radians(_parameters.pitch_limit_min), math::radians(_parameters.pitch_limit_max));

				} else {

					/* normal cruise speed */

					_tecs.update_pitch_throttle(_R_nb, _att.pitch, _global_pos.alt, _global_triplet.current.altitude, calculate_target_airspeed(_parameters.airspeed_trim),
								    _airspeed.indicated_airspeed_m_s, eas2tas,
								    false, math::radians(_parameters.pitch_limit_min),
								    _parameters.throttle_min, _parameters.throttle_max, _parameters.throttle_cruise,
								    math::radians(_parameters.pitch_limit_min), math::radians(_parameters.pitch_limit_max));
				}

			} else if (global_triplet.current.nav_cmd == NAV_CMD_TAKEOFF) {

				_l1_control.navigate_waypoints(prev_wp, next_wp, current_position, ground_speed);
				_att_sp.roll_body = _l1_control.nav_roll();
				_att_sp.yaw_body = _l1_control.nav_bearing();

				/* apply minimum pitch and limit roll if target altitude is not within 10 meters */
				if (altitude_error > 10.0f) {

					/* enforce a minimum of 10 degrees pitch up on takeoff, or take parameter */
					_tecs.update_pitch_throttle(_R_nb, _att.pitch, _global_pos.alt, _global_triplet.current.altitude, calculate_target_airspeed(_parameters.airspeed_min),
								    _airspeed.indicated_airspeed_m_s, eas2tas,
								    true, math::max(math::radians(global_triplet.current.param1), math::radians(10.0f)),
								    _parameters.throttle_min, _parameters.throttle_max, _parameters.throttle_cruise,
								    math::radians(_parameters.pitch_limit_min), math::radians(_parameters.pitch_limit_max));

					/* limit roll motion to ensure enough lift */
					_att_sp.roll_body = math::constrain(_att_sp.roll_body, math::radians(-15.0f), math::radians(15.0f));

				} else {

					_tecs.update_pitch_throttle(_R_nb, _att.pitch, _global_pos.alt, _global_triplet.current.altitude, calculate_target_airspeed(_parameters.airspeed_trim),
								    _airspeed.indicated_airspeed_m_s, eas2tas,
								    false, math::radians(_parameters.pitch_limit_min),
								    _parameters.throttle_min, _parameters.throttle_max, _parameters.throttle_cruise,
								    math::radians(_parameters.pitch_limit_min), math::radians(_parameters.pitch_limit_max));
				}
			}

			// warnx("nav bearing: %8.4f bearing err: %8.4f target bearing: %8.4f", (double)_l1_control.nav_bearing(),
			//       (double)_l1_control.bearing_error(), (double)_l1_control.target_bearing());
			// warnx("prev wp: %8.4f/%8.4f, next wp: %8.4f/%8.4f prev:%s", (double)prev_wp.getX(), (double)prev_wp.getY(),
			//       (double)next_wp.getX(), (double)next_wp.getY(), (global_triplet.previous_valid) ? "valid" : "invalid");

			// XXX at this point we always want no loiter hold if a
			// mission is active
			_loiter_hold = false;

		} else if (_control_mode.flag_armed) {

			/* hold position, but only if armed, climb 20m in case this is engaged on ground level */

			// XXX rework with smarter state machine

			if (!_loiter_hold) {
				_loiter_hold_lat = _global_pos.lat / 1e7f;
				_loiter_hold_lon = _global_pos.lon / 1e7f;
				_loiter_hold_alt = _global_pos.alt + 25.0f;
				_loiter_hold = true;
			}

			altitude_error = _loiter_hold_alt - _global_pos.alt;

			math::Vector2f loiter_hold_pos(_loiter_hold_lat, _loiter_hold_lon);

			/* loiter around current position */
			_l1_control.navigate_loiter(loiter_hold_pos, current_position, _parameters.loiter_hold_radius,
						  1, ground_speed);
			_att_sp.roll_body = _l1_control.nav_roll();
			_att_sp.yaw_body = _l1_control.nav_bearing();

			/* climb with full throttle if the altitude error is bigger than 5 meters */
			bool climb_out = (altitude_error > 3);

			float min_pitch;

			if (climb_out) {
				min_pitch = math::radians(20.0f);

			} else {
				min_pitch = math::radians(_parameters.pitch_limit_min);
			}

			_tecs.update_pitch_throttle(_R_nb, _att.pitch, _global_pos.alt, _loiter_hold_alt, calculate_target_airspeed(_parameters.airspeed_trim),
						    _airspeed.indicated_airspeed_m_s, eas2tas,
						    climb_out, min_pitch,
						    _parameters.throttle_min, _parameters.throttle_max, _parameters.throttle_cruise,
						    math::radians(_parameters.pitch_limit_min), math::radians(_parameters.pitch_limit_max));

			if (climb_out) {
				/* limit roll motion to ensure enough lift */
				_att_sp.roll_body = math::constrain(_att_sp.roll_body, math::radians(-15.0f), math::radians(15.0f));
			}
		}

		/* reset land state */
		if (global_triplet.current.nav_cmd != NAV_CMD_LAND) {
			land_noreturn = false;
		}

		if (was_circle_mode && !_l1_control.circle_mode()) {
			/* just kicked out of loiter, reset roll integrals */
			_att_sp.roll_reset_integral = true;
		}

	} else if (0/* easy mode enabled */) {

		/** EASY FLIGHT **/

		if (0/* switched from another mode to easy */) {
			_seatbelt_hold_heading = _att.yaw;
		}

		if (0/* easy on and manual control yaw non-zero */) {
			_seatbelt_hold_heading = _att.yaw + _manual.yaw;
		}

		/* climb out control */
		bool climb_out = false;

		/* user wants to climb out */
		if (_manual.pitch > 0.3f && _manual.throttle > 0.8f) {
			climb_out = true;
		}

		/* if in seatbelt mode, set airspeed based on manual control */

		// XXX check if ground speed undershoot should be applied here
		float seatbelt_airspeed = _parameters.airspeed_min +
					  (_parameters.airspeed_max - _parameters.airspeed_min) *
					  _manual.throttle;

		_l1_control.navigate_heading(_seatbelt_hold_heading, _att.yaw, ground_speed);
		_att_sp.roll_body = _l1_control.nav_roll();
		_att_sp.yaw_body = _l1_control.nav_bearing();
		_tecs.update_pitch_throttle(_R_nb, _att.pitch, _global_pos.alt, _global_pos.alt + _manual.pitch * 2.0f,
					    seatbelt_airspeed,
					    _airspeed.indicated_airspeed_m_s, eas2tas,
					    false, _parameters.pitch_limit_min,
					    _parameters.throttle_min, _parameters.throttle_max, _parameters.throttle_cruise,
					    _parameters.pitch_limit_min, _parameters.pitch_limit_max);

	} else if (0/* seatbelt mode enabled */) {

		/** SEATBELT FLIGHT **/

		if (0/* switched from another mode to seatbelt */) {
			_seatbelt_hold_heading = _att.yaw;
		}

		if (0/* seatbelt on and manual control yaw non-zero */) {
			_seatbelt_hold_heading = _att.yaw + _manual.yaw;
		}

		/* if in seatbelt mode, set airspeed based on manual control */

		// XXX check if ground speed undershoot should be applied here
		float seatbelt_airspeed = _parameters.airspeed_min +
					  (_parameters.airspeed_max - _parameters.airspeed_min) *
					  _manual.throttle;

		/* user switched off throttle */
		if (_manual.throttle < 0.1f) {
			throttle_max = 0.0f;
			/* switch to pure pitch based altitude control, give up speed */
			_tecs.set_speed_weight(0.0f);
		}

		/* climb out control */
		bool climb_out = false;

		/* user wants to climb out */
		if (_manual.pitch > 0.3f && _manual.throttle > 0.8f) {
			climb_out = true;
		}

		_l1_control.navigate_heading(_seatbelt_hold_heading, _att.yaw, ground_speed);
		_att_sp.roll_body =	_manual.roll;
		_att_sp.yaw_body =	_manual.yaw;
		_tecs.update_pitch_throttle(_R_nb, _att.pitch, _global_pos.alt, _global_pos.alt + _manual.pitch * 2.0f,
					    seatbelt_airspeed,
					    _airspeed.indicated_airspeed_m_s, eas2tas,
					    climb_out, _parameters.pitch_limit_min,
					    _parameters.throttle_min, _parameters.throttle_max, _parameters.throttle_cruise,
					    _parameters.pitch_limit_min, _parameters.pitch_limit_max);

	} else {

		/** MANUAL FLIGHT **/

		/* no flight mode applies, do not publish an attitude setpoint */
		setpoint = false;
	}

	_att_sp.pitch_body = _tecs.get_pitch_demand();
	_att_sp.thrust = math::min(_tecs.get_throttle_demand(), throttle_max);

	return setpoint;
}

void
FixedwingPositionControl::task_main()
{

	/* inform about start */
	warnx("Initializing..");
	fflush(stdout);

	/*
	 * do subscriptions
	 */
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_global_set_triplet_sub = orb_subscribe(ORB_ID(vehicle_global_position_set_triplet));
	_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_accel_sub = orb_subscribe(ORB_ID(sensor_accel));
	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_airspeed_sub = orb_subscribe(ORB_ID(airspeed));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_control_sub = orb_subscribe(ORB_ID(manual_control_setpoint));

	/* rate limit vehicle status updates to 5Hz */
	orb_set_interval(_control_mode_sub, 200);
	/* rate limit position updates to 50 Hz */
	orb_set_interval(_global_pos_sub, 20);

	/* abort on a nonzero return value from the parameter init */
	if (parameters_update()) {
		/* parameter setup went wrong, abort */
		warnx("aborting startup due to errors.");
		_task_should_exit = true;
	}

	/* wakeup source(s) */
	struct pollfd fds[2];

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _global_pos_sub;
	fds[1].events = POLLIN;

	while (!_task_should_exit) {

		/* wait for up to 500ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit, etc. */
		if (pret == 0)
			continue;

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		perf_begin(_loop_perf);

		/* check vehicle status for changes to publication state */
		vehicle_control_mode_poll();

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


			static uint64_t last_run = 0;
			float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too large deltaT's */
			if (deltaT > 1.0f)
				deltaT = 0.01f;

			/* load local copies */
			orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);

			// XXX add timestamp check
			_global_pos_valid = true;

			vehicle_attitude_poll();
			vehicle_setpoint_poll();
			vehicle_accel_poll();
			vehicle_airspeed_poll();
			// vehicle_baro_poll();

			math::Vector2f ground_speed(_global_pos.vx, _global_pos.vy);
			math::Vector2f current_position(_global_pos.lat / 1e7f, _global_pos.lon / 1e7f);

			/*
			 * Attempt to control position, on success (= sensors present and not in manual mode),
			 * publish setpoint.
			 */
			if (control_position(current_position, ground_speed, _global_triplet)) {
				_att_sp.timestamp = hrt_absolute_time();

				/* lazily publish the setpoint only once available */
				if (_attitude_sp_pub > 0) {
					/* publish the attitude setpoint */
					orb_publish(ORB_ID(vehicle_attitude_setpoint), _attitude_sp_pub, &_att_sp);

				} else {
					/* advertise and publish */
					_attitude_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_att_sp);
				}

				float turn_distance = _l1_control.switch_distance(_global_triplet.current.turn_distance_xy);

				/* lazily publish navigation capabilities */
				if (turn_distance != _nav_capabilities.turn_distance && turn_distance > 0) {

					/* set new turn distance */
					_nav_capabilities.turn_distance = turn_distance;

					if (_nav_capabilities_pub > 0) {
						orb_publish(ORB_ID(navigation_capabilities), _nav_capabilities_pub, &_nav_capabilities);
					} else {
						_nav_capabilities_pub = orb_advertise(ORB_ID(navigation_capabilities), &_nav_capabilities);
					}
				}

			}

		}

		perf_end(_loop_perf);
	}

	warnx("exiting.\n");

	_control_task = -1;
	_exit(0);
}

int
FixedwingPositionControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = task_spawn_cmd("fw_pos_control_l1",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_MAX - 5,
				       4048,
				       (main_t)&FixedwingPositionControl::task_main_trampoline,
				       nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int fw_pos_control_l1_main(int argc, char *argv[])
{
	if (argc < 1)
		errx(1, "usage: fw_pos_control_l1 {start|stop|status}");

	if (!strcmp(argv[1], "start")) {

		if (l1_control::g_control != nullptr)
			errx(1, "already running");

		l1_control::g_control = new FixedwingPositionControl;

		if (l1_control::g_control == nullptr)
			errx(1, "alloc failed");

		if (OK != l1_control::g_control->start()) {
			delete l1_control::g_control;
			l1_control::g_control = nullptr;
			err(1, "start failed");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (l1_control::g_control == nullptr)
			errx(1, "not running");

		delete l1_control::g_control;
		l1_control::g_control = nullptr;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (l1_control::g_control) {
			errx(0, "running");

		} else {
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	return 1;
}
