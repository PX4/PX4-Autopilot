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
 * @author Thomas Gubler <thomasgubler@gmail.com>
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
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/navigation_capabilities.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/parameter_update.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/pid/pid.h>
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>
#include <mavlink/mavlink_log.h>
#include <launchdetection/LaunchDetector.h>
#include <ecl/l1/ecl_l1_pos_controller.h>
#include <external_lgpl/tecs/tecs.h>
#include "landingslope.h"
#include "mtecs/mTecs.h"

static int	_control_task = -1;			/**< task handle for sensor task */


/**
 * L1 control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int fw_pos_control_l1_main(int argc, char *argv[]);

using namespace launchdetection;

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
	int		_mavlink_fd;

	bool		_task_should_exit;		/**< if true, sensor task should exit */
	bool		_task_running;			/**< if true, task is running in its mainloop */

	int		_global_pos_sub;
	int		_pos_sp_triplet_sub;
	int		_att_sub;			/**< vehicle attitude subscription */
	int		_airspeed_sub;			/**< airspeed subscription */
	int		_control_mode_sub;		/**< control mode subscription */
	int		_vehicle_status_sub;		/**< vehicle status subscription */
	int 		_params_sub;			/**< notification of parameter updates */
	int 		_manual_control_sub;		/**< notification of manual control updates */
	int		_sensor_combined_sub;		/**< for body frame accelerations */

	orb_advert_t	_attitude_sp_pub;		/**< attitude setpoint */
	orb_advert_t	_tecs_status_pub;		/**< TECS status publication */
	orb_advert_t	_nav_capabilities_pub;		/**< navigation capabilities publication */

	struct vehicle_attitude_s			_att;				/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s		_att_sp;			/**< vehicle attitude setpoint */
	struct navigation_capabilities_s		_nav_capabilities;		/**< navigation capabilities */
	struct manual_control_setpoint_s		_manual;			/**< r/c channel data */
	struct airspeed_s				_airspeed;			/**< airspeed */
	struct vehicle_control_mode_s			_control_mode;			/**< control mode */
	struct vehicle_status_s				_vehicle_status;		/**< vehicle status */
	struct vehicle_global_position_s		_global_pos;			/**< global vehicle position */
	struct position_setpoint_triplet_s		_pos_sp_triplet;		/**< triplet of mission items */
	struct sensor_combined_s			_sensor_combined;		/**< for body frame accelerations */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	/* land states */
	bool land_noreturn_horizontal;
	bool land_noreturn_vertical;
	bool land_stayonground;
	bool land_motor_lim;
	bool land_onslope;
	bool land_useterrain;

	/* takeoff/launch states */
	LaunchDetectionResult launch_detection_state;

	bool last_manual;				///< true if the last iteration was in manual mode (used to determine when a reset is needed)

	/* Landingslope object */
	Landingslope landingslope;
	float flare_curve_alt_rel_last;

	/* heading hold */
	float target_bearing;

	/* Launch detection */
	launchdetection::LaunchDetector launchDetector;

	/* throttle and airspeed states */
	float _airspeed_error;				///< airspeed error to setpoint in m/s
	bool _airspeed_valid;				///< flag if a valid airspeed estimate exists
	uint64_t _airspeed_last_valid;			///< last time airspeed was valid. Used to detect sensor failures
	float _groundspeed_undershoot;			///< ground speed error to min. speed in m/s
	bool _global_pos_valid;				///< global position is valid
	math::Matrix<3, 3> _R_nb;			///< current attitude

	ECL_L1_Pos_Controller				_l1_control;
	TECS						_tecs;
	fwPosctrl::mTecs				_mTecs;
	bool						_was_pos_control_mode;

	struct {
		float l1_period;
		float l1_damping;

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

		float pitch_limit_min;
		float pitch_limit_max;
		float roll_limit;
		float throttle_min;
		float throttle_max;
		float throttle_cruise;
		float throttle_slew_max;

		float throttle_land_max;

		float land_slope_angle;
		float land_H1_virt;
		float land_flare_alt_relative;
		float land_thrust_lim_alt_relative;
		float land_heading_hold_horizontal_distance;
		float land_flare_pitch_min_deg;
		float land_flare_pitch_max_deg;
		int land_use_terrain_estimate;

	}		_parameters;			/**< local copies of interesting parameters */

	struct {

		param_t l1_period;
		param_t l1_damping;

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

		param_t pitch_limit_min;
		param_t pitch_limit_max;
		param_t roll_limit;
		param_t throttle_min;
		param_t throttle_max;
		param_t throttle_cruise;
		param_t throttle_slew_max;

		param_t throttle_land_max;

		param_t land_slope_angle;
		param_t land_H1_virt;
		param_t land_flare_alt_relative;
		param_t land_thrust_lim_alt_relative;
		param_t land_heading_hold_horizontal_distance;
		param_t land_flare_pitch_min_deg;
		param_t land_flare_pitch_max_deg;
		param_t land_use_terrain_estimate;

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
	 * Check for changes in control mode
	 */
	void		vehicle_control_mode_poll();

	/**
	 * Check for changes in vehicle status.
	 */
	void		vehicle_status_poll();

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
	void		vehicle_sensor_combined_poll();

	/**
	 * Check for set triplet updates.
	 */
	void		vehicle_setpoint_poll();

	/**
	 * Publish navigation capabilities
	 */
	void navigation_capabilities_publish();

	/**
	 * Return the terrain estimate during landing: uses the wp altitude value or the terrain estimate if available
	 */
	float get_terrain_altitude_landing(float land_setpoint_alt, const struct vehicle_global_position_s &global_pos);

	/**
	 * Control position.
	 */
	bool		control_position(const math::Vector<2> &global_pos, const math::Vector<3> &ground_speed,
					 const struct position_setpoint_triplet_s &_pos_sp_triplet);

	float calculate_target_airspeed(float airspeed_demand);
	void calculate_gndspeed_undershoot(const math::Vector<2> &current_position, const math::Vector<2> &ground_speed_2d, const struct position_setpoint_triplet_s &pos_sp_triplet);

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main sensor collection task.
	 */
	void		task_main();

	/*
	 * Reset takeoff state
	 */
	void reset_takeoff_state();

	/*
	 * Reset landing state
	 */
	void reset_landing_state();

	/*
	 * Call TECS : a wrapper function to call one of the TECS implementations (mTECS is called only if enabled via parameter)
	 * XXX need to clean up/remove this function once mtecs fully replaces TECS
	 */
	void tecs_update_pitch_throttle(float alt_sp, float v_sp, float eas2tas,
			float pitch_min_rad, float pitch_max_rad,
			float throttle_min, float throttle_max, float throttle_cruise,
			bool climbout_mode, float climbout_pitch_min_rad,
			float altitude,
			const math::Vector<3> &ground_speed,
			tecs_mode mode = TECS_MODE_NORMAL,
			bool pitch_max_special = false);

};

namespace l1_control
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

FixedwingPositionControl	*g_control = nullptr;
}

FixedwingPositionControl::FixedwingPositionControl() :

	_mavlink_fd(-1),
	_task_should_exit(false),
	_task_running(false),

/* subscriptions */
	_global_pos_sub(-1),
	_pos_sp_triplet_sub(-1),
	_att_sub(-1),
	_airspeed_sub(-1),
	_control_mode_sub(-1),
	_vehicle_status_sub(-1),
	_params_sub(-1),
	_manual_control_sub(-1),
	_sensor_combined_sub(-1),

/* publications */
	_attitude_sp_pub(-1),
	_tecs_status_pub(-1),
	_nav_capabilities_pub(-1),

/* states */
	_att(),
	_att_sp(),
	_nav_capabilities(),
	_manual(),
	_airspeed(),
	_control_mode(),
	_vehicle_status(),
	_global_pos(),
	_pos_sp_triplet(),
	_sensor_combined(),

/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "fw l1 control")),

	land_noreturn_horizontal(false),
	land_noreturn_vertical(false),
	land_stayonground(false),
	land_motor_lim(false),
	land_onslope(false),
	land_useterrain(false),
	launch_detection_state(LAUNCHDETECTION_RES_NONE),
	last_manual(false),
	landingslope(),
	flare_curve_alt_rel_last(0.0f),
	target_bearing(0.0f),
	launchDetector(),
	_airspeed_error(0.0f),
	_airspeed_valid(false),
	_airspeed_last_valid(0),
	_groundspeed_undershoot(0.0f),
	_global_pos_valid(false),
	_l1_control(),
	_mTecs(),
	_was_pos_control_mode(false)
{
	_nav_capabilities.turn_distance = 0.0f;

	_parameter_handles.l1_period = param_find("FW_L1_PERIOD");
	_parameter_handles.l1_damping = param_find("FW_L1_DAMPING");

	_parameter_handles.airspeed_min = param_find("FW_AIRSPD_MIN");
	_parameter_handles.airspeed_trim = param_find("FW_AIRSPD_TRIM");
	_parameter_handles.airspeed_max = param_find("FW_AIRSPD_MAX");

	_parameter_handles.pitch_limit_min = param_find("FW_P_LIM_MIN");
	_parameter_handles.pitch_limit_max = param_find("FW_P_LIM_MAX");
	_parameter_handles.roll_limit = param_find("FW_R_LIM");
	_parameter_handles.throttle_min = param_find("FW_THR_MIN");
	_parameter_handles.throttle_max = param_find("FW_THR_MAX");
	_parameter_handles.throttle_slew_max = param_find("FW_THR_SLEW_MAX");
	_parameter_handles.throttle_cruise = param_find("FW_THR_CRUISE");
	_parameter_handles.throttle_land_max = param_find("FW_THR_LND_MAX");

	_parameter_handles.land_slope_angle = param_find("FW_LND_ANG");
	_parameter_handles.land_H1_virt = param_find("FW_LND_HVIRT");
	_parameter_handles.land_flare_alt_relative = param_find("FW_LND_FLALT");
	_parameter_handles.land_thrust_lim_alt_relative = param_find("FW_LND_TLALT");
	_parameter_handles.land_heading_hold_horizontal_distance = param_find("FW_LND_HHDIST");
	_parameter_handles.land_flare_pitch_min_deg = param_find("FW_FLARE_PMIN");
	_parameter_handles.land_flare_pitch_max_deg = param_find("FW_FLARE_PMAX");
	_parameter_handles.land_use_terrain_estimate= param_find("FW_LND_USETER");

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

	param_get(_parameter_handles.airspeed_min, &(_parameters.airspeed_min));
	param_get(_parameter_handles.airspeed_trim, &(_parameters.airspeed_trim));
	param_get(_parameter_handles.airspeed_max, &(_parameters.airspeed_max));

	param_get(_parameter_handles.pitch_limit_min, &(_parameters.pitch_limit_min));
	param_get(_parameter_handles.pitch_limit_max, &(_parameters.pitch_limit_max));
	param_get(_parameter_handles.roll_limit, &(_parameters.roll_limit));
	param_get(_parameter_handles.throttle_min, &(_parameters.throttle_min));
	param_get(_parameter_handles.throttle_max, &(_parameters.throttle_max));
	param_get(_parameter_handles.throttle_cruise, &(_parameters.throttle_cruise));
	param_get(_parameter_handles.throttle_slew_max, &(_parameters.throttle_slew_max));

	param_get(_parameter_handles.throttle_land_max, &(_parameters.throttle_land_max));

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

	param_get(_parameter_handles.land_slope_angle, &(_parameters.land_slope_angle));
	param_get(_parameter_handles.land_H1_virt, &(_parameters.land_H1_virt));
	param_get(_parameter_handles.land_flare_alt_relative, &(_parameters.land_flare_alt_relative));
	param_get(_parameter_handles.land_thrust_lim_alt_relative, &(_parameters.land_thrust_lim_alt_relative));

	/* check if negative value for 2/3 of flare altitude is set for throttle cut */
	if (_parameters.land_thrust_lim_alt_relative < 0.0f) {
		_parameters.land_thrust_lim_alt_relative = 0.66f * _parameters.land_flare_alt_relative;
	}

	param_get(_parameter_handles.land_heading_hold_horizontal_distance, &(_parameters.land_heading_hold_horizontal_distance));
	param_get(_parameter_handles.land_flare_pitch_min_deg, &(_parameters.land_flare_pitch_min_deg));
	param_get(_parameter_handles.land_flare_pitch_max_deg, &(_parameters.land_flare_pitch_max_deg));
	param_get(_parameter_handles.land_use_terrain_estimate, &(_parameters.land_use_terrain_estimate));

	_l1_control.set_l1_damping(_parameters.l1_damping);
	_l1_control.set_l1_period(_parameters.l1_period);
	_l1_control.set_l1_roll_limit(math::radians(_parameters.roll_limit));

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

	/* sanity check parameters */
	if (_parameters.airspeed_max < _parameters.airspeed_min ||
	    _parameters.airspeed_max < 5.0f ||
	    _parameters.airspeed_min > 100.0f ||
	    _parameters.airspeed_trim < _parameters.airspeed_min ||
	    _parameters.airspeed_trim > _parameters.airspeed_max) {
		warnx("error: airspeed parameters invalid");
		return 1;
	}

	/* Update the landing slope */
	landingslope.update(math::radians(_parameters.land_slope_angle), _parameters.land_flare_alt_relative, _parameters.land_thrust_lim_alt_relative, _parameters.land_H1_virt);

	/* Update and publish the navigation capabilities */
	_nav_capabilities.landing_slope_angle_rad = landingslope.landing_slope_angle_rad();
	_nav_capabilities.landing_horizontal_slope_displacement = landingslope.horizontal_slope_displacement();
	_nav_capabilities.landing_flare_length = landingslope.flare_length();
	navigation_capabilities_publish();

	/* Update Launch Detector Parameters */
	launchDetector.updateParams();

	/* Update the mTecs */
	_mTecs.updateParams();

	return OK;
}

void
FixedwingPositionControl::vehicle_control_mode_poll()
{
	bool updated;

	orb_check(_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
	}
}

void
FixedwingPositionControl::vehicle_status_poll()
{
	bool updated;

	orb_check(_vehicle_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);
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
FixedwingPositionControl::vehicle_sensor_combined_poll()
{
	/* check if there is a new position */
	bool sensors_updated;
	orb_check(_sensor_combined_sub, &sensors_updated);

	if (sensors_updated) {
		orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub, &_sensor_combined);
	}
}

void
FixedwingPositionControl::vehicle_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool pos_sp_triplet_updated;
	orb_check(_pos_sp_triplet_sub, &pos_sp_triplet_updated);

	if (pos_sp_triplet_updated) {
		orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);
	}
}

void
FixedwingPositionControl::task_main_trampoline(int argc, char *argv[])
{
	l1_control::g_control = new FixedwingPositionControl();

	if (l1_control::g_control == nullptr) {
		warnx("OUT OF MEM");
		return;
	}

	/* only returns on exit */
	l1_control::g_control->task_main();
	delete l1_control::g_control;
	l1_control::g_control = nullptr;
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
FixedwingPositionControl::calculate_gndspeed_undershoot(const math::Vector<2> &current_position, const math::Vector<2> &ground_speed_2d, const struct position_setpoint_triplet_s &pos_sp_triplet)
{

	if (pos_sp_triplet.current.valid && !(pos_sp_triplet.current.type == SETPOINT_TYPE_LOITER)) {

		/* rotate ground speed vector with current attitude */
		math::Vector<2> yaw_vector(_R_nb(0, 0), _R_nb(1, 0));
		yaw_vector.normalize();
		float ground_speed_body = yaw_vector * ground_speed_2d;

		/* The minimum desired ground speed is the minimum airspeed projected on to the ground using the altitude and horizontal difference between the waypoints if available*/
		float distance = 0.0f;
		float delta_altitude = 0.0f;
		if (pos_sp_triplet.previous.valid) {
			distance = get_distance_to_next_waypoint(pos_sp_triplet.previous.lat, pos_sp_triplet.previous.lon, pos_sp_triplet.current.lat, pos_sp_triplet.current.lon);
			delta_altitude = pos_sp_triplet.current.alt - pos_sp_triplet.previous.alt;
		} else {
			distance = get_distance_to_next_waypoint(current_position(0), current_position(1), pos_sp_triplet.current.lat, pos_sp_triplet.current.lon);
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

void FixedwingPositionControl::navigation_capabilities_publish()
{
	if (_nav_capabilities_pub > 0) {
		orb_publish(ORB_ID(navigation_capabilities), _nav_capabilities_pub, &_nav_capabilities);
	} else {
		_nav_capabilities_pub = orb_advertise(ORB_ID(navigation_capabilities), &_nav_capabilities);
	}
}

float FixedwingPositionControl::get_terrain_altitude_landing(float land_setpoint_alt, const struct vehicle_global_position_s &global_pos)
{
	if (!isfinite(global_pos.terrain_alt)) {
		return land_setpoint_alt;
	}

	/* Decide if the terrain estimation can be used, once we switched to using the terrain we stick with it
	 * for the whole landing */
	if (_parameters.land_use_terrain_estimate && (global_pos.terrain_alt_valid || land_useterrain)) {
		if(!land_useterrain) {
			mavlink_log_info(_mavlink_fd, "#audio: Landing, using terrain estimate");
			land_useterrain = true;
		}
		return global_pos.terrain_alt;
	} else {
		return land_setpoint_alt;
	}
}

bool
FixedwingPositionControl::control_position(const math::Vector<2> &current_position, const math::Vector<3> &ground_speed,
		const struct position_setpoint_triplet_s &pos_sp_triplet)
{
	bool setpoint = true;

	math::Vector<2> ground_speed_2d = {ground_speed(0), ground_speed(1)};
	calculate_gndspeed_undershoot(current_position, ground_speed_2d, pos_sp_triplet);

	float eas2tas = 1.0f; // XXX calculate actual number based on current measurements

	/* filter speed and altitude for controller */
	math::Vector<3> accel_body(_sensor_combined.accelerometer_m_s2);
	math::Vector<3> accel_earth = _R_nb * accel_body;

	if (!_mTecs.getEnabled()) {
		_tecs.update_50hz(_global_pos.alt /* XXX might switch to alt err here */, _airspeed.indicated_airspeed_m_s, _R_nb, accel_body, accel_earth);
	}

	/* define altitude error */
	float altitude_error = _pos_sp_triplet.current.alt - _global_pos.alt;

	/* no throttle limit as default */
	float throttle_max = 1.0f;

	/* AUTONOMOUS FLIGHT */

	// XXX this should only execute if auto AND safety off (actuators active),
	// else integrators should be constantly reset.
	if (pos_sp_triplet.current.valid) {

		if (!_was_pos_control_mode) {
			/* reset integrators */
			if (_mTecs.getEnabled()) {
				_mTecs.resetIntegrators();
				_mTecs.resetDerivatives(_airspeed.true_airspeed_m_s);
			}
		}

		_was_pos_control_mode = true;

		/* get circle mode */
		bool was_circle_mode = _l1_control.circle_mode();

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

		if (pos_sp_triplet.current.type == SETPOINT_TYPE_POSITION) {
			/* waypoint is a plain navigation waypoint */
			_l1_control.navigate_waypoints(prev_wp, curr_wp, current_position, ground_speed_2d);
			_att_sp.roll_body = _l1_control.nav_roll();
			_att_sp.yaw_body = _l1_control.nav_bearing();

			tecs_update_pitch_throttle(_pos_sp_triplet.current.alt, calculate_target_airspeed(_parameters.airspeed_trim), eas2tas,
						math::radians(_parameters.pitch_limit_min), math::radians(_parameters.pitch_limit_max),
						_parameters.throttle_min, _parameters.throttle_max, _parameters.throttle_cruise,
						false, math::radians(_parameters.pitch_limit_min), _global_pos.alt, ground_speed);

		} else if (pos_sp_triplet.current.type == SETPOINT_TYPE_LOITER) {

			/* waypoint is a loiter waypoint */
			_l1_control.navigate_loiter(curr_wp, current_position, pos_sp_triplet.current.loiter_radius,
						  pos_sp_triplet.current.loiter_direction, ground_speed_2d);
			_att_sp.roll_body = _l1_control.nav_roll();
			_att_sp.yaw_body = _l1_control.nav_bearing();

			tecs_update_pitch_throttle(_pos_sp_triplet.current.alt, calculate_target_airspeed(_parameters.airspeed_trim), eas2tas,
						math::radians(_parameters.pitch_limit_min), math::radians(_parameters.pitch_limit_max),
						_parameters.throttle_min, _parameters.throttle_max, _parameters.throttle_cruise,
						false, math::radians(_parameters.pitch_limit_min), _global_pos.alt, ground_speed);

		} else if (pos_sp_triplet.current.type == SETPOINT_TYPE_LAND) {

			float bearing_lastwp_currwp = get_bearing_to_next_waypoint(prev_wp(0), prev_wp(1), curr_wp(0), curr_wp(1));
			float bearing_airplane_currwp = get_bearing_to_next_waypoint(current_position(0), current_position(1), curr_wp(0), curr_wp(1));

			/* Horizontal landing control */
			/* switch to heading hold for the last meters, continue heading hold after */
			float wp_distance = get_distance_to_next_waypoint(current_position(0), current_position(1), curr_wp(0), curr_wp(1));
			/* calculate a waypoint distance value which is 0 when the aircraft is behind the waypoint */
			float wp_distance_save = wp_distance;
			if (fabsf(bearing_airplane_currwp - bearing_lastwp_currwp) >= math::radians(90.0f)) {
				wp_distance_save = 0.0f;
			}

			//warnx("wp dist: %d, alt err: %d, noret: %s", (int)wp_distance, (int)altitude_error, (land_noreturn) ? "YES" : "NO");
			if (wp_distance < _parameters.land_heading_hold_horizontal_distance || land_noreturn_horizontal) {

				/* heading hold, along the line connecting this and the last waypoint */

				if (!land_noreturn_horizontal) {//set target_bearing in first occurrence
					if (pos_sp_triplet.previous.valid) {
						target_bearing = bearing_lastwp_currwp;
					} else {
						target_bearing = _att.yaw;
					}
					mavlink_log_info(_mavlink_fd, "#audio: Landing, heading hold");
				}

//					warnx("NORET: %d, target_bearing: %d, yaw: %d", (int)land_noreturn_horizontal, (int)math::degrees(target_bearing), (int)math::degrees(_att.yaw));

				_l1_control.navigate_heading(target_bearing, _att.yaw, ground_speed_2d);

				/* limit roll motion to prevent wings from touching the ground first */
				_att_sp.roll_body = math::constrain(_att_sp.roll_body, math::radians(-10.0f), math::radians(10.0f));

				land_noreturn_horizontal = true;

			} else {

				/* normal navigation */
				_l1_control.navigate_waypoints(prev_wp, curr_wp, current_position, ground_speed_2d);
			}

			_att_sp.roll_body = _l1_control.nav_roll();
			_att_sp.yaw_body = _l1_control.nav_bearing();


			/* Vertical landing control */
			//xxx: using the tecs altitude controller for slope control for now
			/* apply minimum pitch (flare) and limit roll if close to touch down, altitude error is negative (going down) */
			// XXX this could make a great param

			float throttle_land = _parameters.throttle_min + (_parameters.throttle_max - _parameters.throttle_min) * 0.1f;
			float airspeed_land = 1.3f * _parameters.airspeed_min;
			float airspeed_approach = 1.3f * _parameters.airspeed_min;

			/* Get an estimate of the terrain altitude if available, otherwise terrain_alt will be
			 * equal to _pos_sp_triplet.current.alt */
			float terrain_alt = get_terrain_altitude_landing(_pos_sp_triplet.current.alt, _global_pos);

			/* Calculate distance (to landing waypoint) and altitude of last ordinary waypoint L */
			float L_altitude_rel = _pos_sp_triplet.previous.valid ?
				_pos_sp_triplet.previous.alt - terrain_alt : 0.0f;

			float landing_slope_alt_rel_desired = landingslope.getLandingSlopeRelativeAltitudeSave(wp_distance, bearing_lastwp_currwp, bearing_airplane_currwp);

			/* Check if we should start flaring with a vertical and a
			 * horizontal limit (with some tolerance)
			 * The horizontal limit is only applied when we are in front of the wp
			 */
			if (((_global_pos.alt < terrain_alt + landingslope.flare_relative_alt()) &&
						(wp_distance_save < landingslope.flare_length() + 5.0f)) ||
					land_noreturn_vertical) {  //checking for land_noreturn to avoid unwanted climb out
				/* land with minimal speed */

//					/* force TECS to only control speed with pitch, altitude is only implicitely controlled now */
//					_tecs.set_speed_weight(2.0f);

				/* kill the throttle if param requests it */
				throttle_max = _parameters.throttle_max;

				 if (_global_pos.alt < terrain_alt + landingslope.motor_lim_relative_alt() || land_motor_lim) {
					throttle_max = math::min(throttle_max, _parameters.throttle_land_max);
					if (!land_motor_lim) {
						land_motor_lim  = true;
						mavlink_log_info(_mavlink_fd, "#audio: Landing, limiting throttle");
					}

				 }

				float flare_curve_alt_rel = landingslope.getFlareCurveRelativeAltitudeSave(wp_distance, bearing_lastwp_currwp, bearing_airplane_currwp);

				/* avoid climbout */
				if ((flare_curve_alt_rel_last < flare_curve_alt_rel && land_noreturn_vertical) || land_stayonground)
				{
					flare_curve_alt_rel = 0.0f; // stay on ground
					land_stayonground = true;
				}

				tecs_update_pitch_throttle(terrain_alt + flare_curve_alt_rel,
						calculate_target_airspeed(airspeed_land), eas2tas,
						 math::radians(_parameters.land_flare_pitch_min_deg),
						 math::radians(_parameters.land_flare_pitch_max_deg),
						0.0f, throttle_max, throttle_land,
						false,  land_motor_lim ? math::radians(_parameters.land_flare_pitch_min_deg) : math::radians(_parameters.pitch_limit_min),
						_global_pos.alt, ground_speed,
						land_motor_lim ? TECS_MODE_LAND_THROTTLELIM : TECS_MODE_LAND);

				if (!land_noreturn_vertical) {
					mavlink_log_info(_mavlink_fd, "#audio: Landing, flaring");
					land_noreturn_vertical = true;
				}
				//warnx("Landing:  flare, _global_pos.alt  %.1f, flare_curve_alt %.1f, flare_curve_alt_last %.1f, flare_length %.1f, wp_distance %.1f", _global_pos.alt, flare_curve_alt, flare_curve_alt_last, flare_length, wp_distance);

				flare_curve_alt_rel_last = flare_curve_alt_rel;
			} else {

				 /* intersect glide slope:
				  * minimize speed to approach speed
				  * if current position is higher than the slope follow the glide slope (sink to the
				  * glide slope)
				  * also if the system captures the slope it should stay
				  * on the slope (bool land_onslope)
				  * if current position is below the slope continue at previous wp altitude
				  * until the intersection with slope
				  * */
				float altitude_desired_rel;
				if (_global_pos.alt > terrain_alt + landing_slope_alt_rel_desired || land_onslope) {
					/* stay on slope */
					altitude_desired_rel = landing_slope_alt_rel_desired;
					if (!land_onslope) {
						mavlink_log_info(_mavlink_fd, "#audio: Landing, on slope");
						land_onslope = true;
					}
				} else {
					/* continue horizontally */
					altitude_desired_rel =  _pos_sp_triplet.previous.valid ? L_altitude_rel :
						_global_pos.alt - terrain_alt;
				}

				tecs_update_pitch_throttle(terrain_alt + altitude_desired_rel,
						calculate_target_airspeed(airspeed_approach), eas2tas,
						math::radians(_parameters.pitch_limit_min),
						math::radians(_parameters.pitch_limit_max),
						_parameters.throttle_min,
						_parameters.throttle_max,
						_parameters.throttle_cruise,
						false,
						math::radians(_parameters.pitch_limit_min),
						_global_pos.alt,
						ground_speed);
			}

		} else if (pos_sp_triplet.current.type == SETPOINT_TYPE_TAKEOFF) {

			/* Perform launch detection */
			if (launchDetector.launchDetectionEnabled() &&
					launch_detection_state != LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS) {
				/* Inform user that launchdetection is running */
				static hrt_abstime last_sent = 0;
				if(hrt_absolute_time() - last_sent > 4e6) {
					mavlink_log_info(_mavlink_fd, "#audio: Launchdetection running");
					last_sent = hrt_absolute_time();
				}

				/* Detect launch */
				launchDetector.update(_sensor_combined.accelerometer_m_s2[0]);

				/* update our copy of the laucn detection state */
				launch_detection_state = launchDetector.getLaunchDetected();
			} else	{
				/* no takeoff detection --> fly */
				launch_detection_state = LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS;
			}

			/* Set control values depending on the detection state */
			if (launch_detection_state != LAUNCHDETECTION_RES_NONE) {
				/* Launch has been detected, hence we have to control the plane. */

				_l1_control.navigate_waypoints(prev_wp, curr_wp, current_position, ground_speed_2d);
				_att_sp.roll_body = _l1_control.nav_roll();
				_att_sp.yaw_body = _l1_control.nav_bearing();

				/* Select throttle: only in LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS we want to use
				 * full throttle, otherwise we use the preTakeOff Throttle */
				float takeoff_throttle = launch_detection_state !=
					LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS ?
					launchDetector.getThrottlePreTakeoff() : _parameters.throttle_max;

				/* select maximum pitch: the launchdetector may impose another limit for the pitch
				 * depending on the state of the launch */
				float takeoff_pitch_max_deg = launchDetector.getPitchMax(_parameters.pitch_limit_max);
				float takeoff_pitch_max_rad = math::radians(takeoff_pitch_max_deg);

				/* apply minimum pitch and limit roll if target altitude is not within climbout_diff
				 * meters */
				if (_parameters.climbout_diff > 0.001f && altitude_error > _parameters.climbout_diff) {

					/* enforce a minimum of 10 degrees pitch up on takeoff, or take parameter */
					tecs_update_pitch_throttle(_pos_sp_triplet.current.alt,
							calculate_target_airspeed(1.3f * _parameters.airspeed_min),
							eas2tas,
							math::radians(_parameters.pitch_limit_min),
							takeoff_pitch_max_rad,
							_parameters.throttle_min, takeoff_throttle,
							_parameters.throttle_cruise,
							true,
							math::max(math::radians(pos_sp_triplet.current.pitch_min),
							math::radians(10.0f)),
							_global_pos.alt,
							ground_speed,
							TECS_MODE_TAKEOFF,
							takeoff_pitch_max_deg != _parameters.pitch_limit_max);

					/* limit roll motion to ensure enough lift */
					_att_sp.roll_body = math::constrain(_att_sp.roll_body, math::radians(-15.0f),
							math::radians(15.0f));

				} else {
					tecs_update_pitch_throttle(_pos_sp_triplet.current.alt,
							calculate_target_airspeed(_parameters.airspeed_trim),
							eas2tas,
								math::radians(_parameters.pitch_limit_min),
								math::radians(_parameters.pitch_limit_max),
								_parameters.throttle_min,
								takeoff_throttle,
								_parameters.throttle_cruise,
								false,
								math::radians(_parameters.pitch_limit_min),
								_global_pos.alt,
								ground_speed);
				}
			} else {
				/* Tell the attitude controller to stop integrating while we are waiting
				 * for the launch */
				_att_sp.roll_reset_integral = true;
				_att_sp.pitch_reset_integral = true;
				_att_sp.yaw_reset_integral = true;

				/* Set default roll and pitch setpoints during detection phase */
				_att_sp.roll_body = 0.0f;
				_att_sp.pitch_body = math::max(math::radians(pos_sp_triplet.current.pitch_min),
						math::radians(10.0f));
			}

		}

		/* reset landing state */
		if (pos_sp_triplet.current.type != SETPOINT_TYPE_LAND) {
			reset_landing_state();
		}

		/* reset takeoff/launch state */
		if (pos_sp_triplet.current.type != SETPOINT_TYPE_TAKEOFF) {
			reset_takeoff_state();
		}

		if (was_circle_mode && !_l1_control.circle_mode()) {
			/* just kicked out of loiter, reset roll integrals */
			_att_sp.roll_reset_integral = true;
		}

	} else {

		_was_pos_control_mode = false;

		/** MANUAL FLIGHT **/

		/* no flight mode applies, do not publish an attitude setpoint */
		setpoint = false;

		/* reset landing and takeoff state */
		if (!last_manual) {
			reset_landing_state();
			reset_takeoff_state();
		}
	}

	if (_vehicle_status.engine_failure || _vehicle_status.engine_failure_cmd) {
		/* Set thrust to 0 to minimize damage */
		_att_sp.thrust = 0.0f;
	} else if (pos_sp_triplet.current.type == SETPOINT_TYPE_TAKEOFF &&
			launch_detection_state != LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS) {
		 /* making sure again that the correct thrust is used,
		 * without depending on library calls for safety reasons */
		_att_sp.thrust = launchDetector.getThrottlePreTakeoff();
	} else {
		/* Copy thrust and pitch values from tecs */
		_att_sp.thrust = math::min(_mTecs.getEnabled() ? _mTecs.getThrottleSetpoint() :
				_tecs.get_throttle_demand(), throttle_max);
	}

	/* During a takeoff waypoint while waiting for launch the pitch sp is set
	 * already (not by tecs) */
	if (!(pos_sp_triplet.current.type == SETPOINT_TYPE_TAKEOFF &&
			launch_detection_state == LAUNCHDETECTION_RES_NONE)) {
		_att_sp.pitch_body = _mTecs.getEnabled() ? _mTecs.getPitchSetpoint() : _tecs.get_pitch_demand();
	}

	if (_control_mode.flag_control_position_enabled) {
		last_manual = false;
	} else {
		last_manual = true;
	}


	return setpoint;
}

void
FixedwingPositionControl::task_main()
{

	/*
	 * do subscriptions
	 */
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_airspeed_sub = orb_subscribe(ORB_ID(airspeed));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_control_sub = orb_subscribe(ORB_ID(manual_control_setpoint));

	/* rate limit control mode updates to 5Hz */
	orb_set_interval(_control_mode_sub, 200);
	/* rate limit vehicle status updates to 5Hz */
	orb_set_interval(_vehicle_status_sub, 200);
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

	_task_running = true;

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

		/* check vehicle control mode for changes to publication state */
		vehicle_control_mode_poll();

		/* check vehicle status for changes to publication state */
		vehicle_status_poll();

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

			/* XXX Hack to get mavlink output going */
			if (_mavlink_fd < 0) {
				/* try to open the mavlink log device every once in a while */
				_mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
			}

			/* load local copies */
			orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);

			// XXX add timestamp check
			_global_pos_valid = true;

			vehicle_attitude_poll();
			vehicle_setpoint_poll();
			vehicle_sensor_combined_poll();
			vehicle_airspeed_poll();
			// vehicle_baro_poll();

			math::Vector<3> ground_speed(_global_pos.vel_n, _global_pos.vel_e,  _global_pos.vel_d);
			math::Vector<2> current_position((float)_global_pos.lat, (float)_global_pos.lon);

			/*
			 * Attempt to control position, on success (= sensors present and not in manual mode),
			 * publish setpoint.
			 */
			if (control_position(current_position, ground_speed, _pos_sp_triplet)) {
				_att_sp.timestamp = hrt_absolute_time();

				/* lazily publish the setpoint only once available */
				if (_attitude_sp_pub > 0) {
					/* publish the attitude setpoint */
					orb_publish(ORB_ID(vehicle_attitude_setpoint), _attitude_sp_pub, &_att_sp);

				} else {
					/* advertise and publish */
					_attitude_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_att_sp);
				}

				/* XXX check if radius makes sense here */
				float turn_distance = _l1_control.switch_distance(100.0f);

				/* lazily publish navigation capabilities */
				if (fabsf(turn_distance - _nav_capabilities.turn_distance) > FLT_EPSILON && turn_distance > 0) {

					/* set new turn distance */
					_nav_capabilities.turn_distance = turn_distance;

					navigation_capabilities_publish();

				}

			}

		}

		perf_end(_loop_perf);
	}

	_task_running = false;

	warnx("exiting.\n");

	_control_task = -1;
	_exit(0);
}

void FixedwingPositionControl::reset_takeoff_state()
{
	launch_detection_state = LAUNCHDETECTION_RES_NONE;
	launchDetector.reset();
}

void FixedwingPositionControl::reset_landing_state()
{
	land_noreturn_horizontal = false;
	land_noreturn_vertical = false;
	land_stayonground = false;
	land_motor_lim = false;
	land_onslope = false;
	land_useterrain = false;
}

void FixedwingPositionControl::tecs_update_pitch_throttle(float alt_sp, float v_sp, float eas2tas,
		float pitch_min_rad, float pitch_max_rad,
		float throttle_min, float throttle_max, float throttle_cruise,
		bool climbout_mode, float climbout_pitch_min_rad,
		float altitude,
		const math::Vector<3> &ground_speed,
		tecs_mode mode, bool pitch_max_special)
{
	if (_mTecs.getEnabled()) {
		/* Using mtecs library: prepare arguments for mtecs call */
		float flightPathAngle = 0.0f;
		float ground_speed_length = ground_speed.length();
		if (ground_speed_length > FLT_EPSILON) {
			flightPathAngle = -asinf(ground_speed(2)/ground_speed_length);
		}
		fwPosctrl::LimitOverride limitOverride;
		if (_vehicle_status.engine_failure || _vehicle_status.engine_failure_cmd) {
			/* Force the slow downwards spiral */
			limitOverride.enablePitchMinOverride(-1.0f);
			limitOverride.enablePitchMaxOverride(5.0f);

		} else if (climbout_mode) {
			limitOverride.enablePitchMinOverride(M_RAD_TO_DEG_F * climbout_pitch_min_rad);
		} else {
			limitOverride.disablePitchMinOverride();
		}

		if (pitch_max_special) {
			/* Use the maximum pitch from the argument */
			limitOverride.enablePitchMaxOverride(M_RAD_TO_DEG_F * pitch_max_rad);
		} else {
			/* use pitch max set by MT param */
			limitOverride.disablePitchMaxOverride();
		}
		_mTecs.updateAltitudeSpeed(flightPathAngle, altitude, alt_sp, _airspeed.true_airspeed_m_s, v_sp, mode,
				limitOverride);
	} else {
		if (_vehicle_status.engine_failure || _vehicle_status.engine_failure_cmd) {
			/* Force the slow downwards spiral */
			pitch_min_rad = M_DEG_TO_RAD_F * -1.0f;
			pitch_max_rad = M_DEG_TO_RAD_F * 5.0f;
		}

/* No underspeed protection in landing mode */
		_tecs.set_detect_underspeed_enabled(!(mode == TECS_MODE_LAND || mode == TECS_MODE_LAND_THROTTLELIM));

		/* Using tecs library */
		_tecs.update_pitch_throttle(_R_nb, _att.pitch, altitude, alt_sp, v_sp,
					    _airspeed.indicated_airspeed_m_s, eas2tas,
					    climbout_mode, climbout_pitch_min_rad,
					    throttle_min, throttle_max, throttle_cruise,
					    pitch_min_rad, pitch_max_rad);

		struct TECS::tecs_state s;
		_tecs.get_tecs_state(s);

		struct tecs_status_s t;

		t.timestamp = s.timestamp;

		switch (s.mode) {
			case TECS::ECL_TECS_MODE_NORMAL:
				t.mode = TECS_MODE_NORMAL;
				break;
			case TECS::ECL_TECS_MODE_UNDERSPEED:
				t.mode = TECS_MODE_UNDERSPEED;
				break;
			case TECS::ECL_TECS_MODE_BAD_DESCENT:
				t.mode = TECS_MODE_BAD_DESCENT;
				break;
			case TECS::ECL_TECS_MODE_CLIMBOUT:
				t.mode = TECS_MODE_CLIMBOUT;
				break;
		}

		t.altitudeSp			= s.hgt_dem;
		t.altitude_filtered		= s.hgt;
		t.airspeedSp			= s.spd_dem;
		t.airspeed_filtered		= s.spd;

		t.flightPathAngleSp		= s.dhgt_dem;
		t.flightPathAngle		= s.dhgt;
		t.flightPathAngleFiltered	= s.dhgt;

		t.airspeedDerivativeSp		= s.dspd_dem;
		t.airspeedDerivative		= s.dspd;

		t.totalEnergyRateSp		= s.thr;
		t.totalEnergyRate		= s.ithr;
		t.energyDistributionRateSp	= s.ptch;
		t.energyDistributionRate	= s.iptch;

		if (_tecs_status_pub > 0) {
			orb_publish(ORB_ID(tecs_status), _tecs_status_pub, &t);
		} else {
			_tecs_status_pub = orb_advertise(ORB_ID(tecs_status), &t);
		}
	}
}

int
FixedwingPositionControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = task_spawn_cmd("fw_pos_control_l1",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_MAX - 5,
				       2000,
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

		if (OK != FixedwingPositionControl::start()) {
			err(1, "start failed");
		}

		/* avoid memory fragmentation by not exiting start handler until the task has fully started */
		while (l1_control::g_control == nullptr || !l1_control::g_control->task_running()) {
			usleep(50000);
			printf(".");
			fflush(stdout);
		}
		printf("\n");

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
