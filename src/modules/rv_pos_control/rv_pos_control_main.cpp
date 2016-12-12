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
 * @author Andreas Antener <andreas@uaventure.com>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
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
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/position_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/fw_virtual_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/navigation_capabilities.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>
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
#include <systemlib/param/param.h>
#include <platforms/px4_defines.h>
#include <runway_takeoff/RunwayTakeoff.h>

//#include <controllib/blocks.hpp>
//#include <controllib/block/BlockParam.hpp>
static int	_control_task = -1;			/**< task handle for sensor task */
#define HDG_HOLD_DIST_NEXT 			1.0f 	// initial distance of waypoint in front of plane in heading hold mode
#define HDG_HOLD_REACHED_DIST 		0.2f 	// distance (plane to waypoint in front) at which waypoints are reset in heading hold mode
#define HDG_HOLD_SET_BACK_DIST 		0.2f 		// distance by which previous waypoint is set behind the plane
#define HDG_HOLD_YAWRATE_THRESH 	0.15f 		// max yawrate at which plane locks yaw for heading hold mode
#define HDG_HOLD_MAN_INPUT_THRESH 	0.01f 		// max manual roll input from user which does not change the locked heading
#define TAKEOFF_IDLE				0.2f 		// idle speed for POSCTRL/ATTCTRL (when landed and throttle stick > 0)
#define T_ALT_TIMEOUT 				1 			// time after which we abort landing if terrain estimate is not valid



/**
 * L1 control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int rv_pos_control_main(int argc, char *argv[]);

using namespace launchdetection;

class RoverPositionControl
{
public:
	/**
	 * Constructor
	 */
    RoverPositionControl();

	/**
	 * Destructor, also kills the sensors task.
	 */
	~RoverPositionControl();

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

    int     _local_pos_sub;      /**< vehicle local position */
	int		_global_pos_sub;
	int		_pos_sp_triplet_sub;
	int		_ctrl_state_sub;			/**< control state subscription */
	int		_control_mode_sub;		/**< control mode subscription */
	int		_vehicle_status_sub;		/**< vehicle status subscription */
	int 	_params_sub;			/**< notification of parameter updates */
	int 	_manual_control_sub;		/**< notification of manual control updates */
	int		_sensor_combined_sub;		/**< for body frame accelerations */
	int     _att_sub;

	orb_advert_t	_attitude_sp_pub;		/**< attitude setpoint */
    orb_advert_t    _local_pos_sp_pub;      /**< vehicle local position setpoint publication */
	orb_advert_t	_nav_capabilities_pub;		/**< navigation capabilities publication */

	orb_id_t _attitude_setpoint_id;

	struct control_state_s				_ctrl_state;			/**< control state */
	struct vehicle_attitude_setpoint_s		_att_sp;			/**< vehicle attitude setpoint */
	struct navigation_capabilities_s		_nav_capabilities;		/**< navigation capabilities */
	struct manual_control_setpoint_s		_manual;			/**< r/c channel data */
	struct vehicle_control_mode_s			_control_mode;			/**< control mode */
	struct vehicle_status_s				_vehicle_status;		/**< vehicle status */
	struct vehicle_global_position_s		_global_pos;			/**< global vehicle position */
	struct position_setpoint_triplet_s		_pos_sp_triplet;		/**< triplet of mission items */
	struct sensor_combined_s			_sensor_combined;		/**< for body frame accelerations */
    struct vehicle_local_position_setpoint_s    _local_pos_sp;      /**< vehicle local position setpoint */
    struct vehicle_local_position_s         _local_pos;     /**< vehicle local position */
    struct vehicle_attitude_s               att;



	perf_counter_t	_loop_perf;			/**< loop performance counter */

	float	_hold_alt;				/**< hold altitude for altitude mode */
	float	_takeoff_ground_alt;				/**< ground altitude at which plane was launched */
	float	_hdg_hold_yaw;				/**< hold heading for velocity mode */
	bool	_hdg_hold_enabled;			/**< heading hold enabled */
	bool	_yaw_lock_engaged;			/**< yaw is locked for heading hold */
	float	_althold_epv;				/**< the position estimate accuracy when engaging alt hold */
	bool	_was_in_deadband;				/**< wether the last stick input was in althold deadband */
	float    yaw_dep; /**orientation de  départ**/
	struct position_setpoint_s _hdg_hold_prev_wp;	/**< position where heading hold started */
	struct position_setpoint_s _hdg_hold_curr_wp;	/**< position to which heading hold flies */
	hrt_abstime _control_position_last_called; /**<last call of control_position  */

	/* land states */
	bool land_noreturn_horizontal;
	bool land_noreturn_vertical;
	bool land_stayonground;
	bool land_motor_lim;
	bool land_onslope;
	bool land_useterrain;
    bool _reset_pos_sp;
	bool _run_pos_control;
	bool _mode_auto;
	bool _pos_hold_engaged;



	    math::Vector<3> _pos;
	    math::Vector<3> _pos_sp;
	    math::Vector<3> _vel;
	    math::Vector<3> _vel_sp;
	    math::Vector<3> _vel_prev;          /**< velocity on previous step */
	    math::Vector<3> _vel_ff;
	    math::Vector<3> _vel_sp_prev;
	    math::Vector<3> _thrust_sp_prev;
	    math::Vector<3> _vel_err_d;
	    	// landing relevant states
	float _t_alt_prev_valid;	//**< last terrain estimate which was valid */
	hrt_abstime _time_last_t_alt; //*< time at which we had last valid terrain alt */
	hrt_abstime _time_started_landing;	//*< time at which landing started */
	float height_flare;					//*< estimated height to ground at which flare started */

	bool _was_in_air;	/**< indicated wether the plane was in the air in the previous interation*/
	hrt_abstime _time_went_in_air;	/**< time at which the plane went in the air */

	runwaytakeoff::RunwayTakeoff _runway_takeoff;

	/* takeoff/launch states */
	LaunchDetectionResult launch_detection_state;

	bool last_manual;				///< true if the last iteration was in manual mode (used to determine when a reset is needed)

	/* Landingslope object */

	float flare_curve_alt_rel_last;

	/* heading hold */
	float target_bearing;

	/* Launch detection */
	launchdetection::LaunchDetector launchDetector;

	/* throttle and airspeed states */
	float _airspeed_error;				///< airspeed error to setpoint in m/s
	bool _airspeed_valid;				///< flag if a valid airspeed estimate exists
	uint64_t _airspeed_last_received;			///< last time airspeed was received. Used to detect timeouts.
	float _groundspeed_undershoot;			///< ground speed error to min. speed in m/s
	bool _global_pos_valid;				///< global position is valid
	math::Matrix<3, 3> _R_nb;			///< current attitude
	float _roll;
	float _pitch;
	float _yaw;

	ECL_L1_Pos_Controller				_rv_control;

	enum RV_POSCTRL_MODE {
		RV_POSCTRL_MODE_AUTO,
		RV_POSCTRL_MODE_POSITION,
		RV_POSCTRL_MODE_ALTITUDE,
		RV_POSCTRL_MODE_OTHER,
		RV_POSCTRL_MODE_OFFBOARD
	} _control_mode_current;			///< used to check the mode in the last control loop iteration. Use to check if the last iteration was in the same mode.


	/**
	     * Reset position setpoint to current position.
	     *
	     * This reset will only occur if the _reset_pos_sp flag has been set.
	     * The general logic is to first "activate" the flag in the flight
	     * regime where a switch to a position control mode should hold the
	     * very last position. Once switching to a position control mode
	     * the last position is stored once.
	     */
	void        reset_pos_sp();
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
	void		navigation_capabilities_publish();

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
	     * Set position setpoint using offboard control
	     */
	    void        control_offboard();


	/**
	 * Check if we are in a takeoff situation
	 */
	bool in_takeoff_situation();

	/**
	 * Do takeoff help when in altitude controlled modes
	 * @param hold_altitude altitude setpoint for controller
	 * @param pitch_limit_min minimum pitch allowed
	 */




	/**
	 * Control position.
	 */
	bool		control_position(const math::Vector<2> &global_pos, const math::Vector<3> &ground_speed,
					 const struct position_setpoint_triplet_s &_pos_sp_triplet);


	void		calculate_gndspeed_undershoot(const math::Vector<2> &current_position, const math::Vector<2> &ground_speed_2d,
			const struct position_setpoint_triplet_s &pos_sp_triplet);

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
	void		reset_takeoff_state();

	/*
	 * Reset landing state
	 */
	void		reset_landing_state();


};



namespace rv_control
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

RoverPositionControl	*g_control = nullptr;
}

RoverPositionControl::RoverPositionControl() :

	_mavlink_fd(-1),
	_task_should_exit(false),
	_task_running(false),

	/* subscriptions */
	_local_pos_sub(-1),
	_global_pos_sub(-1),
	_pos_sp_triplet_sub(-1),
	_ctrl_state_sub(-1),
	_control_mode_sub(-1),
	_vehicle_status_sub(-1),
	_params_sub(-1),
	_manual_control_sub(-1),
	_sensor_combined_sub(-1),
	_att_sub(-1),

	/* publications */
	_attitude_sp_pub(nullptr),
    _local_pos_sp_pub(nullptr),
	_nav_capabilities_pub(nullptr),

	/* publication ID */
	_attitude_setpoint_id(0),

	/* states */
	_ctrl_state(),
	_att_sp(),
	_nav_capabilities(),
	_manual(),
	_control_mode(),
	_vehicle_status(),
	_global_pos(),
	_pos_sp_triplet(),
	_sensor_combined(),
	_local_pos_sp(),
	_local_pos(),
	att(),

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

	land_noreturn_horizontal(false),
	land_noreturn_vertical(false),
	land_stayonground(false),
	land_motor_lim(false),
	land_onslope(false),
	land_useterrain(false),
    _reset_pos_sp(true),//F.BERNAT
    _run_pos_control(true),//F.BERNAT
	_t_alt_prev_valid(0),
	_time_last_t_alt(0),
	_time_started_landing(0),
	height_flare(0.0f),
	_was_in_air(false),
	_time_went_in_air(0),
	_runway_takeoff(),
	launch_detection_state(LAUNCHDETECTION_RES_NONE),
	last_manual(false),
	flare_curve_alt_rel_last(0.0f),
	target_bearing(0.0f),
	launchDetector(),
	_airspeed_error(0.0f),
	_airspeed_valid(false),
	_airspeed_last_received(0),
	_groundspeed_undershoot(0.0f),
	_global_pos_valid(false),
	_rv_control(),
	_control_mode_current(RV_POSCTRL_MODE_OTHER)
{
	_nav_capabilities.turn_distance = 0.0f;
    memset(&_local_pos_sp, 0, sizeof(_local_pos_sp));
    memset(&_vehicle_status, 0, sizeof(_vehicle_status));
    memset(&_ctrl_state, 0, sizeof(_ctrl_state));
    _ctrl_state.q[0] = 1.0f;
    memset(&_att_sp, 0, sizeof(_att_sp));
    memset(&_manual, 0, sizeof(_manual));
    memset(&_control_mode, 0, sizeof(_control_mode));
    memset(&_pos_sp_triplet, 0, sizeof(_pos_sp_triplet));
    memset(&_local_pos, 0, sizeof(_local_pos));
    memset(&att, 0, sizeof(att));
    _pos.zero();
    _pos_sp.zero();
    _vel.zero();
    _vel_sp.zero();
    _vel_prev.zero();
    _vel_ff.zero();
    _vel_sp_prev.zero();
    _vel_err_d.zero();
}

RoverPositionControl::~RoverPositionControl()
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

	rv_control::g_control = nullptr;
}



void
RoverPositionControl::vehicle_control_mode_poll()
{
	bool updated;

	orb_check(_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
	}
}

void
RoverPositionControl::vehicle_status_poll()
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

bool
RoverPositionControl::vehicle_manual_control_setpoint_poll()
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
RoverPositionControl::control_state_poll()
{
	/* check if there is a new position */
	bool ctrl_state_updated;
	orb_check(_ctrl_state_sub, &ctrl_state_updated);

	if (ctrl_state_updated) {
		orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);

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
	//_tecs.enable_airspeed(_airspeed_valid);
}

void
RoverPositionControl::vehicle_sensor_combined_poll()
{
	/* check if there is a new position */
	bool sensors_updated;
	orb_check(_sensor_combined_sub, &sensors_updated);

	if (sensors_updated) {
		orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub, &_sensor_combined);
	}
}

void
RoverPositionControl::vehicle_setpoint_poll()
{

	/* check if there is a new setpoint */
	bool pos_sp_triplet_updated;
	orb_check(_pos_sp_triplet_sub, &pos_sp_triplet_updated);

	if (pos_sp_triplet_updated) {
		orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);
	}
}

void
RoverPositionControl::task_main_trampoline(int argc, char *argv[])
{
	rv_control::g_control = new RoverPositionControl();

	if (rv_control::g_control == nullptr) {
		warnx("OUT OF MEM");
		return;
	}

	/* only returns on exit */
	rv_control::g_control->task_main();
	delete rv_control::g_control;
	rv_control::g_control = nullptr;
}


void RoverPositionControl::navigation_capabilities_publish()
{
	_nav_capabilities.timestamp = hrt_absolute_time();

	if (_nav_capabilities_pub != nullptr) {
		orb_publish(ORB_ID(navigation_capabilities), _nav_capabilities_pub, &_nav_capabilities);

	} else {
		_nav_capabilities_pub = orb_advertise(ORB_ID(navigation_capabilities), &_nav_capabilities);
	}
}

void RoverPositionControl::get_waypoint_heading_distance(float heading, float distance,
		struct position_setpoint_s &waypoint_prev, struct position_setpoint_s &waypoint_next, bool flag_init)
{
	waypoint_prev.valid = true;
	waypoint_prev.alt = _hold_alt;
	position_setpoint_s temp_next {};
	position_setpoint_s temp_prev {};

	if (flag_init) {
		// on init set previous waypoint HDG_HOLD_SET_BACK_DIST meters behind us
		waypoint_from_heading_and_distance(_global_pos.lat, _global_pos.lon, heading + 180.0f * M_DEG_TO_RAD_F ,
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

bool RoverPositionControl::in_takeoff_situation()
{

	return true;
}

void
RoverPositionControl::calculate_gndspeed_undershoot(const math::Vector<2> &current_position,
        const math::Vector<2> &ground_speed_2d, const struct position_setpoint_triplet_s &pos_sp_triplet)
{

    if (pos_sp_triplet.current.valid && !(pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LOITER)) {

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


        float ground_speed_desired = 2.0f * cosf(atan2f(delta_altitude, distance));

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

bool
RoverPositionControl::control_position(const math::Vector<2> &current_position, const math::Vector<3> &ground_speed,
		const struct position_setpoint_triplet_s &pos_sp_triplet)
{
    _control_position_last_called = hrt_absolute_time();

    bool setpoint = true;

    _att_sp.fw_control_yaw = false;     // by default we don't want yaw to be contoller directly with rudder
    _att_sp.apply_flaps = false;        // by default we don't use flaps

    /* filter speed and altitude for controller */
    math::Vector<3> accel_body(_sensor_combined.accelerometer_m_s2);
    math::Vector<3> accel_earth = _R_nb * accel_body;
    math::Vector<2> ground_speed_2d = {ground_speed(0), ground_speed(1)};
    calculate_gndspeed_undershoot(current_position, ground_speed_2d, pos_sp_triplet);


    if (_control_mode.flag_control_auto_enabled &&
        pos_sp_triplet.current.valid) {
        /* AUTONOMOUS FLIGHT */

        _control_mode_current = RV_POSCTRL_MODE_AUTO;

        /* reset hold yaw */
        _hdg_hold_yaw = _yaw;


        /* current waypoint (the one currently heading for) */
        math::Vector<2> next_wp((float)pos_sp_triplet.current.lat, (float)pos_sp_triplet.current.lon);

        /* current waypoint (the one currently heading for) */
        math::Vector<2> curr_wp((float)pos_sp_triplet.current.lat, (float)pos_sp_triplet.current.lon);

        /* Initialize attitude controller integrator reset flags to 0 */

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

        if (pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_IDLE) {
            _att_sp.thrust = 0.0f;

        } else if (pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_POSITION) {
            /* waypoint is a plain navigation waypoint */
            _rv_control.navigate_waypoints(prev_wp, curr_wp, current_position, ground_speed_2d);

            _att_sp.yaw_body = _rv_control.nav_bearing();

            _att_sp.thrust = 0.3f;

        } else if (pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LOITER) {

            /* waypoint is a loiter waypoint */
            _rv_control.navigate_loiter(curr_wp, current_position, pos_sp_triplet.current.loiter_radius,
                            pos_sp_triplet.current.loiter_direction, ground_speed_2d);

            _att_sp.yaw_body = _rv_control.nav_bearing();

            _att_sp.thrust = 0.3f;

        } else if (pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {

            // apply full flaps for landings. this flag will also trigger the use of flaperons
            // if they have been enabled using the corresponding parameter
            _att_sp.apply_flaps = true;

            // save time at which we started landing
            if (_time_started_landing == 0) {
                _time_started_landing = hrt_absolute_time();
            }

            float bearing_lastwp_currwp = get_bearing_to_next_waypoint(prev_wp(0), prev_wp(1), curr_wp(0), curr_wp(1));


            /* Horizontal landing control */
            /* switch to heading hold for the last meters, continue heading hold after */
            float wp_distance = get_distance_to_next_waypoint(current_position(0), current_position(1), curr_wp(0), curr_wp(1));
            /* calculate a waypoint distance value which is 0 when the aircraft is behind the waypoint */




            // create virtual waypoint which is on the desired flight path but
            // some distance behind landing waypoint. This will make sure that the plane
            // will always follow the desired flight path even if we get close or past
            // the landing waypoint
            math::Vector<2> curr_wp_shifted;
            double lat;
            double lon;
            create_waypoint_from_line_and_dist(pos_sp_triplet.current.lat, pos_sp_triplet.current.lon, pos_sp_triplet.previous.lat,
                               pos_sp_triplet.previous.lon, -1000.0f, &lat, &lon);
            curr_wp_shifted(0) = (float)lat;
            curr_wp_shifted(1) = (float)lon;

            // we want the plane to keep tracking the desired flight path until we start flaring
            // if we go into heading hold mode earlier then we risk to be pushed away from the runway by cross winds
            //if (land_noreturn_vertical) {
            if (wp_distance < 1 || land_noreturn_horizontal) {

                /* heading hold, along the line connecting this and the last waypoint */

                if (!land_noreturn_horizontal) {//set target_bearing in first occurrence
                    if (pos_sp_triplet.previous.valid) {
                        target_bearing = bearing_lastwp_currwp;

                    } else {
                        target_bearing = _yaw;
                    }

                    mavlink_log_info(_mavlink_fd, "#Landing, heading hold");
                }

//                  warnx("NORET: %d, target_bearing: %d, yaw: %d", (int)land_noreturn_horizontal, (int)math::degrees(target_bearing), (int)math::degrees(_yaw));

                _rv_control.navigate_heading(target_bearing, _yaw, ground_speed_2d);

                land_noreturn_horizontal = true;

            } else {

                /* normal navigation */
                _rv_control.navigate_waypoints(prev_wp, curr_wp, current_position, ground_speed_2d);
            }


            _att_sp.yaw_body = _rv_control.nav_bearing();
            _att_sp.thrust = 0.2f;

        } else if (pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF) {

                // update runway takeoff helper
                _runway_takeoff.update(
                    _ctrl_state.airspeed,
                    15,
                    _global_pos.lat,
                    _global_pos.lon,
                    _mavlink_fd);

                /*
                 * Update navigation: _runway_takeoff returns the start WP according to mode and phase.
                 * If we use the navigator heading or not is decided later.
                 */
                _rv_control.navigate_waypoints(_runway_takeoff.getStartWP(), curr_wp, current_position, ground_speed_2d);


                _att_sp.thrust = 0.3f;

                // assign values

                _att_sp.yaw_body = _rv_control.nav_bearing();
                _att_sp.fw_control_yaw = _runway_takeoff.controlYaw();

                reset_takeoff_state();
        }

        /* reset landing state */
        if (pos_sp_triplet.current.type != position_setpoint_s::SETPOINT_TYPE_LAND) {
            reset_landing_state();
        }

        /* reset takeoff/launch state */
        if (pos_sp_triplet.current.type != position_setpoint_s::SETPOINT_TYPE_TAKEOFF) {
            reset_takeoff_state();
        }

    }else if (_control_mode.flag_control_velocity_enabled &&
           _control_mode.flag_control_altitude_enabled) {
        /* POSITION CONTROL: pitch stick moves altitude setpoint, throttle stick sets airspeed,
           heading is set to a distant waypoint */

        if (_control_mode_current != RV_POSCTRL_MODE_POSITION) {
            /* Need to init because last loop iteration was in a different mode */
            _hold_alt = _global_pos.alt;
            _hdg_hold_yaw = _yaw;
            _hdg_hold_enabled = false; // this makes sure the waypoints are reset below
            _yaw_lock_engaged = false;
        }

        _control_mode_current = RV_POSCTRL_MODE_POSITION;

        /* heading control */

        if (fabsf(_manual.y) < HDG_HOLD_MAN_INPUT_THRESH) {
            /* heading / roll is zero, lock onto current heading */
            if (fabsf(_ctrl_state.yaw_rate) < HDG_HOLD_YAWRATE_THRESH && !_yaw_lock_engaged) {
                // little yaw movement, lock to current heading
                _yaw_lock_engaged = true;

            }


            if (_yaw_lock_engaged) {

                /* just switched back from non heading-hold to heading hold */
                if (!_hdg_hold_enabled) {
                    _hdg_hold_enabled = true;
                    _hdg_hold_yaw = _yaw;

                    get_waypoint_heading_distance(_hdg_hold_yaw, HDG_HOLD_DIST_NEXT, _hdg_hold_prev_wp, _hdg_hold_curr_wp, true);
                }

                /* we have a valid heading hold position, are we too close? */
                if (get_distance_to_next_waypoint(_global_pos.lat, _global_pos.lon,
                                  _hdg_hold_curr_wp.lat, _hdg_hold_curr_wp.lon) < HDG_HOLD_REACHED_DIST) {
                    get_waypoint_heading_distance(_hdg_hold_yaw, HDG_HOLD_DIST_NEXT, _hdg_hold_prev_wp, _hdg_hold_curr_wp, false);
                }

                math::Vector<2> prev_wp;
                prev_wp(0) = (float)_hdg_hold_prev_wp.lat;
                prev_wp(1) = (float)_hdg_hold_prev_wp.lon;

                math::Vector<2> curr_wp;
                curr_wp(0) = (float)_hdg_hold_curr_wp.lat;
                curr_wp(1) = (float)_hdg_hold_curr_wp.lon;

                /* populate l1 control setpoint */
                _rv_control.navigate_waypoints(prev_wp, curr_wp, current_position, ground_speed_2d);


                _att_sp.yaw_body = _rv_control.nav_bearing();

            }

        } else {
            _hdg_hold_enabled = false;
            _yaw_lock_engaged = false;
        }

    } else{

        _control_mode_current = RV_POSCTRL_MODE_OTHER;

        /** MANUAL FLIGHT **/

        // reset terrain estimation relevant values
        _time_started_landing = 0;
        _time_last_t_alt = 0;

        // reset lading abort state
        _nav_capabilities.abort_landing = false;

        /* no flight mode applies, do not publish an attitude setpoint */
        setpoint = false;

        /* reset landing and takeoff state */
        if (!last_manual) {
            reset_landing_state();
            reset_takeoff_state();
        }
      }

    if (_control_mode_current ==  RV_POSCTRL_MODE_AUTO && // launchdetector only available in auto
           pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF ) {
        /* making sure again that the correct thrust is used,
        * without depending on library calls for safety reasons */
        _att_sp.thrust = 0.3f;

    } else if (_control_mode_current ==  RV_POSCTRL_MODE_AUTO &&
           pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF ) {
        _att_sp.thrust = 0.3f;

    } else if (_control_mode_current ==  RV_POSCTRL_MODE_AUTO &&
           pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_IDLE) {
        _att_sp.thrust = 0.0f;


    } else {
        /* Copy thrust and pitch values from tecs */
        if (_vehicle_status.condition_landed &&
                (_control_mode_current == RV_POSCTRL_MODE_POSITION || _control_mode_current == RV_POSCTRL_MODE_ALTITUDE)) {
            // when we are landed in these modes we want the motor to spin
            _att_sp.thrust = 0.0f;//F.BERNAT 0 avant

        } else if (!_control_mode.flag_control_offboard_enabled){
            _att_sp.thrust =0.3f;
        }


    }

    if (_control_mode.flag_control_position_enabled) {
        last_manual = false;

    } else {
        last_manual = true;
    }


    return setpoint;
}

void
RoverPositionControl::reset_pos_sp()
{
    if (_reset_pos_sp) {
        _reset_pos_sp = false;
        /* shift position setpoint to make attitude setpoint continuous */
        _pos_sp(0) = _pos(0) + (_vel(0) - PX4_R(_att_sp.R_body, 0, 2) * _att_sp.thrust /  _vel_sp(0));
        _pos_sp(1) = _pos(1) + (_vel(1) - PX4_R(_att_sp.R_body, 1, 2) * _att_sp.thrust / _vel_sp(1)) ;

        //mavlink_log_info(_mavlink_fd, "[mpc] reset pos sp: %d, %d", (int)_pos_sp(0), (int)_pos_sp(1));
    }
}

void
RoverPositionControl::control_offboard()
{
    bool updated;
    orb_check(_pos_sp_triplet_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);
    }

    if (_pos_sp_triplet.current.valid) {
        if ( _pos_sp_triplet.current.position_valid) {
            /* control position */
            _pos_sp(0) = _pos_sp_triplet.current.x;
            _pos_sp(1) = _pos_sp_triplet.current.y;
      //      fprintf(stderr, "_pos_sp_triplet.current.x=%0.2f,_pos_sp_triplet.current.y=%0.2f \n",(double)_pos_sp_triplet.current.x,(double)_pos_sp_triplet.current.y);

        } else if (_pos_sp_triplet.current.velocity_valid) {
            /* control velocity */
            /* reset position setpoint to current position if needed */
            reset_pos_sp();

            /* set position setpoint move rate */
            _vel_sp(0) = _pos_sp_triplet.current.vx;
            _vel_sp(1) = _pos_sp_triplet.current.vy;

            _run_pos_control = false; /* request velocity setpoint to be used, instead of position setpoint */
        }

        if (_pos_sp_triplet.current.yaw_valid) {
            _att_sp.yaw_body = _pos_sp_triplet.current.yaw;
            fprintf(stderr, "_pos_sp_triplet.current.yaw=%0.2f\n",(double)_pos_sp_triplet.current.yaw);
        }
    }
}


void
RoverPositionControl::task_main()
{
    bool updated;
    float x,y,alpha;
    bool first=true;


	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	_ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
	_sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_control_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
    _local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));//F.BERNAT
    _att_sub = orb_subscribe(ORB_ID(vehicle_attitude));

	/* rate limit control mode updates to 5Hz */
	orb_set_interval(_control_mode_sub, 200);
	/* rate limit vehicle status updates to 5Hz */
	orb_set_interval(_vehicle_status_sub, 200);
	/* rate limit position updates to 50 Hz */
	orb_set_interval(_global_pos_sub, 20);
	/* rate limit position updates to 50 Hz */
	orb_set_interval(_local_pos_sub, 20);
    /* rate limit position updates to 50 Hz */
    orb_set_interval(_att_sub, 20);
	/* wakeup source(s) */

    px4_pollfd_struct_t fds;//F.BERNAT avant 1

	fds.fd = _local_pos_sub;
	fds.events = POLLIN;

	_task_running = true;
	orb_check(_local_pos_sub, &updated);//F.BERNAT
    if (updated) {
         orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
    }//F.BERNAT
	while (!_task_should_exit) {

	    /* wait for up to 500ms for data */
        int pret = px4_poll(&fds, (sizeof(fds) / sizeof(fds)), 100);

        if (pret == 0) { continue; }

        /* this is undesirable but not much we can do - might want to flag unhappy status */
        if (pret < 0) {
            warn("poll error %d, %d", pret, errno);
            continue;
        }

		/* check vehicle control mode for changes to publication state */
		vehicle_control_mode_poll();
		/* check vehicle status for changes to publication state */
		vehicle_status_poll();

		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
		/* only run controller if position changed */

		if (fds.revents & POLLIN) {
			perf_begin(_loop_perf);

			/* XXX Hack to get mavlink output going */
			if (_mavlink_fd < 0) {
				/* try to open the mavlink log device every once in a while */
				_mavlink_fd = px4_open(MAVLINK_LOG_DEVICE, 0);
			}

			/* load local copies */
			orb_check(_global_pos_sub, &updated);//F.BERNAT
			if (updated) {
			  orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);
			  _global_pos_valid = true;
			}

			control_state_poll();
			vehicle_setpoint_poll();
			vehicle_sensor_combined_poll();
			vehicle_manual_control_setpoint_poll();


			math::Vector<3> ground_speed(_global_pos.vel_n, _global_pos.vel_e,  _global_pos.vel_d);
			math::Vector<2> current_position((float)_global_pos.lat, (float)_global_pos.lon);

			/*
			 * Attempt to control position, on success (= sensors present and not in manual mode),
			 * publish setpoint.
			 */
			if (_control_mode.flag_control_offboard_enabled) {
			       orb_copy(ORB_ID(vehicle_attitude), _att_sub, &att);
			       orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);

                   control_offboard();
                   if(first){
                       yaw_dep=att.yaw;
                       first=false;
                   }

                   if ( _pos_sp_triplet.current.valid){
                       x= _pos_sp_triplet.current.x - _local_pos.x;
                       y= _pos_sp_triplet.current.y - _local_pos.y;
                       alpha=atan2f(y,x);
                       //fprintf(stderr, "alpha=%0.2f\n",(double)alpha);
                       //fprintf(stderr, "att.yaw=%0.2f\n",(double)att.yaw);
                       _att_sp.yaw_body=yaw_dep-alpha;
                      // fprintf(stderr, "_local_pos.x=%0.2f,_local_pos.y=%0.2f\n",(double)_local_pos.x,(double)_local_pos.y);

                     //  fprintf(stderr, "_att_sp.yaw_body=%0.2f\n",(double)_att_sp.yaw_body);
                        if ((_att_sp.yaw_body < 0.0f) && (_att_sp.yaw_body < -3.1415926f))
                         _att_sp.yaw_body= _att_sp.yaw_body + 2*3.1415926f;
                        else
                         if ((_att_sp.yaw_body > 0.0f) && (_att_sp.yaw_body >  3.1415926f))
                          _att_sp.yaw_body = _att_sp.yaw_body - 2*3.1415926f;

                        _att_sp.thrust = _pos_sp_triplet.current.vx; //0.3f;
                       }else{
                           _att_sp.thrust = 0.0f;
                       }

                     _att_sp.timestamp = hrt_absolute_time();

                   /* lazily publish the setpoint only once available */
                   if (_attitude_sp_pub != nullptr) {
                       /* publish the attitude setpoint */
                       orb_publish(_attitude_setpoint_id, _attitude_sp_pub, &_att_sp);

                   } else if (_attitude_setpoint_id) {
                       /* advertise and publish */
                       _attitude_sp_pub = orb_advertise(_attitude_setpoint_id, &_att_sp);
                   }

                   /* XXX check if radius makes sense here */
                   float turn_distance = _rv_control.switch_distance(1.0f);

                   /* lazily publish navigation capabilities */
                   if ((hrt_elapsed_time(&_nav_capabilities.timestamp) > 1000000)
                       || (fabsf(turn_distance - _nav_capabilities.turn_distance) > FLT_EPSILON
                       && turn_distance > 0)) {

                       /* set new turn distance */
                       _nav_capabilities.turn_distance = turn_distance;

                       navigation_capabilities_publish();
                   }

			}else if (control_position(current_position, ground_speed, _pos_sp_triplet)) {
				_att_sp.timestamp = hrt_absolute_time();

				/* lazily publish the setpoint only once available */
				if (_attitude_sp_pub != nullptr) {
					/* publish the attitude setpoint */
					orb_publish(_attitude_setpoint_id, _attitude_sp_pub, &_att_sp);

				} else if (_attitude_setpoint_id) {
					/* advertise and publish */
					_attitude_sp_pub = orb_advertise(_attitude_setpoint_id, &_att_sp);
				}

				/* XXX check if radius makes sense here */
				float turn_distance = _rv_control.switch_distance(1.0f);

				/* lazily publish navigation capabilities */
				if ((hrt_elapsed_time(&_nav_capabilities.timestamp) > 1000000)
				    || (fabsf(turn_distance - _nav_capabilities.turn_distance) > FLT_EPSILON
					&& turn_distance > 0)) {

					/* set new turn distance */
					_nav_capabilities.turn_distance = turn_distance;

					navigation_capabilities_publish();

				}

			}

			perf_end(_loop_perf);
		}

	}

	_task_running = false;

	warnx("exiting.\n");

	_control_task = -1;
}

void RoverPositionControl::reset_takeoff_state()
{
	launch_detection_state = LAUNCHDETECTION_RES_NONE;
	launchDetector.reset();
	_runway_takeoff.reset();
}

void RoverPositionControl::reset_landing_state()
{
	land_noreturn_horizontal = false;
	land_noreturn_vertical = false;
	land_stayonground = false;
	land_motor_lim = false;
	land_onslope = false;
	land_useterrain = false;
}


int
RoverPositionControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("rv_pos_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1300,
					   (px4_main_t)&RoverPositionControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int rv_pos_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: rv_pos_control {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (rv_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		if (OK != RoverPositionControl::start()) {
			warn("start failed");
			return 1;
		}

		/* avoid memory fragmentation by not exiting start handler until the task has fully started */
		while (rv_control::g_control == nullptr || !rv_control::g_control->task_running()) {
			usleep(50000);
			printf(".");
			fflush(stdout);
		}

		printf("\n");

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (rv_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete rv_control::g_control;
		rv_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (rv_control::g_control) {
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
