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
 * @file fw_att_control_main.c
 * Implementation of a generic attitude controller based on classic orthogonal PIDs.
 *
 * @author Lorenz Meier 	<lm@inf.ethz.ch>
 * @author Thomas Gubler 	<thomasgubler@gmail.com>
 * @author Roman Bapst		<bapstr@ethz.ch>
 *
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
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_controls_virtual_fw.h>
#include <uORB/topics/actuator_controls_virtual_mc.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/fw_virtual_rates_setpoint.h>
#include <uORB/topics/mc_virtual_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_status.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/pid/pid.h>
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>

#include <ecl/attitude_fw/ecl_pitch_controller.h>
#include <ecl/attitude_fw/ecl_roll_controller.h>
#include <ecl/attitude_fw/ecl_yaw_controller.h>
#include <platforms/px4_defines.h>

/**
 * Fixedwing attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int fw_att_control_main(int argc, char *argv[]);

class FixedwingAttitudeControl
{
public:
	/**
	 * Constructor
	 */
	FixedwingAttitudeControl();

	/**
	 * Destructor, also kills the main task.
	 */
	~FixedwingAttitudeControl();

	/**
	 * Start the main task.
	 *
	 * @return	OK on success.
	 */
	int		start();

	/**
	 * Task status
	 *
	 * @return	true if the mainloop is running
	 */
	bool		task_running() { return _task_running; }

private:

	bool		_task_should_exit;		/**< if true, attitude control task should exit */
	bool		_task_running;			/**< if true, task is running in its mainloop */
	int		_control_task;			/**< task handle */

	int		_att_sub;			/**< vehicle attitude subscription */
	int		_accel_sub;			/**< accelerometer subscription */
	int		_att_sp_sub;			/**< vehicle attitude setpoint */
	int		_attitude_sub;			/**< raw rc channels data subscription */
	int		_airspeed_sub;			/**< airspeed subscription */
	int		_vcontrol_mode_sub;		/**< vehicle status subscription */
	int 		_params_sub;			/**< notification of parameter updates */
	int 		_manual_sub;			/**< notification of manual control updates */
	int		_global_pos_sub;		/**< global position subscription */
	int		_vehicle_status_sub;		/**< vehicle status subscription */

	orb_advert_t	_rate_sp_pub;			/**< rate setpoint publication */
	orb_advert_t	_attitude_sp_pub;		/**< attitude setpoint point */
	orb_advert_t	_actuators_0_pub;		/**< actuator control group 0 setpoint */
	orb_advert_t	_actuators_2_pub;		/**< actuator control group 1 setpoint (Airframe) */

	orb_id_t _rates_sp_id;	// pointer to correct rates setpoint uORB metadata structure
	orb_id_t _actuators_id;	// pointer to correct actuator controls0 uORB metadata structure

	struct vehicle_attitude_s			_att;			/**< vehicle attitude */
	struct accel_report				_accel;			/**< body frame accelerations */
	struct vehicle_attitude_setpoint_s		_att_sp;		/**< vehicle attitude setpoint */
	struct vehicle_rates_setpoint_s			_rates_sp;	/* attitude rates setpoint */
	struct manual_control_setpoint_s		_manual;		/**< r/c channel data */
	struct airspeed_s				_airspeed;		/**< airspeed */
	struct vehicle_control_mode_s			_vcontrol_mode;		/**< vehicle control mode */
	struct actuator_controls_s			_actuators;		/**< actuator control inputs */
	struct actuator_controls_s			_actuators_airframe;	/**< actuator control inputs */
	struct vehicle_global_position_s		_global_pos;		/**< global position */
	struct vehicle_status_s				_vehicle_status;	/**< vehicle status */

	perf_counter_t	_loop_perf;			/**< loop performance counter */
	perf_counter_t	_nonfinite_input_perf;		/**< performance counter for non finite input */
	perf_counter_t	_nonfinite_output_perf;		/**< performance counter for non finite output */

	bool		_setpoint_valid;		/**< flag if the position control setpoint is valid */
	bool		_debug;				/**< if set to true, print debug output */

	struct {
		float tconst;
		float p_p;
		float p_i;
		float p_ff;
		float p_rmax_pos;
		float p_rmax_neg;
		float p_integrator_max;
		float p_roll_feedforward;
		float r_p;
		float r_i;
		float r_ff;
		float r_integrator_max;
		float r_rmax;
		float y_p;
		float y_i;
		float y_d;
		float y_ff;
		float y_roll_feedforward;
		float y_integrator_max;
		float y_coordinated_min_speed;
		int32_t y_coordinated_method;
		float y_rmax;

		float airspeed_min;
		float airspeed_trim;
		float airspeed_max;

		float trim_roll;
		float trim_pitch;
		float trim_yaw;
		float rollsp_offset_deg;		/**< Roll Setpoint Offset in deg */
		float pitchsp_offset_deg;		/**< Pitch Setpoint Offset in deg */
		float rollsp_offset_rad;		/**< Roll Setpoint Offset in rad */
		float pitchsp_offset_rad;		/**< Pitch Setpoint Offset in rad */
		float man_roll_max;				/**< Max Roll in rad */
		float man_pitch_max;			/**< Max Pitch in rad */

		int vtol_type;					/**< VTOL type: 0 = tailsitter, 1 = tiltrotor */

	}		_parameters;			/**< local copies of interesting parameters */

	struct {

		param_t tconst;
		param_t p_p;
		param_t p_i;
		param_t p_ff;
		param_t p_rmax_pos;
		param_t p_rmax_neg;
		param_t p_integrator_max;
		param_t p_roll_feedforward;
		param_t r_p;
		param_t r_i;
		param_t r_ff;
		param_t r_integrator_max;
		param_t r_rmax;
		param_t y_p;
		param_t y_i;
		param_t y_d;
		param_t y_ff;
		param_t y_roll_feedforward;
		param_t y_integrator_max;
		param_t y_coordinated_min_speed;
		param_t y_coordinated_method;
		param_t y_rmax;

		param_t airspeed_min;
		param_t airspeed_trim;
		param_t airspeed_max;

		param_t trim_roll;
		param_t trim_pitch;
		param_t trim_yaw;
		param_t rollsp_offset_deg;
		param_t pitchsp_offset_deg;
		param_t man_roll_max;
		param_t man_pitch_max;

		param_t vtol_type;

	}		_parameter_handles;		/**< handles for interesting parameters */


	ECL_RollController				_roll_ctrl;
	ECL_PitchController				_pitch_ctrl;
	ECL_YawController				_yaw_ctrl;


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
	 * Check for changes in vehicle control mode.
	 */
	void		vehicle_control_mode_poll();

	/**
	 * Check for changes in manual inputs.
	 */
	void		vehicle_manual_poll();


	/**
	 * Check for airspeed updates.
	 */
	void		vehicle_airspeed_poll();

	/**
	 * Check for accel updates.
	 */
	void		vehicle_accel_poll();

	/**
	 * Check for set triplet updates.
	 */
	void		vehicle_setpoint_poll();

	/**
	 * Check for global position updates.
	 */
	void		global_pos_poll();

	/**
	 * Check for vehicle status updates.
	 */
	void		vehicle_status_poll();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main attitude controller collection task.
	 */
	void		task_main();

};

namespace att_control
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

FixedwingAttitudeControl	*g_control = nullptr;
}

FixedwingAttitudeControl::FixedwingAttitudeControl() :

	_task_should_exit(false),
	_task_running(false),
	_control_task(-1),

/* subscriptions */
	_att_sub(-1),
	_accel_sub(-1),
	_airspeed_sub(-1),
	_vcontrol_mode_sub(-1),
	_params_sub(-1),
	_manual_sub(-1),
	_global_pos_sub(-1),
	_vehicle_status_sub(-1),

/* publications */
	_rate_sp_pub(-1),
	_attitude_sp_pub(-1),
	_actuators_0_pub(-1),
	_actuators_2_pub(-1),

	_rates_sp_id(0),
	_actuators_id(0),

/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "fw att control")),
	_nonfinite_input_perf(perf_alloc(PC_COUNT, "fw att control nonfinite input")),
	_nonfinite_output_perf(perf_alloc(PC_COUNT, "fw att control nonfinite output")),
/* states */
	_setpoint_valid(false),
	_debug(false)
{
	/* safely initialize structs */
	_att = {};
	_accel = {};
	_att_sp = {};
	_rates_sp = {};
	_manual = {};
	_airspeed = {};
	_vcontrol_mode = {};
	_actuators = {};
	_actuators_airframe = {};
	_global_pos = {};
	_vehicle_status = {};


	_parameter_handles.tconst = param_find("FW_ATT_TC");
	_parameter_handles.p_p = param_find("FW_PR_P");
	_parameter_handles.p_i = param_find("FW_PR_I");
	_parameter_handles.p_ff = param_find("FW_PR_FF");
	_parameter_handles.p_rmax_pos = param_find("FW_P_RMAX_POS");
	_parameter_handles.p_rmax_neg = param_find("FW_P_RMAX_NEG");
	_parameter_handles.p_integrator_max = param_find("FW_PR_IMAX");
	_parameter_handles.p_roll_feedforward = param_find("FW_P_ROLLFF");

	_parameter_handles.r_p = param_find("FW_RR_P");
	_parameter_handles.r_i = param_find("FW_RR_I");
	_parameter_handles.r_ff = param_find("FW_RR_FF");
	_parameter_handles.r_integrator_max = param_find("FW_RR_IMAX");
	_parameter_handles.r_rmax = param_find("FW_R_RMAX");

	_parameter_handles.y_p = param_find("FW_YR_P");
	_parameter_handles.y_i = param_find("FW_YR_I");
	_parameter_handles.y_ff = param_find("FW_YR_FF");
	_parameter_handles.y_integrator_max = param_find("FW_YR_IMAX");
	_parameter_handles.y_rmax = param_find("FW_Y_RMAX");

	_parameter_handles.airspeed_min = param_find("FW_AIRSPD_MIN");
	_parameter_handles.airspeed_trim = param_find("FW_AIRSPD_TRIM");
	_parameter_handles.airspeed_max = param_find("FW_AIRSPD_MAX");

	_parameter_handles.y_coordinated_min_speed = param_find("FW_YCO_VMIN");
	_parameter_handles.y_coordinated_method = param_find("FW_YCO_METHOD");

	_parameter_handles.trim_roll = param_find("TRIM_ROLL");
	_parameter_handles.trim_pitch = param_find("TRIM_PITCH");
	_parameter_handles.trim_yaw = param_find("TRIM_YAW");
	_parameter_handles.rollsp_offset_deg = param_find("FW_RSP_OFF");
	_parameter_handles.pitchsp_offset_deg = param_find("FW_PSP_OFF");

	_parameter_handles.man_roll_max = param_find("FW_MAN_R_MAX");
	_parameter_handles.man_pitch_max = param_find("FW_MAN_P_MAX");

	_parameter_handles.vtol_type = param_find("VT_TYPE");

	/* fetch initial parameter values */
	parameters_update();
}

FixedwingAttitudeControl::~FixedwingAttitudeControl()
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

	perf_free(_loop_perf);
	perf_free(_nonfinite_input_perf);
	perf_free(_nonfinite_output_perf);

	att_control::g_control = nullptr;
}

int
FixedwingAttitudeControl::parameters_update()
{

	param_get(_parameter_handles.tconst, &(_parameters.tconst));
	param_get(_parameter_handles.p_p, &(_parameters.p_p));
	param_get(_parameter_handles.p_i, &(_parameters.p_i));
	param_get(_parameter_handles.p_ff, &(_parameters.p_ff));
	param_get(_parameter_handles.p_rmax_pos, &(_parameters.p_rmax_pos));
	param_get(_parameter_handles.p_rmax_neg, &(_parameters.p_rmax_neg));
	param_get(_parameter_handles.p_integrator_max, &(_parameters.p_integrator_max));
	param_get(_parameter_handles.p_roll_feedforward, &(_parameters.p_roll_feedforward));

	param_get(_parameter_handles.r_p, &(_parameters.r_p));
	param_get(_parameter_handles.r_i, &(_parameters.r_i));
	param_get(_parameter_handles.r_ff, &(_parameters.r_ff));

	param_get(_parameter_handles.r_integrator_max, &(_parameters.r_integrator_max));
	param_get(_parameter_handles.r_rmax, &(_parameters.r_rmax));

	param_get(_parameter_handles.y_p, &(_parameters.y_p));
	param_get(_parameter_handles.y_i, &(_parameters.y_i));
	param_get(_parameter_handles.y_ff, &(_parameters.y_ff));
	param_get(_parameter_handles.y_integrator_max, &(_parameters.y_integrator_max));
	param_get(_parameter_handles.y_coordinated_min_speed, &(_parameters.y_coordinated_min_speed));
	param_get(_parameter_handles.y_coordinated_method, &(_parameters.y_coordinated_method));
	param_get(_parameter_handles.y_rmax, &(_parameters.y_rmax));

	param_get(_parameter_handles.airspeed_min, &(_parameters.airspeed_min));
	param_get(_parameter_handles.airspeed_trim, &(_parameters.airspeed_trim));
	param_get(_parameter_handles.airspeed_max, &(_parameters.airspeed_max));

	param_get(_parameter_handles.trim_roll, &(_parameters.trim_roll));
	param_get(_parameter_handles.trim_pitch, &(_parameters.trim_pitch));
	param_get(_parameter_handles.trim_yaw, &(_parameters.trim_yaw));
	param_get(_parameter_handles.rollsp_offset_deg, &(_parameters.rollsp_offset_deg));
	param_get(_parameter_handles.pitchsp_offset_deg, &(_parameters.pitchsp_offset_deg));
	_parameters.rollsp_offset_rad = math::radians(_parameters.rollsp_offset_deg);
	_parameters.pitchsp_offset_rad = math::radians(_parameters.pitchsp_offset_deg);
	param_get(_parameter_handles.man_roll_max, &(_parameters.man_roll_max));
	param_get(_parameter_handles.man_pitch_max, &(_parameters.man_pitch_max));
	_parameters.man_roll_max = math::radians(_parameters.man_roll_max);
	_parameters.man_pitch_max = math::radians(_parameters.man_pitch_max);

	param_get(_parameter_handles.vtol_type, &_parameters.vtol_type);

	/* pitch control parameters */
	_pitch_ctrl.set_time_constant(_parameters.tconst);
	_pitch_ctrl.set_k_p(_parameters.p_p);
	_pitch_ctrl.set_k_i(_parameters.p_i);
	_pitch_ctrl.set_k_ff(_parameters.p_ff);
	_pitch_ctrl.set_integrator_max(_parameters.p_integrator_max);
	_pitch_ctrl.set_max_rate_pos(math::radians(_parameters.p_rmax_pos));
	_pitch_ctrl.set_max_rate_neg(math::radians(_parameters.p_rmax_neg));
	_pitch_ctrl.set_roll_ff(_parameters.p_roll_feedforward);

	/* roll control parameters */
	_roll_ctrl.set_time_constant(_parameters.tconst);
	_roll_ctrl.set_k_p(_parameters.r_p);
	_roll_ctrl.set_k_i(_parameters.r_i);
	_roll_ctrl.set_k_ff(_parameters.r_ff);
	_roll_ctrl.set_integrator_max(_parameters.r_integrator_max);
	_roll_ctrl.set_max_rate(math::radians(_parameters.r_rmax));

	/* yaw control parameters */
	_yaw_ctrl.set_k_p(_parameters.y_p);
	_yaw_ctrl.set_k_i(_parameters.y_i);
	_yaw_ctrl.set_k_ff(_parameters.y_ff);
	_yaw_ctrl.set_integrator_max(_parameters.y_integrator_max);
	_yaw_ctrl.set_coordinated_min_speed(_parameters.y_coordinated_min_speed);
	_yaw_ctrl.set_coordinated_method(_parameters.y_coordinated_method);
	_yaw_ctrl.set_max_rate(math::radians(_parameters.y_rmax));

	return OK;
}

void
FixedwingAttitudeControl::vehicle_control_mode_poll()
{
	bool vcontrol_mode_updated;

	/* Check if vehicle control mode has changed */
	orb_check(_vcontrol_mode_sub, &vcontrol_mode_updated);

	if (vcontrol_mode_updated) {

		orb_copy(ORB_ID(vehicle_control_mode), _vcontrol_mode_sub, &_vcontrol_mode);
	}
}

void
FixedwingAttitudeControl::vehicle_manual_poll()
{
	bool manual_updated;

	/* get pilots inputs */
	orb_check(_manual_sub, &manual_updated);

	if (manual_updated) {

		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
	}
}

void
FixedwingAttitudeControl::vehicle_airspeed_poll()
{
	/* check if there is a new position */
	bool airspeed_updated;
	orb_check(_airspeed_sub, &airspeed_updated);

	if (airspeed_updated) {
		orb_copy(ORB_ID(airspeed), _airspeed_sub, &_airspeed);
	}
}

void
FixedwingAttitudeControl::vehicle_accel_poll()
{
	/* check if there is a new position */
	bool accel_updated;
	orb_check(_accel_sub, &accel_updated);

	if (accel_updated) {
		orb_copy(ORB_ID(sensor_accel), _accel_sub, &_accel);
	}
}

void
FixedwingAttitudeControl::vehicle_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool att_sp_updated;
	orb_check(_att_sp_sub, &att_sp_updated);

	if (att_sp_updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _att_sp_sub, &_att_sp);
		_setpoint_valid = true;
	}
}

void
FixedwingAttitudeControl::global_pos_poll()
{
	/* check if there is a new global position */
	bool global_pos_updated;
	orb_check(_global_pos_sub, &global_pos_updated);

	if (global_pos_updated) {
		orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);
	}
}

void
FixedwingAttitudeControl::vehicle_status_poll()
{
	/* check if there is new status information */
	bool vehicle_status_updated;
	orb_check(_vehicle_status_sub, &vehicle_status_updated);

	if (vehicle_status_updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);
		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (!_rates_sp_id) {
			if (_vehicle_status.is_vtol) {
				_rates_sp_id = ORB_ID(fw_virtual_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_virtual_fw);
			} else {
				_rates_sp_id = ORB_ID(vehicle_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_0);
			}
		}
	}
}

void
FixedwingAttitudeControl::task_main_trampoline(int argc, char *argv[])
{
	att_control::g_control->task_main();
}

void
FixedwingAttitudeControl::task_main()
{
	/*
	 * do subscriptions
	 */
	_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_accel_sub = orb_subscribe_multi(ORB_ID(sensor_accel), 0);
	_airspeed_sub = orb_subscribe(ORB_ID(airspeed));
	_vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

	/* rate limit vehicle status updates to 5Hz */
	orb_set_interval(_vcontrol_mode_sub, 200);
	/* do not limit the attitude updates in order to minimize latency.
	 * actuator outputs are still limited by the individual drivers
	 * properly to not saturate IO or physical limitations */

	parameters_update();

	/* get an initial update for all sensor and status data */
	vehicle_airspeed_poll();
	vehicle_setpoint_poll();
	vehicle_accel_poll();
	vehicle_control_mode_poll();
	vehicle_manual_poll();
	vehicle_status_poll();

	/* wakeup source(s) */
	struct pollfd fds[2];

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _att_sub;
	fds[1].events = POLLIN;

	_task_running = true;

	while (!_task_should_exit) {

		static int loop_counter = 0;

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

		/* only update parameters if they changed */
		if (fds[0].revents & POLLIN) {
			/* read from param to clear updated flag */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			/* update parameters from storage */
			parameters_update();
		}

		/* only run controller if attitude changed */
		if (fds[1].revents & POLLIN) {

			static uint64_t last_run = 0;
			float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too large deltaT's */
			if (deltaT > 1.0f)
				deltaT = 0.01f;

			/* load local copies */
			orb_copy(ORB_ID(vehicle_attitude), _att_sub, &_att);

			if (_vehicle_status.is_vtol && _parameters.vtol_type == 0) {
				/* vehicle is a tailsitter, we need to modify the estimated attitude for fw mode
				 *
				 * Since the VTOL airframe is initialized as a multicopter we need to
				 * modify the estimated attitude for the fixed wing operation.
				 * Since the neutral position of the vehicle in fixed wing mode is -90 degrees rotated around
				 * the pitch axis compared to the neutral position of the vehicle in multicopter mode
				 * we need to swap the roll and the yaw axis (1st and 3rd column) in the rotation matrix.
				 * Additionally, in order to get the correct sign of the pitch, we need to multiply
				 * the new x axis of the rotation matrix with -1
				 *
				 * original:			modified:
				 *
				 * Rxx  Ryx  Rzx		-Rzx  Ryx  Rxx
				 * Rxy	Ryy  Rzy		-Rzy  Ryy  Rxy
				 * Rxz	Ryz  Rzz		-Rzz  Ryz  Rxz
				 * */
				math::Matrix<3, 3> R;				//original rotation matrix
				math::Matrix<3, 3> R_adapted;		//modified rotation matrix
				R.set(_att.R);
				R_adapted.set(_att.R);

				/* move z to x */
				R_adapted(0, 0) = R(0, 2);
				R_adapted(1, 0) = R(1, 2);
				R_adapted(2, 0) = R(2, 2);

				/* move x to z */
				R_adapted(0, 2) = R(0, 0);
				R_adapted(1, 2) = R(1, 0);
				R_adapted(2, 2) = R(2, 0);

				/* change direction of pitch (convert to right handed system) */
				R_adapted(0, 0) = -R_adapted(0, 0);
				R_adapted(1, 0) = -R_adapted(1, 0);
				R_adapted(2, 0) = -R_adapted(2, 0);
				math::Vector<3> euler_angles;		//adapted euler angles for fixed wing operation
				euler_angles = R_adapted.to_euler();

				/* fill in new attitude data */
				_att.roll    = euler_angles(0);
				_att.pitch   = euler_angles(1);
				_att.yaw     = euler_angles(2);
				PX4_R(_att.R, 0, 0) = R_adapted(0, 0);
				PX4_R(_att.R, 0, 1) = R_adapted(0, 1);
				PX4_R(_att.R, 0, 2) = R_adapted(0, 2);
				PX4_R(_att.R, 1, 0) = R_adapted(1, 0);
				PX4_R(_att.R, 1, 1) = R_adapted(1, 1);
				PX4_R(_att.R, 1, 2) = R_adapted(1, 2);
				PX4_R(_att.R, 2, 0) = R_adapted(2, 0);
				PX4_R(_att.R, 2, 1) = R_adapted(2, 1);
				PX4_R(_att.R, 2, 2) = R_adapted(2, 2);

				/* lastly, roll- and yawspeed have to be swaped */
				float helper = _att.rollspeed;
				_att.rollspeed = -_att.yawspeed;
				_att.yawspeed = helper;
			}

			vehicle_airspeed_poll();

			vehicle_setpoint_poll();

			vehicle_accel_poll();

			vehicle_control_mode_poll();

			vehicle_manual_poll();

			global_pos_poll();

			vehicle_status_poll();

			/* lock integrator until control is started */
			bool lock_integrator;

			if (_vcontrol_mode.flag_control_attitude_enabled && !_vehicle_status.is_rotary_wing) {
				lock_integrator = false;

			} else {
				lock_integrator = true;
			}

			/* Simple handling of failsafe: deploy parachute if failsafe is on */
			if (_vcontrol_mode.flag_control_termination_enabled) {
				_actuators_airframe.control[7] = 1.0f;
				//warnx("_actuators_airframe.control[1] = 1.0f;");
			} else {
				_actuators_airframe.control[7] = 0.0f;
				//warnx("_actuators_airframe.control[1] = -1.0f;");
			}

			/* if we are in rotary wing mode, do nothing */
			if (_vehicle_status.is_rotary_wing && !_vehicle_status.is_vtol) {
				continue;
			}

			/* default flaps to center */
			float flaps_control = 0.0f;

			/* map flaps by default to manual if valid */
			if (isfinite(_manual.flaps)) {
				flaps_control = _manual.flaps;
			}

			/* decide if in stabilized or full manual control */
			if (_vcontrol_mode.flag_control_attitude_enabled) {
				/* scale around tuning airspeed */
				float airspeed;

				/* if airspeed is not updating, we assume the normal average speed */
				if (bool nonfinite = !isfinite(_airspeed.true_airspeed_m_s) ||
				    hrt_elapsed_time(&_airspeed.timestamp) > 1e6) {
					airspeed = _parameters.airspeed_trim;
					if (nonfinite) {
						perf_count(_nonfinite_input_perf);
					}
				} else {
					/* prevent numerical drama by requiring 0.5 m/s minimal speed */
					airspeed = math::max(0.5f, _airspeed.true_airspeed_m_s);
				}

				/*
				 * For scaling our actuators using anything less than the min (close to stall)
				 * speed doesn't make any sense - its the strongest reasonable deflection we
				 * want to do in flight and its the baseline a human pilot would choose.
				 *
				 * Forcing the scaling to this value allows reasonable handheld tests.
				 */

				float airspeed_scaling = _parameters.airspeed_trim / ((airspeed < _parameters.airspeed_min) ? _parameters.airspeed_min : airspeed);

				float roll_sp = _parameters.rollsp_offset_rad;
				float pitch_sp = _parameters.pitchsp_offset_rad;
				float yaw_manual = 0.0f;
				float throttle_sp = 0.0f;

				/* Read attitude setpoint from uorb if
				 * - velocity control or position control is enabled (pos controller is running)
				 * - manual control is disabled (another app may send the setpoint, but it should
				 *   for sure not be set from the remote control values)
				 */
				if (_vcontrol_mode.flag_control_auto_enabled ||
						!_vcontrol_mode.flag_control_manual_enabled) {
					/* read in attitude setpoint from attitude setpoint uorb topic */
					roll_sp = _att_sp.roll_body + _parameters.rollsp_offset_rad;
					pitch_sp = _att_sp.pitch_body + _parameters.pitchsp_offset_rad;
					throttle_sp = _att_sp.thrust;

					/* reset integrals where needed */
					if (_att_sp.roll_reset_integral) {
						_roll_ctrl.reset_integrator();
					}
					if (_att_sp.pitch_reset_integral) {
						_pitch_ctrl.reset_integrator();
					}
					if (_att_sp.yaw_reset_integral) {
						_yaw_ctrl.reset_integrator();
					}
				} else if (_vcontrol_mode.flag_control_velocity_enabled) {

					/* the pilot does not want to change direction,
					 * take straight attitude setpoint from position controller
					 */
					if (fabsf(_manual.y) < 0.01f && fabsf(_att.roll) < 0.2f) {
						roll_sp = _att_sp.roll_body + _parameters.rollsp_offset_rad;
					} else {
						roll_sp = (_manual.y * _parameters.man_roll_max)
								+ _parameters.rollsp_offset_rad;
					}

					pitch_sp = _att_sp.pitch_body + _parameters.pitchsp_offset_rad;
					throttle_sp = _att_sp.thrust;

					/* reset integrals where needed */
					if (_att_sp.roll_reset_integral) {
						_roll_ctrl.reset_integrator();
					}
					if (_att_sp.pitch_reset_integral) {
						_pitch_ctrl.reset_integrator();
					}
					if (_att_sp.yaw_reset_integral) {
						_yaw_ctrl.reset_integrator();
					}

				} else if (_vcontrol_mode.flag_control_altitude_enabled) {
 					/*
					 * Velocity should be controlled and manual is enabled.
					*/
					roll_sp = (_manual.y * _parameters.man_roll_max) + _parameters.rollsp_offset_rad;
					pitch_sp = _att_sp.pitch_body + _parameters.pitchsp_offset_rad;
					throttle_sp = _att_sp.thrust;

					/* reset integrals where needed */
					if (_att_sp.roll_reset_integral) {
						_roll_ctrl.reset_integrator();
					}
					if (_att_sp.pitch_reset_integral) {
						_pitch_ctrl.reset_integrator();
					}
					if (_att_sp.yaw_reset_integral) {
						_yaw_ctrl.reset_integrator();
					}
				} else {
					/*
					 * Scale down roll and pitch as the setpoints are radians
					 * and a typical remote can only do around 45 degrees, the mapping is
					 * -1..+1 to -man_roll_max rad..+man_roll_max rad (equivalent for pitch)
					 *
					 * With this mapping the stick angle is a 1:1 representation of
					 * the commanded attitude.
					 *
					 * The trim gets subtracted here from the manual setpoint to get
					 * the intended attitude setpoint. Later, after the rate control step the
					 * trim is added again to the control signal.
					 */
					roll_sp = (_manual.y * _parameters.man_roll_max) + _parameters.rollsp_offset_rad;
					pitch_sp = -(_manual.x * _parameters.man_pitch_max) + _parameters.pitchsp_offset_rad;
					/* allow manual control of rudder deflection */
					yaw_manual = _manual.r;
					throttle_sp = _manual.z;

					/*
					 * in manual mode no external source should / does emit attitude setpoints.
					 * emit the manual setpoint here to allow attitude controller tuning
					 * in attitude control mode.
					 */
					struct vehicle_attitude_setpoint_s att_sp;
					att_sp.timestamp = hrt_absolute_time();
					att_sp.roll_body = roll_sp;
					att_sp.pitch_body = pitch_sp;
					att_sp.yaw_body = 0.0f - _parameters.trim_yaw;
					att_sp.thrust = throttle_sp;

					/* lazily publish the setpoint only once available */
					if (_attitude_sp_pub > 0 && !_vehicle_status.is_rotary_wing) {
						/* publish the attitude setpoint */
						orb_publish(ORB_ID(vehicle_attitude_setpoint), _attitude_sp_pub, &att_sp);

					} else if (_attitude_sp_pub < 0 && !_vehicle_status.is_rotary_wing) {
						/* advertise and publish */
						_attitude_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);
					}
				}

				/* If the aircraft is on ground reset the integrators */
				if (_vehicle_status.condition_landed || _vehicle_status.is_rotary_wing) {
					_roll_ctrl.reset_integrator();
					_pitch_ctrl.reset_integrator();
					_yaw_ctrl.reset_integrator();
				}

				/* Prepare speed_body_u and speed_body_w */
				float speed_body_u = 0.0f;
				float speed_body_v = 0.0f;
				float speed_body_w = 0.0f;
				if(_att.R_valid) 	{
					speed_body_u = PX4_R(_att.R, 0, 0) * _global_pos.vel_n + PX4_R(_att.R, 1, 0) * _global_pos.vel_e + PX4_R(_att.R, 2, 0) * _global_pos.vel_d;
					speed_body_v = PX4_R(_att.R, 0, 1) * _global_pos.vel_n + PX4_R(_att.R, 1, 1) * _global_pos.vel_e + PX4_R(_att.R, 2, 1) * _global_pos.vel_d;
					speed_body_w = PX4_R(_att.R, 0, 2) * _global_pos.vel_n + PX4_R(_att.R, 1, 2) * _global_pos.vel_e + PX4_R(_att.R, 2, 2) * _global_pos.vel_d;
				} else	{
					if (_debug && loop_counter % 10 == 0) {
						warnx("Did not get a valid R\n");
					}
				}

				/* Prepare data for attitude controllers */
				struct ECL_ControlData control_input = {};
				control_input.roll = _att.roll;
				control_input.pitch = _att.pitch;
				control_input.yaw = _att.yaw;
				control_input.roll_rate = _att.rollspeed;
				control_input.pitch_rate = _att.pitchspeed;
				control_input.yaw_rate = _att.yawspeed;
				control_input.speed_body_u = speed_body_u;
				control_input.speed_body_v = speed_body_v;
				control_input.speed_body_w = speed_body_w;
				control_input.acc_body_x = _accel.x;
				control_input.acc_body_y = _accel.y;
				control_input.acc_body_z = _accel.z;
				control_input.roll_setpoint = roll_sp;
				control_input.pitch_setpoint = pitch_sp;
				control_input.airspeed_min = _parameters.airspeed_min;
				control_input.airspeed_max = _parameters.airspeed_max;
				control_input.airspeed = airspeed;
				control_input.scaler = airspeed_scaling;
				control_input.lock_integrator = lock_integrator;

				/* Run attitude controllers */
				if (isfinite(roll_sp) && isfinite(pitch_sp)) {
					_roll_ctrl.control_attitude(control_input);
					_pitch_ctrl.control_attitude(control_input);
					_yaw_ctrl.control_attitude(control_input); //runs last, because is depending on output of roll and pitch attitude

					/* Update input data for rate controllers */
					control_input.roll_rate_setpoint = _roll_ctrl.get_desired_rate();
					control_input.pitch_rate_setpoint = _pitch_ctrl.get_desired_rate();
					control_input.yaw_rate_setpoint = _yaw_ctrl.get_desired_rate();

					/* Run attitude RATE controllers which need the desired attitudes from above, add trim */
					float roll_u = _roll_ctrl.control_bodyrate(control_input);
					_actuators.control[0] = (isfinite(roll_u)) ? roll_u + _parameters.trim_roll : _parameters.trim_roll;
					if (!isfinite(roll_u)) {
						_roll_ctrl.reset_integrator();
						perf_count(_nonfinite_output_perf);

						if (_debug && loop_counter % 10 == 0) {
							warnx("roll_u %.4f", (double)roll_u);
						}
					}

					float pitch_u = _pitch_ctrl.control_bodyrate(control_input);
					_actuators.control[1] = (isfinite(pitch_u)) ? pitch_u + _parameters.trim_pitch : _parameters.trim_pitch;
					if (!isfinite(pitch_u)) {
						_pitch_ctrl.reset_integrator();
						perf_count(_nonfinite_output_perf);
						if (_debug && loop_counter % 10 == 0) {
							warnx("pitch_u %.4f, _yaw_ctrl.get_desired_rate() %.4f,"
								" airspeed %.4f, airspeed_scaling %.4f,"
								" roll_sp %.4f, pitch_sp %.4f,"
								" _roll_ctrl.get_desired_rate() %.4f,"
								" _pitch_ctrl.get_desired_rate() %.4f"
								" att_sp.roll_body %.4f",
								(double)pitch_u, (double)_yaw_ctrl.get_desired_rate(),
								(double)airspeed, (double)airspeed_scaling,
								(double)roll_sp, (double)pitch_sp,
								(double)_roll_ctrl.get_desired_rate(),
								(double)_pitch_ctrl.get_desired_rate(),
								(double)_att_sp.roll_body);
						}
					}

					float yaw_u = _yaw_ctrl.control_bodyrate(control_input);
					_actuators.control[2] = (isfinite(yaw_u)) ? yaw_u + _parameters.trim_yaw : _parameters.trim_yaw;

					/* add in manual rudder control */
					_actuators.control[2] += yaw_manual;
					if (!isfinite(yaw_u)) {
						_yaw_ctrl.reset_integrator();
						perf_count(_nonfinite_output_perf);
						if (_debug && loop_counter % 10 == 0) {
							warnx("yaw_u %.4f", (double)yaw_u);
						}
					}

					/* throttle passed through if it is finite and if no engine failure was
					 * detected */
					_actuators.control[3] = (isfinite(throttle_sp) &&
							!(_vehicle_status.engine_failure ||
								_vehicle_status.engine_failure_cmd)) ?
						throttle_sp : 0.0f;
					if (!isfinite(throttle_sp)) {
						if (_debug && loop_counter % 10 == 0) {
							warnx("throttle_sp %.4f", (double)throttle_sp);
						}
					}
				} else {
					perf_count(_nonfinite_input_perf);
					if (_debug && loop_counter % 10 == 0) {
						warnx("Non-finite setpoint roll_sp: %.4f, pitch_sp %.4f", (double)roll_sp, (double)pitch_sp);
					}
				}

				/*
				 * Lazily publish the rate setpoint (for analysis, the actuators are published below)
				 * only once available
				 */
				_rates_sp.roll = _roll_ctrl.get_desired_rate();
				_rates_sp.pitch = _pitch_ctrl.get_desired_rate();
				_rates_sp.yaw = _yaw_ctrl.get_desired_rate();

				_rates_sp.timestamp = hrt_absolute_time();

				if (_rate_sp_pub > 0) {
					/* publish the attitude rates setpoint */
					orb_publish(_rates_sp_id, _rate_sp_pub, &_rates_sp);
				} else if (_rates_sp_id) {
					/* advertise the attitude rates setpoint */
					_rate_sp_pub = orb_advertise(_rates_sp_id, &_rates_sp);
				}

			} else {
				/* manual/direct control */
				_actuators.control[actuator_controls_s::INDEX_ROLL] = _manual.y + _parameters.trim_roll;
				_actuators.control[actuator_controls_s::INDEX_PITCH] = -_manual.x + _parameters.trim_pitch;
				_actuators.control[actuator_controls_s::INDEX_YAW] = _manual.r + _parameters.trim_yaw;
				_actuators.control[actuator_controls_s::INDEX_THROTTLE] = _manual.z;
			}

			_actuators.control[actuator_controls_s::INDEX_FLAPS] = flaps_control;
			_actuators.control[5] = _manual.aux1;
			_actuators.control[6] = _manual.aux2;
			_actuators.control[7] = _manual.aux3;

			/* lazily publish the setpoint only once available */
			_actuators.timestamp = hrt_absolute_time();
			_actuators.timestamp_sample = _att.timestamp;
			_actuators_airframe.timestamp = hrt_absolute_time();
			_actuators_airframe.timestamp_sample = _att.timestamp;

			/* Only publish if any of the proper modes are enabled */
			if(_vcontrol_mode.flag_control_rates_enabled ||
			   _vcontrol_mode.flag_control_attitude_enabled ||
			   _vcontrol_mode.flag_control_manual_enabled)
			{
				/* publish the actuator controls */
				if (_actuators_0_pub > 0) {
					orb_publish(_actuators_id, _actuators_0_pub, &_actuators);
				} else if (_actuators_id) {
					_actuators_0_pub= orb_advertise(_actuators_id, &_actuators);
				}

				if (_actuators_2_pub > 0) {
					/* publish the actuator controls*/
					orb_publish(ORB_ID(actuator_controls_2), _actuators_2_pub, &_actuators_airframe);

				} else {
					/* advertise and publish */
					_actuators_2_pub = orb_advertise(ORB_ID(actuator_controls_2), &_actuators_airframe);
				}
			}
		}

		loop_counter++;
		perf_end(_loop_perf);
	}

	warnx("exiting.\n");

	_control_task = -1;
	_task_running = false;
	_exit(0);
}

int
FixedwingAttitudeControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = task_spawn_cmd("fw_att_control",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_MAX - 5,
				       1600,
				       (main_t)&FixedwingAttitudeControl::task_main_trampoline,
				       nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int fw_att_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		errx(1, "usage: fw_att_control {start|stop|status}");
	}

	if (!strcmp(argv[1], "start")) {

		if (att_control::g_control != nullptr)
			errx(1, "already running");

		att_control::g_control = new FixedwingAttitudeControl;

		if (att_control::g_control == nullptr)
			errx(1, "alloc failed");

		if (OK != att_control::g_control->start()) {
			delete att_control::g_control;
			att_control::g_control = nullptr;
			err(1, "start failed");
		}

		/* check if the waiting is necessary at all */
		if (att_control::g_control == nullptr || !att_control::g_control->task_running()) {

			/* avoid memory fragmentation by not exiting start handler until the task has fully started */
			while (att_control::g_control == nullptr || !att_control::g_control->task_running()) {
				usleep(50000);
				printf(".");
				fflush(stdout);
			}
			printf("\n");
		}
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (att_control::g_control == nullptr)
			errx(1, "not running");

		delete att_control::g_control;
		att_control::g_control = nullptr;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (att_control::g_control) {
			errx(0, "running");

		} else {
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	return 1;
}
