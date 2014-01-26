/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file mavlink_orb_listener.h
 * MAVLink 1.0 protocol interface definition.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#pragma once

class Mavlink;

class MavlinkOrbListener
{
public:
	/**
	 * Constructor
	 */
	MavlinkOrbListener(Mavlink* parent);

	/**
	 * Destructor, also kills the mavlinks task.
	 */
	~MavlinkOrbListener();

	// *
	// * Start the mavlink task.
	//  *
	//  * @return		OK on success.
	 
	// int		start();

	/**
	 * Display the mavlink status.
	 */
	void		status();

	pthread_t uorb_receive_start(void);
	void * uorb_receive_thread(void *arg);

private:

	bool		_task_should_exit;		/**< if true, sensor task should exit */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	Mavlink*	_mavlink;

	unsigned	_n_listeners;

	/**
	 * Shim for calling task_main from task_create.
	 */
	void		task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main sensor collection task.
	 */
	void		task_main() __attribute__((noreturn));

	void		l_sensor_combined(const struct listener *l);
	void		l_vehicle_attitude(const struct listener *l);
	void		l_vehicle_gps_position(const struct listener *l);
	void		l_vehicle_status(const struct listener *l);
	void		l_rc_channels(const struct listener *l);
	void		l_input_rc(const struct listener *l);
	void		l_global_position(const struct listener *l);
	void		l_local_position(const struct listener *l);
	void		l_global_position_setpoint(const struct listener *l);
	void		l_local_position_setpoint(const struct listener *l);
	void		l_attitude_setpoint(const struct listener *l);
	void		l_actuator_outputs(const struct listener *l);
	void		l_actuator_armed(const struct listener *l);
	void		l_manual_control_setpoint(const struct listener *l);
	void		l_vehicle_attitude_controls(const struct listener *l);
	void		l_debug_key_value(const struct listener *l);
	void		l_optical_flow(const struct listener *l);
	void		l_vehicle_rates_setpoint(const struct listener *l);
	void		l_home(const struct listener *l);
	void		l_airspeed(const struct listener *l);
	void		l_nav_cap(const struct listener *l);
	void		l_control_mode(const struct listener *l);

	struct vehicle_global_position_s global_pos;
	struct vehicle_local_position_s local_pos;
	struct navigation_capabilities_s nav_cap;
	struct vehicle_status_s v_status;
	struct vehicle_control_mode_s control_mode;
	struct rc_channels_s rc;
	struct rc_input_values rc_raw;
	struct actuator_armed_s armed;
	struct actuator_controls_s actuators_0;
	struct vehicle_attitude_s att;
	struct airspeed_s airspeed;

	int status_sub;
	int rc_sub;

	unsigned int sensors_raw_counter;
	unsigned int attitude_counter;
	unsigned int gps_counter;

	/*
	 * Last sensor loop time
	 * some outputs are better timestamped
	 * with this "global" reference.
	 */
	uint64_t last_sensor_timestamp;

	hrt_abstime last_sent_vfr;

};
