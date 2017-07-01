/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * All the acknowledgments and credits for the fw wing app are reported in those files.
 *
 * @author Marco Zorzi <mzorzi@student.ethz.ch>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>

#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <systemlib/param/param.h>
#include <systemlib/pid/pid.h>
#include <systemlib/perf_counter.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/uORB.h>

using matrix::Eulerf;
using matrix::Quatf;

class GroundRoverAttitudeControl
{
public:
	GroundRoverAttitudeControl();
	~GroundRoverAttitudeControl();

	int start();
	bool task_running() { return _task_running; }

private:

	bool		_task_should_exit{false};		/**< if true, attitude control task should exit */
	bool		_task_running{false};			/**< if true, task is running in its mainloop */
	int		_control_task{-1};			/**< task handle */

	int		_att_sp_sub{-1};			/**< vehicle attitude setpoint */
	int		_battery_status_sub{-1};		/**< battery status subscription */
	int		_att_sub{-1};		/**< control state subscription */
	int		_manual_sub{-1};			/**< notification of manual control updates */
	int		_params_sub{-1};			/**< notification of parameter updates */
	int		_vcontrol_mode_sub{-1};		/**< vehicle status subscription */

	orb_advert_t	_actuators_0_pub{nullptr};		/**< actuator control group 0 setpoint */

	actuator_controls_s			_actuators {};		/**< actuator control inputs */
	battery_status_s				_battery_status {};	/**< battery status */
	manual_control_setpoint_s		_manual {};		/**< r/c channel data */
	vehicle_attitude_s				_att {};	/**< control state */
	vehicle_attitude_setpoint_s		_att_sp {};		/**< vehicle attitude setpoint */
	vehicle_control_mode_s			_vcontrol_mode {};		/**< vehicle control mode */

	perf_counter_t	_loop_perf;			/**< loop performance counter */
	perf_counter_t	_nonfinite_input_perf;		/**< performance counter for non finite input */
	perf_counter_t	_nonfinite_output_perf;		/**< performance counter for non finite output */

	bool		_debug{false};				/**< if set to true, print debug output */

	struct {
		float w_p;		/**< Proportional gain of the steering controller */
		float w_i;		/**< Integral gain of the steering controller */
		float w_d;		/**< Derivative of the steering controller */
		float w_imax;		/**< maximum integrator level of the steering controller */

		float trim_yaw;
		float man_yaw_scale; 			/**< scale factor applied to yaw actuator control in pure manual mode */

		int32_t bat_scale_en;			/**< Battery scaling enabled */

	} _parameters{};			/**< local copies of interesting parameters */

	struct {
		param_t w_p;
		param_t w_i;
		param_t w_d;
		param_t w_imax;

		param_t trim_yaw;
		param_t man_yaw_scale;

		param_t bat_scale_en;

	} _parameter_handles{};		/**< handles for interesting parameters */

	PID_t			_steering_ctrl{};

	void		parameters_update();

	void		vehicle_control_mode_poll();
	void		manual_control_setpoint_poll();
	void		vehicle_attitude_setpoint_poll();
	void		battery_status_poll();

	static void	task_main_trampoline(int argc, char *argv[]);
	void		task_main();

};
