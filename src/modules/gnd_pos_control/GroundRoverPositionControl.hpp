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
 * All the ackowledgments and credits for the fw wing app are reported in those files.
 *
 * @author Marco Zorzi <mzorzi@student.ethz.ch>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <px4_tasks.h>

#include <cfloat>

#include <drivers/drv_hrt.h>
#include <ecl/l1/ecl_l1_pos_controller.h>
#include <geo/geo.h>
#include <mathlib/mathlib.h>
#include <systemlib/perf_counter.h>
#include <systemlib/pid/pid.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/fw_pos_ctrl_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/sensor_corrected.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/uORB.h>

using matrix::Dcmf;

using uORB::Subscription;

class GroundRoverPositionControl
{
public:
	GroundRoverPositionControl();
	~GroundRoverPositionControl();
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
	orb_advert_t	_attitude_sp_pub{nullptr};		/**< attitude setpoint */
	orb_advert_t	_gnd_pos_ctrl_status_pub{nullptr};		/**< navigation capabilities publication */

	bool		_task_should_exit{false};		/**< if true, sensor task should exit */
	bool		_task_running{false};			/**< if true, task is running in its mainloop */

	int		_control_mode_sub{-1};		/**< control mode subscription */
	int		_global_pos_sub{-1};
	int		_manual_control_sub{-1};		/**< notification of manual control updates */
	int		_params_sub{-1};			/**< notification of parameter updates */
	int		_pos_sp_triplet_sub{-1};

	fw_pos_ctrl_status_s			_gnd_pos_ctrl_status{};		/**< navigation capabilities */
	manual_control_setpoint_s		_manual{};			/**< r/c channel data */
	position_setpoint_triplet_s		_pos_sp_triplet{};		/**< triplet of mission items */
	vehicle_attitude_setpoint_s		_att_sp{};			/**< vehicle attitude setpoint */
	vehicle_control_mode_s			_control_mode{};			/**< control mode */
	vehicle_global_position_s		_global_pos{};			/**< global vehicle position */

	Subscription<vehicle_attitude_s>	_sub_attitude;
	Subscription<sensor_corrected_s>	_sub_sensors;

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	hrt_abstime _control_position_last_called{0}; 	/**<last call of control_position  */

	/* Pid controller for the speed. Here we assume we can control airspeed but the control variable is actually on
	 the throttle. For now just assuming a proportional scaler between controlled airspeed and throttle output.*/
	PID_t _speed_ctrl{};

	// estimator reset counters
	uint8_t _pos_reset_counter{0};		// captures the number of times the estimator has reset the horizontal position

	ECL_L1_Pos_Controller				_gnd_control;

	enum UGV_POSCTRL_MODE {
		UGV_POSCTRL_MODE_AUTO,
		UGV_POSCTRL_MODE_OTHER
	} _control_mode_current{UGV_POSCTRL_MODE_OTHER};			///< used to check the mode in the last control loop iteration. Use to check if the last iteration was in the same mode.

	struct {
		float l1_period;
		float l1_damping;
		float l1_distance;

		float gndspeed_trim;
		float gndspeed_max;

		int32_t speed_control_mode;
		float speed_p;
		float speed_i;
		float speed_d;
		float speed_imax;
		float throttle_speed_scaler;

		float throttle_min;
		float throttle_max;
		float throttle_cruise;
		float throttle_slew_max;

	} _parameters{};			/**< local copies of interesting parameters */

	struct {
		param_t l1_period;
		param_t l1_damping;
		param_t l1_distance;

		param_t gndspeed_trim;
		param_t gndspeed_max;

		param_t speed_control_mode;

		param_t speed_p;
		param_t speed_i;
		param_t speed_d;
		param_t speed_imax;
		param_t throttle_speed_scaler;

		param_t throttle_min;
		param_t throttle_max;
		param_t throttle_cruise;
		param_t throttle_slew_max;

	} _parameter_handles{};		/**< handles for interesting parameters */


	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	void		manual_control_setpoint_poll();
	void		position_setpoint_triplet_poll();
	void		vehicle_control_mode_poll();

	/**
	 * Publish navigation capabilities
	 */
	void		gnd_pos_ctrl_status_publish();

	/**
	 * Control position.
	 */
	bool		control_position(const math::Vector<2> &global_pos, const math::Vector<3> &ground_speed,
					 const position_setpoint_triplet_s &_pos_sp_triplet);

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main sensor collection task.
	 */
	void		task_main();

};
