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
 * @file mc_att_control.h
 * Multicopter attitude controller.
 *
 * @author Tobias Naegeli <naegelit@student.ethz.ch>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 * @author Julian Oes <julian@oes.ch>
 * @author Roman Bapst <bapstr@ethz.ch>
 *
 * The controller has two loops: P loop for angular error and PD loop for angular rate error.
 * Desired rotation calculated keeping in mind that yaw response is normally slower than roll/pitch.
 * For small deviations controller rotates copter to have shortest path of thrust vector and independently rotates around yaw,
 * so actual rotation axis is not constant. For large deviations controller rotates copter around fixed axis.
 * These two approaches fused seamlessly with weight depending on angular error.
 * When thrust vector directed near-horizontally (e.g. roll ~= PI/2) yaw setpoint ignored because of singularity.
 * Controller doesn't use Euler angles for work, they generated only for more human-friendly control and logging.
 * If rotation matrix setpoint is invalid it will be generated from Euler angles for compatibility with old position controllers.
 */

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <systemlib/perf_counter.h>
#include <systemlib/circuit_breaker.h>
#include <lib/mathlib/mathlib.h>

#include "mc_att_control_base.h"

class MulticopterAttitudeControlMultiplatform :
	public MulticopterAttitudeControlBase
{
public:
	/**
	 * Constructor
	 */
	MulticopterAttitudeControlMultiplatform();

	/**
	 * Destructor, also kills the sensors task.
	 */
	~MulticopterAttitudeControlMultiplatform();

	/* Callbacks for topics */
	void handle_vehicle_attitude(const px4_vehicle_attitude &msg);
	void handle_parameter_update(const px4_parameter_update &msg);

	void spin() { _n.spin(); }

private:
	bool		_actuators_0_circuit_breaker_enabled;	/**< circuit breaker to suppress output */

	px4::Publisher<px4_vehicle_rates_setpoint> 	*_v_rates_sp_pub;		/**< rate setpoint publication */
	px4::Publisher<px4_actuator_controls_0> 	*_actuators_0_pub;		/**< attitude actuator controls publication */

	px4::NodeHandle _n;

	px4::AppState _appState;

	struct {
		px4::ParameterFloat roll_p;
		px4::ParameterFloat roll_rate_p;
		px4::ParameterFloat roll_rate_i;
		px4::ParameterFloat roll_rate_d;
		px4::ParameterFloat pitch_p;
		px4::ParameterFloat pitch_rate_p;
		px4::ParameterFloat pitch_rate_i;
		px4::ParameterFloat pitch_rate_d;
		px4::ParameterFloat yaw_p;
		px4::ParameterFloat yaw_rate_p;
		px4::ParameterFloat yaw_rate_i;
		px4::ParameterFloat yaw_rate_d;
		px4::ParameterFloat yaw_ff;
		px4::ParameterFloat yaw_rate_max;

		px4::ParameterFloat acro_roll_max;
		px4::ParameterFloat acro_pitch_max;
		px4::ParameterFloat acro_yaw_max;

	}		_params_handles;		/**< handles for interesting parameters */

	perf_counter_t _loop_perf; /**< loop performance counter */

	/**
	 * Update our local parameter cache.
	 */
	int			parameters_update();
};

