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
 * Implementation of a generic attitude controller based on classic orthogonal PIDs.
 *
 */

#ifndef STICKMAPPER_HPP_
#define STICKMAPPER_HPP_

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_workqueue.h>

#include <controllib/blocks.hpp>
#include <drivers/drv_hrt.h>

#include <mathlib/mathlib.h>

// uORB Subscriptions
#include <uORB/Subscription.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>

// uORB Publications
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>

using namespace control;

class StickMapper : public control::SuperBlock
{
public:
	StickMapper();

	virtual ~StickMapper();

	// get the work queue going.
	void start();
	void stop();

	bool is_running() const { return _taskIsRunning; }

protected:
	// Update our local parameter cache.
	virtual void parameters_update();

	//virtual void set_params_manual_scale(float roll_scale, float pitch_scale, float yaw_scale) = 0;
	//virtual void set_params_acro(float roll_rate_max, float pitch_rate_max, float yaw_rate_max) = 0;

	// manual input scaling
	float  _man_roll_scale{0.0f};			/**< scale factor applied to roll actuator control in pure manual mode */
	float  _man_pitch_scale{0.0f};			/**< scale factor applied to pitch actuator control in pure manual mode */
	float  _man_yaw_scale{0.0f}; 			/**< scale factor applied to yaw actuator control in pure manual mode */

	float _throttle_level{0.0f};

	// acro maximum rates
	float _acro_roll_max{0.0f};
	float _acro_pitch_max{0.0f};
	float _acro_yaw_max{0.0f};
	float _acro_thrust_max{0.0f};

	// attitude
	float _man_roll_max{0.0f};
	float _man_pitch_max{0.0f};

	// roll and pitch offsets
	float _roll_offset{0.0f};
	float _pitch_offset{0.0f};

private:
	static void cycle_trampoline(void *arg);

	void cycle();

	StickMapper(StickMapper *att_controller);
	StickMapper(const StickMapper &) = delete;
	StickMapper &operator=(const StickMapper &) = delete;

	// publish
	void publish_actuator_controls();
	void publish_vehicle_rates_setpoint();
	void publish_vehicle_attitude_setpoint();

	float throttle_curve(float ctl, float ctr);

	// run main stick mapper loop at this rate in Hz
	static constexpr uint32_t STICK_MAPPER_UPDATE_RATE_HZ = 50;

	bool _taskShouldExit{false};
	bool _taskIsRunning{false};

	struct work_s _work {};

	enum {POLL_MANUAL_CONTROL, POLL_CONTROL_MODE, n_poll};
	px4_pollfd_struct_t _polls[n_poll];

	hrt_abstime _timestamp{0};

	// uORB Subscriptions
	uORB::Subscription<manual_control_setpoint_s> _manual_control_setpoint_sub;
	uORB::Subscription<parameter_update_s> _parameter_update_sub;
	uORB::Subscription<vehicle_control_mode_s> _vehicle_control_mode_sub;			/**< vehicle control mode */
	uORB::Subscription<vehicle_status_s> _vehicle_status_sub;				/**< vehicle status */

	// uORB Publications
	uORB::Publication<actuator_controls_s> _actuator_controls_0_pub;		/**< actuator control inputs */
	uORB::Publication<vehicle_attitude_setpoint_s> _vehicle_attitude_setpoint_pub;	/* attitude rates setpoint */
	uORB::Publication<vehicle_rates_setpoint_s> _vehicle_rates_setpoint_pub;	/* attitude rates setpoint */

	// trim
	BlockParamFloat  _trim_roll;
	BlockParamFloat  _trim_pitch;
	BlockParamFloat  _trim_yaw;

};

#endif /* STICKMAPPER_HPP_ */
