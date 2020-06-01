/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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

#include <lib/mixer/Mixer/Mixer.hpp> // Airmode
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_land_detected.h>

/**
 * Airship attitude control app start / stop handling function
 */
extern "C" __EXPORT int airship_att_control_main(int argc, char *argv[]);

class AirshipAttitudeControl : public ModuleBase<AirshipAttitudeControl>, public ModuleParams,
	public px4::WorkItem
{
public:
	AirshipAttitudeControl();

	virtual ~AirshipAttitudeControl();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	void Run() override;

	bool init();

private:

	/**
	 * initialize some vectors/matrices from parameters
	 */
	void		parameters_updated();

	/**
	 * Check for parameter update and handle it.
	 */
	void		parameter_update_poll();
	bool		vehicle_attitude_poll();
	void		vehicle_status_poll();

	void		publish_actuator_controls();
	void		publish_rates_setpoint();
	void		publish_rate_controller_status();

	uORB::Subscription _v_att_sub{ORB_ID(vehicle_attitude)};			/**< vehicle attitude subscription */
	uORB::Subscription _v_att_sp_sub{ORB_ID(vehicle_attitude_setpoint)};		/**< vehicle attitude setpoint subscription */
	uORB::Subscription _v_rates_sp_sub{ORB_ID(vehicle_rates_setpoint)};		/**< vehicle rates setpoint subscription */
	uORB::Subscription _v_control_mode_sub{ORB_ID(vehicle_control_mode)};		/**< vehicle control mode subscription */
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};		/**< parameter updates subscription */
	uORB::Subscription _manual_control_sp_sub{ORB_ID(manual_control_setpoint)};	/**< manual control setpoint subscription */
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};			/**< vehicle status subscription */
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};	/**< vehicle land detected subscription */

	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};

	uORB::PublicationMulti<rate_ctrl_status_s>	_controller_status_pub{ORB_ID(rate_ctrl_status), ORB_PRIO_DEFAULT};	/**< controller status publication */
	uORB::Publication<vehicle_rates_setpoint_s>	_v_rates_sp_pub{ORB_ID(vehicle_rates_setpoint)};			/**< rate setpoint publication */

	orb_advert_t	_actuators_0_pub{nullptr};		/**< attitude actuator controls publication */

	orb_id_t _actuators_id{nullptr};	/**< pointer to correct actuator controls0 uORB metadata structure */
	orb_id_t _attitude_sp_id{nullptr};	/**< pointer to correct attitude setpoint uORB metadata structure */

	bool		_actuators_0_circuit_breaker_enabled{false};	/**< circuit breaker to suppress output */

	struct vehicle_attitude_s		_v_att {};		/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s	_v_att_sp {};		/**< vehicle attitude setpoint */
	struct vehicle_rates_setpoint_s		_v_rates_sp {};		/**< vehicle rates setpoint */
	struct manual_control_setpoint_s	_manual_control_sp {};	/**< manual control setpoint */
	struct vehicle_control_mode_s		_v_control_mode {};	/**< vehicle control mode */
	struct actuator_controls_s		_actuators {};		/**< actuator controls */
	struct vehicle_status_s			_vehicle_status {};	/**< vehicle status */
	struct vehicle_land_detected_s		_vehicle_land_detected {};

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	static constexpr const float initial_update_rate_hz = 250.f; /**< loop update rate used for initialization */
	float _loop_update_rate_hz{initial_update_rate_hz};          /**< current rate-controller loop update rate in [Hz] */

	matrix::Vector3f _rates_sp;			/**< angular rates setpoint */

	float		_thrust_sp{0.0f};		/**< thrust setpoint */
	float _man_yaw_sp{0.f};				/**< current yaw setpoint in manual mode */

	hrt_abstime _task_start{hrt_absolute_time()};
	hrt_abstime _last_run{0};
	float _dt_accumulator{0.0f};
	int _loop_counter{0};

};
