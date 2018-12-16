/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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

#pragma once

#include <string.h>

#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_mixer.h>
#include <drivers/drv_pwm_output.h>
#include <lib/mixer/mixer.h>
#include <perf/perf_counter.h>
#include <px4_common.h>
#include <px4_config.h>
#include <px4_module.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/parameter_update.h>

class PWMSim : public cdev::CDev, public ModuleBase<PWMSim>
{
	static constexpr uint32_t PWM_SIM_DISARMED_MAGIC = 900;
	static constexpr uint32_t PWM_SIM_FAILSAFE_MAGIC = 600;
	static constexpr uint32_t PWM_SIM_PWM_MIN_MAGIC = 1000;
	static constexpr uint32_t PWM_SIM_PWM_MAX_MAGIC = 2000;

public:

	enum Mode {
		MODE_8PWM,
		MODE_16PWM,
		MODE_NONE
	};

	PWMSim();
	virtual ~PWMSim();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static PWMSim *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;


	int		ioctl(device::file_t *filp, int cmd, unsigned long arg) override;

	int		set_pwm_rate(unsigned rate);

	int		set_mode(Mode mode);
	Mode	get_mode() { return _mode; }

private:
	static constexpr unsigned MAX_ACTUATORS = 16;

	Mode		_mode{MODE_NONE};

	int 		_update_rate{400};
	int 		_current_update_rate{0};

	int			_control_subs[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];

	px4_pollfd_struct_t	_poll_fds[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS] {};
	unsigned	_poll_fds_num{0};

	int		_armed_sub{-1};

	actuator_outputs_s _actuator_outputs = {};
	orb_advert_t	_outputs_pub{nullptr};
	orb_advert_t	_mixer_status{nullptr};

	unsigned	_num_outputs{0};

	unsigned 	_pwm_min[MAX_ACTUATORS] {};
	unsigned 	_pwm_max[MAX_ACTUATORS] {};

	uint32_t	_groups_required{0};
	uint32_t	_groups_subscribed{0};

	bool	_armed{false};
	bool	_lockdown{false};
	bool	_failsafe{false};

	MixerGroup	*_mixers{nullptr};

	actuator_controls_s _controls[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS] {};
	orb_id_t	_control_topics[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS] {};

	Mixer::Airmode 	_airmode{Mixer::Airmode::disabled}; 	///< multicopter air-mode

	perf_counter_t	_perf_control_latency;

	static int	control_callback(uintptr_t handle, uint8_t control_group, uint8_t control_index, float &input);

	void 	subscribe();

	void 	update_params();
};

