/****************************************************************************
 *
 *   Copyright (c) 2012-2021 PX4 Development Team. All rights reserved.
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

#include <float.h>
#include <math.h>

#include <board_config.h>

#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_mixer.h>
#include <drivers/drv_pwm_output.h>
#include <lib/cdev/CDev.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <px4_arch/io_timer.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/parameter_update.h>

using namespace time_literals;

#if !defined(DIRECT_PWM_OUTPUT_CHANNELS)
#  error "board_config.h needs to define DIRECT_PWM_OUTPUT_CHANNELS"
#endif

#define PX4FMU_DEVICE_PATH	"/dev/px4fmu"

static constexpr int PWM_OUT_MAX_INSTANCES{(DIRECT_PWM_OUTPUT_CHANNELS > 8) ? 2 : 1};
extern pthread_mutex_t pwm_out_module_mutex;

class PWMOut : public cdev::CDev, public OutputModuleInterface
{
public:
	PWMOut() = delete;
	explicit PWMOut(int instance = 0, uint8_t output_base = 0);

	virtual ~PWMOut();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	void Run() override;

	/** @see ModuleBase::print_status() */
	int print_status();

	bool should_exit() const { return _task_should_exit.load(); }
	void request_stop() { _task_should_exit.store(true); }

	static void lock_module() { pthread_mutex_lock(&pwm_out_module_mutex); }
	static bool trylock_module() { return (pthread_mutex_trylock(&pwm_out_module_mutex) == 0); }
	static void unlock_module() { pthread_mutex_unlock(&pwm_out_module_mutex); }

	static int test(const char *dev);

	int ioctl(device::file_t *filp, int cmd, unsigned long arg) override;

	int init() override;

	uint32_t	get_pwm_mask() const { return _pwm_mask; }
	void		set_pwm_mask(uint32_t mask) { _pwm_mask = mask; }
	uint32_t	get_alt_rate_channels() const { return _pwm_alt_rate_channels; }
	unsigned	get_alt_rate() const { return _pwm_alt_rate; }
	unsigned	get_default_rate() const { return _pwm_default_rate; }

	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

private:
	static constexpr int FMU_MAX_ACTUATORS = DIRECT_PWM_OUTPUT_CHANNELS;
	static_assert(FMU_MAX_ACTUATORS <= MAX_ACTUATORS, "Increase MAX_ACTUATORS if this fails");

	px4::atomic_bool _task_should_exit{false};

	const int _instance;
	const uint32_t _output_base;

	static const int MAX_PER_INSTANCE{8};

	MixingOutput _mixing_output {PARAM_PREFIX, FMU_MAX_ACTUATORS, *this, MixingOutput::SchedulingPolicy::Auto, true};

	uint32_t	_backup_schedule_interval_us{1_s};

	unsigned	_pwm_default_rate{50};
	unsigned	_pwm_alt_rate{50};
	uint32_t	_pwm_alt_rate_channels{0};

	int _timer_rates[MAX_IO_TIMERS] {};

	int		_current_update_rate{0};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	unsigned	_num_outputs{0};
	int		_class_instance{-1};

	bool		_pwm_on{false};
	uint32_t	_pwm_mask{0};
	bool		_pwm_initialized{false};

	unsigned	_num_disarmed_set{0};

	perf_counter_t	_cycle_perf;
	perf_counter_t	_interval_perf;

	void		update_current_rate();
	int			set_pwm_rate(unsigned rate_map, unsigned default_rate, unsigned alt_rate);
	int			pwm_ioctl(device::file_t *filp, int cmd, unsigned long arg);

	bool		update_pwm_out_state(bool on);

	void		update_params();

	PWMOut(const PWMOut &) = delete;
	PWMOut operator=(const PWMOut &) = delete;

};
