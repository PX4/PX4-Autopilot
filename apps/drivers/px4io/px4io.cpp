/****************************************************************************
 *
 *   Copyright (C) 2012,2013 PX4 Development Team. All rights reserved.
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
 * @file px4io.cpp
 * Driver for the PX4IO board.
 *
 * PX4IO is connected via I2C.
 */

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <debug.h>
#include <time.h>
#include <queue.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>

#include <arch/board/board.h>

#include <drivers/device/device.h>
#include <drivers/device/i2c.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_gpio.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_mixer.h>

#include <systemlib/mixer/mixer.h>
#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <systemlib/scheduling_priorities.h>
#include <systemlib/param/param.h>

#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_controls_effective.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/parameter_update.h>

#include <px4io/protocol.h>
#include "uploader.h"


class PX4IO : public device::I2C
{
public:
	PX4IO();
	~PX4IO();

	virtual int		init();

	virtual int		ioctl(file *filp, int cmd, unsigned long arg);
	virtual ssize_t		write(file *filp, const char *buffer, size_t len);

private:
	// XXX
	unsigned		_max_actuators;
	unsigned		_max_rc_input;
	unsigned		_max_relays;
	unsigned		_max_transfer;

	unsigned 		_update_interval; ///< subscription interval limiting send rate

	volatile int		_task;		///< worker task
	volatile bool		_task_should_exit;
	volatile bool		_connected;	///< true once we have received a valid frame

	perf_counter_t		_perf_update;

	/* cached IO state */
	uint16_t		_status;
	uint16_t		_alarms;

	/* subscribed topics */
	int			_t_actuators;	///< actuator output topic
	int			_t_armed;	///< system armed control topic
	int 			_t_vstatus;	///< system / vehicle status
	int			_t_param;	///< parameter update topic

	/* advertised topics */
	orb_advert_t 		_to_input_rc;	///< rc inputs from io
	orb_advert_t 		_to_actuators_effective; ///< effective actuator controls topic
	orb_advert_t		_to_outputs;	///< mixed servo outputs topic
	orb_advert_t		_to_battery;	///< battery status / voltage

	actuator_outputs_s	_outputs;	///< mixed outputs
	actuator_controls_effective_s _controls_effective; ///< effective controls

	const char *volatile	_mix_buf;	///< mixer text buffer
	volatile unsigned	_mix_buf_len;	///< size of the mixer text buffer

	bool			_primary_pwm_device;	///< true if we are the default PWM output


	/**
	 * Trampoline to the worker task
	 */
	static void		task_main_trampoline(int argc, char *argv[]);

	/**
	 * worker task
	 */
	void			task_main();

	/**
	 * Send controls to IO
	 */
	int			io_set_control_state();

	/**
	 * Update IO's arming-related state
	 */
	int			io_set_arming_state();

	/**
	 * Push RC channel configuration to IO.
	 */
	int			io_set_rc_config();

	/**
	 * Fetch status and alarms from IO
	 *
	 * Also publishes battery voltage/current.
	 */
	int			io_get_status();

	/**
	 * Fetch RC inputs from IO.
	 *
	 * @param input_rc	Input structure to populate.
	 * @return		OK if data was returned.
	 */
	int			io_get_raw_rc_input(rc_input_values &input_rc);

	/**
	 * Fetch and publish raw RC input data.
	 */
	int			io_publish_raw_rc();

	/**
	 * Fetch and publish the mixed control values.
	 */
	int			io_publish_mixed_controls();

	/**
	 * Fetch and publish the PWM servo outputs.
	 */
	int			io_publish_pwm_outputs();

	/**
	 * write register(s)
	 *
	 * @param page		Register page to write to.
	 * @param offset	Register offset to start writing at.
	 * @param values	Pointer to array of values to write.
	 * @param num_values	The number of values to write.
	 * @return		Zero if all values were successfully written.
	 */
	int			io_reg_set(uint8_t page, uint8_t offset, const uint16_t *values, unsigned num_values);

	/**
	 * write a register
	 *
	 * @param page		Register page to write to.
	 * @param offset	Register offset to write to.
	 * @param value		Value to write.
	 * @return		Zero if the value was written successfully.
	 */
	int			io_reg_set(uint8_t page, uint8_t offset, const uint16_t value);

	/**
	 * read register(s)
	 *
	 * @param page		Register page to read from.
	 * @param offset	Register offset to start reading from.
	 * @param values	Pointer to array where values should be stored.
	 * @param num_values	The number of values to read.
	 * @return		Zero if all values were successfully read.
	 */
	int			io_reg_get(uint8_t page, uint8_t offset, uint16_t *values, unsigned num_values);

	/**
	 * read a register
	 *
	 * @param page		Register page to read from.
	 * @param offset	Register offset to start reading from.
	 * @return		Register value that was read, or _io_reg_get_error on error.
	 */
	uint32_t		io_reg_get(uint8_t page, uint8_t offset);
	static const uint32_t	_io_reg_get_error = 0x80000000;

	/**
	 * modify a register
	 *
	 * @param page		Register page to modify.
	 * @param offset	Register offset to modify.
	 * @param clearbits	Bits to clear in the register.
	 * @param setbits	Bits to set in the register.
	 */
	int			io_reg_modify(uint8_t page, uint8_t offset, uint16_t clearbits, uint16_t setbits);

	/**
	 * Send mixer definition text to IO
	 */
	int			mixer_send(const char *buf, unsigned buflen);

};


namespace
{

PX4IO	*g_dev;

}

PX4IO::PX4IO() :
	I2C("px4io", "/dev/px4io", PX4_I2C_BUS_ONBOARD, PX4_I2C_OBDEV_PX4IO, 320000),
	_max_actuators(0),
	_max_rc_input(0),
	_max_relays(0),
	_max_transfer(16),	/* sensible default */
	_update_interval(0),
	_task(-1),
	_task_should_exit(false),
	_connected(false),
	_perf_update(perf_alloc(PC_ELAPSED, "px4io update")),
	_t_actuators(-1),
	_t_armed(-1),
	_t_vstatus(-1),
	_to_input_rc(0),
	_to_actuators_effective(0),
	_to_outputs(0),
	_to_battery(0),
	_mix_buf(nullptr),
	_mix_buf_len(0),
	_primary_pwm_device(false)
{
	/* we need this potentially before it could be set in task_main */
	g_dev = this;

	_debug_enabled = true;
}

PX4IO::~PX4IO()
{
	/* tell the task we want it to go away */
	_task_should_exit = true;

	/* spin waiting for the task to stop */
	for (unsigned i = 0; (i < 10) && (_task != -1); i++) {
		/* give it another 100ms */
		usleep(100000);
	}

	/* well, kill it anyway, though this will probably crash */
	if (_task != -1)
		task_delete(_task);

	g_dev = nullptr;
}

int
PX4IO::init()
{
	int ret;

	ASSERT(_task == -1);

	/* do regular cdev init */
	ret = I2C::init();
	if (ret != OK)
		return ret;

	/* get some parameters */
	_max_actuators = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_ACTUATOR_COUNT);
	_max_relays    = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_RELAY_COUNT);
	_max_transfer  = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_MAX_TRANSFER);
	_max_rc_input  = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_RC_INPUT_COUNT);
	if ((_max_actuators < 1) || (_max_actuators == _io_reg_get_error) ||
	    (_max_relays < 1) || (_max_relays == _io_reg_get_error) ||
	    (_max_relays < 16) || (_max_relays == _io_reg_get_error) ||
	    (_max_rc_input < 1) || (_max_rc_input == _io_reg_get_error)) {

		log("failed getting parameters from PX4IO");
		return ret;
	}
	if (_max_rc_input > RC_INPUT_MAX_CHANNELS)
		_max_rc_input = RC_INPUT_MAX_CHANNELS;

	/* publish RC config to IO */
	ret = io_set_rc_config();
	if (ret != OK) {
		log("failed to update RC input config");
		return ret;
	}

	/* try to claim the generic PWM output device node as well - it's OK if we fail at this */
	ret = register_driver(PWM_OUTPUT_DEVICE_PATH, &fops, 0666, (void *)this);

	if (ret == OK) {
		log("default PWM output device");
		_primary_pwm_device = true;
	}

	/* start the IO interface task */
	_task = task_create("px4io", SCHED_PRIORITY_ACTUATOR_OUTPUTS, 4096, (main_t)&PX4IO::task_main_trampoline, nullptr);

	if (_task < 0) {
		debug("task start failed: %d", errno);
		return -errno;
	}

	/* wait a second for it to detect IO */
	for (unsigned i = 0; i < 10; i++) {
		if (_connected) {
			debug("PX4IO connected");
			break;
		}
		usleep(100000);
	}

	if (!_connected) {
		/* error here will result in everything being torn down */
		log("PX4IO not responding");
		return -EIO;
	}

	return OK;
}

void
PX4IO::task_main_trampoline(int argc, char *argv[])
{
	g_dev->task_main();
}

void
PX4IO::task_main()
{
	hrt_abstime last_poll_time = 0;

	log("starting");

	/*
	 * Subscribe to the appropriate PWM output topic based on whether we are the
	 * primary PWM output or not.
	 */
	_t_actuators = orb_subscribe(_primary_pwm_device ? ORB_ID_VEHICLE_ATTITUDE_CONTROLS :
				     ORB_ID(actuator_controls_1));
	orb_set_interval(_t_actuators, 20);		/* default to 50Hz */

	_t_armed = orb_subscribe(ORB_ID(actuator_armed));
	orb_set_interval(_t_armed, 200);		/* 5Hz update rate */

	_t_vstatus = orb_subscribe(ORB_ID(vehicle_status));
	orb_set_interval(_t_vstatus, 200);		/* 5Hz update rate max. */

	_t_param = orb_subscribe(ORB_ID(parameter_update));
	orb_set_interval(_t_param, 500);		/* 2Hz update rate max. */

	/* poll descriptor */
	pollfd fds[4];
	fds[0].fd = _t_actuators;
	fds[0].events = POLLIN;
	fds[1].fd = _t_armed;
	fds[1].events = POLLIN;
	fds[2].fd = _t_vstatus;
	fds[2].events = POLLIN;
	fds[3].fd = _t_param;
	fds[3].events = POLLIN;

	debug("ready");

	/* lock against the ioctl handler */
	lock();

	/* loop talking to IO */
	while (!_task_should_exit) {

		/* adjust update interval */
		if (_update_interval != 0) {
			if (_update_interval < 5)
				_update_interval = 5;
			if (_update_interval > 100)
				_update_interval = 100;
			orb_set_interval(_t_actuators, _update_interval);
			_update_interval = 0;
		}

		/* sleep waiting for topic updates, but no more than 100ms */
		unlock();
		int ret = ::poll(&fds[0], sizeof(fds) / sizeof(fds[0]), 100);
		lock();

		/* this would be bad... */
		if (ret < 0) {
			log("poll error %d", errno);
			continue;
		}

		/* if we have new control data from the ORB, handle it */
		if (fds[0].revents & POLLIN)
			io_set_control_state();

		/* if we have an arming state update, handle it */
		if ((fds[1].revents & POLLIN) || (fds[2].revents & POLLIN))
			io_set_arming_state();

		hrt_abstime now = hrt_absolute_time();

		/*
		 * If this isn't time for the next tick of the polling state machine,
		 * go back to sleep.
		 */
		if ((now - last_poll_time) < 20000)
			continue;

		/*
		 * Pull status and alarms from IO.
		 */
		io_get_status();

		/*
		 * Get R/C input from IO.
		 */
		io_publish_raw_rc();

		/*
		 * Fetch mixed servo controls and PWM outputs from IO.
		 *
		 * XXX We could do this at a reduced rate in many/most cases.
		 */
		io_publish_mixed_controls();
		io_publish_pwm_outputs();

		/*
		 * If parameters have changed, re-send RC mappings to IO
		 *
		 * XXX this may be a bit spammy
		 */
		if (fds[3].revents & POLLIN) {
			parameter_update_s pupdate;

			/* copy to reset the notification */
			orb_copy(ORB_ID(parameter_update), _t_param, &pupdate);

			/* re-upload RC input config as it may have changed */
			io_set_rc_config();
		}
	}

	unlock();

	debug("exiting");

	/* clean up the alternate device node */
	if (_primary_pwm_device)
		unregister_driver(PWM_OUTPUT_DEVICE_PATH);

	/* tell the dtor that we are exiting */
	_task = -1;
	_exit(0);
}

int
PX4IO::io_set_control_state()
{
	actuator_controls_s	controls;	///< actuator outputs
	uint16_t 		regs[_max_actuators];

	/* get controls */
	orb_copy(_primary_pwm_device ? ORB_ID_VEHICLE_ATTITUDE_CONTROLS :
	     ORB_ID(actuator_controls_1), _t_actuators, &controls);

	for (unsigned i = 0; i < _max_actuators; i++)
		regs[i] = FLOAT_TO_REG(controls.control[i]);

	/* copy values to registers in IO */
	return io_reg_set(PX4IO_PAGE_CONTROLS, 0, regs, _max_actuators);
}

int
PX4IO::io_set_arming_state()
{
	actuator_armed_s	armed;		///< system armed state
	vehicle_status_s	vstatus;	///< overall system state

	orb_copy(ORB_ID(actuator_armed), _t_armed, &armed);
	orb_copy(ORB_ID(vehicle_status), _t_vstatus, &vstatus);

	uint16_t set = 0;
	uint16_t clear = 0;

	if (armed.armed) {
		set |= PX4IO_P_SETUP_ARMING_ARM_OK;
	} else {
		clear |= PX4IO_P_SETUP_ARMING_ARM_OK;
	}
	if (vstatus.flag_vector_flight_mode_ok) {
		set |= PX4IO_P_SETUP_ARMING_VECTOR_FLIGHT_OK;
	} else {
		clear |= PX4IO_P_SETUP_ARMING_VECTOR_FLIGHT_OK;					
	}
	if (vstatus.flag_external_manual_override_ok) {
		set |= PX4IO_P_SETUP_ARMING_MANUAL_OVERRIDE;
	} else {
		clear |= PX4IO_P_SETUP_ARMING_MANUAL_OVERRIDE;
	}

	return io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, clear, set);
}

int
PX4IO::io_set_rc_config()
{
	unsigned offset = 0;
	int input_map[_max_rc_input];
	int32_t ichan;
	int ret = OK;

	/*
	 * Generate the input channel -> control channel mapping table;
	 * assign RC_MAP_ROLL/PITCH/YAW/THROTTLE to the canonical
	 * controls.
	 */
	for (unsigned i = 0; i < _max_rc_input; i++)
		input_map[i] = -1;

	param_get(param_find("RC_MAP_ROLL"), &ichan);
	if ((ichan >= 0) && (ichan < (int)_max_rc_input))
		input_map[ichan] = 0;

	param_get(param_find("RC_MAP_PITCH"), &ichan);
	if ((ichan >= 0) && (ichan < (int)_max_rc_input))
		input_map[ichan] = 1;

	param_get(param_find("RC_MAP_YAW"), &ichan);
	if ((ichan >= 0) && (ichan < (int)_max_rc_input))
		input_map[ichan] = 2;

	param_get(param_find("RC_MAP_THROTTLE"), &ichan);
	if ((ichan >= 0) && (ichan < (int)_max_rc_input))
		input_map[ichan] = 3;

	ichan = 4;
	for (unsigned i = 0; i < _max_rc_input; i++)
		if (input_map[i] == -1)
			input_map[i] = ichan++;

	/*
	 * Iterate all possible RC inputs.
	 */
	for (unsigned i = 0; i < _max_rc_input; i++) {
		uint16_t regs[PX4IO_P_RC_CONFIG_STRIDE];
		char pname[16];
		float fval;

		sprintf(pname, "RC%d_MIN", i + 1);
		param_get(param_find(pname), &fval);
		regs[PX4IO_P_RC_CONFIG_MIN] = FLOAT_TO_REG(fval);

		sprintf(pname, "RC%d_TRIM", i + 1);
		param_get(param_find(pname), &fval);
		regs[PX4IO_P_RC_CONFIG_CENTER] = FLOAT_TO_REG(fval);

		sprintf(pname, "RC%d_MAX", i + 1);
		param_get(param_find(pname), &fval);
		regs[PX4IO_P_RC_CONFIG_MAX] = FLOAT_TO_REG(fval);

		sprintf(pname, "RC%d_DZ", i + 1);
		param_get(param_find(pname), &fval);
		regs[PX4IO_P_RC_CONFIG_DEADZONE] = FLOAT_TO_REG(fval);

		regs[PX4IO_P_RC_CONFIG_ASSIGNMENT] = input_map[i];

		regs[PX4IO_P_RC_CONFIG_OPTIONS] = PX4IO_P_RC_CONFIG_OPTIONS_ENABLED;
		sprintf(pname, "RC%d_REV", i + 1);
		param_get(param_find(pname), &fval);
		if (fval > 0)
			regs[PX4IO_P_RC_CONFIG_OPTIONS] |= PX4IO_P_RC_CONFIG_OPTIONS_REVERSE;

		/* send channel config to IO */
		ret = io_reg_set(PX4IO_PAGE_RC_CONFIG, offset, regs, PX4IO_P_RC_CONFIG_STRIDE);
		if (ret != OK)
			break;
		offset += PX4IO_P_RC_CONFIG_STRIDE;
	}

	return ret;
}

int
PX4IO::io_get_status()
{
	uint16_t	regs[4];
	int		ret;

	/* get STATUS_FLAGS, STATUS_ALARMS, STATUS_VBATT, STATUS_IBATT in that order */
	ret = io_reg_get(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_FLAGS, regs, sizeof(regs) / sizeof(regs[0]));
	if (ret != OK)
		return ret;

	_status = regs[0];
	_alarms = regs[1];

	/* XXX handle status */

	/* XXX handle alarms */

	/* only publish if battery has a valid minimum voltage */
	if (regs[2] > 3300) {
		battery_status_s	battery_status;

		battery_status.timestamp = hrt_absolute_time();

		/* voltage is scaled to mV */
		battery_status.voltage_v = regs[2] / 1000.0f;

		/* current scaling should be to cA in order to avoid limiting at 65A */
		battery_status.current_a = regs[3] / 100.f;

		/* this requires integration over time - not currently implemented */
		battery_status.discharged_mah = -1.0f;

		/* lazily publish the battery voltage */
		if (_to_battery > 0) {
			orb_publish(ORB_ID(battery_status), _to_battery, &battery_status);
		} else {
			_to_battery = orb_advertise(ORB_ID(battery_status), &battery_status);
		}
	}
	return ret;
}

int
PX4IO::io_get_raw_rc_input(rc_input_values &input_rc)
{
	uint16_t channel_count;
	int	ret;

	input_rc.timestamp = hrt_absolute_time();

	/* we don't have the status bits, so input_source has to be set elsewhere */
	input_rc.input_source = RC_INPUT_SOURCE_UNKNOWN;
	
	/*
	 * XXX Because the channel count and channel data are fetched
	 *     separately, there is a risk of a race between the two
	 *     that could leave us with channel data and a count that 
	 *     are out of sync.
	 *     Fixing this would require a guarantee of atomicity from
	 *     IO, and a single fetch for both count and channels.
	 *
	 * XXX Since IO has the input calibration info, we ought to be
	 *     able to get the pre-fixed-up controls directly.
	 *
	 * XXX can we do this more cheaply? If we knew we had DMA, it would
	 *     almost certainly be better to just get all the inputs...
	 */
	ret = io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_COUNT, &channel_count, 1);
	if (ret != OK)
		return ret;
	input_rc.channel_count = channel_count;

	if (channel_count > 0)
		ret = io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_BASE, input_rc.values, channel_count);

	return ret;
}

int
PX4IO::io_publish_raw_rc()
{
	/* if no RC, just don't publish */
	if (!(_status & PX4IO_P_STATUS_FLAGS_RC_OK))
		return OK;

	/* fetch values from IO */
	rc_input_values	rc_val;
	rc_val.timestamp = hrt_absolute_time();

	int ret = io_get_raw_rc_input(rc_val);
	if (ret != OK)
		return ret;

	/* sort out the source of the values */
	if (_status & PX4IO_P_STATUS_FLAGS_RC_PPM) {
		rc_val.input_source = RC_INPUT_SOURCE_PX4IO_PPM;
	} else if (_status & PX4IO_P_STATUS_FLAGS_RC_DSM) {
		rc_val.input_source = RC_INPUT_SOURCE_PX4IO_SPEKTRUM;
	} else if (_status & PX4IO_P_STATUS_FLAGS_RC_SBUS) {
		rc_val.input_source = RC_INPUT_SOURCE_PX4IO_SBUS;
	} else {
		rc_val.input_source = RC_INPUT_SOURCE_UNKNOWN;
	}

	/* lazily advertise on first publication */
	if (_to_input_rc == 0) {
		_to_input_rc = orb_advertise(ORB_ID(input_rc), &rc_val);
	} else { 
		orb_publish(ORB_ID(input_rc), _to_input_rc, &rc_val);
	}

	return OK;
}

int
PX4IO::io_publish_mixed_controls()
{
	/* if no FMU comms(!) just don't publish */
	if (!(_status & PX4IO_P_STATUS_FLAGS_FMU_OK))
		return OK;

	/* if not taking raw PPM from us, must be mixing */
	if (_status & PX4IO_P_STATUS_FLAGS_RAW_PPM)
		return OK;

	/* data we are going to fetch */
	actuator_controls_effective_s controls_effective;
	controls_effective.timestamp = hrt_absolute_time();

	/* get actuator controls from IO */
	uint16_t act[_max_actuators];
	int ret = io_reg_get(PX4IO_PAGE_ACTUATORS, 0, act, _max_actuators);
	if (ret != OK)
		return ret;

	/* convert from register format to float */
	for (unsigned i = 0; i < _max_actuators; i++)
		controls_effective.control_effective[i] = REG_TO_FLOAT(act[i]);

	/* laxily advertise on first publication */
	if (_to_actuators_effective == 0) {
		_to_actuators_effective = 
			orb_advertise((_primary_pwm_device ? 
				ORB_ID_VEHICLE_ATTITUDE_CONTROLS_EFFECTIVE : 
				ORB_ID(actuator_controls_effective_1)),
					   &controls_effective);
	} else {
		orb_publish((_primary_pwm_device ? 
			ORB_ID_VEHICLE_ATTITUDE_CONTROLS_EFFECTIVE : 
			ORB_ID(actuator_controls_effective_1)),
			_to_actuators_effective, &controls_effective);
	}

	return OK;
}

int
PX4IO::io_publish_pwm_outputs()
{
	/* if no FMU comms(!) just don't publish */
	if (!(_status & PX4IO_P_STATUS_FLAGS_FMU_OK))
		return OK;

	/* data we are going to fetch */
	actuator_outputs_s outputs;
	outputs.timestamp = hrt_absolute_time();

	/* get servo values from IO */
	uint16_t ctl[_max_actuators];
	int ret = io_reg_get(PX4IO_PAGE_SERVOS, 0, ctl, _max_actuators);
	if (ret != OK)
		return ret;

	/* convert from register format to float */
	for (unsigned i = 0; i < _max_actuators; i++)
		outputs.output[i] = REG_TO_FLOAT(ctl[i]);
	outputs.noutputs = _max_actuators;

	/* lazily advertise on first publication */
	if (_to_outputs == 0) {
		_to_outputs = orb_advertise((_primary_pwm_device ?
			ORB_ID_VEHICLE_CONTROLS :
			ORB_ID(actuator_outputs_1)),
			&outputs);
	} else {
		orb_publish((_primary_pwm_device ?
			ORB_ID_VEHICLE_CONTROLS :
			ORB_ID(actuator_outputs_1)),
			_to_outputs,
			&outputs);
	}

	return OK;
}

int
PX4IO::io_reg_set(uint8_t page, uint8_t offset, const uint16_t *values, unsigned num_values)
{
	i2c_msg_s	msgv[2];
	uint8_t		hdr[2];

	hdr[0] = page;
	hdr[1] = offset;

	msgv[0].flags = 0;
	msgv[0].buffer = hdr;
	msgv[0].length = sizeof(hdr);

	msgv[1].flags = 0;
	msgv[1].buffer = (uint8_t *)(values);
	msgv[1].length = num_values * sizeof(*values);

	return transfer(msgv, 2);
}

int
PX4IO::io_reg_set(uint8_t page, uint8_t offset, uint16_t value)
{
	return io_reg_set(page, offset, &value, 1);
}

int
PX4IO::io_reg_get(uint8_t page, uint8_t offset, uint16_t *values, unsigned num_values)
{
	i2c_msg_s	msgv[2];
	uint8_t		hdr[2];

	hdr[0] = page;
	hdr[1] = offset;

	msgv[0].flags = 0;
	msgv[0].buffer = hdr;
	msgv[0].length = sizeof(hdr);

	msgv[1].flags = I2C_M_READ;
	msgv[1].buffer = (uint8_t *)values;
	msgv[1].length = num_values * sizeof(*values);

	return transfer(msgv, 2);
}

uint32_t
PX4IO::io_reg_get(uint8_t page, uint8_t offset)
{
	uint16_t value;

	if (io_reg_get(page, offset, &value, 1))
		return _io_reg_get_error;

	return value;
}

int
PX4IO::io_reg_modify(uint8_t page, uint8_t offset, uint16_t clearbits, uint16_t setbits)
{
	int ret;
	uint16_t value;

	ret = io_reg_get(page, offset, &value, 1);
	if (ret)
		return ret;
	value &= ~clearbits;
	value |= setbits;

	return io_reg_set(page, offset, &value, 1);
}


#if 0
void
PX4IO::config_send()
{
	px4io_config	cfg;
	int		ret;

	cfg.f2i_config_magic = F2I_CONFIG_MAGIC;

	int val;

	/* maintaing the standard order of Roll, Pitch, Yaw, Throttle */		
	param_get(param_find("RC_MAP_ROLL"), &val);
	cfg.rc_map[0] = val;
	param_get(param_find("RC_MAP_PITCH"), &val);
	cfg.rc_map[1] = val;
	param_get(param_find("RC_MAP_YAW"), &val);
	cfg.rc_map[2] = val;
	param_get(param_find("RC_MAP_THROTTLE"), &val);
	cfg.rc_map[3] = val;

	/* set the individual channel properties */
	char nbuf[16];
	float float_val;
	for (unsigned i = 0; i < 4; i++) {
		sprintf(nbuf, "RC%d_MIN", i + 1);
		param_get(param_find(nbuf), &float_val);	
		cfg.rc_min[i] = float_val;
	}
	for (unsigned i = 0; i < 4; i++) {
		sprintf(nbuf, "RC%d_TRIM", i + 1);	
		param_get(param_find(nbuf), &float_val);	
		cfg.rc_trim[i] = float_val;
	}
	for (unsigned i = 0; i < 4; i++) {
		sprintf(nbuf, "RC%d_MAX", i + 1);	
		param_get(param_find(nbuf), &float_val);	
		cfg.rc_max[i] = float_val;
	}
	for (unsigned i = 0; i < 4; i++) {
		sprintf(nbuf, "RC%d_REV", i + 1);	
		param_get(param_find(nbuf), &float_val);	
		cfg.rc_rev[i] = float_val;
	}
	for (unsigned i = 0; i < 4; i++) {
		sprintf(nbuf, "RC%d_DZ", i + 1);	
		param_get(param_find(nbuf), &float_val);	
		cfg.rc_dz[i] = float_val;
	}

	ret = hx_stream_send(_io_stream, &cfg, sizeof(cfg));

	if (ret)
		debug("config error %d", ret);
}
#endif

int
PX4IO::mixer_send(const char *buf, unsigned buflen)
{
	uint8_t	frame[_max_transfer];
	px4io_mixdata *msg = (px4io_mixdata *)&frame[0];
	unsigned max_len = _max_transfer - sizeof(px4io_mixdata);

	msg->f2i_mixer_magic = F2I_MIXER_MAGIC;
	msg->action = F2I_MIXER_ACTION_RESET;

	do {
		unsigned count = buflen;

		if (count > max_len)
			count = max_len;

		if (count > 0) {
			memcpy(&msg->text[0], buf, count);
			buf += count;
			buflen -= count;
		}

		/*
		 * We have to send an even number of bytes.  This
		 * will only happen on the very last transfer of a
		 * mixer, and we are guaranteed that there will be
		 * space left to round up as _max_transfer will be
		 * even.
		 */
		unsigned total_len = sizeof(px4io_mixdata) + count;
		if (total_len % 1) {
			msg->text[count] = '\0';
			total_len++;
		}

		int ret = io_reg_set(PX4IO_PAGE_MIXERLOAD, 0, (uint16_t *)frame, total_len / 2);

		if (ret) {
			log("mixer send error %d", ret);
			return ret;
		}

		msg->action = F2I_MIXER_ACTION_APPEND;

	} while (buflen > 0);

	/* check for the mixer-OK flag */
	if (io_reg_get(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_FLAGS) & PX4IO_P_STATUS_FLAGS_MIXER_OK)
		return 0;

	/* load must have failed for some reason */
	return -EINVAL;
}

int
PX4IO::ioctl(file *filep, int cmd, unsigned long arg)
{
	int ret = OK;

	/* regular ioctl? */
	switch (cmd) {
	case PWM_SERVO_ARM:
		/* set the 'armed' bit */
		ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, 0, PX4IO_P_SETUP_ARMING_ARM_OK);
		break;

	case PWM_SERVO_DISARM:
		/* clear the 'armed' bit */
		ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, PX4IO_P_SETUP_ARMING_ARM_OK, 0);
		break;

	case PWM_SERVO_SET_UPDATE_RATE:
		/* set the requested rate */
		if ((arg >= 50) && (arg <= 400)) {
			ret = io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_HIGHRATE, arg);
		} else {
			ret = -EINVAL;
		}
		break;

	case PWM_SERVO_SET(0) ... PWM_SERVO_SET(PWM_OUTPUT_MAX_CHANNELS): {

		unsigned channel = cmd - PWM_SERVO_SET(0);
		if ((channel >= _max_actuators) || (arg < 900) || (arg > 2100)) {
			ret = -EINVAL;
		} else {
			/* send a direct PWM value */
			ret = io_reg_set(PX4IO_PAGE_DIRECT_PWM, channel, arg);
		}

		break;
	}

	case PWM_SERVO_GET(0) ... PWM_SERVO_GET(PWM_OUTPUT_MAX_CHANNELS): {

		unsigned channel = cmd - PWM_SERVO_GET(0);

		if (channel >= _max_actuators) {
			ret = -EINVAL;
		} else {
			/* fetch a current PWM value */
			uint32_t value = io_reg_get(PX4IO_PAGE_DIRECT_PWM, channel);
			if (value == _io_reg_get_error) {
				ret = -EIO;
			} else {
				*(servo_position_t *)arg = value;
			}
		}
		break;
	}

	case GPIO_RESET:
		ret = io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_RELAYS, 0);
		break;

	case GPIO_SET:
		arg &= ((1 << _max_relays) - 1);
		ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_RELAYS, 0, arg);
		break;

	case GPIO_CLEAR:
		arg &= ((1 << _max_relays) - 1);
		ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_RELAYS, arg, 0);
		break;

	case GPIO_GET:
		*(uint32_t *)arg = io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_RELAYS);
		if (*(uint32_t *)arg == _io_reg_get_error)
			ret = -EIO;
		break;

	case MIXERIOCGETOUTPUTCOUNT:
		*(unsigned *)arg = _max_actuators;
		break;

	case MIXERIOCRESET:
		ret = 0;	/* load always resets */
		break;

	case MIXERIOCLOADBUF:
		ret = mixer_send((const char *)arg, strnlen(_mix_buf, 1024));
		break;

	default:
		/* not a recognised value */
		ret = -ENOTTY;
	}

	return ret;
}

ssize_t
PX4IO::write(file *filp, const char *buffer, size_t len)
{
	unsigned count = len / 2;
	int ret;

	if (count > 0) {
		if (count > _max_actuators)
			count = _max_actuators;
		ret = io_reg_set(PX4IO_PAGE_DIRECT_PWM, 0, (uint16_t *)buffer, count);
	} else {
		ret = -EINVAL;
	}

	return ret;
}

extern "C" __EXPORT int px4io_main(int argc, char *argv[]);

namespace
{

void
test(void)
{
	int	fd;

	fd = open(PWM_OUTPUT_DEVICE_PATH, 0);

	if (fd < 0) {
		puts("open fail");
		exit(1);
	}

	ioctl(fd, PWM_SERVO_ARM, 0);
	ioctl(fd, PWM_SERVO_SET(0), 1000);
	ioctl(fd, PWM_SERVO_SET(1), 1100);
	ioctl(fd, PWM_SERVO_SET(2), 1200);
	ioctl(fd, PWM_SERVO_SET(3), 1300);
	ioctl(fd, PWM_SERVO_SET(4), 1400);
	ioctl(fd, PWM_SERVO_SET(5), 1500);
	ioctl(fd, PWM_SERVO_SET(6), 1600);
	ioctl(fd, PWM_SERVO_SET(7), 1700);

	close(fd);

	actuator_armed_s aa;

	aa.armed = true;
	aa.lockdown = false;

	orb_advertise(ORB_ID(actuator_armed), &aa);

	exit(0);
}

void
monitor(void)
{
	unsigned cancels = 3;
	printf("Hit <enter> three times to exit monitor mode\n");

	for (;;) {
		pollfd fds[1];

		fds[0].fd = 0;
		fds[0].events = POLLIN;
		poll(fds, 1, 500);

		if (fds[0].revents == POLLIN) {
			int c;
			read(0, &c, 1);

			if (cancels-- == 0)
				exit(0);
		}

#warning implement this

//		if (g_dev != nullptr)
//			g_dev->dump_one = true;
	}
}

}

int
px4io_main(int argc, char *argv[])
{
	if (!strcmp(argv[1], "start")) {

		if (g_dev != nullptr)
			errx(1, "already loaded");

		/* create the driver - it will set g_dev */
		(void)new PX4IO;

		if (g_dev == nullptr)
			errx(1, "driver alloc failed");

		if (OK != g_dev->init()) {
			delete g_dev;
			errx(1, "driver init failed");
		}

		/* look for the optional pwm update rate for the supported modes */
		if (strcmp(argv[2], "-u") == 0 || strcmp(argv[2], "--update-rate") == 0) {
			if (argc > 2 + 1) {
#warning implement this 
			} else {
				fprintf(stderr, "missing argument for pwm update rate (-u)\n");
				return 1;
			}
		}

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {

			if (g_dev != nullptr) {
				/* stop the driver */
				delete g_dev;
			} else {
				errx(1, "not loaded");
			}
			exit(0);
		}


	if (!strcmp(argv[1], "status")) {

			if (g_dev != nullptr)
				printf("[px4io] loaded\n");
			else
				printf("[px4io] not loaded\n");

			exit(0);
		}

	/* note, stop not currently implemented */

	if (!strcmp(argv[1], "update")) {

		if (g_dev != nullptr) {
			printf("[px4io] loaded, detaching first\n");
			/* stop the driver */
			delete g_dev;
		}

		PX4IO_Uploader *up;
		const char *fn[3];

		/* work out what we're uploading... */
		if (argc > 2) {
			fn[0] = argv[2];
			fn[1] = nullptr;

		} else {
			fn[0] = "/fs/microsd/px4io.bin";
			fn[1] =	"/etc/px4io.bin";
			fn[2] =	nullptr;
		}

		up = new PX4IO_Uploader;
		int ret = up->upload(&fn[0]);
		delete up;

		switch (ret) {
		case OK:
			break;

		case -ENOENT:
			errx(1, "PX4IO firmware file not found");

		case -EEXIST:
		case -EIO:
			errx(1, "error updating PX4IO - check that bootloader mode is enabled");

		case -EINVAL:
			errx(1, "verify failed - retry the update");

		case -ETIMEDOUT:
			errx(1, "timed out waiting for bootloader - power-cycle and try again");

		default:
			errx(1, "unexpected error %d", ret);
		}

		return ret;
	}

	if (!strcmp(argv[1], "rx_dsm") ||
	    !strcmp(argv[1], "rx_dsm_10bit") ||
	    !strcmp(argv[1], "rx_dsm_11bit") ||
	    !strcmp(argv[1], "rx_sbus") ||
	    !strcmp(argv[1], "rx_ppm"))
		errx(0, "receiver type is automatically detected, option '%s' is deprecated", argv[1]);

	if (!strcmp(argv[1], "test"))
		test();

	if (!strcmp(argv[1], "monitor"))
		monitor();

	errx(1, "need a command, try 'start', 'stop', 'status', 'test', 'monitor' or 'update'");
}
