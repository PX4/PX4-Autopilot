/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * PX4IO is connected via serial (or possibly some other interface at a later
 * point).
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
#include <termios.h>
#include <fcntl.h>
#include <math.h>

#include <arch/board/board.h>

#include <drivers/device/device.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_gpio.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_mixer.h>

#include <systemlib/mixer/mixer.h>
#include <systemlib/perf_counter.h>
#include <systemlib/hx_stream.h>
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

#include <px4io/protocol.h>
#include "uploader.h"


class PX4IO : public device::I2C
{
public:
	PX4IO();
	~PX4IO();

	virtual int		init();

	virtual int		ioctl(file *filp, int cmd, unsigned long arg);

	/**
	 * Set the PWM via serial update rate
	 * @warning this directly affects CPU load
	 */
	int 			set_pwm_rate(int hz);

	bool			dump_one;

private:
	// XXX
	static const unsigned	_max_actuators = PX4IO_CONTROL_CHANNELS;
	unsigned 		_update_rate;	///< serial send rate in Hz

	volatile int		_task;		///< worker task
	volatile bool		_task_should_exit;
	volatile bool		_connected;	///< true once we have received a valid frame

	/* subscribed topics */
	int			_t_actuators;	///< actuator output topic
	int			_t_armed;	///< system armed control topic
	int 			_t_vstatus;	///< system / vehicle status

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

	uint32_t		_relays;	///< state of the PX4IO relays, one bit per relay

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
	 * Fetch status and alarms from IO
	 */
	int			io_get_status();

	/**
	 * Fetch RC inputs from IO
	 */
	int			io_get_rc_input(rc_input_values &input_rc);

	/**
	 * write register(s)
	 */
	int			io_reg_set(uint8_t page, uint8_t offset, const uint16_t *values, unsigned num_values);

	/**
	 * read register(s)
	 */
	int			io_reg_get(uint8_t page, uint8_t offset, uint16_t *values, unsigned num_values);

	/**
	 * modify s register
	 */
	int			io_reg_modify(uint8_t page, uint8_t offset, uint16_t clearbits, uint16_t setbits);

};


namespace
{

PX4IO	*g_dev;

}

PX4IO::PX4IO() :
	CDev("px4io", "/dev/px4io", PX4_I2C_BUS_ONBOARD, PX4_I2C_OBDEV_PX4IO, 320000),
	dump_one(false),
	_update_rate(50),
	_task(-1),
	_task_should_exit(false),
	_connected(false),
	_t_actuators(-1),
	_t_armed(-1),
	_t_vstatus(-1),
	_to_input_rc(0),
	_to_actuators_effective(0),
	_to_outputs(0),
	_to_battery(0),
	_mix_buf(nullptr),
	_mix_buf_len(0),
	_primary_pwm_device(false),
	_relays(0),
	_switch_armed(false),
	_send_needed(false),
	_config_needed(true)
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

int
PX4IO::set_pwm_rate(int hz)
{
	if (hz > 0 && hz <= 400) {
		_update_rate = hz;
		return OK;
	} else {
		return -EINVAL;
	}
}

void
PX4IO::task_main_trampoline(int argc, char *argv[])
{
	g_dev->task_main();
}

void
PX4IO::task_main()
{
	hrt_abstime_t last_poll_time = 0;
	unsigned poll_phase = 0;

	log("starting");
	unsigned update_rate_in_ms;

	/*
	 * Subscribe to the appropriate PWM output topic based on whether we are the
	 * primary PWM output or not.
	 */
	_t_actuators = orb_subscribe(_primary_pwm_device ? ORB_ID_VEHICLE_ATTITUDE_CONTROLS :
				     ORB_ID(actuator_controls_1));

	/* convert the update rate in hz to milliseconds, rounding down if necessary */
	update_rate_in_ms = 1000 / _update_rate;
	orb_set_interval(_t_actuators, update_rate_in_ms);

	_t_armed = orb_subscribe(ORB_ID(actuator_armed));
	orb_set_interval(_t_armed, 200);		/* 5Hz update rate */

	_t_vstatus = orb_subscribe(ORB_ID(vehicle_status));
	orb_set_interval(_t_vstatus, 200);		/* 5Hz update rate max. */

	/* poll descriptor */
	pollfd fds[3];
	fds[0].fd = _t_actuators;
	fds[0].events = POLLIN;
	fds[1].fd = _t_armed;
	fds[1].events = POLLIN;
	fds[2].fd = _t_vstatus;
	fds[2].events = POLLIN;

	debug("ready");

	/* lock against the ioctl handler */
	lock();

	/* loop talking to IO */
	while (!_task_should_exit) {

		/* sleep waiting for topic updates, but no more than 20ms */
		/* XXX should actually be calculated to keep the poller running tidily */
		unlock();
		int ret = ::poll(&fds[0], sizeof(fds) / sizeof(fds[0]), 20);
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

		hrt_abstime_t now = hrt_absolute_time();

		/*
		 * If this isn't time for the next tick of the polling state machine,
		 * go back to sleep.
		 */
		if ((now - last_poll_time) < 20000)
			continue;

		/*
		 * Pull status and alarms from IO
		 */
		io_get_status();

		switch (poll_phase) {
		case 0:
			/* XXX fetch raw RC values */
			break;
		case 1:
			/* XXX fetch servo outputs */
			break;
		}

#if 0
	/* advertise the limited control inputs */
	memset(&_controls_effective, 0, sizeof(_controls_effective));
	_to_actuators_effective = orb_advertise(_primary_pwm_device ? ORB_ID_VEHICLE_ATTITUDE_CONTROLS_EFFECTIVE : ORB_ID(actuator_controls_1),
				   &_controls_effective);

	/* advertise the mixed control outputs */
	memset(&_outputs, 0, sizeof(_outputs));
	_to_outputs = orb_advertise(_primary_pwm_device ? ORB_ID_VEHICLE_CONTROLS : ORB_ID(actuator_outputs_1),
				   &_outputs);
#endif



	}

	unlock();

out:
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
	     ORB_ID(actuator_controls_1), _t_actuators, &_controls);

	for (unsigned i = 0; i < _max_actuators; i++)
		regs[i] = FLOAT_TO_REG(_controls.control[i]);

	/* copy values to registers in IO */
	io_reg_set(PX4IO_PAGE_CONTROLS, 0, regs, _max_actuators);
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

	io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, clear, set);
}

int
PX4IO::io_get_status()
{
	struct {
		uint16_t status;
		uint16_t alarms;
		uint16_t vbatt;
	} state;
	int		ret;
	bool		rc_valid = false;

	/* get STATUS_FLAGS, STATUS_ALARMS and STATUS_VBATT in that order */
	ret = io_reg_get(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_FLAGS, state, 3);

	/* XXX handle status */

	/* XXX handle alarms */

	/* only publish if battery has a valid minimum voltage */
	if (state.vbatt > 3300) {
		battery_status_s	battery_status;

		battery_status.timestamp = hrt_absolute_time();
		battery_status.voltage_v = state.vbatt / 1000.0f;

		/* current and discharge are currently (ha) unknown */
		battery_status.current_a = -1.0f;
		battery_status.discharged_mah = -1.0f;

		/* lazily publish the battery voltage */
		if (_to_battery > 0) {
			orb_publish(ORB_ID(battery_status), _to_battery, &battery_status);
		} else {
			_to_battery = orb_advertise(ORB_ID(battery_status), &battery_status);
	}

	/*
	 * If we have RC input, get it
	 */
	if (state.status & PX4IO_P_STATUS_FLAGS_RC_OK) {
		rc_input_values		input_rc;

		io_get_rc_input(input_rc);

		if (state.status & PX4IO_P_STATUS_FLAGS_RC_PPM) {
			input_rc.input_source = RC_INPUT_SOURCE_PX4IO_PPM;
		} else if (state.status & RC_INPUT_SOURCE_PX4IO_DSM) {
			input_rc.input_source = RC_INPUT_SOURCE_PX4IO_SPEKTRUM;
		} else if (state.status & RC_INPUT_SOURCE_PX4IO_SBUS) {
			input_rc.input_source = RC_INPUT_SOURCE_PX4IO_SBUS;
		} else {
			input_rc.input_source = RC_INPUT_SOURCE_UNKNOWN;
		}

		if (_to_input_rc > 0) {
			orb_publish(ORB_ID(input_rc), _to_input_rc, &input_rc);
		} else {
			_to_input_rc = orb_advertise(ORB_ID(input_rc), &input_rc);
		}
	}

}

int
PX4IO::io_get_rc_input(rc_input_values &input_rc)
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
	 */
	ret = io_get_reg(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_COUNT, &channel_count, 1);
	if (ret)
		return ret;
	input_rc.channel_count = channel_count;

	if (channel_count > 0)
		ret = io_get_reg(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_BASE, channel_count);

	return ret;
}

int
PX4IO::io_reg_set(uint8_t page, uint8_t offset, const uint16_t *values, unsigned num_values)
{
	i2c_msg_s	msgv[2];
	t8_t		hdr[2];

	hdr[0] = page;
	hdr[1] = offset;

	mgsv[0].flags = 0;
	msgv[0].buffer = hdr;
	msgv[0].length = sizeof(hdr);

	msgv[1].flags = 0;
	msgv[1].buffer = const_cast<uint8_t *>(values);
	msgv[1].length = num_values * sizeof(*values);

	return transfer(msgv, 2);
}

int
PX4IO::io_reg_get(uint8_t page, uint8_t offset, uint16_t *values, unsigned num_values)
{
	i2c_msg_s	msgv[2];
	uint8_t		hdr[2];

	hdr[0] = page;
	hdr[1] = offset;

	mgsv[0].flags = 0;
	msgv[0].buffer = hdr;
	msgv[0].length = sizeof(hdr);

	msgv[1].flags = I2C_M_READ;
	msgv[1].buffer = values;
	msgv[1].length = num_values * sizeof(*values);

	return transfer(msgv, 2);
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




void
PX4IO::io_recv()
{
	uint8_t buf[32];
	int count;

	/*
	 * We are here because poll says there is some data, so this
	 * won't block even on a blocking device.  If more bytes are
	 * available, we'll go back to poll() again...
	 */
	count = ::read(_serial_fd, buf, sizeof(buf));

	/* pass received bytes to the packet decoder */
	for (int i = 0; i < count; i++)
		hx_stream_rx(_io_stream, buf[i]);
}

void
PX4IO::rx_callback_trampoline(void *arg, const void *buffer, size_t bytes_received)
{
	g_dev->rx_callback((const uint8_t *)buffer, bytes_received);
}

void
PX4IO::rx_callback(const uint8_t *buffer, size_t bytes_received)
{
	const px4io_report *rep = (const px4io_report *)buffer;

//	lock();

	/* sanity-check the received frame size */
	if (bytes_received != sizeof(px4io_report)) {
		debug("got %u expected %u", bytes_received, sizeof(px4io_report));
		goto out;
	}

	if (rep->i2f_magic != I2F_MAGIC) {
		debug("bad magic");
		goto out;
	}

	_connected = true;

	/* publish raw rc channel values from IO if valid channels are present */
	if (rep->channel_count > 0) {
		_input_rc.timestamp = hrt_absolute_time();
		_input_rc.channel_count = rep->channel_count;

		for (int i = 0; i < rep->channel_count; i++) {
			_input_rc.values[i] = rep->rc_channel[i];
		}

		orb_publish(ORB_ID(input_rc), _to_input_rc, &_input_rc);
	}

	/* remember the latched arming switch state */
	_switch_armed = rep->armed;

	/* publish battery information */

	/* only publish if battery has a valid minimum voltage */
	if (rep->battery_mv > 3300) {
		_battery_status.timestamp = hrt_absolute_time();
		_battery_status.voltage_v = rep->battery_mv / 1000.0f;
		/* current and discharge are unknown */
		_battery_status.current_a = -1.0f;
		_battery_status.discharged_mah = -1.0f;
		/* announce the battery voltage if needed, just publish else */
		if (_to_battery > 0) {
			orb_publish(ORB_ID(battery_status), _to_battery, &_battery_status);
		} else {
			_to_battery = orb_advertise(ORB_ID(battery_status), &_battery_status);
		}
	}

	_send_needed = true;

	/* if monitoring, dump the received info */
	if (dump_one) {
		dump_one = false;

		printf("IO: %s armed ", rep->armed ? "" : "not");

		for (unsigned i = 0; i < rep->channel_count; i++)
			printf("%d: %d ", i, rep->rc_channel[i]);

		printf("\n");
	}

out:
//	unlock();
	return;
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

int
PX4IO::mixer_send(const char *buf, unsigned buflen)
{
	uint8_t	frame[HX_STREAM_MAX_FRAME];
	px4io_mixdata *msg = (px4io_mixdata *)&frame[0];

	msg->f2i_mixer_magic = F2I_MIXER_MAGIC;
	msg->action = F2I_MIXER_ACTION_RESET;

	do {
		unsigned count = buflen;

		if (count > F2I_MIXER_MAX_TEXT)
			count = F2I_MIXER_MAX_TEXT;

		if (count > 0) {
			memcpy(&msg->text[0], buf, count);
			buf += count;
			buflen -= count;
		}

		int ret = hx_stream_send(_io_stream, msg, sizeof(px4io_mixdata) + count);

		if (ret) {
			log("mixer send error %d", ret);
			return ret;
		}

		msg->action = F2I_MIXER_ACTION_APPEND;

	} while (buflen > 0);

	return 0;
}
#endif

int
PX4IO::ioctl(file *filep, int cmd, unsigned long arg)
{
	int ret = OK;

	lock();

	/* regular ioctl? */
	switch (cmd) {
	case PWM_SERVO_ARM:
		/* fake an armed transition */
		_armed.armed = true;
		_send_needed = true;
		break;

	case PWM_SERVO_DISARM:
		/* fake a disarmed transition */
		_armed.armed = false;
		_send_needed = true;
		break;

	case PWM_SERVO_SET_UPDATE_RATE:
		// not supported yet
		ret = -EINVAL;
		break;

	case PWM_SERVO_SET(0) ... PWM_SERVO_SET(_max_actuators - 1):

		/* fake an update to the selected 'servo' channel */
		if ((arg >= 900) && (arg <= 2100)) {
			_outputs.output[cmd - PWM_SERVO_SET(0)] = arg;
			_send_needed = true;

		} else {
			ret = -EINVAL;
		}

		break;

	case PWM_SERVO_GET(0) ... PWM_SERVO_GET(_max_actuators - 1):
		/* copy the current output value from the channel */
		*(servo_position_t *)arg = _outputs.output[cmd - PWM_SERVO_GET(0)];
		break;

	case GPIO_RESET:
		_relays = 0;
		_send_needed = true;
		break;

	case GPIO_SET:
	case GPIO_CLEAR:
		/* make sure only valid bits are being set */
		if ((arg & ((1UL << PX4IO_RELAY_CHANNELS) - 1)) != arg) {
			ret = EINVAL;
			break;
		}
		if (cmd == GPIO_SET) {
			_relays |= arg;
		} else {
			_relays &= ~arg;
		}
		_send_needed = true;
		break;

	case GPIO_GET:
		*(uint32_t *)arg = _relays;
		break;

	case MIXERIOCGETOUTPUTCOUNT:
		*(unsigned *)arg = PX4IO_CONTROL_CHANNELS;
		break;

	case MIXERIOCRESET:
		ret = 0;	/* load always resets */
		break;

	case MIXERIOCLOADBUF:

		/* set the buffer up for transfer */
		_mix_buf = (const char *)arg;
		_mix_buf_len = strnlen(_mix_buf, 1024);

		/* drop the lock and wait for the thread to clear the transmit */
		unlock();

		while (_mix_buf != nullptr)
			usleep(1000);

		lock();

		ret = 0;
		break;

	default:
		/* not a recognised value */
		ret = -ENOTTY;
	}

	unlock();

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

		if (g_dev != nullptr)
			g_dev->dump_one = true;
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
				g_dev->set_pwm_rate(atoi(argv[2 + 1]));
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
