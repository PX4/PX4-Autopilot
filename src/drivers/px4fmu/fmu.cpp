/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file fmu.cpp
 *
 * Driver/configurator for the PX4 FMU multi-purpose port on v1 and v2 boards.
 */

#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <nuttx/arch.h>

#include <drivers/device/device.h>
#include <drivers/device/i2c.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_input_capture.h>
#include <drivers/drv_gpio.h>
#include <drivers/drv_hrt.h>

#include <board_config.h>

#include <systemlib/px4_macros.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <systemlib/mixer/mixer.h>
#include <systemlib/pwm_limit/pwm_limit.h>
#include <systemlib/board_serial.h>
#include <systemlib/param/param.h>
#include <drivers/drv_mixer.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_input_capture.h>

#include <lib/rc/sbus.h>
#include <lib/rc/dsm.h>
#include <lib/rc/st24.h>
#include <lib/rc/sumd.h>

#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/safety.h>
#include <uORB/topics/adc_report.h>


#ifdef HRT_PPM_CHANNEL
# include <systemlib/ppm_decode.h>
#endif

#include <systemlib/circuit_breaker.h>

#define SCHEDULE_INTERVAL	2000	/**< The schedule interval in usec (500 Hz) */
#define NAN_VALUE	(0.0f/0.0f)		/**< NaN value for throttle lock mode */
#define BUTTON_SAFETY	px4_arch_gpioread(GPIO_BTN_SAFETY)
#define CYCLE_COUNT 10			/* safety switch must be held for 1 second to activate */

/*
 * Define the various LED flash sequences for each system state.
 */
#define LED_PATTERN_FMU_OK_TO_ARM 		0x0003		/**< slow blinking			*/
#define LED_PATTERN_FMU_REFUSE_TO_ARM 	0x5555		/**< fast blinking			*/
#define LED_PATTERN_IO_ARMED 			0x5050		/**< long off, then double blink 	*/
#define LED_PATTERN_FMU_ARMED 			0x5500		/**< long off, then quad blink 		*/
#define LED_PATTERN_IO_FMU_ARMED 		0xffff		/**< constantly on			*/

#if !defined(BOARD_HAS_PWM)
#  error "board_config.h needs to define BOARD_HAS_PWM"
#endif
class PX4FMU : public device::CDev
{
public:
	enum Mode {
		MODE_NONE,
		MODE_2PWM,
		MODE_2PWM2CAP,
		MODE_3PWM,
		MODE_3PWM1CAP,
		MODE_4PWM,
		MODE_6PWM,
		MODE_8PWM,
		MODE_4CAP,
		MODE_5CAP,
		MODE_6CAP,
	};
	PX4FMU();
	virtual ~PX4FMU();

	virtual int	ioctl(file *filp, int cmd, unsigned long arg);
	virtual ssize_t	write(file *filp, const char *buffer, size_t len);

	virtual int	init();

	int		set_mode(Mode mode);
	Mode		get_mode() { return _mode; }

	int		set_pwm_alt_rate(unsigned rate);
	int		set_pwm_alt_channels(uint32_t channels);

	static int	set_i2c_bus_clock(unsigned bus, unsigned clock_hz);

	static void	capture_trampoline(void *context, uint32_t chan_index,
					   hrt_abstime edge_time, uint32_t edge_state,
					   uint32_t overflow);

private:
	enum RC_SCAN {
		RC_SCAN_PPM = 0,
		RC_SCAN_SBUS,
		RC_SCAN_DSM,
		RC_SCAN_SUMD,
		RC_SCAN_ST24
	};
	enum RC_SCAN _rc_scan_state = RC_SCAN_SBUS;

	char const *RC_SCAN_STRING[5] = {
		"PPM",
		"SBUS",
		"DSM",
		"SUMD",
		"ST24"
	};

	hrt_abstime _rc_scan_begin = 0;
	bool _rc_scan_locked = false;
	bool _report_lock = true;

	int32_t _wifi_tx_mode = 1;
	param_t _wifi_tx_param = PARAM_INVALID;

	hrt_abstime _cycle_timestamp = 0;
	hrt_abstime _last_safety_check = 0;

	static const unsigned _max_actuators = DIRECT_PWM_OUTPUT_CHANNELS;

	Mode		_mode;
	unsigned	_pwm_default_rate;
	unsigned	_pwm_alt_rate;
	uint32_t	_pwm_alt_rate_channels;
	unsigned	_current_update_rate;
	struct work_s	_work;
	int		_armed_sub;
	int		_param_sub;
	int		_adc_sub;
	struct rc_input_values	_rc_in;
	float		_analog_rc_rssi_volt;
	bool		_analog_rc_rssi_stable;
	orb_advert_t	_to_input_rc;
	orb_advert_t	_outputs_pub;
	unsigned	_num_outputs;
	int		_class_instance;
	int		_rcs_fd;
	uint8_t _rcs_buf[SBUS_FRAME_SIZE];

	volatile bool	_initialized;
	bool		_throttle_armed;
	bool		_pwm_on;
	uint32_t	_pwm_mask;
	bool		_pwm_initialized;

	MixerGroup	*_mixers;

	uint32_t	_groups_required;
	uint32_t	_groups_subscribed;
	int		_control_subs[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	actuator_controls_s _controls[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	orb_id_t	_control_topics[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	pollfd	_poll_fds[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	unsigned	_poll_fds_num;

	static pwm_limit_t	_pwm_limit;
	static actuator_armed_s	_armed;
	uint16_t	_failsafe_pwm[_max_actuators];
	uint16_t	_disarmed_pwm[_max_actuators];
	uint16_t	_min_pwm[_max_actuators];
	uint16_t	_max_pwm[_max_actuators];
	uint16_t	_reverse_pwm_mask;
	unsigned	_num_failsafe_set;
	unsigned	_num_disarmed_set;
	bool		_safety_off;
	bool		_safety_disabled;
	orb_advert_t		_to_safety;

	static bool	arm_nothrottle() { return (_armed.prearmed && !_armed.armed); }

	static void	cycle_trampoline(void *arg);
	void		cycle();
	void		work_start();
	void		work_stop();

	static int	control_callback(uintptr_t handle,
					 uint8_t control_group,
					 uint8_t control_index,
					 float &input);
	void		capture_callback(uint32_t chan_index,
					 hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow);
	void		subscribe();
	int		set_pwm_rate(unsigned rate_map, unsigned default_rate, unsigned alt_rate);
	int		pwm_ioctl(file *filp, int cmd, unsigned long arg);
	void		update_pwm_rev_mask();
	void		publish_pwm_outputs(uint16_t *values, size_t numvalues);
	void		update_pwm_out_state(bool on);
	void		pwm_output_set(unsigned i, unsigned value);

	struct GPIOConfig {
		uint32_t	input;
		uint32_t	output;
		uint32_t	alt;
	};

	static const GPIOConfig	_gpio_tab[];
	static const unsigned	_ngpio;

	void		gpio_reset(void);
	void		sensor_reset(int ms);
	void		peripheral_reset(int ms);
	void		gpio_set_function(uint32_t gpios, int function);
	void		gpio_write(uint32_t gpios, int function);
	uint32_t	gpio_read(void);
	int		gpio_ioctl(file *filp, int cmd, unsigned long arg);

	int		capture_ioctl(file *filp, int cmd, unsigned long arg);

	/* do not allow to copy due to ptr data members */
	PX4FMU(const PX4FMU &);
	PX4FMU operator=(const PX4FMU &);
	void fill_rc_in(uint16_t raw_rc_count,
			uint16_t raw_rc_values[input_rc_s::RC_INPUT_MAX_CHANNELS],
			hrt_abstime now, bool frame_drop, bool failsafe,
			unsigned frame_drops, int rssi);
	void dsm_bind_ioctl(int dsmMode);
	void set_rc_scan_state(RC_SCAN _rc_scan_state);
	void rc_io_invert();
	void rc_io_invert(bool invert);
	void safety_check_button(void);
	void setWIFIstate(int32_t mode, bool armed);
};

const PX4FMU::GPIOConfig PX4FMU::_gpio_tab[] =	BOARD_FMU_GPIO_TAB;

const unsigned		PX4FMU::_ngpio = arraySize(PX4FMU::_gpio_tab);
pwm_limit_t		PX4FMU::_pwm_limit;
actuator_armed_s	PX4FMU::_armed = {};

namespace
{

PX4FMU	*g_fmu;

} // namespace

PX4FMU::PX4FMU() :
	CDev("fmu", PX4FMU_DEVICE_PATH),
	_mode(MODE_NONE),
	_pwm_default_rate(50),
	_pwm_alt_rate(50),
	_pwm_alt_rate_channels(0),
	_current_update_rate(0),
	_work{},
	_armed_sub(-1),
	_param_sub(-1),
	_adc_sub(-1),
	_rc_in{},
	_analog_rc_rssi_volt(-1.0f),
	_analog_rc_rssi_stable(false),
	_to_input_rc(nullptr),
	_outputs_pub(nullptr),
	_num_outputs(0),
	_class_instance(0),
	_rcs_fd(-1),
	_initialized(false),
	_throttle_armed(false),
	_pwm_on(false),
	_pwm_mask(0),
	_pwm_initialized(false),
	_mixers(nullptr),
	_groups_required(0),
	_groups_subscribed(0),
	_control_subs{ -1},
	_poll_fds_num(0),
	_failsafe_pwm{0},
	_disarmed_pwm{0},
	_reverse_pwm_mask(0),
	_num_failsafe_set(0),
	_num_disarmed_set(0),
	_safety_off(false),
	_safety_disabled(false),
	_to_safety(nullptr)
{
	for (unsigned i = 0; i < _max_actuators; i++) {
		_min_pwm[i] = PWM_DEFAULT_MIN;
		_max_pwm[i] = PWM_DEFAULT_MAX;
	}

	_control_topics[0] = ORB_ID(actuator_controls_0);
	_control_topics[1] = ORB_ID(actuator_controls_1);
	_control_topics[2] = ORB_ID(actuator_controls_2);
	_control_topics[3] = ORB_ID(actuator_controls_3);

	memset(_controls, 0, sizeof(_controls));
	memset(_poll_fds, 0, sizeof(_poll_fds));

	// rc input, published to ORB
	memset(&_rc_in, 0, sizeof(_rc_in));
	_rc_in.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_PPM;

#ifdef GPIO_SBUS_INV
	// this board has a GPIO to control SBUS inversion
	px4_arch_configgpio(GPIO_SBUS_INV);
#endif

	// If there is no safety button, disable it on boot.
#ifndef GPIO_BTN_SAFETY
	_safety_off = true;
#endif

	/* only enable this during development */
	_debug_enabled = false;
}

PX4FMU::~PX4FMU()
{
	if (_initialized) {
		/* tell the task we want it to go away */
		work_stop();

		int i = 10;

		do {
			/* wait 50ms - it should wake every 100ms or so worst-case */
			usleep(50000);
			i--;

		} while (_initialized && i > 0);
	}

	/* clean up the alternate device node */
	unregister_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH, _class_instance);

	g_fmu = nullptr;
}

int
PX4FMU::init()
{
	int ret;

	ASSERT(!_initialized);

	/* do regular cdev init */
	ret = CDev::init();

	if (ret != OK) {
		return ret;
	}

	/* try to claim the generic PWM output device node as well - it's OK if we fail at this */
	_class_instance = register_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH);

	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		/* lets not be too verbose */
	} else if (_class_instance < 0) {
		warnx("FAILED registering class device");
	}

	_safety_disabled = circuit_breaker_enabled("CBRK_IO_SAFETY", CBRK_IO_SAFETY_KEY);

	work_start();

	return OK;
}

void
PX4FMU:: safety_check_button(void)
{
#ifdef GPIO_BTN_SAFETY
	static int counter = 0;
	/*
	 * Debounce the safety button, change state if it has been held for long enough.
	 *
	 */
	bool safety_button_pressed = BUTTON_SAFETY;

	/*
	 * Keep pressed for a while to arm.
	 *
	 * Note that the counting sequence has to be same length
	 * for arming / disarming in order to end up as proper
	 * state machine, keep ARM_COUNTER_THRESHOLD the same
	 * length in all cases of the if/else struct below.
	 */
	if (safety_button_pressed && !_safety_off) {

		if (counter < CYCLE_COUNT) {
			counter++;

		} else if (counter == CYCLE_COUNT) {
			/* switch to armed state */
			_safety_off = true;
			counter++;
		}

	} else if (safety_button_pressed && _safety_off) {

		if (counter < CYCLE_COUNT) {
			counter++;

		} else if (counter == CYCLE_COUNT) {
			/* change to disarmed state and notify the FMU */
			_safety_off = false;
			counter++;
		}

	} else {
		counter = 0;
	}

	/* Select the appropriate LED flash pattern depending on the current IO/FMU arm state */
	uint16_t pattern = LED_PATTERN_FMU_REFUSE_TO_ARM;

	/* cycle the blink state machine at 10Hz */
	static int blink_counter = 0;

	if (_safety_off) {
		if (_armed.armed) {
			pattern = LED_PATTERN_IO_FMU_ARMED;

		} else {
			pattern = LED_PATTERN_IO_ARMED;
		}

	} else if (_armed.armed) {
		pattern = LED_PATTERN_FMU_ARMED;

	} else {
		pattern = LED_PATTERN_FMU_OK_TO_ARM;

	}

	/* Turn the LED on if we have a 1 at the current bit position */
	px4_arch_gpiowrite(GPIO_LED_SAFETY, !(pattern & (1 << blink_counter++)));

	if (blink_counter > 15) {
		blink_counter = 0;
	}

#endif
}

int
PX4FMU::set_mode(Mode mode)
{
	unsigned old_mask = _pwm_mask;

	/*
	 * Configure for PWM output.
	 *
	 * Note that regardless of the configured mode, the task is always
	 * listening and mixing; the mode just selects which of the channels
	 * are presented on the output pins.
	 */
	switch (mode) {
	case MODE_2PWM2CAP:	// v1 multi-port with flow control lines as PWM
		up_input_capture_set(2, Rising, 0, NULL, NULL);
		up_input_capture_set(3, Rising, 0, NULL, NULL);
		DEVICE_DEBUG("MODE_2PWM2CAP");

	// no break
	case MODE_2PWM:	// v1 multi-port with flow control lines as PWM
		DEVICE_DEBUG("MODE_2PWM");

		/* default output rates */
		_pwm_default_rate = 50;
		_pwm_alt_rate = 50;
		_pwm_alt_rate_channels = 0;
		_pwm_mask = 0x3;
		_pwm_initialized = false;

		break;

	case MODE_3PWM1CAP:	// v1 multi-port with flow control lines as PWM
		DEVICE_DEBUG("MODE_3PWM1CAP");
		up_input_capture_set(3, Rising, 0, NULL, NULL);

	// no break
	case MODE_3PWM:	// v1 multi-port with flow control lines as PWM
		DEVICE_DEBUG("MODE_3PWM");

		/* default output rates */
		_pwm_default_rate = 50;
		_pwm_alt_rate = 50;
		_pwm_alt_rate_channels = 0;
		_pwm_mask = 0x7;
		_pwm_initialized = false;
		break;

	case MODE_4PWM: // v1 or v2 multi-port as 4 PWM outs
		DEVICE_DEBUG("MODE_4PWM");

		/* default output rates */
		_pwm_default_rate = 50;
		_pwm_alt_rate = 50;
		_pwm_alt_rate_channels = 0;
		_pwm_mask = 0xf;
		_pwm_initialized = false;

		break;

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 6

	case MODE_6PWM:
		DEVICE_DEBUG("MODE_6PWM");

		/* default output rates */
		_pwm_default_rate = 50;
		_pwm_alt_rate = 50;
		_pwm_alt_rate_channels = 0;
		_pwm_mask = 0x3f;
		_pwm_initialized = false;

		break;
#endif

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 8

	case MODE_8PWM: // AeroCore PWMs as 8 PWM outs
		DEVICE_DEBUG("MODE_8PWM");
		/* default output rates */
		_pwm_default_rate = 50;
		_pwm_alt_rate = 50;
		_pwm_alt_rate_channels = 0;
		_pwm_mask = 0xff;
		_pwm_initialized = false;
		break;
#endif

	case MODE_NONE:
		DEVICE_DEBUG("MODE_NONE");

		_pwm_default_rate = 10;	/* artificially reduced output rate */
		_pwm_alt_rate = 10;
		_pwm_alt_rate_channels = 0;
		_pwm_mask = 0x0;
		_pwm_initialized = false;

		if (old_mask != _pwm_mask) {
			/* disable servo outputs - no need to set rates */
			up_pwm_servo_deinit();
		}

		break;

	default:
		return -EINVAL;
	}

	_mode = mode;
	return OK;
}

int
PX4FMU::set_pwm_rate(uint32_t rate_map, unsigned default_rate, unsigned alt_rate)
{
	DEVICE_DEBUG("set_pwm_rate %x %u %u", rate_map, default_rate, alt_rate);

	for (unsigned pass = 0; pass < 2; pass++) {
		for (unsigned group = 0; group < _max_actuators; group++) {

			// get the channel mask for this rate group
			uint32_t mask = up_pwm_servo_get_rate_group(group);

			if (mask == 0) {
				continue;
			}

			// all channels in the group must be either default or alt-rate
			uint32_t alt = rate_map & mask;

			if (pass == 0) {
				// preflight
				if ((alt != 0) && (alt != mask)) {
					warn("rate group %u mask %x bad overlap %x", group, mask, alt);
					// not a legal map, bail
					return -EINVAL;
				}

			} else {
				// set it - errors here are unexpected
				if (alt != 0) {
					if (up_pwm_servo_set_rate_group_update(group, _pwm_alt_rate) != OK) {
						warn("rate group set alt failed");
						return -EINVAL;
					}

				} else {
					if (up_pwm_servo_set_rate_group_update(group, _pwm_default_rate) != OK) {
						warn("rate group set default failed");
						return -EINVAL;
					}
				}
			}
		}
	}

	_pwm_alt_rate_channels = rate_map;
	_pwm_default_rate = default_rate;
	_pwm_alt_rate = alt_rate;

	return OK;
}

int
PX4FMU::set_pwm_alt_rate(unsigned rate)
{
	return set_pwm_rate(_pwm_alt_rate_channels, _pwm_default_rate, rate);
}

int
PX4FMU::set_pwm_alt_channels(uint32_t channels)
{
	return set_pwm_rate(channels, _pwm_default_rate, _pwm_alt_rate);
}

int
PX4FMU::set_i2c_bus_clock(unsigned bus, unsigned clock_hz)
{
	return device::I2C::set_bus_clock(bus, clock_hz);
}

void
PX4FMU::subscribe()
{
	/* subscribe/unsubscribe to required actuator control groups */
	uint32_t sub_groups = _groups_required & ~_groups_subscribed;
	uint32_t unsub_groups = _groups_subscribed & ~_groups_required;
	_poll_fds_num = 0;

	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		if (sub_groups & (1 << i)) {
			DEVICE_DEBUG("subscribe to actuator_controls_%d", i);
			_control_subs[i] = orb_subscribe(_control_topics[i]);
		}

		if (unsub_groups & (1 << i)) {
			DEVICE_DEBUG("unsubscribe from actuator_controls_%d", i);
			::close(_control_subs[i]);
			_control_subs[i] = -1;
		}

		if (_control_subs[i] > 0) {
			_poll_fds[_poll_fds_num].fd = _control_subs[i];
			_poll_fds[_poll_fds_num].events = POLLIN;
			_poll_fds_num++;
		}
	}
}

void
PX4FMU::update_pwm_rev_mask()
{
	_reverse_pwm_mask = 0;

	for (unsigned i = 0; i < _max_actuators; i++) {
		char pname[16];
		int32_t ival;

		/* fill the channel reverse mask from parameters */
		sprintf(pname, "PWM_AUX_REV%d", i + 1);
		param_t param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			param_get(param_h, &ival);
			_reverse_pwm_mask |= ((int16_t)(ival != 0)) << i;
		}
	}
}

void
PX4FMU::publish_pwm_outputs(uint16_t *values, size_t numvalues)
{
	actuator_outputs_s outputs = {};
	outputs.noutputs = numvalues;
	outputs.timestamp = hrt_absolute_time();

	for (size_t i = 0; i < _max_actuators; ++i) {
		outputs.output[i] = i < numvalues ? (float)values[i] : 0;
	}

	if (_outputs_pub == nullptr) {
		int instance = -1;
		_outputs_pub = orb_advertise_multi(ORB_ID(actuator_outputs), &outputs, &instance, ORB_PRIO_DEFAULT);

	} else {
		orb_publish(ORB_ID(actuator_outputs), _outputs_pub, &outputs);
	}
}


void
PX4FMU::work_start()
{
	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&PX4FMU::cycle_trampoline, this, 0);
}

void
PX4FMU::cycle_trampoline(void *arg)
{
	PX4FMU *dev = reinterpret_cast<PX4FMU *>(arg);

	dev->cycle();
}

void
PX4FMU::capture_trampoline(void *context, uint32_t chan_index,
			   hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow)
{
	PX4FMU *dev = reinterpret_cast<PX4FMU *>(context);
	dev->capture_callback(chan_index, edge_time, edge_state, overflow);
}

void
PX4FMU::capture_callback(uint32_t chan_index,
			 hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow)
{
	fprintf(stdout, "FMU: Capture chan:%d time:%lld state:%d overflow:%d\n", chan_index, edge_time, edge_state, overflow);
}

void
PX4FMU::fill_rc_in(uint16_t raw_rc_count,
		   uint16_t raw_rc_values[input_rc_s::RC_INPUT_MAX_CHANNELS],
		   hrt_abstime now, bool frame_drop, bool failsafe,
		   unsigned frame_drops, int rssi = -1)
{
	// fill rc_in struct for publishing
	_rc_in.channel_count = raw_rc_count;

	if (_rc_in.channel_count > input_rc_s::RC_INPUT_MAX_CHANNELS) {
		_rc_in.channel_count = input_rc_s::RC_INPUT_MAX_CHANNELS;
	}

	unsigned valid_chans = 0;

	for (unsigned i = 0; i < _rc_in.channel_count; i++) {
		_rc_in.values[i] = raw_rc_values[i];

		if (raw_rc_values[i] != UINT16_MAX) {
			valid_chans++;
		}
	}

	_rc_in.timestamp_publication = now;
	_rc_in.timestamp_last_signal = _rc_in.timestamp_publication;
	_rc_in.rc_ppm_frame_length = 0;

	/* fake rssi if no value was provided */
	if (rssi == -1) {

		/* set RSSI if analog RSSI input is present */
		if (_analog_rc_rssi_stable) {
			float rssi_analog = ((_analog_rc_rssi_volt - 0.2f) / 3.0f) * 100.0f;

			if (rssi_analog > 100.0f) {
				rssi_analog = 100.0f;
			}

			if (rssi_analog < 0.0f) {
				rssi_analog = 0.0f;
			}

			_rc_in.rssi = rssi_analog;

		} else {
			_rc_in.rssi = 255;
		}

	} else {
		_rc_in.rssi = rssi;
	}

	if (valid_chans == 0) {
		_rc_in.rssi = 0;
	}

	_rc_in.rc_failsafe = failsafe;
	_rc_in.rc_lost = (valid_chans == 0);
	_rc_in.rc_lost_frame_count = frame_drops;
	_rc_in.rc_total_frame_count = 0;
}

#ifdef RC_SERIAL_PORT
void PX4FMU::set_rc_scan_state(RC_SCAN newState)
{
//    warnx("RCscan: %s failed, trying %s", PX4FMU::RC_SCAN_STRING[_rc_scan_state], PX4FMU::RC_SCAN_STRING[newState]);
	_rc_scan_begin = 0;
	_rc_scan_state = newState;
}

void PX4FMU::rc_io_invert(bool invert)
{
	INVERT_RC_INPUT(invert);

	if (!invert) {
		// set FMU_RC_OUTPUT high to pull RC_INPUT up
		px4_arch_gpiowrite(GPIO_RC_OUT, 1);
	}
}
#endif

void
PX4FMU::pwm_output_set(unsigned i, unsigned value)
{
	if (_pwm_initialized) {
		up_pwm_servo_set(i, value);
	}
}

void
PX4FMU::update_pwm_out_state(bool on)
{
	if (on && !_pwm_initialized && _pwm_mask != 0) {
		up_pwm_servo_init(_pwm_mask);
		set_pwm_rate(_pwm_alt_rate_channels, _pwm_default_rate, _pwm_alt_rate);
		_pwm_initialized = true;

	} else {
		_pwm_initialized = false;
	}

	up_pwm_servo_arm(on);
}

void PX4FMU::setWIFIstate(int32_t wifi_mode, bool armed)
{
#ifdef WIFI_TX
	switch (wifi_mode) {
	case 0: /* always off */
		WIFI_TX(0);
		break;

	case 1: /* off when armed */
		if (armed) {
			WIFI_TX(0);

		} else {
			WIFI_TX(1);
		}

		break;

	case 2: /* always on */
		WIFI_TX(1);
		break;
	}
#endif
}

void
PX4FMU::cycle()
{
	if (!_initialized) {
		/* force a reset of the update rate */
		_current_update_rate = 0;

		_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
		_param_sub = orb_subscribe(ORB_ID(parameter_update));
		_adc_sub = orb_subscribe(ORB_ID(adc_report));

		/* initialize PWM limit lib */
		pwm_limit_init(&_pwm_limit);

		update_pwm_rev_mask();

#ifdef WIFI_TX
		/* read wifi TX control parameter and init the TX enable pin */
		if (_wifi_tx_param != PARAM_INVALID) {
			_wifi_tx_param = param_find("WIFI_TX_MODE");
			param_get(_wifi_tx_param, &_wifi_tx_mode);
		}

		if (_wifi_tx_mode != 0) {
			/* WIFI should be on at boot time */
			WIFI_TX(1);
		}
#endif

#ifdef RC_SERIAL_PORT
		// dsm_init sets some file static variables and returns a file descriptor
		_rcs_fd = dsm_init(RC_SERIAL_PORT);
		// assume SBUS input
		sbus_config(_rcs_fd, false);
		// disable CPPM input by mapping it away from the timer capture input
		px4_arch_unconfiggpio(GPIO_PPM_IN);
#endif

		_initialized = true;
	}

	if (_groups_subscribed != _groups_required) {
		subscribe();
		_groups_subscribed = _groups_required;
		/* force setting update rate */
		_current_update_rate = 0;
	}

	/*
	 * Adjust actuator topic update rate to keep up with
	 * the highest servo update rate configured.
	 *
	 * We always mix at max rate; some channels may update slower.
	 */
	unsigned max_rate = (_pwm_default_rate > _pwm_alt_rate) ? _pwm_default_rate : _pwm_alt_rate;

	if (_current_update_rate != max_rate) {
		_current_update_rate = max_rate;
		int update_rate_in_ms = int(1000 / _current_update_rate);

		/* reject faster than 500 Hz updates */
		if (update_rate_in_ms < 2) {
			update_rate_in_ms = 2;
		}

		/* reject slower than 10 Hz updates */
		if (update_rate_in_ms > 100) {
			update_rate_in_ms = 100;
		}

		DEVICE_DEBUG("adjusted actuator update interval to %ums", update_rate_in_ms);

		for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
			if (_control_subs[i] > 0) {
				orb_set_interval(_control_subs[i], update_rate_in_ms);
			}
		}

		// set to current max rate, even if we are actually checking slower/faster
		_current_update_rate = max_rate;
	}

	/* check if anything updated */
	int ret = ::poll(_poll_fds, _poll_fds_num, 0);

	/* log if main actuator updated and sync */
	int main_out_latency = 0;

	/* this would be bad... */
	if (ret < 0) {
		DEVICE_LOG("poll error %d", errno);

	} else if (ret == 0) {
		/* timeout: no control data, switch to failsafe values */
//			warnx("no PWM: failsafe");

	} else {

		/* get controls for required topics */
		unsigned poll_id = 0;

		for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
			if (_control_subs[i] > 0) {
				if (_poll_fds[poll_id].revents & POLLIN) {
					orb_copy(_control_topics[i], _control_subs[i], &_controls[i]);

					/* main outputs */
					if (i == 0) {
//						main_out_latency = hrt_absolute_time() - _controls[i].timestamp - 250;
//						warnx("lat: %llu", hrt_absolute_time() - _controls[i].timestamp);

						/* do only correct within the current phase */
						if (abs(main_out_latency) > SCHEDULE_INTERVAL) {
							main_out_latency = SCHEDULE_INTERVAL;
						}

						if (main_out_latency < 250) {
							main_out_latency = 0;
						}
					}
				}

				poll_id++;
			}
		}

		/* can we mix? */
		if (_mixers != nullptr) {

			size_t num_outputs;

			switch (_mode) {
			case MODE_2PWM:
			case MODE_2PWM2CAP:
				num_outputs = 2;
				break;

			case MODE_3PWM:
			case MODE_3PWM1CAP:
				num_outputs = 3;
				break;

			case MODE_4PWM:
				num_outputs = 4;
				break;

			case MODE_6PWM:
				num_outputs = 6;
				break;

			case MODE_8PWM:
				num_outputs = 8;
				break;

			default:
				num_outputs = 0;
				break;
			}

			/* do mixing */
			float outputs[_max_actuators];
			num_outputs = _mixers->mix(outputs, num_outputs, NULL);

			/* disable unused ports by setting their output to NaN */
			for (size_t i = 0; i < sizeof(outputs) / sizeof(outputs[0]); i++) {
				if (i >= num_outputs) {
					outputs[i] = NAN_VALUE;
				}
			}

			uint16_t pwm_limited[_max_actuators];

			/* the PWM limit call takes care of out of band errors, NaN and constrains */
			pwm_limit_calc(_throttle_armed, arm_nothrottle(), num_outputs, _reverse_pwm_mask, _disarmed_pwm, _min_pwm, _max_pwm,
				       outputs, pwm_limited, &_pwm_limit);

			/* overwrite outputs in case of lockdown with disarmed PWM values */
			if (_armed.lockdown) {
				for (size_t i = 0; i < num_outputs; i++) {
					pwm_limited[i] = _disarmed_pwm[i];
				}
			}

			/* output to the servos */
			for (size_t i = 0; i < num_outputs; i++) {
				pwm_output_set(i, pwm_limited[i]);
			}

			publish_pwm_outputs(pwm_limited, num_outputs);
		}
	}

	_cycle_timestamp = hrt_absolute_time();

#ifdef GPIO_BTN_SAFETY

	if (_cycle_timestamp - _last_safety_check >= (unsigned int)1e5) {
		_last_safety_check = _cycle_timestamp;

		/**
		 * Get and handle the safety status at 10Hz
		 */
		struct safety_s safety = {};

		if (_safety_disabled) {
			/* safety switch disabled, turn LED on solid */
			px4_arch_gpiowrite(GPIO_LED_SAFETY, 0);
			_safety_off = true;

		} else {
			/* read safety switch input and control safety switch LED at 10Hz */
			safety_check_button();
		}

		safety.timestamp = hrt_absolute_time();

		if (_safety_off) {
			safety.safety_off = true;
			safety.safety_switch_available = true;

		} else {
			safety.safety_off = false;
			safety.safety_switch_available = true;
		}

		/* lazily publish the safety status */
		if (_to_safety != nullptr) {
			orb_publish(ORB_ID(safety), _to_safety, &safety);

		} else {
			_to_safety = orb_advertise(ORB_ID(safety), &safety);
		}
	}

#endif
	/* check arming state */
	bool updated = false;
	orb_check(_armed_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);

		/* update the armed status and check that we're not locked down */
		_throttle_armed = _safety_off && _armed.armed && !_armed.lockdown;

		/* update WIFI TX state */
		setWIFIstate(_wifi_tx_mode, _armed.armed);

		/* update PWM status if armed or if disarmed PWM values are set */
		bool pwm_on = _armed.armed || _num_disarmed_set > 0;

		if (_pwm_on != pwm_on) {
			_pwm_on = pwm_on;

			update_pwm_out_state(pwm_on);
		}
	}

	orb_check(_param_sub, &updated);

	if (updated) {
		/* TODO: since pupdate is never used, these 2 lines must be present to achieve a side-effect? */
		parameter_update_s pupdate;
		orb_copy(ORB_ID(parameter_update), _param_sub, &pupdate);

		update_pwm_rev_mask();

		int32_t dsm_bind_val;
		param_t dsm_bind_param;

		/* see if bind parameter has been set, and reset it to -1 */
		param_get(dsm_bind_param = param_find("RC_DSM_BIND"), &dsm_bind_val);

		if (dsm_bind_val > -1) {
			dsm_bind_ioctl(dsm_bind_val);
			dsm_bind_val = -1;
			param_set(dsm_bind_param, &dsm_bind_val);
		}

		/* check for update to WIFI control parameter */
		if (_wifi_tx_param != PARAM_INVALID) {
			param_get(_wifi_tx_param, &_wifi_tx_mode);
			setWIFIstate(_wifi_tx_mode, _armed.armed);
		}
	}

	/* update ADC sampling */
#ifdef ADC_RC_RSSI_CHANNEL
	orb_check(_adc_sub, &updated);

	if (updated) {

		struct adc_report_s adc;
		orb_copy(ORB_ID(adc_report), _adc_sub, &adc);
		const unsigned adc_chans = sizeof(adc.channel_id) / sizeof(adc.channel_id[0]);

		for (unsigned i = 0; i < adc_chans; i++) {
			if (adc.channel_id[i] == ADC_RC_RSSI_CHANNEL) {

				if (_analog_rc_rssi_volt < 0.0f) {
					_analog_rc_rssi_volt = adc.channel_value[i];
				}

				_analog_rc_rssi_volt = _analog_rc_rssi_volt * 0.995f + adc.channel_value[i] * 0.005f;

				/* only allow this to be used if we see a high RSSI once */
				if (_analog_rc_rssi_volt > 2.5f) {
					_analog_rc_rssi_stable = true;
				}
			}
		}
	}

#endif

	bool rc_updated = false;

#ifdef RC_SERIAL_PORT
	// This block scans for a supported serial RC input and locks onto the first one found
	// Scan for 100 msec, then switch protocol
	constexpr hrt_abstime rc_scan_max = 100 * 1000;

	bool sbus_failsafe, sbus_frame_drop;
	uint16_t raw_rc_values[input_rc_s::RC_INPUT_MAX_CHANNELS];
	uint16_t raw_rc_count;
	unsigned frame_drops;
	bool dsm_11_bit;


	if (_report_lock && _rc_scan_locked) {
		_report_lock = false;
		//warnx("RCscan: %s RC input locked", RC_SCAN_STRING[_rc_scan_state]);
	}

	// read all available data from the serial RC input UART
	int newBytes = ::read(_rcs_fd, &_rcs_buf[0], SBUS_FRAME_SIZE);

	switch (_rc_scan_state) {
	case RC_SCAN_SBUS:
		if (_rc_scan_begin == 0) {
			_rc_scan_begin = _cycle_timestamp;
			// Configure serial port for SBUS
			sbus_config(_rcs_fd, false);
			rc_io_invert(true);

		} else if (_rc_scan_locked
			   || _cycle_timestamp - _rc_scan_begin < rc_scan_max) {

			// parse new data
			if (newBytes > 0) {
				rc_updated = sbus_parse(_cycle_timestamp, &_rcs_buf[0], newBytes, &raw_rc_values[0], &raw_rc_count, &sbus_failsafe,
							&sbus_frame_drop, &frame_drops, input_rc_s::RC_INPUT_MAX_CHANNELS);

				if (rc_updated) {
					// we have a new SBUS frame. Publish it.
					_rc_in.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_SBUS;

					fill_rc_in(raw_rc_count, raw_rc_values, _cycle_timestamp,
						   sbus_frame_drop, sbus_failsafe, frame_drops);
					_rc_scan_locked = true;
				}
			}

		} else {
			// Scan the next protocol
			set_rc_scan_state(RC_SCAN_DSM);
		}

		break;

	case RC_SCAN_DSM:
		if (_rc_scan_begin == 0) {
			_rc_scan_begin = _cycle_timestamp;
//			// Configure serial port for DSM
			dsm_config(_rcs_fd);
			rc_io_invert(false);

		} else if (_rc_scan_locked
			   || _cycle_timestamp - _rc_scan_begin < rc_scan_max) {

			if (newBytes > 0) {
				// parse new data
				rc_updated = dsm_parse(_cycle_timestamp, &_rcs_buf[0], newBytes, &raw_rc_values[0], &raw_rc_count,
						       &dsm_11_bit, &frame_drops, input_rc_s::RC_INPUT_MAX_CHANNELS);

				if (rc_updated) {
					// we have a new DSM frame. Publish it.
					_rc_in.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_DSM;
					fill_rc_in(raw_rc_count, raw_rc_values, _cycle_timestamp,
						   false, false, frame_drops);
					_rc_scan_locked = true;
				}
			}

		} else {
			// Scan the next protocol
			set_rc_scan_state(RC_SCAN_ST24);
		}

		break;

	case RC_SCAN_ST24:
		if (_rc_scan_begin == 0) {
			_rc_scan_begin = _cycle_timestamp;
			// Configure serial port for DSM
			dsm_config(_rcs_fd);
			rc_io_invert(false);

		} else if (_rc_scan_locked
			   || _cycle_timestamp - _rc_scan_begin < rc_scan_max) {

			if (newBytes > 0) {
				// parse new data
				uint8_t st24_rssi, rx_count;

				rc_updated = false;

				for (unsigned i = 0; i < (unsigned)newBytes; i++) {
					/* set updated flag if one complete packet was parsed */
					st24_rssi = RC_INPUT_RSSI_MAX;
					rc_updated = (OK == st24_decode(_rcs_buf[i], &st24_rssi, &rx_count,
									&raw_rc_count, raw_rc_values, input_rc_s::RC_INPUT_MAX_CHANNELS));
				}

				if (rc_updated) {
					// we have a new ST24 frame. Publish it.
					_rc_in.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_ST24;
					fill_rc_in(raw_rc_count, raw_rc_values, _cycle_timestamp,
						   false, false, frame_drops, st24_rssi);
					_rc_scan_locked = true;
				}
			}

		} else {
			// Scan the next protocol
			set_rc_scan_state(RC_SCAN_SUMD);
		}

		break;

	case RC_SCAN_SUMD:
		if (_rc_scan_begin == 0) {
			_rc_scan_begin = _cycle_timestamp;
			// Configure serial port for DSM
			dsm_config(_rcs_fd);
			rc_io_invert(false);

		} else if (_rc_scan_locked
			   || _cycle_timestamp - _rc_scan_begin < rc_scan_max) {

			if (newBytes > 0) {
				// parse new data
				uint8_t sumd_rssi, rx_count;

				rc_updated = false;

				for (unsigned i = 0; i < (unsigned)newBytes; i++) {
					/* set updated flag if one complete packet was parsed */
					sumd_rssi = RC_INPUT_RSSI_MAX;
					rc_updated = (OK == sumd_decode(_rcs_buf[i], &sumd_rssi, &rx_count,
									&raw_rc_count, raw_rc_values, input_rc_s::RC_INPUT_MAX_CHANNELS));
				}

				if (rc_updated) {
					// we have a new SUMD frame. Publish it.
					_rc_in.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_SUMD;
					fill_rc_in(raw_rc_count, raw_rc_values, _cycle_timestamp,
						   false, false, frame_drops, sumd_rssi);
					_rc_scan_locked = true;
				}
			}

		} else {
			// Scan the next protocol
			set_rc_scan_state(RC_SCAN_SUMD);
		}

		set_rc_scan_state(RC_SCAN_PPM);
		break;

	case RC_SCAN_PPM:
		// skip PPM if it's not supported
#ifdef HRT_PPM_CHANNEL
		if (_rc_scan_begin == 0) {
			_rc_scan_begin = _cycle_timestamp;
			// Configure timer input pin for CPPM
			px4_arch_configgpio(GPIO_PPM_IN);
			rc_io_invert(false);

		} else if (_rc_scan_locked
			   || _cycle_timestamp - _rc_scan_begin < rc_scan_max) {

			// see if we have new PPM input data
			if ((ppm_last_valid_decode != _rc_in.timestamp_last_signal)
			    && ppm_decoded_channels > 3) {
				// we have a new PPM frame. Publish it.
				rc_updated = true;
				_rc_in.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_PPM;
				fill_rc_in(ppm_decoded_channels, ppm_buffer, _cycle_timestamp,
					   false, false, 0);
				_rc_scan_locked = true;
				_rc_in.rc_ppm_frame_length = ppm_frame_length;
				_rc_in.timestamp_last_signal = ppm_last_valid_decode;
			}

		} else {
			// disable CPPM input by mapping it away from the timer capture input
			px4_arch_unconfiggpio(GPIO_PPM_IN);
			// Scan the next protocol
			set_rc_scan_state(RC_SCAN_SBUS);
		}

#else   // skip PPM if it's not supported
		set_rc_scan_state(RC_SCAN_SBUS);

#endif  // HRT_PPM_CHANNEL

		break;
	}

#else  // RC_SERIAL_PORT not defined
#ifdef HRT_PPM_CHANNEL

	// see if we have new PPM input data
	if ((ppm_last_valid_decode != _rc_in.timestamp_last_signal)
	    && ppm_decoded_channels > 3) {
		// we have a new PPM frame. Publish it.
		rc_updated = true;
		fill_rc_in(ppm_decoded_channels, ppm_buffer, hrt_absolute_time(),
			   false, false, 0);
	}

#endif  // HRT_PPM_CHANNEL
#endif  // RC_SERIAL_PORT

	if (rc_updated) {
		/* lazily advertise on first publication */
		if (_to_input_rc == nullptr) {
			_to_input_rc = orb_advertise(ORB_ID(input_rc), &_rc_in);

		} else {
			orb_publish(ORB_ID(input_rc), _to_input_rc, &_rc_in);
		}
	}

	work_queue(HPWORK, &_work, (worker_t)&PX4FMU::cycle_trampoline, this,
		   USEC2TICK(SCHEDULE_INTERVAL - main_out_latency));
}

void PX4FMU::work_stop()
{
	work_cancel(HPWORK, &_work);

	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		if (_control_subs[i] > 0) {
			::close(_control_subs[i]);
			_control_subs[i] = -1;
		}
	}

	::close(_armed_sub);
	::close(_param_sub);

	/* make sure servos are off */
	up_pwm_servo_deinit();

	DEVICE_LOG("stopping");

	/* note - someone else is responsible for restoring the GPIO config */

	/* tell the dtor that we are exiting */
	_initialized = false;
}

int
PX4FMU::control_callback(uintptr_t handle,
			 uint8_t control_group,
			 uint8_t control_index,
			 float &input)
{
	const actuator_controls_s *controls = (actuator_controls_s *)handle;

	input = controls[control_group].control[control_index];

	/* limit control input */
	if (input > 1.0f) {
		input = 1.0f;

	} else if (input < -1.0f) {
		input = -1.0f;
	}

	/* motor spinup phase - lock throttle to zero */
	if (_pwm_limit.state == PWM_LIMIT_STATE_RAMP) {
		if ((control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE ||
		     control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE_ALTERNATE) &&
		    control_index == actuator_controls_s::INDEX_THROTTLE) {
			/* limit the throttle output to zero during motor spinup,
			 * as the motors cannot follow any demand yet
			 */
			input = 0.0f;
		}
	}

	/* throttle not arming - mark throttle input as invalid */
	if (arm_nothrottle()) {
		if ((control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE ||
		     control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE_ALTERNATE) &&
		    control_index == actuator_controls_s::INDEX_THROTTLE) {
			/* set the throttle to an invalid value */
			input = NAN_VALUE;
		}
	}

	return 0;
}

int
PX4FMU::ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret;

	/* try it as a GPIO ioctl first */
	ret = gpio_ioctl(filp, cmd, arg);

	if (ret != -ENOTTY) {
		return ret;
	}

	/* try it as a Capture ioctl next */
	ret = capture_ioctl(filp, cmd, arg);

	if (ret != -ENOTTY) {
		return ret;
	}

	/* if we are in valid PWM mode, try it as a PWM ioctl as well */
	switch (_mode) {
	case MODE_2PWM:
	case MODE_3PWM:
	case MODE_4PWM:
	case MODE_2PWM2CAP:
	case MODE_3PWM1CAP:
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 6
	case MODE_6PWM:
#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 8
	case MODE_8PWM:
#endif
		ret = pwm_ioctl(filp, cmd, arg);
		break;

	default:
		DEVICE_DEBUG("not in a PWM mode");
		break;
	}

	/* if nobody wants it, let CDev have it */
	if (ret == -ENOTTY) {
		ret = CDev::ioctl(filp, cmd, arg);
	}

	return ret;
}

int
PX4FMU::pwm_ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret = OK;

	lock();

	switch (cmd) {
	case PWM_SERVO_ARM:
		update_pwm_out_state(true);
		break;

	case PWM_SERVO_SET_ARM_OK:
	case PWM_SERVO_CLEAR_ARM_OK:
		break;

	case PWM_SERVO_SET_FORCE_SAFETY_OFF:
		/* force safety switch off */
		_safety_off = true;
		break;

	case PWM_SERVO_SET_FORCE_SAFETY_ON:
		/* force safety switch on */
		_safety_off = false;
		break;

	case PWM_SERVO_DISARM:
		update_pwm_out_state(false);
		break;

	case PWM_SERVO_GET_DEFAULT_UPDATE_RATE:
		*(uint32_t *)arg = _pwm_default_rate;
		break;

	case PWM_SERVO_SET_UPDATE_RATE:
		ret = set_pwm_rate(_pwm_alt_rate_channels, _pwm_default_rate, arg);
		break;

	case PWM_SERVO_GET_UPDATE_RATE:
		*(uint32_t *)arg = _pwm_alt_rate;
		break;

	case PWM_SERVO_SET_SELECT_UPDATE_RATE:
		ret = set_pwm_rate(arg, _pwm_default_rate, _pwm_alt_rate);
		break;

	case PWM_SERVO_GET_SELECT_UPDATE_RATE:
		*(uint32_t *)arg = _pwm_alt_rate_channels;
		break;

	case PWM_SERVO_SET_FAILSAFE_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > _max_actuators) {
				ret = -EINVAL;
				break;
			}

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (pwm->values[i] == 0) {
					/* ignore 0 */
				} else if (pwm->values[i] > PWM_HIGHEST_MAX) {
					_failsafe_pwm[i] = PWM_HIGHEST_MAX;

				} else if (pwm->values[i] < PWM_LOWEST_MIN) {
					_failsafe_pwm[i] = PWM_LOWEST_MIN;

				} else {
					_failsafe_pwm[i] = pwm->values[i];
				}
			}

			/*
			 * update the counter
			 * this is needed to decide if disarmed PWM output should be turned on or not
			 */
			_num_failsafe_set = 0;

			for (unsigned i = 0; i < _max_actuators; i++) {
				if (_failsafe_pwm[i] > 0) {
					_num_failsafe_set++;
				}
			}

			break;
		}

	case PWM_SERVO_GET_FAILSAFE_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _max_actuators; i++) {
				pwm->values[i] = _failsafe_pwm[i];
			}

			pwm->channel_count = _max_actuators;
			break;
		}

	case PWM_SERVO_SET_DISARMED_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > _max_actuators) {
				ret = -EINVAL;
				break;
			}

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (pwm->values[i] == 0) {
					/* ignore 0 */
				} else if (pwm->values[i] > PWM_HIGHEST_MAX) {
					_disarmed_pwm[i] = PWM_HIGHEST_MAX;

				} else if (pwm->values[i] < PWM_LOWEST_MIN) {
					_disarmed_pwm[i] = PWM_LOWEST_MIN;

				} else {
					_disarmed_pwm[i] = pwm->values[i];
				}
			}

			/*
			 * update the counter
			 * this is needed to decide if disarmed PWM output should be turned on or not
			 */
			_num_disarmed_set = 0;

			for (unsigned i = 0; i < _max_actuators; i++) {
				if (_disarmed_pwm[i] > 0) {
					_num_disarmed_set++;
				}
			}

			break;
		}

	case PWM_SERVO_GET_DISARMED_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _max_actuators; i++) {
				pwm->values[i] = _disarmed_pwm[i];
			}

			pwm->channel_count = _max_actuators;
			break;
		}

	case PWM_SERVO_SET_MIN_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > _max_actuators) {
				ret = -EINVAL;
				break;
			}

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (pwm->values[i] == 0) {
					/* ignore 0 */
				} else if (pwm->values[i] > PWM_HIGHEST_MIN) {
					_min_pwm[i] = PWM_HIGHEST_MIN;

				} else if (pwm->values[i] < PWM_LOWEST_MIN) {
					_min_pwm[i] = PWM_LOWEST_MIN;

				} else {
					_min_pwm[i] = pwm->values[i];
				}
			}

			break;
		}

	case PWM_SERVO_GET_MIN_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _max_actuators; i++) {
				pwm->values[i] = _min_pwm[i];
			}

			pwm->channel_count = _max_actuators;
			arg = (unsigned long)&pwm;
			break;
		}

	case PWM_SERVO_SET_MAX_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > _max_actuators) {
				ret = -EINVAL;
				break;
			}

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (pwm->values[i] == 0) {
					/* ignore 0 */
				} else if (pwm->values[i] < PWM_LOWEST_MAX) {
					_max_pwm[i] = PWM_LOWEST_MAX;

				} else if (pwm->values[i] > PWM_HIGHEST_MAX) {
					_max_pwm[i] = PWM_HIGHEST_MAX;

				} else {
					_max_pwm[i] = pwm->values[i];
				}
			}

			break;
		}

	case PWM_SERVO_GET_MAX_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _max_actuators; i++) {
				pwm->values[i] = _max_pwm[i];
			}

			pwm->channel_count = _max_actuators;
			arg = (unsigned long)&pwm;
			break;
		}

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 8

	case PWM_SERVO_SET(7):
	case PWM_SERVO_SET(6):
		if (_mode < MODE_8PWM) {
			ret = -EINVAL;
			break;
		}

#endif

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 6

	case PWM_SERVO_SET(5):
	case PWM_SERVO_SET(4):
		if (_mode < MODE_6PWM) {
			ret = -EINVAL;
			break;
		}

#endif

	/* FALLTHROUGH */
	case PWM_SERVO_SET(3):
		if (_mode < MODE_4PWM) {
			ret = -EINVAL;
			break;
		}

	/* FALLTHROUGH */
	case PWM_SERVO_SET(2):
		if (_mode < MODE_3PWM) {
			ret = -EINVAL;
			break;
		}

	/* FALLTHROUGH */
	case PWM_SERVO_SET(1):
	case PWM_SERVO_SET(0):
		if (arg <= 2100) {
			up_pwm_servo_set(cmd - PWM_SERVO_SET(0), arg);

		} else {
			ret = -EINVAL;
		}

		break;

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 8

	case PWM_SERVO_GET(7):
	case PWM_SERVO_GET(6):
		if (_mode < MODE_8PWM) {
			ret = -EINVAL;
			break;
		}

#endif

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 6

	case PWM_SERVO_GET(5):
	case PWM_SERVO_GET(4):
		if (_mode < MODE_6PWM) {
			ret = -EINVAL;
			break;
		}

#endif

	/* FALLTHROUGH */
	case PWM_SERVO_GET(3):
		if (_mode < MODE_4PWM) {
			ret = -EINVAL;
			break;
		}

	/* FALLTHROUGH */
	case PWM_SERVO_GET(2):
		if (_mode < MODE_3PWM) {
			ret = -EINVAL;
			break;
		}

	/* FALLTHROUGH */
	case PWM_SERVO_GET(1):
	case PWM_SERVO_GET(0):
		*(servo_position_t *)arg = up_pwm_servo_get(cmd - PWM_SERVO_GET(0));
		break;

	case PWM_SERVO_GET_RATEGROUP(0):
	case PWM_SERVO_GET_RATEGROUP(1):
	case PWM_SERVO_GET_RATEGROUP(2):
	case PWM_SERVO_GET_RATEGROUP(3):
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 6
	case PWM_SERVO_GET_RATEGROUP(4):
	case PWM_SERVO_GET_RATEGROUP(5):
#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 8
	case PWM_SERVO_GET_RATEGROUP(6):
	case PWM_SERVO_GET_RATEGROUP(7):
#endif
		*(uint32_t *)arg = up_pwm_servo_get_rate_group(cmd - PWM_SERVO_GET_RATEGROUP(0));
		break;

	case PWM_SERVO_GET_COUNT:
	case MIXERIOCGETOUTPUTCOUNT:
		switch (_mode) {

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 8

		case MODE_8PWM:
			*(unsigned *)arg = 8;
			break;
#endif

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 6

		case MODE_6PWM:
			*(unsigned *)arg = 6;
			break;
#endif

		case MODE_4PWM:
			*(unsigned *)arg = 4;
			break;

		case MODE_3PWM:
		case MODE_3PWM1CAP:
			*(unsigned *)arg = 3;
			break;

		case MODE_2PWM:
		case MODE_2PWM2CAP:
			*(unsigned *)arg = 2;
			break;

		default:
			ret = -EINVAL;
			break;
		}

		break;

	case PWM_SERVO_SET_COUNT: {
			/* change the number of outputs that are enabled for
			 * PWM. This is used to change the split between GPIO
			 * and PWM under control of the flight config
			 * parameters. Note that this does not allow for
			 * changing a set of pins to be used for serial on
			 * FMUv1
			 */
			switch (arg) {
			case 0:
				set_mode(MODE_NONE);
				break;

			case 2:
				set_mode(MODE_2PWM);
				break;

			case 3:
				set_mode(MODE_3PWM);
				break;

			case 4:
				set_mode(MODE_4PWM);
				break;

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >=6

			case 6:
				set_mode(MODE_6PWM);
				break;
#endif

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >=8

			case 8:
				set_mode(MODE_8PWM);
				break;
#endif

			default:
				ret = -EINVAL;
				break;
			}

			break;
		}

	case PWM_SERVO_SET_MODE: {
			switch (arg) {
			case PWM_SERVO_MODE_NONE:
				ret = set_mode(MODE_NONE);
				break;

			case PWM_SERVO_MODE_2PWM:
				ret = set_mode(MODE_2PWM);
				break;

			case PWM_SERVO_MODE_2PWM2CAP:
				ret = set_mode(MODE_2PWM2CAP);
				break;

			case PWM_SERVO_MODE_3PWM:
				ret = set_mode(MODE_3PWM);
				break;

			case PWM_SERVO_MODE_3PWM1CAP:
				ret = set_mode(MODE_3PWM1CAP);
				break;

			case PWM_SERVO_MODE_4PWM:
				ret = set_mode(MODE_4PWM);
				break;

			case PWM_SERVO_MODE_6PWM:
				ret = set_mode(MODE_6PWM);
				break;

			case PWM_SERVO_MODE_8PWM:
				ret = set_mode(MODE_8PWM);
				break;

			case PWM_SERVO_MODE_4CAP:
				ret = set_mode(MODE_4CAP);
				break;

			case PWM_SERVO_MODE_5CAP:
				ret = set_mode(MODE_5CAP);
				break;

			case PWM_SERVO_MODE_6CAP:
				ret = set_mode(MODE_6CAP);
				break;

			default:
				ret = -EINVAL;
			}

			break;
		}

#ifdef RC_SERIAL_PORT

	case DSM_BIND_START:
		/* only allow DSM2, DSM-X and DSM-X with more than 7 channels */
		warnx("fmu pwm_ioctl: DSM_BIND_START, arg: %lu", arg);

		if (arg == DSM2_BIND_PULSES ||
		    arg == DSMX_BIND_PULSES ||
		    arg == DSMX8_BIND_PULSES) {

			dsm_bind(DSM_CMD_BIND_POWER_DOWN, 0);
			usleep(500000);

			dsm_bind(DSM_CMD_BIND_SET_RX_OUT, 0);

			dsm_bind(DSM_CMD_BIND_POWER_UP, 0);
			usleep(72000);

			dsm_bind(DSM_CMD_BIND_SEND_PULSES, arg);
			usleep(50000);

			dsm_bind(DSM_CMD_BIND_REINIT_UART, 0);

			ret = OK;

		} else {
			ret = -EINVAL;
		}

		break;
#endif

	case MIXERIOCRESET:
		if (_mixers != nullptr) {
			delete _mixers;
			_mixers = nullptr;
			_groups_required = 0;
		}

		break;

	case MIXERIOCADDSIMPLE: {
			mixer_simple_s *mixinfo = (mixer_simple_s *)arg;

			SimpleMixer *mixer = new SimpleMixer(control_callback,
							     (uintptr_t)_controls, mixinfo);

			if (mixer->check()) {
				delete mixer;
				_groups_required = 0;
				ret = -EINVAL;

			} else {
				if (_mixers == nullptr)
					_mixers = new MixerGroup(control_callback,
								 (uintptr_t)_controls);

				_mixers->add_mixer(mixer);
				_mixers->groups_required(_groups_required);
			}

			break;
		}

	case MIXERIOCLOADBUF: {
			const char *buf = (const char *)arg;
			unsigned buflen = strnlen(buf, 1024);

			if (_mixers == nullptr) {
				_mixers = new MixerGroup(control_callback, (uintptr_t)_controls);
			}

			if (_mixers == nullptr) {
				_groups_required = 0;
				ret = -ENOMEM;

			} else {

				ret = _mixers->load_from_buf(buf, buflen);

				if (ret != 0) {
					DEVICE_DEBUG("mixer load failed with %d", ret);
					delete _mixers;
					_mixers = nullptr;
					_groups_required = 0;
					ret = -EINVAL;

				} else {

					_mixers->groups_required(_groups_required);
				}
			}

			break;
		}

	default:
		ret = -ENOTTY;
		break;
	}

	unlock();

	return ret;
}

/*
  this implements PWM output via a write() method, for compatibility
  with px4io
 */
ssize_t
PX4FMU::write(file *filp, const char *buffer, size_t len)
{
	unsigned count = len / 2;
	uint16_t values[8];

	if (count > BOARD_HAS_PWM) {
		// we have at most BOARD_HAS_PWM outputs
		count = BOARD_HAS_PWM;
	}

	// allow for misaligned values
	memcpy(values, buffer, count * 2);

	for (uint8_t i = 0; i < count; i++) {
		if (values[i] != PWM_IGNORE_THIS_CHANNEL) {
			up_pwm_servo_set(i, values[i]);
		}
	}

	return count * 2;
}

void
PX4FMU::sensor_reset(int ms)
{
	if (ms < 1) {
		ms = 1;
	}

	board_spi_reset(ms);
}

void
PX4FMU::peripheral_reset(int ms)
{
	if (ms < 1) {
		ms = 10;
	}

	board_peripheral_reset(ms);
}

void
PX4FMU::gpio_reset(void)
{
	/*
	 * Setup default GPIO config - all pins as GPIOs, input if
	 * possible otherwise output if possible.
	 */
	for (unsigned i = 0; i < _ngpio; i++) {
		if (_gpio_tab[i].input != 0) {
			px4_arch_configgpio(_gpio_tab[i].input);

		} else if (_gpio_tab[i].output != 0) {
			px4_arch_configgpio(_gpio_tab[i].output);
		}
	}

#if defined(GPIO_GPIO_DIR)
	/* if we have a GPIO direction control, set it to zero (input) */
	px4_arch_gpiowrite(GPIO_GPIO_DIR, 0);
	px4_arch_configgpio(GPIO_GPIO_DIR);
#endif
}

void
PX4FMU::gpio_set_function(uint32_t gpios, int function)
{
#if defined(BOARD_GPIO_SHARED_BUFFERED_BITS) && defined(GPIO_GPIO_DIR)

	/*
	 * GPIOs 0 and 1 must have the same direction as they are buffered
	 * by a shared 2-port driver.  Any attempt to set either sets both.
	 */
	if (gpios & BOARD_GPIO_SHARED_BUFFERED_BITS) {
		gpios |= BOARD_GPIO_SHARED_BUFFERED_BITS;

		/* flip the buffer to output mode if required */
		if (GPIO_SET_OUTPUT == function ||
		    GPIO_SET_OUTPUT_LOW == function ||
		    GPIO_SET_OUTPUT_HIGH == function) {
			px4_arch_gpiowrite(GPIO_GPIO_DIR, 1);
		}
	}

#endif

	/* configure selected GPIOs as required */
	for (unsigned i = 0; i < _ngpio; i++) {
		if (gpios & (1 << i)) {
			switch (function) {
			case GPIO_SET_INPUT:
				px4_arch_configgpio(_gpio_tab[i].input);
				break;

			case GPIO_SET_OUTPUT:
				px4_arch_configgpio(_gpio_tab[i].output);
				break;

			case GPIO_SET_OUTPUT_LOW:
				px4_arch_configgpio((_gpio_tab[i].output & ~(GPIO_OUTPUT_SET)) | GPIO_OUTPUT_CLEAR);
				break;

			case GPIO_SET_OUTPUT_HIGH:
				px4_arch_configgpio((_gpio_tab[i].output & ~(GPIO_OUTPUT_CLEAR)) | GPIO_OUTPUT_SET);
				break;

			case GPIO_SET_ALT_1:
				if (_gpio_tab[i].alt != 0) {
					px4_arch_configgpio(_gpio_tab[i].alt);
				}

				break;
			}
		}
	}

#if defined(BOARD_GPIO_SHARED_BUFFERED_BITS) && defined(GPIO_GPIO_DIR)

	/* flip buffer to input mode if required */
	if ((GPIO_SET_INPUT == function) && (gpios & BOARD_GPIO_SHARED_BUFFERED_BITS)) {
		px4_arch_gpiowrite(GPIO_GPIO_DIR, 0);
	}

#endif
}

void
PX4FMU::gpio_write(uint32_t gpios, int function)
{
	int value = (function == GPIO_SET) ? 1 : 0;

	for (unsigned i = 0; i < _ngpio; i++)
		if (gpios & (1 << i)) {
			px4_arch_gpiowrite(_gpio_tab[i].output, value);
		}
}

uint32_t
PX4FMU::gpio_read(void)
{
	uint32_t bits = 0;

	for (unsigned i = 0; i < _ngpio; i++)
		if (px4_arch_gpioread(_gpio_tab[i].input)) {
			bits |= (1 << i);
		}

	return bits;
}

int
PX4FMU::capture_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int	ret = -EINVAL;

	lock();

	input_capture_config_t *pconfig = 0;

	input_capture_stats_t *stats = (input_capture_stats_t *)arg;

	if (_mode == MODE_3PWM1CAP || _mode == MODE_2PWM2CAP) {

		pconfig = (input_capture_config_t *)arg;
	}

	switch (cmd) {

	case INPUT_CAP_SET:
		if (pconfig) {
			ret =  up_input_capture_set(pconfig->channel, pconfig->edge, pconfig->filter,
						    pconfig->callback, pconfig->context);
		}

		break;

	case INPUT_CAP_SET_CALLBACK:
		if (pconfig) {
			ret =  up_input_capture_set_callback(pconfig->channel, pconfig->callback, pconfig->context);
		}

		break;

	case INPUT_CAP_GET_CALLBACK:
		if (pconfig) {
			ret =  up_input_capture_get_callback(pconfig->channel, &pconfig->callback, &pconfig->context);
		}

		break;

	case INPUT_CAP_GET_STATS:
		if (arg) {
			ret =  up_input_capture_get_stats(stats->chan_in_edges_out, stats, false);
		}

		break;

	case INPUT_CAP_GET_CLR_STATS:
		if (arg) {
			ret =  up_input_capture_get_stats(stats->chan_in_edges_out, stats, true);
		}

		break;

	case INPUT_CAP_SET_EDGE:
		if (pconfig) {
			ret =  up_input_capture_set_trigger(pconfig->channel, pconfig->edge);
		}

		break;

	case INPUT_CAP_GET_EDGE:
		if (pconfig) {
			ret =  up_input_capture_get_trigger(pconfig->channel, &pconfig->edge);
		}

		break;

	case INPUT_CAP_SET_FILTER:
		if (pconfig) {
			ret =  up_input_capture_set_filter(pconfig->channel, pconfig->filter);
		}

		break;

	case INPUT_CAP_GET_FILTER:
		if (pconfig) {
			ret =  up_input_capture_get_filter(pconfig->channel, &pconfig->filter);
		}

		break;

	case INPUT_CAP_GET_COUNT:
		ret = OK;

		switch (_mode) {
		case MODE_3PWM1CAP:
			*(unsigned *)arg = 1;
			break;

		case MODE_2PWM2CAP:
			*(unsigned *)arg = 2;
			break;

		default:
			ret = -EINVAL;
			break;
		}

		break;

	case INPUT_CAP_SET_COUNT:
		ret = OK;

		switch (_mode) {
		case MODE_3PWM1CAP:
			set_mode(MODE_3PWM1CAP);
			break;

		case MODE_2PWM2CAP:
			set_mode(MODE_2PWM2CAP);
			break;

		default:
			ret = -EINVAL;
			break;
		}

		break;

	default:
		ret = -ENOTTY;
		break;
	}

	unlock();
	return ret;
}

int
PX4FMU::gpio_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int	ret = OK;

	lock();

	switch (cmd) {

	case GPIO_RESET:
		gpio_reset();
		break;

	case GPIO_SENSOR_RAIL_RESET:
		sensor_reset(arg);
		break;

	case GPIO_PERIPHERAL_RAIL_RESET:
		peripheral_reset(arg);
		break;

	case GPIO_SET_OUTPUT:
	case GPIO_SET_OUTPUT_LOW:
	case GPIO_SET_OUTPUT_HIGH:
	case GPIO_SET_INPUT:
	case GPIO_SET_ALT_1:
		gpio_set_function(arg, cmd);
		break;

	case GPIO_SET_ALT_2:
	case GPIO_SET_ALT_3:
	case GPIO_SET_ALT_4:
		ret = -EINVAL;
		break;

	case GPIO_SET:
	case GPIO_CLEAR:
		gpio_write(arg, cmd);
		break;

	case GPIO_GET:
		*(uint32_t *)arg = gpio_read();
		break;

	default:
		ret = -ENOTTY;
	}

	unlock();

	return ret;
}

void
PX4FMU::dsm_bind_ioctl(int dsmMode)
{
	if (!_armed.armed) {
//      mavlink_log_info(&_mavlink_log_pub, "[FMU] binding DSM%s RX", (dsmMode == 0) ? "2" : ((dsmMode == 1) ? "-X" : "-X8"));
		warnx("[FMU] binding DSM%s RX", (dsmMode == 0) ? "2" : ((dsmMode == 1) ? "-X" : "-X8"));
		int ret = ioctl(nullptr, DSM_BIND_START,
				(dsmMode == 0) ? DSM2_BIND_PULSES : ((dsmMode == 1) ? DSMX_BIND_PULSES : DSMX8_BIND_PULSES));

		if (ret) {
//            mavlink_log_critical(&_mavlink_log_pub, "binding failed.");
			warnx("binding failed.");
		}

	} else {
//        mavlink_log_info(&_mavlink_log_pub, "[FMU] system armed, bind request rejected");
		warnx("[FMU] system armed, bind request rejected");
	}
}

namespace
{

enum PortMode {
	PORT_MODE_UNSET = 0,
	PORT_FULL_GPIO,
	PORT_FULL_SERIAL,
	PORT_FULL_PWM,
	PORT_GPIO_AND_SERIAL,
	PORT_PWM_AND_SERIAL,
	PORT_PWM_AND_GPIO,
	PORT_PWM4,
	PORT_PWM3,
	PORT_PWM2,
	PORT_PWM3CAP1,
	PORT_PWM2CAP2,
	PORT_CAPTURE,
};

PortMode g_port_mode;

int
fmu_new_mode(PortMode new_mode)
{
	uint32_t gpio_bits;
	PX4FMU::Mode servo_mode;
	bool mode_with_input = false;

	gpio_bits = 0;
	servo_mode = PX4FMU::MODE_NONE;

	switch (new_mode) {
	case PORT_FULL_GPIO:
	case PORT_MODE_UNSET:
		break;

	case PORT_FULL_PWM:
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM == 4
		/* select 4-pin PWM mode */
		servo_mode = PX4FMU::MODE_4PWM;
#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM == 6
		servo_mode = PX4FMU::MODE_6PWM;
#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM == 8
		servo_mode = PX4FMU::MODE_8PWM;
#endif
		break;

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 6

	case PORT_PWM4:
		/* select 4-pin PWM mode */
		servo_mode = PX4FMU::MODE_4PWM;
		break;

	case PORT_PWM3:
		/* select 4-pin PWM mode */
		servo_mode = PX4FMU::MODE_3PWM;
		break;

	case PORT_PWM3CAP1:
		/* select 3-pin PWM mode 1 capture */
		servo_mode = PX4FMU::MODE_3PWM1CAP;
		mode_with_input = true;
		break;

	case PORT_PWM2:
		/* select 2-pin PWM mode */
		servo_mode = PX4FMU::MODE_2PWM;
		break;

	case PORT_PWM2CAP2:
		/* select 2-pin PWM mode 2 capture */
		servo_mode = PX4FMU::MODE_2PWM2CAP;
		mode_with_input = true;
		break;
#endif

		/* mixed modes supported on v1 board only */
#if defined(BOARD_HAS_MULTI_PURPOSE_GPIO)

	case PORT_FULL_SERIAL:
		/* set all multi-GPIOs to serial mode */
		gpio_bits = GPIO_MULTI_1 | GPIO_MULTI_2 | GPIO_MULTI_3 | GPIO_MULTI_4;
		mode_with_input = true;
		break;

	case PORT_GPIO_AND_SERIAL:
		/* set RX/TX multi-GPIOs to serial mode */
		gpio_bits = GPIO_MULTI_3 | GPIO_MULTI_4;
		mode_with_input = true;
		break;

	case PORT_PWM_AND_SERIAL:
		/* select 2-pin PWM mode */
		servo_mode = PX4FMU::MODE_2PWM;
		/* set RX/TX multi-GPIOs to serial mode */
		gpio_bits = GPIO_MULTI_3 | GPIO_MULTI_4;
		mode_with_input = true;
		break;

	case PORT_PWM_AND_GPIO:
		/* select 2-pin PWM mode */
		servo_mode = PX4FMU::MODE_2PWM;
		mode_with_input = true;
		break;
#endif

	default:
		return -1;
	}

	if (servo_mode != g_fmu->get_mode()) {

		/* reset to all-inputs */
		if (mode_with_input) {
			g_fmu->ioctl(0, GPIO_RESET, 0);

			/* adjust GPIO config for serial mode(s) */
			if (gpio_bits != 0) {
				g_fmu->ioctl(0, GPIO_SET_ALT_1, gpio_bits);
			}
		}

		/* (re)set the PWM output mode */
		g_fmu->set_mode(servo_mode);
	}

	return OK;
}

int fmu_new_i2c_speed(unsigned bus, unsigned clock_hz)
{
	return PX4FMU::set_i2c_bus_clock(bus, clock_hz);
}

int
fmu_start(void)
{
	int ret = OK;

	if (g_fmu == nullptr) {

		g_fmu = new PX4FMU;

		if (g_fmu == nullptr) {
			ret = -ENOMEM;

		} else {
			ret = g_fmu->init();

			if (ret != OK) {
				delete g_fmu;
				g_fmu = nullptr;
			}
		}
	}

	return ret;
}

int
fmu_stop(void)
{
	int ret = OK;

	if (g_fmu != nullptr) {

		delete g_fmu;
		g_fmu = nullptr;
	}

	return ret;
}

void
sensor_reset(int ms)
{
	int	 fd;

	fd = open(PX4FMU_DEVICE_PATH, O_RDWR);

	if (fd < 0) {
		errx(1, "open fail");
	}

	if (ioctl(fd, GPIO_SENSOR_RAIL_RESET, ms) < 0) {
		warnx("sensor rail reset failed");
	}

	close(fd);
}

void
peripheral_reset(int ms)
{
	int	 fd;

	fd = open(PX4FMU_DEVICE_PATH, O_RDWR);

	if (fd < 0) {
		errx(1, "open fail");
	}

	if (ioctl(fd, GPIO_PERIPHERAL_RAIL_RESET, ms) < 0) {
		warnx("peripheral rail reset failed");
	}

	close(fd);
}

void
test(void)
{
	int	 fd;
	unsigned servo_count = 0;
	unsigned capture_count = 0;
	unsigned pwm_value = 1000;
	int	 direction = 1;
	int	 ret;
	uint32_t rate_limit = 0;
	struct input_capture_t {
		bool valid;
		input_capture_config_t  chan;
	} capture_conf[INPUT_CAPTURE_MAX_CHANNELS];

	fd = open(PX4FMU_DEVICE_PATH, O_RDWR);

	if (fd < 0) {
		errx(1, "open fail");
	}

	if (ioctl(fd, PWM_SERVO_ARM, 0) < 0) { err(1, "servo arm failed"); }

	if (ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count) != 0) {
		err(1, "Unable to get servo count\n");
	}

	if (ioctl(fd, INPUT_CAP_GET_COUNT, (unsigned long)&capture_count) != 0) {
		fprintf(stdout, "Not in a capture mode\n");
	}

	warnx("Testing %u servos and %u input captures", (unsigned)servo_count, capture_count);
	memset(capture_conf, 0, sizeof(capture_conf));

	if (capture_count != 0) {
		for (unsigned i = 0; i < capture_count; i++) {
			// Map to channel number
			capture_conf[i].chan.channel = i + servo_count;

			/* Save handler */
			if (ioctl(fd, INPUT_CAP_GET_CALLBACK, (unsigned long)&capture_conf[i].chan.channel) != 0) {
				err(1, "Unable to get capture callback for chan %u\n", capture_conf[i].chan.channel);

			} else {
				input_capture_config_t conf = capture_conf[i].chan;
				conf.callback = &PX4FMU::capture_trampoline;
				conf.context = g_fmu;

				if (ioctl(fd, INPUT_CAP_SET_CALLBACK, (unsigned long)&conf) == 0) {
					capture_conf[i].valid = true;

				} else {
					err(1, "Unable to set capture callback for chan %u\n", capture_conf[i].chan.channel);
				}
			}

		}
	}

	struct pollfd fds;

	fds.fd = 0; /* stdin */

	fds.events = POLLIN;

	warnx("Press CTRL-C or 'c' to abort.");

	for (;;) {
		/* sweep all servos between 1000..2000 */
		servo_position_t servos[servo_count];

		for (unsigned i = 0; i < servo_count; i++) {
			servos[i] = pwm_value;
		}

		if (direction == 1) {
			// use ioctl interface for one direction
			for (unsigned i = 0; i < servo_count;	i++) {
				if (ioctl(fd, PWM_SERVO_SET(i), servos[i]) < 0) {
					err(1, "servo %u set failed", i);
				}
			}

		} else {
			// and use write interface for the other direction
			ret = write(fd, servos, sizeof(servos));

			if (ret != (int)sizeof(servos)) {
				err(1, "error writing PWM servo data, wrote %u got %d", sizeof(servos), ret);
			}
		}

		if (direction > 0) {
			if (pwm_value < 2000) {
				pwm_value++;

			} else {
				direction = -1;
			}

		} else {
			if (pwm_value > 1000) {
				pwm_value--;

			} else {
				direction = 1;
			}
		}

		/* readback servo values */
		for (unsigned i = 0; i < servo_count; i++) {
			servo_position_t value;

			if (ioctl(fd, PWM_SERVO_GET(i), (unsigned long)&value)) {
				err(1, "error reading PWM servo %d", i);
			}

			if (value != servos[i]) {
				errx(1, "servo %d readback error, got %u expected %u", i, value, servos[i]);
			}
		}

		if (capture_count != 0 && (++rate_limit % 500 == 0)) {
			for (unsigned i = 0; i < capture_count; i++) {
				if (capture_conf[i].valid) {
					input_capture_stats_t stats;
					stats.chan_in_edges_out = capture_conf[i].chan.channel;

					if (ioctl(fd, INPUT_CAP_GET_STATS, (unsigned long)&stats) != 0) {
						err(1, "Unable to get stats for chan %u\n", capture_conf[i].chan.channel);

					} else {
						fprintf(stdout, "FMU: Status chan:%u edges: %d last time:%lld last state:%d overflows:%d lantency:%u\n",
							capture_conf[i].chan.channel,
							stats.chan_in_edges_out,
							stats.last_time,
							stats.last_edge,
							stats.overflows,
							stats.latnecy);
					}
				}
			}

		}

		/* Check if user wants to quit */
		char c;
		ret = poll(&fds, 1, 0);

		if (ret > 0) {

			read(0, &c, 1);

			if (c == 0x03 || c == 0x63 || c == 'q') {
				warnx("User abort\n");
				break;
			}
		}
	}

	if (capture_count != 0) {
		for (unsigned i = 0; i < capture_count; i++) {
			// Map to channel number
			if (capture_conf[i].valid) {
				/* Save handler */
				if (ioctl(fd, INPUT_CAP_SET_CALLBACK, (unsigned long)&capture_conf[i].chan) != 0) {
					err(1, "Unable to set capture callback for chan %u\n", capture_conf[i].chan.channel);
				}
			}
		}
	}

	close(fd);


	exit(0);
}

void
fake(int argc, char *argv[])
{
	if (argc < 5) {
		errx(1, "fmu fake <roll> <pitch> <yaw> <thrust> (values -100 .. 100)");
	}

	actuator_controls_s ac;

	ac.control[0] = strtol(argv[1], 0, 0) / 100.0f;

	ac.control[1] = strtol(argv[2], 0, 0) / 100.0f;

	ac.control[2] = strtol(argv[3], 0, 0) / 100.0f;

	ac.control[3] = strtol(argv[4], 0, 0) / 100.0f;

	orb_advert_t handle = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &ac);

	if (handle == nullptr) {
		errx(1, "advertise failed");
	}

	actuator_armed_s aa;

	aa.armed = true;
	aa.lockdown = false;

	handle = orb_advertise(ORB_ID(actuator_armed), &aa);

	if (handle == nullptr) {
		errx(1, "advertise failed 2");
	}

	exit(0);
}

} // namespace

extern "C" __EXPORT int fmu_main(int argc, char *argv[]);

int
fmu_main(int argc, char *argv[])
{
	PortMode new_mode = PORT_MODE_UNSET;
	const char *verb = argv[1];

	/* does not operate on a FMU instance */
	if (!strcmp(verb, "i2c")) {
		if (argc > 3) {
			int bus = strtol(argv[2], 0, 0);
			int clock_hz = strtol(argv[3], 0, 0);
			int ret = fmu_new_i2c_speed(bus, clock_hz);

			if (ret) {
				errx(ret, "setting I2C clock failed");
			}

			exit(0);

		} else {
			warnx("i2c cmd args: <bus id> <clock Hz>");
		}
	}

	if (!strcmp(verb, "stop")) {
		fmu_stop();
		errx(0, "FMU driver stopped");
	}

	if (!strcmp(verb, "id")) {
		uint8_t id[12];
		(void)get_board_serial(id);

		errx(0, "Board serial:\n %02X%02X%02X%02X %02X%02X%02X%02X %02X%02X%02X%02X",
		     (unsigned)id[0], (unsigned)id[1], (unsigned)id[2], (unsigned)id[3], (unsigned)id[4], (unsigned)id[5],
		     (unsigned)id[6], (unsigned)id[7], (unsigned)id[8], (unsigned)id[9], (unsigned)id[10], (unsigned)id[11]);
	}

	if (fmu_start() != OK) {
		errx(1, "failed to start the FMU driver");
	}

	/*
	 * Mode switches.
	 */
	if (!strcmp(verb, "mode_gpio")) {
		new_mode = PORT_FULL_GPIO;

	} else if (!strcmp(verb, "mode_pwm")) {
		new_mode = PORT_FULL_PWM;

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 6

	} else if (!strcmp(verb, "mode_pwm4")) {
		new_mode = PORT_PWM4;

	} else if (!strcmp(verb, "mode_pwm2")) {
		new_mode = PORT_PWM2;

	} else if (!strcmp(verb, "mode_pwm3")) {
		new_mode = PORT_PWM3;

	} else if (!strcmp(verb, "mode_pwm3cap1")) {
		new_mode = PORT_PWM3CAP1;

	} else if (!strcmp(verb, "mode_pwm2cap2")) {
		new_mode = PORT_PWM2CAP2;
#endif
#if defined(BOARD_HAS_MULTI_PURPOSE_GPIO)

	} else if (!strcmp(verb, "mode_serial")) {
		new_mode = PORT_FULL_SERIAL;

	} else if (!strcmp(verb, "mode_gpio_serial")) {
		new_mode = PORT_GPIO_AND_SERIAL;

	} else if (!strcmp(verb, "mode_pwm_serial")) {
		new_mode = PORT_PWM_AND_SERIAL;

	} else if (!strcmp(verb, "mode_pwm_gpio")) {
		new_mode = PORT_PWM_AND_GPIO;
#endif
	}

	/* was a new mode set? */
	if (new_mode != PORT_MODE_UNSET) {

		/* yes but it's the same mode */
		if (new_mode == g_port_mode) {
			return OK;
		}

		/* switch modes */
		int ret = fmu_new_mode(new_mode);
		exit(ret == OK ? 0 : 1);
	}

	if (!strcmp(verb, "test")) {
		test();
	}

	if (!strcmp(verb, "info")) {
#ifdef RC_SERIAL_PORT
		warnx("frame drops: %u", sbus_dropped_frames());
#endif
		return 0;
	}

	if (!strcmp(verb, "fake")) {
		fake(argc - 1, argv + 1);
	}

	if (!strcmp(verb, "sensor_reset")) {
		if (argc > 2) {
			int reset_time = strtol(argv[2], 0, 0);
			sensor_reset(reset_time);

		} else {
			sensor_reset(0);
			warnx("resettet default time");
		}

		exit(0);
	}

	if (!strcmp(verb, "peripheral_reset")) {
		if (argc > 2) {
			int reset_time = strtol(argv[2], 0, 0);
			peripheral_reset(reset_time);

		} else {
			peripheral_reset(0);
			warnx("resettet default time");
		}

		exit(0);
	}

	fprintf(stderr, "FMU: unrecognised command %s, try:\n", verb);
#if defined(BOARD_HAS_MULTI_PURPOSE_GPIO)
	fprintf(stderr,
		"  mode_gpio, mode_serial, mode_pwm, mode_gpio_serial, mode_pwm_serial, mode_pwm_gpio, test, fake, sensor_reset, id\n");
#elif defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 6
	fprintf(stderr, "  mode_gpio, mode_pwm, mode_pwm4, test, sensor_reset [milliseconds], i2c <bus> <hz>\n");
#endif
	exit(1);
}
