/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file aerocore_rc.cpp
 * Driver for the Spektrum satellite RC receiver.
 */

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <arch/board/board.h>

#include <drivers/device/device.h>
//#include <drivers/drv_baro.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_rc_input.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_sensor.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <uORB/topics/rc_channels.h>
#include <mavlink/mavlink_log.h>

#include "aerocore_rc.h"

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

/* helper macro for handling report buffer indices */
#define INCREMENT(_x, _lim)	do { __typeof__(_x) _tmp = _x+1; if (_tmp >= _lim) _tmp = 0; _x = _tmp; } while(0)

/* helper macro for arithmetic - returns the square of the argument */
#define POW2(_x)		((_x) * (_x))

#define AEROCOREIO_RC_DEVICE_PATH	"/dev/spektrum"
#define AEROCOREIO_RC_CONVERSION_INTERVAL	1000	/* microseconds */

class AeroCoreIO : public device::CDev
{
public:
	AeroCoreIO();
	~AeroCoreIO();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

protected:
	struct work_s		_work;
	unsigned		_measure_ticks;
	unsigned		_max_rc_input;

	unsigned		_rc_chan_count;		///< Internal copy of the last seen number of RC channels
	uint64_t		_rc_last_valid;		///< last valid timestamp

	ringbuffer::RingBuffer		*_reports;

	uint16_t		_status;		///< Various IO status flags

	orb_advert_t		_rc_topic;
	orb_advert_t        _to_input_rc;

	int			_class_instance;
	int			_mavlink_fd;

	perf_counter_t		_sample_perf;
	perf_counter_t		_measure_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t		_buffer_overflows;
	perf_counter_t		_perf_chan_count;	///<local performance counter for channel number changes


	/**
	 * Initialize the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void			start_cycle();

	/**
	 * Stop the automatic measurement state machine.
	 */
	void			stop_cycle();

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 *
	 * This is the heart of the measurement state machine.  This function
	 * alternately starts a measurement, or collects the data from the
	 * previous measurement.
	 *
	 * When the interval between measurements is greater than the minimum
	 * measurement interval, a gap is inserted between collection
	 * and measurement to provide the most recent measurement possible
	 * at the next interval.
	 */
	void			cycle();

	/**
	 * Static trampoline from the workq context; because we don't have a
	 * generic workq wrapper yet.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		cycle_trampoline(void *arg);

	int io_get_raw_rc_input(rc_input_values &input_rc);
	int io_publish_raw_rc();
	/**
	* Push RC channel configuration to IO.
	*/
	int	io_set_rc_config();

};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int aerocore_io_main(int argc, char *argv[]);

AeroCoreIO::AeroCoreIO() :
	CDev("AeroCoreIO", AEROCOREIO_RC_DEVICE_PATH),
	_measure_ticks(0),
	_max_rc_input(0),
	_rc_chan_count(0),
	_rc_last_valid(0),
	_reports(nullptr),
	_status(0),
	_rc_topic(nullptr), //changed from -1
	_to_input_rc(nullptr), //changed from -1
	_class_instance(-1),
	_mavlink_fd(-1),
	_sample_perf(perf_alloc(PC_ELAPSED, "ms5611_read")),
	_measure_perf(perf_alloc(PC_ELAPSED, "ms5611_measure")),
	_comms_errors(perf_alloc(PC_COUNT, "ms5611_comms_errors")),
	_buffer_overflows(perf_alloc(PC_COUNT, "ms5611_buffer_overflows")),
	_perf_chan_count(perf_alloc(PC_COUNT, "io rc #"))
{
	// work_cancel in stop_cycle called from the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

AeroCoreIO::~AeroCoreIO()
{
	/* make sure we are truly inactive */
	stop_cycle();

	if (_class_instance != -1) {
		unregister_class_devname(AEROCOREIO_RC_DEVICE_PATH, _class_instance);
	}

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_measure_perf);
	perf_free(_comms_errors);
	perf_free(_buffer_overflows);

}

int
AeroCoreIO::init()
{
	int ret;

	_mavlink_fd = ::open(MAVLINK_LOG_DEVICE, 0);

	_max_rc_input = PX4IO_RC_INPUT_CHANNELS;

	ret = CDev::init();

	if (ret != OK) {
		debug("CDev init failed");
		goto out;
	}

	// init the controls
	controls_init();

	// publish the RC config values to the register
	ret = io_set_rc_config();

	if (ret != OK) {
		log("failed to update RC input config");
		mavlink_log_info(_mavlink_fd, "[IO] RC config upload fail");
		return ret;
	}

out:
	return ret;
}

int
AeroCoreIO::io_set_rc_config()
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
	/* fill the mapping with an error condition triggering value */
	for (unsigned i = 0; i < _max_rc_input; i++) {
		input_map[i] = UINT8_MAX;
	}

	/*
	* NOTE: The indices for mapped channels are 1-based
	* for compatibility reasons with existing
	* autopilots / GCS'.
	*/
	/* ROLL */
	param_get(param_find("RC_MAP_ROLL"), &ichan);

	if ((ichan > 0) && (ichan <= (int)_max_rc_input)) {
		input_map[ichan - 1] = 0;
	}

	/* PITCH */
	param_get(param_find("RC_MAP_PITCH"), &ichan);

	if ((ichan > 0) && (ichan <= (int)_max_rc_input)) {
		input_map[ichan - 1] = 1;
	}

	/* YAW */
	param_get(param_find("RC_MAP_YAW"), &ichan);

	if ((ichan > 0) && (ichan <= (int)_max_rc_input)) {
		input_map[ichan - 1] = 2;
	}

	/* THROTTLE */
	param_get(param_find("RC_MAP_THROTTLE"), &ichan);

	if ((ichan > 0) && (ichan <= (int)_max_rc_input)) {
		input_map[ichan - 1] = 3;
	}

	/* FLAPS */
	param_get(param_find("RC_MAP_FLAPS"), &ichan);

	if ((ichan > 0) && (ichan <= (int)_max_rc_input)) {
		input_map[ichan - 1] = 4;
	}

	/* MAIN MODE SWITCH */
	param_get(param_find("RC_MAP_MODE_SW"), &ichan);

	if ((ichan > 0) && (ichan <= (int)_max_rc_input)) {
		/* use out of normal bounds index to indicate special channel */
		input_map[ichan - 1] = PX4IO_P_RC_CONFIG_ASSIGNMENT_MODESWITCH;
	}

	/*
	* Iterate all possible RC inputs.
	*/
	for (unsigned i = 0; i < _max_rc_input; i++) {
		uint16_t regs[PX4IO_P_RC_CONFIG_STRIDE];
		char pname[16];
		float fval;
		/*
		* RC params are floats, but do only
		* contain integer values. Do not scale
		* or cast them, let the auto-typeconversion
		* do its job here.
		* Channels: 500 - 2500
		* Inverted flag: -1 (inverted) or 1 (normal)
		*/
		sprintf(pname, "RC%d_MIN", i + 1);
		param_get(param_find(pname), &fval);
		regs[PX4IO_P_RC_CONFIG_MIN] = fval;
		sprintf(pname, "RC%d_TRIM", i + 1);
		param_get(param_find(pname), &fval);
		regs[PX4IO_P_RC_CONFIG_CENTER] = fval;
		sprintf(pname, "RC%d_MAX", i + 1);
		param_get(param_find(pname), &fval);
		regs[PX4IO_P_RC_CONFIG_MAX] = fval;
		sprintf(pname, "RC%d_DZ", i + 1);
		param_get(param_find(pname), &fval);
		regs[PX4IO_P_RC_CONFIG_DEADZONE] = fval;
		regs[PX4IO_P_RC_CONFIG_ASSIGNMENT] = input_map[i];
		regs[PX4IO_P_RC_CONFIG_OPTIONS] = PX4IO_P_RC_CONFIG_OPTIONS_ENABLED;
		sprintf(pname, "RC%d_REV", i + 1);
		param_get(param_find(pname), &fval);

		/*
		* This has been taken for the sake of compatibility
		* with APM's setup / mission planner: normal: 1,
		* inverted: -1
		*/
		if (fval < 0) {
			regs[PX4IO_P_RC_CONFIG_OPTIONS] |= PX4IO_P_RC_CONFIG_OPTIONS_REVERSE;
		}

		/* send channel config to IO */
		memcpy(&r_page_rc_input_config[offset], &regs, sizeof(uint16_t) * PX4IO_P_RC_CONFIG_STRIDE);
		offset += PX4IO_P_RC_CONFIG_STRIDE;
	}

	return ret;
}

int
AeroCoreIO::io_get_raw_rc_input(rc_input_values &input_rc)
{
	uint32_t channel_count;
	int	ret = 0;

	/* we don't have the status bits, so input_source has to be set elsewhere */
	input_rc.input_source = RC_INPUT_SOURCE_UNKNOWN;

	static const unsigned prolog = (PX4IO_P_RAW_RC_BASE - PX4IO_P_RAW_RC_COUNT);
	uint16_t regs[RC_INPUT_MAX_CHANNELS + prolog];

	/*
	 * Read the channel count and the first 9 channels.
	 *
	 * This should be the common case (9 channel R/C control being a reasonable upper bound).
	 */
	input_rc.timestamp_publication = hrt_absolute_time();

	memcpy(&regs[0], &r_page_raw_rc_input[PX4IO_P_RAW_RC_COUNT],
	       2 * (prolog + 9)); // get the count, start there, get the all the values inbetween BASE and COUNT and then 9 more

	if (ret != OK) {
		return ret;
	}

	/*
	 * Get the channel count any any extra channels. This is no more expensive than reading the
	 * channel count once.
	 */
	channel_count = regs[PX4IO_P_RAW_RC_COUNT];

	if (channel_count != _rc_chan_count) {
		perf_count(_perf_chan_count);
	}

	_rc_chan_count = channel_count;

	input_rc.rc_ppm_frame_length = regs[PX4IO_P_RAW_RC_DATA];
	input_rc.rssi = regs[PX4IO_P_RAW_RC_NRSSI];
	input_rc.rc_failsafe = (regs[PX4IO_P_RAW_RC_FLAGS] & PX4IO_P_RAW_RC_FLAGS_FAILSAFE);
	input_rc.rc_lost_frame_count = regs[PX4IO_P_RAW_LOST_FRAME_COUNT];
	input_rc.rc_total_frame_count = regs[PX4IO_P_RAW_FRAME_COUNT];

	/* rc_lost has to be set before the call to this function */
	if (!input_rc.rc_lost && !input_rc.rc_failsafe) {
		_rc_last_valid = input_rc.timestamp_publication;
	}

	input_rc.timestamp_last_signal = _rc_last_valid;

	if (channel_count > 9) {
		memcpy(&regs[prolog + 9], &r_page_raw_rc_input[PX4IO_P_RAW_RC_BASE + 9], 2 * (channel_count - 9));

		if (ret != OK) {
			return ret;
		}
	}

	input_rc.channel_count = channel_count;
	memcpy(input_rc.values, &regs[prolog], channel_count * 2);

	return ret;
}

int
AeroCoreIO::io_publish_raw_rc()
{

	/* fetch values from IO */
	rc_input_values	rc_val;

	uint16_t status = r_status_flags;

	/* set the RC status flag ORDER MATTERS! */
	rc_val.rc_lost = !(status & PX4IO_P_STATUS_FLAGS_RC_OK);

	int ret = io_get_raw_rc_input(rc_val);

	if (ret != OK) {
		return ret;
	}

	/* sort out the source of the values */
	if (status & PX4IO_P_STATUS_FLAGS_RC_PPM) {
		rc_val.input_source = RC_INPUT_SOURCE_PX4IO_PPM;

	} else if (status & PX4IO_P_STATUS_FLAGS_RC_DSM) {
		rc_val.input_source = RC_INPUT_SOURCE_PX4IO_SPEKTRUM;

	} else if (status & PX4IO_P_STATUS_FLAGS_RC_SBUS) {
		rc_val.input_source = RC_INPUT_SOURCE_PX4IO_SBUS;

	} else {
		rc_val.input_source = RC_INPUT_SOURCE_UNKNOWN;

		/* only keep publishing RC input if we ever got a valid input */
		if (_rc_last_valid == 0) {
			/* we have never seen valid RC signals, abort */
			return OK;
		}
	}

	/* lazily advertise on first publication */
	if (_to_input_rc <= 0) {
		_to_input_rc = orb_advertise(ORB_ID(input_rc), &rc_val);

	} else {
		orb_publish(ORB_ID(input_rc), _to_input_rc, &rc_val);
	}

	return OK;
}

ssize_t
AeroCoreIO::read(struct file *filp, char *buffer, size_t buflen)
{
	int ret = 0;
	return ret;
}

int
AeroCoreIO::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop_cycle();
				_measure_ticks = 0;
				return OK;

			/* external signalling not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(AEROCOREIO_RC_CONVERSION_INTERVAL);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start_cycle();
					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					unsigned ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(AEROCOREIO_RC_CONVERSION_INTERVAL)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start_cycle();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return (1000 / _measure_ticks);

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = irqsave();

			if (!_reports->resize(arg)) {
				irqrestore(flags);
				return -ENOMEM;
			}

			irqrestore(flags);
			return OK;
		}

	case SENSORIOCGQUEUEDEPTH:
		return _reports->size();

	case SENSORIOCRESET:
		/*
		 * Since we are initialized, we do not need to do anything, since the
		 * PROM is correctly read and the part does not need to be configured.
		 */
		return OK;

	default:
		break;
	}

	/* give it to the bus-specific superclass */
	return CDev::ioctl(filp, cmd, arg);
}

void
AeroCoreIO::start_cycle()
{

	/* reset the report ring and state machine */
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&AeroCoreIO::cycle_trampoline, this, 1);

}

void
AeroCoreIO::stop_cycle()
{
	work_cancel(HPWORK, &_work);
}

void
AeroCoreIO::cycle_trampoline(void *arg)
{
	AeroCoreIO *dev = reinterpret_cast<AeroCoreIO *>(arg);
	dev->cycle();
}

void
AeroCoreIO::cycle()
{
	// get spektrum data
	controls_tick();

	// get the data from registers
	io_publish_raw_rc();

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&AeroCoreIO::cycle_trampoline,
		   this,
		   USEC2TICK(AEROCOREIO_RC_CONVERSION_INTERVAL));
}

void
AeroCoreIO::print_info()
{
	printf("Spektrum receiver for AeroCore\n");
}

extern "C" __EXPORT int aerocore_rc_main(int argc, char *argv[]);

/**
 * Local functions in support of the shell command.
 */
namespace aerocore_rc
{

AeroCoreIO	*g_dev;

void	start();
void	test();
void	reset();
void	info();
//void	calibrate(unsigned altitude);

/**
 * Start the driver.
 */
void
start()
{
	int fd;

	if (g_dev != nullptr)
		/* if already started, the still command succeeded */
	{
		errx(0, "already started");
	}

	g_dev = new AeroCoreIO();

	if (g_dev == nullptr) {
		//delete interface;
		errx(1, "failed to allocate driver");
	}

	if (g_dev->init() != OK) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(AEROCOREIO_RC_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		warnx("can't open baro device");
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		warnx("failed setting default poll rate");
		goto fail;
	}

	exit(0);

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
	int fd = open(AEROCOREIO_RC_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'aerocore_io start' if the driver is not running)", AEROCOREIO_RC_DEVICE_PATH);
	}

	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(AEROCOREIO_RC_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
	}

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr) {
		errx(1, "driver not running");
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

} // namespace

int
aerocore_rc_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		//printf("Starting AeroCore Spektrum receiver ... \n");
		aerocore_rc::start();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test")) {
		aerocore_rc::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset")) {
		aerocore_rc::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info")) {
		aerocore_rc::info();
	}

	errx(1, "unrecognised command, try 'start', 'test', 'reset' or 'info'");
}
