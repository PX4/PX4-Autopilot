/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
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
 * @file input_pwm.cpp
 *
 * Driver for PWM RC inputs.  Creates a device for each timer defined in board specific file.
 * Channel number is defined by the order of the channels listed in board specific file (index).
 */

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
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

#include <board_config.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/device/device.h>
#include <drivers/drv_sensor.h>
#include <drivers/drv_input_pwm.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/stm32/drv_input_pwm_channels.h>

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>

#include <float.h>
#include <stm32.h>
#include <stm32_gpio.h>
#include <stm32_tim.h>

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

/* PWM signals are 20ms long, so max rate should be 50Hz */
#define INPUT_PWM_INTERVAL		20000 /* microseconds */

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

/** PWM decoder state machine */
struct pwmState {
        uint16_t        last_edge;      /**< last capture time */
        enum {
                UNSYNCH = 0,
                ARM,
                ACTIVE,
                INACTIVE
        } phase;
};


class InputPWM : public device::CDev
{
public:
	InputPWM(uint8_t timer_index, const char *devName);
	virtual ~InputPWM();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

	/* static member to hold pwm values for channels from all timers */
	/* written by everyone but read/reported by instance 0 */
	static volatile uint16_t	input_pwm_channel_values[INPUT_PWM_MAX_CHANNELS];
	/* bool flags to indicate if the channel is active (decoded) */
	static volatile bool 		input_pwm_channel_decoded[INPUT_PWM_MAX_CHANNELS];

private:
	work_s			_work;
	unsigned		_measure_ticks;

	RingBuffer		*_reports;

	int			_temp_variable;

	orb_advert_t		_input_rc_topic;

	perf_counter_t		_sample_perf;
	perf_counter_t		_buffer_overflows;

	uint8_t			_timer_index;				/* index of timer in board config file */
	uint8_t			_num_channels;				/* number of channels tied to this timer */
	uint8_t			_global_channel_index[INPUT_PWM_MAX_CHANNELS_PER_TIMER]; /* index of where this timer's channels lie globally */

	volatile uint16_t	_temp_rc_buffer[INPUT_PWM_MAX_CHANNELS_PER_TIMER]; 			/* stores PWM values as they're read in */
//	volatile uint8_t	_decoded_channels;			/* actual number of input channels seen */

	pwmState		_pwmChannels[INPUT_PWM_MAX_CHANNELS_PER_TIMER];			/* holds pwm decoder state */
									/* since each instance connects to a timer */
									/* max channels is 4 */

	/**
	 * Initialise the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void			start();

	/**
	 * Stop the automatic measurement state machine.
	 */
	void			stop();

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

	void 			interrupt(void *ctx);

	void			update_pwm_state(uint8_t timer_channel, uint16_t count);

};

/* need to define our static pointer */
volatile uint16_t	InputPWM::input_pwm_channel_values[INPUT_PWM_MAX_CHANNELS];
/* bool flags to indicate if the channel is active (decoded) */
volatile bool 		InputPWM::input_pwm_channel_decoded[INPUT_PWM_MAX_CHANNELS];

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int input_pwm_main(int argc, char *argv[]);

InputPWM::InputPWM(uint8_t timer_index, const char* devName) :
	CDev("InputPWM", devName, input_pwm_timers[timer_index].irq_vector),
	_measure_ticks(0),
	_reports(nullptr),
	_input_rc_topic(-1),
	_temp_variable(2345),
	_sample_perf(perf_alloc(PC_ELAPSED, "input_pwm_read")),
	_buffer_overflows(perf_alloc(PC_COUNT, "input_pwm_buffer_overflows")),
	_timer_index(timer_index)
{
	_debug_enabled = false;

	_pwmChannels[0].phase = pwmState::ARM;
	_pwmChannels[1].phase = pwmState::ARM;
	_pwmChannels[2].phase = pwmState::ARM;
	_pwmChannels[3].phase = pwmState::ARM;

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

InputPWM::~InputPWM()
{
	/* make sure we are truly inactive */
	stop();

	if (_reports != nullptr)
		delete _reports;

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_buffer_overflows);
}

void
InputPWM::update_pwm_state(uint8_t timer_channel, uint16_t count)
{
        uint16_t width;

        /* how long since the last edge? - this handles counter wrapping implicitely. */
        width = count - _pwmChannels[timer_channel-1].last_edge;

	switch (_pwmChannels[timer_channel-1].phase) {

        case pwmState::ARM:

                /* frame length is everything including the start gap */
		_pwmChannels[timer_channel-1].phase = pwmState::ACTIVE;
                break;

        case pwmState::ACTIVE:
                /* if the mark-mark timing is out of bounds, abandon the frame */
                if ((width < INPUT_PWM_MIN) || (width > INPUT_PWM_MAX))
		{
			/* missed an edge, stay ACTIVE and we'll catch it on the next go */
		}
		else
		{
                /* if we have room to store the value, do so */
			/* store in local buffer -- used in read command */
                        _temp_rc_buffer[timer_channel-1] = width;
			/* store in global buffer */
			input_pwm_channel_values[_global_channel_index[timer_channel-1]] = width;
			/* set decoded flag */
			input_pwm_channel_decoded[_global_channel_index[timer_channel-1]] = true;
	                _pwmChannels[timer_channel-1].phase = pwmState::ARM;

		}
                break;

        }

        _pwmChannels[timer_channel-1].last_edge = count;

        return;
}

int
InputPWM::init()
{
	int ret = ERROR;

	ret = CDev::init();
	if (ret != OK) {
		debug("CDev init failed");
		return ERROR;
	}

	/* allocate basic report buffers */
	_reports = new RingBuffer(2, sizeof(rc_input_values));
	if (_reports == nullptr)
		return ERROR;

	struct rc_input_values zero_report;
	memset(&zero_report, 0, sizeof(zero_report));
	_input_rc_topic = orb_advertise(ORB_ID(input_rc), &zero_report);

	if (_input_rc_topic < 0)
		debug("failed to create input_rc object");

	int channel_count = 0;
	/* loop through list of all channels and get this timer's channels */
	for (int i = 0;i<INPUT_PWM_MAX_CHANNELS;i++) {
		if ((input_pwm_channels[i].timer_index == _timer_index) && (input_pwm_channels[i].timer_channel != 0)) {
			/* too many channels on a timer? */
			if ((channel_count < INPUT_PWM_MAX_CHANNELS_PER_TIMER) && (input_pwm_channels[i].timer_channel-1 < INPUT_PWM_MAX_CHANNELS_PER_TIMER)) {
				_global_channel_index[input_pwm_channels[i].timer_channel-1] = i;
				channel_count++;
			}
			else {
				// too many channels
				warnx("Too many channels listed in board input_pwm config file");
			}
		}
	}
	_num_channels = channel_count;

	up_input_pwm_timer_init(_timer_index);

	this->interrupt_enable();

	ret = OK;
}

ssize_t
InputPWM::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct rc_input_values);
	struct rc_input_values *buf = reinterpret_cast<struct rc_input_values *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1)
		return -ENOSPC;

	/* time stamp it */
	buf->timestamp_publication = hrt_absolute_time();

	/* record the actual number of received channels */
	buf->channel_count = _num_channels;

	/* no way to receive signal strength, set to 100% = 255 */
	buf->rssi = 255;

	/* set the input source to unknown */
	buf->input_source = RC_INPUT_SOURCE_UNKNOWN;

	/* loop through the temp buffer and fill the report values */
	for (int i=0;i<_num_channels;i++)
		buf->values[i] = _temp_rc_buffer[i];

	return sizeof(struct rc_input_values);
}

int
InputPWM::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case SENSORIOCSPOLLRATE: {
		switch (arg) {

			/* switching to manual polling */
		case SENSOR_POLLRATE_MANUAL:
			stop();
			_measure_ticks = 0;
			return OK;

			/* external signalling (DRDY) not supported */
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
				_measure_ticks = USEC2TICK(INPUT_PWM_INTERVAL);

				/* if we need to start the poll state machine, do it */
				if (want_start)
					start();

				return OK;
			}

			/* adjust to a legal polling interval in Hz */
		default: {
				/* do we need to start internal polling? */
				bool want_start = (_measure_ticks == 0);

				/* convert hz to tick interval via microseconds */
				unsigned ticks = USEC2TICK(1000000 / arg);

				/* check against maximum rate */
				if (ticks < USEC2TICK(INPUT_PWM_INTERVAL))
					return -EINVAL;

				/* update interval for next measurement */
				_measure_ticks = ticks;

				/* if we need to start the poll state machine, do it */
				if (want_start)
					start();

				return OK;
			}
		}
	}

	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0)
			return SENSOR_POLLRATE_MANUAL;

		return 1000000/TICK2USEC(_measure_ticks);

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100))
				return -EINVAL;

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
		return OK;

	case RC_INPUT_GET: {
		/* fetch R/C input values into (rc_input_values *)arg */
			struct rc_input_values *report = (rc_input_values *)arg;
			int ret;
			ret = read(0, (char *)report, sizeof(*report));
			if (ret > 0)
				return OK;
			else
				return ret;
		}

	default:
		/* give it to the superclass */
		return CDev::ioctl(filp, cmd, arg);
	}
}

void
InputPWM::interrupt(void *ctx)
{
	uint8_t timer_channel;
	uint16_t value[_num_channels];
	up_input_pwm_timer_isr(_timer_index, &timer_channel, value);

	/* parse the channel flags */
	for (int i=0;i<_num_channels;i++)
		if (timer_channel & (1 << i))
			update_pwm_state(i+1, value[i]);
}

void
InputPWM::start()
{
	/* reset the report ring and state machine */
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&InputPWM::cycle_trampoline, this, 1);
}

void
InputPWM::stop()
{
	work_cancel(HPWORK, &_work);
}

void
InputPWM::cycle_trampoline(void *arg)
{
	InputPWM *dev = (InputPWM *)arg;
	dev->cycle();
}

void
InputPWM::cycle()
{

	int ret;

	perf_begin(_sample_perf);

	struct rc_input_values report;

	/* copy the current pwm values by reading them in */
	ret = read(0, (char *)&report, sizeof(report));
	if (ret < 0) {
		/* couldn't read data, make empty report */
		report.timestamp_publication = hrt_absolute_time();
		report.channel_count = 0;
		report.rssi = 0;
		report.input_source = RC_INPUT_SOURCE_UNKNOWN;
		if (_reports->force(&report)){
			perf_count(_buffer_overflows);
		}
		perf_end(_sample_perf);
		return;
	}

	if (_reports->force(&report)) {
		perf_count(_buffer_overflows);
	}

	perf_end(_sample_perf);

	// only _timer_index = 0 should publish a report of all rc inputs
	if (_timer_index == 0) {
		int decoded_channels = 0;
		report.timestamp_publication = hrt_absolute_time();
		report.timestamp_last_signal = report.timestamp_publication;
		report.rssi = 255;
		report.rc_failsafe = false;
		report.rc_lost = false;
		report.rc_lost_frame_count = 0;
		report.rc_total_frame_count = 0;
		report.rc_ppm_frame_length = 0;
		report.input_source = RC_INPUT_SOURCE_UNKNOWN;
		/* find out how many are actually active */
		for (int i=0;i < INPUT_PWM_MAX_CHANNELS; i++) {
			if (input_pwm_channel_decoded[i]) {
				report.values[i] = input_pwm_channel_values[i];
				decoded_channels++;
			}
			else
				report.values[i] = 0;
		}
		report.channel_count = decoded_channels;

		/* publish the data */
		orb_publish(ORB_ID(input_rc), _input_rc_topic, &report);
		/* notify anyone waiting for the data */
		poll_notify(POLLIN);
	}

	/* schedule a fresh cycle call */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&InputPWM::cycle_trampoline,
		   this,
		   _measure_ticks);

}

void
InputPWM::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_buffer_overflows);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	_reports->print_info("report queue");

	/* print out the rc channels */
	int channel_count = 0;
	for (int i=0;i<INPUT_PWM_MAX_CHANNELS;i++) {
		if (input_pwm_channel_decoded[i]) {
			printf("	channel[%d]: %d\n", i+1, input_pwm_channel_values[i]);
			channel_count++;
		}
	}
	if (channel_count == 0)
		printf("	No active channels, %d expected for timer[%d]\n", _num_channels, _timer_index);
}

/**
 * Local functions in support of the shell command.
 */
namespace input_pwm
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
const int ERROR = -1;

InputPWM	**g_dev; /* 4 * 3 input channels */
int 		num_timers = 0; /* number of rc_input devices */

void	start();
void	info();

/**
 * Start the driver.
 */
void
start()
{
	int fd;
	int irq;
	char devName[50];

	if (g_dev != nullptr)
		/* if already started, the still command succeeded */
		errx(0, "already started");

	g_dev = new InputPWM*[INPUT_PWM_MAX_TIMERS];

	/* get the number of timers */
	for (int i=0;i<INPUT_PWM_MAX_TIMERS;i++) {
		if (input_pwm_timers[i].base != 0) {


			/* create the driver */
			/* for each bank of rc inputs (timer), create an instance */
			/* XXX The number of timers could be stored in a parameter */
			/* get the irq number */
			sprintf(devName, "%s%d", RC_INPUT_DEVICE_PATH, i);
			g_dev[i] = new InputPWM(i, devName);

			if (g_dev[i] != nullptr && OK != g_dev[i]->init()) {
				delete g_dev[i];
				warnx("Could not start rc_input device [%d]", i);
			}


			/* set the poll rate to default, starts automatic data collection */
			fd = open(devName, O_RDONLY);

			if (fd < 0 ) {
				delete g_dev[i];
				warnx("Could not open rc_input device [%d]", i);
			}
			if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
				delete g_dev[i];
				warnx("Could not set poll rate for rc_input device [%d]", i);
			}

			num_timers++;
		}
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
 * Print a little info about the driver.
 */
void
info()
{
	// just print out index 0 timer
	if (g_dev[0] == nullptr)
		errx(1, "driver not running");

	for (int i=0;i<num_timers;i++) {
		printf("state @ %p\n", g_dev[i]);
		g_dev[i]->print_info();
	}

	exit(0);
}

} // namespace

int
input_pwm_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start"))
		input_pwm::start();

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status"))
		input_pwm::info();

	errx(1, "unrecognized command, try 'start' or 'info'");
}
