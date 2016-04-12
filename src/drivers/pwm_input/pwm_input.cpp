/****************************************************************************
 *
 *   Copyright (c) 2015 Andrew Tridgell. All rights reserved.
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
 * @file pwm_input.cpp
 *
 * PWM input driver based on earlier driver from Evan Slatyer,
 * which in turn was based on drv_hrt.c
 *
 * @author: Andrew Tridgell
 * @author: Ban Siesta <bansiesta@gmail.com>
 */

#include <px4_config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdbool.h>

#include <assert.h>
#include <debug.h>
#include <time.h>
#include <queue.h>
#include <errno.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <board_config.h>
#include <drivers/drv_pwm_input.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>

#include "chip.h"
#include "up_internal.h"
#include "up_arch.h"

#include "stm32.h"
#include "stm32_gpio.h"
#include "stm32_tim.h"
#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/pwm_input.h>
#include <uORB/topics/subsystem_info.h>

#include <drivers/drv_device.h>
#include <drivers/device/device.h>
#include <drivers/device/ringbuffer.h>

#include <systemlib/perf_counter.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#if HRT_TIMER == PWMIN_TIMER
#error cannot share timer between HRT and PWMIN
#endif

#if !defined(GPIO_PWM_IN) || !defined(PWMIN_TIMER) || !defined(PWMIN_TIMER_CHANNEL)
#error PWMIN defines are needed in board_config.h for this board
#endif

/* PWMIN configuration */
#if   PWMIN_TIMER == 1
# define PWMIN_TIMER_BASE	STM32_TIM1_BASE
# define PWMIN_TIMER_POWER_REG	STM32_RCC_APB2ENR
# define PWMIN_TIMER_POWER_BIT	RCC_APB2ENR_TIM1EN
# define PWMIN_TIMER_VECTOR	STM32_IRQ_TIM1CC
# define PWMIN_TIMER_CLOCK	STM32_APB2_TIM1_CLKIN
#elif PWMIN_TIMER == 2
# define PWMIN_TIMER_BASE	STM32_TIM2_BASE
# define PWMIN_TIMER_POWER_REG	STM32_RCC_APB1ENR
# define PWMIN_TIMER_POWER_BIT	RCC_APB1ENR_TIM2EN
# define PWMIN_TIMER_VECTOR	STM32_IRQ_TIM2
# define PWMIN_TIMER_CLOCK	STM32_APB1_TIM2_CLKIN
#elif PWMIN_TIMER == 3
# define PWMIN_TIMER_BASE	STM32_TIM3_BASE
# define PWMIN_TIMER_POWER_REG	STM32_RCC_APB1ENR
# define PWMIN_TIMER_POWER_BIT	RCC_APB1ENR_TIM3EN
# define PWMIN_TIMER_VECTOR	STM32_IRQ_TIM3
# define PWMIN_TIMER_CLOCK	STM32_APB1_TIM3_CLKIN
#elif PWMIN_TIMER == 4
# define PWMIN_TIMER_BASE	STM32_TIM4_BASE
# define PWMIN_TIMER_POWER_REG	STM32_RCC_APB1ENR
# define PWMIN_TIMER_POWER_BIT	RCC_APB1ENR_TIM4EN
# define PWMIN_TIMER_VECTOR	STM32_IRQ_TIM4
# define PWMIN_TIMER_CLOCK	STM32_APB1_TIM4_CLKIN
#elif PWMIN_TIMER == 5
# define PWMIN_TIMER_BASE	STM32_TIM5_BASE
# define PWMIN_TIMER_POWER_REG	STM32_RCC_APB1ENR
# define PWMIN_TIMER_POWER_BIT	RCC_APB1ENR_TIM5EN
# define PWMIN_TIMER_VECTOR	STM32_IRQ_TIM5
# define PWMIN_TIMER_CLOCK	STM32_APB1_TIM5_CLKIN
#elif PWMIN_TIMER == 8
# define PWMIN_TIMER_BASE	STM32_TIM8_BASE
# define PWMIN_TIMER_POWER_REG	STM32_RCC_APB2ENR
# define PWMIN_TIMER_POWER_BIT	RCC_APB2ENR_TIM8EN
# define PWMIN_TIMER_VECTOR	STM32_IRQ_TIM8CC
# define PWMIN_TIMER_CLOCK	STM32_APB2_TIM8_CLKIN
#elif PWMIN_TIMER == 9
# define PWMIN_TIMER_BASE	STM32_TIM9_BASE
# define PWMIN_TIMER_POWER_REG	STM32_RCC_APB2ENR
# define PWMIN_TIMER_POWER_BIT	RCC_APB2ENR_TIM9EN
# define PWMIN_TIMER_VECTOR	STM32_IRQ_TIM1BRK
# define PWMIN_TIMER_CLOCK	STM32_APB2_TIM9_CLKIN
#elif PWMIN_TIMER == 10
# define PWMIN_TIMER_BASE	STM32_TIM10_BASE
# define PWMIN_TIMER_POWER_REG	STM32_RCC_APB2ENR
# define PWMIN_TIMER_POWER_BIT	RCC_APB2ENR_TIM10EN
# define PWMIN_TIMER_VECTOR	STM32_IRQ_TIM1UP
# define PWMIN_TIMER_CLOCK	STM32_APB2_TIM10_CLKIN
#elif PWMIN_TIMER == 11
# define PWMIN_TIMER_BASE	STM32_TIM11_BASE
# define PWMIN_TIMER_POWER_REG	STM32_RCC_APB2ENR
# define PWMIN_TIMER_POWER_BIT	RCC_APB2ENR_TIM11EN
# define PWMIN_TIMER_VECTOR	STM32_IRQ_TIM1TRGCOM
# define PWMIN_TIMER_CLOCK	STM32_APB2_TIM11_CLKIN
#elif PWMIN_TIMER == 12
# define PWMIN_TIMER_BASE	STM32_TIM12_BASE
# define PWMIN_TIMER_POWER_REG	STM32_RCC_APB1ENR
# define PWMIN_TIMER_POWER_BIT	RCC_APB1ENR_TIM12EN
# define PWMIN_TIMER_VECTOR	STM32_IRQ_TIM8BRK
# define PWMIN_TIMER_CLOCK	STM32_APB1_TIM12_CLKIN
#else
# error PWMIN_TIMER must be a value between 1 and 12
#endif

/*
 * HRT clock must be at least 1MHz
 */
#if PWMIN_TIMER_CLOCK <= 1000000
# error PWMIN_TIMER_CLOCK must be greater than 1MHz
#endif

/*
 * Timer register accessors
 */
#define REG(_reg)	(*(volatile uint32_t *)(PWMIN_TIMER_BASE + _reg))

#define rCR1		REG(STM32_GTIM_CR1_OFFSET)
#define rCR2		REG(STM32_GTIM_CR2_OFFSET)
#define rSMCR		REG(STM32_GTIM_SMCR_OFFSET)
#define rDIER		REG(STM32_GTIM_DIER_OFFSET)
#define rSR		REG(STM32_GTIM_SR_OFFSET)
#define rEGR		REG(STM32_GTIM_EGR_OFFSET)
#define rCCMR1		REG(STM32_GTIM_CCMR1_OFFSET)
#define rCCMR2		REG(STM32_GTIM_CCMR2_OFFSET)
#define rCCER		REG(STM32_GTIM_CCER_OFFSET)
#define rCNT		REG(STM32_GTIM_CNT_OFFSET)
#define rPSC		REG(STM32_GTIM_PSC_OFFSET)
#define rARR		REG(STM32_GTIM_ARR_OFFSET)
#define rCCR1		REG(STM32_GTIM_CCR1_OFFSET)
#define rCCR2		REG(STM32_GTIM_CCR2_OFFSET)
#define rCCR3		REG(STM32_GTIM_CCR3_OFFSET)
#define rCCR4		REG(STM32_GTIM_CCR4_OFFSET)
#define rDCR		REG(STM32_GTIM_DCR_OFFSET)
#define rDMAR		REG(STM32_GTIM_DMAR_OFFSET)

/*
 * Specific registers and bits used by HRT sub-functions
 */
#if PWMIN_TIMER_CHANNEL == 1
#define rCCR_PWMIN_A		rCCR1			/* compare register for PWMIN */
#define DIER_PWMIN_A		(GTIM_DIER_CC1IE) 	/* interrupt enable for PWMIN */
#define SR_INT_PWMIN_A		GTIM_SR_CC1IF		/* interrupt status for PWMIN */
#define rCCR_PWMIN_B		rCCR2 			/* compare register for PWMIN */
#define SR_INT_PWMIN_B		GTIM_SR_CC2IF		/* interrupt status for PWMIN */
#define CCMR1_PWMIN		((0x02 << GTIM_CCMR1_CC2S_SHIFT) | (0x01 << GTIM_CCMR1_CC1S_SHIFT))
#define CCMR2_PWMIN		0
#define CCER_PWMIN		(GTIM_CCER_CC2P | GTIM_CCER_CC1E | GTIM_CCER_CC2E)
#define SR_OVF_PWMIN		(GTIM_SR_CC1OF | GTIM_SR_CC2OF)
#define SMCR_PWMIN_1		(0x05 << GTIM_SMCR_TS_SHIFT)
#define SMCR_PWMIN_2		((0x04 << GTIM_SMCR_SMS_SHIFT) | SMCR_PWMIN_1)
#elif PWMIN_TIMER_CHANNEL == 2
#define rCCR_PWMIN_A		rCCR2			/* compare register for PWMIN */
#define DIER_PWMIN_A		(GTIM_DIER_CC2IE)	/* interrupt enable for PWMIN */
#define SR_INT_PWMIN_A		GTIM_SR_CC2IF		/* interrupt status for PWMIN */
#define rCCR_PWMIN_B		rCCR1			/* compare register for PWMIN */
#define DIER_PWMIN_B		GTIM_DIER_CC1IE		/* interrupt enable for PWMIN */
#define SR_INT_PWMIN_B		GTIM_SR_CC1IF		/* interrupt status for PWMIN */
#define CCMR1_PWMIN		((0x01 << GTIM_CCMR1_CC2S_SHIFT) | (0x02 << GTIM_CCMR1_CC1S_SHIFT))
#define CCMR2_PWMIN		0
#define CCER_PWMIN		(GTIM_CCER_CC1P | GTIM_CCER_CC1E | GTIM_CCER_CC2E)
#define SR_OVF_PWMIN		(GTIM_SR_CC1OF | GTIM_SR_CC2OF)
#define SMCR_PWMIN_1		(0x06 << GTIM_SMCR_TS_SHIFT)
#define SMCR_PWMIN_2		((0x04 << GTIM_SMCR_SMS_SHIFT) | SMCR_PWMIN_1)
#else
#error PWMIN_TIMER_CHANNEL must be either 1 and 2.
#endif

#define TIMEOUT_POLL 300000 /* reset after no response over this time in microseconds [0.3s] */
#define TIMEOUT_READ 200000 /* don't reset if the last read is back more than this time in microseconds [0.2s] */

class PWMIN : device::CDev
{
public:
	PWMIN();
	virtual ~PWMIN();

	virtual int init();
	virtual int open(struct file *filp);
	virtual ssize_t read(struct file *filp, char *buffer, size_t buflen);
	virtual int ioctl(struct file *filp, int cmd, unsigned long arg);

	void publish(uint16_t status, uint32_t period, uint32_t pulse_width);
	void print_info(void);

private:
	uint32_t _error_count;
	uint32_t _pulses_captured;
	uint32_t _last_period;
	uint32_t _last_width;
	hrt_abstime _last_poll_time;
	hrt_abstime _last_read_time;
	ringbuffer::RingBuffer *_reports;
	bool _timer_started;

	perf_counter_t _perf_reset;
	perf_counter_t _perf_interrupt;
	perf_counter_t _perf_read;

	void _timer_init(void);
};

static int pwmin_tim_isr(int irq, void *context);
static void pwmin_start();
static void pwmin_info(void);
static void pwmin_test(void);
static void pwmin_reset(void);

static PWMIN *g_dev;

PWMIN::PWMIN() :
	CDev("pwmin", PWMIN0_DEVICE_PATH),
	_error_count(0),
	_pulses_captured(0),
	_last_period(0),
	_last_width(0),
	_reports(nullptr),
	_timer_started(false),
	_perf_reset(perf_alloc(PC_COUNT, "pwm_input_reset")),
	_perf_read(perf_alloc(PC_ELAPSED, "pwm_input_read")),
	_perf_interrupt(perf_alloc(PC_ELAPSED, "pwm_input_interrupt"))
{
}

PWMIN::~PWMIN()
{
	if (_reports != nullptr) {
		delete _reports;
	}
	perf_free(_perf_reset);
	perf_free(_perf_read);
	perf_free(_perf_interrupt);
}

/*
 * initialise the driver. This doesn't actually start the timer (that
 * is done on open). We don't start the timer to allow for this driver
 * to be started in init scripts when the user may be using the input
 * pin as PWM output
 */
int
PWMIN::init()
{
	/* we just register the device in /dev, and only actually
	 * activate the timer when requested to when the device is opened */
	CDev::init();

	_reports = new ringbuffer::RingBuffer(2, sizeof(struct pwm_input_s));
	if (_reports == nullptr) {
		return -ENOMEM;
	}

	return OK;
}

/*
 * Initialise the timer we are going to use.
 */
void PWMIN::_timer_init(void)
{
	/* run with interrupts disabled in case the timer is already
	 * setup. We don't want it firing while we are doing the setup */
	irqstate_t flags = irqsave();
	stm32_configgpio(GPIO_PWM_IN);

	/* claim our interrupt vector */
	irq_attach(PWMIN_TIMER_VECTOR, pwmin_tim_isr);

	/* Clear no bits, set timer enable bit.*/
	modifyreg32(PWMIN_TIMER_POWER_REG, 0, PWMIN_TIMER_POWER_BIT);

	/* disable and configure the timer */
	rCR1 = 0;
	rCR2 = 0;
	rSMCR = 0;
	rDIER = DIER_PWMIN_A;
	rCCER = 0;		/* unlock CCMR* registers */
	rCCMR1 = CCMR1_PWMIN;
	rCCMR2 = CCMR2_PWMIN;
	rSMCR = SMCR_PWMIN_1;	/* Set up mode */
	rSMCR = SMCR_PWMIN_2;	/* Enable slave mode controller */
	rCCER = CCER_PWMIN;
	rDCR = 0;

	/* for simplicity scale by the clock in MHz. This gives us
	 * readings in microseconds which is typically what is needed
	 * for a PWM input driver */
	uint32_t prescaler = PWMIN_TIMER_CLOCK / 1000000UL;

	/*
	 * define the clock speed. We want the highest possible clock
	 * speed that avoids overflows.
	 */
	rPSC = prescaler - 1;

	/* run the full span of the counter. All timers can handle
	 * uint16 */
	rARR = UINT16_MAX;

	/* generate an update event; reloads the counter, all registers */
	rEGR = GTIM_EGR_UG;

	/* enable the timer */
	rCR1 = GTIM_CR1_CEN;

	/* enable interrupts */
	up_enable_irq(PWMIN_TIMER_VECTOR);

	irqrestore(flags);

	_timer_started = true;

	perf_count(_perf_reset);
}

/*
 * hook for open of the driver. We start the timer at this point, then
 * leave it running
 */
int
PWMIN::open(struct file *filp)
{
	if (g_dev == nullptr) {
		return -EIO;
	}

	int ret = CDev::open(filp);

	if (ret == OK && !_timer_started) {
		g_dev->_timer_init();
	}

	return ret;
}


/*
 * handle ioctl requests
 */
int
PWMIN::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 500)) {
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
		/* user has asked for the timer to be reset. This may
		 * be needed if the pin was used for a different
		 * purpose (such as PWM output) */
		_timer_init();
		return OK;

	default:
		/* give it to the superclass */
		return CDev::ioctl(filp, cmd, arg);
	}
}


/*
 * read some samples from the device
 */
ssize_t
PWMIN::read(struct file *filp, char *buffer, size_t buflen)
{
	perf_begin(_perf_read);

	_last_read_time = hrt_absolute_time();

	unsigned count = buflen / sizeof(struct pwm_input_s);
	struct pwm_input_s *buf = reinterpret_cast<struct pwm_input_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		perf_end(_perf_read);
		return -ENOSPC;
	}

	while (count--) {
		if (_reports->get(buf)) {
			ret += sizeof(struct pwm_input_s);
			buf++;
		}
	}

	perf_end(_perf_read);

	/* if there was no data, warn the caller */
	return ret ? ret : -EAGAIN;
}

/*
 * publish some data from the ISR in the ring buffer
 */
void PWMIN::publish(uint16_t status, uint32_t period, uint32_t pulse_width)
{
	perf_count(_perf_interrupt);

	/* if we missed an edge, we have to give up */
	if (status & SR_OVF_PWMIN) {
		_error_count++;
		return;
	}

	_last_poll_time = hrt_absolute_time();

	struct pwm_input_s pwmin_report;
	pwmin_report.timestamp = _last_poll_time;
	pwmin_report.error_count = _error_count;
	pwmin_report.period = period;
	pwmin_report.pulse_width = pulse_width;

	_reports->force(&pwmin_report);
}

/*
 * print information on the last captured
 */
void PWMIN::print_info(void)
{
	if (!_timer_started) {
		printf("timer not started - try the 'test' command\n");

	} else {
		printf("count=%u period=%u width=%u\n",
		       (unsigned)_pulses_captured,
		       (unsigned)_last_period,
		       (unsigned)_last_width);
		perf_print_counter(_perf_interrupt);
		perf_print_counter(_perf_read);
		perf_print_counter(_perf_reset);
	}
}


/*
 * Handle the interrupt, gathering pulse data
 */
static int pwmin_tim_isr(int irq, void *context)
{
	uint16_t status = rSR;
	uint32_t period = rCCR_PWMIN_A;
	uint32_t pulse_width = rCCR_PWMIN_B;

	/* ack the interrupts we just read */
	rSR = 0;

	if (g_dev != nullptr) {
		g_dev->publish(status, period, pulse_width);
	}

	return OK;
}

/*
 * start the driver
 */
static void pwmin_start()
{
	if (g_dev != nullptr) {
		errx(1, "driver already started");
	}

	g_dev = new PWMIN();

	if (g_dev == nullptr) {
		errx(1, "driver allocation failed");
	}

	if (g_dev->init() != OK) {
		errx(1, "driver init failed");
	}

	exit(0);
}

/*
 * test the driver
 */
static void pwmin_test(void)
{
	int fd = open(PWMIN0_DEVICE_PATH, O_RDONLY);

	if (fd == -1) {
		errx(1, "Failed to open device");
	}

	uint64_t start_time = hrt_absolute_time();

	printf("Showing samples for 5 seconds\n");

	while (hrt_absolute_time() < start_time + 5U * 1000UL * 1000UL) {
		struct pwm_input_s buf;

		if (::read(fd, &buf, sizeof(buf)) == sizeof(buf)) {
			printf("period=%u width=%u error_count=%u\n",
			       (unsigned)buf.period,
			       (unsigned)buf.pulse_width,
			       (unsigned)buf.error_count);

		} else {
			/* no data, retry in 2 ms */
			::usleep(2000);
		}
	}

	close(fd);
	exit(0);
}

/*
 * reset the timer
 */
static void pwmin_reset(void)
{
	int fd = open(PWMIN0_DEVICE_PATH, O_RDONLY);

	if (fd == -1) {
		errx(1, "Failed to open device");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) != OK) {
		errx(1, "reset failed");
	}

	close(fd);
	exit(0);
}

/*
 * show some information on the driver
 */
static void pwmin_info(void)
{
	if (g_dev == nullptr) {
		printf("driver not started\n");
		exit(1);
	}

	g_dev->print_info();
	exit(0);
}


/*
 * driver entry point
 */
int pwm_input_main(int argc, char *argv[])
{
	const char *verb = argv[1];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		pwmin_start();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		pwmin_info();
	}

	/*
	 * print test results
	 */
	if (!strcmp(verb, "test")) {
		pwmin_test();
	}

	/*
	 * reset the timer
	 */
	if (!strcmp(verb, "reset")) {
		pwmin_reset();
	}

	errx(1, "unrecognized command, try 'start', 'info', 'reset' or 'test'");
	return 0;
}
