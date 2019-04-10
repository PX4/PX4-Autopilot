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
 * @file adc.cpp
 *
 * Driver for the STM32 ADC.
 *
 * This is a low-rate driver, designed for sampling things like voltages
 * and so forth. It avoids the gross complexity of the NuttX ADC driver.
 */

#include <px4_config.h>
#include <board_config.h>
#include <drivers/device/device.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>

#include <arch/board/board.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_adc.h>

#include <stm32_adc.h>
#include <stm32_gpio.h>

#include <systemlib/err.h>
#include <perf/perf_counter.h>

#include <uORB/topics/system_power.h>
#include <uORB/topics/adc_report.h>

#if defined(ADC_CHANNELS)

/*
 * Register accessors.
 * For now, no reason not to just use ADC1.
 */
#define REG(_reg)	(*(volatile uint32_t *)(STM32_ADC1_BASE + _reg))

#define rSR		REG(STM32_ADC_SR_OFFSET)
#define rCR1		REG(STM32_ADC_CR1_OFFSET)
#define rCR2		REG(STM32_ADC_CR2_OFFSET)
#define rSMPR1		REG(STM32_ADC_SMPR1_OFFSET)
#define rSMPR2		REG(STM32_ADC_SMPR2_OFFSET)
#define rJOFR1		REG(STM32_ADC_JOFR1_OFFSET)
#define rJOFR2		REG(STM32_ADC_JOFR2_OFFSET)
#define rJOFR3		REG(STM32_ADC_JOFR3_OFFSET)
#define rJOFR4		REG(STM32_ADC_JOFR4_OFFSET)
#define rHTR		REG(STM32_ADC_HTR_OFFSET)
#define rLTR		REG(STM32_ADC_LTR_OFFSET)
#define rSQR1		REG(STM32_ADC_SQR1_OFFSET)
#define rSQR2		REG(STM32_ADC_SQR2_OFFSET)
#define rSQR3		REG(STM32_ADC_SQR3_OFFSET)
#define rJSQR		REG(STM32_ADC_JSQR_OFFSET)
#define rJDR1		REG(STM32_ADC_JDR1_OFFSET)
#define rJDR2		REG(STM32_ADC_JDR2_OFFSET)
#define rJDR3		REG(STM32_ADC_JDR3_OFFSET)
#define rJDR4		REG(STM32_ADC_JDR4_OFFSET)
#define rDR		REG(STM32_ADC_DR_OFFSET)

#ifdef STM32_ADC_CCR
# define rCCR		REG(STM32_ADC_CCR_OFFSET)

/* Assuming VDC 2.4 - 3.6 */

#define ADC_MAX_FADC 36000000

#  if STM32_PCLK2_FREQUENCY/2 <= ADC_MAX_FADC
#    define ADC_CCR_ADCPRE_DIV     ADC_CCR_ADCPRE_DIV2
#  elif STM32_PCLK2_FREQUENCY/4 <= ADC_MAX_FADC
#    define ADC_CCR_ADCPRE_DIV     ADC_CCR_ADCPRE_DIV4
#  elif STM32_PCLK2_FREQUENCY/6 <= ADC_MAX_FADC
#   define ADC_CCR_ADCPRE_DIV     ADC_CCR_ADCPRE_DIV6
#  elif STM32_PCLK2_FREQUENCY/8 <= ADC_MAX_FADC
#   define ADC_CCR_ADCPRE_DIV     ADC_CCR_ADCPRE_DIV8
#  else
#    error "ADC PCLK2 too high - no divisor found "
#  endif
#endif

class ADC : public cdev::CDev
{
public:
	ADC(uint32_t channels);
	~ADC();

	virtual int		init();

	virtual int		ioctl(file *filp, int cmd, unsigned long arg);
	virtual ssize_t		read(file *filp, char *buffer, size_t len);

protected:
	virtual int		open_first(struct file *filp);
	virtual int		close_last(struct file *filp);

private:
	static const hrt_abstime _tickrate = 10000;	/**< 100Hz base rate */

	hrt_call		_call;
	perf_counter_t		_sample_perf;

	unsigned		_channel_count;
	px4_adc_msg_t		*_samples;		/**< sample buffer */

	orb_advert_t		_to_system_power;
	orb_advert_t		_to_adc_report;

	/** work trampoline */
	static void		_tick_trampoline(void *arg);

	/** worker function */
	void			_tick();

	/**
	 * Sample a single channel and return the measured value.
	 *
	 * @param channel		The channel to sample.
	 * @return			The sampled value, or 0xffff if
	 *				sampling failed.
	 */
	uint16_t		_sample(unsigned channel);

	// update system_power ORB topic, only on FMUv2
	void update_system_power(hrt_abstime now);

	void update_adc_report(hrt_abstime now);
};

ADC::ADC(uint32_t channels) :
	CDev(ADC0_DEVICE_PATH),
	_sample_perf(perf_alloc(PC_ELAPSED, "adc_samples")),
	_channel_count(0),
	_samples(nullptr),
	_to_system_power(nullptr),
	_to_adc_report(nullptr)
{
	/* always enable the temperature sensor */
	channels |= 1 << 16;

	/* allocate the sample array */
	for (unsigned i = 0; i < 32; i++) {
		if (channels & (1 << i)) {
			_channel_count++;
		}
	}

	if (_channel_count > PX4_MAX_ADC_CHANNELS) {
		PX4_ERR("PX4_MAX_ADC_CHANNELS is too small:is %d needed:%d", PX4_MAX_ADC_CHANNELS, _channel_count);
	}

	_samples = new px4_adc_msg_t[_channel_count];

	/* prefill the channel numbers in the sample array */
	if (_samples != nullptr) {
		unsigned index = 0;

		for (unsigned i = 0; i < 32; i++) {
			if (channels & (1 << i)) {
				_samples[index].am_channel = i;
				_samples[index].am_data = 0;
				index++;
			}
		}
	}
}

ADC::~ADC()
{
	if (_samples != nullptr) {
		delete _samples;
	}
}

int board_adc_init()
{
	static bool once = false;

	if (!once) {

		once = true;

		/* do calibration if supported */
#ifdef ADC_CR2_CAL
		rCR2 |= ADC_CR2_CAL;
		px4_usleep(100);

		if (rCR2 & ADC_CR2_CAL) {
			return -1;
		}

#endif

		/* arbitrarily configure all channels for 55 cycle sample time */
		rSMPR1 = 0b00000011011011011011011011011011;
		rSMPR2 = 0b00011011011011011011011011011011;

		/* XXX for F2/4, might want to select 12-bit mode? */
		rCR1 = 0;

		/* enable the temperature sensor / Vrefint channel if supported*/
		rCR2 =
#ifdef ADC_CR2_TSVREFE
			/* enable the temperature sensor in CR2 */
			ADC_CR2_TSVREFE |
#endif
			0;

		/* Soc have CCR */
#ifdef STM32_ADC_CCR
#  ifdef ADC_CCR_TSVREFE
		/* enable temperature sensor in CCR */
		rCCR = ADC_CCR_TSVREFE | ADC_CCR_ADCPRE_DIV;
#  else
		rCCR = ADC_CCR_ADCPRE_DIV;
#  endif
#endif

		/* configure for a single-channel sequence */
		rSQR1 = 0;
		rSQR2 = 0;
		rSQR3 = 0;	/* will be updated with the channel each tick */

		/* power-cycle the ADC and turn it on */
		rCR2 &= ~ADC_CR2_ADON;
		px4_usleep(10);
		rCR2 |= ADC_CR2_ADON;
		px4_usleep(10);
		rCR2 |= ADC_CR2_ADON;
		px4_usleep(10);

		/* kick off a sample and wait for it to complete */
		hrt_abstime now = hrt_absolute_time();
		rCR2 |= ADC_CR2_SWSTART;

		while (!(rSR & ADC_SR_EOC)) {

			/* don't wait for more than 500us, since that means something broke - should reset here if we see this */
			if ((hrt_absolute_time() - now) > 500) {
				return -1;
			}
		}
	} // once

	return OK;
}

int
ADC::init()
{
	int rv = board_adc_init();

	if (rv < 0) {
		PX4_DEBUG("sample timeout");
		return rv;
	}

	/* create the device node */
	return CDev::init();
}

int
ADC::ioctl(file *filp, int cmd, unsigned long arg)
{
	return -ENOTTY;
}

ssize_t
ADC::read(file *filp, char *buffer, size_t len)
{
	const size_t maxsize = sizeof(px4_adc_msg_t) * _channel_count;

	if (len > maxsize) {
		len = maxsize;
	}

	/* block interrupts while copying samples to avoid racing with an update */
	irqstate_t flags = px4_enter_critical_section();
	memcpy(buffer, _samples, len);
	px4_leave_critical_section(flags);

	return len;
}

int
ADC::open_first(struct file *filp)
{
	/* get fresh data */
	_tick();

	/* and schedule regular updates */
	hrt_call_every(&_call, _tickrate, _tickrate, _tick_trampoline, this);

	return 0;
}

int
ADC::close_last(struct file *filp)
{
	hrt_cancel(&_call);
	return 0;
}

void
ADC::_tick_trampoline(void *arg)
{
	(reinterpret_cast<ADC *>(arg))->_tick();
}

void
ADC::_tick()
{
	hrt_abstime now = hrt_absolute_time();

	/* scan the channel set and sample each */
	for (unsigned i = 0; i < _channel_count; i++) {
		_samples[i].am_data = _sample(_samples[i].am_channel);
	}

	update_adc_report(now);
	update_system_power(now);
}

void
ADC::update_adc_report(hrt_abstime now)
{
	adc_report_s adc = {};
	adc.timestamp = now;

	unsigned max_num = _channel_count;

	if (max_num > (sizeof(adc.channel_id) / sizeof(adc.channel_id[0]))) {
		max_num = (sizeof(adc.channel_id) / sizeof(adc.channel_id[0]));
	}

	for (unsigned i = 0; i < max_num; i++) {
		adc.channel_id[i] = _samples[i].am_channel;
		adc.channel_value[i] = _samples[i].am_data * 3.3f / 4096.0f;
	}

	int instance;
	orb_publish_auto(ORB_ID(adc_report), &_to_adc_report, &adc, &instance, ORB_PRIO_HIGH);
}

void
ADC::update_system_power(hrt_abstime now)
{
#if defined (BOARD_ADC_USB_CONNECTED)
	system_power_s system_power = {};
	system_power.timestamp = now;

	system_power.voltage5v_v = 0;
	system_power.voltage3v3_v = 0;
	system_power.v3v3_valid = 0;

	/* Assume HW provides only ADC_SCALED_V5_SENSE */
	int cnt = 1;
	/* HW provides both ADC_SCALED_V5_SENSE and ADC_SCALED_V3V3_SENSORS_SENSE */
#  if defined(ADC_SCALED_V5_SENSE) && defined(ADC_SCALED_V3V3_SENSORS_SENSE)
	cnt++;
#  endif

	for (unsigned i = 0; i < _channel_count; i++) {
#  if defined(ADC_SCALED_V5_SENSE)

		if (_samples[i].am_channel == ADC_SCALED_V5_SENSE) {
			// it is 2:1 scaled
			system_power.voltage5v_v = _samples[i].am_data * (ADC_V5_V_FULL_SCALE / 4096.0f);
			cnt--;

		} else
#  endif
#  if defined(ADC_SCALED_V3V3_SENSORS_SENSE)
		{
			if (_samples[i].am_channel == ADC_SCALED_V3V3_SENSORS_SENSE) {
				// it is 2:1 scaled
				system_power.voltage3v3_v = _samples[i].am_data * (ADC_3V3_SCALE * (3.3f / 4096.0f));
				system_power.v3v3_valid = 1;
				cnt--;
			}
		}

#  endif

		if (cnt == 0) {
			break;
		}
	}

	/* Note once the board_config.h provides BOARD_ADC_USB_CONNECTED,
	 * It must provide the true logic GPIO BOARD_ADC_xxxx macros.
	 */
	// these are not ADC related, but it is convenient to
	// publish these to the same topic

	system_power.usb_connected = BOARD_ADC_USB_CONNECTED;
	/* If provided used the Valid signal from HW*/
#if defined(BOARD_ADC_USB_VALID)
	system_power.usb_valid = BOARD_ADC_USB_VALID;
#else
	/* If not provided then use connected */
	system_power.usb_valid  = system_power.usb_connected;
#endif

	/* The valid signals (HW dependent) are associated with each brick */

	bool  valid_chan[BOARD_NUMBER_BRICKS] = BOARD_BRICK_VALID_LIST;
	system_power.brick_valid = 0;

	for (int b = 0; b < BOARD_NUMBER_BRICKS; b++) {
		system_power.brick_valid |=  valid_chan[b] ? 1 << b : 0;
	}

	system_power.servo_valid   = BOARD_ADC_SERVO_VALID;

	// OC pins are active low
	system_power.periph_5v_oc  = BOARD_ADC_PERIPH_5V_OC;
	system_power.hipower_5v_oc = BOARD_ADC_HIPOWER_5V_OC;

	/* lazily publish */
	if (_to_system_power != nullptr) {
		orb_publish(ORB_ID(system_power), _to_system_power, &system_power);

	} else {
		_to_system_power = orb_advertise(ORB_ID(system_power), &system_power);
	}

#endif // BOARD_ADC_USB_CONNECTED
}

uint16_t board_adc_sample(unsigned channel)
{
	/* clear any previous EOC */

	if (rSR & ADC_SR_EOC) {
		rSR &= ~ADC_SR_EOC;
	}

	/* run a single conversion right now - should take about 60 cycles (a few microseconds) max */
	rSQR3 = channel;
	rCR2 |= ADC_CR2_SWSTART;

	/* wait for the conversion to complete */
	hrt_abstime now = hrt_absolute_time();

	while (!(rSR & ADC_SR_EOC)) {

		/* don't wait for more than 50us, since that means something broke - should reset here if we see this */
		if ((hrt_absolute_time() - now) > 50) {
			return 0xffff;
		}
	}

	/* read the result and clear EOC */
	uint16_t result = rDR;
	return result;
}

uint16_t
ADC::_sample(unsigned channel)
{
	perf_begin(_sample_perf);
	uint16_t result = board_adc_sample(channel);

	if (result == 0xffff) {
		PX4_ERR("sample timeout");
	}

	perf_end(_sample_perf);
	return result;
}

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int adc_main(int argc, char *argv[]);

namespace
{
ADC	*g_adc;

void
test(void)
{

	int fd = open(ADC0_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "can't open ADC device");
	}

	for (unsigned i = 0; i < 50; i++) {
		px4_adc_msg_t data[PX4_MAX_ADC_CHANNELS];
		ssize_t count = read(fd, data, sizeof(data));

		if (count < 0) {
			errx(1, "read error");
		}

		unsigned channels = count / sizeof(data[0]);

		for (unsigned j = 0; j < channels; j++) {
			printf("%d: %u  ", data[j].am_channel, data[j].am_data);
		}

		printf("\n");
		px4_usleep(500000);
	}

	exit(0);
}
}

int
adc_main(int argc, char *argv[])
{
	if (g_adc == nullptr) {
		/* XXX this hardcodes the default channel set for the board in board_config.h - should be configurable */
		g_adc = new ADC(ADC_CHANNELS);

		if (g_adc == nullptr) {
			errx(1, "couldn't allocate the ADC driver");
		}

		if (g_adc->init() != OK) {
			delete g_adc;
			errx(1, "ADC init failed");
		}
	}

	if (argc > 1) {
		if (!strcmp(argv[1], "test")) {
			test();
		}
	}

	exit(0);
}
#endif
