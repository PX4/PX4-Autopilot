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

#endif /* ADC_CHANNELS */
