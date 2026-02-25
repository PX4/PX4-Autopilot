/****************************************************************************
 *
 *   Copyright (C) 2025 PX4 Development Team. All rights reserved.
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
#include <nuttx/ioexpander/gpio.h>
#include <lib/drivers/device/Device.hpp>
#include <px4_platform/gpio/mcp.hpp>
#include <uORB/topics/gpio_config.h>
#include <uORB/topics/gpio_out.h>
#include <uORB/PublicationMulti.hpp>
#include <drivers/drv_hrt.h>

static int mcp230XX_read(struct gpio_dev_s *dev, bool *value)
{
	mcp_gpio_dev_s *gpio = (struct mcp_gpio_dev_s *)dev;
	*value = gpio->callback_handler->input & gpio->mask;
	return OK;
}

static uORB::PublicationMulti<gpio_out_s> toGpioOut{ORB_ID(gpio_out)};
static int mcp230XX_write(struct gpio_dev_s *dev, bool value)
{
	mcp_gpio_dev_s *gpio = (struct mcp_gpio_dev_s *)dev;
	gpio_out_s msg{
		hrt_absolute_time(),
		gpio->callback_handler->dev_id,
		gpio->mask,			// clear mask
		value ? gpio->mask : 0u,	// set mask
	};
	return toGpioOut.publish(msg) ? OK : -ETIMEDOUT;
}

static uORB::PublicationMulti<gpio_config_s> toGpioConfig{ORB_ID(gpio_config)};
static int mcp230XX_setpintype(struct gpio_dev_s *dev, enum gpio_pintype_e pintype)
{
	mcp_gpio_dev_s *gpio = (struct mcp_gpio_dev_s *)dev;
	gpio_config_s msg{
		hrt_absolute_time(),
		gpio->callback_handler->dev_id,
		gpio->mask,
	};

	switch (pintype) {
	case GPIO_INPUT_PIN:
		msg.config = gpio_config_s::INPUT;
		break;

	case GPIO_INPUT_PIN_PULLUP:
		msg.config = gpio_config_s::INPUT_PULLUP;
		break;

	case GPIO_OUTPUT_PIN:
		msg.config = gpio_config_s::OUTPUT;
		break;

	default:
		return -ENOTSUP;
	}

	return toGpioConfig.publish(msg) ? OK : -ETIMEDOUT;
}

static const struct gpio_operations_s mcp_gpio_ops {
	mcp230XX_read,
	mcp230XX_write,
	nullptr,
	nullptr,
	mcp230XX_setpintype,
};

int mcp230XX_register_gpios(uint8_t i2c_bus, uint8_t i2c_addr, int first_minor, uint16_t dir_mask, int num_pins, uint8_t device_type,
			    mcp_gpio_dev_s *_gpio)
{
	bool all_registered = true;

	for (int i = 0; i < num_pins; i++) {
		uint16_t mask = 1u << i;

		if (!_gpio[i].registered) {
			if (dir_mask & mask) {
				_gpio[i] = mcp_gpio_dev_s{{GPIO_INPUT_PIN, {}, (uintptr_t) &mcp_gpio_ops}, mask, false, nullptr};

			} else {
				_gpio[i] = mcp_gpio_dev_s{{GPIO_OUTPUT_PIN, {}, (uintptr_t) &mcp_gpio_ops}, mask, false, nullptr};
			}

			int ret = gpio_pin_register(&_gpio[i].gpio, first_minor + i);

			if (ret != OK) {
				all_registered = false;

			} else {
				_gpio[i].registered = true;
			}
		}
	}

	const auto device_id = device::Device::DeviceId{device::Device::DeviceBusType_I2C, i2c_bus, i2c_addr, device_type};
	CallbackHandler *callback_handler = new CallbackHandler(ORB_ID(gpio_in));
	callback_handler->dev_id = device_id.devid;
	bool callback_registered = callback_handler->registerCallback();

	if (!all_registered || !callback_registered) {
		for (int i = 0; i < num_pins; i++) {
			if (_gpio[i].registered) {
				gpio_pin_unregister(&_gpio[i].gpio, first_minor + i);
				_gpio[i].registered = false;
			}
		}

		callback_handler->unregisterCallback();
		delete callback_handler;
		return ERROR;
	}

	for (int i = 0; i < num_pins; i++) {
		_gpio[i].callback_handler = callback_handler;
	}

	return OK;
}

int mcp230XX_unregister_gpios(int first_minor, int num_pins, mcp_gpio_dev_s *_gpio)
{
	if (_gpio[0].callback_handler) {
		CallbackHandler *callback_handler = _gpio[0].callback_handler;
		callback_handler->unregisterCallback();
		delete callback_handler;
	}

	for (int i = 0; i < num_pins; ++i) {
		if (_gpio[i].registered) {
			mcp230XX_setpintype(&_gpio[i].gpio, GPIO_INPUT_PIN);
			gpio_pin_unregister(&_gpio[i].gpio, first_minor + i);
			_gpio[i].registered = false;
			_gpio[i].callback_handler = nullptr;
		}
	}

	return OK;
}
