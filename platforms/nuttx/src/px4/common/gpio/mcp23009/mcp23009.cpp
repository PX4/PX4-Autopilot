/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
#include <drivers/drv_sensor.h>
#include <lib/drivers/device/Device.hpp>
#include <uORB/topics/gpio_config.h>
#include <uORB/topics/gpio_in.h>
#include <uORB/topics/gpio_out.h>
#include <uORB/topics/gpio_request.h>
#include <uORB/Publication.hpp>
#include <uORB/SubscriptionCallback.hpp>

static uint32_t DEVID{0};
struct mcp23009_gpio_dev_s {
	struct gpio_dev_s gpio;
	uint8_t mask;
};

/* Copy the read input data */
class ReadCallback : public uORB::SubscriptionCallback
{
public:
	using SubscriptionCallback::SubscriptionCallback;

	void call() override
	{
		px4::msg::GpioIn new_input;

		if (update(&new_input) && new_input.device_id == DEVID) {
			input = new_input.state;
		}

	}

	uint8_t input;
};

static uORB::Publication<px4::msg::GpioRequest> toGpioRequest{ORB_ID(gpio_request)};
static ReadCallback fromGpioIn{ORB_ID(gpio_in)};
static int mcp23009_read(struct gpio_dev_s *dev, bool *value)
{
	mcp23009_gpio_dev_s *gpio = (struct mcp23009_gpio_dev_s *)dev;
	*value = fromGpioIn.input & gpio->mask;
	return OK;
}

static uORB::Publication<gpio_out_s> toGpioOut{ORB_ID(gpio_out)};
static int mcp23009_write(struct gpio_dev_s *dev, bool value)
{
	mcp23009_gpio_dev_s *gpio = (struct mcp23009_gpio_dev_s *)dev;
	gpio_out_s msg{
		hrt_absolute_time(),
		DEVID,
		gpio->mask,			// clear mask
		value ? gpio->mask : 0u,	// set mask
	};
	return toGpioOut.publish(msg) ? OK : -ETIMEDOUT;
}

static uORB::Publication<gpio_config_s> toGpioConfig{ORB_ID(gpio_config)};
static int mcp23009_setpintype(struct gpio_dev_s *dev, enum gpio_pintype_e pintype)
{
	mcp23009_gpio_dev_s *gpio = (struct mcp23009_gpio_dev_s *)dev;
	gpio_config_s msg{
		hrt_absolute_time(),
		DEVID,
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



// ----------------------------------------------------------------------------
static const struct gpio_operations_s mcp23009_gpio_ops {
	mcp23009_read,
	mcp23009_write,
	nullptr,
	nullptr,
	mcp23009_setpintype,
};

static constexpr uint8_t NUM_GPIOS = 8;
static mcp23009_gpio_dev_s _gpio[NUM_GPIOS] {
	{ {GPIO_INPUT_PIN, {}, &mcp23009_gpio_ops}, (1u << 0) },
	{ {GPIO_INPUT_PIN, {}, &mcp23009_gpio_ops}, (1u << 1) },
	{ {GPIO_INPUT_PIN, {}, &mcp23009_gpio_ops}, (1u << 2) },
	{ {GPIO_INPUT_PIN, {}, &mcp23009_gpio_ops}, (1u << 3) },
	{ {GPIO_INPUT_PIN, {}, &mcp23009_gpio_ops}, (1u << 4) },
	{ {GPIO_INPUT_PIN, {}, &mcp23009_gpio_ops}, (1u << 5) },
	{ {GPIO_INPUT_PIN, {}, &mcp23009_gpio_ops}, (1u << 6) },
	{ {GPIO_INPUT_PIN, {}, &mcp23009_gpio_ops}, (1u << 7) }
};

// ----------------------------------------------------------------------------
int mcp23009_register_gpios(uint8_t i2c_bus, uint8_t i2c_addr, int first_minor)
{
	const auto device_id = device::Device::DeviceId{
		device::Device::DeviceBusType_I2C, i2c_bus, i2c_addr, DRV_GPIO_DEVTYPE_MCP23009};
	DEVID = device_id.devid;

	for (int i = 0; i < NUM_GPIOS; ++i) {
		int ret = gpio_pin_register(&_gpio[i].gpio, first_minor + i);

		if (ret != OK) {
			return ret;
		}
	}

	fromGpioIn.registerCallback();
	return OK;
}

int mcp23009_unregister_gpios(int first_minor)
{
	for (int i = 0; i < NUM_GPIOS; ++i) {
		mcp23009_setpintype(&_gpio[i].gpio, GPIO_INPUT_PIN);
		gpio_pin_unregister(&_gpio[i].gpio, first_minor + i);
	}

	fromGpioIn.unregisterCallback();
	return OK;
}
