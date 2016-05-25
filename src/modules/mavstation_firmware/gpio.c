
#include <stdint.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <stm32_gpio.h>

#include "gpio.h"
#include "hardware.h"

struct btn {
	uint32_t gpio;
	bool state;
};

static const uint32_t leds[] = { GPIO_LED1, GPIO_LED2, GPIO_LED3 };
static const int num_leds = sizeof(leds) / sizeof(leds[0]);

static struct btn buttons[] = { { .gpio = GPIO_BTN0, .state = false }
	, { .gpio = GPIO_BTN1, .state = false }
	, { .gpio = GPIO_BTN2, .state = false }
	, { .gpio = GPIO_BTN3, .state = false }
	, { .gpio = GPIO_BTN4, .state = false }
};
static int num_buttons = sizeof(buttons) / sizeof(buttons[0]);

void gpio_interface_init(void)
{
	for (int i = 0; i < num_leds; i++) {
		stm32_gpiowrite(leds[i], true);
		stm32_configgpio(leds[i]);
	}

	for (int i = 0; i < num_buttons; i++) {
		stm32_configgpio(buttons[i].gpio);
	}

	stm32_gpiowrite(GPIO_USART2MUX, false);
	stm32_configgpio(GPIO_USART2MUX);
}

void gpio_interface_tick(void)
{
	for (int i = 0; i < num_buttons; i++) {
		buttons[i].state = stm32_gpioread(buttons[i].gpio);
	}
}

void gpio_interface_setusart2mux(bool conn_to_rpi)
{
	stm32_gpiowrite(GPIO_USART2MUX, conn_to_rpi);
}

void gpio_interface_setled(int led, bool on)
{
	if (led >= 0 && led < num_leds) {
		stm32_gpiowrite(leds[led], !on);
	}
}

bool gpio_interface_getbtn(int btn)
{
	if (btn >= 0 && btn < num_buttons) {
		return buttons[btn].state;

	} else { return false; }
}

