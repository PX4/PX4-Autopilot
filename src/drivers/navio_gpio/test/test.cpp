#include "navio_gpio.h"

#include <stdio.h>
#include <unistd.h>

#define LED_CNF     (GPIO_CNF_OUTPUT)
#define LED_pinR    GPIO_PIN4
#define LED_pinG    GPIO_PIN27
#define LED_pinB    GPIO_PIN6

using namespace navio_gpio;

int do_test();

int do_test()
{
	Gpio gpio;

	if (gpio.start() < 0) {
		return -1;
	}

	gpio.configgpio(LED_CNF | LED_pinR);
	gpio.configgpio(LED_CNF | LED_pinG);
	gpio.configgpio(LED_CNF | LED_pinB);

	gpio.gpiowrite(LED_pinR, 0);
	gpio.gpiowrite(LED_pinG, 0);
	gpio.gpiowrite(LED_pinB, 0);
	printf("off\n");
	sleep(2);

	gpio.gpiowrite(LED_pinR, 1);
	gpio.gpiowrite(LED_pinG, 0);
	gpio.gpiowrite(LED_pinB, 0);
	printf("red\n");
	sleep(2);

	gpio.gpiowrite(LED_pinR, 0);
	gpio.gpiowrite(LED_pinG, 1);
	gpio.gpiowrite(LED_pinB, 0);
	printf("green\n");
	sleep(2);

	gpio.gpiowrite(LED_pinR, 0);
	gpio.gpiowrite(LED_pinG, 0);
	gpio.gpiowrite(LED_pinB, 1);
	printf("blue\n");
	sleep(2);

	gpio.gpiowrite(LED_pinR, 0);
	gpio.gpiowrite(LED_pinG, 0);
	gpio.gpiowrite(LED_pinB, 0);
	printf("off\n");
	gpio.stop();

	return 0;
}
