#include "navio_gpio.h"

#include <stdio.h>
#include <unistd.h>

#define LED_CNF     (GPIO_CNF_OUTPUT)
#define LED_pinR    GPIO_PIN4
#define LED_pinG    GPIO_PIN27
#define LED_pinB    GPIO_PIN6

#define LED_OFF 1
#define LED_ON  0

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


	gpio.gpiowrite(LED_pinR, LED_OFF);
	gpio.gpiowrite(LED_pinG, LED_OFF);
	gpio.gpiowrite(LED_pinB, LED_OFF);
	printf("off\n");
	sleep(2);

	gpio.gpiowrite(LED_pinR, LED_ON);
	gpio.gpiowrite(LED_pinG, LED_OFF);
	gpio.gpiowrite(LED_pinB, LED_OFF);
	printf("red\n");
	sleep(2);

	gpio.gpiowrite(LED_pinR, LED_OFF);
	gpio.gpiowrite(LED_pinG, LED_ON);
	gpio.gpiowrite(LED_pinB, LED_OFF);
	printf("green\n");
	sleep(2);

	gpio.gpiowrite(LED_pinR, LED_OFF);
	gpio.gpiowrite(LED_pinG, LED_OFF);
	gpio.gpiowrite(LED_pinB, LED_ON);
	printf("blue\n");
	sleep(2);

	gpio.gpiowrite(LED_pinR, LED_OFF);
	gpio.gpiowrite(LED_pinG, LED_OFF);
	gpio.gpiowrite(LED_pinB, LED_OFF);
	printf("off\n");
	gpio.stop();

	return 0;
}
