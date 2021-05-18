#include <px4_platform_common/module.h>


#include "chip.h"
#include "stm32_gpio.h"
#include "board_config.h"

#include <nuttx/board.h>
#include <arch/board/board.h>

// v2
#ifdef CONFIG_ARCH_CHIP_STM32H743ZI

#include "modalai_fc-v2.h"

void modalai_print_usage_v2(void)
{
	return;
}
void modalai_print_usage_con_gpio_test_v2(void)
{
	return;
}
int modalai_con_gpio_test_v2(uint8_t con, uint8_t pin, bool state)
{
	return 0;
}
int modalai_led_test_v2(void)
{
	PX4_INFO("Running led test");

	stm32_configgpio(GPIO_nLED_RED);
	stm32_configgpio(GPIO_nLED_GREEN);
	stm32_configgpio(GPIO_nLED_BLUE);

	int i = 0;

	for (i = 0; i < 3; i++) {
		usleep(1000 * 100);
		stm32_gpiowrite(GPIO_nLED_RED, false);

		usleep(1000 * 100);
		stm32_gpiowrite(GPIO_nLED_RED, true);

		usleep(1000 * 100);
		stm32_gpiowrite(GPIO_nLED_GREEN, false);
		usleep(1000 * 100);
		stm32_gpiowrite(GPIO_nLED_GREEN, true);

		usleep(1000 * 100);
		stm32_gpiowrite(GPIO_nLED_BLUE, false);
		usleep(1000 * 100);
		stm32_gpiowrite(GPIO_nLED_BLUE, true);
	}

	return 0;
}

bool test_pair(uint32_t output_pin, uint32_t input_pin)
{

	bool state = false;

	stm32_gpiowrite(output_pin, true);
	usleep(1000 * 10);
	state = stm32_gpioread(input_pin);

	if (state != true) {
		return false;
	}

	usleep(1000 * 10);

	stm32_gpiowrite(output_pin, false);
	usleep(1000 * 10);
	state = stm32_gpioread(input_pin);

	if (state != false) {
		return false;
	}

	return true;
}

int modalai_buzz_test_v2(eHW_TYPE hw_type)
{
	PX4_INFO("test: buzz");
	usleep(1000 * 100 * 10);

	if (hw_type == eM0079) {
		PX4_INFO("Using M0079 config");

	} else {
		return -1;

	}

	if (hw_type == eM0079) {
		//
		// J1
		//
		PX4_INFO(">> Testing J1");
		stm32_configgpio(M0079_J1_PIN_2_OUT); // 2-3
		stm32_configgpio(M0079_J1_PIN_3_IN);  // 3-2
		stm32_configgpio(M0079_J1_PIN_4_OUT); // 4-5
		stm32_configgpio(M0079_J1_PIN_5_IN);  // 5-4

		if (test_pair(M0079_J1_PIN_2_OUT, M0079_J1_PIN_3_IN)) {
			PX4_INFO("PASS: M0079_J1_PIN_2_OUT M0079_J1_PIN_3_IN");

		} else {
			PX4_ERR("FAIL: M0079_J1_PIN_2_OUT M0079_J1_PIN_3_IN");
		}

		if (test_pair(M0079_J1_PIN_4_OUT, M0079_J1_PIN_5_IN)) {
			PX4_INFO("PASS: M0079_J1_PIN_4_OUT M0079_J1_PIN_5_IN");

		} else {
			PX4_ERR("FAIL: M0079_J1_PIN_4_OUT M0079_J1_PIN_5_IN");
		}

		//
		// J5
		//
		PX4_INFO(">> Testing J5");
		stm32_configgpio(M0079_J5_PIN_2_OUT); // 2-4
		stm32_configgpio(M0079_J5_PIN_3_OUT); // 3-5
		stm32_configgpio(M0079_J5_PIN_4_IN);  // 4-2
		stm32_configgpio(M0079_J5_PIN_5_IN);  // 5-3

		if (test_pair(M0079_J5_PIN_2_OUT, M0079_J5_PIN_4_IN)) {
			PX4_INFO("PASS: M0079_J5_PIN_2_OUT M0079_J5_PIN_4_IN");

		} else {
			PX4_ERR("FAIL: M0079_J5_PIN_2_OUT M0079_J5_PIN_4_IN");
		}

		if (test_pair(M0079_J5_PIN_3_OUT, M0079_J5_PIN_5_IN)) {
			PX4_INFO("PASS: M0079_J5_PIN_3_OUT M0079_J5_PIN_5_IN");

		} else {
			PX4_ERR("FAIL: M0079_J5_PIN_3_OUT M0079_J5_PIN_5_IN");
		}

		//
		// J7
		//
		PX4_INFO(">> Testing J7");
		stm32_configgpio(M0079_J7_PIN_2_OUT); // 2-6
		stm32_configgpio(M0079_J7_PIN_3_OUT); // 3-7
		stm32_configgpio(M0079_J7_PIN_4_OUT); // 4-8
		stm32_configgpio(M0079_J7_PIN_5_OUT); // 5-9
		stm32_configgpio(M0079_J7_PIN_6_IN); // 6-2
		stm32_configgpio(M0079_J7_PIN_7_IN); // 7-3
		stm32_configgpio(M0079_J7_PIN_8_IN); // 8-4
		stm32_configgpio(M0079_J7_PIN_9_IN); // 9-5

		if (test_pair(M0079_J7_PIN_2_OUT, M0079_J7_PIN_6_IN)) {
			PX4_INFO("PASS: M0079_J7_PIN_2_OUT M0079_J7_PIN_6_IN");

		} else {
			PX4_ERR("FAIL: M0079_J7_PIN_2_OUT M0079_J7_PIN_6_IN");
		}

		if (test_pair(M0079_J7_PIN_3_OUT, M0079_J7_PIN_7_IN)) {
			PX4_INFO("PASS: M0079_J7_PIN_3_OUT M0079_J7_PIN_7_IN");

		} else {
			PX4_ERR("FAIL: M0079_J7_PIN_3_OUT M0079_J7_PIN_7_IN");
		}

		if (test_pair(M0079_J7_PIN_4_OUT, M0079_J7_PIN_8_IN)) {
			PX4_INFO("PASS: M0079_J7_PIN_4_OUT M0079_J7_PIN_8_IN");

		} else {
			PX4_ERR("FAIL: M0079_J7_PIN_4_OUT M0079_J7_PIN_8_IN");
		}

		if (test_pair(M0079_J7_PIN_5_OUT, M0079_J7_PIN_9_IN)) {
			PX4_INFO("PASS: M0079_J7_PIN_5_OUT M0079_J7_PIN_9_IN");

		} else {
			PX4_ERR("FAIL: M0079_J7_PIN_5_OUT M0079_J7_PIN_9_IN");
		}

		//
		// J10
		//
		PX4_INFO(">> Testing J10");
		stm32_configgpio(M0079_J10_PIN_2_OUT); // 2-4
		stm32_configgpio(M0079_J10_PIN_3_OUT); // 3-5
		stm32_configgpio(M0079_J10_PIN_4_IN);  // 4-2
		stm32_configgpio(M0079_J10_PIN_5_IN);  // 5-3

		if (test_pair(M0079_J10_PIN_2_OUT, M0079_J10_PIN_4_IN)) {
			PX4_INFO("PASS: M0079_J10_PIN_2_OUT M0079_J10_PIN_4_IN");

		} else {
			PX4_ERR("FAIL: M0079_J10_PIN_2_OUT M0079_J10_PIN_4_IN");
		}

		if (test_pair(M0079_J10_PIN_3_OUT, M0079_J10_PIN_5_IN)) {
			PX4_INFO("PASS: M0079_J10_PIN_3_OUT M0079_J10_PIN_5_IN");

		} else {
			PX4_ERR("FAIL: M0079_J10_PIN_3_OUT M0079_J10_PIN_5_IN");
		}


		//
		// J13
		//
		PX4_INFO(">> Testing J13");
		stm32_configgpio(M0079_J12_PIN_2_OUT); // 2-3
		stm32_configgpio(M0079_J12_PIN_3_IN);  // 3-2

		if (test_pair(M0079_J12_PIN_2_OUT, M0079_J12_PIN_3_IN)) {
			PX4_INFO("PASS: M0079_J12_PIN_2 M0079_J12_PIN_3");

		} else {
			PX4_ERR("FAIL: M0079_J12_PIN_2 M0079_J12_PIN_3");
		}

		//
		// J14
		//
		/*
		PX4_INFO(">> Testing J14");
		stm32_configgpio(M0079_J15_PIN_2_OUT);  // 2-3
		stm32_configgpio(M0079_J15_PIN_3_IN);   // 3-2
		stm32_configgpio(M0079_J15_PIN_4_OUT);  // 4-5
		stm32_configgpio(M0079_J15_PIN_5_IN);   // 5-4
		stm32_configgpio(M0079_J15_PIN_6_OUT);  // 6-7

		// ADCs
		//stm32_configgpio(M0079_J15_PIN_7_IN);   // 7-6
		//stm32_configgpio(M0079_J15_PIN_8_OUT);  // 8-9
		//stm32_configgpio(M0079_J15_PIN_9_IN);   // 9-8
		//stm32_configgpio(M0079_J15_PIN_10_OUT); // 10-11
		//stm32_configgpio(M0079_J15_PIN_11_IN);  // 11-10

		if (test_pair(M0079_J15_PIN_2_OUT, M0079_J15_PIN_3_IN)) {
			PX4_INFO("PASS: M0079_J15_PIN_2_OUT M0079_J15_PIN_3_IN");

		} else {
			PX4_ERR("FAIL: M0079_J15_PIN_2_OUT M0079_J15_PIN_3_IN");
		}
		if (test_pair(M0079_J15_PIN_4_OUT, M0079_J15_PIN_5_IN)) {
			PX4_INFO("PASS: M0079_J15_PIN_4_OUT M0079_J15_PIN_5_IN");

		} else {
			PX4_ERR("FAIL: M0079_J15_PIN_4_OUT M0079_J15_PIN_5_IN");
		}
		if (test_pair(M0079_J15_PIN_6_OUT, M0079_J15_PIN_7_IN)) {
			PX4_INFO("PASS: M0079_J15_PIN_6_OUT M0079_J15_PIN_7_IN");

		} else {
			PX4_ERR("FAIL: M0079_J15_PIN_6_OUT M0079_J15_PIN_7_IN");
		}
		if (test_pair(M0079_J15_PIN_8_OUT, M0079_J15_PIN_9_IN)) {
			PX4_INFO("PASS: M0079_J15_PIN_8_OUT M0079_J15_PIN_9_IN");

		} else {
			PX4_ERR("FAIL: M0079_J15_PIN_8_OUT M0079_J15_PIN_9_IN");
		}
		if (test_pair(M0079_J15_PIN_10_OUT, M0079_J15_PIN_11_IN)) {
			PX4_INFO("PASS: M0079_J15_PIN_10_OUT M0079_J15_PIN_11_IN");

		} else {
			PX4_ERR("FAIL: M0079_J15_PIN_10_OUT M0079_J15_PIN_11_IN");
		}
		*/
	}

	return 0;
}

int modalai_hw_detect_v2(eHW_TYPE hw_type)
{
	int result = -1;

	if (hw_type == eM0079) {
		PX4_INFO("V230 - M0079");
		result = 0;

	} else {
		PX4_ERR("Unknown hardware");
	}

	return result;
}

#endif //CONFIG_ARCH_CHIP_STM32H743ZI
