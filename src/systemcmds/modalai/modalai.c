#include <px4_platform_common/module.h>


#include "chip.h"
#include "stm32_gpio.h"
#include "board_config.h"

#include <nuttx/board.h>
#include <arch/board/board.h>

// v2
#ifdef CONFIG_ARCH_CHIP_STM32H743ZI
#include "modalai_fc-v2.h"
#define MODALAI_FC_V2 1
#else
#include "modalai_fc-v1.h"
#endif



__EXPORT int modalai_main(int argc, char *argv[]);

int modalai_main(int argc, char *argv[])
{
	int hw_rev = board_get_hw_revision();
	int hw_ver = board_get_hw_version();

	eHW_TYPE hw_type = eHwNone;

#ifdef MODALAI_FC_V2

	if (hw_rev == 0 && hw_ver == 3) {
		hw_type = eM0079;

	} else {
		return -1;

	}

#else

	if (hw_rev == 6 && hw_ver == 0) {
		hw_type = eM0018;

	} else if (hw_rev == 0 && hw_ver == 1) {
		hw_type = eM0019;

	} else if (hw_rev == 0 && hw_ver == 2) {
		hw_type = eM0051;

	} else {
		return -1;

	}

#endif

	if (argc <= 1) {
#ifdef MODALAI_FC_V2
		modalai_print_usage_v2();
#else
		modalai_print_usage_v1();
#endif
		return 1;
	}

	if (!strcmp(argv[1], "led")) {
#ifdef MODALAI_FC_V2
		return modalai_led_test_v2();
#else
		return modalai_led_test_v1();
#endif

	} else if (!strcmp(argv[1], "con")) {
		if (argc <= 2) {
			PRINT_MODULE_USAGE_COMMAND("con");
			PRINT_MODULE_USAGE_ARG("<1,4,5,6,7,9,10,12,13>", "Connector ID", false);
			PRINT_MODULE_USAGE_ARG("<uint>", "Pin Number", false);
			PRINT_MODULE_USAGE_ARG("0 | 1", "<output state> (defaults to 0)", false);
			return 1;
		}

		uint8_t con = 0;
		uint8_t pin = 0;
		bool state = false;

		if (argc > 2) {
			con = atoi(argv[2]);
		}

		if (argc > 3) {
			pin = atoi(argv[3]);
		}

		if (argc > 4) {
			state = atoi(argv[4]);
		}

#ifdef MODALAI_FC_V2
		return modalai_con_gpio_test_v2(con, pin, state);
#else
		return modalai_con_gpio_test_v1(con, pin, state);
#endif

	} else if (!strcmp(argv[1], "buzz")) {

#ifdef MODALAI_FC_V2
		return modalai_buzz_test_v2(hw_type);
#else
		return modalai_buzz_test_v1(hw_type);
#endif



	} else if (!strcmp(argv[1], "detect")) {
#ifdef MODALAI_FC_V2
		modalai_hw_detect_v2(hw_type);
#else
		modalai_hw_detect_v1(hw_type);
#endif

		return 0;
	}

#ifdef MODALAI_FC_V2
	modalai_print_usage_v2();
#else
	modalai_print_usage_v1();
#endif
	return -EINVAL;
}
