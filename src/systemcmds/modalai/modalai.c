#include <px4_platform_common/module.h>


#include "chip.h"
#include "stm32_gpio.h"
#include "board_config.h"

#include <nuttx/board.h>
#include <arch/board/board.h>

typedef enum {
	eHwUnknown = -1,
	eHwNone = 0,
	eM0018,           // Flight Core
	eM0019,           // VOXL Flight
	eM0051,
	eM0052,
	eM0053
} eHW_TYPE;


#define _MK_GPIO_INPUT(def)  (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_PULLUP))
#define _MK_GPIO_OUTPUT(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR))


//
// Flight Core - J1 - Primary MSS Communications Interface
// VOXL Flight - NA
//
#define J1_PIN2_IN  _MK_GPIO_INPUT(GPIO_UART5_RX)
#define J1_PIN3     _MK_GPIO_OUTPUT(GPIO_UART5_TX)
#define J1_PIN4     _MK_GPIO_OUTPUT(GPIO_UART5_RTS)
#define J1_PIN6_IN  _MK_GPIO_INPUT(GPIO_UART5_CTS)

//
// STM JTAG Programming Header
// Flight Core - J2
// VOXL Flight - J1001
//

//
// USB 2.0 Full-Speed Downstream Device Port
// Flight Core - J
// VOXL Flight - J1006
//

//
// Spare MSS Comms
// Flight Core - J4
// VOXL Flight - J1002
//
#define J4_PIN2       _MK_GPIO_OUTPUT(GPIO_USART2_RX)
#define J1002_PIN2    J4_PIN2
#define J4_PIN3       _MK_GPIO_OUTPUT(GPIO_USART2_TX)
#define J1002_PIN3    J4_PIN3
#define J4_PIN4       _MK_GPIO_OUTPUT(GPIO_USART2_RTS)
#define J1002_PIN4    J4_PIN4
#define J4_PIN4_IN    _MK_GPIO_INPUT(GPIO_USART2_RTS)
#define J1002_PIN4_IN J4_PIN4_IN
#define J4_PIN6       _MK_GPIO_OUTPUT(GPIO_USART2_CTS)
#define J4_PIN6_IN    _MK_GPIO_INPUT(GPIO_USART2_CTS)
#define J1002_PIN6_IN J4_PIN6_IN
#define J4_PIN7       _MK_GPIO_OUTPUT(GPIO_VOXL_STATUS_OUT)
#define J4_PIN7_IN    _MK_GPIO_INPUT(GPIO_VOXL_STATUS_OUT)
#define J4_PIN8       _MK_GPIO_OUTPUT(GPIO_VOXL_STATUS_IN)
#define J4_PIN8_IN    _MK_GPIO_INPUT(GPIO_VOXL_STATUS_IN)

//
// TELEMETRY CONNECTOR
// Flight Core - J5
// VOXL Flight - J1010
//
#define J5_PIN2       _MK_GPIO_OUTPUT(GPIO_UART7_TX)
#define J1010_PIN2    J5_PIN2
#define J5_PIN3       _MK_GPIO_OUTPUT(GPIO_UART7_RX)
#define J1010_PIN3    J5_PIN3
#define J5_PIN4       _MK_GPIO_OUTPUT(GPIO_UART7_CTS)
#define J1010_PIN4    J5_PIN4
#define J5_PIN4_IN    _MK_GPIO_INPUT(GPIO_UART7_CTS)
#define J1010_PIN4_IN J5_PIN4_IN
#define J5_PIN5       _MK_GPIO_OUTPUT(GPIO_UART7_RTS)
#define J1010_PIN5    J5_PIN5
#define J5_PIN5_IN    _MK_GPIO_INPUT(GPIO_UART7_RTS)
#define J1010_PIN5_IN J5_PIN5_IN

//
// EXPANSION CONNECTOR
// Flight Core - J6
// VOXL Flight - J1009
//
#define J6_PIN2        _MK_GPIO_OUTPUT(GPIO_UART4_TX_5)
#define J1009_PIN2     J6_PIN2
#define J6_PIN3        _MK_GPIO_OUTPUT(GPIO_UART4_RX_5)
#define J1009_PIN3     J6_PIN3
#define J6_PIN4        _MK_GPIO_OUTPUT(GPIO_I2C3_SCL_2)
#define J1009_PIN4     J6_PIN4
#define J6_PIN4_IN     _MK_GPIO_INPUT(GPIO_I2C3_SCL_2)
#define J1009_PIN4_IN  J6_PIN4_IN
#define J6_PIN5        _MK_GPIO_OUTPUT(GPIO_I2C3_SDA_2)
#define J1009_PIN5     J6_PIN5
#define J6_PIN5_IN     _MK_GPIO_INPUT(GPIO_I2C3_SDA_2)
#define J1009_PIN5_IN  J6_PIN5_IN

//
// Flight Core - J7 - PWM Output Connector
// VOXL Flight - J1007
// M0051       - J13
//
#define J7_PIN2          _MK_GPIO_OUTPUT(GPIO_TIM1_CH4OUT_2)
#define J1007_PIN2       J7_PIN2
#define M0051J13_PIN2    J7_PIN2
#define J7_PIN3          _MK_GPIO_OUTPUT(GPIO_TIM1_CH3OUT_1)
#define J1007_PIN3       J7_PIN3
#define M0051J13_PIN3    J7_PIN3
#define J7_PIN4          _MK_GPIO_OUTPUT(GPIO_TIM1_CH2OUT_2)
#define J1007_PIN4       J7_PIN4
#define M0051J13_PIN4    J7_PIN4
#define J7_PIN5          _MK_GPIO_OUTPUT(GPIO_TIM1_CH1OUT_1)
#define J1007_PIN5       J7_PIN5
#define M0051J13_PIN5    J7_PIN5
#define J7_PIN6          _MK_GPIO_OUTPUT(GPIO_TIM4_CH2OUT_2)
#define J1007_PIN6       J7_PIN6
#define M0051J13_PIN6    J7_PIN6
#define J7_PIN6_IN       _MK_GPIO_INPUT(GPIO_TIM4_CH2OUT_2)
#define J1007_PIN6_IN    J7_PIN6_IN
#define M0051J13_PIN6_IN J7_PIN6_IN
#define J7_PIN7          _MK_GPIO_OUTPUT(GPIO_TIM4_CH3OUT_2)
#define J1007_PIN7       J7_PIN7
#define M0051J13_PIN7    J7_PIN7
#define J7_PIN7_IN       _MK_GPIO_INPUT(GPIO_TIM4_CH3OUT_2)
#define J1007_PIN7_IN    J7_PIN7_IN
#define M0051J13_PIN7_IN J7_PIN7_IN
#define J7_PIN8          _MK_GPIO_OUTPUT(GPIO_TIM4_CH1OUT_2)
#define J1007_PIN8       J7_PIN8
#define M0051J13_PIN8    J7_PIN8
#define J7_PIN8_IN       _MK_GPIO_INPUT(GPIO_TIM4_CH1OUT_2)
#define J1007_PIN8_IN    J7_PIN8_IN
#define M0051J13_PIN8_IN J7_PIN8_IN
#define J7_PIN9          _MK_GPIO_OUTPUT(GPIO_TIM4_CH4OUT_2)
#define J1007_PIN9       J7_PIN9
#define M0051J13_PIN9    J7_PIN9
#define J7_PIN9_IN       _MK_GPIO_INPUT(GPIO_TIM4_CH4OUT_2)
#define J1007_PIN9_IN    J7_PIN9_IN
#define M0051J13_PIN9_IN J7_PIN9_IN

//
// CAN 1 Peripheral Connector
// Flight Core - J8
// VOXL Flight - J1008
//
//#define J8_PIN2     _MK_GPIO_OUTPUT()
//#define J8_PIN3     _MK_GPIO_OUTPUT()

// PPM (RC) IN
// Flight Core - J9
// VOXL Flight - J1003
//
#define J9_PIN2_IN    _MK_GPIO_INPUT(GPIO_TIM8_CH1IN_2)
#define J1003_PIN2_IN J9_PIN2_IN

//
// GPS CONNECTOR
// Flight Core - J10
// VOXL Flight - J1012
// M0051       - J15
//
#define J10_PIN2         _MK_GPIO_OUTPUT(GPIO_USART1_TX_3)
#define J1012_PIN2       J10_PIN2
#define M0051J15_PIN2    J10_PIN2
#define J10_PIN3         _MK_GPIO_OUTPUT(GPIO_USART1_RX_3)
#define J1012_PIN3       J10_PIN3
#define M0051J15_PIN3    J10_PIN3
#define J10_PIN4         _MK_GPIO_OUTPUT(GPIO_I2C1_SCL_2)
#define J1012_PIN4       J10_PIN4
#define M0051J15_PIN4    J10_PIN4
#define J10_PIN4_IN      _MK_GPIO_INPUT(GPIO_I2C1_SCL_2)
#define J1012_PIN4_IN    J10_PIN4_IN
#define M0051J15_PIN4_IN J10_PIN4_IN
#define J10_PIN5         _MK_GPIO_OUTPUT(GPIO_I2C1_SDA_1)
#define J1012_PIN5       J10_PIN5
#define M0051J15_PIN5    J10_PIN5
#define J10_PIN5_IN      _MK_GPIO_INPUT(GPIO_I2C1_SDA_1)
#define J1012_PIN5_IN    J10_PIN5_IN
#define M0051J15_PIN5_IN J10_PIN5_IN

//
// Spektrum RC Input Connector
// Flight Core - J12
// VOXL Flight - J1004
// M0051       - J14
//
#define J12_PIN1      GPIO_VDD_3V3_SPEKTRUM_POWER_EN
#define J1004_PIN1       J12_PIN1
#define M0051J14_PIN1    J12_PIN1
#define J12_PIN2         _MK_GPIO_OUTPUT(GPIO_USART6_TX_1)
#define J1004_PIN2       J12_PIN2
#define M0051J14_PIN2    J12_PIN2
#define J12_PIN2_IN      _MK_GPIO_INPUT(GPIO_USART6_TX_1)
#define J1004_PIN2_IN    J12_PIN2_IN
#define M0051J14_PIN2_IN J12_PIN2_IN
#define J12_PIN3         _MK_GPIO_OUTPUT(GPIO_USART6_RX_1)
#define J1004_PIN3       J12_PIN3
#define M0051J14_PIN3    J12_PIN3
#define J12_PIN3_IN      _MK_GPIO_INPUT(GPIO_USART6_RX_1)
#define J1004_PIN3_IN    J12_PIN3_IN
#define M0051J14_PIN3_IN J12_PIN3_IN

//
// I2C Display / Spare Sensor Connector
// Flight Core - J13
// VOXL Flight - J1011
//
#define J13_PIN3   _MK_GPIO_OUTPUT(GPIO_I2C2_SDA_2)
#define J1011_PIN3 J13_PIN3
#define J13_PIN4   _MK_GPIO_OUTPUT(GPIO_I2C2_SCL_2)
#define J1011_PIN4 J13_PIN4
#define J13_PIN5   _MK_GPIO_OUTPUT(GPIO_PF3_EVENTOUT)
#define J1011_PIN5 J13_PIN5

__EXPORT int modalai_main(int argc, char *argv[]);

static void print_usage(void)
{
	PRINT_MODULE_DESCRIPTION("ModalAI Test utility\n");

	PRINT_MODULE_USAGE_NAME_SIMPLE("modalai", "command");

	PRINT_MODULE_USAGE_COMMAND_DESCR("led", "LED Test");
	PRINT_MODULE_USAGE_COMMAND_DESCR("con", "Connector Output Test (as GPIO)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("buzz", "Automated buzz out test");
	PRINT_MODULE_USAGE_COMMAND_DESCR("detect", "Detect board type");
}

static void print_usage_con_gpio_test(void)
{
	PRINT_MODULE_USAGE_NAME_SIMPLE("modalai con", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("1",  "W<3,6> R<2,6>, <0-1>");
	PRINT_MODULE_USAGE_COMMAND_DESCR("4",  "W<2-4,6-7> R<8>, <0-1>");
	PRINT_MODULE_USAGE_COMMAND_DESCR("5",  "W<2-5>,   <0-1>");
	PRINT_MODULE_USAGE_COMMAND_DESCR("6",  "W<2-5>,   <0-1>");
	PRINT_MODULE_USAGE_COMMAND_DESCR("7",  "W<2-9>,   <0-1>");
	PRINT_MODULE_USAGE_COMMAND_DESCR("9",  "R<2>");
	PRINT_MODULE_USAGE_COMMAND_DESCR("10", "W<2-5>,   <0-1>");
	PRINT_MODULE_USAGE_COMMAND_DESCR("12", "W<1-3>,   <0-1>");
	PRINT_MODULE_USAGE_COMMAND_DESCR("13", "W<3-5>,   <0-1>");
}


static int led_test(void)
{
	PX4_INFO("Running led test");

	stm32_configgpio(GPIO_nLED_RED);
	stm32_configgpio(GPIO_nLED_GREEN);
	stm32_configgpio(GPIO_nLED_BLUE);
	stm32_configgpio(GPIO_nLED_2_RED);
	stm32_configgpio(GPIO_nLED_2_GREEN);
	stm32_configgpio(GPIO_nLED_2_BLUE);

	int i = 0;

	for (i = 0; i < 3; i++) {
		usleep(1000 * 100);
		stm32_gpiowrite(GPIO_nLED_RED, false);
		stm32_gpiowrite(GPIO_nLED_2_RED, false);
		usleep(1000 * 100);
		stm32_gpiowrite(GPIO_nLED_RED, true);
		stm32_gpiowrite(GPIO_nLED_2_RED, true);

		usleep(1000 * 100);
		stm32_gpiowrite(GPIO_nLED_GREEN, false);
		stm32_gpiowrite(GPIO_nLED_2_GREEN, false);
		usleep(1000 * 100);
		stm32_gpiowrite(GPIO_nLED_GREEN, true);
		stm32_gpiowrite(GPIO_nLED_2_GREEN, true);

		usleep(1000 * 100);
		stm32_gpiowrite(GPIO_nLED_BLUE, false);
		stm32_gpiowrite(GPIO_nLED_2_BLUE, false);
		usleep(1000 * 100);
		stm32_gpiowrite(GPIO_nLED_BLUE, true);
		stm32_gpiowrite(GPIO_nLED_2_BLUE, true);
	}

	return OK;
}

static int con_gpio_test(uint8_t con, uint8_t pin, bool state)
{
	// validate
	switch (con) {
	// Primary MSS Communications Interface
	case 1:
		switch (pin) {
		case 2:
			stm32_configgpio(J1_PIN2_IN);
			state = stm32_gpioread(J1_PIN2_IN);
			break;

		case 3:
			stm32_configgpio(J1_PIN3);
			stm32_gpiowrite(J1_PIN3, state);
			break;

		case 4:
			stm32_configgpio(J1_PIN4);
			stm32_gpiowrite(J1_PIN4, state);
			break;

		case 6:
			stm32_configgpio(J1_PIN6_IN);
			state = stm32_gpioread(J1_PIN6_IN);
			break;

		default:
			print_usage_con_gpio_test();
			return -1;
		}

		break;

	// STM JTAG Programming Header
	case 2:
		print_usage_con_gpio_test();
		return -1;

	// USB 2.0 Full-Speed Downstream Device Port
	case 3:
		print_usage_con_gpio_test();
		return -1;

	// Spare MSS Communications Interface
	case 4:
		switch (pin) {
		case 2:
			stm32_configgpio(J4_PIN2);
			stm32_gpiowrite(J4_PIN2, state);
			break;

		case 3:
			stm32_configgpio(J4_PIN3);
			stm32_gpiowrite(J4_PIN3, state);
			break;

		case 4:
			stm32_configgpio(J4_PIN4);
			stm32_gpiowrite(J4_PIN4, state);
			break;

		case 6:
			stm32_configgpio(J4_PIN6);
			stm32_gpiowrite(J4_PIN6, state);
			break;

		case 7:
			stm32_configgpio(J4_PIN7);
			stm32_gpiowrite(J4_PIN7, state);
			break;

		case 8:
			stm32_configgpio(J4_PIN8);
			state = stm32_gpioread(J4_PIN8);
			break;

		default:
			print_usage_con_gpio_test();
			return -1;
		}

		break;

	// TELEMETRY CONNECTOR
	case 5:
		switch (pin) {
		case 2:
			stm32_configgpio(J5_PIN2);
			stm32_gpiowrite(J5_PIN2, state);
			break;

		case 3:
			stm32_configgpio(J5_PIN3);
			stm32_gpiowrite(J5_PIN3, state);
			break;

		case 4:
			stm32_configgpio(J5_PIN4);
			stm32_gpiowrite(J5_PIN4, state);
			break;

		case 5:
			stm32_configgpio(J5_PIN5);
			stm32_gpiowrite(J5_PIN5, state);
			break;

		default:
			print_usage_con_gpio_test();
			return -1;
		}

		break;

	// EXPANSION CONNECTOR
	case 6:
		switch (pin) {
		case 2:
			stm32_configgpio(J6_PIN2);
			stm32_gpiowrite(J6_PIN2, state);
			break;

		case 3:
			stm32_configgpio(J6_PIN3);
			stm32_gpiowrite(J6_PIN3, state);
			break;

		case 4:
			stm32_configgpio(J6_PIN4);
			stm32_gpiowrite(J6_PIN4, state);
			break;

		case 5:
			stm32_configgpio(J6_PIN5);
			stm32_gpiowrite(J6_PIN5, state);
			break;

		default:
			print_usage_con_gpio_test();
			return -1;
		}

		break;

	// PWM Output Connector
	case 7:
		switch (pin) {
		case 2:
			stm32_configgpio(J7_PIN2);
			stm32_gpiowrite(J7_PIN2, state);
			break;

		case 3:
			stm32_configgpio(J7_PIN3);
			stm32_gpiowrite(J7_PIN3, state);
			break;

		case 4:
			stm32_configgpio(J7_PIN4);
			stm32_gpiowrite(J7_PIN4, state);
			break;

		case 5:
			stm32_configgpio(J7_PIN5);
			stm32_gpiowrite(J7_PIN5, state);
			break;

		case 6:
			stm32_configgpio(J7_PIN6);
			stm32_gpiowrite(J7_PIN6, state);
			break;

		case 7:
			stm32_configgpio(J7_PIN7);
			stm32_gpiowrite(J7_PIN7, state);
			break;

		case 8:
			stm32_configgpio(J7_PIN8);
			stm32_gpiowrite(J7_PIN8, state);
			break;

		case 9:
			stm32_configgpio(J7_PIN9);
			stm32_gpiowrite(J7_PIN9, state);
			break;

		default:
			print_usage_con_gpio_test();
			return -1;
		}

		break;

	// CAN 1 Peripheral Connector
	case 8:
		print_usage_con_gpio_test();
		return -1;

	// PPM (RC) IN
	case 9:
		switch (pin) {
		case 2:
			stm32_configgpio(J9_PIN2_IN);
			state = stm32_gpioread(J9_PIN2_IN);
			break;

		default:
			print_usage_con_gpio_test();
			return -1;
		}

		break;

	// GPS CONNECTOR
	case 10:
		switch (pin) {
		case 2:
			stm32_configgpio(J10_PIN2);
			stm32_gpiowrite(J10_PIN2, state);
			break;

		case 3:
			stm32_configgpio(J10_PIN3);
			stm32_gpiowrite(J10_PIN3, state);
			break;

		case 4:
			stm32_configgpio(J10_PIN4);
			stm32_gpiowrite(J10_PIN4, state);
			break;

		case 5:
			stm32_configgpio(J10_PIN5);
			stm32_gpiowrite(J10_PIN5, state);
			break;

		default:
			print_usage_con_gpio_test();
			return -1;
		}

		break;

	// Micro SD Card Slot
	case 11:
		print_usage_con_gpio_test();
		return -1;

	// Spektrum RC Input Connector
	case 12:
		switch (pin) {
		case 1:
			VDD_3V3_SPEKTRUM_POWER_EN(state);
			break;

		case 2:
			__asm("nop");
			stm32_configgpio(J12_PIN2);
			stm32_gpiowrite(J12_PIN2, state);
			//state = stm32_gpioread(J12_PIN2);
			__asm("nop");
			break;

		case 3:
			stm32_configgpio(J12_PIN3);
			stm32_gpiowrite(J12_PIN3, state);
			break;

		default:
			print_usage_con_gpio_test();
			return -1;
		}

		break;

	// I2C DISPLAY / SPARE SENSOR CONNECTOR
	case 13:
		switch (pin) {
		case 3:
			stm32_configgpio(J13_PIN3);
			stm32_gpiowrite(J13_PIN3, state);
			break;

		case 4:
			stm32_configgpio(J13_PIN4);
			stm32_gpiowrite(J13_PIN4, state);
			break;

		case 5:
			stm32_configgpio(J13_PIN5);
			stm32_gpiowrite(J13_PIN5, state);
			break;

		default:
			print_usage_con_gpio_test();
			return -1;
		}

		break;
	}

	printf("GPIO - Con: %d, Pin: %d, State: %d\n", con, pin, state);
	return OK;
}


static bool test_pair(uint32_t output_pin, uint32_t input_pin)
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

static int buzz_test(eHW_TYPE hw_type)
{
	PX4_INFO("test: buzz");
	usleep(1000 * 100 * 10);

	if (hw_type == eM0018) {
		PX4_INFO("Using Flight Core Config");

	} else if (hw_type == eM0019) {
		PX4_INFO("Using VOXL-Flight Config");

	} else if (hw_type == eM0051) {
		PX4_INFO("Using M0051 Config");

	} else {
		return -1;
	}

	if (hw_type == eM0018) {
		PX4_INFO(">> Testing J1");
		stm32_configgpio(J1_PIN2_IN); // 2 [in] to 4 [out]
		stm32_configgpio(J1_PIN3); // 3 [out] to 6 [in]
		stm32_configgpio(J1_PIN4); // 4 [out] to 2 [in]
		stm32_configgpio(J1_PIN6_IN); // 6 [in] to 3 [out]

		if (test_pair(J1_PIN4, J1_PIN2_IN)) {
			PX4_INFO("PASS: J1P4-J1P2");

		} else {
			PX4_ERR("FAIL: J1P4-J1P2 ----------------------------------------");
		}

		if (test_pair(J1_PIN3, J1_PIN6_IN)) {
			PX4_INFO("PASS: J1P3-J1P6");

		} else {
			PX4_ERR("FAIL: J1P3-J1P6 ----------------------------------------");
		}

	} else if (hw_type == eM0019) {
		// NA on VOXL-Flight (internally routed)
	}

	if (hw_type == eM0018) {
		PX4_INFO(">> Testing J4");
		stm32_configgpio(J4_PIN2);    // 2 [out] 6 [in]
		stm32_configgpio(J4_PIN3);    // 3 [out] 7 [in]
		stm32_configgpio(J4_PIN4);    // 4 [out] 8 [in]
		stm32_configgpio(J4_PIN6_IN); // 2 [out] 6 [in]
		stm32_configgpio(J4_PIN7_IN); // 3 [out] 7 [in]
		stm32_configgpio(J4_PIN8_IN); // 4 [out] 8 [in]

		if (test_pair(J4_PIN2, J4_PIN6_IN)) {
			PX4_INFO("PASS: J4P2-J4P6");

		} else {
			PX4_ERR("FAIL: J4P2-J4P6 ----------------------------------------");
		}

		if (test_pair(J4_PIN3, J4_PIN7_IN)) {
			PX4_INFO("PASS: J4P3-J4P7");

		} else {
			PX4_ERR("FAIL: J4P3-J4P7 ----------------------------------------");
		}

		if (test_pair(J4_PIN4, J4_PIN8_IN)) {
			PX4_INFO("PASS: J4P4-J4P8");

		} else {
			PX4_ERR("FAIL: J4P4-J4P8 ----------------------------------------");
		}

	} else if (hw_type == eM0019) {
		PX4_INFO(">> Testing J1002");
		stm32_configgpio(J1002_PIN2);    // 2 [out] 4 [in]
		stm32_configgpio(J1002_PIN3);    // 3 [out] 6 [in]
		stm32_configgpio(J1002_PIN4_IN); // 2 [out] 4 [in]
		stm32_configgpio(J1002_PIN6_IN); // 3 [out] 6 [in]

		if (test_pair(J1002_PIN2, J1002_PIN4_IN)) {
			PX4_INFO("PASS: J1002P2-J1002P4");

		} else {
			PX4_ERR("FAIL: J1002P2-J1002P4 ----------------------------------------");
		}

		if (test_pair(J1002_PIN3, J1002_PIN6_IN)) {
			PX4_INFO("PASS: J1002P3-J1002P6");

		} else {
			PX4_ERR("FAIL: J1002P3-J1002P6 ----------------------------------------");
		}

	}

	if (hw_type == eM0018) {
		PX4_INFO(">> Testing J5");
		stm32_configgpio(J5_PIN2);    // 2 [out] 4 [in]
		stm32_configgpio(J5_PIN3);    // 3 [out] 5 [in]
		stm32_configgpio(J5_PIN4_IN); // 4 [in] 2 [out]
		stm32_configgpio(J5_PIN5_IN); // 5 [in] 3 [out]

		if (test_pair(J5_PIN2, J5_PIN4_IN)) {
			PX4_INFO("PASS: J5P2-J5P4");

		} else {
			PX4_ERR("FAIL: J5P2-J5P4 ----------------------------------------");
		}

		if (test_pair(J5_PIN3, J5_PIN5_IN)) {
			PX4_INFO("PASS: J5P3-J5P5");

		} else {
			PX4_ERR("FAIL: J5P3-J5P5 ----------------------------------------");
		}

	} else if (hw_type == eM0019) {
		PX4_INFO(">> Testing J1010");
		stm32_configgpio(J1010_PIN2);    // 2 [out] 4 [in]
		stm32_configgpio(J1010_PIN3);    // 3 [out] 5 [in]
		stm32_configgpio(J1010_PIN4_IN); // 4 [in] 2 [out]
		stm32_configgpio(J1010_PIN5_IN); // 5 [in] 3 [out]

		if (test_pair(J1010_PIN2, J1010_PIN4_IN)) {
			PX4_INFO("PASS: J1010P2-J1010P4");

		} else {
			PX4_ERR("FAIL: J1010P2-J1010P4 ----------------------------------------");
		}

		if (test_pair(J1010_PIN3, J1010_PIN5_IN)) {
			PX4_INFO("PASS: J1010P3-J1010P5");

		} else {
			PX4_ERR("FAIL: J1010P3-J1010P5 ----------------------------------------");
		}

	}

	if (hw_type == eM0018) {
		PX4_INFO(">> Testing J6");
		stm32_configgpio(J6_PIN2);    // 2 [out] 4 [in]
		stm32_configgpio(J6_PIN3);    // 3 [out] 5 [in]
		stm32_configgpio(J6_PIN4_IN); // 4 [in] 2 [out]
		stm32_configgpio(J6_PIN5_IN); // 5 [in] 3 [out]

		if (test_pair(J6_PIN2, J6_PIN4_IN)) {
			PX4_INFO("PASS: J6P2-J6P4");

		} else {
			PX4_ERR("FAIL: J6P2-J6P4 ----------------------------------------");
		}

		if (test_pair(J6_PIN3, J6_PIN5_IN)) {
			PX4_INFO("PASS: J6P3-J6P5");

		} else {
			PX4_ERR("FAIL: J6P3-J6P5 ----------------------------------------");
		}

	} else if (hw_type == eM0019) {
		PX4_INFO(">> Testing J1009");
		stm32_configgpio(J1009_PIN2);    // 2 [out] 4 [in]
		stm32_configgpio(J1009_PIN3);    // 3 [out] 5 [in]
		stm32_configgpio(J1009_PIN4_IN); // 4 [in] 2 [out]
		stm32_configgpio(J1009_PIN5_IN); // 5 [in] 3 [out]

		if (test_pair(J1009_PIN2, J1009_PIN4_IN)) {
			PX4_INFO("PASS: J1009P2-J1009P4");

		} else {
			PX4_ERR("FAIL: J1009P2-J1009P4 ----------------------------------------");
		}

		if (test_pair(J1009_PIN3, J1009_PIN5_IN)) {
			PX4_INFO("PASS: J1009P3-J1009P5");

		} else {
			PX4_ERR("FAIL: J1009P3-J1009P5 ----------------------------------------");
		}
	}


	if (hw_type == eM0018) {
		PX4_INFO(">> Testing J7");
		stm32_configgpio(J7_PIN2);    // 2 [out] 6 [in]
		stm32_configgpio(J7_PIN3);    // 3 [out] 7 [in]
		stm32_configgpio(J7_PIN4);    // 4 [out] 8 [in]
		stm32_configgpio(J7_PIN5);    // 5 [out] 9 [in]
		stm32_configgpio(J7_PIN6_IN); // 6 [in] 2 [out]
		stm32_configgpio(J7_PIN7_IN); // 7 [in] 3 [out]
		stm32_configgpio(J7_PIN8_IN); // 8 [in] 4 [out]
		stm32_configgpio(J7_PIN9_IN); // 9 [in] 5 [out]

		if (test_pair(J7_PIN2, J7_PIN6_IN)) {
			PX4_INFO("PASS: J7P2-J7P6");

		} else {
			PX4_ERR("FAIL: J7P2-J7P6 ----------------------------------------");
		}

		if (test_pair(J7_PIN3, J7_PIN7_IN)) {
			PX4_INFO("PASS: J7P3-J7P7");

		} else {
			PX4_ERR("FAIL: J7P3-J7P7 ----------------------------------------");
		}

		if (test_pair(J7_PIN4, J7_PIN8_IN)) {
			PX4_INFO("PASS: J7P4-J7P8");

		} else {
			PX4_ERR("FAIL: J7P4-J7P8 ----------------------------------------");
		}

		if (test_pair(J7_PIN5, J7_PIN9_IN)) {
			PX4_INFO("PASS: J7P5-J7P9");

		} else {
			PX4_ERR("FAIL: J7P5-J7P9 ----------------------------------------");
		}

	} else if (hw_type == eM0019) {
		PX4_INFO(">> Testing J1007");
		stm32_configgpio(J1007_PIN2);    // 2 [out] 6 [in]
		stm32_configgpio(J1007_PIN3);    // 3 [out] 7 [in]
		stm32_configgpio(J1007_PIN4);    // 4 [out] 8 [in]
		stm32_configgpio(J1007_PIN5);    // 5 [out] 9 [in]
		stm32_configgpio(J1007_PIN6_IN); // 6 [in] 2 [out]
		stm32_configgpio(J1007_PIN7_IN); // 7 [in] 3 [out]
		stm32_configgpio(J1007_PIN8_IN); // 8 [in] 4 [out]
		stm32_configgpio(J1007_PIN9_IN); // 9 [in] 5 [out]

		if (test_pair(J1007_PIN2, J1007_PIN6_IN)) {
			PX4_INFO("PASS: J1007P2-J1007P6");

		} else {
			PX4_ERR("FAIL: J1007P2-J1007P6 ----------------------------------------");
		}

		if (test_pair(J1007_PIN3, J1007_PIN7_IN)) {
			PX4_INFO("PASS: J1007P3-J1007P7");

		} else {
			PX4_ERR("FAIL: J1007P3-J1007P7 ----------------------------------------");
		}

		if (test_pair(J1007_PIN4, J1007_PIN8_IN)) {
			PX4_INFO("PASS: J1007P4-J1007P8");

		} else {
			PX4_ERR("FAIL: J1007P4-J1007P8 ----------------------------------------");
		}

		if (test_pair(J1007_PIN5, J1007_PIN9_IN)) {
			PX4_INFO("PASS: J1007P5-J1007P9");

		} else {
			PX4_ERR("FAIL: J1007P5-J1007P9 ----------------------------------------");
		}

	} else if (hw_type == eM0051) {
		PX4_INFO(">> Testing M0051 J13");
		stm32_configgpio(M0051J13_PIN2);    // 2 [out] 6 [in]
		stm32_configgpio(M0051J13_PIN3);    // 3 [out] 7 [in]
		stm32_configgpio(M0051J13_PIN4);    // 4 [out] 8 [in]
		stm32_configgpio(M0051J13_PIN5);    // 5 [out] 9 [in]
		stm32_configgpio(M0051J13_PIN6_IN); // 6 [in] 2 [out]
		stm32_configgpio(M0051J13_PIN7_IN); // 7 [in] 3 [out]
		stm32_configgpio(M0051J13_PIN8_IN); // 8 [in] 4 [out]
		stm32_configgpio(M0051J13_PIN9_IN); // 9 [in] 5 [out]

		if (test_pair(M0051J13_PIN2, M0051J13_PIN6_IN)) {
			PX4_INFO("PASS: J13_P2-J13_P6");

		} else {
			PX4_ERR("FAIL: J13_P2-J13_P6 ----------------------------------------");
		}

		if (test_pair(M0051J13_PIN3, M0051J13_PIN7_IN)) {
			PX4_INFO("PASS: JJ13_P3-J13_P7");

		} else {
			PX4_ERR("FAIL: J13_P3-J13_7P7 ----------------------------------------");
		}

		if (test_pair(M0051J13_PIN4, M0051J13_PIN8_IN)) {
			PX4_INFO("PASS: J13_P4-J13_P8");

		} else {
			PX4_ERR("FAIL: J13_P4-J13_P8 ----------------------------------------");
		}

		if (test_pair(M0051J13_PIN5, M0051J13_PIN9_IN)) {
			PX4_INFO("PASS: J13_P5-J13_P9");

		} else {
			PX4_ERR("FAIL: J13_P5-J13_P9 ----------------------------------------");
		}
	}

	if (hw_type == eM0018) {
		PX4_INFO(">> Testing J10");
		stm32_configgpio(J10_PIN2);    // 2 [out] 4 [in]
		stm32_configgpio(J10_PIN3);    // 3 [out] 5 [in]
		stm32_configgpio(J10_PIN4_IN); // 4 [in] 2 [out]
		stm32_configgpio(J10_PIN5_IN); // 5 [in] 3 [out]

		if (test_pair(J10_PIN2, J10_PIN4_IN)) {
			PX4_INFO("PASS: J10P2-J10P4");

		} else {
			PX4_ERR("FAIL: J10P2-J10P4 --------------------------------------");
		}

		if (test_pair(J10_PIN3, J10_PIN5_IN)) {
			PX4_INFO("PASS: J10P3-J10P5");

		} else {
			PX4_ERR("FAIL: J10P3-J10P5 --------------------------------------");
		}

	} else if (hw_type == eM0019) {
		PX4_INFO(">> Testing J1012");
		stm32_configgpio(J1012_PIN2);    // 2 [out] 4 [in]
		stm32_configgpio(J1012_PIN3);    // 3 [out] 5 [in]
		stm32_configgpio(J1012_PIN4_IN); // 4 [in] 2 [out]
		stm32_configgpio(J1012_PIN5_IN); // 5 [in] 3 [out]

		if (test_pair(J1012_PIN2, J1012_PIN4_IN)) {
			PX4_INFO("PASS: J1012P2-J1120P4");

		} else {
			PX4_ERR("FAIL: J1012P2-J1012P4 --------------------------------------");
		}

		if (test_pair(J1012_PIN3, J1012_PIN5_IN)) {
			PX4_INFO("PASS: J1012P3-J1012P5");

		} else {
			PX4_ERR("FAIL: J1012P3-J1012P5 --------------------------------------");
		}

	} else if (hw_type == eM0051) {
		PX4_INFO(">> Testing M0051 J15");
		stm32_configgpio(M0051J15_PIN2);    // 2 [out] 4 [in]
		stm32_configgpio(M0051J15_PIN3);    // 3 [out] 5 [in]
		stm32_configgpio(M0051J15_PIN4_IN); // 4 [in] 2 [out]
		stm32_configgpio(M0051J15_PIN5_IN); // 5 [in] 3 [out]

		if (test_pair(M0051J15_PIN2, M0051J15_PIN4_IN)) {
			PX4_INFO("PASS: J15_P2-J15_P4");

		} else {
			PX4_ERR("FAIL: J15_P2-JJ15_P4 --------------------------------------");
		}

		if (test_pair(M0051J15_PIN3, M0051J15_PIN5_IN)) {
			PX4_INFO("PASS: J15_P3-J15_P5");

		} else {
			PX4_ERR("FAIL: J15_P3-J15_P5 --------------------------------------");
		}

	}

	if (hw_type == eM0018) {
		PX4_INFO(">> Testing J9/J12/J13");
		stm32_configgpio(J9_PIN2_IN);  // J9-2 [in] J13-5 [out]

		stm32_configgpio(J12_PIN2_IN); // J12-2 [in] J13-3 [out]
		stm32_configgpio(J12_PIN3_IN); // J12-3 [in] J13-4 [out]

		stm32_configgpio(J13_PIN3);    // J13-3 [out] J12-2 [in]
		stm32_configgpio(J13_PIN4);    // J13-4 [out] J12-3 [in]
		stm32_configgpio(J13_PIN5);    // J13-5 [out] J9-2 [in]

		if (test_pair(J13_PIN3, J12_PIN2_IN)) {
			PX4_INFO("PASS: J13P3-J12P2");

		} else {
			PX4_ERR("FAIL: J13P3-J12P2 --------------------------------------");
		}

		if (test_pair(J13_PIN4, J12_PIN3_IN)) {
			PX4_INFO("PASS: J13P4-J12P3");

		} else {
			PX4_ERR("FAIL: J13P4-J12P3 --------------------------------------");
		}

		if (test_pair(J13_PIN5, J9_PIN2_IN)) {
			PX4_INFO("PASS: J13P5-J9P2");

		} else {
			PX4_ERR("FAIL: J13P5-J9P2 --------------------------------------");
		}

	} else if (hw_type == eM0019) {
		PX4_INFO(">> Testing J1003/J1004/J1011");
		stm32_configgpio(J1003_PIN2_IN);  // J1003-2 [in] J13-5 [out]

		stm32_configgpio(J1004_PIN2_IN); // J1004-2 [in] J13-3 [out]
		stm32_configgpio(J1004_PIN3_IN); // J1004-3 [in] J13-4 [out]

		stm32_configgpio(J1011_PIN3);    // J1011-3 [out] J12-2 [in]
		stm32_configgpio(J1011_PIN4);    // J1011-4 [out] J12-3 [in]
		stm32_configgpio(J1011_PIN5);    // J1011-5 [out] J9-2 [in]

		if (test_pair(J1011_PIN3, J1004_PIN2_IN)) {
			PX4_INFO("PASS: J1011P3-J1004P2");

		} else {
			PX4_ERR("FAIL: J1011P3-J1004P2 --------------------------------------");
		}

		if (test_pair(J1011_PIN4, J1004_PIN3_IN)) {
			PX4_INFO("PASS: J1011P4-J1004P3");

		} else {
			PX4_ERR("FAIL: J1011P4-J1004P3 --------------------------------------");
		}

		if (test_pair(J1011_PIN5, J1003_PIN2_IN)) {
			PX4_INFO("PASS: J1011P5-J1011P5");

		} else {
			PX4_ERR("FAIL: J1011P5-J1011P5 --------------------------------------");
		}

	} else if (hw_type == eM0051) {
		PX4_INFO(">> Testing M0051 J14");
		stm32_configgpio(M0051J14_PIN2);  // J14-2 [out] J14-3 [in]
		stm32_configgpio(M0051J14_PIN3_IN); // J14-3 [in] J14-2 [out]

		if (test_pair(M0051J14_PIN2, M0051J14_PIN3_IN)) {
			PX4_INFO("PASS: J14_P2-J14_P3");

		} else {
			PX4_ERR("FAIL: J14_P2-J14_P3 --------------------------------------");
		}

	}

	return 0;
}


int modalai_main(int argc, char *argv[])
{
	int hw_rev = board_get_hw_revision();
	int hw_ver = board_get_hw_version();
	eHW_TYPE hw_type = eHwNone;

	if (hw_rev == 6 && hw_ver == 0) {
		hw_type = eM0018;

	} else if (hw_rev == 0 && hw_ver == 1) {
		hw_type = eM0019;

	} else if (hw_rev == 0 && hw_ver == 2) {
		hw_type = eM0051;

	} else {
		return -1;

	}

	if (argc <= 1) {
		print_usage();
		return 1;
	}

	if (!strcmp(argv[1], "led")) {
		return led_test();

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

		return con_gpio_test(con, pin, state);

	} else if (!strcmp(argv[1], "buzz")) {
		return buzz_test(hw_type);

	} else if (!strcmp(argv[1], "detect")) {
		if (hw_type == eM0018) {
			PX4_INFO("V106 - Flight Core");

		} else if (hw_type == eM0019) {
			PX4_INFO("V110 - VOXL-Flight");

		} else if (hw_type == eM0051) {
			PX4_INFO("V120 - M0051");

		}  else {
			PX4_ERR("Unknown hardware");
			return -1;
		}

		return 0;
	}

	print_usage();
	return -EINVAL;
}
