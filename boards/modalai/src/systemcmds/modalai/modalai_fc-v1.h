#ifndef MODALAI_FC_V1_H_
#define MODALAI_FC_V1_H_

typedef enum {
	eHwUnknown = -1,
	eHwNone = 0,
	eM0018,           // Flight Core
	eM0019,           // VOXL Flight
	eM0051
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


void 	modalai_print_usage_v1(void);
void 	modalai_print_usage_con_gpio_test_v1(void);
int  	modalai_con_gpio_test_v1(uint8_t con, uint8_t pin, bool state);
int  	modalai_led_test_v1(void);
int  	modalai_buzz_test_v1(eHW_TYPE type);
int	modalai_hw_detect_v1(eHW_TYPE type);

#endif //MODALAI_FC_V1_H_
