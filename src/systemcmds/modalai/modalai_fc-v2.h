#ifndef MODALAI_FC_V2_H_
#define MODALAI_FC_V2_H_

typedef enum {
	eHwUnknown = -1,
	eHwNone = 0,
	eM0079
} eHW_TYPE;

#define _MK_GPIO_INPUT(def)  (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_PULLUP))
#define _MK_GPIO_OUTPUT(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR))

//
// TELEM1
//   M0079- J1
//             PF6   PIN2 - out
//             PE8   PIN3 - in
//	       PF8   PIN4 - out
//	       PE10  PIN4 - in
//
#define M0079_J1_PIN_2_OUT             _MK_GPIO_OUTPUT(GPIO_PORTF|GPIO_PIN6)
#define M0079_J1_PIN_3_IN              _MK_GPIO_INPUT(GPIO_PORTE|GPIO_PIN8)
#define M0079_J1_PIN_4_OUT             _MK_GPIO_OUTPUT(GPIO_PORTF|GPIO_PIN8)
#define M0079_J1_PIN_5_IN              _MK_GPIO_INPUT(GPIO_PORTE|GPIO_PIN10)

//
// TELEM2
//   M0079- J5
//             PC12 PIN2 - out
//             PD2  PIN3 - out
//	       PC9  PIN4 - in
//	       PC8  PIN4 - in
//
#define M0079_J5_PIN_2_OUT             _MK_GPIO_OUTPUT(GPIO_PORTC|GPIO_PIN12)
#define M0079_J5_PIN_3_OUT             _MK_GPIO_OUTPUT(GPIO_PORTD|GPIO_PIN2)
#define M0079_J5_PIN_4_IN              _MK_GPIO_INPUT(GPIO_PORTC|GPIO_PIN9)
#define M0079_J5_PIN_5_IN              _MK_GPIO_INPUT(GPIO_PORTC|GPIO_PIN8)

//
// PWM Output
//   M0079- J7
//             PI0  PIN2 - out
//             PH12 PIN3 - out
//             PH11 PIN4 - out
//             PH10 PIN5 - out
//
//             PD13 PIN6 - in
//             PD14 PIN7 - in
//             PH6  PIN8 - in
//             PH9  PIN9 - in
//
#define M0079_J7_PIN_2_OUT             _MK_GPIO_OUTPUT(GPIO_PORTI|GPIO_PIN0)
#define M0079_J7_PIN_3_OUT             _MK_GPIO_OUTPUT(GPIO_PORTH|GPIO_PIN12)
#define M0079_J7_PIN_4_OUT             _MK_GPIO_OUTPUT(GPIO_PORTH|GPIO_PIN11)
#define M0079_J7_PIN_5_OUT             _MK_GPIO_OUTPUT(GPIO_PORTH|GPIO_PIN10)
#define M0079_J7_PIN_6_IN              _MK_GPIO_INPUT(GPIO_PORTD|GPIO_PIN13)
#define M0079_J7_PIN_7_IN              _MK_GPIO_INPUT(GPIO_PORTD|GPIO_PIN14)
#define M0079_J7_PIN_8_IN              _MK_GPIO_INPUT(GPIO_PORTH|GPIO_PIN6)
#define M0079_J7_PIN_9_IN              _MK_GPIO_INPUT(GPIO_PORTH|GPIO_PIN9)

//
// GPS/Mag
//   M0079- J10
//             PB6 PIN2 - out
//             PB7 PIN3 - out
//	       PB8 PIN4 - in
//	       PB9 PIN4 - in
//
#define M0079_J10_PIN_2_OUT            _MK_GPIO_OUTPUT(GPIO_PORTB|GPIO_PIN6)
#define M0079_J10_PIN_3_OUT            _MK_GPIO_OUTPUT(GPIO_PORTB|GPIO_PIN7)
#define M0079_J10_PIN_4_IN             _MK_GPIO_INPUT(GPIO_PORTB|GPIO_PIN8)
#define M0079_J10_PIN_5_IN             _MK_GPIO_INPUT(GPIO_PORTB|GPIO_PIN9)

//
// Spektrum RC Input Connector
//   M0079- J12
//             PC6 PIN2 - out
//             PC7 PIN3 - in
//
#define M0079_J12_PIN_2_OUT             _MK_GPIO_OUTPUT(GPIO_PORTC|GPIO_PIN6)
#define M0079_J12_PIN_3_IN              _MK_GPIO_INPUT(GPIO_PORTC|GPIO_PIN7)

void	modalai_print_usage_v2(void);
void	modalai_print_usage_con_gpio_test_v2(void);
int 	modalai_con_gpio_test_v2(uint8_t con, uint8_t pin, bool state);
int 	modalai_led_test_v2(void);
int 	modalai_buzz_test_v2(eHW_TYPE type);
int 	modalai_hw_detect_v2(eHW_TYPE type);

#endif //MODALAI_FC_V2_H_
