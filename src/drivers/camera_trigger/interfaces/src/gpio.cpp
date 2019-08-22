#ifdef __PX4_NUTTX

#include "gpio.h"
#include <cstring>

constexpr uint32_t CameraInterfaceGPIO::_gpios[ngpios];

CameraInterfaceGPIO::CameraInterfaceGPIO():
	CameraInterface(),
	_trigger_invert(false),
	_triggers{0}
{
	_p_polarity = param_find("TRIG_POLARITY");

	// polarity of the trigger (0 = active low, 1 = active high )
	int32_t polarity;
	param_get(_p_polarity, &polarity);
	_trigger_invert = (polarity == 0);

	get_pins();
	setup();
}

void CameraInterfaceGPIO::setup()
{
	for (unsigned i = 0, t = 0; i < arraySize(_pins); i++) {

		// Pin range is from 1 to 5 or 6, indexes are 0 to 4 or 5

		if (_pins[i] >= 0 && _pins[i] < (int)arraySize(_gpios)) {
			uint32_t gpio = _gpios[_pins[i]];
			_triggers[t++] = gpio;
			px4_arch_configgpio(gpio);
			px4_arch_gpiowrite(gpio, false ^ _trigger_invert);
		}
	}
}

void CameraInterfaceGPIO::trigger(bool trigger_on_true)
{
	bool trigger_state = trigger_on_true ^ _trigger_invert;

	for (unsigned i = 0; i < arraySize(_triggers); i++) {

		if (_triggers[i] == 0) { break; }

		px4_arch_gpiowrite(_triggers[i], trigger_state);
	}
}

void CameraInterfaceGPIO::info()
{
	if (ngpios == 6) {
		PX4_INFO("GPIO trigger mode, pins enabled : [%d][%d][%d][%d][%d][%d], polarity : %s",
			 _pins[5], _pins[4], _pins[3], _pins[2], _pins[1], _pins[0],
			 _trigger_invert ? "ACTIVE_LOW" : "ACTIVE_HIGH");
	}

	if (ngpios == 5) {
		PX4_INFO("GPIO trigger mode, pins enabled : [%d][%d][%d][%d][%d], polarity : %s",
			 _pins[4], _pins[3], _pins[2], _pins[1], _pins[0],
			 _trigger_invert ? "ACTIVE_LOW" : "ACTIVE_HIGH");
	}
}

#endif /* ifdef __PX4_NUTTX */
