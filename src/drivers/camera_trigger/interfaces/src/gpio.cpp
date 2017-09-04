#ifdef __PX4_NUTTX

#include "gpio.h"

constexpr uint32_t CameraInterfaceGPIO::_gpios[6];

CameraInterfaceGPIO::CameraInterfaceGPIO():
	CameraInterface(),
	_polarity(0)
{
	_p_polarity = param_find("TRIG_POLARITY");
	param_get(_p_polarity, &_polarity);

	get_pins();
	setup();
}

CameraInterfaceGPIO::~CameraInterfaceGPIO()
{
}

void CameraInterfaceGPIO::setup()
{
	for (unsigned i = 0; i < arraySize(_pins); i++) {
		px4_arch_configgpio(_gpios[_pins[i]]);
		px4_arch_gpiowrite(_gpios[_pins[i]], !_polarity);
	}
}

void CameraInterfaceGPIO::trigger(bool enable)
{
	if (enable) {
		for (unsigned i = 0; i < arraySize(_pins); i++) {
			if (_pins[i] >= 0) {
				// ACTIVE_LOW == 1
				px4_arch_gpiowrite(_gpios[_pins[i]], _polarity);
			}
		}

	} else {
		for (unsigned i = 0; i < arraySize(_pins); i++) {
			if (_pins[i] >= 0) {
				// ACTIVE_LOW == 1
				px4_arch_gpiowrite(_gpios[_pins[i]], !_polarity);
			}
		}
	}
}

void CameraInterfaceGPIO::info()
{
	PX4_INFO("GPIO trigger mode, pins enabled : [%d][%d][%d][%d][%d][%d], polarity : %s",
		 _pins[5], _pins[4], _pins[3], _pins[2], _pins[1], _pins[0],
		 _polarity ? "ACTIVE_HIGH" : "ACTIVE_LOW");
}

#endif /* ifdef __PX4_NUTTX */
