#ifdef __PX4_NUTTX

#include <sys/ioctl.h>
#include <lib/mathlib/mathlib.h>

#include "drivers/drv_pwm_trigger.h"
#include "seagull_map2.h"

// PWM levels of the interface to Seagull MAP 2 converter to
// Multiport (http://www.seagulluav.com/manuals/Seagull_MAP2-Manual.pdf)
#define PWM_CAMERA_DISARMED			900
#define PWM_CAMERA_ON				1100
#define PWM_CAMERA_AUTOFOCUS_SHOOT	1300
#define PWM_CAMERA_NEUTRAL			1500
#define PWM_CAMERA_INSTANT_SHOOT	1700
#define PWM_CAMERA_OFF				1900
#define PWM_2_CAMERA_KEEP_ALIVE		1700
#define PWM_2_CAMERA_ON_OFF			1900

// TODO : cleanup using arraySize() macro

CameraInterfaceSeagull::CameraInterfaceSeagull():
	CameraInterface(),
	_camera_is_on(false)
{
	_p_pin = param_find("TRIG_PINS");
	int pin_list;
	param_get(_p_pin, &pin_list);

	// Set all pins as invalid
	for (unsigned i = 0; i < sizeof(_pins) / sizeof(_pins[0]); i++) {
		_pins[i] = -1;
	}

	// Convert number to individual channels
	unsigned i = 0;
	int single_pin;

	while ((single_pin = pin_list % 10)) {

		_pins[i] = single_pin - 1;

		if (_pins[i] < 0) {
			_pins[i] = -1;
		}

		pin_list /= 10;
		i++;
	}

	setup();
}

CameraInterfaceSeagull::~CameraInterfaceSeagull()
{
	// Deinitialise pwm channels
	up_pwm_trigger_deinit();
}

void CameraInterfaceSeagull::setup()
{
	for (unsigned i = 0; i < sizeof(_pins) / sizeof(_pins[0]); i = i + 2) {
		if (_pins[i] >= 0 && _pins[i + 1] >= 0) {
			uint8_t pin_bitmask = (1 << _pins[i + 1]) | (1 << _pins[i]);
			up_pwm_trigger_init(pin_bitmask);
			up_pwm_trigger_set(_pins[i + 1], math::constrain(PWM_CAMERA_DISARMED, PWM_CAMERA_DISARMED, 2000));
			up_pwm_trigger_set(_pins[i], math::constrain(PWM_CAMERA_DISARMED, PWM_CAMERA_DISARMED, 2000));
		}
	}
}

void CameraInterfaceSeagull::trigger(bool enable)
{

	if (!_camera_is_on) {
		return;
	}

	for (unsigned i = 0; i < sizeof(_pins) / sizeof(_pins[0]); i = i + 2) {
		if (_pins[i] >= 0 && _pins[i + 1] >= 0) {
			// Set all valid pins to shoot or neutral levels
			up_pwm_trigger_set(_pins[i + 1], math::constrain(enable ? PWM_CAMERA_INSTANT_SHOOT : PWM_CAMERA_NEUTRAL, 1000, 2000));
		}
	}
}

void CameraInterfaceSeagull::keep_alive(bool signal_on)
{
	// This should alternate between signal_on and !signal_on to keep the camera alive

	if (!_camera_is_on) {
		return;
	}

	for (unsigned i = 0; i < sizeof(_pins) / sizeof(_pins[0]); i = i + 2) {
		if (_pins[i] >= 0 && _pins[i + 1] >= 0) {
			// Set channel 2 pin to keep_alive or netural signal
			up_pwm_trigger_set(_pins[i], math::constrain(signal_on ? PWM_2_CAMERA_KEEP_ALIVE : PWM_CAMERA_NEUTRAL, 1000, 2000));
		}
	}
}

void CameraInterfaceSeagull::turn_on_off(bool enable)
{

	for (unsigned i = 0; i < sizeof(_pins) / sizeof(_pins[0]); i = i + 2) {
		if (_pins[i] >= 0 && _pins[i + 1] >= 0) {
			// For now, set channel one to neutral upon startup.
			up_pwm_trigger_set(_pins[i + 1], math::constrain(PWM_CAMERA_NEUTRAL, 1000, 2000));
			up_pwm_trigger_set(_pins[i], math::constrain(enable ? PWM_2_CAMERA_ON_OFF : PWM_CAMERA_NEUTRAL, 1000, 2000));
		}
	}

	if (!enable) { _camera_is_on = !_camera_is_on; }
}

void CameraInterfaceSeagull::info()
{
	PX4_INFO("PWM trigger mode (Seagull MAP2) , pins enabled : [%d][%d][%d][%d][%d][%d]",
		 _pins[5], _pins[4], _pins[3], _pins[2], _pins[1], _pins[0]);
}

#endif /* ifdef __PX4_NUTTX */
