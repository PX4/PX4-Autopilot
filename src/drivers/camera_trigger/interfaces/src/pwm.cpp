#ifdef __PX4_NUTTX

#include <sys/ioctl.h>
#include <lib/mathlib/mathlib.h>

#include "drivers/drv_pwm_trigger.h"
#include "pwm.h"

// TODO : make these parameters later
#define PWM_CAMERA_SHOOT 1900
#define PWM_CAMERA_NEUTRAL 1500

CameraInterfacePWM::CameraInterfacePWM():
	CameraInterface()
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

CameraInterfacePWM::~CameraInterfacePWM()
{
	// Deinitialise trigger channels
	up_pwm_trigger_deinit();
}

void CameraInterfacePWM::setup()
{
	// Precompute the bitmask for enabled channels
	uint8_t pin_bitmask = 0;

	for (unsigned i = 0; i < arraySize(_pins); i++) {
		if (_pins[i] >= 0) {
			pin_bitmask |= (1 << _pins[i]);
		}
	}

	// Initialize and arm channels
	up_pwm_trigger_init(pin_bitmask);

	// Set neutral pulsewidths
	for (unsigned i = 0; i < arraySize(_pins); i++) {
		if (_pins[i] >= 0) {
			up_pwm_trigger_set(_pins[i], math::constrain(PWM_CAMERA_NEUTRAL, PWM_CAMERA_NEUTRAL, 2000));
		}
	}

}

void CameraInterfacePWM::trigger(bool enable)
{
	for (unsigned i = 0; i < arraySize(_pins); i++) {
		if (_pins[i] >= 0) {
			// Set all valid pins to shoot or neutral levels
			up_pwm_trigger_set(_pins[i], math::constrain(enable ? PWM_CAMERA_SHOOT : PWM_CAMERA_NEUTRAL, 1000, 2000));
		}
	}
}

void CameraInterfacePWM::info()
{
	// TODO : cleanup this output for all trigger types
	PX4_INFO("PWM trigger mode (generic), pins enabled : [%d][%d][%d][%d][%d][%d]",
		 _pins[5], _pins[4], _pins[3], _pins[2], _pins[1], _pins[0]);
}

#endif /* ifdef __PX4_NUTTX */
