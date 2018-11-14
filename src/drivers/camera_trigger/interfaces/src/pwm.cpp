#ifdef __PX4_NUTTX

#include <sys/ioctl.h>
#include <lib/mathlib/mathlib.h>
#include <parameters/param.h>


#include "drivers/drv_pwm_trigger.h"
#include "pwm.h"


// TODO : make these parameters later
//#define PWM_CAMERA_SHOOT 1900
//#define PWM_CAMERA_NEUTRAL 1500

int32_t pwm_camera_shoot = 0;
param_get(param_find("TRIG_PWM_SHOOT"), &pwm_camera_shoot);
int32_t pwm_camera_neutral = 0;
param_get(param_find("TRIG_PWM_NEUTRAL"), &pwm_camera_neutral);


CameraInterfacePWM::CameraInterfacePWM():
	CameraInterface()
{
	get_pins();
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
			up_pwm_trigger_set(_pins[i], math::constrain(pwm_camera_neutral, 0, 2000));
		}
	}

}

void CameraInterfacePWM::trigger(bool trigger_on_true)
{
	for (unsigned i = 0; i < arraySize(_pins); i++) {
		if (_pins[i] >= 0) {
			// Set all valid pins to shoot or neutral levels
			up_pwm_trigger_set(_pins[i], math::constrain(trigger_on_true ? pwm_camera_shoot : pwm_camera_neutral, 0, 2000));
		}
	}
}

void CameraInterfacePWM::info()
{
	PX4_INFO("PWM trigger mode (generic), pins enabled : [%d][%d][%d][%d][%d][%d]",
		 _pins[5], _pins[4], _pins[3], _pins[2], _pins[1], _pins[0]);
}

#endif /* ifdef __PX4_NUTTX */
