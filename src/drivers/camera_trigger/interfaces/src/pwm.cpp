#include <px4.h>
#include <sys/ioctl.h>
#include <lib/mathlib/mathlib.h>

#include <uORB/topics/trigger_pwm.h>
#include "drivers/drv_pwm_output.h"
#include "pwm.h"

// PWM levels of the interface to seagull MAP converter to
// Multiport (http://www.seagulluav.com/manuals/Seagull_MAP2-Manual.pdf)
#define PWM_CAMERA_DISARMED			900
#define PWM_CAMERA_ON				1100
#define PWM_CAMERA_AUTOFOCUS_SHOOT	1300
#define PWM_CAMERA_NEUTRAL			1500
#define PWM_CAMERA_INSTANT_SHOOT	1700
#define PWM_CAMERA_OFF				1900
#define PWM_2_CAMERA_KEEP_ALIVE		1700
#define PWM_2_CAMERA_ON_OFF			1900

CameraInterfacePWM::CameraInterfacePWM():
	CameraInterface(),
	_pwm_pub {nullptr, nullptr},
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

		// Wingtra safety check, pins can only be on 5 or 6!!!
		if (_pins[i] != 4 && _pins[i] != 5) {
			_pins[i] = -1;
		}

		pin_list /= 10;
		i++;
	}

	_orb_id[0] = 0;
	_orb_id[1] = 1;

	setup();
}

CameraInterfacePWM::~CameraInterfacePWM()
{
	// Deinitialise pwm channels
	// Can currently not be used, because up_pwm_servo_deinit() will deinitialise all pwm channels
	// up_pwm_servo_deinit();
}

void CameraInterfacePWM::setup()
{
	for (unsigned i = 0; i < sizeof(_pins) / sizeof(_pins[0]); i = i + 2) {
		if (_pins[i] >= 0 && _pins[i + 1] >= 0) {
			uint8_t pin_bitmask = (1 << _pins[i + 1]) | (1 << _pins[i]);
			up_pwm_servo_init(pin_bitmask);
			up_pwm_servo_set(_pins[i + 1], math::constrain(PWM_CAMERA_DISARMED, PWM_CAMERA_DISARMED, 2000));
			up_pwm_servo_set(_pins[i], math::constrain(PWM_CAMERA_DISARMED, PWM_CAMERA_DISARMED, 2000));

			trigger_pwm_s pwm;
			pwm.timestamp = hrt_absolute_time();
			pwm.pwm = PWM_CAMERA_DISARMED;
			orb_publish_auto(ORB_ID(trigger_pwm), &(_pwm_pub[1]), &pwm, &(_orb_id[1]), ORB_PRIO_DEFAULT);
			pwm.pwm = PWM_CAMERA_DISARMED;
			orb_publish_auto(ORB_ID(trigger_pwm), &(_pwm_pub[0]), &pwm, &(_orb_id[0]), ORB_PRIO_DEFAULT);
		}
	}
}

void CameraInterfacePWM::trigger(bool enable)
{
	// This only starts working upon prearming

	if (!_camera_is_on) {
		return;
	}


	trigger_pwm_s pwm;
	pwm.timestamp = hrt_absolute_time();

	if (enable) {
		// Set all valid pins to shoot level
		for (unsigned i = 0; i < sizeof(_pins) / sizeof(_pins[0]); i = i + 2) {
			if (_pins[i] >= 0 && _pins[i + 1] >= 0) {
				up_pwm_servo_set(_pins[i + 1], math::constrain(PWM_CAMERA_INSTANT_SHOOT, 1000, 2000));

				pwm.pwm = PWM_CAMERA_INSTANT_SHOOT;
				orb_publish_auto(ORB_ID(trigger_pwm), &(_pwm_pub[1]), &pwm, &(_orb_id[1]), ORB_PRIO_DEFAULT);
			}
		}

	} else {
		// Set all valid pins back to neutral level
		for (unsigned i = 0; i < sizeof(_pins) / sizeof(_pins[0]); i = i + 2) {
			if (_pins[i] >= 0 && _pins[i + 1] >= 0) {
				up_pwm_servo_set(_pins[i + 1], math::constrain(PWM_CAMERA_NEUTRAL, 1000, 2000));

				pwm.pwm = PWM_CAMERA_NEUTRAL;
				orb_publish_auto(ORB_ID(trigger_pwm), &(_pwm_pub[1]), &pwm, &(_orb_id[1]), ORB_PRIO_DEFAULT);
			}
		}
	}
}

void CameraInterfacePWM::keep_alive(bool signal_on)
{
	// This should alternate between signal_on and !signal_on to keep the camera alive

	if (!_camera_is_on) {
		return;
	}

	trigger_pwm_s pwm;
	pwm.timestamp = hrt_absolute_time();

	if (signal_on) {
		// Set channel 2 pin to keep_alive signal
		for (unsigned i = 0; i < sizeof(_pins) / sizeof(_pins[0]); i = i + 2) {
			if (_pins[i] >= 0 && _pins[i + 1] >= 0) {
				up_pwm_servo_set(_pins[i], math::constrain(PWM_2_CAMERA_KEEP_ALIVE, 1000, 2000));

				pwm.pwm = PWM_2_CAMERA_KEEP_ALIVE;
				orb_publish_auto(ORB_ID(trigger_pwm), &(_pwm_pub[0]), &pwm, &(_orb_id[0]), ORB_PRIO_DEFAULT);
			}
		}

	} else {
		// Set channel 2 pin to neutral signal
		for (unsigned i = 0; i < sizeof(_pins) / sizeof(_pins[0]); i = i + 2) {
			if (_pins[i] >= 0 && _pins[i + 1] >= 0) {
				up_pwm_servo_set(_pins[i], math::constrain(PWM_CAMERA_NEUTRAL, 1000, 2000));

				pwm.pwm = PWM_CAMERA_NEUTRAL;
				orb_publish_auto(ORB_ID(trigger_pwm), &(_pwm_pub[0]), &pwm, &(_orb_id[0]), ORB_PRIO_DEFAULT);
			}
		}
	}
}

void CameraInterfacePWM::turn_on_off(bool enable)
{

	trigger_pwm_s pwm;
	pwm.timestamp = hrt_absolute_time();

	if (enable) {
		// For now, set channel one on neutral upon startup.
		for (unsigned i = 0; i < sizeof(_pins) / sizeof(_pins[0]); i = i + 2) {
			if (_pins[i] >= 0 && _pins[i + 1] >= 0) {
				up_pwm_servo_set(_pins[i + 1], math::constrain(PWM_CAMERA_NEUTRAL, 1000, 2000));
				up_pwm_servo_set(_pins[i], math::constrain(PWM_2_CAMERA_ON_OFF, 1000, 2000));

				pwm.pwm = PWM_CAMERA_NEUTRAL;
				orb_publish_auto(ORB_ID(trigger_pwm), &(_pwm_pub[1]), &pwm, &(_orb_id[1]), ORB_PRIO_DEFAULT);
				pwm.pwm = PWM_2_CAMERA_ON_OFF;
				orb_publish_auto(ORB_ID(trigger_pwm), &(_pwm_pub[0]), &pwm, &(_orb_id[0]), ORB_PRIO_DEFAULT);
			}
		}

	} else {
		// For now, set channel one on neutral upon startup.
		for (unsigned i = 0; i < sizeof(_pins) / sizeof(_pins[0]); i = i + 2) {
			if (_pins[i] >= 0 && _pins[i + 1] >= 0) {
				up_pwm_servo_set(_pins[i + 1], math::constrain(PWM_CAMERA_NEUTRAL, 1000, 2000));
				up_pwm_servo_set(_pins[i], math::constrain(PWM_CAMERA_NEUTRAL, 1000, 2000));

				pwm.pwm = PWM_CAMERA_NEUTRAL;
				orb_publish_auto(ORB_ID(trigger_pwm), &(_pwm_pub[1]), &pwm, &(_orb_id[1]), ORB_PRIO_DEFAULT);
				pwm.pwm = PWM_CAMERA_NEUTRAL;
				orb_publish_auto(ORB_ID(trigger_pwm), &(_pwm_pub[0]), &pwm, &(_orb_id[0]), ORB_PRIO_DEFAULT);
			}
		}

		_camera_is_on = !_camera_is_on;
	}
}

void CameraInterfacePWM::info()
{
	warnx("PWM - interface, pin config: %d,%d,%d", _pins[0] + 1, _pins[1] + 1, _pins[2] + 1);
}
