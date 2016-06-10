/**
 * @file pwm.h
 *
 * Interface with cameras via pwm.
 *
 */
#pragma once

#include <systemlib/param/param.h>

#include <uORB/topics/vehicle_status.h>
#include "camera_interface.h"

// TODO(birchera): check if this is the right device and addresses
#define PWM_DEVICE_PATH			"/dev/pwm_output0"
#define PWM_CAMERA_BASE			0x2a00
#define PWM_CAMERA_SET(_pin)	_PX4_IOC(PWM_CAMERA_BASE, 0x30 + _pin)
// PWM levels of the interface to seagull MAP converter to
// Multiport (http://www.seagulluav.com/manuals/Seagull_MAP2-Manual.pdf)
#define PWM_CAMERA_DISARMED			90 // TODO(birchera): check here value
#define PWM_CAMERA_ON				1100
#define PWM_CAMERA_AUTOFOCUS_SHOOT	1300
#define PWM_CAMERA_NEUTRAL			1500
#define PWM_CAMERA_INSTANT_SHOOT	1700
#define PWM_CAMERA_OFF				1900


class CameraInterfacePWM : public CameraInterface
{
public:
	CameraInterfacePWM();
	virtual ~CameraInterfacePWM();

	void setup();

	void trigger(bool enable);

	int powerOn();
	int powerOff();

	int _pins[6];
private:

	int _vehicle_status_sub;
	struct vehicle_status_s _vehicle_status;

	param_t _p_pin;
	const char *_pwm_dev;
	int _pwm_fd;
	bool _camera_is_on;

};
