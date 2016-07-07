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

class CameraInterfacePWM : public CameraInterface
{
public:
	CameraInterfacePWM();
	virtual ~CameraInterfacePWM();

	void trigger(bool enable);
	void keep_alive(bool signal_on);

	int powerOn();
	int powerOff();

	void info();

	int _pins[6];
private:
	void setup();

	param_t _p_pin;
	bool _camera_is_on;

};
