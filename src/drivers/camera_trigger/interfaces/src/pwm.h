/**
 * @file pwm.h
 *
 * Interface with cameras via pwm.
 *
 */
#pragma once

#ifdef __PX4_NUTTX

#include <drivers/drv_hrt.h>
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

	void turn_on_off(bool enable);

	void info();

	int _pins[6];
private:
	void setup();

	param_t _p_pin;
	bool _camera_is_on;

};

#endif /* ifdef __PX4_NUTTX */
