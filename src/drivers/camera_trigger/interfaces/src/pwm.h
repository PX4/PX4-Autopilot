/**
 * @file pwm.h
 *
 * Interface with cameras via pwm.
 *
 */
#pragma once

#include "camera_interface.h"


class CameraInterfacePWM : public CameraInterface
{
public:
	CameraInterfacePWM();
	virtual ~CameraInterfacePWM();

	void setup();

	void trigger(bool enable);

	int powerOn();
	int powerOff();

private:

};
