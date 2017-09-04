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
#include <px4_log.h>

#include "camera_interface.h"

class CameraInterfacePWM : public CameraInterface
{
public:
	CameraInterfacePWM();
	virtual ~CameraInterfacePWM();

	void trigger(bool enable);

	void info();

private:

	void setup();

};

#endif /* ifdef __PX4_NUTTX */
