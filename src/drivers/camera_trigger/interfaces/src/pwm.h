/**
 * @file pwm.h
 *
 * Interface with cameras via pwm.
 *
 */
#pragma once

#ifdef __PX4_NUTTX

#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <px4_platform_common/log.h>

#include "camera_interface.h"

class CameraInterfacePWM : public CameraInterface
{
public:
	CameraInterfacePWM();
	virtual ~CameraInterfacePWM();

	void trigger(bool trigger_on_true);

	void info();

private:
	int32_t _pwm_camera_shoot = 0;
	int32_t _pwm_camera_neutral = 0;
	void setup();

};

#endif /* ifdef __PX4_NUTTX */
