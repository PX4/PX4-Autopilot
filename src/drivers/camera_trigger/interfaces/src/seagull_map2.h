/**
 * @file seagull_map2.h
 *
 * Interface supported cameras using a Seagull MAP2 interface.
 *
 */
#pragma once

#ifdef __PX4_NUTTX

#include <drivers/drv_hrt.h>

#include "camera_interface.h"

class CameraInterfaceSeagull : public CameraInterface
{
public:
	CameraInterfaceSeagull();
	virtual ~CameraInterfaceSeagull();

	void trigger(bool enable);
	void keep_alive(bool signal_on);

	void turn_on_off(bool enable);

	void info();

private:
	void setup();

	bool _camera_is_on;

};

#endif /* ifdef __PX4_NUTTX */
