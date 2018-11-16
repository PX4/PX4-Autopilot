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

	void trigger(bool trigger_on_true);

	void send_keep_alive(bool enable);

	void send_toggle_power(bool enable);

	void info();

	bool has_power_control() { return true; }

	bool is_powered_on() { return _camera_is_on; }

private:
	void setup();

	bool _camera_is_on;

};

#endif /* ifdef __PX4_NUTTX */
