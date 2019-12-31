/**
 * @file camera_interface.h
 */

#pragma once

#include <parameters/param.h>
#include <px4_platform_common/log.h>

#define arraySize(a) (sizeof((a))/sizeof(((a)[0])))

class CameraInterface
{
public:

	/**
	 * Constructor
	 */
	CameraInterface();

	/**
	 * Destructor.
	 */
	virtual ~CameraInterface() = default;

	/**
	 * trigger the camera
	 * @param enable
	 */
	virtual void trigger(bool trigger_on_true) {}

	/**
	 * send command to turn the camera on/off
	 * @param enable
	 */
	virtual void send_toggle_power(bool enable) {}

	/**
	 * send command to prevent the camera from sleeping
	 * @param enable
	 */
	virtual void send_keep_alive(bool enable) {}

	/**
	 * Display info.
	 */
	virtual void info() {}

	/**
	 * Checks if the interface has support for
	 * camera power control
	 * @return true if power control is supported
	 */
	virtual bool has_power_control() { return false; }

	/**
	 * Checks if the camera connected to the interface
	 * is turned on.
	 * @return true if camera is on
	 */
	virtual bool is_powered_on() { return true; }

protected:

	/**
	 * setup the interface
	 */
	virtual void setup() {}

	/**
	 * get the hardware configuration
	 */
	void get_pins();

	param_t _p_pin;

	int _pins[6];

};
