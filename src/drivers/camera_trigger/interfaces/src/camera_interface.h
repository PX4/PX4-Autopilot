/**
 * @file camera_interface.h
 */

#pragma once

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
	virtual ~CameraInterface();

	/**
	 * trigger the camera
	 * @param trigger:
	 */
	virtual void trigger(bool enable) {};

	/**
	 * turn on/off the camera
	 * @param enable:
	 */
	virtual void turn_on_off(bool enable) {};

	/**
	 * prevent the camera from sleeping
	 * @param keep alive signal:
	 */
	virtual void keep_alive(bool signal_on) {};

	/**
	 * Display info.
	 */
	virtual void info() {};

	/**
	 * Power on the camera
	 * @return 0 on success, <0 on error
	 */
	virtual int powerOn() { return -1; }

	/**
	 * Power off the camera
	 * @return 0 on success, <0 on error
	 */
	virtual int powerOff() { return -1; }


protected:

	/**
	 * setup the interface
	 */
	virtual void setup() {};

};
