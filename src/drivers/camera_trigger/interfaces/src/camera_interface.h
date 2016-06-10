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
