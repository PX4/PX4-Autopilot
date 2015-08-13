/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file LandDetector.h
 * Abstract interface for land detector algorithms
 *
 * @author Johan Jansen <jnsn.johan@gmail.com>
 */

#ifndef __LAND_DETECTOR_H__
#define __LAND_DETECTOR_H__

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_land_detected.h>

class LandDetector
{
public:

	LandDetector();
	virtual ~LandDetector();

	/**
	 * @return true if this task is currently running
	 **/
	inline bool isRunning() const {return _taskIsRunning;}

	/**
	 * @return the current landed state
	 */
	bool isLanded() { return _landDetected.landed; }

	/**
	 * @brief  Tells the Land Detector task that it should exit
	 **/
	void shutdown();

	/**
	 * @brief Blocking function that should be called from it's own task thread. This method will
	 *        run the underlying algorithm at the desired update rate and publish if the landing state changes.
	 **/
	void start();

protected:

	/**
	* @brief Pure abstract method that must be overriden by sub-classes. This actually runs the underlying algorithm
	* @return true if a landing was detected and this should be broadcast to the rest of the system
	**/
	virtual bool update() = 0;

	/**
	* @brief Pure abstract method that is called by this class once for initializing the uderlying algorithm (memory allocation,
	*        uORB topic subscription, creating file descriptors, etc.)
	**/
	virtual void initialize() = 0;

	/**
	* @brief Convenience function for polling uORB subscriptions
	* @return true if there was new data and it was successfully copied
	**/
	bool orb_update(const struct orb_metadata *meta, int handle, void *buffer);

	static constexpr uint32_t LAND_DETECTOR_UPDATE_RATE = 50;        /**< Run algorithm at 50Hz */

	static constexpr uint64_t LAND_DETECTOR_TRIGGER_TIME = 2000000;  /**< usec that landing conditions have to hold
                                                                          before triggering a land */
	static constexpr uint64_t LAND_DETECTOR_ARM_PHASE_TIME = 1000000;	/**< time interval in which wider acceptance thresholds are used after arming */

	orb_advert_t				_landDetectedPub;		/**< publisher for position in local frame */
	struct vehicle_land_detected_s		_landDetected;			/**< local vehicle position */
	uint64_t				_arming_time;			/**< timestamp of arming time */

private:
	bool _taskShouldExit;                                               /**< true if it is requested that this task should exit */
	bool _taskIsRunning;                                                /**< task has reached main loop and is currently running */
};

#endif //__LAND_DETECTOR_H__
