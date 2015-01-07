#include "LandDetector.h"
#include <unistd.h>                 //usleep
#include <drivers/drv_hrt.h>

LandDetector::LandDetector() :
	_landDetectedPub(-1),
	_landDetected({0, false}),
	_taskShouldExit(false),
	_taskIsRunning(false)
{
	//Advertise the first land detected uORB
	_landDetected.timestamp = hrt_absolute_time();
	_landDetected.landed = false;
	_landDetectedPub = orb_advertise(ORB_ID(vehicle_land_detected), &_landDetected);
}

LandDetector::~LandDetector()
{
	_taskShouldExit = true;
	close(_landDetectedPub);
}

void LandDetector::shutdown()
{
	_taskShouldExit = true;
}

void LandDetector::start()
{
	//Make sure this method has not already been called by another thread
	if (isRunning()) {
		return;
	}

	//Task is now running, keep doing so until shutdown() has been called
	_taskIsRunning = true;
	_taskShouldExit = false;

	while (isRunning()) {

		bool landDetected = update();

		//Publish if land detection state has changed
		if (_landDetected.landed != landDetected) {
			_landDetected.timestamp = hrt_absolute_time();
			_landDetected.landed = landDetected;

			/* publish the land detected broadcast */
			orb_publish(ORB_ID(vehicle_land_detected), _landDetectedPub, &_landDetected);
		}

		//Limit loop rate
		usleep(1000000 / LAND_DETECTOR_UPDATE_RATE);
	}

	_taskIsRunning = false;
	_exit(0);
}

bool LandDetector::orb_update(const struct orb_metadata *meta, int handle, void *buffer)
{
	bool newData = false;

	//Check if there is new data to grab
	if (orb_check(handle, &newData) != OK) {
		return false;
	}

	if (!newData) {
		return false;
	}

	if (orb_copy(meta, handle, buffer) != OK) {
		return false;
	}

	return true;
}
