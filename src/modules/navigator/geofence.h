/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file geofence.h
 * Provides functions for handling the geofence
 *
 * @author Jean Cyr <jean.m.cyr@gmail.com>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#ifndef GEOFENCE_H_
#define GEOFENCE_H_

#include <uORB/topics/fence.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/sensor_combined.h>
#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

#define GEOFENCE_FILENAME "/fs/microsd/etc/geofence.txt"

class Geofence : public control::SuperBlock
{
public:
	Geofence();
	~Geofence();

	/* Altitude mode, corresponding to the param GF_ALTMODE */
	enum {
		GF_ALT_MODE_WGS84 = 0,
		GF_ALT_MODE_AMSL = 1
	};

	/* Source, corresponding to the param GF_SOURCE */
	enum {
		GF_SOURCE_GLOBALPOS = 0,
		GF_SOURCE_GPS = 1
	};

	/**
	 * Return whether system is inside geofence.
	 *
	 * Calculate whether point is inside arbitrary polygon
	 * @param craft pointer craft coordinates
	 * @return true: system is inside fence, false: system is outside fence
	 */
	bool inside(const struct vehicle_global_position_s &global_position,
			const struct vehicle_gps_position_s &gps_position,float baro_altitude_amsl);
	bool inside_polygon(double lat, double lon, float altitude);

	int clearDm();

	bool valid();

	/**
	 * Specify fence vertex position.
	 */
	void addPoint(int argc, char *argv[]);

	void publishFence(unsigned vertices);

	int loadFromFile(const char *filename);

	bool isEmpty() {return _verticesCount == 0;}

	int getAltitudeMode() { return _param_altitude_mode.get(); }

	int getSource() { return _param_source.get(); }

	void setMavlinkFd(int value) { _mavlinkFd = value; }

private:
	orb_advert_t	_fence_pub;			/**< publish fence topic */

	float			_altitude_min;
	float			_altitude_max;

	unsigned 			_verticesCount;

	/* Params */
	control::BlockParamInt _param_geofence_on;
	control::BlockParamInt _param_altitude_mode;
	control::BlockParamInt _param_source;
	control::BlockParamInt _param_counter_threshold;

	uint8_t			_outside_counter;

	int _mavlinkFd;

	bool inside(double lat, double lon, float altitude);
	bool inside(const struct vehicle_global_position_s &global_position);
	bool inside(const struct vehicle_global_position_s &global_position, float baro_altitude_amsl);
};


#endif /* GEOFENCE_H_ */
