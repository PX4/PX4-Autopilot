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
#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

#define GEOFENCE_FILENAME "/fs/microsd/etc/geofence.txt"

class Geofence : public control::SuperBlock
{
private:
	orb_advert_t	_fence_pub;			/**< publish fence topic */

	float			_altitude_min;
	float			_altitude_max;

	unsigned 			_verticesCount;

	/* Params */
	control::BlockParamInt param_geofence_on;
public:
	Geofence();
	~Geofence();

	/**
	 * Return whether craft is inside geofence.
	 *
	 * Calculate whether point is inside arbitrary polygon
	 * @param craft pointer craft coordinates
	 * @param fence pointer to array of coordinates, one per vertex. First and last vertex are assumed connected
	 * @return true: craft is inside fence, false:craft is outside fence
	 */
	bool inside(const struct vehicle_global_position_s *craft);
	bool inside(double lat, double lon, float altitude);

	int clearDm();

	bool valid();

	/**
	 * Specify fence vertex position.
	 */
	void addPoint(int argc, char *argv[]);

	void publishFence(unsigned vertices);

	int loadFromFile(const char *filename);

	bool isEmpty() {return _verticesCount == 0;}
};


#endif /* GEOFENCE_H_ */
