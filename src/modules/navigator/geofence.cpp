/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: @author Jean Cyr <jean.m.cyr@gmail.com>
 *           @author Thomas Gubler <thomasgubler@gmail.com>
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
 * @file geofence.cpp
 * Provides functions for handling the geofence
 */
#include "geofence.h"

#include <uORB/topics/vehicle_global_position.h>
#include <string.h>
#include <dataman/dataman.h>
#include <systemlib/err.h>
#include <stdlib.h>

Geofence::Geofence() : _fence_pub(-1)
{
	memset(&_fence, 0, sizeof(_fence));
}

Geofence::~Geofence()
{

}


bool Geofence::inside(const struct vehicle_global_position_s *vehicle)
{

	/* Adaptation of algorithm originally presented as
	 * PNPOLY - Point Inclusion in Polygon Test
	 * W. Randolph Franklin (WRF) */

	unsigned int i, j, vertices = _fence.count;
	bool c = false;
	double lat = vehicle->lat / 1e7d;
	double lon = vehicle->lon / 1e7d;

	// skip vertex 0 (return point)
	for (i = 0, j = vertices - 1; i < vertices; j = i++)
		if (((_fence.vertices[i].lon) >= lon != (_fence.vertices[j].lon >= lon)) &&
		    (lat <= (_fence.vertices[j].lat - _fence.vertices[i].lat) * (lon - _fence.vertices[i].lon) /
		     (_fence.vertices[j].lon - _fence.vertices[i].lon) + _fence.vertices[i].lat))
			c = !c;
	return c;
}

bool
Geofence::load(unsigned vertices)
{
	struct fence_s temp_fence;

	unsigned i;
	for (i = 0; i < vertices; i++) {
		if (dm_read(DM_KEY_FENCE_POINTS, i, temp_fence.vertices + i, sizeof(struct fence_vertex_s)) != sizeof(struct fence_vertex_s)) {
			break;
		}
	}

	temp_fence.count = i;

	if (valid())
		memcpy(&_fence, &temp_fence, sizeof(_fence));
	else
		warnx("Invalid fence file, ignored!");

	return _fence.count != 0;
}

bool
Geofence::valid()
{
	// NULL fence is valid
	if (_fence.count == 0) {
		return true;
	}

	// Otherwise
	if ((_fence.count < 4) || (_fence.count > GEOFENCE_MAX_VERTICES)) {
		warnx("Fence must have at least 3 sides and not more than %d", GEOFENCE_MAX_VERTICES - 1);
		return false;
	}

	return true;
}

void
Geofence::addPoint(int argc, char *argv[])
{
	int ix, last;
	double lon, lat;
	struct fence_vertex_s vertex;
	char *end;

	if ((argc == 1) && (strcmp("-clear", argv[0]) == 0)) {
		dm_clear(DM_KEY_FENCE_POINTS);
		publishFence(0);
		return;
	}

	if (argc < 3)
		errx(1, "Specify: -clear | sequence latitude longitude [-publish]");

	ix = atoi(argv[0]);
	if (ix >= DM_KEY_FENCE_POINTS_MAX)
		errx(1, "Sequence must be less than %d", DM_KEY_FENCE_POINTS_MAX);

	lat = strtod(argv[1], &end);
	lon = strtod(argv[2], &end);

	last = 0;
	if ((argc > 3) && (strcmp(argv[3], "-publish") == 0))
		last = 1;

	vertex.lat = (float)lat;
	vertex.lon = (float)lon;

	if (dm_write(DM_KEY_FENCE_POINTS, ix, DM_PERSIST_POWER_ON_RESET, &vertex, sizeof(vertex)) == sizeof(vertex)) {
		if (last)
			publishFence((unsigned)ix + 1);
		return;
	}

	errx(1, "can't store fence point");
}

void
Geofence::publishFence(unsigned vertices)
{
	if (_fence_pub == -1)
		_fence_pub = orb_advertise(ORB_ID(fence), &vertices);
	else
		orb_publish(ORB_ID(fence), _fence_pub, &vertices);
}

