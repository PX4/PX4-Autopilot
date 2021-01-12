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
 * @file fake_geofence.h
 * Provides a fake geofence interface to use for testing
 *
 * @author Roman Bapst
 * @author Julian Kent
 */


#include"../geofence.h"
#include <lib/mathlib/mathlib.h>

class FakeGeofence : public Geofence
{
public:
	FakeGeofence() :
		Geofence(nullptr)
	{};

	virtual ~FakeGeofence() {};

	virtual bool isInsidePolygonOrCircle(double lat, double lon, float altitude)
	{
		switch (_probe_function_behavior) {
		case ALL_POINTS_OUTSIDE: {
				return _allPointsOutside(lat, lon, altitude);
			}

		case LEFT_INSIDE_RIGHT_OUTSIDE: {
				return _left_inside_right_outside(lat, lon, altitude);
			}

		case RIGHT_INSIDE_LEFT_OUTSIDE: {
				return _right_inside_left_outside(lat, lon, altitude);
			}

		case GF_BOUNDARY_20M_AHEAD: {
				return _gf_boundary_is_20m_ahead(lat, lon, altitude);
			}

		default:
			return _allPointsOutside(lat, lon, altitude);
		}
	}

	enum PROBE_FUNC_ENUM {
		ALL_POINTS_OUTSIDE = 0,
		LEFT_INSIDE_RIGHT_OUTSIDE,
		RIGHT_INSIDE_LEFT_OUTSIDE,
		GF_BOUNDARY_20M_AHEAD
	};

	void setProbeFunctionBehavior(PROBE_FUNC_ENUM func) {_probe_function_behavior = func;}


private:

	PROBE_FUNC_ENUM _probe_function_behavior = ALL_POINTS_OUTSIDE;

	bool _allPointsOutside(double lat, double lon, float alt)
	{
		return false;
	}

	bool _left_inside_right_outside(double lat, double lon, float alt)
	{
		static int flag = true;

		if (flag) {
			flag = false;
			return true;

		} else {
			return false;
		}
	}

	bool _right_inside_left_outside(double lat, double lon, float alt)
	{
		static int flag = false;

		if (flag) {
			flag = false;
			return true;

		} else {
			flag = true;
			return false;
		}
	}

	bool _gf_boundary_is_20m_ahead(double lat, double lon, float alt)
	{
		struct map_projection_reference_s ref = {};
		matrix::Vector2<double> home_global(42.1, 8.2);
		map_projection_init(&ref, home_global(0), home_global(1));

		float x, y;
		map_projection_project(&ref, lat, lon, &x, &y);
		matrix::Vector2f waypoint_local(x, y);

		if (waypoint_local(0) >= 20.0f) {
			return false;
		}

		return true;
	}
};

typedef enum {
	DM_PERSIST_POWER_ON_RESET = 0,	/* Data survives all resets */
	DM_PERSIST_IN_FLIGHT_RESET,     /* Data survives in-flight resets only */
	DM_PERSIST_VOLATILE             /* Data does not survive resets */
} dm_persitence_t;

typedef enum {
	DM_KEY_SAFE_POINTS = 0,		/* Safe points coordinates, safe point 0 is home point */
	DM_KEY_FENCE_POINTS,		/* Fence vertex coordinates */
	DM_KEY_WAYPOINTS_OFFBOARD_0,	/* Mission way point coordinates sent over mavlink */
	DM_KEY_WAYPOINTS_OFFBOARD_1,	/* (alternate between 0 and 1) */
	DM_KEY_WAYPOINTS_ONBOARD,	/* Mission way point coordinates generated onboard */
	DM_KEY_MISSION_STATE,		/* Persistent mission state */
	DM_KEY_COMPAT,
	DM_KEY_NUM_KEYS			/* Total number of item types defined */
} dm_item_t;

typedef enum {
	DM_INIT_REASON_POWER_ON = 0,	/* Data survives resets */
	DM_INIT_REASON_IN_FLIGHT,		/* Data survives in-flight resets only */
	DM_INIT_REASON_VOLATILE			/* Data does not survive reset */
} dm_reset_reason;

extern "C" {
	__EXPORT ssize_t
	dm_read(
		dm_item_t item,			/* The item type to retrieve */
		unsigned index,			/* The index of the item */
		void *buffer,			/* Pointer to caller data buffer */
		size_t buflen			/* Length in bytes of data to retrieve */
	) {return 0;};

	/** write to the data manager store */
	__EXPORT ssize_t
	dm_write(
		dm_item_t  item,		/* The item type to store */
		unsigned index,			/* The index of the item */
		dm_persitence_t persistence,	/* The persistence level of this item */
		const void *buffer,		/* Pointer to caller data buffer */
		size_t buflen			/* Length in bytes of data to retrieve */
	) {return 0;};

	/**
	 * Lock all items of a type. Can be used for atomic updates of multiple items (single items are always updated
	 * atomically).
	 * Note that this lock is independent from dm_read & dm_write calls.
	 * @return 0 on success and lock taken, -1 on error (lock not taken, errno set)
	 */
	__EXPORT int
	dm_lock(
		dm_item_t item			/* The item type to lock */
	) {return 0;};

	/**
	 * Try to lock all items of a type (@see sem_trywait()).
	 * @return 0 if lock is taken, -1 otherwise (on error or if already locked. errno is set accordingly)
	 */
	__EXPORT int
	dm_trylock(
		dm_item_t item			/* The item type to lock */
	) {return 0;};

	/** Unlock all items of a type */
	__EXPORT void
	dm_unlock(
		dm_item_t item			/* The item type to unlock */
	) {};

	/** Erase all items of this type */
	__EXPORT int
	dm_clear(
		dm_item_t item			/* The item type to clear */
	) {return 0;};

	/** Tell the data manager about the type of the last reset */
	__EXPORT int
	dm_restart(
		dm_reset_reason restart_type	/* The last reset type */
	) {return 0;};
}
