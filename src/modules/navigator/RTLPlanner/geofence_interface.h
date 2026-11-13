#pragma once

#include <cstdint>
#include <matrix/math.hpp>
#include "../navigation.h"

struct PolygonInfo {
	uint16_t fence_type; ///< one of MAV_CMD_NAV_FENCE_* (can also be a circular region)
	uint8_t dataman_index;
	FenceLevel fence_level{FenceLevel::Hard}; ///< geofence or pre-geofence
	union {
		uint16_t vertex_count;
		float circle_radius;
	};
};

static_assert(sizeof(PolygonInfo) == 8, "Keep geofence area descriptors compact");

class GeofenceInterface
{
public:
	virtual ~GeofenceInterface() = default;

	virtual PolygonInfo getPolygonInfoByIndex(int index) = 0;

	virtual matrix::Vector2<double> getPolygonVertexByIndex(int poly_idx, int vertex_idx) = 0;

	virtual int getNumPolygons() const = 0;
};
