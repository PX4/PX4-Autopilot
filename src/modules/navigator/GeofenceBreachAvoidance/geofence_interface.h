#pragma once

#include <matrix/math.hpp>
#include "../navigation.h"

class GeofenceInterface
{
public:
	virtual ~GeofenceInterface() = default;

	virtual PolygonInfo getPolygonInfoByIndex(int index) = 0;

	virtual matrix::Vector2<double> getPolygonVertexByIndex(int poly_idx, int vertex_idx) = 0;

	virtual int getNumPolygons() const = 0;
};
