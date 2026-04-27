#pragma once

#include <matrix/math.hpp>
#include "../navigation.h"

class GeofenceInterface
{
public:
	virtual ~GeofenceInterface() = default;

	virtual bool checkIfLineViolatesAnyFence(const matrix::Vector2f &start_local,
			const matrix::Vector2f &end_local,
			const matrix::Vector2<double> &reference) = 0;

	virtual PolygonInfo getPolygonInfoByIndex(int index) = 0;

	virtual matrix::Vector2<double> getPolygonVertexByIndex(int poly_idx, int vertex_idx) = 0;

	virtual int getNumPolygons() const = 0;
};
