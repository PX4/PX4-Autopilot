/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file TerrainDem.hpp
 * Terrain elevation model loaded from a GeoTIFF DEM, for SIH in SITL.
 *
 * The raster is read with libtiff/libgeotiff. A DEM in any CRS with an EPSG code is
 * resampled once at load time with PROJ onto a regular WGS84 lat/lon grid, so that
 * elevation() is a cheap bilinear lookup in the simulation loop.
 */

#pragma once

#include "TerrainBase.hpp"

#include <string>
#include <vector>

class TerrainDem : public TerrainBase
{
public:
	/** DEM elevation, the default altitude outside the DEM */
	bool altitude(double lat_deg, double lon_deg, float &alt) const override;

	/** march along the ray, sampling each DEM cell at least twice */
	float rayDistance(double lat_deg, double lon_deg, float alt, const matrix::Vector3f &dir_ned,
			  float max_distance) const override;

	void printStatus() const override;

	/**
	 * Load a single-band GeoTIFF DEM.
	 * @return false (and print the reason) if the file cannot be used
	 */
	bool load(const char *path);

	/**
	 * Terrain elevation [m AMSL] at a WGS84 position, interpolated bilinearly.
	 * @return false outside the DEM or next to a nodata cell
	 */
	bool elevation(double lat_deg, double lon_deg, float &alt) const;

	bool loaded() const { return !_grid.data.empty(); }

	/** smallest grid spacing [m] */
	float cellSizeM() const { return _cell_size_m; }

	const char *path() const { return _path.c_str(); }
	int epsg() const { return _epsg; }
	int width() const { return _grid.width; }
	int height() const { return _grid.height; }

	double latMin() const { return _grid.y0 + (_grid.height - 1) * _grid.dy; }
	double latMax() const { return _grid.y0; }
	double lonMin() const { return _grid.x0; }
	double lonMax() const { return _grid.x0 + (_grid.width - 1) * _grid.dx; }

	/** register the GDAL_NODATA TIFF tag with libtiff (done by load(), public for writing test files) */
	static void registerTiffTags();

	static constexpr unsigned kTiffTagGdalNodata = 42113;

private:
	// regular raster, value of cell (col, row) at x = x0 + col * dx, y = y0 + row * dy
	struct Grid {
		std::vector<float> data; // row-major, NAN for nodata
		int width{0};
		int height{0};
		double x0{0.};
		double y0{0.};
		double dx{0.};
		double dy{0.};

		bool sample(double x, double y, float &value) const;
	};

	bool resampleToWgs84(const Grid &src);

	Grid _grid{}; // x: longitude [deg], y: latitude [deg], dy < 0 (row 0 is the northern edge)
	float _cell_size_m{0.f};
	int _epsg{0};
	std::string _path;
};
