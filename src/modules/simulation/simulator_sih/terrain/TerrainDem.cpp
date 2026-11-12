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

#include "TerrainDem.hpp"

#include <lib/geo/geo.h>
#include <px4_platform_common/log.h>

#include <geotiff.h>
#include <geovalues.h>
#include <xtiffio.h>
#include <proj.h>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace
{

constexpr double kMetersPerDegree = 6371000.0 * M_PI / 180.0;
constexpr size_t kMaxCells = 200'000'000; // ~800 MB of floats

constexpr const char *kConvertHint =
	"convert with: gdalwarp -t_srs EPSG:4326 -ot Float32 -co COMPRESS=LZW <in> <out.tif>";

TIFFExtendProc parent_tag_extender = nullptr;
const TIFFFieldInfo *custom_field_info = nullptr; // set by TerrainDem::registerTiffTags()
int custom_field_count = 0;

void tagExtender(TIFF *tif)
{
	TIFFMergeFieldInfo(tif, custom_field_info, custom_field_count);

	if (parent_tag_extender) {
		parent_tag_extender(tif);
	}
}

// RAII wrapper so that every early return closes the file
struct GeoTiffFile {
	TIFF *tif{nullptr};
	GTIF *gtif{nullptr};

	~GeoTiffFile()
	{
		if (gtif) {
			GTIFFree(gtif);
		}

		if (tif) {
			XTIFFClose(tif);
		}
	}
};

// read one sample of a raster row/tile buffer as float
bool sampleReader(uint16_t bits, uint16_t format, float ( *&read)(const uint8_t *, size_t))
{
	read = nullptr;

	if (format == SAMPLEFORMAT_UINT) {
		if (bits == 8) {
			read = [](const uint8_t *p, size_t i) { return static_cast<float>(p[i]); };

		} else if (bits == 16) {
			read = [](const uint8_t *p, size_t i) { uint16_t v; memcpy(&v, p + 2 * i, 2); return static_cast<float>(v); };

		} else if (bits == 32) {
			read = [](const uint8_t *p, size_t i) { uint32_t v; memcpy(&v, p + 4 * i, 4); return static_cast<float>(v); };
		}

	} else if (format == SAMPLEFORMAT_INT) {
		if (bits == 8) {
			read = [](const uint8_t *p, size_t i) { return static_cast<float>(static_cast<int8_t>(p[i])); };

		} else if (bits == 16) {
			read = [](const uint8_t *p, size_t i) { int16_t v; memcpy(&v, p + 2 * i, 2); return static_cast<float>(v); };

		} else if (bits == 32) {
			read = [](const uint8_t *p, size_t i) { int32_t v; memcpy(&v, p + 4 * i, 4); return static_cast<float>(v); };
		}

	} else if (format == SAMPLEFORMAT_IEEEFP) {
		if (bits == 32) {
			read = [](const uint8_t *p, size_t i) { float v; memcpy(&v, p + 4 * i, 4); return v; };

		} else if (bits == 64) {
			read = [](const uint8_t *p, size_t i) { double v; memcpy(&v, p + 8 * i, 8); return static_cast<float>(v); };
		}
	}

	return read != nullptr;
}

} // namespace

void TerrainDem::registerTiffTags()
{
	static const TIFFFieldInfo field_info[] = {
		{kTiffTagGdalNodata, -1, -1, TIFF_ASCII, FIELD_CUSTOM, 1, 0, const_cast<char *>("GDALNoDataValue")},
	};

	static bool registered = false;

	if (!registered) {
		registered = true;
		custom_field_info = field_info;
		custom_field_count = sizeof(field_info) / sizeof(field_info[0]);
		parent_tag_extender = TIFFSetTagExtender(tagExtender);
	}
}

bool TerrainDem::Grid::sample(double x, double y, float &value) const
{
	const double fx = (x - x0) / dx;
	const double fy = (y - y0) / dy;

	// accept up to half a cell outside the outermost cell centres (also rejects NAN)
	if (!(fx >= -0.5 && fy >= -0.5 && fx <= width - 0.5 && fy <= height - 0.5)) {
		return false;
	}

	const double cx = fmin(fmax(fx, 0.), width - 1.);
	const double cy = fmin(fmax(fy, 0.), height - 1.);
	const int col = std::min(static_cast<int>(cx), width - 2);
	const int row = std::min(static_cast<int>(cy), height - 2);
	const float tx = static_cast<float>(cx - col);
	const float ty = static_cast<float>(cy - row);

	const size_t i = static_cast<size_t>(row) * width + col;
	const float v00 = data[i];
	const float v01 = data[i + 1];
	const float v10 = data[i + width];
	const float v11 = data[i + width + 1];

	if (std::isnan(v00) || std::isnan(v01) || std::isnan(v10) || std::isnan(v11)) {
		return false;
	}

	value = (1.f - ty) * ((1.f - tx) * v00 + tx * v01) + ty * ((1.f - tx) * v10 + tx * v11);
	return true;
}

bool TerrainDem::load(const char *path)
{
	registerTiffTags();

	GeoTiffFile file;
	file.tif = XTIFFOpen(path, "r");

	if (!file.tif) {
		PX4_ERR("terrain: cannot open %s", path);
		return false;
	}

	file.gtif = GTIFNew(file.tif);

	if (!file.gtif) {
		PX4_ERR("terrain: %s has no GeoTIFF keys, %s", path, kConvertHint);
		return false;
	}

	TIFF *tif = file.tif;
	GTIF *gtif = file.gtif;

	// raster layout
	uint32_t width = 0;
	uint32_t height = 0;
	uint16_t samples_per_pixel = 1;
	uint16_t bits_per_sample = 0;
	uint16_t sample_format = SAMPLEFORMAT_UINT;
	TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &width);
	TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &height);
	TIFFGetFieldDefaulted(tif, TIFFTAG_SAMPLESPERPIXEL, &samples_per_pixel);
	TIFFGetFieldDefaulted(tif, TIFFTAG_BITSPERSAMPLE, &bits_per_sample);
	TIFFGetFieldDefaulted(tif, TIFFTAG_SAMPLEFORMAT, &sample_format);

	if (samples_per_pixel != 1) {
		PX4_ERR("terrain: %s has %u bands, a single elevation band is required", path, samples_per_pixel);
		return false;
	}

	if (width < 2 || height < 2 || static_cast<size_t>(width) * height > kMaxCells) {
		PX4_ERR("terrain: %s is %ux%u cells, it must be between 2x2 and %zu cells (crop or downsample it)",
			path, width, height, kMaxCells);
		return false;
	}

	float (*read_sample)(const uint8_t *, size_t) = nullptr;

	if (!sampleReader(bits_per_sample, sample_format, read_sample)) {
		PX4_ERR("terrain: %s has unsupported samples (%u bit, format %u), %s", path, bits_per_sample, sample_format,
			kConvertHint);
		return false;
	}

	// georeferencing: pixel corner (0, 0) in model coordinates and pixel size
	Grid src{};
	uint16_t count = 0;
	double *values = nullptr;

	if (TIFFGetField(tif, TIFFTAG_GEOTRANSMATRIX, &count, &values) && count >= 16) {
		if (fabs(values[1]) > 0. || fabs(values[4]) > 0.) {
			PX4_ERR("terrain: %s is rotated, %s", path, kConvertHint);
			return false;
		}

		src.dx = values[0];
		src.dy = values[5];
		src.x0 = values[3];
		src.y0 = values[7];

	} else {
		double *tiepoint = nullptr;
		uint16_t tiepoint_count = 0;

		if (!TIFFGetField(tif, TIFFTAG_GEOPIXELSCALE, &count, &values) || count < 2
		    || !TIFFGetField(tif, TIFFTAG_GEOTIEPOINTS, &tiepoint_count, &tiepoint) || tiepoint_count < 6) {
			PX4_ERR("terrain: %s has no pixel scale and tiepoint, %s", path, kConvertHint);
			return false;
		}

		src.dx = values[0];
		src.dy = -values[1];
		src.x0 = tiepoint[3] - tiepoint[0] * src.dx;
		src.y0 = tiepoint[4] - tiepoint[1] * src.dy;
	}

	unsigned short raster_type = RasterPixelIsArea;
	GTIFKeyGet(gtif, GTRasterTypeGeoKey, &raster_type, 0, 1);

	if (raster_type != RasterPixelIsPoint) {
		// the model coordinates refer to the pixel corner, move them to the pixel centre
		src.x0 += 0.5 * src.dx;
		src.y0 += 0.5 * src.dy;
	}

	if (!(src.dx > 0.) || !(src.dy < 0.)) {
		PX4_ERR("terrain: %s must be north-up with increasing columns towards east, %s", path, kConvertHint);
		return false;
	}

	// coordinate reference system
	unsigned short model_type = 0;
	unsigned short epsg = 0;
	GTIFKeyGet(gtif, GTModelTypeGeoKey, &model_type, 0, 1);

	if (model_type == ModelTypeGeographic) {
		GTIFKeyGet(gtif, GeographicTypeGeoKey, &epsg, 0, 1);

		unsigned short angular_units = Angular_Degree;
		GTIFKeyGet(gtif, GeogAngularUnitsGeoKey, &angular_units, 0, 1);

		if (angular_units != Angular_Degree) {
			PX4_ERR("terrain: %s geographic units are not degrees, %s", path, kConvertHint);
			return false;
		}

	} else if (model_type == ModelTypeProjected) {
		GTIFKeyGet(gtif, ProjectedCSTypeGeoKey, &epsg, 0, 1);
	}

	if (epsg == 0 || epsg == KvUserDefined) {
		PX4_ERR("terrain: %s has no EPSG coded CRS, %s", path, kConvertHint);
		return false;
	}

	// nodata
	bool has_nodata = false;
	float nodata = 0.f;
	char *nodata_str = nullptr;

	if (TIFFGetField(tif, kTiffTagGdalNodata, &nodata_str) && nodata_str) {
		char *end = nullptr;
		nodata = strtof(nodata_str, &end);
		has_nodata = (end != nodata_str) && !std::isnan(nodata);
	}

	// samples
	src.width = width;
	src.height = height;
	src.data.resize(static_cast<size_t>(width) * height);

	const auto store = [&](uint32_t col, uint32_t row, float value) {
		if (has_nodata && fabsf(value - nodata) <= FLT_EPSILON * fmaxf(1.f, fabsf(nodata))) {
			value = NAN;
		}

		src.data[static_cast<size_t>(row) * width + col] = value;
	};

	if (TIFFIsTiled(tif)) {
		uint32_t tile_width = 0;
		uint32_t tile_height = 0;
		TIFFGetField(tif, TIFFTAG_TILEWIDTH, &tile_width);
		TIFFGetField(tif, TIFFTAG_TILELENGTH, &tile_height);
		std::vector<uint8_t> buffer(TIFFTileSize(tif));

		for (uint32_t tile_row = 0; tile_row < height; tile_row += tile_height) {
			for (uint32_t tile_col = 0; tile_col < width; tile_col += tile_width) {
				if (TIFFReadTile(tif, buffer.data(), tile_col, tile_row, 0, 0) < 0) {
					PX4_ERR("terrain: %s: failed to read tile at %u,%u", path, tile_col, tile_row);
					return false;
				}

				for (uint32_t r = 0; r < tile_height && tile_row + r < height; r++) {
					for (uint32_t c = 0; c < tile_width && tile_col + c < width; c++) {
						store(tile_col + c, tile_row + r, read_sample(buffer.data(), static_cast<size_t>(r) * tile_width + c));
					}
				}
			}
		}

	} else {
		std::vector<uint8_t> buffer(TIFFScanlineSize(tif));

		for (uint32_t row = 0; row < height; row++) {
			if (TIFFReadScanline(tif, buffer.data(), row, 0) < 0) {
				PX4_ERR("terrain: %s: failed to read row %u", path, row);
				return false;
			}

			for (uint32_t col = 0; col < width; col++) {
				store(col, row, read_sample(buffer.data(), col));
			}
		}
	}

	_epsg = epsg;
	_path = path;

	if (epsg == 4326) {
		_grid = std::move(src);
		const double lat_center = _grid.y0 + 0.5 * (_grid.height - 1) * _grid.dy;
		_cell_size_m = static_cast<float>(kMetersPerDegree * fmin(-_grid.dy, _grid.dx * cos(lat_center * M_PI / 180.)));

	} else if (!resampleToWgs84(src)) {
		_grid = Grid{};
		return false;
	}

	printStatus();

	return true;
}

bool TerrainDem::resampleToWgs84(const Grid &src)
{
	char crs[32];
	snprintf(crs, sizeof(crs), "EPSG:%d", _epsg);

	PJ_CONTEXT *ctx = proj_context_create();
	PJ *wgs84_to_src = nullptr;

	{
		PJ *transform = proj_create_crs_to_crs(ctx, "EPSG:4326", crs, nullptr);

		if (transform) {
			// lon/lat and easting/northing axis order, whatever the CRS definitions say
			wgs84_to_src = proj_normalize_for_visualization(ctx, transform);
			proj_destroy(transform);
		}
	}

	if (!wgs84_to_src) {
		PX4_ERR("terrain: PROJ cannot transform WGS84 to %s (%s), %s", crs,
			proj_context_errno_string(ctx, proj_context_errno(ctx)), kConvertHint);
		proj_context_destroy(ctx);
		return false;
	}

	// WGS84 bounding box of the source raster, from points along its edges
	constexpr int kEdgePoints = 32;
	std::vector<double> lon;
	std::vector<double> lat;

	for (int i = 0; i <= kEdgePoints; i++) {
		const double t = static_cast<double>(i) / kEdgePoints;
		const double col = t * (src.width - 1);
		const double row = t * (src.height - 1);
		const double edge_cols[4] = {col, col, 0., src.width - 1.};
		const double edge_rows[4] = {0., src.height - 1., row, row};

		for (int edge = 0; edge < 4; edge++) {
			lon.push_back(src.x0 + edge_cols[edge] * src.dx);
			lat.push_back(src.y0 + edge_rows[edge] * src.dy);
		}
	}

	// the centre pixel and its neighbours, to keep the source resolution
	const double center_col = 0.5 * (src.width - 1);
	const double center_row = 0.5 * (src.height - 1);
	const size_t center = lon.size();
	lon.insert(lon.end(), {src.x0 + center_col * src.dx, src.x0 + (center_col + 1.) * src.dx, src.x0 + center_col * src.dx});
	lat.insert(lat.end(), {src.y0 + center_row * src.dy, src.y0 + center_row * src.dy, src.y0 + (center_row + 1.) * src.dy});

	proj_trans_generic(wgs84_to_src, PJ_INV, lon.data(), sizeof(double), lon.size(), lat.data(), sizeof(double), lat.size(),
			   nullptr, 0, 0, nullptr, 0, 0);

	double lon_min = static_cast<double>(INFINITY);
	double lon_max = -static_cast<double>(INFINITY);
	double lat_min = static_cast<double>(INFINITY);
	double lat_max = -static_cast<double>(INFINITY);

	for (size_t i = 0; i < lon.size(); i++) {
		if (!std::isfinite(lon[i]) || !std::isfinite(lat[i])) {
			PX4_ERR("terrain: %s extent cannot be transformed to WGS84", _path.c_str());
			proj_destroy(wgs84_to_src);
			proj_context_destroy(ctx);
			return false;
		}

		lon_min = fmin(lon_min, lon[i]);
		lon_max = fmax(lon_max, lon[i]);
		lat_min = fmin(lat_min, lat[i]);
		lat_max = fmax(lat_max, lat[i]);
	}

	const double cos_lat = cos(lat[center] * M_PI / 180.);
	const double step_col_m = kMetersPerDegree * hypot(lat[center + 1] - lat[center], (lon[center + 1] - lon[center]) * cos_lat);
	const double step_row_m = kMetersPerDegree * hypot(lat[center + 2] - lat[center], (lon[center + 2] - lon[center]) * cos_lat);
	const double cell_m = fmin(step_col_m, step_row_m);

	Grid dst{};
	dst.dx = cell_m / (kMetersPerDegree * cos_lat);
	dst.dy = -cell_m / kMetersPerDegree;
	dst.x0 = lon_min;
	dst.y0 = lat_max;
	dst.width = static_cast<int>((lon_max - lon_min) / dst.dx) + 1;
	dst.height = static_cast<int>((lat_max - lat_min) / -dst.dy) + 1;

	if (dst.width < 2 || dst.height < 2 || static_cast<size_t>(dst.width) * dst.height > kMaxCells) {
		PX4_ERR("terrain: %s resampled to WGS84 is %dx%d cells (crop or downsample it)", _path.c_str(), dst.width, dst.height);
		proj_destroy(wgs84_to_src);
		proj_context_destroy(ctx);
		return false;
	}

	dst.data.resize(static_cast<size_t>(dst.width) * dst.height);

	// transform one row of WGS84 cell centres at a time into the source CRS and sample the source there
	std::vector<double> x(dst.width);
	std::vector<double> y(dst.width);

	for (int row = 0; row < dst.height; row++) {
		for (int col = 0; col < dst.width; col++) {
			x[col] = dst.x0 + col * dst.dx;
			y[col] = dst.y0 + row * dst.dy;
		}

		proj_trans_generic(wgs84_to_src, PJ_FWD, x.data(), sizeof(double), x.size(), y.data(), sizeof(double), y.size(),
				   nullptr, 0, 0, nullptr, 0, 0);

		for (int col = 0; col < dst.width; col++) {
			float value;

			if (!src.sample(x[col], y[col], value)) {
				value = NAN;
			}

			dst.data[static_cast<size_t>(row) * dst.width + col] = value;
		}
	}

	proj_destroy(wgs84_to_src);
	proj_context_destroy(ctx);

	_grid = std::move(dst);
	_cell_size_m = static_cast<float>(cell_m);
	return true;
}

bool TerrainDem::altitude(double lat_deg, double lon_deg, float &alt) const
{
	if (!loaded()) {
		return TerrainFlat::altitude(lat_deg, lon_deg, alt);
	}

	if (_grid.sample(lon_deg, lat_deg, alt)) {
		return true;
	}

	alt = _default_alt;
	return false;
}

float TerrainDem::vehicleGroundAltitude(double lat_deg, double lon_deg)
{
	float alt;
	const bool inside = altitude(lat_deg, lon_deg, alt);

	if (!inside && !_vehicle_outside) {
		PX4_WARN("terrain: vehicle outside the DEM, using flat ground at %.1f m", (double)_default_alt);

	} else if (inside && _vehicle_outside) {
		PX4_INFO("terrain: vehicle back inside the DEM");
	}

	_vehicle_outside = !inside;
	return alt;
}

float TerrainDem::rayDistance(double lat_deg, double lon_deg, float alt, const matrix::Vector3f &dir_ned,
			      float max_distance) const
{
	if (!loaded()) {
		return TerrainFlat::rayDistance(lat_deg, lon_deg, alt, dir_ned, max_distance);
	}

	if (dir_ned(2) <= 0.f) {
		return NAN;
	}

	float ground_alt;
	altitude(lat_deg, lon_deg, ground_alt);
	float prev_height = alt - ground_alt; // height above the terrain along the ray

	if (prev_height <= 0.f) {
		return 0.f;
	}

	MapProjection ref{lat_deg, lon_deg};

	// sample each DEM cell at least twice, the bilinear terrain is then followed within centimetres
	static constexpr float STEPS_PER_CELL = 2.f;
	static constexpr float MAX_STEP = 1.f; // [m]
	const float step = std::min(_cell_size_m / STEPS_PER_CELL, MAX_STEP);
	float prev_distance = 0.f;

	for (float distance = step; distance < max_distance + step; distance += step) {
		const matrix::Vector3f point = dir_ned * distance;
		double lat;
		double lon;
		ref.reproject(point(0), point(1), lat, lon);
		altitude(lat, lon, ground_alt);
		const float height = (alt - point(2)) - ground_alt;

		if (height <= 0.f) {
			// interpolate the crossing between the last two samples
			return prev_distance + (distance - prev_distance) * prev_height / (prev_height - height);
		}

		prev_distance = distance;
		prev_height = height;
	}

	return NAN;
}

void TerrainDem::printStatus() const
{
	if (!loaded()) {
		TerrainFlat::printStatus();
		return;
	}

	PX4_INFO("terrain: %s", path());
	PX4_INFO("terrain: EPSG:%d, %dx%d cells of %.1f m", epsg(), width(), height(), (double)_cell_size_m);
	PX4_INFO("terrain: lat [%.5f, %.5f] lon [%.5f, %.5f]", latMin(), latMax(), lonMin(), lonMax());

	if (_vehicle_outside) {
		PX4_INFO("terrain: vehicle outside the DEM");
	}
}
