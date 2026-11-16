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

#define MODULE_NAME "TerrainDemTest"

#include <gtest/gtest.h>

#include "TerrainDem.hpp"

#include <geotiff.h>
#include <geovalues.h>
#include <xtiffio.h>

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

// TerrainDem logs with PX4_INFO/PX4_WARN/PX4_ERR, satisfying them from px4_platform drags uORB and
// the POSIX daemon into this host unit test
__BEGIN_DECLS

void px4_log_info(const char *moduleName, const char *fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	vfprintf(stdout, fmt, args);
	va_end(args);
	fputc('\n', stdout);
}

void px4_log_warn(const char *moduleName, const char *fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	vfprintf(stderr, fmt, args);
	va_end(args);
	fputc('\n', stderr);
}

void px4_log_error(const char *moduleName, const char *fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	vfprintf(stderr, fmt, args);
	va_end(args);
	fputc('\n', stderr);
}

__END_DECLS

// friend of TerrainDem, gives the tests access to its private metadata
class TerrainDemTest : public testing::Test
{
public:
	static void registerTiffTags() { TerrainDem::registerTiffTags(); }
	static constexpr unsigned kTiffTagGdalNodata = TerrainDem::kTiffTagGdalNodata;

	static int epsg(const TerrainDem &dem) { return dem.epsg(); }
	static int width(const TerrainDem &dem) { return dem.width(); }
	static int height(const TerrainDem &dem) { return dem.height(); }
	static float cellSizeM(const TerrainDem &dem) { return dem.cellSizeM(); }
};

namespace
{

struct TiffSpec {
	uint32_t width{3};
	uint32_t height{3};
	unsigned short model_type{ModelTypeGeographic};
	unsigned short epsg{4326};
	double corner_x{8.0};   // model coordinates of the corner of pixel (0, 0)
	double corner_y{47.0};
	double scale_x{0.01};
	double scale_y{0.01};
	bool pixel_is_point{false};
	uint16_t bits{32};
	uint16_t format{SAMPLEFORMAT_IEEEFP};
	uint16_t compression{COMPRESSION_NONE};
	uint32_t tile{0};       // 0: strips
	uint16_t bands{1};
	const char *nodata{nullptr};
};

using ValueFn = std::function<double(uint32_t col, uint32_t row)>;

void writeSample(uint8_t *buffer, size_t index, const TiffSpec &spec, double value)
{
	if (spec.format == SAMPLEFORMAT_IEEEFP) {
		const float v = static_cast<float>(value);
		memcpy(buffer + 4 * index, &v, 4);

	} else {
		const int16_t v = static_cast<int16_t>(lround(value));
		memcpy(buffer + 2 * index, &v, 2);
	}
}

std::string writeTiff(const std::string &name, const TiffSpec &spec, const ValueFn &value)
{
	TerrainDemTest::registerTiffTags();

	const std::string path = testing::TempDir() + name;
	TIFF *tif = XTIFFOpen(path.c_str(), "w");

	if (!tif) {
		ADD_FAILURE() << "cannot create " << path;
		return path;
	}

	TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, spec.width);
	TIFFSetField(tif, TIFFTAG_IMAGELENGTH, spec.height);
	TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, spec.bands);
	TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, spec.bits);
	TIFFSetField(tif, TIFFTAG_SAMPLEFORMAT, spec.format);
	TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
	TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
	TIFFSetField(tif, TIFFTAG_COMPRESSION, spec.compression);

	if (spec.tile) {
		TIFFSetField(tif, TIFFTAG_TILEWIDTH, spec.tile);
		TIFFSetField(tif, TIFFTAG_TILELENGTH, spec.tile);

	} else {
		TIFFSetField(tif, TIFFTAG_ROWSPERSTRIP, 1);
	}

	if (spec.nodata) {
		TIFFSetField(tif, TerrainDemTest::kTiffTagGdalNodata, spec.nodata);
	}

	double tiepoint[6] = {0., 0., 0., spec.corner_x, spec.corner_y, 0.};
	double scale[3] = {spec.scale_x, spec.scale_y, 0.};
	TIFFSetField(tif, TIFFTAG_GEOTIEPOINTS, 6, tiepoint);
	TIFFSetField(tif, TIFFTAG_GEOPIXELSCALE, 3, scale);

	GTIF *gtif = GTIFNew(tif);
	GTIFKeySet(gtif, GTModelTypeGeoKey, TYPE_SHORT, 1, spec.model_type);
	GTIFKeySet(gtif, GTRasterTypeGeoKey, TYPE_SHORT, 1, spec.pixel_is_point ? RasterPixelIsPoint : RasterPixelIsArea);
	GTIFKeySet(gtif, spec.model_type == ModelTypeGeographic ? GeographicTypeGeoKey : ProjectedCSTypeGeoKey,
		   TYPE_SHORT, 1, spec.epsg);
	GTIFWriteKeys(gtif);
	GTIFFree(gtif);

	const size_t sample_size = spec.bits / 8;

	if (spec.tile) {
		std::vector<uint8_t> buffer(spec.tile * spec.tile * spec.bands * sample_size);

		for (uint32_t tile_row = 0; tile_row < spec.height; tile_row += spec.tile) {
			for (uint32_t tile_col = 0; tile_col < spec.width; tile_col += spec.tile) {
				std::fill(buffer.begin(), buffer.end(), 0);

				for (uint32_t r = 0; r < spec.tile && tile_row + r < spec.height; r++) {
					for (uint32_t c = 0; c < spec.tile && tile_col + c < spec.width; c++) {
						for (uint16_t b = 0; b < spec.bands; b++) {
							writeSample(buffer.data(), (r * spec.tile + c) * spec.bands + b, spec, value(tile_col + c, tile_row + r));
						}
					}
				}

				TIFFWriteTile(tif, buffer.data(), tile_col, tile_row, 0, 0);
			}
		}

	} else {
		std::vector<uint8_t> buffer(spec.width * spec.bands * sample_size);

		for (uint32_t row = 0; row < spec.height; row++) {
			for (uint32_t col = 0; col < spec.width; col++) {
				for (uint16_t b = 0; b < spec.bands; b++) {
					writeSample(buffer.data(), col * spec.bands + b, spec, value(col, row));
				}
			}

			TIFFWriteScanline(tif, buffer.data(), row, 0);
		}
	}

	XTIFFClose(tif);
	return path;
}

// WGS84 position of the centre of pixel (col, row) of a geographic PixelIsArea raster
double pixelLon(const TiffSpec &spec, double col) { return spec.corner_x + (col + 0.5) * spec.scale_x; }
double pixelLat(const TiffSpec &spec, double row) { return spec.corner_y - (row + 0.5) * spec.scale_y; }

double ramp(uint32_t col, uint32_t row) { return 400. + 10. * col + 100. * row; }

} // namespace

TEST_F(TerrainDemTest, MissingFile)
{
	TerrainDem dem("/nonexistent/dem.tif");
	EXPECT_FALSE(dem.loaded());
}

TEST_F(TerrainDemTest, GeographicFloat32)
{
	const TiffSpec spec{};
	TerrainDem dem(writeTiff("float32.tif", spec, ramp).c_str());
	ASSERT_TRUE(dem.loaded());
	EXPECT_EQ(epsg(dem), 4326);
	EXPECT_EQ(width(dem), 3);
	EXPECT_EQ(height(dem), 3);

	float alt = 0.f;

	// cell centres
	for (uint32_t row = 0; row < spec.height; row++) {
		for (uint32_t col = 0; col < spec.width; col++) {
			ASSERT_TRUE(dem.altitude(pixelLat(spec, row), pixelLon(spec, col), alt));
			EXPECT_FLOAT_EQ(alt, ramp(col, row));
		}
	}

	// bilinear between cell centres
	ASSERT_TRUE(dem.altitude(pixelLat(spec, 0.5), pixelLon(spec, 1.25), alt));
	EXPECT_NEAR(alt, 400. + 12.5 + 50., 1e-3);

	// within the outer half cell the edge value is used
	ASSERT_TRUE(dem.altitude(spec.corner_y - 0.001, spec.corner_x + 0.001, alt));
	EXPECT_FLOAT_EQ(alt, ramp(0, 0));

	// outside
	EXPECT_FALSE(dem.altitude(spec.corner_y + 0.001, pixelLon(spec, 1), alt));
	EXPECT_FALSE(dem.altitude(pixelLat(spec, 1), spec.corner_x + 0.031, alt));
	EXPECT_FALSE(dem.altitude(NAN, NAN, alt));

	EXPECT_NEAR(cellSizeM(dem), 0.01 * 6371000. * M_PI / 180. * cos(46.985 * M_PI / 180.), 1.);
}

TEST_F(TerrainDemTest, VehicleGroundAltitude)
{
	const TiffSpec spec{};
	TerrainDem dem;
	dem.setDefaultAltitude(123.f);

	// flat until a DEM is loaded
	EXPECT_FLOAT_EQ(dem.vehicleGroundAltitude(pixelLat(spec, 1), pixelLon(spec, 1)), 123.f);

	ASSERT_TRUE(dem.load(writeTiff("vehicle.tif", spec, ramp).c_str()));
	EXPECT_FLOAT_EQ(dem.vehicleGroundAltitude(pixelLat(spec, 1), pixelLon(spec, 1)), ramp(1, 1));

	// outside the DEM the ground is flat at the default altitude, and back on the DEM when reentering
	EXPECT_FLOAT_EQ(dem.vehicleGroundAltitude(spec.corner_y + 0.001, pixelLon(spec, 1)), 123.f);
	EXPECT_FLOAT_EQ(dem.vehicleGroundAltitude(pixelLat(spec, 2), pixelLon(spec, 0)), ramp(0, 2));
}

TEST_F(TerrainDemTest, Int16LzwTiled)
{
	TiffSpec spec{};
	spec.width = 21; // partial tiles
	spec.height = 19;
	spec.bits = 16;
	spec.format = SAMPLEFORMAT_INT;
	spec.compression = COMPRESSION_LZW;
	spec.tile = 16;

	TerrainDem dem(writeTiff("int16_lzw_tiled.tif", spec, ramp).c_str());
	ASSERT_TRUE(dem.loaded());

	float alt = 0.f;
	const uint32_t points[][2] = {{0, 0}, {15, 15}, {16, 16}, {20, 18}, {17, 3}};

	for (const auto &p : points) {
		ASSERT_TRUE(dem.altitude(pixelLat(spec, p[1]), pixelLon(spec, p[0]), alt));
		EXPECT_FLOAT_EQ(alt, ramp(p[0], p[1]));
	}

	ASSERT_TRUE(dem.altitude(pixelLat(spec, 15.5), pixelLon(spec, 15.5), alt));
	EXPECT_NEAR(alt, 400. + 155. + 1550., 1e-2);
}

TEST_F(TerrainDemTest, PixelIsPoint)
{
	TiffSpec spec{};
	spec.pixel_is_point = true;

	TerrainDem dem(writeTiff("pixel_is_point.tif", spec, ramp).c_str());
	ASSERT_TRUE(dem.loaded());

	// the tiepoint is the centre of pixel (0, 0)
	float alt = 0.f;
	ASSERT_TRUE(dem.altitude(spec.corner_y, spec.corner_x, alt));
	EXPECT_FLOAT_EQ(alt, ramp(0, 0));
	ASSERT_TRUE(dem.altitude(spec.corner_y - 2. * spec.scale_y, spec.corner_x + 2. * spec.scale_x, alt));
	EXPECT_FLOAT_EQ(alt, ramp(2, 2));
}

TEST_F(TerrainDemTest, Nodata)
{
	TiffSpec spec{};
	spec.width = 4;
	spec.nodata = "-9999";

	TerrainDem dem(writeTiff("nodata.tif", spec, [](uint32_t col, uint32_t row) {
		return (col == 3 && row == 2) ? -9999. : ramp(col, row);
	}).c_str());
	ASSERT_TRUE(dem.loaded());

	float alt = 0.f;
	EXPECT_TRUE(dem.altitude(pixelLat(spec, 0.5), pixelLon(spec, 0.5), alt));
	EXPECT_TRUE(dem.altitude(pixelLat(spec, 1), pixelLon(spec, 2), alt));
	// interpolating with the nodata cell
	EXPECT_FALSE(dem.altitude(pixelLat(spec, 1.5), pixelLon(spec, 2.5), alt));
	EXPECT_FALSE(dem.altitude(pixelLat(spec, 2), pixelLon(spec, 3), alt));
}

TEST_F(TerrainDemTest, RejectsMultipleBands)
{
	TiffSpec spec{};
	spec.bands = 3;

	TerrainDem dem(writeTiff("rgb.tif", spec, ramp).c_str());
	EXPECT_FALSE(dem.loaded());
}

TEST_F(TerrainDemTest, RejectsUserDefinedCrs)
{
	TiffSpec spec{};
	spec.epsg = KvUserDefined;

	TerrainDem dem(writeTiff("user_defined.tif", spec, ramp).c_str());
	EXPECT_FALSE(dem.loaded());
}

TEST_F(TerrainDemTest, ProjectedUtm)
{
	// 1 km x 1 km at 10 m in UTM zone 32N (EPSG:32632) around Zurich, a plane rising towards north-east
	TiffSpec spec{};
	spec.width = 100;
	spec.height = 100;
	spec.model_type = ModelTypeProjected;
	spec.epsg = 32632;
	spec.corner_x = 465000.;
	spec.corner_y = 5250000.;
	spec.scale_x = 10.;
	spec.scale_y = 10.;

	const auto plane = [](double easting, double northing) {
		return 500. + 0.1 * (easting - 465000.) + 0.05 * (northing - 5249000.);
	};

	TerrainDem dem(writeTiff("utm.tif", spec, [&](uint32_t col, uint32_t row) {
		return plane(spec.corner_x + (col + 0.5) * spec.scale_x, spec.corner_y - (row + 0.5) * spec.scale_y);
	}).c_str());
	ASSERT_TRUE(dem.loaded());

	EXPECT_EQ(epsg(dem), 32632);
	EXPECT_NEAR(cellSizeM(dem), 10.f, 0.5f);

	// UTM 32N E 465500 N 5249500 (centre of the raster) is 47.3980425 N, 8.5427984 E (cs2cs EPSG:32632 EPSG:4326)
	float alt = 0.f;
	ASSERT_TRUE(dem.altitude(47.3980425, 8.5427984, alt));
	EXPECT_NEAR(alt, plane(465500., 5249500.), 0.5);

	// outside the raster
	EXPECT_FALSE(dem.altitude(47.40, 8.70, alt));
}
