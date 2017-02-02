#include <geo_lookup/geo_mag_declination.c>

#include "gtest/gtest.h"

TEST(GeoMagTest, IndexBoundsCheck)
{
	unsigned index = get_lookup_table_index(90.0f, SAMPLING_MIN_LAT, SAMPLING_MAX_LAT);
	ASSERT_EQ((unsigned)11, index);
	index = get_lookup_table_index(-90.0f, SAMPLING_MIN_LAT, SAMPLING_MAX_LAT);
	ASSERT_EQ((unsigned)0, index);
	index = get_lookup_table_index(180.0f, SAMPLING_MIN_LON, SAMPLING_MAX_LON);
	ASSERT_EQ((unsigned)35, index);
	index = get_lookup_table_index(-180.0f, SAMPLING_MIN_LON, SAMPLING_MAX_LON);
	ASSERT_EQ((unsigned)0, index);
}

